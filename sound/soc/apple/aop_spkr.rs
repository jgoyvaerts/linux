// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! J616s (T6040) internal speakers on the AOP `spkr` stream.
//!
//! The AOP firmware runs MCA cluster 0 for this stream (`spS `, serializer `ms00`, `mck0` from the
//! 12 MHz crystal): a 250-bit frame at 48 kHz with six 32-bit playback slots (24 valid bits) and
//! twelve 16-bit V/I sense slots coming back on the same serializer. The base-ns ADMAC moves both:
//! channel 0 (TX0) the playback, channel 1 (RX0) the sense, through AOP DART
//! stream 8. The AP side only attaches the
//! service, requests its power states and runs the DMA; the six SN012776 amps are their own codec
//! driver's, linked to this component by macaudio's J616 variant.
//!
//! This is an ASoC component next to the microphone card of the parent module: DPCM front-end PCMs
//! on the ADMAC channels (`j616-pcm-0..2`, the names macaudio's front-end links bind to) and one
//! back-end DAI, `spkr`, for the stream. The DAI table order is ABI: `sound-dai = <&aop_audio 3>`
//! is `spkr`.
//!
//! Power, from aurora's J616s control test (`spkr` attached, `pw1 `, `pwrd` with output flag 2,
//! `pw1 `, `idle`, each confirmed through property 200): `pw1 ` powers the audio fabric, the ADMAC
//! included, and `pwrd` runs the serializer clocks. The ADMAC registers only answer with the fabric
//! powered, so the back-end startup raises `pw1 ` before a front-end requests its channel (DPCM
//! starts the back-end first), and every front-end holds its own reference until it has released
//! the channel (DPCM shuts the back-end down before it closes the front-end). `pwrd` is requested
//! in the back-end prepare, before the amps unmute, and dropped in hw_free, after they muted: the
//! clocks run whenever an amp may be active.
//!
//! The structure (component context with the descriptor as its first field, DAI tables, dmaengine
//! front-ends) follows aurora's t8140_aop_audio.rs (Copyright (C) 2026 The Aurora Silicon
//! Contributors).

use core::{mem, ptr, slice};

use kernel::{
    bindings, c_str, device,
    error::from_err_ptr,
    new_mutex,
    prelude::*,
    soc::apple::aop::from_fourcc,
    sync::{aref::ARef, Arc, Mutex},
    types::Opaque,
};

use super::{
    AudioAttachDevice, AudioSetDeviceProp, PowerSetting, SndSocAopData, EPIC_SUBTYPE_WRAPPED_CALL,
};

const CALLTYPE_AUDIO_GET_PROP: u32 = 0xc3000004;
const AUDIO_DEV_SPKR: u32 = from_fourcc(b"spkr");
const PROP_POWER_STATE: u32 = 200;
const PROP_POWER_REQUEST: u32 = 202;
const STATE_IDLE: u32 = from_fourcc(b"idle");
const STATE_PW1: u32 = from_fourcc(b"pw1 ");
const STATE_PWRD: u32 = from_fourcc(b"pwrd");
/// The power request parameter for `pwrd` on an output stream (aurora: "output flag 2").
const PWRD_OUTPUT: u32 = 2;
/// kIOReturnBusy: the firmware keeps an attachment for its lifetime, so a reload sees this.
const AUDIO_RET_BUSY: u32 = 0xe00002d5;

/// Property read (0xc3000004): the same header as the property write, with a trailing 1.
#[repr(C, packed(4))]
#[derive(Clone, Copy, Default)]
struct AudioGetDeviceProp {
    _zero0: u32,
    unk0: u32,
    calltype: u32,
    _zero1: [u8; 16],
    _pad0: u32,
    len: u64,
    dev_id: u32,
    modifier: u32,
    unk6: u32,
}

impl AudioGetDeviceProp {
    fn new(dev_id: u32, modifier: u32) -> Self {
        Self {
            unk0: 0xffffffff,
            calltype: CALLTYPE_AUDIO_GET_PROP,
            len: 0x30,
            dev_id,
            modifier,
            unk6: 1,
            ..Default::default()
        }
    }
}

/// The `spkr` service: attachment, power-state users and whether the serializer runs.
struct ServiceState {
    attached: bool,
    /// back-end directions started plus front-ends holding a DMA channel
    users: u32,
    running: bool,
    sequence: u32,
}

#[pin_data]
pub(crate) struct Speakers {
    aop: Arc<SndSocAopData>,
    #[pin]
    state: Mutex<ServiceState>,
}

impl Speakers {
    fn call_ret<T>(&self, msg: &T, ret_len: usize) -> Result<KVec<u8>> {
        // SAFETY: T is a plain packed firmware record.
        let bytes =
            unsafe { slice::from_raw_parts(ptr::from_ref(msg).cast::<u8>(), mem::size_of::<T>()) };
        let (retcode, ret) = self.aop.adata.epic_call_ret(
            &self.aop.service,
            EPIC_SUBTYPE_WRAPPED_CALL,
            bytes,
            ret_len,
        )?;
        if retcode != 0 {
            dev_err!(
                self.aop.dev,
                "M4_SPKR property read failed, return code {:#x}\n",
                retcode
            );
            return Err(EIO);
        }
        Ok(ret)
    }

    /// Property 200: the service's current power state (reply: u32 length, then the FourCC).
    fn power_state(&self) -> Result<u32> {
        let ret = self.call_ret(
            &AudioGetDeviceProp::new(AUDIO_DEV_SPKR, PROP_POWER_STATE),
            8,
        )?;
        if ret.len() < 8 || u32::from_le_bytes([ret[0], ret[1], ret[2], ret[3]]) != 4 {
            dev_err!(
                self.aop.dev,
                "M4_SPKR malformed power-state reply ({} bytes)\n",
                ret.len()
            );
            return Err(EIO);
        }
        Ok(u32::from_le_bytes([ret[4], ret[5], ret[6], ret[7]]))
    }

    /// Property 202, then the property 200 readback: a state the firmware did not take fails the
    /// stream.
    fn set_power(&self, st: &mut ServiceState, state: u32, param: u32) -> Result<()> {
        st.sequence = st.sequence.wrapping_add(1);
        let msg = AudioSetDeviceProp::new(
            AUDIO_DEV_SPKR,
            PROP_POWER_REQUEST,
            PowerSetting::new(AUDIO_DEV_SPKR, st.sequence, state, param),
        );
        let ret = self.aop.epic_wrapped_call(&msg)?;
        if ret != 0 {
            dev_err!(
                self.aop.dev,
                "M4_SPKR power request {:#x} rejected: {:#x}\n",
                state,
                ret
            );
            return Err(EIO);
        }
        if self.power_state()? != state {
            return Err(EIO);
        }
        Ok(())
    }

    /// A user of the service: the first one attaches it (once per firmware lifetime) and raises
    /// `pw1 `.
    fn get(&self) -> Result<()> {
        let mut st = self.state.lock();
        if st.users == 0 {
            // A power-down that failed or timed out can leave the firmware in another state than
            // ours: note it; pw1 is valid from any.
            if st.attached {
                match self.power_state() {
                    Ok(now) if now != STATE_IDLE => {
                        dev_warn!(self.aop.dev, "M4_SPKR found state {:#x} at startup\n", now)
                    }
                    _ => {}
                }
            }
            if !st.attached {
                let ret = self
                    .aop
                    .epic_wrapped_call(&AudioAttachDevice::new(AUDIO_DEV_SPKR))?;
                if ret != 0 && ret != AUDIO_RET_BUSY {
                    dev_err!(
                        self.aop.dev,
                        "M4_SPKR attach failed, return code {:#x}\n",
                        ret
                    );
                    return Err(EIO);
                }
                st.attached = true;
            }
            self.set_power(&mut st, STATE_PW1, 0)?;
        }
        st.users += 1;
        Ok(())
    }

    /// The last user takes the service back through `pw1 ` (if it still runs) to `idle`.
    fn put(&self) {
        let mut st = self.state.lock();
        if st.users == 0 {
            dev_warn!(self.aop.dev, "M4_SPKR unbalanced put\n");
            return;
        }
        st.users -= 1;
        if st.users != 0 {
            return;
        }
        if st.running {
            if let Err(e) = self.set_power(&mut st, STATE_PW1, 0) {
                dev_err!(self.aop.dev, "M4_SPKR pw1 at shutdown failed: {:?}\n", e);
            }
            st.running = false;
        }
        if let Err(e) = self.set_power(&mut st, STATE_IDLE, 0) {
            dev_err!(self.aop.dev, "M4_SPKR idle failed: {:?}\n", e);
        }
    }

    /// `pwrd`: the firmware starts the serializer clocks (before the amps unmute).
    fn run(&self) -> Result<()> {
        let mut st = self.state.lock();
        if st.users == 0 {
            return Err(EINVAL);
        }
        if !st.running {
            self.set_power(&mut st, STATE_PWRD, PWRD_OUTPUT)?;
            st.running = true;
        }
        Ok(())
    }

    /// Back to `pw1 ` (after the amps muted).
    fn stop(&self) {
        let mut st = self.state.lock();
        if st.running {
            if let Err(e) = self.set_power(&mut st, STATE_PW1, 0) {
                dev_err!(self.aop.dev, "M4_SPKR pw1 failed: {:?}\n", e);
            }
            st.running = false;
        }
    }
}

// ---- the ASoC component ----

const FE_COUNT: usize = 3;
/// DAI ids: 0 lets ASoC assign one, so the front-ends carry 1..=3 and the back-end 10.
const FE_DAI_ID_BASE: u32 = 1;
const BE_DAI_ID_SPKR: u32 = 10;
/// Channel names on the audio node, per front-end and stream [playback, capture]. j616-pcm-0
/// (macaudio's "Primary", the jack on other machines) has none: this machine's jack is not on this
/// stream.
const FE_DMA_NAMES: [[Option<&CStr>; 2]; FE_COUNT] = [
    [None, None],
    [Some(c_str!("spkr")), None],
    [None, Some(c_str!("sense"))],
];
const SPKR_CHANNELS: u32 = 6;
const SENSE_CHANNELS: u32 = 12;
/// The serializer takes 24-bit samples from the low bits of each 32-bit DMA word (as the AOP's
/// other 24-in-32 streams do); the sense words are 16-bit. SNDRV_PCM_FMTBIT_S24_LE and _S16_LE
/// (formats 6 and 2; the bindings only carry the float one).
const PLAYBACK_FORMATS: u64 = 1 << 6;
const SENSE_FORMATS: u64 = 1 << 2;
const PCM_BUFFER_MAX: usize = 1024 * 1024;
const PCM_PERIOD_BYTES_MIN: usize = 256;
const PCM_PERIOD_BYTES_MAX: usize = 0x10000;
const DAI_COUNT: usize = FE_COUNT + 1;

/// Everything ASoC keeps pointers into; C retains the address of `component`, so this lives from
/// before registration until unregistration has finished every callback.
#[repr(C)]
struct AsocContext {
    component: bindings::snd_soc_component_driver,
    speakers: Arc<Speakers>,
    dais: KBox<Opaque<[bindings::snd_soc_dai_driver; DAI_COUNT]>>,
    _ops: KBox<DaiOps>,
}

const _: () = assert!(mem::offset_of!(AsocContext, component) == 0);

/// # Safety
///
/// `component` must belong to this module's registered component.
unsafe fn context<'a>(component: *mut bindings::snd_soc_component) -> &'a AsocContext {
    // SAFETY: by the caller's contract; the descriptor is the first field of its repr(C) context
    // and C keeps the exact pointer passed to registration.
    unsafe { &*(*component).driver.cast::<AsocContext>() }
}

/// # Safety
///
/// `dai` must be a DAI of this module's registered component.
unsafe fn dai_speakers<'a>(dai: *mut bindings::snd_soc_dai) -> &'a Speakers {
    // SAFETY: by the caller's contract.
    unsafe { &context((*dai).component).speakers }
}

/// # Safety
///
/// `substream` must be live and belong to one of this component's links.
unsafe fn rtd_of(
    substream: *mut bindings::snd_pcm_substream,
) -> *mut bindings::snd_soc_pcm_runtime {
    // SAFETY: by the caller's contract.
    unsafe { (*substream).private_data.cast() }
}

/// # Safety
///
/// `rtd` must be a live ASoC runtime.
unsafe fn is_be(rtd: *mut bindings::snd_soc_pcm_runtime) -> bool {
    // SAFETY: by the caller's contract.
    unsafe { (*(*rtd).dai_link).no_pcm() != 0 }
}

/// The front-end index of a runtime (from its CPU DAI's id), `None` for a back-end.
///
/// # Safety
///
/// `rtd` must be a live ASoC runtime of one of this component's links.
unsafe fn fe_index(rtd: *mut bindings::snd_soc_pcm_runtime) -> Option<usize> {
    // SAFETY: by the caller's contract; the first DAI of a runtime is its CPU DAI.
    let id = unsafe { (**(*rtd).dais).id } as u32;
    if (FE_DAI_ID_BASE..FE_DAI_ID_BASE + FE_COUNT as u32).contains(&id) {
        Some((id - FE_DAI_ID_BASE) as usize)
    } else {
        None
    }
}

/// # Safety
///
/// `substream` must be live.
unsafe fn is_playback(substream: *mut bindings::snd_pcm_substream) -> bool {
    // SAFETY: by the caller's contract.
    unsafe { (*substream).stream == bindings::SNDRV_PCM_STREAM_PLAYBACK as i32 }
}

fn dmaengine_hw() -> bindings::snd_pcm_hardware {
    bindings::snd_pcm_hardware {
        info: bindings::SNDRV_PCM_INFO_MMAP
            | bindings::SNDRV_PCM_INFO_MMAP_VALID
            | bindings::SNDRV_PCM_INFO_INTERLEAVED,
        buffer_bytes_max: PCM_BUFFER_MAX,
        period_bytes_min: PCM_PERIOD_BYTES_MIN,
        period_bytes_max: PCM_PERIOD_BYTES_MAX,
        periods_min: 2,
        periods_max: (PCM_BUFFER_MAX / PCM_PERIOD_BYTES_MIN) as u32,
        fifo_size: 16,
        ..Default::default()
    }
}

/// Front-end open: a power-state reference (the back-end already raised `pw1 `, which powers the
/// ADMAC), then the channel and the dmaengine runtime on it.
///
/// # Safety
///
/// Called by ASoC on this module's registered component with live arguments.
unsafe extern "C" fn pcm_open(
    component: *mut bindings::snd_soc_component,
    substream: *mut bindings::snd_pcm_substream,
) -> i32 {
    // SAFETY: ASoC calls the component op with live arguments of one of its links.
    let (spk, rtd) = unsafe { (&context(component).speakers, rtd_of(substream)) };
    // SAFETY: as above.
    if unsafe { is_be(rtd) } {
        return 0;
    }
    // SAFETY: as above.
    let Some(fe) = (unsafe { fe_index(rtd) }) else {
        return EINVAL.to_errno();
    };
    // SAFETY: as above.
    let playback = unsafe { is_playback(substream) };
    let Some(name) = FE_DMA_NAMES[fe][usize::from(!playback)] else {
        return ENODEV.to_errno();
    };
    if let Err(e) = spk.get() {
        return e.to_errno();
    }
    // SAFETY: the device is live and the name NUL-terminated.
    let chan = match from_err_ptr(unsafe {
        bindings::dma_request_chan(spk.aop.dev.as_raw(), name.as_char_ptr())
    }) {
        Ok(chan) => chan,
        Err(e) => {
            dev_err!(
                spk.aop.dev,
                "M4_SPKR front-end {}: no DMA channel {:?}: {:?}\n",
                fe,
                name,
                e
            );
            spk.put();
            return e.to_errno();
        }
    };
    let formats = if playback {
        PLAYBACK_FORMATS
    } else {
        SENSE_FORMATS
    };
    let ret = pcm_open_dmaengine(substream, chan, formats);
    if ret < 0 {
        // SAFETY: the channel was requested above and is not attached to the substream.
        unsafe { bindings::dma_release_channel(chan) };
        spk.put();
    }
    ret
}

fn pcm_open_dmaengine(
    substream: *mut bindings::snd_pcm_substream,
    chan: *mut bindings::dma_chan,
    formats: u64,
) -> i32 {
    // SAFETY: a live substream and a channel the caller requested.
    let unallocated =
        unsafe { (*substream).dma_buffer.dev.type_ == bindings::SNDRV_DMA_TYPE_UNKNOWN as i32 };
    if unallocated {
        // SAFETY: as above; the channel's device is the ADMAC behind AOP DART stream 8 and outlives
        // the buffer.
        let ret = unsafe {
            bindings::snd_pcm_set_managed_buffer(
                substream,
                bindings::SNDRV_DMA_TYPE_DEV as i32,
                (*(*chan).device).dev,
                0,
                0,
            )
        };
        if ret < 0 {
            return ret;
        }
    }
    let mut hw = dmaengine_hw();
    let mut dma_data = bindings::snd_dmaengine_dai_dma_data::default();
    // SAFETY: the substream and the channel are live for the call.
    let ret = unsafe {
        bindings::snd_dmaengine_pcm_refine_runtime_hwparams(substream, &mut dma_data, &mut hw, chan)
    };
    if ret < 0 {
        return ret;
    }
    hw.formats &= formats;
    if hw.formats == 0 {
        return EINVAL.to_errno();
    }
    // SAFETY: the runtime exists for the op; on success the dmaengine runtime owns `chan`.
    unsafe {
        (*(*substream).runtime).hw = hw;
        bindings::snd_dmaengine_pcm_open(substream, chan)
    }
}

/// # Safety
///
/// Called by ASoC on this module's registered component with live arguments.
unsafe extern "C" fn pcm_close(
    component: *mut bindings::snd_soc_component,
    substream: *mut bindings::snd_pcm_substream,
) -> i32 {
    // SAFETY: ASoC calls the component op with live arguments of one of its links.
    let (spk, rtd) = unsafe { (&context(component).speakers, rtd_of(substream)) };
    // SAFETY: as above.
    if unsafe { is_be(rtd) } {
        return 0;
    }
    // SAFETY: as above.
    let Some(fe) = (unsafe { fe_index(rtd) }) else {
        return 0;
    };
    // SAFETY: as above.
    if FE_DMA_NAMES[fe][usize::from(!unsafe { is_playback(substream) })].is_none() {
        return 0;
    }
    // The channel goes first, with the fabric still powered by this front-end's reference. SAFETY:
    // the dmaengine runtime was set up by `pcm_open`.
    let ret = unsafe { bindings::snd_dmaengine_pcm_close_release_chan(substream) };
    spk.put();
    ret
}

/// The ADMAC moves one frame per burst of up to four words: FRAME_4_WORDS for both streams, as the
/// ADT's records for this stream select (0x22 32-bit playback, 0x21 16-bit sense).
///
/// # Safety
///
/// Called by ASoC on this module's registered component with live arguments.
unsafe extern "C" fn pcm_hw_params(
    _component: *mut bindings::snd_soc_component,
    substream: *mut bindings::snd_pcm_substream,
    params: *mut bindings::snd_pcm_hw_params,
) -> i32 {
    // SAFETY: as for `pcm_open`.
    if unsafe { is_be(rtd_of(substream)) } {
        return 0;
    }
    // SAFETY: the dmaengine runtime was set up by `pcm_open`.
    let chan = unsafe { bindings::snd_dmaengine_pcm_get_chan(substream) };
    let mut cfg = bindings::dma_slave_config::default();
    // SAFETY: live substream and parameters.
    let rc = unsafe { bindings::snd_hwparams_to_dma_slave_config(substream, params, &mut cfg) };
    if rc < 0 {
        return rc;
    }
    // SAFETY: as above.
    if unsafe { is_playback(substream) } {
        cfg.dst_port_window_size = 4;
    } else {
        cfg.src_port_window_size = 4;
    }
    // SAFETY: `chan` is the live channel behind this substream.
    unsafe {
        match (*(*chan).device).device_config {
            Some(f) => f(chan, &mut cfg),
            None => ENOSYS.to_errno(),
        }
    }
}

/// # Safety
///
/// Called by ASoC on this module's registered component with live arguments.
unsafe extern "C" fn pcm_trigger(
    _component: *mut bindings::snd_soc_component,
    substream: *mut bindings::snd_pcm_substream,
    cmd: i32,
) -> i32 {
    // SAFETY: as for `pcm_open`.
    if unsafe { is_be(rtd_of(substream)) } {
        return 0;
    }
    // SAFETY: the dmaengine runtime was set up by `pcm_open`.
    unsafe { bindings::snd_dmaengine_pcm_trigger(substream, cmd) }
}

/// # Safety
///
/// Called by ASoC on this module's registered component with live arguments.
unsafe extern "C" fn pcm_sync_stop(
    _component: *mut bindings::snd_soc_component,
    substream: *mut bindings::snd_pcm_substream,
) -> i32 {
    // SAFETY: as for `pcm_open`.
    if unsafe { is_be(rtd_of(substream)) } {
        return 0;
    }
    // SAFETY: the dmaengine runtime was set up by `pcm_open`.
    unsafe { bindings::snd_dmaengine_pcm_sync_stop(substream) }
}

/// # Safety
///
/// Called by ASoC on this module's registered component with live arguments.
unsafe extern "C" fn pcm_pointer(
    _component: *mut bindings::snd_soc_component,
    substream: *mut bindings::snd_pcm_substream,
) -> bindings::snd_pcm_uframes_t {
    // SAFETY: as for `pcm_open`.
    if unsafe { is_be(rtd_of(substream)) } {
        return (-i64::from(bindings::ENOTSUPP)) as bindings::snd_pcm_uframes_t;
    }
    // SAFETY: the dmaengine runtime was set up by `pcm_open`.
    unsafe { bindings::snd_dmaengine_pcm_pointer(substream) }
}

// The firmware runs the serializer: format, slots and clocks are its own, whatever the machine
// driver asks.
unsafe extern "C" fn dai_set_fmt(_dai: *mut bindings::snd_soc_dai, _fmt: u32) -> i32 {
    0
}
unsafe extern "C" fn dai_set_tdm_slot(
    _dai: *mut bindings::snd_soc_dai,
    _tx: u32,
    _rx: u32,
    _slots: i32,
    _width: i32,
) -> i32 {
    0
}
unsafe extern "C" fn dai_set_sysclk(
    _dai: *mut bindings::snd_soc_dai,
    _id: i32,
    _freq: u32,
    _dir: i32,
) -> i32 {
    0
}
unsafe extern "C" fn dai_set_bclk_ratio(_dai: *mut bindings::snd_soc_dai, _ratio: u32) -> i32 {
    0
}

/// Back-end startup, either direction: `pw1 ` (the first user attaches the service).
///
/// # Safety
///
/// Called by ASoC with a live DAI of this component.
unsafe extern "C" fn spkr_startup(
    _substream: *mut bindings::snd_pcm_substream,
    dai: *mut bindings::snd_soc_dai,
) -> i32 {
    // SAFETY: by the caller's contract.
    match unsafe { dai_speakers(dai) }.get() {
        Ok(()) => 0,
        Err(e) => e.to_errno(),
    }
}

/// Playback prepare: `pwrd`. ASoC prepares this CPU DAI before it unmutes the amps.
///
/// # Safety
///
/// Called by ASoC with a live DAI of this component and a live substream.
unsafe extern "C" fn spkr_prepare(
    substream: *mut bindings::snd_pcm_substream,
    dai: *mut bindings::snd_soc_dai,
) -> i32 {
    // SAFETY: by the caller's contract.
    if !unsafe { is_playback(substream) } {
        return 0;
    }
    // SAFETY: by the caller's contract.
    match unsafe { dai_speakers(dai) }.run() {
        Ok(()) => 0,
        Err(e) => e.to_errno(),
    }
}

/// Playback hw_free: back to `pw1 `. ASoC mutes the amps before it frees the CPU DAI.
///
/// # Safety
///
/// Called by ASoC with a live DAI of this component and a live substream.
unsafe extern "C" fn spkr_hw_free(
    substream: *mut bindings::snd_pcm_substream,
    dai: *mut bindings::snd_soc_dai,
) -> i32 {
    // SAFETY: by the caller's contract.
    if unsafe { is_playback(substream) } {
        // SAFETY: by the caller's contract.
        unsafe { dai_speakers(dai) }.stop();
    }
    0
}

/// # Safety
///
/// Called by ASoC with a live DAI of this component.
unsafe extern "C" fn spkr_shutdown(
    _substream: *mut bindings::snd_pcm_substream,
    dai: *mut bindings::snd_soc_dai,
) {
    // SAFETY: by the caller's contract.
    unsafe { dai_speakers(dai) }.put();
}

struct DaiOps {
    fe: bindings::snd_soc_dai_ops,
    spkr: bindings::snd_soc_dai_ops,
}

impl DaiOps {
    fn new() -> Self {
        let fe = bindings::snd_soc_dai_ops {
            set_fmt: Some(dai_set_fmt),
            set_tdm_slot: Some(dai_set_tdm_slot),
            set_sysclk: Some(dai_set_sysclk),
            set_bclk_ratio: Some(dai_set_bclk_ratio),
            ..Default::default()
        };
        DaiOps {
            spkr: bindings::snd_soc_dai_ops {
                startup: Some(spkr_startup),
                prepare: Some(spkr_prepare),
                hw_free: Some(spkr_hw_free),
                shutdown: Some(spkr_shutdown),
                ..fe
            },
            fe,
        }
    }
}

fn stream(name: &'static CStr, formats: u64, channels: u32) -> bindings::snd_soc_pcm_stream {
    bindings::snd_soc_pcm_stream {
        stream_name: name.as_char_ptr(),
        formats,
        rates: bindings::SNDRV_PCM_RATE_48000,
        rate_min: 48000,
        rate_max: 48000,
        channels_min: channels,
        channels_max: channels,
        ..Default::default()
    }
}

/// [0..=2] the front-ends, [3] the back-end. The order is ABI (`sound-dai = <&aop_audio 3>`).
fn dai_drivers(ops: &DaiOps) -> [bindings::snd_soc_dai_driver; DAI_COUNT] {
    let mut dais = [
        bindings::snd_soc_dai_driver {
            name: c_str!("j616-pcm-0").as_char_ptr(),
            id: FE_DAI_ID_BASE,
            ops: &ops.fe,
            // No stream of this machine's: both directions exist only for macaudio's fixed routes
            // ("PCM0 TX", "PCM0 RX"), whose widgets must exist for the card to bind; opening either
            // fails (ENODEV).
            playback: stream(c_str!("PCM0 TX"), PLAYBACK_FORMATS, SPKR_CHANNELS),
            capture: stream(c_str!("PCM0 RX"), SENSE_FORMATS, SENSE_CHANNELS),
            ..Default::default()
        },
        bindings::snd_soc_dai_driver {
            name: c_str!("j616-pcm-1").as_char_ptr(),
            id: FE_DAI_ID_BASE + 1,
            ops: &ops.fe,
            playback: stream(c_str!("PCM1 TX"), PLAYBACK_FORMATS, SPKR_CHANNELS),
            ..Default::default()
        },
        bindings::snd_soc_dai_driver {
            name: c_str!("j616-pcm-2").as_char_ptr(),
            id: FE_DAI_ID_BASE + 2,
            ops: &ops.fe,
            capture: stream(c_str!("PCM2 RX"), SENSE_FORMATS, SENSE_CHANNELS),
            ..Default::default()
        },
        bindings::snd_soc_dai_driver {
            name: c_str!("spkr").as_char_ptr(),
            id: BE_DAI_ID_SPKR,
            ops: &ops.spkr,
            playback: stream(c_str!("SPKR TX"), PLAYBACK_FORMATS, SPKR_CHANNELS),
            capture: stream(c_str!("SPKR RX"), SENSE_FORMATS, SENSE_CHANNELS),
            ..Default::default()
        },
    ];
    // The sense runs at the playback's rate (macaudio reads the back-end's symmetric rate).
    dais[DAI_COUNT - 1].set_symmetric_rate(1);
    dais
}

/// The registered component; dropping it unregisters it (and with it any card that uses its DAIs).
pub(crate) struct SpeakerComponent {
    dev: ARef<device::Device>,
    ctx: KBox<AsocContext>,
}

impl SpeakerComponent {
    pub(crate) fn register(aop: Arc<SndSocAopData>) -> Result<Self> {
        let held = aop.clone();
        let speakers = Arc::pin_init(
            pin_init!(Speakers {
                aop: held,
                state <- new_mutex!(ServiceState {
                    attached: false,
                    users: 0,
                    running: false,
                    sequence: 0,
                }),
            }),
            GFP_KERNEL,
        )?;
        let ops = KBox::new(DaiOps::new(), GFP_KERNEL)?;
        let dais = KBox::new(Opaque::new(dai_drivers(&ops)), GFP_KERNEL)?;
        let component = bindings::snd_soc_component_driver {
            name: c_str!("t6040-aop-spkr").as_char_ptr(),
            open: Some(pcm_open),
            close: Some(pcm_close),
            hw_params: Some(pcm_hw_params),
            trigger: Some(pcm_trigger),
            sync_stop: Some(pcm_sync_stop),
            pointer: Some(pcm_pointer),
            ..Default::default()
        };
        let ctx = KBox::new(
            AsocContext {
                component,
                speakers,
                dais,
                _ops: ops,
            },
            GFP_KERNEL,
        )?;
        // SAFETY: the descriptor and the DAI table are heap allocations owned by `ctx`, which
        // `Self` keeps until after unregistration.
        let ret = unsafe {
            bindings::snd_soc_register_component(
                aop.dev.as_raw(),
                &ctx.component,
                ctx.dais.get().cast(),
                DAI_COUNT as i32,
            )
        };
        if ret < 0 {
            dev_err!(
                aop.dev,
                "M4_SPKR unable to register the ASoC component: {}\n",
                ret
            );
            return Err(Error::from_errno(ret));
        }
        Ok(Self {
            dev: aop.dev.clone(),
            ctx,
        })
    }
}

impl Drop for SpeakerComponent {
    fn drop(&mut self) {
        // SAFETY: registered in `register`; the context stays allocated until unregistration has
        // finished every PCM and DAI callback.
        unsafe {
            bindings::snd_soc_unregister_component_by_driver(self.dev.as_raw(), &self.ctx.component)
        };
    }
}

// SAFETY: the raw pointers in the context are C descriptors that are only read after registration.
unsafe impl Send for SpeakerComponent {}
// SAFETY: as above.
unsafe impl Sync for SpeakerComponent {}
