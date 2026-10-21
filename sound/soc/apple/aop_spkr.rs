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
//! The 3.5 mm jack is two more services of the same firmware: `cout` (MCA cluster 2, serializer
//! `ms02`, `mck2` from `nco3` at 3.072 MHz: a 64-bit frame at 48 kHz, two 32-bit playback slots,
//! 24 valid bits) on base-ns ADMAC channel 4 (TX2), and `cin ` (the headset microphone, one slot
//! back on the same serializer) on channel 5 (RX2). The CS42L84 codec on i2c2 is its own driver's.
//! The firmware arbitrates the MCA and the audio power states between the speaker and jack streams,
//! so the two never program shared registers behind each other's back. Each service has its own
//! attachment and power states: `pw1 ` from back-end startup, `pwrd` (output flag 2 for `cout`,
//! input flag 1 for `cin `, as the microphones use) from prepare.
//!
//! The jack is a second ASoC component on the same device, for a sound card of its own (macaudio's
//! J616 jack variant): front-ends `j616-jack-pcm-0` (the jack) and `j616-jack-pcm-1` (routes only)
//! and the `jack` back-end, `sound-dai = <&aop_audio 4>`. A component binds to one card only, and
//! a card binds only once every codec of it has probed: on its own card, a jack codec that never
//! answers costs the jack, not the speakers. The speaker component is the same with or without the
//! jack. The jack component exists only when the audio node lists the "cout" and "cin" DMA
//! channels.
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
/// The jack's output and headset microphone (ADT iop-audio-controller identifiers 'tuoc', ' nic').
const AUDIO_DEV_COUT: u32 = from_fourcc(b"cout");
const AUDIO_DEV_CIN: u32 = from_fourcc(b"cin ");
const PROP_POWER_STATE: u32 = 200;
const PROP_POWER_REQUEST: u32 = 202;
const STATE_IDLE: u32 = from_fourcc(b"idle");
const STATE_PW1: u32 = from_fourcc(b"pw1 ");
const STATE_PWRD: u32 = from_fourcc(b"pwrd");
/// The power request parameter for `pwrd` on an output stream (aurora: "output flag 2").
const PWRD_OUTPUT: u32 = 2;
/// The same for an input stream (the microphones' `pwrd` request uses 1).
const PWRD_INPUT: u32 = 1;
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

/// A firmware audio service: attachment, power-state users and whether the serializer runs.
struct ServiceState {
    attached: bool,
    /// back-end directions started plus front-ends holding a DMA channel
    users: u32,
    running: bool,
    sequence: u32,
}

#[pin_data]
pub(crate) struct Service {
    aop: Arc<SndSocAopData>,
    dev_id: u32,
    /// The log prefix (the speaker checks parse the `M4_SPKR` lines).
    tag: &'static str,
    /// The `pwrd` request parameter: output or input.
    pwrd_param: u32,
    #[pin]
    state: Mutex<ServiceState>,
}

impl Service {
    fn new(
        aop: Arc<SndSocAopData>,
        dev_id: u32,
        tag: &'static str,
        pwrd_param: u32,
    ) -> Result<Arc<Self>> {
        Arc::pin_init(
            pin_init!(Service {
                aop,
                dev_id,
                tag,
                pwrd_param,
                state <- new_mutex!(ServiceState {
                    attached: false,
                    users: 0,
                    running: false,
                    sequence: 0,
                }),
            }),
            GFP_KERNEL,
        )
    }

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
                "{} property read failed, return code {:#x}\n",
                self.tag,
                retcode
            );
            return Err(EIO);
        }
        Ok(ret)
    }

    /// Property 200: the service's current power state (reply: u32 length, then the FourCC).
    fn power_state(&self) -> Result<u32> {
        let ret = self.call_ret(&AudioGetDeviceProp::new(self.dev_id, PROP_POWER_STATE), 8)?;
        if ret.len() < 8 || u32::from_le_bytes([ret[0], ret[1], ret[2], ret[3]]) != 4 {
            dev_err!(
                self.aop.dev,
                "{} malformed power-state reply ({} bytes)\n",
                self.tag,
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
            self.dev_id,
            PROP_POWER_REQUEST,
            PowerSetting::new(self.dev_id, st.sequence, state, param),
        );
        let ret = self.aop.epic_wrapped_call(&msg)?;
        if ret != 0 {
            dev_err!(
                self.aop.dev,
                "{} power request {:#x} rejected: {:#x}\n",
                self.tag,
                state,
                ret
            );
            return Err(EIO);
        }
        let now = self.power_state()?;
        dev_info!(
            self.aop.dev,
            "{} power {:#x} param {} -> state {:#x}\n",
            self.tag,
            state,
            param,
            now
        );
        if now != state {
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
                        dev_warn!(
                            self.aop.dev,
                            "{} found state {:#x} at startup\n",
                            self.tag,
                            now
                        )
                    }
                    _ => {}
                }
            }
            if !st.attached {
                let ret = self
                    .aop
                    .epic_wrapped_call(&AudioAttachDevice::new(self.dev_id))?;
                if ret != 0 && ret != AUDIO_RET_BUSY {
                    dev_err!(
                        self.aop.dev,
                        "{} attach failed, return code {:#x}\n",
                        self.tag,
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
            dev_warn!(self.aop.dev, "{} unbalanced put\n", self.tag);
            return;
        }
        st.users -= 1;
        if st.users != 0 {
            return;
        }
        if st.running {
            if let Err(e) = self.set_power(&mut st, STATE_PW1, 0) {
                dev_err!(
                    self.aop.dev,
                    "{} pw1 at shutdown failed: {:?}\n",
                    self.tag,
                    e
                );
            }
            st.running = false;
        }
        if let Err(e) = self.set_power(&mut st, STATE_IDLE, 0) {
            dev_err!(self.aop.dev, "{} idle failed: {:?}\n", self.tag, e);
        }
    }

    /// `pwrd`: the firmware starts the serializer clocks (before the amps or the codec unmute).
    fn run(&self) -> Result<()> {
        let mut st = self.state.lock();
        if st.users == 0 {
            return Err(EINVAL);
        }
        if !st.running {
            self.set_power(&mut st, STATE_PWRD, self.pwrd_param)?;
            st.running = true;
        }
        Ok(())
    }

    /// Back to `pw1 ` (after the amps or the codec muted).
    fn stop(&self) {
        let mut st = self.state.lock();
        if st.running {
            if let Err(e) = self.set_power(&mut st, STATE_PW1, 0) {
                dev_err!(self.aop.dev, "{} pw1 failed: {:?}\n", self.tag, e);
            }
            st.running = false;
        }
    }
}

// ---- the ASoC component ----

const FE_COUNT: usize = 5;
/// DAI ids: 0 lets ASoC assign one, so the front-ends carry 1..=5 (the speaker component's three,
/// then the jack component's two) and the back-ends 10 and 11.
const FE_DAI_ID_BASE: u32 = 1;
const BE_DAI_ID_SPKR: u32 = 10;
const BE_DAI_ID_JACK: u32 = 11;
/// The jack back-end's `sound-dai` argument on the audio node: 0..=3 are the speaker component's
/// DAIs by table index.
const DT_DAI_JACK: u32 = 4;
/// Which service's power a front-end stream needs.
#[derive(Clone, Copy)]
enum Svc {
    Spkr,
    Cout,
    Cin,
}
/// A front-end stream: its channel name on the audio node, its service and its sample formats.
struct FeStream {
    dma: &'static CStr,
    svc: Svc,
    formats: u64,
}
/// Per front-end and stream [playback, capture]. The speaker card: j616-pcm-0 is macaudio's
/// "Primary" (the jack on other machines; none here), j616-pcm-1 its "Secondary", the speakers,
/// j616-pcm-2 the speaker sense. The jack card: j616-jack-pcm-0 its "Primary", the jack,
/// j616-jack-pcm-1 its "Secondary" (none).
static FE_STREAMS: [[Option<FeStream>; 2]; FE_COUNT] = [
    [None, None],
    [
        Some(FeStream {
            dma: c_str!("spkr"),
            svc: Svc::Spkr,
            formats: PLAYBACK_FORMATS,
        }),
        None,
    ],
    [
        None,
        Some(FeStream {
            dma: c_str!("sense"),
            svc: Svc::Spkr,
            formats: SENSE_FORMATS,
        }),
    ],
    [
        Some(FeStream {
            dma: c_str!("cout"),
            svc: Svc::Cout,
            formats: PLAYBACK_FORMATS,
        }),
        Some(FeStream {
            dma: c_str!("cin"),
            svc: Svc::Cin,
            formats: PLAYBACK_FORMATS,
        }),
    ],
    [None, None],
];
const SPKR_CHANNELS: u32 = 6;
const SENSE_CHANNELS: u32 = 12;
const JACK_CHANNELS: u32 = 2;
const HEADSET_CHANNELS: u32 = 1;
/// The serializer takes 24-bit samples from the low bits of each 32-bit DMA word (as the AOP's
/// other 24-in-32 streams do; the jack's records say 24 valid bits as well); the sense words are
/// 16-bit. SNDRV_PCM_FMTBIT_S24_LE and _S16_LE (formats 6 and 2; the bindings only carry the float
/// one).
const PLAYBACK_FORMATS: u64 = 1 << 6;
const SENSE_FORMATS: u64 = 1 << 2;
const PCM_BUFFER_MAX: usize = 1024 * 1024;
const PCM_PERIOD_BYTES_MIN: usize = 256;
const PCM_PERIOD_BYTES_MAX: usize = 0x10000;
/// One ADMAC frame (FRAME_4_WORDS): the jack's 8- and 4-byte frames need whole bursts per period.
const ADMAC_BURST_BYTES: u32 = 16;

/// Everything ASoC keeps pointers into, one per component; C retains the address of `component`,
/// so this lives from before registration until unregistration has finished every callback.
#[repr(C)]
struct AsocContext {
    component: bindings::snd_soc_component_driver,
    spkr: Arc<Service>,
    cout: Arc<Service>,
    cin: Arc<Service>,
    dais: KVec<Opaque<bindings::snd_soc_dai_driver>>,
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

impl AsocContext {
    fn service(&self, svc: Svc) -> &Service {
        match svc {
            Svc::Spkr => &self.spkr,
            Svc::Cout => &self.cout,
            Svc::Cin => &self.cin,
        }
    }
}

/// The service behind a back-end DAI and direction, and whether its prepare runs the serializer:
/// the speaker sense rides on the playback's `pwrd`, the jack's directions are separate services.
///
/// # Safety
///
/// `dai` must be a DAI of this module's registered component and `substream` live.
unsafe fn be_service<'a>(
    dai: *mut bindings::snd_soc_dai,
    substream: *mut bindings::snd_pcm_substream,
) -> (&'a Service, bool) {
    // SAFETY: by the caller's contract.
    let (ctx, id, playback) = unsafe {
        (
            context((*dai).component),
            (*dai).id as u32,
            is_playback(substream),
        )
    };
    match (id, playback) {
        (BE_DAI_ID_JACK, true) => (&ctx.cout, true),
        (BE_DAI_ID_JACK, false) => (&ctx.cin, true),
        (_, playback) => (&ctx.spkr, playback),
    }
}

/// The front-end stream of a runtime and substream, `None` for a back-end or a stream this machine
/// does not have.
///
/// # Safety
///
/// `rtd` must be a live runtime of one of this component's links and `substream` live.
unsafe fn fe_stream(
    rtd: *mut bindings::snd_soc_pcm_runtime,
    substream: *mut bindings::snd_pcm_substream,
) -> Option<(usize, &'static FeStream)> {
    // SAFETY: by the caller's contract.
    let (fe, playback) = unsafe { (fe_index(rtd)?, is_playback(substream)) };
    Some((fe, FE_STREAMS[fe][usize::from(!playback)].as_ref()?))
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
    let (ctx, rtd) = unsafe { (context(component), rtd_of(substream)) };
    // SAFETY: as above.
    if unsafe { is_be(rtd) } {
        return 0;
    }
    // SAFETY: as above.
    if unsafe { fe_index(rtd) }.is_none() {
        return EINVAL.to_errno();
    }
    // SAFETY: as above.
    let Some((fe, stream)) = (unsafe { fe_stream(rtd, substream) }) else {
        return ENODEV.to_errno();
    };
    let svc = ctx.service(stream.svc);
    if let Err(e) = svc.get() {
        return e.to_errno();
    }
    // SAFETY: the device is live and the name NUL-terminated.
    let chan = match from_err_ptr(unsafe {
        bindings::dma_request_chan(svc.aop.dev.as_raw(), stream.dma.as_char_ptr())
    }) {
        Ok(chan) => chan,
        Err(e) => {
            dev_err!(
                svc.aop.dev,
                "{} front-end {}: no DMA channel {:?}: {:?}\n",
                svc.tag,
                fe,
                stream.dma,
                e
            );
            svc.put();
            return e.to_errno();
        }
    };
    let mut ret = pcm_open_dmaengine(substream, chan, stream.formats);
    if ret >= 0 && !matches!(stream.svc, Svc::Spkr) {
        // SAFETY: the runtime exists for the op.
        ret = unsafe {
            bindings::snd_pcm_hw_constraint_step(
                (*substream).runtime,
                0,
                bindings::SNDRV_PCM_HW_PARAM_PERIOD_BYTES as _,
                ADMAC_BURST_BYTES as _,
            )
        };
        if ret < 0 {
            // SAFETY: the dmaengine runtime owns the channel since the open succeeded.
            unsafe { bindings::snd_dmaengine_pcm_close_release_chan(substream) };
            svc.put();
            return ret;
        }
    }
    if ret < 0 {
        // SAFETY: the channel was requested above and is not attached to the substream.
        unsafe { bindings::dma_release_channel(chan) };
        svc.put();
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
    let (ctx, rtd) = unsafe { (context(component), rtd_of(substream)) };
    // SAFETY: as above.
    if unsafe { is_be(rtd) } {
        return 0;
    }
    // SAFETY: as above.
    let Some((_, stream)) = (unsafe { fe_stream(rtd, substream) }) else {
        return 0;
    };
    // The channel goes first, with the fabric still powered by this front-end's reference. SAFETY:
    // the dmaengine runtime was set up by `pcm_open`.
    let ret = unsafe { bindings::snd_dmaengine_pcm_close_release_chan(substream) };
    ctx.service(stream.svc).put();
    ret
}

/// The ADMAC moves one frame per burst of up to four words: FRAME_4_WORDS for every stream, as the
/// ADT's records for these streams select (0x22 32-bit, 0x21 16-bit; the jack's records list both).
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

/// Back-end startup: `pw1 ` (the first user attaches the service).
///
/// # Safety
///
/// Called by ASoC with a live DAI of this component and a live substream.
unsafe extern "C" fn be_startup(
    substream: *mut bindings::snd_pcm_substream,
    dai: *mut bindings::snd_soc_dai,
) -> i32 {
    // SAFETY: by the caller's contract.
    match unsafe { be_service(dai, substream) }.0.get() {
        Ok(()) => 0,
        Err(e) => e.to_errno(),
    }
}

/// Prepare: `pwrd`. ASoC prepares this CPU DAI before it unmutes the amps or the codec.
///
/// # Safety
///
/// Called by ASoC with a live DAI of this component and a live substream.
unsafe extern "C" fn be_prepare(
    substream: *mut bindings::snd_pcm_substream,
    dai: *mut bindings::snd_soc_dai,
) -> i32 {
    // SAFETY: by the caller's contract.
    let (svc, runs) = unsafe { be_service(dai, substream) };
    if !runs {
        return 0;
    }
    match svc.run() {
        Ok(()) => 0,
        Err(e) => e.to_errno(),
    }
}

/// hw_free: back to `pw1 `. ASoC mutes the amps or the codec before it frees the CPU DAI.
///
/// # Safety
///
/// Called by ASoC with a live DAI of this component and a live substream.
unsafe extern "C" fn be_hw_free(
    substream: *mut bindings::snd_pcm_substream,
    dai: *mut bindings::snd_soc_dai,
) -> i32 {
    // SAFETY: by the caller's contract.
    let (svc, runs) = unsafe { be_service(dai, substream) };
    if runs {
        svc.stop();
    }
    0
}

/// # Safety
///
/// Called by ASoC with a live DAI of this component and a live substream.
unsafe extern "C" fn be_shutdown(
    substream: *mut bindings::snd_pcm_substream,
    dai: *mut bindings::snd_soc_dai,
) {
    // SAFETY: by the caller's contract.
    unsafe { be_service(dai, substream) }.0.put();
}

struct DaiOps {
    fe: bindings::snd_soc_dai_ops,
    be: bindings::snd_soc_dai_ops,
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
            be: bindings::snd_soc_dai_ops {
                startup: Some(be_startup),
                prepare: Some(be_prepare),
                hw_free: Some(be_hw_free),
                shutdown: Some(be_shutdown),
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

/// The speaker component: [0..=2] the front-ends, [3] the back-end. The order is ABI
/// (`sound-dai = <&aop_audio 3>`).
fn speaker_dais(ops: &DaiOps) -> [bindings::snd_soc_dai_driver; 4] {
    let mut dais = [
        bindings::snd_soc_dai_driver {
            name: c_str!("j616-pcm-0").as_char_ptr(),
            id: FE_DAI_ID_BASE,
            ops: &ops.fe,
            // No stream of this card's: both directions exist only for macaudio's fixed routes
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
            ops: &ops.be,
            playback: stream(c_str!("SPKR TX"), PLAYBACK_FORMATS, SPKR_CHANNELS),
            capture: stream(c_str!("SPKR RX"), SENSE_FORMATS, SENSE_CHANNELS),
            ..Default::default()
        },
    ];
    // The sense runs at the playback's rate (macaudio reads the back-end's symmetric rate).
    dais[3].set_symmetric_rate(1);
    dais
}

/// The jack component: [0] the jack's front-end, [1] the second front-end macaudio's fixed routes
/// need, [2] the back-end (`sound-dai = <&aop_audio 4>`, see `jack_of_xlate_dai_name`).
fn jack_dais(ops: &DaiOps) -> [bindings::snd_soc_dai_driver; 3] {
    let mut dais = [
        bindings::snd_soc_dai_driver {
            name: c_str!("j616-jack-pcm-0").as_char_ptr(),
            id: FE_DAI_ID_BASE + 3,
            ops: &ops.fe,
            playback: stream(c_str!("PCM0 TX"), PLAYBACK_FORMATS, JACK_CHANNELS),
            capture: stream(c_str!("PCM0 RX"), PLAYBACK_FORMATS, HEADSET_CHANNELS),
            ..Default::default()
        },
        bindings::snd_soc_dai_driver {
            name: c_str!("j616-jack-pcm-1").as_char_ptr(),
            id: FE_DAI_ID_BASE + 4,
            ops: &ops.fe,
            // No stream: "PCM1 TX" exists for macaudio's fixed routes; opening it fails (ENODEV).
            playback: stream(c_str!("PCM1 TX"), PLAYBACK_FORMATS, JACK_CHANNELS),
            ..Default::default()
        },
        bindings::snd_soc_dai_driver {
            name: c_str!("jack").as_char_ptr(),
            id: BE_DAI_ID_JACK,
            ops: &ops.be,
            playback: stream(c_str!("JACK TX"), PLAYBACK_FORMATS, JACK_CHANNELS),
            capture: stream(c_str!("JACK RX"), PLAYBACK_FORMATS, HEADSET_CHANNELS),
            ..Default::default()
        },
    ];
    // Both directions share the serializer's frame.
    dais[2].set_symmetric_rate(1);
    dais
}

/// Both components sit on the audio node, and ASoC resolves a `sound-dai` there by asking each
/// component in turn (newest first) until one answers. The speaker component answers 0..=3 by
/// table index and refuses 4 (EINVAL, out of range). This one answers 4 and defers the rest: a
/// speaker card that probes after this component registered but before the speaker component did
/// must wait for it, not fail.
///
/// # Safety
///
/// Called by ASoC with live arguments.
unsafe extern "C" fn jack_of_xlate_dai_name(
    _component: *mut bindings::snd_soc_component,
    args: *const bindings::of_phandle_args,
    dai_name: *mut *const kernel::ffi::c_char,
) -> i32 {
    // SAFETY: by the caller's contract.
    unsafe {
        if (*args).args_count != 1 || (*args).args[0] != DT_DAI_JACK {
            return EPROBE_DEFER.to_errno();
        }
        *dai_name = c_str!("jack").as_char_ptr();
    }
    0
}

/// One registered component; dropping it unregisters it (and with it any card that uses its DAIs).
struct Registration {
    dev: ARef<device::Device>,
    ctx: KBox<AsocContext>,
}

impl Registration {
    fn new(
        aop: &SndSocAopData,
        services: [&Arc<Service>; 3],
        component: bindings::snd_soc_component_driver,
        dai_drivers: fn(&DaiOps) -> Result<KVec<Opaque<bindings::snd_soc_dai_driver>>>,
    ) -> Result<Self> {
        let ops = KBox::new(DaiOps::new(), GFP_KERNEL)?;
        let dais = dai_drivers(&ops)?;
        let [spkr, cout, cin] = services.map(Arc::clone);
        let ctx = KBox::new(
            AsocContext {
                component,
                spkr,
                cout,
                cin,
                dais,
                _ops: ops,
            },
            GFP_KERNEL,
        )?;
        // SAFETY: the descriptor and the DAI table are heap allocations owned by `ctx`, which
        // `Self` keeps until after unregistration.
        let ret = unsafe {
            bindings::snd_soc_register_component_d(
                aop.dev.as_raw(),
                &ctx.component,
                Opaque::cast_into(ctx.dais.as_ptr()),
                ctx.dais.len() as i32,
            )
        };
        if ret < 0 {
            return Err(Error::from_errno(ret));
        }
        Ok(Self {
            dev: aop.dev.clone(),
            ctx,
        })
    }
}

impl Drop for Registration {
    fn drop(&mut self) {
        // SAFETY: registered in `new`; the context stays allocated until unregistration has
        // finished every PCM and DAI callback.
        unsafe {
            bindings::snd_soc_unregister_component_by_driver(self.dev.as_raw(), &self.ctx.component)
        };
    }
}

fn to_kvec<const N: usize>(
    dais: [bindings::snd_soc_dai_driver; N],
) -> Result<KVec<Opaque<bindings::snd_soc_dai_driver>>> {
    let mut v = KVec::with_capacity(N, GFP_KERNEL)?;
    for dai in dais {
        v.push(Opaque::new(dai), GFP_KERNEL)?;
    }
    Ok(v)
}

fn component_driver(name: &'static CStr) -> bindings::snd_soc_component_driver {
    bindings::snd_soc_component_driver {
        name: name.as_char_ptr(),
        open: Some(pcm_open),
        close: Some(pcm_close),
        hw_params: Some(pcm_hw_params),
        trigger: Some(pcm_trigger),
        sync_stop: Some(pcm_sync_stop),
        pointer: Some(pcm_pointer),
        ..Default::default()
    }
}

/// The speaker component, and the jack's where the audio node lists its DMA channels.
pub(crate) struct SpeakerComponent {
    _spkr: Registration,
    _jack: Option<Registration>,
}

impl SpeakerComponent {
    pub(crate) fn register(aop: Arc<SndSocAopData>) -> Result<Self> {
        let spkr = Service::new(aop.clone(), AUDIO_DEV_SPKR, "M4_SPKR", PWRD_OUTPUT)?;
        let cout = Service::new(aop.clone(), AUDIO_DEV_COUT, "M4_JACK cout", PWRD_OUTPUT)?;
        let cin = Service::new(aop.clone(), AUDIO_DEV_CIN, "M4_JACK cin", PWRD_INPUT)?;
        let services = [&spkr, &cout, &cin];
        let has_jack = [c_str!("cout"), c_str!("cin")].iter().all(|name| {
            aop.fwnode
                .property_match_string(c_str!("dma-names"), name)
                .is_ok()
        });
        // The jack goes first (see `jack_of_xlate_dai_name`); its failure costs the jack only.
        let jack = if has_jack {
            let mut driver = component_driver(c_str!("t6040-aop-jack"));
            driver.of_xlate_dai_name = Some(jack_of_xlate_dai_name);
            match Registration::new(&aop, services, driver, |ops| to_kvec(jack_dais(ops))) {
                Ok(r) => {
                    dev_info!(
                        aop.dev,
                        "M4_JACK component registered (cout/cin streams, base-ns ADMAC channels 4 and 5)\n"
                    );
                    Some(r)
                }
                Err(e) => {
                    dev_err!(
                        aop.dev,
                        "M4_JACK unable to register the ASoC component: {:?}\n",
                        e
                    );
                    None
                }
            }
        } else {
            None
        };
        let spkr = Registration::new(
            &aop,
            services,
            component_driver(c_str!("t6040-aop-spkr")),
            |ops| to_kvec(speaker_dais(ops)),
        )
        .inspect_err(|e| {
            dev_err!(
                aop.dev,
                "M4_SPKR unable to register the ASoC component: {}\n",
                e.to_errno()
            )
        })?;
        dev_info!(
            aop.dev,
            "M4_SPKR component registered (spkr stream, base-ns ADMAC channels 0 and 1)\n"
        );
        Ok(Self {
            _spkr: spkr,
            _jack: jack,
        })
    }
}

// SAFETY: the raw pointers in the contexts are C descriptors that are only read after
// registration.
unsafe impl Send for SpeakerComponent {}
// SAFETY: as above.
unsafe impl Sync for SpeakerComponent {}
