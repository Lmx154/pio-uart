//! On-board check of `pio_uart::UartRx`, in loop-back on GPIO34 (the ESC telemetry input of the MARV V2
//! board; nothing sends on it now). No real device is on the wire.
//!
//! Who is where, all in PIO1 (GPIOBASE 16, window GPIO16-47, needed for a pin above 31):
//! - state machine 0: a dummy program, loaded and running before `UartRx` is built, to show that
//!   build and release leave a neighbour alone (it stands for the LED driver of the firmware);
//! - state machine 1: `UartRx` on GPIO34, pad pull-up on (the code under test);
//! - state machine 2: the SENDER, test code: `out pins, 1 [15]` with autopull, one bit per 16 PIO
//!   cycles from words the CPU packs, so any bit pattern, any baud rate and a low stop bit are possible.
//!   It drives GPIO34 push-pull, high when idle.
//!
//! A low level no state machine could make (a runt pulse, a long low line) is made with the pad's
//! output override in IO_BANK0, which beats every PIO latch.
//!
//! GPIO36-39 (the ESC signal lines) are inputs with pad pull-ups from start to end and are never
//! driven; nothing is sent to an ESC.
#![no_std]
#![no_main]

use cortex_m::peripheral::DWT;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::gpio::{Input, Level, Pull};
use embassy_rp::peripherals::PIO1;
use embassy_rp::pio::program::pio_asm;
use embassy_rp::pio::{
    self, Common, Config as PioConfig, Direction, FifoJoin, Pio, ShiftConfig, ShiftDirection,
    StateMachine,
};
use embassy_rp::{bind_interrupts, pac};
use embassy_time::{Duration, Timer, with_timeout};
use embedded_io_async::Read;
use fixed::types::U24F8;
use panic_probe as _;
use pio_uart::timing::{self, Divider};
use pio_uart::{Config, UartRx, UartRxBuildError, UartRxError, UartRxParts, UartRxStats};

bind_interrupts!(struct Irqs {
    PIO1_IRQ_0 => pio::InterruptHandler<PIO1>;
});

macro_rules! check {
    ($fails:expr, $cond:expr, $fmt:literal $(, $arg:expr)* $(,)?) => {
        if $cond {
            defmt::info!($fmt, "PASS" $(, $arg)*);
        } else {
            $fails += 1;
            defmt::error!($fmt, "FAIL" $(, $arg)*);
        }
    };
}

const CLK_HZ: u32 = 150_000_000;
const BAUD: u32 = 115_200;
const GPIO: usize = 34;
/// GPIO34 is bit 2 of SIO bank 1.
const SIO_BIT: u32 = 2;
/// 16 PIO cycles per bit, 8 bits in a word of 32: bytes of the tolerance runs.
const STREAM_BYTES: usize = 4608;
const ESC_FIRST: usize = 36;

type Rx = UartRx<'static, PIO1, 1>;

// ---------------------------------------------------------------------------------------------
// The line
// ---------------------------------------------------------------------------------------------

fn pad_high() -> bool {
    (pac::SIO.gpio_in(1).read() >> SIO_BIT) & 1 != 0
}

/// Forces the pad low (output enabled, level low) whatever the PIO latches say, or gives it back.
fn force_low(on: bool) {
    pac::IO_BANK0.gpio(GPIO).ctrl().modify(|w| {
        if on {
            w.set_oeover(pac::io::vals::Oeover::ENABLE);
            w.set_outover(pac::io::vals::Outover::LOW);
        } else {
            w.set_oeover(pac::io::vals::Oeover::NORMAL);
            w.set_outover(pac::io::vals::Outover::NORMAL);
        }
    });
}

fn spin(cycles: u32) {
    let t0 = DWT::cycle_count();
    while DWT::cycle_count().wrapping_sub(t0) < cycles {}
}

/// A low pulse of about `cycles` CPU cycles, interrupts off.
fn low_pulse(cycles: u32) {
    cortex_m::interrupt::free(|_| {
        force_low(true);
        spin(cycles);
        force_low(false);
    });
}

/// The ESC signal lines GPIO36-39: pad pull-up on, no pull-down, owned by SIO (no PIO block), no
/// output enable, no override.
fn esc_ok() -> bool {
    let mut ok = pac::SIO.gpio_oe(1).value().read() & (0xF << (ESC_FIRST as u32 - 32)) == 0;
    for g in ESC_FIRST..ESC_FIRST + 4 {
        let pad = pac::PADS_BANK0.gpio(g).read();
        let ctrl = pac::IO_BANK0.gpio(g).ctrl().read();
        ok &= pad.pue()
            && !pad.pde()
            && ctrl.funcsel() == 5
            && ctrl.oeover() == pac::io::vals::Oeover::NORMAL
            && ctrl.outover() == pac::io::vals::Outover::NORMAL;
    }
    ok
}

// ---------------------------------------------------------------------------------------------
// Bytes and bits
// ---------------------------------------------------------------------------------------------

/// The byte at position `i`: the first 256 hold every value once, the rest look random.
fn expected(i: usize) -> u8 {
    if i < 256 {
        (i as u8).wrapping_mul(197).wrapping_add(13)
    } else {
        (((i as u32).wrapping_mul(2_654_435_761) >> 13) ^ i as u32) as u8
    }
}

/// The 10 bits of one frame, first bit sent in bit 0: start 0, data LSB first, stop.
fn frame(byte: u8, stop: bool) -> (u32, u32) {
    ((u32::from(byte) << 1) | (u32::from(stop) << 9), 10)
}

/// Packs `(bits, length)` chunks into 32-bit words, first bit in bit 0; the last word is filled
/// with ones and followed by two words of ones, so the line ends high.
struct Packer<I> {
    it: I,
    acc: u64,
    n: u32,
    tail: u32,
}

impl<I: Iterator<Item = (u32, u32)>> Packer<I> {
    fn new(it: I) -> Self {
        Self {
            it,
            acc: 0,
            n: 0,
            tail: 2,
        }
    }
}

impl<I: Iterator<Item = (u32, u32)>> Iterator for Packer<I> {
    type Item = u32;
    fn next(&mut self) -> Option<u32> {
        while self.n < 32 {
            match self.it.next() {
                Some((v, l)) => {
                    self.acc |= u64::from(v & ((1u32 << l) - 1)) << self.n;
                    self.n += l;
                }
                None => break,
            }
        }
        if self.n >= 32 {
            let w = self.acc as u32;
            self.acc >>= 32;
            self.n -= 32;
            return Some(w);
        }
        if self.n > 0 {
            let w = (self.acc as u32) | (u32::MAX << self.n);
            self.n = 0;
            self.acc = 0;
            return Some(w);
        }
        if self.tail > 0 {
            self.tail -= 1;
            return Some(u32::MAX);
        }
        None
    }
}

// ---------------------------------------------------------------------------------------------
// The sender (PIO1, state machine 2)
// ---------------------------------------------------------------------------------------------

struct Sender {
    sm: StateMachine<'static, PIO1, 2>,
    cfg: PioConfig<'static, PIO1>,
    origin: u8,
    baud: u32,
}

impl Sender {
    /// Sets the bit rate (16 PIO cycles per bit), with the state machine stopped and the line
    /// high. Returns the divider.
    fn set_baud(&mut self, baud: u32) -> Divider {
        let d = timing::divider(CLK_HZ, baud).unwrap();
        self.baud = baud;
        self.cfg.clock_divider = U24F8::from_bits(d.q8);
        self.sm.set_enable(false);
        self.sm.set_config(&self.cfg);
        self.sm.restart();
        self.sm.clear_fifos();
        // SAFETY: the state machine is disabled; the jump only sets its program counter.
        unsafe { self.sm.exec_jmp(self.origin) };
        let _ = self.sm.tx().stalled();
        d
    }

    /// Sends the words, one bit per 16 PIO cycles, first bit in bit 0 of the first word, and
    /// returns when they have left. True when the CPU kept the FIFO fed (no underrun).
    async fn send<I: Iterator<Item = u32>>(&mut self, mut words: I) -> bool {
        let mut held = None;
        for w in words.by_ref() {
            if !self.sm.tx().try_push(w) {
                held = Some(w);
                break;
            }
        }
        self.sm.set_enable(true);
        let _ = self.sm.tx().stalled();
        let mut next = held.or_else(|| words.next());
        while let Some(w) = next {
            self.sm.tx().wait_push(w).await;
            next = words.next();
        }
        let fed = !self.sm.tx().stalled();
        // up to 8 words in the FIFO and one in the shifter are still to go out
        Timer::after_micros((9 * 32 + 16) * 1_000_000 / u64::from(self.baud)).await;
        fed
    }

    async fn send_bytes(&mut self, bytes: &[u8]) -> bool {
        self.send(Packer::new(bytes.iter().map(|&b| frame(b, true))))
            .await
    }
}

// ---------------------------------------------------------------------------------------------
// Receiving
// ---------------------------------------------------------------------------------------------

struct Delta {
    bytes_received: u32,
    framing_errors: u32,
    overruns: u32,
    runt_starts: u32,
}

#[derive(Default)]
struct Got {
    n: usize,
    wrong: usize,
    overruns: u32,
    seen: [u32; 8],
}

impl Got {
    fn values(&self) -> u32 {
        self.seen.iter().map(|w| w.count_ones()).sum()
    }
}

/// Reads until the line has been quiet for 30 ms, comparing with `expected` in order.
async fn collect(rx: &mut Rx, got: &mut Got) {
    let mut buf = [0u8; 32];
    loop {
        match with_timeout(Duration::from_millis(30), rx.read(&mut buf)).await {
            Ok(Ok(n)) => {
                for &b in buf.iter().take(n) {
                    if b == expected(got.n) {
                        if let Some(w) = got.seen.get_mut(usize::from(b) / 32) {
                            *w |= 1 << (b % 32);
                        }
                    } else {
                        got.wrong += 1;
                    }
                    got.n += 1;
                }
            }
            Ok(Err(UartRxError::Overrun)) => got.overruns += 1,
            Ok(Err(_)) => {}
            Err(_) => break,
        }
    }
}

/// The bytes `n` bytes of `expected`, sent back to back at `baud`, received by `rx`.
async fn stream(rx: &mut Rx, tx: &mut Sender, baud: u32, count: usize) -> (Got, bool, Delta) {
    tx.set_baud(baud);
    let s0 = rx.stats();
    let mut got = Got::default();
    let words = Packer::new((0..count).map(|i| frame(expected(i), true)));
    let (fed, ()) = join(tx.send(words), collect(rx, &mut got)).await;
    let s1 = rx.stats();
    let (bytes_received, framing_errors, overruns, runt_starts) = delta(s0, s1);
    let d = Delta {
        bytes_received,
        framing_errors,
        overruns,
        runt_starts,
    };
    (got, fed, d)
}

/// Reads for up to `ms` and returns the bytes (at most 32), swallowing overruns.
async fn read_some(rx: &mut Rx, out: &mut [u8; 32], ms: u64) -> usize {
    let mut n = 0;
    loop {
        match with_timeout(Duration::from_millis(ms), rx.read(&mut out[n..])).await {
            Ok(Ok(k)) => {
                n += k;
                if n == out.len() {
                    return n;
                }
            }
            Ok(Err(_)) => {}
            Err(_) => return n,
        }
    }
}

fn delta(a: UartRxStats, b: UartRxStats) -> (u32, u32, u32, u32) {
    (
        b.bytes_received - a.bytes_received,
        b.framing_errors - a.framing_errors,
        b.overruns - a.overruns,
        b.runt_starts - a.runt_starts,
    )
}

// ---------------------------------------------------------------------------------------------
// Build, release, neighbour
// ---------------------------------------------------------------------------------------------

type Spare = UartRxParts<'static, PIO1, 1>;

fn build(
    c: &mut Common<'static, PIO1>,
    spare: Spare,
    baud: u32,
) -> Result<Rx, (UartRxBuildError, Spare)> {
    let config = Config::new().with_baud(baud).with_clk_sys_hz(CLK_HZ);
    match UartRx::new(c, spare.sm, spare.pin, config) {
        Ok(rx) => Ok(rx),
        Err(e) => Err((
            e.reason,
            UartRxParts {
                sm: e.sm,
                pin: e.pin,
            },
        )),
    }
}

/// What the neighbour's registers and instructions look like.
#[derive(PartialEq, Eq, Clone, Copy)]
struct Neighbour {
    pinctrl: u32,
    execctrl: u32,
    shiftctrl: u32,
    code: [u16; 4],
}

fn neighbour_snapshot(origin: usize) -> Neighbour {
    let sm = pac::PIO1.sm(0);
    let mut code = [0u16; 4];
    for (i, c) in code.iter_mut().enumerate() {
        *c = pac::PIO1.instr_mem((origin + i) % 32).read().instr_mem();
    }
    Neighbour {
        pinctrl: sm.pinctrl().read().0,
        execctrl: sm.execctrl().read().0,
        shiftctrl: sm.shiftctrl().read().0,
        code,
    }
}

/// The dummy neighbour is enabled and its counter moves over 1 ms.
fn neighbour_running(sm: &mut StateMachine<'static, PIO1, 0>) -> bool {
    let enabled = pac::PIO1.ctrl().read().sm_enable() & 1 != 0;
    let sample = |sm: &mut StateMachine<'static, PIO1, 0>| {
        while sm.rx().try_pull().is_some() {}
        for _ in 0..1_000_000 {
            if let Some(v) = sm.rx().try_pull() {
                return Some(v);
            }
        }
        None
    };
    let a = sample(sm);
    spin(150_000);
    let b = sample(sm);
    enabled && a.is_some() && b.is_some() && a != b
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    defmt::info!("[INFO] uart_rx_loopback: start");
    let mut p = embassy_rp::init(Default::default());
    let mut core = cortex_m::Peripherals::take().unwrap();
    core.DCB.enable_trace();
    core.DWT.enable_cycle_counter();
    let mut fails = 0u32;
    check!(
        fails,
        embassy_rp::clocks::clk_sys_freq() == CLK_HZ,
        "[{=str}] clk_sys is 150 MHz"
    );

    // GPIO36-39: inputs with pad pull-ups for the whole program, never driven.
    let _esc = [
        Input::new(p.PIN_36, Pull::Up),
        Input::new(p.PIN_37, Pull::Up),
        Input::new(p.PIN_38, Pull::Up),
        Input::new(p.PIN_39, Pull::Up),
    ];
    check!(
        fails,
        esc_ok(),
        "[{=str}] GPIO36-39 are inputs with pad pull-ups, owned by SIO, not driven (start)"
    );

    // GPIO34 must follow its pad pull-up (nothing holds it low). The pull-down is only reported:
    // an ESC that idles its telemetry output high through the 1 k resistor would show high here.
    let (up, down_high, up2) = {
        let pin = &mut p.PIN_34;
        let up = {
            let i = Input::new(pin.reborrow(), Pull::Up);
            Timer::after_millis(2).await;
            i.is_high()
        };
        let down_high = {
            let i = Input::new(pin.reborrow(), Pull::Down);
            Timer::after_millis(5).await;
            i.is_high()
        };
        let up2 = {
            let i = Input::new(pin.reborrow(), Pull::Up);
            Timer::after_millis(2).await;
            i.is_high()
        };
        (up, down_high, up2)
    };
    defmt::info!(
        "[STAT] GPIO34 before anything is driven: pull-up reads high {=bool}, pull-down reads high {=bool} (true: something outside holds it high), pull-up again {=bool}",
        up,
        down_high,
        up2
    );
    check!(
        fails,
        up && up2,
        "[{=str}] GPIO34 reads high with its pad pull-up (nothing holds it low)"
    );
    if !(up && up2) {
        defmt::error!("[FAIL] GPIO34 is held low from outside: stopping before anything is driven");
        hil_common::exit(fails);
    }

    let Pio {
        common: mut c,
        sm0: dummy_sm,
        sm1,
        sm2: sender_sm,
        ..
    } = Pio::new(p.PIO1, Irqs);
    let mut dummy_sm = dummy_sm;
    // The sender's pin and the receiver's pin are two handles of GPIO34 in the same PIO block.
    // SAFETY: the sender drives the pin that `UartRx` reads, on purpose; this program is the only
    // user of GPIO34 and the two handles are never used to configure the pad at the same time.
    let mut sender_pin = c.make_pio_pin(unsafe { p.PIN_34.clone_unchecked() });
    let rx_pin = c.make_pio_pin(p.PIN_34);
    sender_pin.set_pull(Pull::Up);
    let mut spare: Spare = UartRxParts {
        sm: sm1,
        pin: rx_pin,
    };

    // The neighbour: a counter that pushes its value, 4 instructions, in the same window.
    let dummy_prg = pio_asm!(
        ".wrap_target",
        "    mov isr, x",
        "    push noblock",
        "    jmp x-- after",
        "after:",
        "    nop [31]",
        ".wrap",
    );
    let dummy_loaded = c.load_program(&dummy_prg.program);
    let dummy_origin = usize::from(dummy_loaded.origin);
    {
        let mut cfg = PioConfig::<PIO1>::default();
        cfg.use_program(&dummy_loaded, &[]);
        cfg.clock_divider = U24F8::from_num(100);
        cfg.set_in_pins(&[&sender_pin]);
        dummy_sm.set_config(&cfg);
        dummy_sm.set_enable(true);
    }

    // The sender: one instruction, autopull of 32 bits, 16 cycles per bit.
    let send_prg = pio_asm!("    out pins, 1 [15]");
    let send_loaded = c.load_program(&send_prg.program);
    let mut cfg = PioConfig::<PIO1>::default();
    cfg.use_program(&send_loaded, &[]);
    cfg.shift_out = ShiftConfig {
        threshold: 32,
        direction: ShiftDirection::Right,
        auto_fill: true,
    };
    cfg.fifo_join = FifoJoin::TxOnly;
    cfg.set_out_pins(&[&sender_pin]);
    let mut tx = Sender {
        sm: sender_sm,
        cfg,
        origin: send_loaded.origin,
        baud: BAUD,
    };
    tx.set_baud(BAUD);
    tx.sm.set_pins(Level::High, &[&sender_pin]);
    tx.sm.set_pin_dirs(Direction::Out, &[&sender_pin]);
    Timer::after_millis(2).await;
    check!(
        fails,
        pad_high(),
        "[{=str}] the sender holds GPIO34 high when idle"
    );

    let before = neighbour_snapshot(dummy_origin);
    check!(
        fails,
        neighbour_running(&mut dummy_sm),
        "[{=str}] the neighbour on PIO1 state machine 0 runs before UartRx is built"
    );

    // Errors hand the parts back, and nothing is left loaded.
    let filler = pio_asm!(
        "    nop", "    nop", "    nop", "    nop", "    nop", "    nop", "    nop", "    nop",
        "    nop", "    nop", "    nop", "    nop", "    nop", "    nop", "    nop", "    nop",
        "    nop", "    nop", "    nop", "    nop", "    nop", "    nop", "    nop", "    nop",
        "    nop", "    nop", "    nop",
    );
    let fill = c.try_load_program(&filler.program);
    check!(
        fails,
        fill.is_ok(),
        "[{=str}] the 27 instructions left by the neighbour (4) and the sender (1) are free"
    );
    if let Ok(fill) = fill {
        match build(&mut c, spare, BAUD) {
            Ok(rx) => {
                let parts = rx.release(&mut c);
                spare = parts;
                check!(
                    fails,
                    false,
                    "[{=str}] a block with no room refuses UartRx::new"
                );
            }
            Err((reason, back)) => {
                spare = back;
                check!(
                    fails,
                    reason == UartRxBuildError::NoSpace,
                    "[{=str}] a block with no room refuses UartRx::new (NoSpace), the state machine comes back"
                );
            }
        }
        // SAFETY: the filler is never run.
        unsafe { c.free_instr(fill.used_memory) };
    }
    match build(&mut c, spare, 0) {
        Ok(rx) => {
            let parts = rx.release(&mut c);
            spare = parts;
            check!(fails, false, "[{=str}] baud 0 is refused");
        }
        Err((reason, back)) => {
            spare = back;
            check!(
                fails,
                reason == UartRxBuildError::InvalidBaud,
                "[{=str}] baud 0 is refused (InvalidBaud), the state machine comes back"
            );
        }
    }

    // ---------------------------------------------------------------------------------------
    // The receiver on GPIO34
    // ---------------------------------------------------------------------------------------
    let mut rx = match build(&mut c, spare, BAUD) {
        Ok(rx) => rx,
        Err((reason, _)) => {
            defmt::error!("[FAIL] UartRx::new failed: {}", reason);
            hil_common::exit(fails + 1);
        }
    };
    let d = rx.divider();
    defmt::info!(
        "[INFO] divider {=u32} + {=u8}/256, error {=i32} ppm; sample cycles start {=u32}, stop {=u32}",
        d.integer(),
        d.fraction(),
        d.error_ppm,
        timing::uart_rx_sample_cycle(0),
        timing::uart_rx_sample_cycle(9)
    );

    // Criterion 3: the window and the JMP pin field, read back from the registers.
    let gpiobase = if pac::PIO1.gpiobase().read().gpiobase() {
        16u8
    } else {
        0
    };
    let jmp = pac::PIO1.sm(1).execctrl().read().jmp_pin();
    let in_base = pac::PIO1.sm(1).pinctrl().read().in_base();
    defmt::info!(
        "[STAT] GPIO{=usize}: GPIOBASE {=u8}, JMP pin field {=u8}, in base {=u8}; counted from GPIOBASE: {=u8}",
        GPIO,
        gpiobase,
        jmp,
        in_base,
        timing::pin_field(GPIO as u8).unwrap_or(255)
    );
    check!(
        fails,
        gpiobase == 16 && jmp == 18 && in_base == 18,
        "[{=str}] GPIOBASE 16, JMP pin field 18 (GPIO34 - 16, not 34), in base 18"
    );
    check!(
        fails,
        pac::PADS_BANK0.gpio(GPIO).read().pue() && !pac::PADS_BANK0.gpio(GPIO).read().pde(),
        "[{=str}] the pad pull-up of GPIO34 is on"
    );
    check!(
        fails,
        neighbour_snapshot(dummy_origin) == before && neighbour_running(&mut dummy_sm),
        "[{=str}] the neighbour's registers and code are unchanged and it runs with UartRx built"
    );

    // Tolerance: 4608 bytes, back to back, one stop bit, all 256 values.
    let rx_q8 = timing::divider(CLK_HZ, BAUD).unwrap().q8;
    for (name, baud) in [
        ("-3 %", BAUD / 100 * 97),
        ("exact", BAUD),
        ("+3 %", BAUD / 100 * 103),
    ] {
        let tq8 = timing::divider(CLK_HZ, baud).unwrap();
        let ratio_ppm = (i64::from(rx_q8) * 1_000_000 / i64::from(tq8.q8)) - 1_000_000;
        let (got, fed, s) = stream(&mut rx, &mut tx, baud, STREAM_BYTES).await;
        defmt::info!(
            "[STAT] sender {=str}: {=u32} baud (divider error {=i32} ppm, rate against receiver {=i64} ppm): sent {=usize}, got {=usize}, wrong {=usize}, values {=u32}, framing {=u32}, runts {=u32}, overruns {=u32}, fed {=bool}",
            name,
            baud,
            tq8.error_ppm,
            ratio_ppm,
            STREAM_BYTES,
            got.n,
            got.wrong,
            got.values(),
            s.framing_errors,
            s.runt_starts,
            got.overruns + s.overruns,
            fed
        );
        check!(
            fails,
            fed && got.n == STREAM_BYTES
                && got.wrong == 0
                && got.values() == 256
                && s.framing_errors == 0
                && s.runt_starts == 0
                && got.overruns == 0
                && s.overruns == 0
                && s.bytes_received as usize == STREAM_BYTES,
            "[{=str}] 4608 bytes back to back, sender {=str}: all received, all right, 256 values, 0 errors",
            name
        );
    }
    let (low, high) = timing::uart_rx_tolerance_ppm();
    defmt::info!(
        "[INFO] tolerance by arithmetic: {=i32} ppm to +{=i32} ppm of baud (not measured beyond the runs here)",
        low,
        high
    );

    // Negative controls: the check must report errors when the sender is 6 % off.
    for (name, baud) in [("-6 %", BAUD / 100 * 94), ("+6 %", BAUD / 100 * 106)] {
        let tq8 = timing::divider(CLK_HZ, baud).unwrap();
        let ratio_ppm = (i64::from(rx_q8) * 1_000_000 / i64::from(tq8.q8)) - 1_000_000;
        let (got, fed, s) = stream(&mut rx, &mut tx, baud, STREAM_BYTES).await;
        let missing = STREAM_BYTES.saturating_sub(got.n);
        let errors = got.wrong + missing + s.framing_errors as usize + s.runt_starts as usize;
        defmt::info!(
            "[STAT] NEGATIVE CONTROL sender {=str}: {=u32} baud (rate against receiver {=i64} ppm): sent {=usize}, got {=usize}, wrong {=usize}, missing {=usize}, framing {=u32}, runts {=u32}, overruns {=u32}, errors {=usize}",
            name,
            baud,
            ratio_ppm,
            STREAM_BYTES,
            got.n,
            got.wrong,
            missing,
            s.framing_errors,
            s.runt_starts,
            got.overruns + s.overruns,
            errors
        );
        check!(
            fails,
            fed && got.n > 0 && errors > 0,
            "[{=str}] control: with the sender at {=str} (it ran: FIFO kept fed, bytes arrived) the check reports errors",
            name
        );
    }
    tx.set_baud(BAUD);
    Timer::after_millis(2).await;

    let mut buf = [0u8; 32];

    // A good byte after all that, and a pending read dropped after 5 ms loses nothing.
    let s0 = rx.stats();
    let dropped = with_timeout(Duration::from_millis(5), rx.read(&mut buf))
        .await
        .is_err();
    tx.send_bytes(&[1, 2, 3, 4, 5]).await;
    let n = read_some(&mut rx, &mut buf, 5).await;
    let s1 = rx.stats();
    check!(
        fails,
        dropped && n == 5 && buf[..5] == [1, 2, 3, 4, 5] && delta(s0, s1) == (5, 0, 0, 0),
        "[{=str}] a read dropped after 5 ms with no data loses nothing: 5 bytes sent later all arrive"
    );

    // Runt starts: a low pulse that is gone before the middle of the start bit.
    for (name, cycles) in [("about 3 PIO cycles", 250u32), ("about 5 PIO cycles", 400)] {
        let s0 = rx.stats();
        for _ in 0..20 {
            low_pulse(cycles);
            Timer::after_micros(400).await;
            let n = read_some(&mut rx, &mut buf, 1).await;
            if n != 0 {
                fails += 1;
                defmt::error!("[FAIL] a runt start gave a byte");
            }
        }
        let s1 = rx.stats();
        defmt::info!(
            "[STAT] 20 runt pulses of {=str} ({=u32} CPU cycles asked): bytes {=u32}, framing {=u32}, runts {=u32}",
            name,
            cycles,
            delta(s0, s1).0,
            delta(s0, s1).1,
            delta(s0, s1).3
        );
        check!(
            fails,
            delta(s0, s1) == (0, 0, 0, 20),
            "[{=str}] 20 runt starts of {=str}: no byte, no framing error, 20 counted",
            name
        );
    }
    // Each followed at once by a real byte: the byte arrives alone.
    let s0 = rx.stats();
    let mut wrong = 0u32;
    for i in 0..20u8 {
        low_pulse(250);
        // the byte starts about one bit (16 PIO cycles) after the pulse began
        spin(800);
        let byte = [0xA5 ^ i];
        tx.send_bytes(&byte).await;
        let n = read_some(&mut rx, &mut buf, 2).await;
        if n != 1 || buf[0] != byte[0] {
            wrong += 1;
        }
    }
    let s1 = rx.stats();
    defmt::info!(
        "[STAT] 20 runt pulses, each followed by a byte about one bit later: wrong {=u32}, bytes {=u32}, framing {=u32}, overruns {=u32}, runts {=u32}",
        wrong,
        delta(s0, s1).0,
        delta(s0, s1).1,
        delta(s0, s1).2,
        delta(s0, s1).3
    );
    check!(
        fails,
        wrong == 0 && delta(s0, s1) == (20, 0, 0, 20),
        "[{=str}] 20 runt starts, each followed at once by a byte: the byte arrives alone"
    );
    // Control: a low pulse of one bit time is a start bit; it gives the byte 0xFF.
    let s0 = rx.stats();
    low_pulse(1_300);
    let n = read_some(&mut rx, &mut buf, 3).await;
    let s1 = rx.stats();
    defmt::info!(
        "[STAT] CONTROL of the runt check: a low pulse of about one bit (1300 CPU cycles) gave {=usize} byte(s), first 0x{=u8:x}, runts {=u32}",
        n,
        buf[0],
        delta(s0, s1).3
    );
    check!(
        fails,
        n == 1 && buf[0] == 0xFF && delta(s0, s1) == (1, 0, 0, 0),
        "[{=str}] control: a pulse of one bit time is not a runt (byte 0xFF, runts unchanged)"
    );

    // A low stop bit: counted, no byte; the next good byte arrives.
    let s0 = rx.stats();
    let bits = [
        frame(0xA5, false),
        (0x3FF, 4),
        frame(0x3C, true),
        frame(0xC3, true),
    ];
    tx.send(Packer::new(bits.into_iter())).await;
    let n = read_some(&mut rx, &mut buf, 5).await;
    let s1 = rx.stats();
    check!(
        fails,
        n == 2 && buf[..2] == [0x3C, 0xC3] && delta(s0, s1) == (2, 1, 0, 0),
        "[{=str}] a low stop bit is counted as 1 framing error and gives no byte; the next 2 bytes arrive"
    );
    // A line held low for 3 ms (a break): one framing error, then the receiver waits for high.
    let s0 = rx.stats();
    force_low(true);
    Timer::after_millis(3).await;
    force_low(false);
    Timer::after_millis(1).await;
    tx.send_bytes(&[0x5A]).await;
    let n = read_some(&mut rx, &mut buf, 5).await;
    let s1 = rx.stats();
    check!(
        fails,
        n == 1 && buf[0] == 0x5A && delta(s0, s1) == (1, 1, 0, 0),
        "[{=str}] a line held low 3 ms (about 35 bit times): 1 framing error, no byte, no phantom after it"
    );

    // Overrun: 20 bytes with nobody reading; the next read says so; the receiver carries on.
    let s0 = rx.stats();
    let mut twenty = [0u8; 20];
    for (i, b) in twenty.iter_mut().enumerate() {
        *b = expected(i);
    }
    tx.send_bytes(&twenty).await;
    let first = rx.read(&mut buf).await;
    let flagged = matches!(first, Err(UartRxError::Overrun));
    tx.send_bytes(&[9, 8, 7, 6, 5]).await;
    let n = read_some(&mut rx, &mut buf, 5).await;
    let s1 = rx.stats();
    defmt::info!(
        "[STAT] overrun: 20 bytes sent into a FIFO of {=usize}, first read {=str}, then {=usize} fresh bytes; overruns {=u32} (one count per read that finds the flag)",
        timing::RX_JOINED_FIFO_WORDS,
        if flagged { "Overrun" } else { "data" },
        n,
        delta(s0, s1).2
    );
    check!(
        fails,
        flagged && n == 5 && buf[..5] == [9, 8, 7, 6, 5] && delta(s0, s1) == (5, 0, 1, 0),
        "[{=str}] 20 bytes with nobody reading: the next read returns Overrun (counted), the next 5 bytes arrive"
    );

    // Release and build 10 times, the neighbour untouched, instruction memory back to its start.
    let mut cycles_ok = 0;
    let mut mem_ok = 0;
    let mut near_ok = 0;
    let mut data_ok = 0;
    let parts = rx.release(&mut c);
    spare = parts;
    for _ in 0..10 {
        let fill = c.try_load_program(&filler.program);
        if let Ok(fill) = fill {
            mem_ok += 1;
            // SAFETY: the filler is never run.
            unsafe { c.free_instr(fill.used_memory) };
        }
        let Ok(mut rx) = build(&mut c, spare, BAUD) else {
            fails += 1;
            defmt::error!("[FAIL] UartRx::new failed in the build and release loop");
            hil_common::exit(fails);
        };
        cycles_ok += 1;
        let ok_near =
            neighbour_snapshot(dummy_origin) == before && neighbour_running(&mut dummy_sm);
        near_ok += u32::from(ok_near);
        tx.send_bytes(&[0x42, 0x24]).await;
        let n = read_some(&mut rx, &mut buf, 3).await;
        data_ok += u32::from(n == 2 && buf[..2] == [0x42, 0x24]);
        let parts = rx.release(&mut c);
        spare = parts;
        near_ok += u32::from(
            neighbour_snapshot(dummy_origin) == before && neighbour_running(&mut dummy_sm),
        );
    }
    defmt::info!(
        "[STAT] 10 x build and release: built {=u32}, 27 free before each build {=u32}, neighbour unchanged and running {=u32} of 20 looks, bytes right {=u32}",
        cycles_ok,
        mem_ok,
        near_ok,
        data_ok
    );
    let fill = c.try_load_program(&filler.program);
    let end_free = fill.is_ok();
    if let Ok(fill) = fill {
        // SAFETY: the filler is never run.
        unsafe { c.free_instr(fill.used_memory) };
    }
    check!(
        fails,
        cycles_ok == 10 && mem_ok == 10 && end_free && data_ok == 10,
        "[{=str}] 10 times build and release: the 27 free instructions are free before every build and at the end, a byte passes each time"
    );
    check!(
        fails,
        near_ok == 20,
        "[{=str}] the neighbour on PIO1 state machine 0 keeps its code and registers and keeps running across all 10 builds and releases"
    );

    // Built on a line that is low: no phantom byte, however long.
    force_low(true);
    Timer::after_millis(1).await;
    let Ok(mut rx) = build(&mut c, spare, BAUD) else {
        fails += 1;
        defmt::error!("[FAIL] UartRx::new failed on a low line");
        hil_common::exit(fails);
    };
    let s0 = rx.stats();
    Timer::after_millis(5).await;
    let quiet_low = read_some(&mut rx, &mut buf, 2).await == 0;
    force_low(false);
    Timer::after_millis(2).await;
    let quiet_rise = read_some(&mut rx, &mut buf, 2).await == 0;
    tx.send_bytes(&[0x96]).await;
    let n = read_some(&mut rx, &mut buf, 3).await;
    let s1 = rx.stats();
    check!(
        fails,
        quiet_low && quiet_rise && n == 1 && buf[0] == 0x96 && delta(s0, s1) == (1, 0, 0, 0),
        "[{=str}] built on a line held low 6 ms (50+ bit times): no byte, no framing error, none when it rises; the next byte arrives"
    );
    let parts = rx.release(&mut c);
    drop(parts);

    check!(
        fails,
        esc_ok(),
        "[{=str}] GPIO36-39 are still inputs with pad pull-ups, owned by SIO, not driven (end)"
    );
    check!(
        fails,
        neighbour_running(&mut dummy_sm),
        "[{=str}] the neighbour is still running at the end"
    );
    defmt::info!(
        "[INFO] not measured: any real ESC telemetry; the width of the runt pulses on the pad"
    );
    hil_common::exit(fails);
}
