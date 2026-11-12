//! On-board check of `pio_uart::OneWire`, in loop-back on the four spare pins GPIO44-47 (header J6
//! rows 11-14, nothing fitted), against a TEST PEER written for this program. No real device is
//! on the wires. Nothing here touches GPIO34 or GPIO36-39, the ADC, USB or flash.
//!
//! Who is where:
//! - PIO2 state machine 0: `OneWire` on GPIO44-47 (the code under test).
//! - PIO2 state machine 1: the peer's transmitter. It loads before `OneWire` and runs `OneWire`'s own
//!   transmit code (a `mov pc` into it), because a pad is driven by ONE PIO block, the one its
//!   function select names, so the peer must be in the block that drives the pad. Its front end
//!   waits for the first start bit of our message, counts cycles, then jumps into the transmit code:
//!   the reply starts at a chosen time after our stop bit, with no CPU in the path. (The CPU loads
//!   its x and y with forced instructions, so the front end is 3 instructions.)
//! - PIO1 state machines 0-3: four receivers, one per pin (the peer's ears, and the check that
//!   the unselected lines carry nothing). They run the same receive loop as `OneWire`.
//! - PIO0 state machine 0: the observer. It times the high and low runs of the selected pin to
//!   one PIO cycle (6.7 ns) and pushes the counts; the CPU rebuilds the waveform from them.
//!
//! A low level that no state machine could make (a runt pulse, a line still held low while the
//! receiver is re-armed) is made with the pad's output override in IO_BANK0, which beats every
//! PIO latch.
//!
//! The pads of one GPIO belong to the PIO block that made them last, so the pins are made in the
//! order PIO1, PIO0, PIO2: PIO2 drives.
#![no_std]
#![no_main]

use core::cell::Cell;
use core::future::Future;
use core::pin::pin;
use core::task::{Context, Poll, Waker};

use cortex_m::peripheral::DWT;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::join::join3;
use embassy_futures::select::{Either, select};
use embassy_rp::Peri;
use embassy_rp::gpio::{AnyPin, Input, Pull};
use embassy_rp::peripherals::{PIO0, PIO1, PIO2};
use embassy_rp::pio::program::{InstructionOperands, OutDestination, SetDestination, pio_asm};
use embassy_rp::pio::{
    self, Config as PioConfig, FifoJoin, Pin, Pio, ShiftConfig, ShiftDirection, StateMachine,
};
use embassy_rp::{bind_interrupts, pac};
use embassy_time::{Duration, Timer, with_timeout};
use embedded_io_async::{Read, Write};
use fixed::types::U24F8;
use panic_probe as _;
use pio_uart::timing::{self, RxWord};
use pio_uart::{Config, Error, OneWire};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    PIO1_IRQ_0 => pio::InterruptHandler<PIO1>;
    PIO2_IRQ_0 => pio::InterruptHandler<PIO2>;
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
const BAUD: u32 = 19_200;
/// Quarter PIO cycles in one bit: 150 MHz * 4 / 19 200.
const BIT_Q: u64 = (CLK_HZ as u64) * 4 / BAUD as u64;
/// The first spare GPIO and the number of lines.
const FIRST_GPIO: usize = 44;
const LINES: usize = 4;
/// GPIO44 is PIO pin 28 with GPIOBASE 16.
const OE_SHIFT: u32 = 28;
const OE_MASK: u32 = 0xF << OE_SHIFT;
const MAX_MSG: usize = 259;
const OBS_WORDS: usize = 4096;
/// Cycles added to the peer's delay so that its start bit never begins before our stop bit ends.
/// `OneWire` lets go of the line one cycle before the nominal end of its last stop bit.
const GUARD_CYCLES: i64 = 2;
/// Cycles from the peer front end seeing our first start bit to the peer's start bit, besides the
/// delay count: the loop runs x + 1 times, then mov pc, pull, out, mov isr, set pindirs, pull.
const PEER_OVERHEAD: i64 = 8;

/// Reply delays asked for, in microseconds, and the same in PIO cycles (16 per 52.08 us bit).
const DELAYS_US: [u32; 4] = [0, 26, 52, 1000];
const DELAYS_CYC: [i64; 4] = [0, 8, 16, 307];
const LENGTHS: [usize; 7] = [1, 2, 3, 4, 5, 21, 259];

fn ns_of_q(q: i64) -> i64 {
    q * 5 / 3
}

struct Rng(u32);
impl Rng {
    fn next(&mut self) -> u32 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.0 = x;
        x
    }
    fn fill(&mut self, buf: &mut [u8]) {
        for b in buf {
            *b = (self.next() >> 11) as u8;
        }
    }
}

const PULL_BLOCK: u16 = InstructionOperands::PULL {
    if_empty: false,
    block: true,
}
.encode();
const OUT_X_32: u16 = InstructionOperands::OUT {
    destination: OutDestination::X,
    bit_count: 32,
}
.encode();

const fn set(dest: SetDestination, data: u8) -> u16 {
    InstructionOperands::SET {
        destination: dest,
        data,
    }
    .encode()
}

fn shift_right() -> ShiftConfig {
    ShiftConfig {
        threshold: 32,
        direction: ShiftDirection::Right,
        auto_fill: false,
    }
}

/// `Config::set_jmp_pin` would store the absolute GPIO number; the field counts from GPIOBASE.
fn set_jmp_pin<PIO: pio::Instance>(cfg: &mut PioConfig<'_, PIO>, gpio: u8) {
    let mut exec = cfg.get_exec();
    exec.jmp_pin = gpio - 16;
    // SAFETY: only the JMP PIN source changes; the wrap settings are the ones `use_program` wrote.
    unsafe { cfg.set_exec(exec) };
}

// ---------------------------------------------------------------------------------------------
// The monitors (PIO1)
// ---------------------------------------------------------------------------------------------

struct Mons {
    sm: (
        StateMachine<'static, PIO1, 0>,
        StateMachine<'static, PIO1, 1>,
        StateMachine<'static, PIO1, 2>,
        StateMachine<'static, PIO1, 3>,
    ),
    cfg: [PioConfig<'static, PIO1>; LINES],
    origin: u8,
}

fn mon_reset<const S: usize>(
    sm: &mut StateMachine<'static, PIO1, S>,
    cfg: &PioConfig<'static, PIO1>,
    origin: u8,
) {
    sm.set_enable(false);
    sm.set_config(cfg);
    sm.restart();
    sm.clear_fifos();
    let _ = sm.rx().stalled();
    // SAFETY: the state machine is disabled; the jump only sets its program counter.
    unsafe { sm.exec_jmp(origin) };
    sm.set_enable(true);
}

/// Collects `want` records from one monitor. Returns the framing errors seen.
async fn mon_collect<const S: usize>(
    sm: &mut StateMachine<'static, PIO1, S>,
    out: &mut [u8],
    want: usize,
) -> u32 {
    let mut got = 0;
    let mut framing = 0;
    while got < want {
        let w = sm.rx().wait_pull().await;
        match timing::decode_rx_word(w) {
            RxWord::Byte(b) => {
                if let Some(slot) = out.get_mut(got) {
                    *slot = b;
                }
                got += 1;
            }
            RxWord::FramingError => {
                framing += 1;
                got += 1;
            }
            RxWord::TxDone | RxWord::Runt => {}
        }
    }
    framing
}

/// True when the receiver saw nothing: empty FIFO and no stall.
fn mon_quiet<const S: usize>(sm: &mut StateMachine<'static, PIO1, S>) -> bool {
    let empty = sm.rx().empty();
    let stalled = sm.rx().stalled();
    empty && !stalled
}

impl Mons {
    fn reset_all(&mut self) {
        let o = self.origin;
        mon_reset(&mut self.sm.0, &self.cfg[0], o);
        mon_reset(&mut self.sm.1, &self.cfg[1], o);
        mon_reset(&mut self.sm.2, &self.cfg[2], o);
        mon_reset(&mut self.sm.3, &self.cfg[3], o);
    }

    async fn collect(&mut self, line: usize, out: &mut [u8], want: usize) -> u32 {
        match line {
            0 => mon_collect(&mut self.sm.0, out, want).await,
            1 => mon_collect(&mut self.sm.1, out, want).await,
            2 => mon_collect(&mut self.sm.2, out, want).await,
            _ => mon_collect(&mut self.sm.3, out, want).await,
        }
    }

    /// Bit mask of the lines whose receiver saw anything.
    fn noisy(&mut self) -> u8 {
        let mut m = 0;
        if !mon_quiet(&mut self.sm.0) {
            m |= 1;
        }
        if !mon_quiet(&mut self.sm.1) {
            m |= 2;
        }
        if !mon_quiet(&mut self.sm.2) {
            m |= 4;
        }
        if !mon_quiet(&mut self.sm.3) {
            m |= 8;
        }
        m
    }
}

// ---------------------------------------------------------------------------------------------
// The peer's transmitter (PIO2, state machine 1)
// ---------------------------------------------------------------------------------------------

struct Peer {
    sm: StateMachine<'static, PIO2, 1>,
    cfg: [PioConfig<'static, PIO2>; LINES],
    front: u8,
    words: [u32; 72],
    len: usize,
    next: usize,
}

impl Peer {
    fn quiesce(&mut self, line: usize) {
        self.sm.set_enable(false);
        self.sm.set_config(&self.cfg[line]);
        self.sm.restart();
        self.sm.clear_fifos();
        // SAFETY: the state machine is disabled; these set the pin latch and direction of the line.
        unsafe {
            self.sm.exec_instr(set(SetDestination::PINDIRS, 0));
            self.sm.exec_instr(set(SetDestination::PINS, 1));
        }
    }

    fn load(&mut self, words: impl Iterator<Item = u32>) {
        self.len = 0;
        self.next = 0;
        for w in words {
            if let Some(slot) = self.words.get_mut(self.len) {
                *slot = w;
                self.len += 1;
            }
        }
    }

    fn push_initial(&mut self) {
        while self.next < self.len {
            let Some(&w) = self.words.get(self.next) else {
                break;
            };
            if !self.sm.tx().try_push(w) {
                break;
            }
            self.next += 1;
        }
    }

    async fn feed(&mut self) {
        while self.next < self.len {
            let Some(&w) = self.words.get(self.next) else {
                break;
            };
            self.sm.tx().wait_push(w).await;
            self.next += 1;
        }
    }

    /// The reply `msg` goes out `x + 1 + overhead` cycles after the first start bit of the
    /// message reaches the front end. x is loaded by forced `pull` and `out x, 32`, y by a forced
    /// `set y`, all with the state machine disabled.
    fn arm_delayed(&mut self, line: usize, x: u32, tx_entry: u8, msg: &[u8]) {
        self.quiesce(line);
        self.sm.tx().push(x);
        // SAFETY: the state machine is disabled; the word just pushed goes to x, the transmit
        // entry (at most 31) to y, and the jump sets the program counter to the front end.
        unsafe {
            self.sm.exec_instr(PULL_BLOCK);
            cortex_m::asm::delay(2_000);
            self.sm.exec_instr(OUT_X_32);
            cortex_m::asm::delay(2_000);
            self.sm.exec_instr(
                InstructionOperands::SET {
                    destination: SetDestination::Y,
                    data: tx_entry,
                }
                .encode(),
            );
            self.sm.exec_jmp(self.front);
        }
        self.load(timing::fifo_image(msg).into_iter().flatten());
        self.push_initial();
        self.sm.set_enable(true);
    }

    /// Loads `msg` for an immediate send into the transmit code; `go` starts it.
    fn prepare_now(&mut self, line: usize, tx_entry: u8, msg: &[u8]) {
        self.quiesce(line);
        self.load(timing::fifo_image(msg).into_iter().flatten());
        self.push_initial();
        // SAFETY: the state machine is disabled; the jump sets its program counter.
        unsafe { self.sm.exec_jmp(tx_entry) };
    }

    fn go(&mut self) {
        self.sm.set_enable(true);
    }

    /// Sends `msg` at once, straight into the transmit code.
    fn arm_now(&mut self, line: usize, tx_entry: u8, msg: &[u8]) {
        self.prepare_now(line, tx_entry, msg);
        self.go();
    }

    /// Drives the line low push-pull, until `release_low`.
    fn drive_low(&mut self, line: usize) {
        self.quiesce(line);
        // SAFETY: the state machine is disabled; latch low, then drive.
        unsafe {
            self.sm.exec_instr(set(SetDestination::PINS, 0));
            self.sm.exec_instr(set(SetDestination::PINDIRS, 1));
        }
    }

    fn release_low(&mut self) {
        // SAFETY: the state machine is disabled; release, latch high.
        unsafe {
            self.sm.exec_instr(set(SetDestination::PINDIRS, 0));
            self.sm.exec_instr(set(SetDestination::PINS, 1));
        }
    }

    fn stop(&mut self) {
        self.sm.set_enable(false);
        // SAFETY: the state machine is disabled; release, latch high.
        unsafe {
            self.sm.exec_instr(set(SetDestination::PINDIRS, 0));
            self.sm.exec_instr(set(SetDestination::PINS, 1));
        }
    }
}

// ---------------------------------------------------------------------------------------------
// The observer (PIO0, state machine 0) and the waveform analysis
// ---------------------------------------------------------------------------------------------

struct Observer {
    sm: StateMachine<'static, PIO0, 0>,
    cfg: [PioConfig<'static, PIO0>; LINES],
    origin: u8,
}

impl Observer {
    /// Starts watching `line`, which must be idle high.
    fn start(&mut self, line: usize) {
        self.sm.set_enable(false);
        self.sm.set_config(&self.cfg[line]);
        self.sm.restart();
        self.sm.clear_fifos();
        let _ = self.sm.rx().stalled();
        // SAFETY: the state machine is disabled; the jump only sets its program counter.
        unsafe { self.sm.exec_jmp(self.origin) };
        self.sm.set_enable(true);
    }

    fn stop(&mut self) {
        self.sm.set_enable(false);
    }

    /// Takes words until `done` is set and the line has been quiet for 3 ms. Returns the count.
    async fn drain(&mut self, buf: &mut [u32], done: &Cell<bool>) -> usize {
        let mut n = 0;
        loop {
            while let Some(w) = self.sm.rx().try_pull() {
                if let Some(slot) = buf.get_mut(n) {
                    *slot = w;
                    n += 1;
                }
            }
            match select(self.sm.rx().wait_pull(), Timer::after_millis(3)).await {
                Either::First(w) => {
                    if let Some(slot) = buf.get_mut(n) {
                        *slot = w;
                        n += 1;
                    }
                }
                Either::Second(()) => {
                    if done.get() && self.sm.rx().empty() {
                        break;
                    }
                }
            }
        }
        n
    }
}

/// Edges rebuilt from the observer's words: time in quarter cycles from the first rising edge, and
/// the level after the edge.
struct Edges {
    t: [i64; OBS_WORDS],
    level: [bool; OBS_WORDS],
    n: usize,
}

impl Edges {
    fn from_words(words: &[u32]) -> Self {
        let mut e = Edges {
            t: [0; OBS_WORDS],
            level: [false; OBS_WORDS],
            n: 0,
        };
        let mut now: i64 = 0;
        for (i, &w) in words.iter().enumerate().skip(1) {
            // odd words are high runs (ending in a falling edge), even words low runs
            let run = w as i64;
            let high_run = i % 2 == 1;
            now += 4 * if high_run { 4 + 2 * run } else { 6 + 2 * run };
            if e.n < OBS_WORDS {
                e.t[e.n] = now;
                e.level[e.n] = !high_run;
                e.n += 1;
            }
        }
        e
    }

    fn level_at(&self, t: i64) -> bool {
        let times = self.t.get(..self.n).unwrap_or(&[]);
        let idx = times.partition_point(|&x| x <= t);
        if idx == 0 {
            true
        } else {
            self.level.get(idx - 1).copied().unwrap_or(true)
        }
    }

    /// The first falling edge at or after `t`.
    fn fall_from(&self, t: i64) -> Option<i64> {
        let times = self.t.get(..self.n)?;
        let start = times.partition_point(|&x| x < t);
        (start..self.n).find_map(|i| {
            if !self.level.get(i).copied().unwrap_or(true) {
                self.t.get(i).copied()
            } else {
                None
            }
        })
    }
}

struct Wave {
    bytes_ok: usize,
    bytes_bad: usize,
    start_bad: usize,
    stop_bad: usize,
    /// Measured mean bit time in quarter cycles * 1000 / ... see `bit_ppm`.
    bit_ppm: i64,
    bit_ns: i64,
    gap_max_q: i64,
    gap_min_q: i64,
    shortest_run_q: i64,
    first_start: i64,
    last_start: i64,
    edges: usize,
}

/// Decodes `msg` out of the edges starting at the first falling edge.
fn analyze(e: &Edges, msg: &[u8]) -> Wave {
    let mut w = Wave {
        bytes_ok: 0,
        bytes_bad: 0,
        start_bad: 0,
        stop_bad: 0,
        bit_ppm: 0,
        bit_ns: 0,
        gap_max_q: i64::MIN,
        gap_min_q: i64::MAX,
        shortest_run_q: i64::MAX,
        first_start: 0,
        last_start: 0,
        edges: e.n,
    };
    for i in 1..e.n {
        let run = e.t[i] - e.t[i - 1];
        if run < w.shortest_run_q {
            w.shortest_run_q = run;
        }
    }
    let Some(mut s) = e.fall_from(i64::MIN) else {
        w.bytes_bad = msg.len();
        return w;
    };
    w.first_start = s;
    let mut prev: Option<i64> = None;
    for (i, &byte) in msg.iter().enumerate() {
        if i > 0 {
            let Some(ns) = e.fall_from(s + BIT_Q as i64 * 19 / 2) else {
                w.bytes_bad += msg.len() - i;
                break;
            };
            s = ns;
        }
        if let Some(p) = prev {
            let gap = s - p - 10 * BIT_Q as i64;
            w.gap_max_q = w.gap_max_q.max(gap);
            w.gap_min_q = w.gap_min_q.min(gap);
        }
        prev = Some(s);
        w.last_start = s;
        let mut ok = true;
        if e.level_at(s + BIT_Q as i64 / 2) {
            w.start_bad += 1;
            ok = false;
        }
        for k in 0..8 {
            let at = s + BIT_Q as i64 * (3 + 2 * k) / 2;
            if e.level_at(at) != (byte >> k & 1 == 1) {
                ok = false;
            }
        }
        if !e.level_at(s + BIT_Q as i64 * 19 / 2) {
            w.stop_bad += 1;
            ok = false;
        }
        if ok {
            w.bytes_ok += 1;
        } else {
            w.bytes_bad += 1;
        }
    }
    if msg.len() > 1 {
        let pitch = (w.last_start - w.first_start) / (msg.len() as i64 - 1);
        w.bit_ppm = (pitch - 10 * BIT_Q as i64) * 1_000_000 / (10 * BIT_Q as i64);
        w.bit_ns = ns_of_q(pitch) / 10;
    }
    w
}

// ---------------------------------------------------------------------------------------------
// Pads and the line state
// ---------------------------------------------------------------------------------------------

fn pad_levels() -> u8 {
    ((pac::SIO.gpio_in(1).read() >> (FIRST_GPIO as u32 - 32)) & 0xF) as u8
}

fn oe_bits() -> u8 {
    ((pac::PIO2.dbg_padoe().read() & OE_MASK) >> OE_SHIFT) as u8
}

/// Forces the pad of `line` low (output enabled, level low) whatever the PIO latches say, or
/// gives it back to its function.
fn force_low(line: usize, on: bool) {
    pac::IO_BANK0.gpio(FIRST_GPIO + line).ctrl().modify(|w| {
        if on {
            w.set_oeover(pac::io::vals::Oeover::ENABLE);
            w.set_outover(pac::io::vals::Outover::LOW);
        } else {
            w.set_oeover(pac::io::vals::Oeover::NORMAL);
            w.set_outover(pac::io::vals::Outover::NORMAL);
        }
    });
}

/// Waits `cycles` CPU cycles by the cycle counter (`asm::delay` takes about 2.7 cycles a count).
fn spin(cycles: u32) {
    let t0 = DWT::cycle_count();
    while DWT::cycle_count().wrapping_sub(t0) < cycles {}
}

/// A low pulse of about `cycles` CPU cycles on `line`, interrupts off.
fn low_pulse(line: usize, cycles: u32) {
    cortex_m::interrupt::free(|_| {
        force_low(line, true);
        spin(cycles);
        force_low(line, false);
    });
}

fn pullups() -> u8 {
    let mut m = 0;
    for l in 0..LINES {
        if pac::PADS_BANK0.gpio(FIRST_GPIO + l).read().pue() {
            m |= 1 << l;
        }
    }
    m
}

fn pulldowns() -> u8 {
    let mut m = 0;
    for l in 0..LINES {
        if pac::PADS_BANK0.gpio(FIRST_GPIO + l).read().pde() {
            m |= 1 << l;
        }
    }
    m
}

/// All four lines are inputs (no output enable from PIO2), pad pull-up on, no pull-down, high.
fn idle_ok() -> bool {
    oe_bits() == 0 && pullups() == 0xF && pulldowns() == 0 && pad_levels() == 0xF
}

// ---------------------------------------------------------------------------------------------
// The rig
// ---------------------------------------------------------------------------------------------

type Ow = OneWire<'static, PIO2, 0, 4>;

struct Rig {
    mons: Mons,
    peer: Peer,
    obs: Observer,
    rng: Rng,
    guard: i64,
}

impl Rig {
    /// One exchange: we write `tx`, the peer answers `rx` `delay_cycles` PIO cycles after our
    /// stop bit, we read it. Returns a description of what went wrong.
    async fn exchange(
        &mut self,
        ow: &mut Ow,
        line: usize,
        tx: &[u8],
        rx: &[u8],
        delay_cycles: i64,
    ) -> Result<(), &'static str> {
        ow.select(line).map_err(|_| "select")?;
        self.mons.reset_all();
        let x = (160 * tx.len() as i64 + delay_cycles + self.guard - PEER_OVERHEAD).max(0) as u32;
        self.peer.arm_delayed(line, x, ow.tx_entry(), rx);
        let mut seen = [0u8; 2 * MAX_MSG];
        let mut reply = [0u8; MAX_MSG];
        let want = tx.len() + rx.len();
        let budget = Duration::from_millis(300 + want as u64);
        let mons = &mut self.mons;
        let peer = &mut self.peer;
        let r = with_timeout(
            budget,
            join3(
                async {
                    ow.write(tx).await?;
                    match reply.get_mut(..rx.len()) {
                        Some(buf) => ow.read_exact(buf).await.map_err(|_| Error::Overrun),
                        None => Ok(()),
                    }
                },
                peer.feed(),
                mons.collect(line, &mut seen, want),
            ),
        )
        .await;
        self.peer.stop();
        let (io, (), framing) = r.map_err(|_| "timeout")?;
        io.map_err(|_| "io error")?;
        if framing != 0 {
            return Err("monitor framing error");
        }
        if seen.get(..tx.len()) != Some(tx) {
            return Err("peer received other bytes");
        }
        if seen.get(tx.len()..want) != Some(rx) {
            return Err("monitor saw other reply bytes");
        }
        if reply.get(..rx.len()) != Some(rx) {
            return Err("OneWire read other bytes");
        }
        let quiet = self.mons.noisy() & !(1 << line);
        if quiet != 0 {
            return Err("another line carried traffic");
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------------------------
// The checks
// ---------------------------------------------------------------------------------------------

/// Check 1.
async fn check_pins_free(p: &mut embassy_rp::Peripherals) -> bool {
    let mut ok = true;
    // SAFETY: the pins are borrowed for a moment, here, before anything else owns them.
    let mut pins: [Peri<'static, AnyPin>; LINES] = unsafe {
        [
            p.PIN_44.clone_unchecked().into(),
            p.PIN_45.clone_unchecked().into(),
            p.PIN_46.clone_unchecked().into(),
            p.PIN_47.clone_unchecked().into(),
        ]
    };
    for (i, pin) in pins.iter_mut().enumerate() {
        let up = {
            let input = Input::new(pin.reborrow(), Pull::Up);
            Timer::after_micros(500).await;
            input.is_high()
        };
        let down = {
            let input = Input::new(pin.reborrow(), Pull::Down);
            Timer::after_micros(500).await;
            input.is_low()
        };
        let up2 = {
            let input = Input::new(pin.reborrow(), Pull::Up);
            Timer::after_micros(500).await;
            input.is_high()
        };
        let pass = up && down && up2;
        defmt::info!(
            "[{=str}] GPIO{=usize} follows its pad pull: up {=bool}, down low {=bool}, up again {=bool}",
            if pass { "PASS" } else { "FAIL" },
            FIRST_GPIO + i,
            up,
            down,
            up2
        );
        ok &= pass;
    }
    ok
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    defmt::info!("[INFO] one_wire_loopback: start");
    let mut p = embassy_rp::init(Default::default());
    let mut core = cortex_m::Peripherals::take().unwrap();
    core.DCB.enable_trace();
    core.DWT.enable_cycle_counter();
    let hz = embassy_rp::clocks::clk_sys_freq();
    let mut fails = 0u32;
    check!(fails, hz == CLK_HZ, "[{=str}] clk_sys is 150 MHz");

    // 1. The spare pins are free.
    let free = check_pins_free(&mut p).await;
    check!(
        fails,
        free,
        "[{=str}] check 1: GPIO44-47 follow their pad pull-up and pull-down, nothing drives them"
    );
    if !free {
        defmt::error!(
            "[FAIL] a spare pin is driven from outside: stopping before anything is driven"
        );
        hil_common::exit(fails);
    }

    let div = timing::divider(hz, BAUD).unwrap();
    let div_fixed = U24F8::from_bits(div.q8);
    defmt::info!(
        "[INFO] divider {=u32} + {=u8}/256, error {=i32} ppm, bit time {=u32} ns",
        div.integer(),
        div.fraction(),
        div.error_ppm,
        timing::bit_time_ns(BAUD).unwrap_or(0)
    );

    // Pins and blocks. GPIO44-47 are made by PIO1 first, then PIO0, and by PIO2 last.
    // SAFETY (all `clone_unchecked` below): one test program shares the four pins between three
    // PIO blocks on purpose, as inputs for PIO0 and PIO1 and as the driver for PIO2.
    macro_rules! four_pins {
        ($common:expr) => {
            unsafe {
                [
                    $common.make_pio_pin(p.PIN_44.clone_unchecked()),
                    $common.make_pio_pin(p.PIN_45.clone_unchecked()),
                    $common.make_pio_pin(p.PIN_46.clone_unchecked()),
                    $common.make_pio_pin(p.PIN_47.clone_unchecked()),
                ]
            }
        };
    }
    let Pio {
        common: mut c1,
        sm0: m0,
        sm1: m1,
        sm2: m2,
        sm3: m3,
        ..
    } = Pio::new(p.PIO1, Irqs);
    let pins1: [Pin<'static, PIO1>; 4] = four_pins!(c1);
    let Pio {
        common: mut c0,
        sm0: o0,
        ..
    } = Pio::new(p.PIO0, Irqs);
    let pins0: [Pin<'static, PIO0>; 4] = four_pins!(c0);
    let Pio {
        common: mut c2,
        sm0: ow_sm,
        sm1: peer_sm,
        ..
    } = Pio::new(p.PIO2, Irqs);
    let pins2: [Pin<'static, PIO2>; 4] = four_pins!(c2);

    // Monitors: the receive loop of OneWire.
    let mon_prg = pio_asm!(
        ".wrap_target",
        "    wait 0 pin 0",
        "    mov isr, null [20]",
        "    set x, 7",
        "bit:",
        "    in pins, 1",
        "    jmp x-- bit [14]",
        "    in pins, 1",
        "    push noblock",
        "    wait 1 pin 0",
        ".wrap",
    );
    let mon_loaded = c1.load_program(&mon_prg.program);
    let mon_cfg: [PioConfig<'static, PIO1>; LINES] = core::array::from_fn(|l| {
        let mut c = PioConfig::default();
        c.use_program(&mon_loaded, &[]);
        c.clock_divider = div_fixed;
        c.shift_in = shift_right();
        c.shift_out = shift_right();
        c.set_in_pins(&[&pins1[l]]);
        c
    });
    let mons = Mons {
        sm: (m0, m1, m2, m3),
        cfg: mon_cfg,
        origin: mon_loaded.origin,
    };

    // Observer.
    let obs_prg = pio_asm!(
        "lo_start:",
        "    mov x, ~null",
        "lo_loop:",
        "    jmp pin hi_push",
        "    jmp x-- lo_loop",
        "    jmp lo_start",
        "hi_push:",
        "    mov isr, ~x",
        "    push block",
        "    mov x, ~null",
        "hi_loop:",
        "    jmp pin hi_dec",
        "    jmp lo_push",
        "hi_dec:",
        "    jmp x-- hi_loop",
        "    jmp hi_loop",
        "lo_push:",
        "    mov isr, ~x",
        "    push block",
        "    jmp lo_start",
    );
    let obs_loaded = c0.load_program(&obs_prg.program);
    let obs_cfg: [PioConfig<'static, PIO0>; LINES] = core::array::from_fn(|l| {
        let mut c = PioConfig::default();
        c.use_program(&obs_loaded, &[]);
        c.fifo_join = FifoJoin::RxOnly;
        c.shift_in = shift_right();
        // The in pins are what puts PIO0 in the GPIO16-47 window; the program never uses `in`.
        c.set_in_pins(&[&pins0[l]]);
        set_jmp_pin(&mut c, (FIRST_GPIO + l) as u8);
        c
    });
    let obs = Observer {
        sm: o0,
        cfg: obs_cfg,
        origin: obs_loaded.origin,
    };

    // Peer front end, loaded before OneWire so that OneWire takes what is left.
    let peer_prg = pio_asm!(
        "front:",
        "    wait 0 pin 0",
        "dl:",
        "    jmp x-- dl",
        "    mov pc, y",
    );
    let peer_loaded = c2.load_program(&peer_prg.program);
    let peer_cfg: [PioConfig<'static, PIO2>; LINES] = core::array::from_fn(|l| {
        let mut c = PioConfig::default();
        c.use_program(&peer_loaded, &[]);
        c.clock_divider = div_fixed;
        c.shift_in = shift_right();
        c.shift_out = shift_right();
        c.set_in_pins(&[&pins2[l]]);
        c.set_out_pins(&[&pins2[l]]);
        c.set_set_pins(&[&pins2[l]]);
        c
    });
    let peer = Peer {
        sm: peer_sm,
        cfg: peer_cfg,
        front: peer_loaded.origin,
        words: [0; 72],
        len: 0,
        next: 0,
    };

    let config = Config::new().with_baud(BAUD).with_clk_sys_hz(hz);
    let mut ow: Ow = match OneWire::new(c2, ow_sm, pins2, config) {
        Ok(ow) => ow,
        Err(e) => {
            defmt::error!("[FAIL] OneWire::new failed: {}", e.reason);
            hil_common::exit(1);
        }
    };
    defmt::info!(
        "[INFO] PIO2: peer front end {=u8} instructions at {=u8}, OneWire program at {=u8} (tx entry); {=u8} + 27 of 32 slots",
        peer_prg.program.code.len() as u8,
        peer_loaded.origin,
        ow.tx_entry(),
        peer_prg.program.code.len() as u8
    );
    check!(
        fails,
        idle_ok(),
        "[{=str}] after new: every pin an input, pad pull-up on, no output enable, line high (oe {=u8}, pull-ups {=u8})",
        oe_bits(),
        pullups()
    );

    let mut rig = Rig {
        mons,
        peer,
        obs,
        rng: Rng(0x1357_9BDF),
        guard: GUARD_CYCLES,
    };
    let mut tx = [0u8; MAX_MSG];
    let mut rx = [0u8; MAX_MSG];

    // ---- Check 2: waveform of a write ----
    let mut words = [0u32; OBS_WORDS];
    for line in 0..LINES {
        let n = MAX_MSG;
        rig.rng.fill(&mut tx[..n]);
        let _ = ow.select(line);
        rig.obs.start(line);
        let done = Cell::new(false);
        let mut got = 0;
        let r = with_timeout(
            Duration::from_millis(1000),
            embassy_futures::join::join(
                async {
                    let r = ow.write(&tx[..n]).await;
                    done.set(true);
                    r
                },
                rig.obs.drain(&mut words, &done),
            ),
        )
        .await;
        rig.obs.stop();
        let wr = match r {
            Ok((wr, cnt)) => {
                got = cnt;
                wr
            }
            Err(_) => Err(Error::Underrun),
        };
        let edges = Edges::from_words(&words[..got]);
        let w = analyze(&edges, &tx[..n]);
        let limit = timing::bit_time_ns(BAUD).unwrap_or(0) as i64;
        let ns = ns_of_q(BIT_Q as i64);
        defmt::info!(
            "[STAT] waveform GPIO{=usize}: {=usize} bytes, {=usize} edges, bit time {=i64} ns (nominal {=i64} ns, {=i64} ppm), shortest run {=i64} ns",
            FIRST_GPIO + line,
            n,
            w.edges,
            w.bit_ns,
            ns,
            w.bit_ppm,
            ns_of_q(w.shortest_run_q)
        );
        defmt::info!(
            "[STAT] gap between a stop bit and the next start bit, GPIO{=usize}: longest {=i64} ns, shortest {=i64} ns, one bit is {=i64} ns",
            FIRST_GPIO + line,
            ns_of_q(w.gap_max_q),
            ns_of_q(w.gap_min_q),
            limit
        );
        check!(
            fails,
            wr == Ok(n),
            "[{=str}] check 2: write of 259 bytes on GPIO{=usize} returned Ok(259)",
            FIRST_GPIO + line
        );
        check!(
            fails,
            w.bytes_ok == n && w.start_bad == 0 && w.stop_bad == 0,
            "[{=str}] check 2: GPIO{=usize}: start bit, 8 data bits LSB first, stop bit right in {=usize} of 259 bytes (bad starts {=usize}, bad stops {=usize})",
            FIRST_GPIO + line,
            w.bytes_ok,
            w.start_bad,
            w.stop_bad
        );
        check!(
            fails,
            w.bit_ns > 0 && (w.bit_ns - limit).abs() * 100 < limit,
            "[{=str}] check 2: GPIO{=usize}: bit time {=i64} ns within 1 % of {=i64} ns",
            FIRST_GPIO + line,
            w.bit_ns,
            limit
        );
        check!(
            fails,
            ns_of_q(w.gap_max_q) < limit,
            "[{=str}] check 2: GPIO{=usize}: longest gap between a stop bit and the next start bit {=i64} ns is below one bit time",
            FIRST_GPIO + line,
            ns_of_q(w.gap_max_q)
        );
        check!(
            fails,
            idle_ok(),
            "[{=str}] after the write on GPIO{=usize}: every pin an input with pull-up",
            FIRST_GPIO + line
        );
    }
    ow.select(0).unwrap();

    // ---- Check 3: the line is driven only during a write ----
    for line in 0..LINES {
        let n = 21;
        rig.rng.fill(&mut tx[..n]);
        let _ = ow.select(line);
        // a first run warms the caches; its numbers are dropped
        let _ = oe_trace(&mut ow, &tx[..n]).await;
        let r = oe_trace(&mut ow, &tx[..n]).await;
        defmt::info!(
            "[STAT] output enable GPIO{=usize}: on {=i64} CPU cycles before the first start bit is seen, off {=i64} CPU cycles after the nominal end of the last stop bit (a bit is 7812 CPU cycles), other lines' output enable seen: 0x{=u8:x}, write result ok {=bool}",
            FIRST_GPIO + line,
            r.first_fall - r.oe_on,
            r.oe_off - r.stop_end,
            r.other_oe,
            r.ok
        );
        check!(
            fails,
            r.ok,
            "[{=str}] check 3: GPIO{=usize}: write completed",
            FIRST_GPIO + line
        );
        check!(
            fails,
            r.oe_on_seen && r.oe_on <= r.first_fall + 40,
            "[{=str}] check 3: GPIO{=usize}: output enable on from the first start bit",
            FIRST_GPIO + line
        );
        check!(
            fails,
            r.oe_gaps == 0,
            "[{=str}] check 3: GPIO{=usize}: output enable stayed on for the whole message ({=u32} samples with it off)",
            FIRST_GPIO + line,
            r.oe_gaps
        );
        check!(
            fails,
            r.oe_off_seen && (r.oe_off - r.stop_end).abs() <= 7812 / 4,
            "[{=str}] check 3: GPIO{=usize}: output enable off within a quarter bit (13 us) of the end of the last stop bit, and not later than that ({=i64} CPU cycles from it)",
            FIRST_GPIO + line,
            r.oe_off - r.stop_end
        );
        check!(
            fails,
            r.other_oe == 0,
            "[{=str}] check 3: GPIO{=usize}: the other three lines were never driven",
            FIRST_GPIO + line
        );
    }
    ow.select(0).unwrap();

    // ---- Peer timing: what the delays really are (observer) ----
    // The peer's start bit cannot come sooner than about 1.3 PIO cycles (4 us) after our stop bit:
    // both state machines are in PIO2, which has ONE output-enable latch per pin, so the peer's
    // `set pindirs, 1` (two cycles before its start bit) must come after our release, or our
    // release would undo it. The guard of 2 cycles keeps a cycle of margin for the phase between
    // the two state machines' clocks.
    let cases: [(usize, i64); 4] = [
        (0, GUARD_CYCLES),
        (1, GUARD_CYCLES),
        (2, GUARD_CYCLES),
        (3, GUARD_CYCLES),
    ];
    for (ci, &(di, guard)) in cases.iter().enumerate() {
        let dc = DELAYS_CYC[di];
        let n = 5;
        rig.rng.fill(&mut tx[..n]);
        rig.rng.fill(&mut rx[..5]);
        let line = 1;
        let _ = ow.select(line);
        rig.mons.reset_all();
        let x = (160 * n as i64 + dc + guard - PEER_OVERHEAD).max(0) as u32;
        rig.peer.arm_delayed(line, x, ow.tx_entry(), &rx[..5]);
        rig.obs.start(line);
        let done = Cell::new(false);
        let mut reply = [0u8; 5];
        let r = with_timeout(
            Duration::from_millis(500),
            join3(
                async {
                    ow.write(&tx[..n]).await?;
                    let r = ow.read_exact(&mut reply).await.map_err(|_| Error::Overrun);
                    done.set(true);
                    r
                },
                rig.peer.feed(),
                rig.obs.drain(&mut words, &done),
            ),
        )
        .await;
        rig.peer.stop();
        rig.obs.stop();
        if let Ok((io, (), cnt)) = r {
            let edges = Edges::from_words(&words[..cnt]);
            // our message, then the reply, as one waveform: decode the reply from the end of ours
            let w = analyze(&edges, &tx[..n]);
            let ours_end = w.last_start + 10 * BIT_Q as i64;
            let reply_start = edges.fall_from(ours_end - BIT_Q as i64 / 2);
            let rw_ok = reply_start.map(|s| {
                let mut ok = true;
                for (i, &b) in rx[..5].iter().enumerate() {
                    let sb = s + 10 * BIT_Q as i64 * i as i64;
                    for k in 0..8 {
                        if edges.level_at(sb + BIT_Q as i64 * (3 + 2 * k) / 2) != (b >> k & 1 == 1)
                        {
                            ok = false;
                        }
                    }
                }
                ok
            });
            let g = reply_start
                .map(|s| ns_of_q(s - ours_end))
                .unwrap_or(i64::MIN);
            defmt::info!(
                "[STAT] peer reply delay asked {=u32} us ({=i64} cycles, guard {=i64}): from the end of our last stop bit to the peer's start bit {=i64} ns; reply on the wire right {=bool}; our read right {=bool}",
                DELAYS_US[di],
                dc,
                guard,
                g,
                rw_ok.unwrap_or(false),
                io.is_ok() && reply == rx[..5]
            );
            // one PIO cycle at this divider is 3255 ns
            let target = dc * 3255;
            let low = if guard < 0 { -3000 } else { 0 };
            check!(
                fails,
                g >= low
                    && g >= target + low
                    && g <= target + 20_000
                    && rw_ok == Some(true)
                    && io.is_ok()
                    && reply == rx[..5],
                "[{=str}] peer timing {=usize}: reply starts {=i64} ns after our stop bit for a {=u32} us delay (not before it, at most 20 us over); on the wire and read back right",
                ci,
                g,
                DELAYS_US[di]
            );
        } else {
            let st = ow.stats();
            defmt::info!(
                "[INFO] timed out: sent {=u32}, received {=u32}, framing {=u32}, overruns {=u32}, underruns {=u32}, discarded {=u32}; PIO2 SM0 pc {=u8}, SM1 pc {=u8}",
                st.bytes_sent,
                st.bytes_received,
                st.framing_errors,
                st.overruns,
                st.underruns,
                st.bytes_discarded,
                pac::PIO2.sm(0).addr().read().addr(),
                pac::PIO2.sm(1).addr().read().addr()
            );
            check!(
                fails,
                false,
                "[{=str}] peer timing {=usize}: exchange timed out",
                ci
            );
        }
    }
    ow.select(0).unwrap();

    // ---- Check 4 and 5: echo on every line, every length, every delay; unselected lines quiet ----
    let mut exchanges = 0u32;
    let mut bad = 0u32;
    let mut sent = 0u32;
    let mut received = 0u32;
    let before = ow.stats();
    for line in 0..LINES {
        for (di, &dc) in DELAYS_CYC.iter().enumerate() {
            for &n in &LENGTHS {
                let m = n;
                rig.rng.fill(&mut tx[..n]);
                rig.rng.fill(&mut rx[..m]);
                exchanges += 1;
                match rig.exchange(&mut ow, line, &tx[..n], &rx[..m], dc).await {
                    Ok(()) => {
                        sent += n as u32;
                        received += m as u32;
                    }
                    Err(why) => {
                        bad += 1;
                        if bad <= 8 {
                            defmt::error!(
                                "[INFO] echo failed: line {=usize}, length {=usize}, delay {=u32} us: {=str}",
                                line,
                                n,
                                DELAYS_US[di],
                                why
                            );
                        }
                    }
                }
            }
        }
    }
    let after = ow.stats();
    defmt::info!(
        "[STAT] echo: {=u32} exchanges, bytes sent {=u32}, received {=u32}; stats: sent +{=u32}, received +{=u32}, framing +{=u32}, overruns +{=u32}, underruns +{=u32}",
        exchanges,
        sent,
        received,
        after.bytes_sent - before.bytes_sent,
        after.bytes_received - before.bytes_received,
        after.framing_errors - before.framing_errors,
        after.overruns - before.overruns,
        after.underruns - before.underruns
    );
    check!(
        fails,
        bad == 0,
        "[{=str}] check 4: echo on 4 lines x 4 delays x 7 lengths ({=u32} exchanges): {=u32} failed",
        exchanges,
        bad
    );
    check!(
        fails,
        after.bytes_sent - before.bytes_sent == sent
            && after.bytes_received - before.bytes_received == received
            && after.framing_errors == before.framing_errors
            && after.overruns == before.overruns
            && after.underruns == before.underruns,
        "[{=str}] check 4: OneWire's own counters agree: bytes sent, received; no framing error, overrun or underrun"
    );
    check!(
        fails,
        bad == 0,
        "[{=str}] check 5: in every exchange only the selected line carried traffic (the other three receivers saw no start bit)"
    );
    check!(
        fails,
        idle_ok(),
        "[{=str}] after the echo run: every pin an input with pull-up"
    );
    {
        // positive control for the receivers: traffic on line 1 while line 0 is selected shows up
        // on receiver 1 and on no other
        let _ = ow.select(0);
        rig.mons.reset_all();
        rig.peer.arm_now(1, ow.tx_entry(), &[1, 2, 3]);
        rig.peer.feed().await;
        Timer::after_millis(5).await;
        rig.peer.stop();
        let mask = rig.mons.noisy();
        check!(
            fails,
            mask == 0b0010,
            "[{=str}] check 5: positive control, three bytes sent on line 1 are seen by receiver 1 and by no other (mask 0b{=u8:b})",
            mask
        );
    }

    // ---- Check 6: positive controls for the two error counters ----
    {
        let _ = ow.select(2);
        rig.mons.reset_all();
        let s0 = ow.stats();
        // a low stop bit: the line low for 12 bit times, then a good byte 0xA5
        let mut buf = [0u8; 8];
        rig.peer.drive_low(2);
        let low = async {
            Timer::after_micros(625).await;
            rig.peer.release_low();
            Timer::after_micros(300).await;
        };
        let (a, ()) = {
            let rd = with_timeout(Duration::from_millis(3), ow.read(&mut buf));
            let r = embassy_futures::join::join(rd, low).await;
            (r.0, r.1)
        };
        let first_timeout = a.is_err();
        let s1 = ow.stats();
        check!(
            fails,
            first_timeout
                && s1.framing_errors == s0.framing_errors + 1
                && s1.bytes_received == s0.bytes_received,
            "[{=str}] check 6: the line held low for 12 bit times: no byte delivered, framing errors {=u32} -> {=u32}",
            s0.framing_errors,
            s1.framing_errors
        );
        rig.peer.arm_now(2, ow.tx_entry(), &[0xA5]);
        let r = with_timeout(Duration::from_millis(50), ow.read(&mut buf)).await;
        rig.peer.stop();
        let s2 = ow.stats();
        check!(
            fails,
            matches!(r, Ok(Ok(1))) && buf[0] == 0xA5 && s2.framing_errors == s1.framing_errors,
            "[{=str}] check 6: the next good byte 0xA5 arrives alone, framing errors still {=u32}",
            s2.framing_errors
        );
        // overrun: 16 bytes while nothing reads
        let mut burst = [0u8; 16];
        rig.rng.fill(&mut burst);
        rig.peer.arm_now(2, ow.tx_entry(), &burst);
        rig.peer.feed().await;
        Timer::after_millis(20).await;
        rig.peer.stop();
        let s3 = ow.stats();
        let r1 = with_timeout(Duration::from_millis(50), ow.read(&mut buf)).await;
        let s4 = ow.stats();
        check!(
            fails,
            matches!(r1, Ok(Err(Error::Overrun))) && s4.overruns == s3.overruns + 1,
            "[{=str}] check 6: 16 bytes with nothing reading: the next read reports the overrun, overruns {=u32} -> {=u32}",
            s3.overruns,
            s4.overruns
        );
        let r2 = with_timeout(Duration::from_millis(5), ow.read(&mut buf)).await;
        check!(
            fails,
            r2.is_err(),
            "[{=str}] check 6: after the overrun the FIFO is empty and the error is cleared (the next read just waits)"
        );
        rig.rng.fill(&mut tx[..21]);
        rig.rng.fill(&mut rx[..21]);
        let e = rig.exchange(&mut ow, 2, &tx[..21], &rx[..21], 0).await;
        check!(
            fails,
            e.is_ok(),
            "[{=str}] check 6: a fresh echo of 21 bytes passes after the overrun"
        );
        ow.select(0).unwrap();
    }

    // ---- Check 7: break ----
    {
        let _ = ow.select(1);
        let ok1 = ow.break_start().is_ok();
        Timer::after_micros(500).await;
        let lv = pad_levels();
        let oe = oe_bits();
        check!(
            fails,
            ok1 && lv == 0 && oe == 0xF,
            "[{=str}] check 7: during the break all four pins read low (0x{=u8:x}) and all four are driven (0x{=u8:x})",
            lv,
            oe
        );
        Timer::after_millis(2).await;
        let again = ow.break_start().err();
        check!(
            fails,
            again == Some(Error::BreakActive),
            "[{=str}] check 7: a second break_start is refused"
        );
        let w = ow.write(&[1]).await;
        check!(
            fails,
            w == Err(Error::BreakActive),
            "[{=str}] check 7: a write during the break is refused"
        );
        ow.break_end();
        Timer::after_micros(300).await;
        check!(
            fails,
            idle_ok(),
            "[{=str}] check 7: after the break all four are inputs with pull-ups (oe 0x{=u8:x}, pull-ups 0x{=u8:x}, levels 0x{=u8:x})",
            oe_bits(),
            pullups(),
            pad_levels()
        );
        rig.rng.fill(&mut tx[..21]);
        rig.rng.fill(&mut rx[..21]);
        let e = rig.exchange(&mut ow, 1, &tx[..21], &rx[..21], 8).await;
        check!(
            fails,
            e.is_ok(),
            "[{=str}] check 7: an echo of 21 bytes passes after the break"
        );
        ow.select(0).unwrap();
    }

    // ---- Check 8: a dropped read loses nothing ----
    {
        let _ = ow.select(3);
        let mut buf = [0u8; 8];
        let first = with_timeout(Duration::from_millis(5), ow.read(&mut buf)).await;
        check!(
            fails,
            first.is_err(),
            "[{=str}] check 8: a read with no data is dropped after 5 ms"
        );
        let five = [0x11, 0x22, 0x33, 0x44, 0x55];
        let tx_entry = ow.tx_entry();
        let mut got = [0u8; 5];
        let peer = &mut rig.peer;
        let send = async {
            Timer::after_millis(3).await;
            peer.arm_now(3, tx_entry, &five);
            peer.feed().await;
        };
        let rd = with_timeout(Duration::from_millis(100), ow.read_exact(&mut got));
        let (r, ()) = embassy_futures::join::join(rd, send).await;
        rig.peer.stop();
        check!(
            fails,
            matches!(r, Ok(Ok(()))) && got == five,
            "[{=str}] check 8: the read issued again gets all 5 bytes (got {=[u8]:x})",
            got
        );
        ow.select(0).unwrap();
    }

    // ---- Extra: a dropped write lets go of the line at once ----
    {
        let _ = ow.select(2);
        let data = [0x55u8; 100];
        let r = with_timeout(Duration::from_millis(3), ow.write(&data)).await;
        let released = idle_ok();
        check!(
            fails,
            r.is_err() && released,
            "[{=str}] extra: a write of 100 bytes dropped after 3 ms: line released and all pins idle at once (oe 0x{=u8:x}, levels 0x{=u8:x})",
            oe_bits(),
            pad_levels()
        );
        rig.rng.fill(&mut tx[..21]);
        rig.rng.fill(&mut rx[..21]);
        let e = rig.exchange(&mut ow, 2, &tx[..21], &rx[..21], 8).await;
        check!(
            fails,
            e.is_ok(),
            "[{=str}] extra: an echo of 21 bytes passes after the dropped write"
        );
        ow.select(0).unwrap();
    }

    // ---- Extra: positive control for the underrun report ----
    {
        let _ = ow.select(0);
        rig.rng.fill(&mut tx[..100]);
        let s0 = ow.stats();
        let r = with_timeout(
            Duration::from_millis(500),
            embassy_futures::join::join(
                ow.write(&tx[..100]),
                // 20 ms with the CPU busy: more than the FIFO and the output register last
                async { spin(3_000_000) },
            ),
        )
        .await;
        let s1 = ow.stats();
        check!(
            fails,
            matches!(r, Ok((Err(Error::Underrun), ())))
                && s1.underruns == s0.underruns + 1
                && s1.bytes_sent == s0.bytes_sent + 100,
            "[{=str}] extra: 20 ms without a refill: the write ends with Underrun after all 100 bytes, underruns {=u32} -> {=u32}",
            s0.underruns,
            s1.underruns
        );
        check!(
            fails,
            idle_ok(),
            "[{=str}] extra: the line is idle after the underrun"
        );
        rig.rng.fill(&mut tx[..21]);
        rig.rng.fill(&mut rx[..21]);
        let e = rig.exchange(&mut ow, 0, &tx[..21], &rx[..21], 8).await;
        check!(
            fails,
            e.is_ok(),
            "[{=str}] extra: an echo of 21 bytes passes after the underrun"
        );
    }

    // ---- Check 10: runt starts are not bytes ----
    {
        let line = 1usize;
        let _ = ow.select(line);
        for (name, cycles) in [("a quarter bit", 1953u32), ("a third of a bit", 2604)] {
            // the width the pad really had, seen by the observer
            let done = Cell::new(false);
            rig.obs.start(line);
            let (_, cnt) = embassy_futures::join::join(
                async {
                    Timer::after_micros(200).await;
                    low_pulse(line, cycles);
                    done.set(true);
                },
                rig.obs.drain(&mut words, &done),
            )
            .await;
            rig.obs.stop();
            let e = Edges::from_words(&words[..cnt]);
            let width = if e.n >= 2 {
                ns_of_q(e.t[1] - e.t[0])
            } else {
                -1
            };
            let nominal = cycles as i64 * 20 / 3;
            defmt::info!(
                "[STAT] runt pulse of {=str}: asked {=u32} CPU cycles ({=i64} ns), the pad was low {=i64} ns, the receiver's check is at 8 PIO cycles = 26040 ns",
                name,
                cycles,
                nominal,
                width
            );
            check!(
                fails,
                width >= nominal - 500 && width <= nominal + 2000,
                "[{=str}] check 10: the runt pulse of {=str} lasts {=i64} ns on the pad",
                name,
                width
            );
            let _ = ow.select(line);
            // 20 runts alone: no byte, no framing error, the runt counter moves by one each
            let mut bad = 0u32;
            let mut buf = [0u8; 4];
            for _ in 0..20 {
                let s0 = ow.stats();
                low_pulse(line, cycles);
                let r = with_timeout(Duration::from_millis(3), ow.read(&mut buf)).await;
                let s1 = ow.stats();
                if r.is_ok()
                    || s1.runt_starts != s0.runt_starts + 1
                    || s1.bytes_received != s0.bytes_received
                    || s1.framing_errors != s0.framing_errors
                {
                    bad += 1;
                }
            }
            check!(
                fails,
                bad == 0,
                "[{=str}] check 10: 20 runt pulses of {=str}: no byte, no framing error, runt counter +1 each ({=u32} wrong)",
                name,
                bad
            );
            // 20 runts each followed at once by a real byte: the byte arrives alone
            let mut bad = 0u32;
            for _ in 0..20 {
                let byte = [(rig.rng.next() >> 9) as u8];
                let s0 = ow.stats();
                rig.peer.prepare_now(line, ow.tx_entry(), &byte);
                cortex_m::interrupt::free(|_| {
                    force_low(line, true);
                    spin(cycles);
                    force_low(line, false);
                    rig.peer.go();
                });
                let r = with_timeout(Duration::from_millis(20), ow.read(&mut buf)).await;
                rig.peer.stop();
                let s1 = ow.stats();
                if !matches!(r, Ok(Ok(1)))
                    || buf[0] != byte[0]
                    || s1.runt_starts != s0.runt_starts + 1
                    || s1.framing_errors != s0.framing_errors
                {
                    bad += 1;
                }
                let _ = with_timeout(Duration::from_millis(1), ow.read(&mut buf)).await;
            }
            check!(
                fails,
                bad == 0,
                "[{=str}] check 10: 20 runt pulses of {=str}, each followed at once by a real byte: the byte arrives alone ({=u32} wrong)",
                name,
                bad
            );
        }
        ow.select(0).unwrap();
    }

    // ---- Check 11: an armed receiver waits for the line to be high before it looks for a start ----
    {
        let line = 1usize;
        let _ = ow.select(line);
        let mut buf = [0u8; 4];
        // after break_end, with the line held low for 100 us more (a slow rise)
        let s0 = ow.stats();
        let _ = ow.break_start();
        Timer::after_millis(1).await;
        force_low(line, true);
        ow.break_end();
        spin(15_000);
        force_low(line, false);
        let r = with_timeout(Duration::from_millis(3), ow.read(&mut buf)).await;
        let s1 = ow.stats();
        check!(
            fails,
            r.is_err()
                && s1.bytes_received == s0.bytes_received
                && s1.framing_errors == s0.framing_errors
                && s1.runt_starts == s0.runt_starts,
            "[{=str}] check 11: read first after break_end with the line low 100 us longer: nothing arrives (bytes {=u32}, framing {=u32}, runts {=u32} -> {=u32}, {=u32}, {=u32})",
            s0.bytes_received,
            s0.framing_errors,
            s0.runt_starts,
            s1.bytes_received,
            s1.framing_errors,
            s1.runt_starts
        );
        rig.rng.fill(&mut tx[..21]);
        rig.rng.fill(&mut rx[..21]);
        let e = rig.exchange(&mut ow, line, &tx[..21], &rx[..21], 8).await;
        check!(
            fails,
            e.is_ok(),
            "[{=str}] check 11: an echo of 21 bytes passes after it"
        );
        // after a write dropped while the line is low, 20 different cut points
        let data = [0u8; 100];
        let mut bad = 0u32;
        for k in 0..20u64 {
            let s0 = ow.stats();
            let cut = embassy_futures::select::select(ow.write(&data), async {
                Timer::after_micros(3000 + 41 * k).await;
                force_low(line, true);
            })
            .await;
            spin(15_000);
            force_low(line, false);
            let r = with_timeout(Duration::from_millis(3), ow.read(&mut buf)).await;
            let s1 = ow.stats();
            if !matches!(cut, Either::Second(()))
                || r.is_ok()
                || s1.bytes_received != s0.bytes_received
                || s1.framing_errors != s0.framing_errors
                || s1.runt_starts != s0.runt_starts
            {
                bad += 1;
            }
        }
        check!(
            fails,
            bad == 0,
            "[{=str}] check 11: read first after a write dropped with the line low, 20 cut points: nothing arrives ({=u32} wrong)",
            bad
        );
        rig.rng.fill(&mut tx[..21]);
        rig.rng.fill(&mut rx[..21]);
        let e = rig.exchange(&mut ow, line, &tx[..21], &rx[..21], 8).await;
        check!(
            fails,
            e.is_ok(),
            "[{=str}] check 11: an echo of 21 bytes passes after them"
        );
        ow.select(0).unwrap();
    }

    // ---- Check 9: release and build again, 100 times ----
    {
        let mut bad = 0u32;
        let mut idle_bad = 0u32;
        for i in 0..100usize {
            let parts = ow.release();
            if !idle_ok() {
                idle_bad += 1;
            }
            ow = match OneWire::new(parts.common, parts.sm, parts.pins, config) {
                Ok(ow) => ow,
                Err(e) => {
                    defmt::error!("[FAIL] rebuild {=usize}: {}", i, e.reason);
                    hil_common::exit(fails + 1);
                }
            };
            if !idle_ok() {
                idle_bad += 1;
            }
            rig.rng.fill(&mut tx[..21]);
            rig.rng.fill(&mut rx[..21]);
            if rig
                .exchange(&mut ow, i % LINES, &tx[..21], &rx[..21], 8)
                .await
                .is_err()
            {
                bad += 1;
            }
        }
        check!(
            fails,
            bad == 0,
            "[{=str}] check 9: 100 x release, new, echo of 21 bytes: {=u32} failed",
            bad
        );
        check!(
            fails,
            idle_bad == 0,
            "[{=str}] check 9: after every release and every new, all pins inputs with pull-ups ({=u32} exceptions)",
            idle_bad
        );
    }

    let total = ow.stats();
    defmt::info!(
        "[STAT] OneWire counters at the end of the last rebuild: sent {=u32}, received {=u32}, framing {=u32}, overruns {=u32}, underruns {=u32}, discarded {=u32}",
        total.bytes_sent,
        total.bytes_received,
        total.framing_errors,
        total.overruns,
        total.underruns,
        total.bytes_discarded
    );
    let parts = ow.release();
    drop(parts);
    check!(
        fails,
        idle_ok(),
        "[{=str}] at the end: every pin an input with pull-up, no output enable (oe 0x{=u8:x})",
        oe_bits()
    );
    hil_common::exit(fails);
}

struct OeTrace {
    ok: bool,
    first_fall: i64,
    stop_end: i64,
    oe_on: i64,
    oe_off: i64,
    oe_on_seen: bool,
    oe_off_seen: bool,
    oe_gaps: u32,
    other_oe: u8,
}

/// Writes `data` on the selected line while polling the output-enable bits and the pad level of
/// every line as fast as the CPU can, with the cycle counter as the clock. The write future is
/// polled every 1024 turns of the loop.
async fn oe_trace(ow: &mut Ow, data: &[u8]) -> OeTrace {
    let line = ow.selected();
    let bit = 1u8 << line;
    let mut t = OeTrace {
        ok: false,
        first_fall: 0,
        stop_end: 0,
        oe_on: 0,
        oe_off: 0,
        oe_on_seen: false,
        oe_off_seen: false,
        oe_gaps: 0,
        other_oe: 0,
    };
    let mut fut = pin!(ow.write(data));
    let mut cx = Context::from_waker(Waker::noop());
    let t0 = DWT::cycle_count();
    let mut turn = 0u32;
    let mut last_oe = 0u8;
    let mut last_lv = 0xFu8;
    let mut fell = false;
    let mut finished_at: Option<i64> = None;
    loop {
        let now = DWT::cycle_count().wrapping_sub(t0) as i64;
        let oe = oe_bits();
        let lv = pad_levels();
        t.other_oe |= oe & !bit;
        if oe & bit != 0 && !t.oe_on_seen {
            t.oe_on_seen = true;
            t.oe_on = now;
        }
        if oe & bit == 0 && last_oe & bit != 0 {
            t.oe_off_seen = true;
            t.oe_off = now;
        }
        last_oe = oe;
        if lv & bit == 0 && last_lv & bit != 0 && !fell {
            fell = true;
            t.first_fall = now;
            t.stop_end = now + 78_125 * data.len() as i64;
        }
        last_lv = lv;
        // between the first start bit and a bit before the end, the output enable must be on
        if fell && now > t.first_fall + 400 && now < t.stop_end - 7812 / 2 && oe & bit == 0 {
            t.oe_gaps += 1;
        }
        turn = turn.wrapping_add(1);
        if turn.is_multiple_of(1024)
            && finished_at.is_none()
            && let Poll::Ready(r) = fut.as_mut().poll(&mut cx)
        {
            t.ok = r == Ok(data.len());
            finished_at = Some(now);
        }
        if let Some(f) = finished_at
            && now > f + 150_000
        {
            break;
        }
        if now > 150_000_000 {
            break;
        }
    }
    t
}
