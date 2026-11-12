//! `OneWire`: a half-duplex 8N1 serial line on PIO, on one selected pin of up to four.
//!
//! One state machine does everything. Its program (21 instructions) has two halves:
//!
//! ```text
//! tx:    pull block            ; the count word: bytes - 1
//!        out y, 32
//!        mov isr, ~null        ; the end-of-write marker, pushed at the end
//!        set pindirs, 1        ; the pin is driven, latch high
//! byte:  pull ifempty block    ; a new word every fourth byte (stalls here, line high, if empty)
//!        set pins, 0 [14]      ; start bit, 16 cycles with the next line
//!        set x, 7
//! bit:   out pins, 1 [14]      ; data bit, LSB first, 16 cycles with the jmp
//!        jmp x-- bit
//!        set pins, 1 [13]      ; stop bit, 16 cycles with the jmp and the next pull
//!        jmp y-- byte
//!        set pindirs, 0        ; released: the pad pull-up holds the line
//!        push noblock          ; the end-of-write marker
//! rx:    wait 0 pin 0          ; start bit (wrap target)
//!        mov isr, null [20]    ; and on to the middle of data bit 0, 24 cycles after the edge
//!        set x, 7
//! rxbit: in pins, 1            ; sample
//!        jmp x-- rxbit [14]
//!        in pins, 1            ; the stop bit, in the middle of it
//!        push noblock          ; one word per frame: stop bit, data bit 7 .. 0 in bits 31..23
//!        wait 1 pin 0          ; wait out a low stop bit
//! ```
//!
//! The state machine rests in `rx`. A write pushes the message to the TX FIFO and forces a jump to
//! `tx`; at the end the program itself returns to `rx`: it lets go of the line in the last cycle of
//! the last stop bit (the line is high, so nothing changes on the wire), pushes the marker at the
//! first cycle after that bit, and waits for a start bit one cycle later. `wait` looks at the level,
//! so a reply that has already begun is seen at once. The divider is `clk_sys / (baud * 16)`.
//!
//! The pin fields of the state machine are written by `StateMachine::set_config`, which counts
//! them from GPIOBASE. The program uses `wait pin` and `in pins`, which count from the in base,
//! and never `jmp pin`, whose field `Config::set_jmp_pin` would fill with the absolute GPIO number.
#![allow(unsafe_code)]

use embassy_rp::gpio::Pull;
use embassy_rp::pio::program::{InstructionOperands, SetDestination, pio_asm};
use embassy_rp::pio::{
    Common, Config as PioConfig, Instance, LoadedProgram, Pin, ShiftConfig, ShiftDirection,
    StateMachine,
};
use fixed::types::U24F8;

use crate::timing::{self, Divider, MAX_LINES, RxWord};

/// What to build a `OneWire` with. Build it with [`Config::new`] and the `with_...` setters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// The baud rate. Default 115 200.
    pub baud: u32,
    /// The system clock in Hz, from `embassy_rp::clocks::clk_sys_freq()`. Default 150 MHz.
    pub clk_sys_hz: u32,
}

impl Config {
    /// 115 200 baud at a 150 MHz system clock.
    pub const fn new() -> Self {
        Self {
            baud: 115_200,
            clk_sys_hz: 150_000_000,
        }
    }

    /// Sets the baud rate.
    pub const fn with_baud(mut self, baud: u32) -> Self {
        self.baud = baud;
        self
    }

    /// Sets the system clock in Hz.
    pub const fn with_clk_sys_hz(mut self, clk_sys_hz: u32) -> Self {
        self.clk_sys_hz = clk_sys_hz;
        self
    }
}

impl Default for Config {
    fn default() -> Self {
        Self::new()
    }
}

/// Why a `OneWire` could not be built.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum BuildError {
    /// The baud rate is 0 or its divider is outside 1.0 to 65536.0 at this system clock.
    InvalidBaud,
    /// The pins are not consecutive GPIO numbers in ascending order.
    PinsNotConsecutive,
    /// The block has no room for the 21 instructions of the program.
    NoSpace,
}

/// The parts a `OneWire` is built from and gives back.
pub struct Parts<'d, PIO: Instance, const SM: usize, const N: usize> {
    /// The PIO block's instruction memory and pin registration.
    pub common: Common<'d, PIO>,
    /// The state machine.
    pub sm: StateMachine<'d, PIO, SM>,
    /// The pins, one per line, consecutive GPIO numbers.
    pub pins: [Pin<'d, PIO>; N],
}

/// A failed [`OneWire::new`]: the reason, and the parts, unchanged.
pub struct NewError<'d, PIO: Instance, const SM: usize, const N: usize> {
    /// Why.
    pub reason: BuildError,
    /// The parts that were passed in.
    pub parts: Parts<'d, PIO, SM, N>,
}

impl<PIO: Instance, const SM: usize, const N: usize> core::fmt::Debug for NewError<'_, PIO, SM, N> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("NewError")
            .field("reason", &self.reason)
            .finish_non_exhaustive()
    }
}

/// What can go wrong with a read, a write or a call on the line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Bytes arrived faster than they were read and some were dropped. The receive FIFO has been
    /// emptied; the stream has a gap of unknown length.
    Overrun,
    /// The write ran out of words at a word boundary: the bytes were all sent, but with a gap
    /// before at least one start bit.
    Underrun,
    /// `select` was given a line this `OneWire` does not have.
    InvalidLine,
    /// A break is on; call `break_end` first.
    BreakActive,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            Error::Overrun => "receive overrun",
            Error::Underrun => "transmit underrun",
            Error::InvalidLine => "no such line",
            Error::BreakActive => "a break is on",
        })
    }
}

impl core::error::Error for Error {}

impl embedded_io_async::Error for Error {
    fn kind(&self) -> embedded_io_async::ErrorKind {
        embedded_io_async::ErrorKind::Other
    }
}

/// Counters, saturating, never cleared by the crate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Stats {
    /// Frames with a low stop bit that were dropped.
    pub framing_errors: u32,
    /// Receive overruns found (reported by a read, or found and dropped by a write).
    pub overruns: u32,
    /// Writes that ran out of words at a word boundary.
    pub underruns: u32,
    /// Bytes sent.
    pub bytes_sent: u32,
    /// Bytes handed to callers by `read`.
    pub bytes_received: u32,
    /// Bytes received but dropped unread: by a write, which starts a new exchange, or by a
    /// `select` or a break.
    pub bytes_discarded: u32,
}

fn bump(counter: &mut u32, by: usize) {
    *counter = counter.saturating_add(u32::try_from(by).unwrap_or(u32::MAX));
}

/// A half-duplex 8N1 serial line on one of up to `N` consecutive pins. See the crate README.
pub struct OneWire<'d, PIO: Instance, const SM: usize, const N: usize> {
    common: Common<'d, PIO>,
    sm: StateMachine<'d, PIO, SM>,
    pins: [Pin<'d, PIO>; N],
    loaded: LoadedProgram<'d, PIO>,
    base: PioConfig<'d, PIO>,
    divider: Divider,
    tx_addr: u8,
    rx_addr: u8,
    line: usize,
    in_break: bool,
    stats: Stats,
}

/// Restores the line and the state machine if a write is dropped before it ends.
struct TxGuard<'a, 'd, PIO: Instance + 'd, const SM: usize, const N: usize> {
    ow: &'a mut OneWire<'d, PIO, SM, N>,
    armed: bool,
}

impl<'d, PIO: Instance + 'd, const SM: usize, const N: usize> Drop for TxGuard<'_, 'd, PIO, SM, N> {
    fn drop(&mut self) {
        if self.armed {
            self.ow.park();
        }
    }
}

const fn set_pins(data: u8) -> u16 {
    InstructionOperands::SET {
        destination: SetDestination::PINS,
        data,
    }
    .encode()
}

const fn set_pindirs(data: u8) -> u16 {
    InstructionOperands::SET {
        destination: SetDestination::PINDIRS,
        data,
    }
    .encode()
}

impl<'d, PIO: Instance + 'd, const SM: usize, const N: usize> OneWire<'d, PIO, SM, N> {
    /// Takes the parts, loads the program and parks every pin as an input with its pad pull-up
    /// on, line 0 selected, the receiver armed.
    ///
    /// `N` must be 1 to 4 (a compile-time check). The pins must be consecutive GPIO numbers in
    /// ascending order, made with this `Common`, and made after any pin of the same GPIO that
    /// another PIO block's `make_pio_pin` touched (the last one decides which block drives the
    /// pad). This sets the block's GPIOBASE window to match the pins: another state machine of
    /// the block must use pins of the same window (all below 32, or all 16 and above).
    ///
    /// # Errors
    ///
    /// See [`BuildError`]; the parts come back in the error.
    pub fn new(
        mut common: Common<'d, PIO>,
        sm: StateMachine<'d, PIO, SM>,
        mut pins: [Pin<'d, PIO>; N],
        config: Config,
    ) -> Result<Self, NewError<'d, PIO, SM, N>> {
        const { assert!(N >= 1 && N <= MAX_LINES, "OneWire serves 1 to 4 pins") };
        let fail = |reason, common, sm, pins| {
            Err(NewError {
                reason,
                parts: Parts { common, sm, pins },
            })
        };
        let consecutive = pins
            .iter()
            .zip(pins.iter().skip(1))
            .all(|(a, b)| a.pin().checked_add(1) == Some(b.pin()));
        if !consecutive {
            return fail(BuildError::PinsNotConsecutive, common, sm, pins);
        }
        let Some(divider) = timing::divider(config.clk_sys_hz, config.baud) else {
            return fail(BuildError::InvalidBaud, common, sm, pins);
        };
        let prg = pio_asm!(
            "public tx:",
            "    pull block",
            "    out y, 32",
            "    mov isr, ~null",
            "    set pindirs, 1",
            "byte:",
            "    pull ifempty block",
            "    set pins, 0 [14]",
            "    set x, 7",
            "bit:",
            "    out pins, 1 [14]",
            "    jmp x-- bit",
            "    set pins, 1 [13]",
            "    jmp y-- byte",
            "    set pindirs, 0",
            "    push noblock",
            ".wrap_target",
            "public rx:",
            "    wait 0 pin 0",
            "    mov isr, null [20]",
            "    set x, 7",
            "rxbit:",
            "    in pins, 1",
            "    jmp x-- rxbit [14]",
            "    in pins, 1",
            "    push noblock",
            "    wait 1 pin 0",
            ".wrap",
        );
        let loaded = match common.try_load_program(&prg.program) {
            Ok(loaded) => loaded,
            Err(_) => return fail(BuildError::NoSpace, common, sm, pins),
        };
        let tx_addr = loaded.origin.wrapping_add(prg.public_defines.tx as u8) % 32;
        let rx_addr = loaded.origin.wrapping_add(prg.public_defines.rx as u8) % 32;
        for pin in &mut pins {
            pin.set_pull(Pull::Up);
        }
        let mut base = PioConfig::default();
        base.use_program(&loaded, &[]);
        base.clock_divider = U24F8::from_bits(divider.q8);
        let shift = ShiftConfig {
            threshold: 32,
            direction: ShiftDirection::Right,
            auto_fill: false,
        };
        base.shift_in = shift;
        base.shift_out = shift;
        let mut this = Self {
            common,
            sm,
            pins,
            loaded,
            base,
            divider,
            tx_addr,
            rx_addr,
            line: 0,
            in_break: false,
            stats: Stats::default(),
        };
        this.idle_all();
        this.attach(0);
        Ok(this)
    }

    /// Stops the state machine, frees the program's instructions, leaves every pin an input with
    /// its pad pull-up on, and gives the parts back.
    pub fn release(mut self) -> Parts<'d, PIO, SM, N> {
        self.idle_all();
        let Self {
            mut common,
            sm,
            pins,
            loaded,
            ..
        } = self;
        // SAFETY: the state machine is disabled (idle_all) and only this program's instructions
        // are freed; the caller owned the rest of the block's memory and any other state machine
        // runs its own.
        unsafe { common.free_instr(loaded.used_memory) };
        Parts { common, sm, pins }
    }

    /// The number of lines (pins).
    pub const fn lines(&self) -> usize {
        N
    }

    /// The selected line.
    pub const fn selected(&self) -> usize {
        self.line
    }

    /// The clock divider in use and its error.
    pub const fn divider(&self) -> Divider {
        self.divider
    }

    /// The counters.
    pub const fn stats(&self) -> Stats {
        self.stats
    }

    /// The absolute address of the transmit entry of the program. Test use only.
    #[cfg(feature = "_test-hooks")]
    pub const fn tx_entry(&self) -> u8 {
        self.tx_addr
    }

    /// Makes `line` the one the next write and read use. The other lines stay inputs with pull-ups
    /// and are never driven. Bytes received but not yet read are dropped.
    ///
    /// # Errors
    ///
    /// [`Error::InvalidLine`] for a line `>= N`; [`Error::BreakActive`] during a break.
    pub fn select(&mut self, line: usize) -> Result<(), Error> {
        if self.in_break {
            return Err(Error::BreakActive);
        }
        if line >= N {
            return Err(Error::InvalidLine);
        }
        self.count_discarded();
        self.attach(line);
        Ok(())
    }

    /// Drives all lines low together, at once. The caller times the gap and calls
    /// [`break_end`](Self::break_end). Nothing else may be called on the line meanwhile. Bytes
    /// received but not yet read are dropped.
    ///
    /// # Errors
    ///
    /// [`Error::BreakActive`] if a break is already on.
    pub fn break_start(&mut self) -> Result<(), Error> {
        if self.in_break {
            return Err(Error::BreakActive);
        }
        self.count_discarded();
        self.sm.set_enable(false);
        let mask = Self::all_mask();
        let mut cfg = self.base;
        cfg.set_set_pins(&self.pins.each_ref());
        self.sm.set_config(&cfg);
        self.force(set_pins(0));
        self.force(set_pindirs(mask));
        self.in_break = true;
        Ok(())
    }

    /// Releases all lines and arms the receiver on the selected line again. Does nothing if no
    /// break is on.
    pub fn break_end(&mut self) {
        if !self.in_break {
            return;
        }
        self.idle_all();
        self.attach(self.line);
    }

    fn all_mask() -> u8 {
        (1u8 << N) - 1
    }

    /// Runs one forced instruction on the disabled state machine.
    fn force(&mut self, instr: u16) {
        // SAFETY: only called with the state machine disabled, with SET and JMP instructions that
        // touch the pin latch, the pin directions or the program counter.
        unsafe { self.sm.exec_instr(instr) };
    }

    /// With the state machine disabled: every pin latch high, every pin an input, no break.
    fn idle_all(&mut self) {
        self.sm.set_enable(false);
        let mut cfg = self.base;
        cfg.set_set_pins(&self.pins.each_ref());
        self.sm.set_config(&cfg);
        self.force(set_pindirs(0));
        self.force(set_pins(Self::all_mask()));
        self.in_break = false;
    }

    /// Points the state machine at `line` and parks it, receiver armed.
    fn attach(&mut self, line: usize) {
        let Some(pin) = self.pins.get(line) else {
            return;
        };
        let mut cfg = self.base;
        cfg.set_in_pins(&[pin]);
        cfg.set_out_pins(&[pin]);
        cfg.set_set_pins(&[pin]);
        self.sm.set_enable(false);
        self.sm.set_config(&cfg);
        self.line = line;
        self.park();
    }

    /// Stops the state machine, empties both FIFOs, releases the selected line with its latch
    /// high, and starts it again in the receiver.
    fn park(&mut self) {
        self.sm.set_enable(false);
        self.sm.restart();
        self.sm.clear_fifos();
        self.force(set_pindirs(0));
        self.force(set_pins(1));
        // SAFETY: as in `force`: a jump to the receiver, with the state machine disabled.
        unsafe { self.sm.exec_jmp(self.rx_addr) };
        let _ = self.sm.rx().stalled();
        let _ = self.sm.tx().stalled();
        self.sm.set_enable(true);
    }

    /// Drops what is in the RX FIFO, counting it, and clears the stall flags.
    fn count_discarded(&mut self) {
        while let Some(word) = self.sm.rx().try_pull() {
            match timing::decode_rx_word(word) {
                RxWord::Byte(_) => bump(&mut self.stats.bytes_discarded, 1),
                RxWord::FramingError => bump(&mut self.stats.framing_errors, 1),
                RxWord::TxDone => {}
            }
        }
        if self.sm.rx().stalled() {
            bump(&mut self.stats.overruns, 1);
        }
        let _ = self.sm.tx().stalled();
    }

    async fn write_impl(&mut self, data: &[u8]) -> Result<usize, Error> {
        if self.in_break {
            return Err(Error::BreakActive);
        }
        let n = data.len();
        let Some(mut words) = timing::fifo_image(data) else {
            return Ok(0);
        };
        self.count_discarded();
        let tx_addr = self.tx_addr;
        let mut guard = TxGuard {
            ow: self,
            armed: true,
        };
        let mut pending = words.next();
        while let Some(word) = pending {
            if !guard.ow.sm.tx().try_push(word) {
                break;
            }
            pending = words.next();
        }
        // SAFETY: a forced jump to the transmit entry. The state machine is enabled and sits in
        // the receiver or in a `wait`; the jump leaves it there and the program takes over.
        unsafe { guard.ow.sm.exec_jmp(tx_addr) };
        while let Some(word) = pending {
            guard.ow.sm.tx().wait_push(word).await;
            pending = words.next();
        }
        loop {
            let word = guard.ow.sm.rx().wait_pull().await;
            match timing::decode_rx_word(word) {
                RxWord::TxDone => break,
                RxWord::Byte(_) => bump(&mut guard.ow.stats.bytes_discarded, 1),
                RxWord::FramingError => bump(&mut guard.ow.stats.framing_errors, 1),
            }
        }
        let stalled = guard.ow.sm.tx().stalled();
        guard.armed = false;
        bump(&mut guard.ow.stats.bytes_sent, n);
        if stalled {
            bump(&mut guard.ow.stats.underruns, 1);
            Err(Error::Underrun)
        } else {
            Ok(n)
        }
    }

    /// Takes the bytes in the RX FIFO into `buf` without waiting. Returns how many.
    fn take(&mut self, buf: &mut [u8], mut n: usize) -> usize {
        while let Some(slot) = buf.get_mut(n) {
            let Some(word) = self.sm.rx().try_pull() else {
                break;
            };
            n += self.accept(word, slot);
        }
        n
    }

    /// Handles one word from the RX FIFO. Returns 1 if it was a byte, now in `slot`.
    fn accept(&mut self, word: u32, slot: &mut u8) -> usize {
        match timing::decode_rx_word(word) {
            RxWord::Byte(byte) => {
                *slot = byte;
                bump(&mut self.stats.bytes_received, 1);
                1
            }
            RxWord::FramingError => {
                bump(&mut self.stats.framing_errors, 1);
                0
            }
            RxWord::TxDone => 0,
        }
    }

    async fn read_impl(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        if self.in_break {
            return Err(Error::BreakActive);
        }
        if buf.is_empty() {
            return Ok(0);
        }
        if self.sm.rx().stalled() {
            bump(&mut self.stats.overruns, 1);
            while let Some(word) = self.sm.rx().try_pull() {
                if timing::decode_rx_word(word) == RxWord::FramingError {
                    bump(&mut self.stats.framing_errors, 1);
                } else if matches!(timing::decode_rx_word(word), RxWord::Byte(_)) {
                    bump(&mut self.stats.bytes_discarded, 1);
                }
            }
            return Err(Error::Overrun);
        }
        loop {
            let n = self.take(buf, 0);
            if n > 0 {
                return Ok(n);
            }
            let word = self.sm.rx().wait_pull().await;
            let mut first = 0u8;
            if self.accept(word, &mut first) == 1 {
                if let Some(slot) = buf.first_mut() {
                    *slot = first;
                }
                return Ok(self.take(buf, 1));
            }
        }
    }
}

impl<'d, PIO: Instance + 'd, const SM: usize, const N: usize> embedded_io_async::ErrorType
    for OneWire<'d, PIO, SM, N>
{
    type Error = Error;
}

impl<'d, PIO: Instance + 'd, const SM: usize, const N: usize> embedded_io_async::Read
    for OneWire<'d, PIO, SM, N>
{
    /// Waits for at least one byte and returns every byte that is in the receive FIFO, up to the
    /// length of `buf`. Dropping the future loses nothing. A byte with a low stop bit is counted
    /// and dropped. An overrun since the last call is returned first, as [`Error::Overrun`].
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        self.read_impl(buf).await
    }
}

impl<'d, PIO: Instance + 'd, const SM: usize, const N: usize> embedded_io_async::Write
    for OneWire<'d, PIO, SM, N>
{
    /// Sends all of `data` back to back on the selected line and returns the length once the last
    /// stop bit has ended and the line is released. Bytes received but not yet read are dropped
    /// first. If the future is dropped before it ends, the line is released at once and the rest
    /// of the message is not sent.
    async fn write(&mut self, data: &[u8]) -> Result<usize, Error> {
        self.write_impl(data).await
    }

    /// Nothing to do: `write` returns only when the bytes have left.
    async fn flush(&mut self) -> Result<(), Error> {
        Ok(())
    }
}
