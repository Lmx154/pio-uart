//! `UartRx`: a receive-only 8N1 serial line on PIO, on any one pin.
//!
//! One state machine, 13 instructions, no use of the pin as an output. It borrows the PIO block's
//! `Common` only to load its program and make its pin when built and to free the program when
//! released, so other state machines of the block keep running and keep their own instructions.
//!
//! ```text
//! rx:    wait 0 pin 0          ; cycle 0: the start edge (wrap target)
//!        nop [5]               ; cycles 1 to 6
//!        jmp pin runt          ; cycle 7: the line is high again, so it was a runt, not a start bit
//!        set x, 7 [14]         ; cycles 8 to 22
//! rxbit: in pins, 1            ; cycle 23 + 16 k: data bit k, least significant first
//!        jmp x-- rxbit [14]
//!        in pins, 1            ; cycle 151: the stop bit
//!        push noblock          ; one word per frame: stop bit, data bit 7 .. 0 in bits 31..23
//! arm:   wait 1 pin 0          ; wait out a low stop bit (wrap top); also the entry at start
//! runt:  mov isr, ~null        ; (after the wrap)
//!        in null, 1            ; 0x7FFF_FFFF, the runt marker
//!        push noblock
//!        jmp rx
//! ```
//!
//! 16 PIO cycles per bit (divider `clk_sys / (baud * 16)`). The edge is seen 0 to 1 cycle after it
//! happens, so the samples fall 7 to 8 cycles after the edge for the start bit and 16 later for
//! each further bit; see [`timing::uart_rx_tolerance_ppm`]. The state machine starts at `arm`, so
//! a line that is low when it is built, or stays low after a low stop bit, never makes a frame:
//! the receiver first waits for the line to be high, and only then for a start bit.
//!
//! The pin fields (in base) are written by `StateMachine::set_config`, which counts them from
//! GPIOBASE. The JMP pin field is written absolute by embassy-rp 0.10.0 (`Config::set_jmp_pin`),
//! which is wrong for a pin above 31 in the GPIO16-47 window; here it is set from
//! [`timing::pin_field`] to the number counted from GPIOBASE, the same as the in base.
#![allow(unsafe_code)]

use embassy_rp::gpio::Pull;
use embassy_rp::pio::program::pio_asm;
use embassy_rp::pio::{
    Common, Config as PioConfig, FifoJoin, Instance, LoadedProgram, Pin, ShiftConfig,
    ShiftDirection, StateMachine,
};
use fixed::types::U24F8;

use crate::one_wire::Config;
use crate::timing::{self, Divider, RxWord};

/// Why a `UartRx` could not be built.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum BuildError {
    /// The baud rate is 0 or its divider is outside 1.0 to 65536.0 at this system clock.
    InvalidBaud,
    /// The block has no room for the 13 instructions of the program.
    NoSpace,
    /// The pin is above GPIO47, which no PIO block reaches.
    InvalidPin,
}

/// The parts a `UartRx` gives back: its state machine and its pin.
pub struct Parts<'d, PIO: Instance, const SM: usize> {
    /// The state machine.
    pub sm: StateMachine<'d, PIO, SM>,
    /// The pin, as given to [`UartRx::new`], pad pull-up on.
    pub pin: Pin<'d, PIO>,
}

/// A failed [`UartRx::new`]: the reason, and the state machine and pin, unchanged.
pub struct NewError<'d, PIO: Instance, const SM: usize> {
    /// Why.
    pub reason: BuildError,
    /// The state machine that was passed in.
    pub sm: StateMachine<'d, PIO, SM>,
    /// The pin that was passed in, untouched.
    pub pin: Pin<'d, PIO>,
}

impl<PIO: Instance, const SM: usize> core::fmt::Debug for NewError<'_, PIO, SM> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("NewError")
            .field("reason", &self.reason)
            .finish_non_exhaustive()
    }
}

/// What can go wrong with a read.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Bytes arrived faster than they were read and some were dropped. The receive FIFO has been
    /// emptied; the stream has a gap of unknown length.
    Overrun,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            Error::Overrun => "receive overrun",
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
    /// Bytes handed to callers by `read`.
    pub bytes_received: u32,
    /// Frames with a low stop bit that were dropped.
    pub framing_errors: u32,
    /// Overrun EVENTS found by a `read`, not lost bytes. One count is one `read` that found the
    /// receiver's overrun flag set (and emptied the FIFO); the flag says that at least one byte
    /// was dropped, not how many.
    pub overruns: u32,
    /// Low pulses that were gone again at the middle of a start bit: ignored, no byte.
    pub runt_starts: u32,
}

fn bump(counter: &mut u32) {
    *counter = counter.saturating_add(1);
}

/// A receive-only 8N1 serial line on one pin. See the crate README.
pub struct UartRx<'d, PIO: Instance, const SM: usize> {
    sm: StateMachine<'d, PIO, SM>,
    pin: Pin<'d, PIO>,
    loaded: LoadedProgram<'d, PIO>,
    divider: Divider,
    arm_addr: u8,
    stats: Stats,
}

impl<'d, PIO: Instance + 'd, const SM: usize> UartRx<'d, PIO, SM> {
    /// Takes the state machine and the pin, loads the program into `common`, turns the pad pull-up
    /// of the pin on, and starts the state machine, waiting for the line to be high.
    ///
    /// The pin must have been made with this `Common`'s `make_pio_pin`, and made after any pin of
    /// the same GPIO that another PIO block's `make_pio_pin` touched (the last one decides which
    /// block drives the pad). This sets the block's GPIOBASE window to match the pin (0 for GPIO0
    /// to 31, 16 for GPIO32 to 47): another state machine of the block must use pins of the same
    /// window. The pin is only ever read; nothing here sets a pin direction or a pin level, so the
    /// line may be driven by another state machine of the same block (a loop-back test does).
    ///
    /// # Errors
    ///
    /// See [`BuildError`]; the state machine and the pin come back in the error, and nothing is
    /// loaded or changed.
    pub fn new(
        common: &mut Common<'d, PIO>,
        sm: StateMachine<'d, PIO, SM>,
        mut pin: Pin<'d, PIO>,
        config: Config,
    ) -> Result<Self, NewError<'d, PIO, SM>> {
        let Some(divider) = timing::divider(config.clk_sys_hz, config.baud) else {
            return Err(NewError {
                reason: BuildError::InvalidBaud,
                sm,
                pin,
            });
        };
        let Some(field) = timing::pin_field(pin.pin()) else {
            return Err(NewError {
                reason: BuildError::InvalidPin,
                sm,
                pin,
            });
        };
        let prg = pio_asm!(
            ".wrap_target",
            "rx:",
            "    wait 0 pin 0",
            "    nop [5]",
            "    jmp pin runt",
            "    set x, 7 [14]",
            "rxbit:",
            "    in pins, 1",
            "    jmp x-- rxbit [14]",
            "    in pins, 1",
            "    push noblock",
            "public arm:",
            "    wait 1 pin 0",
            ".wrap",
            "runt:",
            "    mov isr, ~null",
            "    in null, 1",
            "    push noblock",
            "    jmp rx",
        );
        let loaded = match common.try_load_program(&prg.program) {
            Ok(loaded) => loaded,
            Err(_) => {
                return Err(NewError {
                    reason: BuildError::NoSpace,
                    sm,
                    pin,
                });
            }
        };
        let arm_addr = loaded.origin.wrapping_add(prg.public_defines.arm as u8) % 32;
        pin.set_pull(Pull::Up);
        let mut cfg = PioConfig::default();
        cfg.use_program(&loaded, &[]);
        cfg.clock_divider = U24F8::from_bits(divider.q8);
        let shift = ShiftConfig {
            threshold: 32,
            direction: ShiftDirection::Right,
            auto_fill: false,
        };
        cfg.shift_in = shift;
        cfg.shift_out = shift;
        cfg.fifo_join = FifoJoin::RxOnly;
        cfg.set_in_pins(&[&pin]);
        let mut exec = cfg.get_exec();
        exec.jmp_pin = field;
        // SAFETY: only the JMP PIN source changes, to the pin counted from GPIOBASE like the in
        // base; the wrap and side-set settings are the ones `use_program` wrote.
        unsafe { cfg.set_exec(exec) };
        let mut this = Self {
            sm,
            pin,
            loaded,
            divider,
            arm_addr,
            stats: Stats::default(),
        };
        this.sm.set_enable(false);
        this.sm.set_config(&cfg);
        this.start();
        Ok(this)
    }

    /// Stops the state machine, frees the program's instructions in `common` (the same `Common`
    /// that was given to [`new`](Self::new)), leaves the pad pull-up on, and gives the state
    /// machine and the pin back.
    pub fn release(mut self, common: &mut Common<'d, PIO>) -> Parts<'d, PIO, SM> {
        self.sm.set_enable(false);
        self.sm.restart();
        self.sm.clear_fifos();
        let _ = self.sm.rx().stalled();
        let Self {
            sm, pin, loaded, ..
        } = self;
        // SAFETY: the state machine is disabled and only this program's instructions are freed;
        // every other state machine of the block runs its own instructions.
        unsafe { common.free_instr(loaded.used_memory) };
        Parts { sm, pin }
    }

    /// The clock divider in use and its error.
    pub const fn divider(&self) -> Divider {
        self.divider
    }

    /// The counters.
    pub const fn stats(&self) -> Stats {
        self.stats
    }

    /// With the state machine disabled: empties the FIFO and the flags and runs it from `arm`.
    fn start(&mut self) {
        self.sm.restart();
        self.sm.clear_fifos();
        // SAFETY: a jump to the receiver's `arm`, with the state machine disabled.
        unsafe { self.sm.exec_jmp(self.arm_addr) };
        let _ = self.sm.rx().stalled();
        self.sm.set_enable(true);
    }

    /// Handles one word from the RX FIFO. Returns true if it was a byte, now in `slot`.
    fn accept(&mut self, word: u32, slot: &mut u8) -> bool {
        match timing::decode_rx_word(word) {
            RxWord::Byte(byte) => {
                *slot = byte;
                bump(&mut self.stats.bytes_received);
                true
            }
            RxWord::FramingError => {
                bump(&mut self.stats.framing_errors);
                false
            }
            RxWord::Runt => {
                bump(&mut self.stats.runt_starts);
                false
            }
            RxWord::TxDone => false,
        }
    }

    async fn read_impl(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        if self.sm.rx().stalled() {
            bump(&mut self.stats.overruns);
            while let Some(word) = self.sm.rx().try_pull() {
                match timing::decode_rx_word(word) {
                    RxWord::FramingError => bump(&mut self.stats.framing_errors),
                    RxWord::Runt => bump(&mut self.stats.runt_starts),
                    RxWord::Byte(_) | RxWord::TxDone => {}
                }
            }
            return Err(Error::Overrun);
        }
        let mut n = 0;
        loop {
            while let Some(slot) = buf.get_mut(n) {
                let Some(word) = self.sm.rx().try_pull() else {
                    break;
                };
                if self.accept(word, slot) {
                    n += 1;
                }
            }
            if n > 0 {
                return Ok(n);
            }
            let word = self.sm.rx().wait_pull().await;
            if let Some(slot) = buf.first_mut()
                && self.accept(word, slot)
            {
                n = 1;
            }
        }
    }
}

impl<'d, PIO: Instance + 'd, const SM: usize> embedded_io_async::ErrorType for UartRx<'d, PIO, SM> {
    type Error = Error;
}

impl<'d, PIO: Instance + 'd, const SM: usize> embedded_io_async::Read for UartRx<'d, PIO, SM> {
    /// Waits for at least one byte and returns every byte that is in the receive FIFO, up to the
    /// length of `buf`. Dropping the future loses nothing. A frame with a low stop bit is counted
    /// and dropped, and so is a runt start (a low pulse gone again at the middle of the start
    /// bit). An overrun since the last call is returned first, as [`Error::Overrun`], and that
    /// read empties the FIFO.
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        self.read_impl(buf).await
    }
}
