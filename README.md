# pio-uart

Byte transports on PIO for the RP2350, `no_std`. They carry bytes and know no protocol. Version 0.1 has two transports, for embassy-rp 0.10 (feature `embassy-rp-0-10`): `OneWire`, a half-duplex 8N1 serial line on one selected pin of up to four consecutive pins, and `UartRx`, a receive-only 8N1 line on any one pin (including GPIO32 to 47) that owns only its state machine and its pin (made with `make_pio_pin` by the caller). The core, always built and without a dependency, is the divider and timing arithmetic and the packing of bytes into FIFO words. `defmt` 1.x is optional. It is written for the MARV V2 Rev 1 flight controller but takes the baud rate and clock as parameters.

Status: version 0.1.0, `publish = false`, and the API may still change. `OneWire` has been tested on a MARV V2 Rev 1 board in loop-back on spare pins against a test peer written for the purpose (2026-11-12, see "Measured on the MARV V2 Rev 1 board"). `UartRx` has been tested on the same board in loop-back on GPIO34 against a sender written for the test (2026-11-12, see the same section). Neither has been run against any real device; no real ESC telemetry has been received.

## Works with

The timing arithmetic is pure and works on any target. `OneWire` and `UartRx` need the RP2040/RP2350 PIO and are written for `embassy-rp` 0.10, behind the `embassy-rp-0-10` feature; they need no embassy executor and no `embassy-time`. No back-end for `rp2040-hal` or `rp235x-hal` exists; the core and the transport traits are the place to add one.

## Quick start

The arithmetic (compiled by `cargo test`):

```rust
use pio_uart::timing::{bit_time_ns, count_word, decode_rx_word, divider, fifo_image, gpiobase_for_pin, pin_field, tx_refill_window_ns, uart_rx_tolerance_ppm, RxWord};

// 19 200 baud at 150 MHz: 16 PIO cycles per bit, divider 488 + 72/256, exact.
let d = divider(150_000_000, 19_200).unwrap();
assert_eq!((d.integer(), d.fraction(), d.error_ppm), (488, 72, 0));
assert_eq!(bit_time_ns(19_200), Some(52_083));

// Five bytes go to the TX FIFO as three words: the count minus one, then four bytes and one byte.
assert_eq!(count_word(5), Some(4));
let words: Vec<u32> = fifo_image(&[1, 2, 3, 4, 5]).unwrap().collect();
assert_eq!(words, [4, 0x0403_0201, 5]);

// The CPU has 6.25 ms to refill a free TX FIFO slot before a write stalls.
assert_eq!(tx_refill_window_ns(19_200), Some(6_250_000));

// A received frame is one word: stop bit, then data bit 7 down to bit 0, in bits 31..23.
assert_eq!(decode_rx_word(0x8000_0000 | 0xA5 << 23), RxWord::Byte(0xA5));
assert_eq!(decode_rx_word(0xA5 << 23), RxWord::FramingError);

// UartRx on GPIO34: the PIO block's window starts at GPIO16, so the pin fields hold 34 - 16.
assert_eq!((gpiobase_for_pin(34), pin_field(34)), (Some(16), Some(18)));
// The receiver tolerates a sender from 4.6 % slow to 3.9 % fast, by arithmetic (back-to-back bytes).
assert_eq!(uart_rx_tolerance_ppm(), (-46_357, 38_961));
```

`OneWire` (feature `embassy-rp-0-10`; the chip feature, for example `embassy-rp/rp235xb`, comes from the application). Not compiled here, because the README is built on a host:

```rust,ignore
use embedded_io_async::{Read, Write};
use pio_uart::{Config, OneWire};

bind_interrupts!(struct Irqs { PIO2_IRQ_0 => pio::InterruptHandler<PIO2>; });

let Pio { mut common, sm0, .. } = Pio::new(p.PIO2, Irqs);
let pins = [
    common.make_pio_pin(p.PIN_44),
    common.make_pio_pin(p.PIN_45),
    common.make_pio_pin(p.PIN_46),
    common.make_pio_pin(p.PIN_47),
];
let config = Config::new().with_baud(19_200).with_clk_sys_hz(clocks::clk_sys_freq());
let mut line = OneWire::new(common, sm0, pins, config).unwrap();

line.select(2).unwrap();
line.write(&request).await.unwrap();                       // back to back, returns after the last stop bit
let n = with_timeout(REPLY_TIMEOUT, line.read(&mut reply)).await; // the caller's timeout, no byte lost if it fires

line.break_start().unwrap();   // all four pins low, together
Timer::after(BREAK_TIME).await;
line.break_end();

let parts = line.release();    // Common, StateMachine and the pins, exactly as given
```

`UartRx` (same feature). It does NOT take the block's `Common`: it borrows it to load its program and again to free it. The pin is made by the caller with `common.make_pio_pin`, as for `OneWire`. Another state machine of the block, with its own program, keeps running:

```rust,ignore
use embedded_io_async::Read;
use pio_uart::{Config, UartRx};

bind_interrupts!(struct Irqs { PIO1_IRQ_0 => pio::InterruptHandler<PIO1>; });

let Pio { mut common, sm0, sm1, .. } = Pio::new(p.PIO1, Irqs);   // sm0: the LED driver, window GPIO16-47
let config = Config::new().with_baud(115_200).with_clk_sys_hz(clocks::clk_sys_freq());
let pin = common.make_pio_pin(p.PIN_34);
let mut rx = UartRx::new(&mut common, sm1, pin, config).unwrap();   // pad pull-up on; an error hands sm1 and the pin back

let mut buf = [0u8; 10];
let n = with_timeout(TIMEOUT, rx.read(&mut buf)).await;   // the caller's timeout, no byte lost if it fires
let stats = rx.stats();        // bytes_received, framing_errors, overruns, runt_starts

let parts = rx.release(&mut common);   // parts.sm, parts.pin: build again with them; exactly its 13 instructions are freed
```

`Config` is `#[non_exhaustive]`: build it with `Config::new()` and the `with_...` setters. `Config::new()` is 115 200 baud at 150 MHz. Both transports use it.

## What the application provides

- The parts, by value: the PIO block's `Common`, one `StateMachine`, and 1 to 4 `Pin`s made with that `Common::make_pio_pin`, consecutive GPIO numbers in ascending order (pins 30 and 31 do not wrap to 32). `OneWire::new` returns the parts unchanged inside the error if it cannot take them (`BuildError::InvalidBaud`, `PinsNotConsecutive`, `NoSpace`).
- The baud rate and `clk_sys` in Hz. The divider is `clk_sys / (baud * 16)` in 1/256 steps; it must lie between 1.0 and 65536.0.
- The PIO interrupt binding for the block (`bind_interrupts!`), as for any embassy-rp PIO use: reads and writes wait on FIFO interrupts.
- Timeouts. The crate has no timer; the caller wraps a read in its own timeout. Dropping a pending read loses nothing.
- Time for the break: `break_start`, the caller waits, `break_end`.

For `UartRx`: the block's `Common` by `&mut` at `new` and at `release` (the same one), one `StateMachine` by value, and the pin by value, made with that `Common::make_pio_pin` (last, if other PIO blocks look at the same GPIO, as in rule 2); the baud rate and `clk_sys`; the PIO interrupt binding; the caller's timeouts.

## Rules for correct use

Rules 1 to 13 are for `OneWire`; rules U1 to U9 follow them and are for `UartRx`.

1. The holder owns all 32 instruction slots of the block, in the sense that it loads 27 of them when built and frees exactly those on `release`. Any other state machine of the block must leave those 27 alone (a second state machine of the block can use the other 5).
2. Make the pins last. A pad is driven by the one PIO block its function select names, the block that called `make_pio_pin` on it last. If other blocks look at the same GPIOs (a logic-analyser state machine, a test peer), make their `Pin`s first and the `OneWire` pins last.
3. The block's GPIOBASE window is set from the pins. Other state machines of the same block must use pins of the same window (all below 32, or all 16 and above); GPIO44 to 47 are in the upper one. Pin fields are counted from GPIOBASE inside the crate; the program never uses `jmp pin`, whose field `Config::set_jmp_pin` fills with the absolute GPIO number.
4. A write is a whole message, sent back to back (stop bit straight into the next start bit). Keep the executor free enough to refill the TX FIFO: the FIFO holds 4 words, each word 4 bytes, and the output register holds one more. Once a FIFO slot is free the CPU has `3 words * 4 bytes * 10 bits` = 120 bit times to fill it, 6.25 ms at 19 200 baud, 1.04 ms at 115 200 (`timing::tx_refill_window_ns`). If a write runs out of words it waits at a word boundary with the line high, finishes, and returns `Error::Underrun` (a gap before a start bit, never a damaged bit by design; the gap is not measured by the on-board test).
5. Read in time. Each received byte takes one word of a 4-deep RX FIFO: 4 byte times to empty it, 2.08 ms at 19 200 baud (`timing::rx_drain_window_ns`). A byte that finds it full is dropped and the overrun is reported by the next `read` as `Error::Overrun`; that read also empties the FIFO, so the bytes before the gap are dropped with it. The state machine keeps tracking the line, so later bytes are good.
6. A byte whose stop bit is low is a framing error: counted in `stats()`, dropped, never delivered as data. A line held low (a break) gives one such byte and the receiver then waits for the line to go high.
7. A `write`, a `select`, a `break_start` and `release` throw away what has been received and not read; the bytes are counted in `stats().bytes_discarded`. A write starts a new exchange: read the reply to the last request before sending the next. An overrun, a framing error or a runt start that one of them finds in the receive FIFO or in the receiver's flags is counted in `stats()` and cleared: the next `read` does NOT report it. Only a `read` that finds the overrun flag set returns `Error::Overrun`; this is also in the rustdoc of `read`.
8. Exactly one line is active. `select(line)` points the state machine at it; the other lines stay inputs with pad pull-ups and are never driven. Only a write and a break drive anything.
9. A write drives the line push-pull only for the bytes. The program lets go in the last cycle of the last stop bit (the line is high, so nothing changes), 1/16 bit early. The end marker is pushed in the first cycle after that bit and the `wait 0 pin 0` of the receiver runs one cycle after the push. A device may start its reply the moment the stop bit ends: `wait 0 pin` looks at the level, so a start bit that has already begun is seen at once. Tracing the program (the review did this, and so did I) a reply that starts exactly at the end of our stop bit is sampled 0 to 2 cycles late of sixteen, which is correct; this is arithmetic, the board test cannot make such a reply (see "Tests"). A reply that starts BEFORE our stop bit has ended is contention on the line and can be mis-framed; a device must wait for the end of the stop bit.
10. If the future of a `write` is dropped before it ends, the line is released and the receiver re-armed at once, in the drop; the rest of the message is not sent. A dropped `read` leaves the driver as it was.
10a. A receiver that is armed after the line was driven or held low (after a dropped write, a break, a `select`, a `new`, and after every frame) first waits for the line to be HIGH, and only then for a start bit. So a line that is still low or still rising does not make a phantom byte (checked: check 11). The one exception is the arming at the end of a write, which does not wait for high so that a reply that begins at once is not missed (rule 9).
10b. The receiver looks at the line again in the middle of the start bit, 8 cycles (half a bit) after the falling edge. If it is high, the pulse is a runt: no byte, `stats().runt_starts` counts it, and the receiver waits for a new edge. Limit (by arithmetic, not measured): a low pulse that is still low at that point passes the check; a pulse of about half a bit to a bit and a half that is high at the first data sample is read as a start bit followed by ones and delivered as 0xFF (a start bit 12 cycles long cannot be told from a start bit with a bad rate). The pulses the board test makes are 4.5 and 5.7 cycles long.
11. A break (`break_start`) drives all of its pins low in one instruction and holds them until `break_end`. `read`, `write`, `select` and a second `break_start` return `Error::BreakActive` meanwhile.
12. Idle state at all times outside a write or a break: every pin an input, pad pull-up on, nothing driven.
13. The baud rate is only as accurate as `clk_sys`. The receiver samples in the middle of each bit and of the stop bit, 9.5 bit times after the start edge; by arithmetic (not measured) it tolerates a far-end rate error of about 4 %.

U1. `UartRx::new(&mut common, sm, pin, config)` loads 13 instructions into `common`, turns the pad pull-up of the pin on and starts the state machine. The pin is made by the caller with the same `Common`'s `make_pio_pin`, once. If `new` cannot build (`BuildError::InvalidBaud`, `NoSpace`, `InvalidPin`) nothing is loaded or changed and the error carries the state machine and the pin back. `release(&mut common)` stops the state machine, frees exactly the instructions `new` loaded, and returns `Parts { sm, pin }` with the pad pull-up still on; `new` accepts those parts again. Other state machines of the block, and their instructions, are not touched.
U2. The block's GPIOBASE window is set from the pin: 0 for GPIO0 to 31, 16 for GPIO32 to 47 (the rule embassy-rp 0.10 applies; `timing::gpiobase_for_pin`). Every other state machine of the block must use the same window. The in base and the JMP pin field are written counted from GPIOBASE (`timing::pin_field`: 18 for GPIO34); the program uses `jmp pin` for the runt check, and embassy-rp 0.10.0's `Config::set_jmp_pin` would store 34, which in this window means another pin, so the crate fills the field itself.
U3. `UartRx` never sets a pin direction or a pin level. The pin is only read, so another state machine of the block may drive the same pin (the board test does), and the application decides the pad's pull.
U4. Read in time. The RX FIFO is joined with the unused TX FIFO: 8 words, one per received byte, so 8 byte times to empty it: 694 us at 115 200 baud (`timing::uart_rx_drain_window_ns`). A byte that finds it full is dropped and the receiver goes on by itself. The next `read` that finds the overrun flag set returns `Error::Overrun`, counts one in `stats().overruns` (one per such read: an overrun EVENT, not a count of lost bytes; the hardware flag says "at least one byte was dropped") and empties the FIFO, so the bytes before the gap are dropped with it; later bytes are good.
U5. A frame whose stop bit is low gives no byte: `stats().framing_errors` counts it, and the receiver then waits for the line to be HIGH before it looks for the next start bit. A line held low (a break) gives one framing error.
U6. The state machine starts by waiting for the line to be high. Built on a line that is low, it makes no byte, no framing error and no runt, however long the line stays low; the same after a low stop bit and after a runt.
U7. The receiver looks at the line again 7 to 8 cycles (about half a bit) after the falling edge it saw. If the line is high, the pulse is a runt: no byte, `stats().runt_starts` counts it. A low pulse that is still low at that point is a start bit; a pulse of one bit time followed by high gives 0xFF (the board test shows this as a control). A real start bit that begins less than about 12 cycles (three quarters of a bit) after the start of a glitch is merged with it: the first version of the board check, which started the byte about 3 cycles after the pulse, failed (log `hil/logs/uart_rx_loopback-20261112-174858.log`, the cause not captured); the final check starts the byte about one bit after the pulse began.
U8. Baud tolerance, by arithmetic (`timing::uart_rx_tolerance_ppm`): a sender from 46 357 ppm slow to 38 961 ppm fast, in bytes sent back to back with one stop bit; the fast limit is the receiver being ready for the next start bit again 154 cycles after an edge, the slow limit is the stop bit being sampled at cycle 151 against 9 sender bit times. Measured: error-free at -3 %, exact and +3 % (4608 bytes each); with the sender at -6 % and +6 % it reports errors. The limits themselves are not measured. The baud rate is only as accurate as `clk_sys`.
U9. The crate has no timer and the read has no timeout; the caller wraps a read in its own. Dropping a pending `read` loses nothing. A `read` of an empty buffer returns 0.
## What the crate does

- `timing` (no dependency): `divider(clk_sys, baud)` with its error in ppm, `bit_time_ns`, `byte_time_ns`, `tx_refill_window_ns`, `rx_drain_window_ns`, `cycles_for_ns`, `count_word`, `pack_word`, `word_count`, `fifo_image`, `decode_rx_word`. 16 PIO cycles per bit.
- `timing` also has, for `UartRx`: `uart_rx_sample_cycle`, `uart_rx_tolerance_ppm`, `uart_rx_drain_window_ns`, `gpiobase_for_pin`, `pin_field` and the constants `RX_JOINED_FIFO_WORDS`, `UART_RX_SAMPLE_BASE_CYCLE`, `UART_RX_REARM_CYCLE`, `PIN_COUNT`.
- `OneWire::new`, `release`, `select`, `selected`, `lines`, `break_start`, `break_end`, `stats`, `divider`; `embedded_io_async::{ErrorType, Read, Write}` (0.7) on the selected line. `Write::write` sends the whole slice and returns its length; `flush` does nothing because `write` returns only when the bytes have left.
- The program is 27 instructions: the transmitter (13: `pull block` for the count word, `pull ifempty block` every fourth byte, `set pins`/`out pins` bits of 16 cycles) and the receiver (14: `wait 0 pin 0`, a look at the line in the middle of the start bit (`mov x, pins`, `jmp x-- runt`), samples in the middle of each bit, one `push noblock` per frame carrying the stop bit and the byte, `wait 1 pin 0`, and a 4-instruction runt path that pushes a marker word). The JMP pin is not used (the look is `mov x, pins`, which the HAL masks to the one in-pin), so no pin field is written absolute. The end of a write is signalled by the program with a marker word in the RX FIFO, pushed after the release; that is what `write` waits for. FIFOs are not joined; no DMA.
- `Stats` (saturating, `#[non_exhaustive]`, never cleared by the crate): `framing_errors`, `overruns`, `underruns`, `bytes_sent`, `bytes_received`, `bytes_discarded`, `runt_starts`.
- Errors are `#[non_exhaustive]`: `Overrun`, `Underrun`, `InvalidLine`, `BreakActive`; all map to `ErrorKind::Other` for embedded-io.
- `UartRx::new`, `release`, `divider`, `stats`; `embedded_io_async::{ErrorType, Read}` (0.7). Types `UartRxBuildError`, `UartRxNewError`, `UartRxParts`, `UartRxError` (`Overrun`, maps to `ErrorKind::Other`; `#[non_exhaustive]`) and `UartRxStats` (saturating, `#[non_exhaustive]`, never cleared by the crate): `bytes_received`, `framing_errors`, `overruns` (overrun events found by a `read`, not lost bytes), `runt_starts`. `Config` is the one `OneWire` uses.
- The `UartRx` program is 13 instructions, 16 PIO cycles per bit, divider `clk_sys / (baud * 16)` as for `OneWire`: `wait 0 pin 0` (cycle 0, the start edge), `nop [5]`, `jmp pin runt` (cycle 7: line high again, a runt), `set x, 7 [14]`, eight `in pins, 1` at cycles 23, 39, ... 135 (data bits, least significant first, `jmp x-- [14]`), `in pins, 1` at cycle 151 (the stop bit), `push noblock` (one word per frame, stop bit and data bit 7 to 0 in bits 31 to 23, the same word `timing::decode_rx_word` decodes), `wait 1 pin 0` (wrap top, also the start entry), then outside the wrap the runt path (`mov isr, ~null`, `in null, 1`, `push noblock`, `jmp rx`: the marker 0x7FFF_FFFF). At 115 200 baud and 150 MHz: divider 81 + 97/256 (20 833/256), so a bit is 16 PIO cycles = 1302.1 clk_sys cycles = 8.68 us (the actual rate is 16 ppm above 115 200); the start bit is looked at 7 to 8 cycles after the edge (about 4 us, 0.47 bit) and the stop bit 151 to 152 cycles after it (9.44 to 9.5 bit times).
- Feature `_test-hooks` (outside the promise) adds `OneWire::tx_entry`, the address of the transmit entry, which the on-board test needs to run the transmit code on a second state machine.

## Not covered

- `UartRx` has no transmit, no echo, no break detection as an event of its own (a break is one framing error), no parity, 7 or 9 data bits, 2 stop bits or inverted line, no DMA, no count of how many bytes an overrun lost (only how many reads found the flag), no change of baud rate on a built receiver (release and build again), no `release` without the `Common` it was built with.
- Anything but 8N1: no parity, no 7 or 9 data bits, no 2 stop bits, no inverted line.
- Timeouts, retries, protocol, checksums, echo suppression (there is no echo: the receiver is off while the line is driven).
- DMA. A long write is fed by the CPU; a long read is drained by the CPU.
- More than four lines on one `OneWire`, pins that wrap past GPIO31, pins in both GPIOBASE windows at once.
- Changing the baud rate on a built `OneWire` (build it again; `release` and `new` take microseconds).
- Detecting a break as an event of its own: it shows as one framing error.
- Open-drain drive: the line is driven push-pull for the bytes and released to the pad pull-up.
- A real device. The board test was run against a peer built for the test only.
- Host tests of the PIO program itself; it is only run on the board.
- The `release`-while-another-holder compile-fail test: the types are not `Clone` and the parts move by value, but no compile-fail doctest exists yet.

## Tests

Host: `cargo test` (the arithmetic, proptests of the divider, the word packing and the frame decoder, and the examples in this file, which are doctests). `OneWire` does not build on the host; its lints and build are `cargo clippy --lib --target thumbv8m.main-none-eabihf --features embassy-rp-0-10,_test-hooks,defmt,embassy-rp/rp235xb -- -D warnings` and the same with `cargo build`. (`--all-features` on the host cannot build embassy-rp, which needs a chip feature and the ARM target; `--all-targets` on the ARM target cannot build the dev-dependency proptest.)

On the board, `UartRx`: `tools/run.sh uart_rx_loopback 180`. It uses PIO1 and GPIO34 only (PIO1 state machine 0 a dummy neighbour, 1 `UartRx`, 2 the sender, whose one-instruction program `out pins, 1 [15]` with autopull sends any bit pattern at any rate), keeps GPIO36 to 39 as inputs with pad pull-ups and never drives them, and sends nothing to an ESC. The checks: the pad pull-up reads high; the window (`GPIOBASE` 16, JMP pin field 18, in base 18, read back from the registers); `new` refuses a full block and baud 0 and hands the state machine back; 4608 bytes (all 256 values) back to back with one stop bit at -3 %, exact and +3 % with 0 errors; NEGATIVE CONTROLS at -6 % and +6 % (errors must be reported); a read dropped after 5 ms loses nothing; 20 + 20 runt pulses (no byte, counted) and 20 runts each followed by a byte; a control that a pulse of one bit is a start bit (0xFF); a low stop bit (1 framing error, no byte); a line held low 3 ms (1 framing error, nothing after); 20 bytes with nobody reading (Overrun, counted once, the next 5 bytes arrive); 10 times build and release with a 27-instruction filler that must load before every build and at the end (exact instruction accounting) and the neighbour's code, registers and counter unchanged and running; built on a line held low 6 ms (no byte, no framing error, none on the rise); GPIO36 to 39 unchanged at start and end.

On the board, `OneWire`: `tools/run.sh one_wire_loopback 300` (it takes the machine-wide probe lock and keeps its log under `hil/logs/`, which git ignores). The program prints `[PASS]`/`[FAIL]`/`[STAT]`/`[INFO]` lines and `RESULT: PASS|FAIL`; the exit status is the number of failed checks. It touches only GPIO44 to 47 (header J6 rows 11 to 14, nothing fitted), PIO0, PIO1 and PIO2, and never GPIO34 or GPIO36 to 39, the ADC, USB or flash. Before driving anything it shows that each of the four pins follows its pad pull-up and pull-down. The checks:

1. The four spare pins are free.
2. Waveform of a 259-byte write on each line, seen by an observer (a PIO0 state machine counting the high and low runs of the pin at 6.7 ns): every start bit, 8 data bits LSB first and stop bit; bit time within 1 % of 52.08 us; the longest gap between a stop bit and the next start bit below one bit time.
3. Output enable (PIO2 `DBG_PADOE`, polled by the CPU with the cycle counter) is on from the first start bit to the end of the message, off within a quarter bit of the end of the last stop bit, and off on the three other lines at all times.
4. Echo on each of the four lines with message lengths 1, 2, 3, 4, 5, 21 and 259 in each direction (the reply is as long as the message), four reply delays (0, 26 us, 52 us, 1 ms), random bytes: 112 exchanges, the peer receives exactly what is written, `OneWire` reads exactly what the peer sends, the counters agree.
5. Only the selected line carries traffic (the other three receivers see no start bit), with a positive control that a receiver on an unselected line does see traffic sent there.
6. Positive controls for the counters: the line held low for 12 bit times gives one framing error and no byte; 16 bytes with nothing reading give an overrun on the next read, then a fresh echo passes.
7. Break: all four pins read low and all four are driven; afterwards all four are inputs with pull-ups; a second `break_start` and a write during the break are refused; an echo passes afterwards.
8. A pending read dropped after 5 ms and issued again loses nothing (5 bytes sent later all arrive).
9. 100 times `release()`, `OneWire::new` from the returned parts, an echo of 21 bytes; all pins idle after each step.
10. Runt starts (made with the pad's output override, which beats every PIO latch; their width is measured by the observer): 20 low pulses of about a quarter bit (14.9 us) and 20 of about a third of a bit (18.6 us): no byte, no framing error, the runt counter moves by one each; then 20 + 20 of the same, each followed at once by a real byte: the byte arrives alone.
11. Arming after a low line (also with the override): a `read` first, after `break_end` with the line held low 100 us longer, gets nothing and counts no framing error; the same after a write of 100 zero bytes dropped with the line forced low, at 20 different cut points; an echo of 21 bytes passes after each group.
12. Beyond the brief: a write dropped after 3 ms releases the line at once; a CPU stall of 20 ms in the middle of a 100-byte write ends with `Underrun` after all 100 bytes were sent and the next echo passes.

Negative controls of 10 and 11, run on the board with the fix taken out of the program: with `jmp x-- runt` replaced by `nop`, check 10 fails in all four of its groups that read after a runt (the runt is read as a byte); with the receiver armed at `rx` instead of `arm` after a park and a break, check 11 fails in both its groups (a phantom byte arrives).

How the test peer works, because it sets what the test can show: the peer's transmitter is a second state machine of PIO2 that runs `OneWire`'s own transmit code (a `mov pc` into it) behind a front end of 3 instructions (`wait 0 pin 0`, a counting loop, `mov pc, y`; the CPU loads x and y with forced instructions) that waits for our first start bit and counts cycles, so the reply starts a chosen time after our stop bit with no CPU in the path. It has to be in PIO2 because a pad is driven by the block its function select names. Both state machines then share the one output-enable latch per pin that a PIO block has, so the peer's own `set pindirs, 1` (two cycles before its start bit) has to come after our release or the release would undo it: the earliest reply the rig can make begins about 1.3 cycles (4 us) after our stop bit, and it is run with a guard that makes it 6 to 9 us. A reply that begins exactly at the end of the stop bit, or earlier, has not been tested; the argument for the first is rule 9, and the second is contention. The receivers are four PIO1 state machines running the same receive loop as `OneWire`; what they decode is checked against the observer's waveform (check 2), which is independent of that loop. The bit time is measured against the same `clk_sys` that makes it, so the crystal's accuracy is not tested.

## Measured on the MARV V2 Rev 1 board

2026-11-12, release build, `clk_sys` 150 MHz, 19 200 baud, USB attached. The program ran in LOOP-BACK on the spare pins GPIO44 to 47 only, against a test peer written for the test (described under "Tests"), and against NO real device. The final program was run twice in a row after the receiver fixes (start-bit check, wait for high): logs `hil/logs/one_wire_loopback-20261112-151514.log` and `hil/logs/one_wire_loopback-20261112-151805.log` (kept locally, not in git). Both exited 0 with 86 `[PASS]` lines and no `[FAIL]`; the figures below are from the second. (Earlier runs of the program before the fixes, 76 `[PASS]`, are not counted here.)

- Program: 27 instructions (PIO2 slots 3 to 29; the test peer's 3-instruction front end sits in slots 0 to 2; 2 slots free). Divider 488 + 72/256, 0 ppm error, bit time 52 083 ns.
- Waveform of a 259-byte write (observer, 6.7 ns resolution), the four lines: all 259 bytes right on every line (start bit low, 8 data bits LSB first, stop bit high); bit time 52 083 ns (0 ppm against the same clock); shortest run 52 080 ns; longest gap between the end of a stop bit and the next start bit 6 ns (one PIO clock, in the observer's resolution), shortest -6 ns, where one bit is 52 083 ns. 1418, 1442, 1444 and 1438 edges.
- Output enable (21-byte write per line, `DBG_PADOE` and the pad level polled by the CPU, the cycle counter as clock): on from before the first start bit seen to the end of the message with no gap, off 377 to 441 CPU cycles (2.5 to 2.9 us) before the nominal end of the last stop bit, as designed (one PIO cycle, 3.26 us, early); the other three lines' output enable never seen (0x0) on any of the four runs.
- Reply timing the peer made, from the end of our last stop bit to the start bit of the peer's reply, seen by the observer: asked 0 us, 7.7 us; asked 26 us, 33.7 us; asked 52 us, 59.8 us; asked 1 ms, 1007.0 us. The "0 us" row is the closest this rig comes (see "Tests"); the reply and our read were right in all four.
- Echo: 112 exchanges (4 lines x 4 delays x 7 lengths, message and reply of equal length), 4720 bytes each way, 0 failed; `stats()` agreed (4720 sent, 4720 received, 0 framing errors, 0 overruns, 0 underruns).
- Receivers on the unselected lines saw nothing in any of the 112 exchanges; the positive control (three bytes sent on line 1 while line 0 is selected) was seen by receiver 1 and by no other.
- Counters: the line held low for 12 bit times gave 1 framing error and no byte, and the next good byte (0xA5) arrived alone; 16 bytes with nothing reading gave `Error::Overrun` from the next read (overruns 0 to 1), and a fresh echo of 21 bytes then passed.
- Runt starts: the override pulses were 14.9 us (asked 13.0) and 18.5 us (asked 17.4) wide on the pad, against the check at 26.0 us (8 PIO cycles). 20 pulses of each, alone: no byte, no framing error, the runt counter +1 each (0 wrong); 20 of each followed at once by a real byte: the byte arrived alone (0 wrong).
- Arming after a low line: a read first after `break_end` with the line held low 100 us longer: nothing arrived, no framing error, no runt; the same after a write of 100 zero bytes dropped with the line forced low, at 20 cut points (0 wrong); an echo of 21 bytes passed after each group.
- Negative controls, run on the board with the fix removed (logs `-151429` and `-151450`, same folder): without the start-bit check check 10 failed 4 groups of its 6 (the runt was delivered as a byte), without the wait for high check 11 failed both groups (a phantom byte).
- Break: all four pins read low and all four had output enable on 0.5 ms after `break_start`; 0.3 ms after `break_end` all four were inputs with pull-ups on, reading high.
- A read dropped after 5 ms with no data and issued again got all 5 bytes sent 3 ms later.
- 100 x `release()`, `OneWire::new` from the returned parts and an echo of 21 bytes: 0 failed; after every release and every new all four pins were inputs with pull-ups on and nothing driven.
- A write of 100 bytes dropped after 3 ms left the line released and idle at once, and the next echo passed. A 20 ms CPU stall in the middle of a 100-byte write ended with `Error::Underrun` after all 100 bytes and `underruns` 0 to 1; the next echo passed. What was on the wire during the stall was not captured.
- Before anything was driven: each of GPIO44 to 47 read high with the pad pull-up, low with the pull-down, high again with the pull-up (nothing external drives them).
- Not measured: the cost in CPU time of a call or of a poll, behaviour with another task loading the CPU, temperature, other baud rates, a low pulse of half a bit or more (rule 10b), a reply that starts at or before the end of our stop bit, and anything on GPIO34 or GPIO36 to 39 (not touched).

## Measured, `UartRx`, on the MARV V2 Rev 1 board

2026-11-12, release build, `clk_sys` 150 MHz, 115 200 baud, USB attached, GPIO34 only, loop-back against a sender written for the test, NO real device: no real ESC telemetry has been received. The final program ran twice in a row, both exit 0, 0 `[FAIL]`: logs `hil/logs/uart_rx_loopback-20261112-175711.log` and `hil/logs/uart_rx_loopback-20261112-175716.log` (kept locally, not in git; the figures are the same in both except where the second is named). Earlier runs of earlier versions of the program: `-174304` stopped at its first check (GPIO34 did not read low with the pad pull-down; see below), `-174858` failed the runt-then-byte check (see rule U7); neither is counted.

- Before anything was driven, GPIO34 read high with the pad pull-up, AND read high with the pad pull-down: on this board the line is held high from outside (the ESC's telemetry output idles high and reaches the pad through 1 k); this is why the pull-down is only reported. The loop-back drives the line push-pull from a second state machine through that same 1 k resistor.
- Program 13 instructions; divider 81 + 97/256, receiver +16 ppm; build and release leave the neighbour's 4 instructions, its registers and its counter unchanged and running (20 looks of 20).
- Window: `GPIOBASE` 16, JMP pin field 18, in base 18 for GPIO34 (read back from PIO1 registers).
- Tolerance, 4608 bytes back to back each, 256 values: sender -3 % (111 744 baud, -30 031 ppm against the receiver's actual rate): 4608 received, 0 wrong, 0 framing, 0 runts, 0 overruns; exact: the same; +3 % (118 656 baud, +29 959 ppm): the same.
- NEGATIVE CONTROLS: sender -6 % (-60 010 ppm): 2333 of 4608 received, 2326 to 2328 of them wrong (the count moves by a byte or two from run to run), 2275 framing errors; sender +6 % (+59 987 ppm): 1126 received, 1117 to 1119 wrong, 2370 framing errors. Both reported errors, so the check can fail; the control also requires that the sender ran (its FIFO was kept fed and bytes arrived).
- Runts: 20 pulses of about 3 and 20 of about 5 PIO cycles asked (250 and 400 CPU cycles; the width on the pad was NOT measured): 0 bytes, 0 framing errors, 20 runts counted each; 20 runts each followed about one bit later by a byte: 20 bytes right, 20 runts. Control: a low pulse of about one bit time gave one byte 0xFF and no runt.
- A low stop bit: 1 framing error, no byte, the next two bytes arrived; a line held low 3 ms: 1 framing error, no byte, then a good byte.
- Overrun: 20 bytes into the 8-word FIFO with nobody reading: the next read returned `Overrun`, `overruns` 1 (a flag count, not a byte count), then 5 fresh bytes arrived.
- A read dropped after 5 ms with no data lost nothing (5 bytes sent later all arrived).
- 10 x build and release: 10 built, the 27 free instructions free before every build and at the end, a byte passed each time. Built on a line held low for 6 ms: no byte, no framing error, none on the rise, the next byte arrived.
- GPIO36 to 39: inputs with pad pull-ups, owned by SIO, no output enable, no override, at the start and at the end.
- Not measured: any real ESC telemetry or any real device; the width of the runt pulses on the pad; other baud rates; temperature; a CPU load on the executor (overruns were made on purpose, not by load); the tolerance limits of `timing::uart_rx_tolerance_ppm` themselves (only ±3 % and ±6 %); a sender with more than one stop bit or with gaps; the cost in CPU time of a read; the fix-removed negative controls for the wait for high and the runt check (not run for `UartRx`); the break and low-stop checks with a receiver whose `wait 1` is removed.

## Features, layout, licence

Features, all off by default and additive: `defmt` (`defmt::Format` for the public types), `embassy-rp-0-10` (`OneWire` and `UartRx`, with `embassy-rp` 0.10 and `embedded-io-async` 0.7; the chip feature comes from the application), `_test-hooks` (see above).

```text
Cargo.toml   the crate
src/         lib.rs, timing.rs (the core), one_wire.rs (OneWire and its PIO program), uart_rx.rs (UartRx and its PIO program)
hil/         on-board programs one_wire_loopback and uart_rx_loopback (own workspace; src/bin/, logs/ is git-ignored)
tools/       run.sh: builds a hil/ program, flashes and runs it under the machine-wide probe lock
LICENSE      MIT
```
