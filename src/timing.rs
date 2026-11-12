//! Divider and timing arithmetic of the serial line, and the packing of bytes into FIFO words.
//!
//! Pure functions, no I/O. One serial bit takes [`CYCLES_PER_BIT`] PIO clock cycles, so the PIO
//! clock divider is `clk_sys / (baud * 16)`, in 24.8 fixed point.

/// PIO clock cycles per serial bit.
pub const CYCLES_PER_BIT: u32 = 16;
/// Bits on the wire per byte: start, 8 data, stop.
pub const BITS_PER_BYTE: u32 = 10;
/// Bytes in one FIFO word of a message to send.
pub const BYTES_PER_WORD: usize = 4;
/// Depth of the TX FIFO in words (the FIFOs are not joined).
pub const TX_FIFO_WORDS: usize = 4;
/// Depth of the RX FIFO in words; each received byte takes one.
pub const RX_FIFO_WORDS: usize = 4;
/// Most lines (pins) one `OneWire` serves.
pub const MAX_LINES: usize = 4;
/// Depth of the RX FIFO in words when it is joined with the unused TX FIFO, as `UartRx` does.
pub const RX_JOINED_FIFO_WORDS: usize = 8;
/// The PIO cycle, counted from the one in which the receiver's `wait 0` sees the falling edge, at
/// which `UartRx` looks at the start bit again. The same offset, plus 16 per bit, is where it
/// samples every later bit.
pub const UART_RX_SAMPLE_BASE_CYCLE: u32 = 7;
/// The PIO cycle, counted the same way, in which the `UartRx` receiver is waiting for the next
/// start bit again after a good frame (stop-bit sample, push, `wait 1`, then `wait 0`).
pub const UART_RX_REARM_CYCLE: u32 = 154;
/// Most pins of the chip: GPIO0 to GPIO47.
pub const PIN_COUNT: u8 = 48;
/// The word the program pushes into the RX FIFO when the last stop bit of a write has ended.
pub const TX_DONE_WORD: u32 = 0xFFFF_FFFF;
/// The word the program pushes into the RX FIFO when a start bit was gone again at its middle.
pub const RUNT_WORD: u32 = 0x7FFF_FFFF;

/// A PIO clock divider in 24.8 fixed point, and how far it is from the wanted baud rate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Divider {
    /// The divider times 256: bits 31..8 are the integer part, bits 7..0 the fraction.
    pub q8: u32,
    /// Actual baud rate minus wanted, divided by wanted, in parts per million (negative: slow).
    pub error_ppm: i32,
}

impl Divider {
    /// The integer part of the divider.
    pub const fn integer(self) -> u32 {
        self.q8 >> 8
    }

    /// The fraction of the divider in 1/256.
    pub const fn fraction(self) -> u8 {
        (self.q8 & 0xFF) as u8
    }
}

/// The PIO clock divider for `baud` with a system clock of `clk_sys_hz`: `clk_sys / (baud * 16)`
/// rounded to the nearest 1/256, with its error. `None` when `baud` is 0 or the divider is
/// outside 1.0 to 65536.0, the range of the hardware.
///
/// ```
/// use pio_uart::timing::divider;
/// let d = divider(150_000_000, 19_200).unwrap();
/// assert_eq!((d.integer(), d.fraction(), d.error_ppm), (488, 72, 0));
/// ```
pub const fn divider(clk_sys_hz: u32, baud: u32) -> Option<Divider> {
    if baud == 0 {
        return None;
    }
    let num = clk_sys_hz as u64 * CYCLES_PER_BIT as u64;
    let baud64 = baud as u64;
    let q8 = (num + baud64 / 2) / baud64;
    if q8 < 256 || q8 > 65_536 * 256 {
        return None;
    }
    let actual_num = q8 * baud64;
    let diff = num as i64 - actual_num as i64;
    let ppm = diff * 1_000_000 / actual_num as i64;
    Some(Divider {
        q8: q8 as u32,
        error_ppm: ppm as i32,
    })
}

/// One bit time in nanoseconds, rounded to nearest. `None` when `baud` is 0.
pub const fn bit_time_ns(baud: u32) -> Option<u32> {
    if baud == 0 {
        return None;
    }
    let baud = baud as u64;
    Some(((1_000_000_000 + baud / 2) / baud) as u32)
}

/// The time of one byte on the wire (10 bits) in nanoseconds, rounded to nearest. `None` when
/// `baud` is 0.
pub const fn byte_time_ns(baud: u32) -> Option<u32> {
    if baud == 0 {
        return None;
    }
    let baud = baud as u64;
    Some(((BITS_PER_BYTE as u64 * 1_000_000_000 + baud / 2) / baud) as u32)
}

/// The time the CPU has to refill the TX FIFO once a slot is free, in nanoseconds: the other
/// `TX_FIFO_WORDS - 1` words already in the FIFO last that long. Longer than this and a write
/// stalls at a word boundary (a gap before a start bit, never a damaged bit). At 19 200 baud it
/// is 6.25 ms. `None` when `baud` is 0.
pub const fn tx_refill_window_ns(baud: u32) -> Option<u64> {
    if baud == 0 {
        return None;
    }
    let bits = (TX_FIFO_WORDS as u64 - 1) * BYTES_PER_WORD as u64 * BITS_PER_BYTE as u64;
    Some((bits * 1_000_000_000 + baud as u64 / 2) / baud as u64)
}

/// The time the CPU has to take a received byte out of the RX FIFO before the next one is
/// lost, in nanoseconds: `RX_FIFO_WORDS` byte times. `None` when `baud` is 0.
pub const fn rx_drain_window_ns(baud: u32) -> Option<u64> {
    if baud == 0 {
        return None;
    }
    let bits = RX_FIFO_WORDS as u64 * BITS_PER_BYTE as u64;
    Some((bits * 1_000_000_000 + baud as u64 / 2) / baud as u64)
}

/// The time the CPU has to take received bytes out of the joined RX FIFO of `UartRx` before the
/// next one is lost, in nanoseconds: `RX_JOINED_FIFO_WORDS` byte times. `None` when `baud` is 0.
pub const fn uart_rx_drain_window_ns(baud: u32) -> Option<u64> {
    if baud == 0 {
        return None;
    }
    let bits = RX_JOINED_FIFO_WORDS as u64 * BITS_PER_BYTE as u64;
    Some((bits * 1_000_000_000 + baud as u64 / 2) / baud as u64)
}

/// The PIO cycle, counted from the cycle in which the receiver sees the falling edge, at which
/// `UartRx` looks at bit `bit`: 0 is the start bit (the runt check), 1 to 8 are data bits 0 to 7,
/// 9 is the stop bit. 16 cycles apart, the first at [`UART_RX_SAMPLE_BASE_CYCLE`].
pub const fn uart_rx_sample_cycle(bit: u32) -> u32 {
    CYCLES_PER_BIT * bit + UART_RX_SAMPLE_BASE_CYCLE
}

/// How far the sender's baud rate may differ from the receiver's, in ppm of the receiver's:
/// `(low, high)`, `low` negative (slower), `high` positive (faster).
///
/// The model: the receiver sees the edge between 0 and 1 cycle late, then samples bit `n` at cycle
/// [`uart_rx_sample_cycle`]. A sample is right when it lies inside the sender's bit. A sender that
/// is too slow fails first at the stop bit (earliest possible sample against the start of that
/// bit); one that is too fast fails first when the next start bit arrives before the receiver is
/// waiting for it again ([`UART_RX_REARM_CYCLE`]) in back-to-back bytes, or at the stop bit,
/// whichever is closer. Both limits are rounded towards the middle, so a rate inside them is
/// inside the model. A rate outside them is not necessarily wrong in practice: this is
/// arithmetic, the board decides.
pub const fn uart_rx_tolerance_ppm() -> (i32, i32) {
    let c = CYCLES_PER_BIT as u64;
    let mut fast = 1_000_000 * c * 10 / UART_RX_REARM_CYCLE as u64;
    let mut slow = 0u64;
    let mut n = 1;
    while n <= 9 {
        let sample = uart_rx_sample_cycle(n) as u64;
        let f = 1_000_000 * c * (n as u64 + 1) / (sample + 1);
        if f < fast {
            fast = f;
        }
        let s = (1_000_000 * c * n as u64).div_ceil(sample);
        if s > slow {
            slow = s;
        }
        n += 1;
    }
    (slow as i32 - 1_000_000, fast as i32 - 1_000_000)
}

/// The GPIOBASE (0 or 16) a PIO block must have for a state machine to reach GPIO `pin`, by the
/// rule embassy-rp 0.10 applies when it writes a state machine's configuration: pins below 32 use
/// 0, pins 32 to 47 use 16. `None` above GPIO47.
pub const fn gpiobase_for_pin(pin: u8) -> Option<u8> {
    if pin < 32 {
        Some(0)
    } else if pin < PIN_COUNT {
        Some(16)
    } else {
        None
    }
}

/// The value a PIO pin field (in base, JMP pin) holds for GPIO `pin`: the number counted from
/// [`gpiobase_for_pin`]. `None` above GPIO47.
pub const fn pin_field(pin: u8) -> Option<u8> {
    match gpiobase_for_pin(pin) {
        Some(base) => Some(pin - base),
        None => None,
    }
}

/// PIO clock cycles in `ns` nanoseconds at the given divider, rounded to nearest.
pub const fn cycles_for_ns(clk_sys_hz: u32, divider: Divider, ns: u64) -> u64 {
    let num = ns as u128 * clk_sys_hz as u128 * 256;
    let den = 1_000_000_000u128 * divider.q8 as u128;
    ((num + den / 2) / den) as u64
}

/// The first word of a message to send: the number of bytes minus one. `None` for no bytes or
/// more than 2^32.
pub const fn count_word(bytes: usize) -> Option<u32> {
    if bytes == 0 {
        return None;
    }
    let n = (bytes - 1) as u64;
    if n > u32::MAX as u64 {
        None
    } else {
        Some(n as u32)
    }
}

/// Up to four bytes as one FIFO word: the first byte in bits 7..0, the missing ones 0. Bytes
/// beyond the fourth are ignored.
pub fn pack_word(chunk: &[u8]) -> u32 {
    chunk
        .iter()
        .take(BYTES_PER_WORD)
        .enumerate()
        .fold(0, |word, (i, byte)| word | (*byte as u32) << (8 * i))
}

/// The number of FIFO words a message of `bytes` bytes takes: the count word and one word per
/// four bytes.
pub const fn word_count(bytes: usize) -> usize {
    1 + bytes.div_ceil(BYTES_PER_WORD)
}

/// The FIFO words of a message in the order the program takes them: [`count_word`], then the
/// bytes four at a time with [`pack_word`]. `None` for an empty message.
pub fn fifo_image(bytes: &[u8]) -> Option<impl Iterator<Item = u32> + '_> {
    let count = count_word(bytes.len())?;
    Some(core::iter::once(count).chain(bytes.chunks(BYTES_PER_WORD).map(pack_word)))
}

/// What one word taken from the RX FIFO is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RxWord {
    /// A byte with a high stop bit.
    Byte(u8),
    /// A byte whose stop bit was low: it is not data.
    FramingError,
    /// The last stop bit of a write has ended and the line is released.
    TxDone,
    /// A low pulse that was already gone at the middle of a start bit: not a start, no byte.
    Runt,
}

/// Decodes a word of the RX FIFO: the received frame is in bits 31..23 (stop bit, then data bit 7
/// down to data bit 0), the other bits are 0; [`TX_DONE_WORD`] is the end-of-write marker and
/// [`RUNT_WORD`] the marker of a runt start.
pub const fn decode_rx_word(word: u32) -> RxWord {
    if word == TX_DONE_WORD {
        RxWord::TxDone
    } else if word == RUNT_WORD {
        RxWord::Runt
    } else if word >> 31 == 0 {
        RxWord::FramingError
    } else {
        RxWord::Byte((word >> 23) as u8)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;
    use std::vec::Vec;

    #[test]
    fn divider_19200_at_150_mhz_is_exact() {
        let d = divider(150_000_000, 19_200).unwrap();
        assert_eq!(d.q8, 125_000);
        assert_eq!((d.integer(), d.fraction()), (488, 72));
        assert_eq!(d.error_ppm, 0);
    }

    #[test]
    fn divider_115200_at_150_mhz_error_is_small() {
        let d = divider(150_000_000, 115_200).unwrap();
        assert_eq!((d.integer(), d.fraction()), (81, 97));
        assert!(d.error_ppm.abs() < 100, "{}", d.error_ppm);
    }

    #[test]
    fn divider_rejects_out_of_range() {
        assert_eq!(divider(150_000_000, 0), None);
        assert_eq!(divider(150_000_000, 10_000_000), None);
        assert_eq!(divider(150_000_000, 1), None);
        assert!(divider(150_000_000, 9_000_000).is_some());
        assert!(divider(150_000_000, 150).is_some());
        assert_eq!(divider(150_000_000, 36), None);
    }

    #[test]
    fn divider_error_has_the_sign_of_the_actual_rate() {
        // 12 MHz * 16 / 9600 = 20000 exactly. A little faster rounds down (the actual rate is
        // above the wanted one); a little slower rounds up.
        let d = divider(12_000_100, 9_600).unwrap();
        assert_eq!(d.q8, 20_000);
        assert!(d.error_ppm > 0);
        let d = divider(12_000_300, 9_600).unwrap();
        assert_eq!(d.q8, 20_001);
        assert!(d.error_ppm < 0);
    }

    #[test]
    fn bit_and_byte_time() {
        assert_eq!(bit_time_ns(19_200), Some(52_083));
        assert_eq!(byte_time_ns(19_200), Some(520_833));
        assert_eq!(bit_time_ns(0), None);
        assert_eq!(byte_time_ns(0), None);
    }

    #[test]
    fn refill_and_drain_windows_at_19200() {
        assert_eq!(tx_refill_window_ns(19_200), Some(6_250_000));
        assert_eq!(rx_drain_window_ns(19_200), Some(2_083_333));
        assert_eq!(tx_refill_window_ns(0), None);
        assert_eq!(rx_drain_window_ns(0), None);
    }

    #[test]
    fn cycles_for_ns_matches_the_bit_grid() {
        let d = divider(150_000_000, 19_200).unwrap();
        assert_eq!(cycles_for_ns(150_000_000, d, 52_083), 16);
        assert_eq!(cycles_for_ns(150_000_000, d, 1_000_000), 307);
        assert_eq!(cycles_for_ns(150_000_000, d, 0), 0);
    }

    #[test]
    fn count_word_is_bytes_minus_one() {
        assert_eq!(count_word(0), None);
        assert_eq!(count_word(1), Some(0));
        assert_eq!(count_word(259), Some(258));
        assert_eq!(count_word(u32::MAX as usize), Some(u32::MAX - 1));
        #[cfg(target_pointer_width = "64")]
        {
            assert_eq!(count_word(u32::MAX as usize + 1), Some(u32::MAX));
            assert_eq!(count_word(u32::MAX as usize + 2), None);
        }
    }

    #[test]
    fn pack_word_puts_the_first_byte_in_the_low_bits() {
        assert_eq!(pack_word(&[]), 0);
        assert_eq!(pack_word(&[0xAB]), 0xAB);
        assert_eq!(pack_word(&[1, 2]), 0x0201);
        assert_eq!(pack_word(&[1, 2, 3, 4]), 0x0403_0201);
        assert_eq!(pack_word(&[1, 2, 3, 4, 5]), 0x0403_0201);
    }

    #[test]
    fn word_count_and_fifo_image_agree() {
        for n in 1..=40usize {
            let bytes: Vec<u8> = (0..n as u8).collect();
            let image: Vec<u32> = fifo_image(&bytes).unwrap().collect();
            assert_eq!(image.len(), word_count(n));
            assert_eq!(image[0], n as u32 - 1);
        }
        assert!(fifo_image(&[]).is_none());
        assert_eq!(word_count(1), 2);
        assert_eq!(word_count(4), 2);
        assert_eq!(word_count(5), 3);
        let image: Vec<u32> = fifo_image(&[1, 2, 3, 4, 5]).unwrap().collect();
        assert_eq!(image, [4, 0x0403_0201, 5]);
    }

    #[test]
    fn decode_rx_word_cases() {
        assert_eq!(decode_rx_word(0xFF80_0000), RxWord::Byte(0xFF));
        assert_eq!(decode_rx_word(0x8000_0000), RxWord::Byte(0x00));
        assert_eq!(decode_rx_word(0xA5u32 << 23 | 1 << 31), RxWord::Byte(0xA5));
        assert_eq!(decode_rx_word(0), RxWord::FramingError);
        assert_eq!(decode_rx_word(0x7F80_0000), RxWord::FramingError);
        assert_eq!(decode_rx_word(TX_DONE_WORD), RxWord::TxDone);
        assert_eq!(decode_rx_word(RUNT_WORD), RxWord::Runt);
        // a frame with a low stop bit and data 0xFF is not the runt marker
        assert_eq!(decode_rx_word(0x7F80_0000), RxWord::FramingError);
    }

    #[test]
    fn integer_and_fraction_split_the_divider() {
        let d = Divider {
            q8: 0x0003_0280,
            error_ppm: 0,
        };
        assert_eq!((d.integer(), d.fraction()), (0x302, 0x80));
    }

    #[test]
    fn uart_rx_sample_cycles_are_16_apart_from_7() {
        assert_eq!(uart_rx_sample_cycle(0), 7);
        assert_eq!(uart_rx_sample_cycle(1), 23);
        assert_eq!(uart_rx_sample_cycle(8), 135);
        assert_eq!(uart_rx_sample_cycle(9), 151);
        // the stop bit is sampled before the receiver is re-armed
        assert!(uart_rx_sample_cycle(9) < UART_RX_REARM_CYCLE);
    }

    #[test]
    fn uart_rx_tolerance_numbers() {
        // slow: bit 9 starts at 9 sender bits and is sampled at cycle 151: 144 / 151
        // fast: the re-arm at cycle 154 against 10 sender bits (160 cycles): 160 / 154
        assert_eq!(uart_rx_tolerance_ppm(), (-46_357, 38_961));
    }

    /// An independent model of the tolerance: every sample, with the edge seen 0 to 1 cycle late,
    /// lies in the sender's bit, and the next start bit comes after the receiver re-arms.
    fn model_ok(ratio: f64) -> bool {
        // ratio is the sender baud over the receiver baud; one sender bit lasts 16 / ratio cycles
        let bit = 16.0 / ratio;
        for late_q in 0..=8 {
            let late = late_q as f64 / 8.0;
            for n in 1..=9u32 {
                let t = uart_rx_sample_cycle(n) as f64 + late;
                if t < n as f64 * bit || t > (n + 1) as f64 * bit {
                    return false;
                }
            }
        }
        10.0 * bit >= UART_RX_REARM_CYCLE as f64
    }

    #[test]
    fn uart_rx_tolerance_matches_the_model_and_covers_3_percent() {
        let (low, high) = uart_rx_tolerance_ppm();
        assert!(model_ok(1.0 + low as f64 / 1e6));
        assert!(model_ok(1.0 + high as f64 / 1e6));
        assert!(!model_ok(1.0 + (low - 1_000) as f64 / 1e6));
        assert!(!model_ok(1.0 + (high + 1_000) as f64 / 1e6));
        for percent in [-3.0, 0.0, 3.0] {
            assert!(model_ok(1.0 + percent / 100.0), "{percent}");
        }
        for percent in [-6.0, 6.0] {
            assert!(!model_ok(1.0 + percent / 100.0), "{percent}");
        }
    }

    #[test]
    fn uart_rx_drain_window_is_eight_bytes() {
        assert_eq!(uart_rx_drain_window_ns(115_200), Some(694_444));
        assert_eq!(uart_rx_drain_window_ns(0), None);
    }

    #[test]
    fn gpiobase_and_pin_field_cases() {
        assert_eq!(gpiobase_for_pin(0), Some(0));
        assert_eq!(gpiobase_for_pin(31), Some(0));
        assert_eq!(gpiobase_for_pin(32), Some(16));
        assert_eq!(gpiobase_for_pin(34), Some(16));
        assert_eq!(gpiobase_for_pin(47), Some(16));
        assert_eq!(gpiobase_for_pin(48), None);
        assert_eq!(pin_field(5), Some(5));
        assert_eq!(pin_field(31), Some(31));
        assert_eq!(pin_field(32), Some(16));
        assert_eq!(pin_field(34), Some(18));
        assert_eq!(pin_field(47), Some(31));
        assert_eq!(pin_field(48), None);
    }

    proptest! {
        #[test]
        fn pin_field_fits_five_bits_and_adds_up(pin in 0u8..48) {
            let base = gpiobase_for_pin(pin).unwrap();
            let field = pin_field(pin).unwrap();
            prop_assert!(field < 32);
            prop_assert_eq!(base + field, pin);
        }

        #[test]
        fn divider_error_is_within_half_a_step(clk in 1_000_000u32..400_000_000, baud in 300u32..4_000_000) {
            if let Some(d) = divider(clk, baud) {
                // rounding to 1/256 of a divider that is at least 1.0 is at most 0.5/256 of it
                let bound = 1_000_000.0 * 0.5 / d.q8 as f64 + 1.0;
                prop_assert!((d.error_ppm as f64).abs() <= bound);
                let exact = clk as f64 * 16.0 / baud as f64 / 256.0 * 256.0;
                prop_assert!((d.q8 as f64 - exact).abs() <= 0.5 + 1e-6);
            }
        }

        #[test]
        fn rx_word_round_trips_a_frame(byte in any::<u8>(), stop in any::<bool>()) {
            let word = (stop as u32) << 31 | (byte as u32) << 23;
            let want = if stop { RxWord::Byte(byte) } else { RxWord::FramingError };
            prop_assert_eq!(decode_rx_word(word), want);
        }

        #[test]
        fn fifo_image_carries_every_byte_in_order(bytes in proptest::collection::vec(any::<u8>(), 1..300)) {
            let image: Vec<u32> = fifo_image(&bytes).unwrap().collect();
            prop_assert_eq!(image[0] as usize, bytes.len() - 1);
            let mut out = Vec::new();
            for w in &image[1..] {
                out.extend_from_slice(&w.to_le_bytes());
            }
            out.truncate(bytes.len());
            prop_assert_eq!(out, bytes);
        }
    }
}
