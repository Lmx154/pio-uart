//! Byte transports on PIO for the RP2350. They carry bytes and know no protocol.
//!
//! The core (always built, no dependency) is the arithmetic: the PIO clock divider for a baud
//! rate and its error in ppm, bit and frame times, and how bytes are packed into FIFO words
//! ([`timing`]). With the feature `embassy-rp-0-10` the crate adds [`OneWire`], a half-duplex
//! 8N1 serial line on one selected pin of up to four consecutive pins, built from the PIO block's
//! `Common`, one `StateMachine` and the pins, all taken by value and handed back by
//! [`OneWire::release`].
//!
//! Examples: see README.md, whose examples are compiled by `cargo test`.
#![no_std]
#![cfg_attr(not(feature = "embassy-rp-0-10"), forbid(unsafe_code))]
#![cfg_attr(feature = "embassy-rp-0-10", deny(unsafe_code))]
#![warn(missing_docs)]
#![cfg_attr(
    not(test),
    deny(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::unreachable,
        clippy::todo,
        clippy::unimplemented,
        clippy::indexing_slicing,
        clippy::undocumented_unsafe_blocks
    )
)]

#[cfg(test)]
extern crate std;

#[cfg(feature = "embassy-rp-0-10")]
mod one_wire;
pub mod timing;

#[cfg(feature = "embassy-rp-0-10")]
pub use one_wire::{BuildError, Config, Error, NewError, OneWire, Parts, Stats};

#[cfg(doctest)]
#[doc = include_str!("../README.md")]
pub struct ReadmeDoctests;
