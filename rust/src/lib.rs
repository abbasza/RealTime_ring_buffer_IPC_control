//! Library half of this crate: shared, unit-tested building blocks used (or
//! usable) by the demo binaries in `src/bin/`.
//!
//! `src/bin/*.rs` demonstrate the *transport* comparison (lock-free vs
//! lock-based IPC). `canfd` below demonstrates the *wire framing* layer one
//! level below that: how a joint-space setpoint actually gets packed into
//! the bytes that go out over CAN-FD to the Joint MCU tier. It's real,
//! `#![no_std]`-compatible, unit-tested logic — not a register-level driver
//! (this repo has no physical CAN transceiver to drive), so the README and
//! interview notes are explicit about that boundary: this is the framing
//! layer a real CAN-FD driver's `transmit()` call would sit underneath.
pub mod canfd;
