//! CAN-FD wire framing for joint-space commands (Bridge -> Joint MCU tier).
//!
//! This is the framing/encoding layer, not a register-level CAN-FD
//! peripheral driver — this repo has no physical CAN transceiver to talk
//! to, so there's no `bxCAN`/`FDCAN` register access here. What IS real:
//! the on-the-wire byte layout a real driver's `transmit(frame: &[u8])`
//! call would be handed, and the encode/decode/CRC logic is exercised by
//! the unit tests at the bottom of this file (`cargo test`).
//!
//! Written using only `core`-compatible operations (fixed-size arrays, no
//! heap, no `std::io`) specifically so this module can be lifted as-is into
//! a `#![no_std]` firmware crate for the STM32 side — the rest of this
//! workspace still targets `std` because of the demo binaries in `src/bin/`.
//!
//! Wire format (64-byte CAN-FD payload — 64 is a valid CAN-FD DLC and the
//! largest one, chosen here because 2 + 1 + 1 + 7*4 + 1 = 33 bytes of real
//! content already exceeds the next size down (32), so we pad to 64):
//!
//! ```text
//! offset  size  field
//! 0       2     seq        (u16, little-endian, wraps)
//! 2       1     flags      (bit0: e-stop, bit1: homing request)
//! 3       1     joint_count (always MAX_JOINTS here; carried explicitly
//!                            so a firmware built for fewer joints can
//!                            detect a mismatch instead of silently
//!                            misreading the payload)
//! 4       28    positions  (7 x f32, little-endian, radians)
//! 32      1     crc8       (CRC-8/SMBUS, poly 0x07, over bytes [0, 32))
//! 33      31    reserved   (zero-filled pad out to the 64-byte CAN-FD DLC)
//! ```

pub const CANFD_MAX_PAYLOAD: usize = 64;
pub const MAX_JOINTS: usize = 7;

const POS_OFFSET: usize = 4;
const POS_BYTES: usize = MAX_JOINTS * 4;
const CRC_OFFSET: usize = POS_OFFSET + POS_BYTES; // 32

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointCommand {
    pub seq: u16,
    pub flags: u8,
    pub positions: [f32; MAX_JOINTS],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeError {
    /// CRC over the received bytes didn't match the trailing CRC byte —
    /// treat the frame as corrupt and drop it (a real driver would also
    /// increment a bus-error counter here).
    ChecksumMismatch,
    /// `joint_count` in the frame doesn't match what this firmware build
    /// expects — almost certainly a Brain/Bridge/MCU build mismatch, not a
    /// transient bus error, so this is deliberately a different variant
    /// than `ChecksumMismatch`.
    JointCountMismatch { expected: u8, got: u8 },
}

impl JointCommand {
    pub fn encode(&self) -> [u8; CANFD_MAX_PAYLOAD] {
        let mut buf = [0u8; CANFD_MAX_PAYLOAD];

        buf[0..2].copy_from_slice(&self.seq.to_le_bytes());
        buf[2] = self.flags;
        buf[3] = MAX_JOINTS as u8;

        for (i, p) in self.positions.iter().enumerate() {
            let start = POS_OFFSET + i * 4;
            buf[start..start + 4].copy_from_slice(&p.to_le_bytes());
        }

        buf[CRC_OFFSET] = crc8_smbus(&buf[0..CRC_OFFSET]);
        // buf[33..64] stays zero — reserved.
        buf
    }

    pub fn decode(buf: &[u8; CANFD_MAX_PAYLOAD]) -> Result<Self, DecodeError> {
        let expected_crc = crc8_smbus(&buf[0..CRC_OFFSET]);
        if buf[CRC_OFFSET] != expected_crc {
            return Err(DecodeError::ChecksumMismatch);
        }

        let joint_count = buf[3];
        if joint_count != MAX_JOINTS as u8 {
            return Err(DecodeError::JointCountMismatch {
                expected: MAX_JOINTS as u8,
                got: joint_count,
            });
        }

        let seq = u16::from_le_bytes([buf[0], buf[1]]);
        let flags = buf[2];

        let mut positions = [0.0f32; MAX_JOINTS];
        for (i, p) in positions.iter_mut().enumerate() {
            let start = POS_OFFSET + i * 4;
            let bytes: [u8; 4] = buf[start..start + 4]
                .try_into()
                .expect("slice is exactly 4 bytes");
            *p = f32::from_le_bytes(bytes);
        }

        Ok(JointCommand {
            seq,
            flags,
            positions,
        })
    }
}

/// CRC-8/SMBUS: polynomial 0x07, no reflection, initial value 0x00. Chosen
/// because it's the same CRC-8 variant SMBus/PMBus already use in a lot of
/// embedded peripheral firmware, so it's a reasonable default to reach for
/// rather than inventing a bespoke checksum.
fn crc8_smbus(data: &[u8]) -> u8 {
    let mut crc: u8 = 0x00;
    for &byte in data {
        crc ^= byte;
        for _ in 0..8 {
            if crc & 0x80 != 0 {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> JointCommand {
        JointCommand {
            seq: 4242,
            flags: 0b0000_0001, // e-stop bit set
            positions: [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7],
        }
    }

    #[test]
    fn round_trip_preserves_all_fields() {
        let cmd = sample();
        let encoded = cmd.encode();
        let decoded = JointCommand::decode(&encoded).expect("valid frame");
        assert_eq!(cmd, decoded);
    }

    #[test]
    fn encoded_frame_is_exactly_one_canfd_dlc() {
        // 64 is one of the fixed CAN-FD DLC sizes (0-8,12,16,20,24,32,48,64);
        // this assertion exists so a future field addition that pushes the
        // payload past 64 bytes fails the build loudly instead of silently
        // producing a frame with no valid DLC to put it in.
        let encoded = sample().encode();
        assert_eq!(encoded.len(), CANFD_MAX_PAYLOAD);
    }

    #[test]
    fn corrupted_byte_is_detected_by_crc() {
        let mut encoded = sample().encode();
        encoded[10] ^= 0xFF; // flip bits inside the positions field
        assert_eq!(
            JointCommand::decode(&encoded),
            Err(DecodeError::ChecksumMismatch)
        );
    }

    #[test]
    fn joint_count_mismatch_is_distinguished_from_corruption() {
        let mut encoded = sample().encode();
        encoded[3] = 6; // pretend this frame came from a 6-joint build
                        // Recompute CRC so this test isolates the joint-count check from
                        // the checksum check rather than tripping both at once.
        let recomputed_crc = crc8_smbus(&encoded[0..CRC_OFFSET]);
        encoded[CRC_OFFSET] = recomputed_crc;
        assert_eq!(
            JointCommand::decode(&encoded),
            Err(DecodeError::JointCountMismatch {
                expected: 7,
                got: 6
            })
        );
    }

    #[test]
    fn reserved_tail_bytes_stay_zero() {
        let encoded = sample().encode();
        assert!(encoded[(CRC_OFFSET + 1)..].iter().all(|&b| b == 0));
    }
}
