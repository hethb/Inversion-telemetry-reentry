//! Telemetry wire protocol shared by the flight simulator and the ground station.
//!
//! Frames follow the shape of a CCSDS Space Packet (CCSDS 133.0-B):
//!
//! ```text
//! +----------------- primary header (6 B, big-endian) -----------------+
//! | ver:3 | type:1 | sec_hdr:1 | apid:11 | seq_flags:2 | seq_count:14  |
//! | data_length:16  (= len(secondary header + payload + crc) - 1)      |
//! +--------------------------------------------------------------------+
//! | secondary header: mission elapsed time, u64 microseconds (8 B)     |
//! | payload: fixed layout per APID, big-endian (see `APIDS`)           |
//! | CRC-16/CCITT-FALSE over everything above (2 B)                     |
//! +--------------------------------------------------------------------+
//! ```
//!
//! Every payload layout is described by a static decommutation table
//! (`CHANNELS`), so the ground station decodes any packet generically and a
//! new sensor is a one-line table change.

use serde::Serialize;

pub const VERSION: u8 = 0;
pub const PRIMARY_HEADER_LEN: usize = 6;
pub const SECONDARY_HEADER_LEN: usize = 8;
pub const CRC_LEN: usize = 2;
pub const SEQ_MODULO: u16 = 1 << 14;

/// Application process identifiers: one per telemetry packet type.
pub mod apid {
    pub const NAV: u16 = 0x010;
    pub const THERMAL: u16 = 0x020;
    pub const POWER: u16 = 0x030;
    pub const STATUS: u16 = 0x040;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub enum FieldType {
    F32,
    U8,
}

impl FieldType {
    pub const fn size(self) -> usize {
        match self {
            FieldType::F32 => 4,
            FieldType::U8 => 1,
        }
    }
}

/// One decommutated measurement inside a packet payload.
#[derive(Debug, Clone, Copy, Serialize)]
pub struct Channel {
    pub id: u16,
    pub name: &'static str,
    pub label: &'static str,
    pub unit: &'static str,
    pub apid: u16,
    pub offset: usize,
    pub ty: FieldType,
}

#[derive(Debug, Clone, Copy, Serialize)]
pub struct ApidInfo {
    pub apid: u16,
    pub name: &'static str,
    pub payload_len: usize,
}

pub const APIDS: &[ApidInfo] = &[
    ApidInfo {
        apid: apid::NAV,
        name: "NAV",
        payload_len: 28,
    },
    ApidInfo {
        apid: apid::THERMAL,
        name: "THERMAL",
        payload_len: 16,
    },
    ApidInfo {
        apid: apid::POWER,
        name: "POWER",
        payload_len: 12,
    },
    ApidInfo {
        apid: apid::STATUS,
        name: "STATUS",
        payload_len: 2,
    },
];

macro_rules! ch {
    ($id:expr, $name:expr, $label:expr, $unit:expr, $apid:expr, $off:expr, $ty:ident) => {
        Channel {
            id: $id,
            name: $name,
            label: $label,
            unit: $unit,
            apid: $apid,
            offset: $off,
            ty: FieldType::$ty,
        }
    };
}

/// Decommutation table: where every channel lives inside its packet.
pub const CHANNELS: &[Channel] = &[
    ch!(0, "nav.altitude", "Altitude", "km", apid::NAV, 0, F32),
    ch!(1, "nav.velocity", "Velocity", "m/s", apid::NAV, 4, F32),
    ch!(2, "nav.fpa", "Flight path angle", "deg", apid::NAV, 8, F32),
    ch!(3, "nav.accel", "Sensed accel", "g", apid::NAV, 12, F32),
    ch!(4, "nav.lat", "Latitude", "deg", apid::NAV, 16, F32),
    ch!(5, "nav.lon", "Longitude", "deg", apid::NAV, 20, F32),
    ch!(6, "nav.vspeed", "Vertical speed", "m/s", apid::NAV, 24, F32),
    ch!(
        7,
        "thermal.heatshield",
        "Heatshield temp",
        "°C",
        apid::THERMAL,
        0,
        F32
    ),
    ch!(
        8,
        "thermal.heatflux",
        "Stagnation heat flux",
        "W/cm²",
        apid::THERMAL,
        4,
        F32
    ),
    ch!(
        9,
        "thermal.backshell",
        "Backshell temp",
        "°C",
        apid::THERMAL,
        8,
        F32
    ),
    ch!(
        10,
        "thermal.payload",
        "Payload bay temp",
        "°C",
        apid::THERMAL,
        12,
        F32
    ),
    ch!(11, "power.bus_v", "Bus voltage", "V", apid::POWER, 0, F32),
    ch!(12, "power.current", "Bus current", "A", apid::POWER, 4, F32),
    ch!(13, "power.soc", "Battery SOC", "%", apid::POWER, 8, F32),
    ch!(14, "status.phase", "Mission phase", "", apid::STATUS, 0, U8),
    ch!(15, "status.flags", "Status flags", "", apid::STATUS, 1, U8),
];

/// Status flag bits (channel `status.flags`).
pub mod flags {
    pub const DROGUE_DEPLOYED: u8 = 1 << 0;
    pub const MAIN_DEPLOYED: u8 = 1 << 1;
    pub const BEACON_ON: u8 = 1 << 2;
    pub const TOUCHDOWN: u8 = 1 << 3;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize)]
#[repr(u8)]
pub enum Phase {
    Orbit = 0,
    DeorbitBurn = 1,
    Coast = 2,
    Entry = 3,
    Drogue = 4,
    Main = 5,
    Landed = 6,
}

impl Phase {
    pub const ALL: [Phase; 7] = [
        Phase::Orbit,
        Phase::DeorbitBurn,
        Phase::Coast,
        Phase::Entry,
        Phase::Drogue,
        Phase::Main,
        Phase::Landed,
    ];

    pub fn from_u8(v: u8) -> Option<Phase> {
        Phase::ALL.get(v as usize).copied()
    }

    pub fn name(self) -> &'static str {
        match self {
            Phase::Orbit => "ORBIT",
            Phase::DeorbitBurn => "DEORBIT BURN",
            Phase::Coast => "COAST",
            Phase::Entry => "ENTRY",
            Phase::Drogue => "DROGUE",
            Phase::Main => "MAIN CHUTE",
            Phase::Landed => "LANDED",
        }
    }
}

pub fn apid_info(apid: u16) -> Option<&'static ApidInfo> {
    APIDS.iter().find(|a| a.apid == apid)
}

pub fn channels_for(apid: u16) -> impl Iterator<Item = &'static Channel> {
    CHANNELS.iter().filter(move |c| c.apid == apid)
}

/// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF), the CRC CCSDS links commonly use.
pub fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

#[derive(Debug, Clone, PartialEq)]
pub struct Packet {
    pub apid: u16,
    pub seq: u16,
    /// Mission elapsed time in microseconds.
    pub met_us: u64,
    pub payload: Vec<u8>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub enum DecodeError {
    TooShort {
        len: usize,
    },
    BadVersion(u8),
    NoSecondaryHeader,
    LengthMismatch {
        declared: usize,
        actual: usize,
    },
    BadCrc {
        expected: u16,
        computed: u16,
    },
    UnknownApid(u16),
    PayloadSize {
        apid: u16,
        expected: usize,
        actual: usize,
    },
}

impl std::fmt::Display for DecodeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            DecodeError::TooShort { len } => write!(f, "frame too short ({len} B)"),
            DecodeError::BadVersion(v) => write!(f, "bad packet version {v}"),
            DecodeError::NoSecondaryHeader => write!(f, "missing secondary header"),
            DecodeError::LengthMismatch { declared, actual } => {
                write!(f, "length field says {declared} B, frame has {actual} B")
            }
            DecodeError::BadCrc { expected, computed } => {
                write!(
                    f,
                    "CRC mismatch (frame {expected:#06x}, computed {computed:#06x})"
                )
            }
            DecodeError::UnknownApid(a) => write!(f, "unknown APID {a:#05x}"),
            DecodeError::PayloadSize {
                apid,
                expected,
                actual,
            } => {
                write!(
                    f,
                    "APID {apid:#05x} payload {actual} B, expected {expected} B"
                )
            }
        }
    }
}

impl std::error::Error for DecodeError {}

impl Packet {
    pub fn encode(&self) -> Vec<u8> {
        let data_field_len = SECONDARY_HEADER_LEN + self.payload.len() + CRC_LEN;
        let mut out = Vec::with_capacity(PRIMARY_HEADER_LEN + data_field_len);
        // version(3) | type=0 telemetry (1) | secondary header present (1) | apid(11)
        let word0: u16 = ((VERSION as u16) << 13) | (1 << 11) | (self.apid & 0x07FF);
        // sequence flags = 0b11 (unsegmented) | 14-bit sequence count
        let word1: u16 = (0b11 << 14) | (self.seq % SEQ_MODULO);
        let word2: u16 = (data_field_len - 1) as u16;
        out.extend_from_slice(&word0.to_be_bytes());
        out.extend_from_slice(&word1.to_be_bytes());
        out.extend_from_slice(&word2.to_be_bytes());
        out.extend_from_slice(&self.met_us.to_be_bytes());
        out.extend_from_slice(&self.payload);
        let crc = crc16(&out);
        out.extend_from_slice(&crc.to_be_bytes());
        out
    }

    pub fn decode(frame: &[u8]) -> Result<Packet, DecodeError> {
        let min = PRIMARY_HEADER_LEN + SECONDARY_HEADER_LEN + CRC_LEN;
        if frame.len() < min {
            return Err(DecodeError::TooShort { len: frame.len() });
        }
        let word0 = u16::from_be_bytes([frame[0], frame[1]]);
        let word1 = u16::from_be_bytes([frame[2], frame[3]]);
        let word2 = u16::from_be_bytes([frame[4], frame[5]]);
        let version = (word0 >> 13) as u8;
        if version != VERSION {
            return Err(DecodeError::BadVersion(version));
        }
        if word0 & (1 << 11) == 0 {
            return Err(DecodeError::NoSecondaryHeader);
        }
        let declared = PRIMARY_HEADER_LEN + word2 as usize + 1;
        if declared != frame.len() {
            return Err(DecodeError::LengthMismatch {
                declared,
                actual: frame.len(),
            });
        }
        let body = &frame[..frame.len() - CRC_LEN];
        let expected = u16::from_be_bytes([frame[frame.len() - 2], frame[frame.len() - 1]]);
        let computed = crc16(body);
        if expected != computed {
            return Err(DecodeError::BadCrc { expected, computed });
        }
        let apid = word0 & 0x07FF;
        let info = apid_info(apid).ok_or(DecodeError::UnknownApid(apid))?;
        let mut met = [0u8; 8];
        met.copy_from_slice(&frame[PRIMARY_HEADER_LEN..PRIMARY_HEADER_LEN + SECONDARY_HEADER_LEN]);
        let payload = body[PRIMARY_HEADER_LEN + SECONDARY_HEADER_LEN..].to_vec();
        if payload.len() != info.payload_len {
            return Err(DecodeError::PayloadSize {
                apid,
                expected: info.payload_len,
                actual: payload.len(),
            });
        }
        Ok(Packet {
            apid,
            seq: word1 & 0x3FFF,
            met_us: u64::from_be_bytes(met),
            payload,
        })
    }

    /// Decommutate the payload into (channel id, engineering value) pairs.
    pub fn decom(&self) -> Vec<(u16, f64)> {
        channels_for(self.apid)
            .filter(|c| c.offset + c.ty.size() <= self.payload.len())
            .map(|c| {
                let b = &self.payload[c.offset..c.offset + c.ty.size()];
                let v = match c.ty {
                    FieldType::F32 => f32::from_be_bytes([b[0], b[1], b[2], b[3]]) as f64,
                    FieldType::U8 => b[0] as f64,
                };
                (c.id, v)
            })
            .collect()
    }
}

/// Builds a fixed-layout payload for a given APID.
pub struct PayloadWriter {
    buf: Vec<u8>,
}

impl PayloadWriter {
    pub fn new(apid: u16) -> Self {
        let len = apid_info(apid).map(|a| a.payload_len).unwrap_or(0);
        PayloadWriter { buf: vec![0; len] }
    }
    pub fn f32(mut self, offset: usize, v: f32) -> Self {
        self.buf[offset..offset + 4].copy_from_slice(&v.to_be_bytes());
        self
    }
    pub fn u8(mut self, offset: usize, v: u8) -> Self {
        self.buf[offset] = v;
        self
    }
    pub fn finish(self) -> Vec<u8> {
        self.buf
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> Packet {
        Packet {
            apid: apid::NAV,
            seq: 42,
            met_us: 123_456_789,
            payload: PayloadWriter::new(apid::NAV)
                .f32(0, 120.5)
                .f32(4, 7801.0)
                .f32(12, 3.2)
                .finish(),
        }
    }

    #[test]
    fn crc_matches_reference_vector() {
        // Standard check value for CRC-16/CCITT-FALSE.
        assert_eq!(crc16(b"123456789"), 0x29B1);
    }

    #[test]
    fn round_trip() {
        let p = sample();
        let frame = p.encode();
        assert_eq!(Packet::decode(&frame).unwrap(), p);
    }

    #[test]
    fn decom_reads_engineering_values() {
        let vals = sample().decom();
        let alt = vals.iter().find(|(id, _)| *id == 0).unwrap().1;
        assert!((alt - 120.5).abs() < 1e-6);
        assert_eq!(vals.len(), channels_for(apid::NAV).count());
    }

    #[test]
    fn detects_bit_flip() {
        let mut frame = sample().encode();
        frame[10] ^= 0x01;
        assert!(matches!(
            Packet::decode(&frame),
            Err(DecodeError::BadCrc { .. })
        ));
    }

    #[test]
    fn detects_truncation() {
        let frame = sample().encode();
        assert!(matches!(
            Packet::decode(&frame[..frame.len() - 3]),
            Err(DecodeError::LengthMismatch { .. })
        ));
    }

    #[test]
    fn sequence_counter_wraps_at_14_bits() {
        let mut p = sample();
        p.seq = SEQ_MODULO + 5;
        assert_eq!(Packet::decode(&p.encode()).unwrap().seq, 5);
    }

    #[test]
    fn payload_layouts_fit_their_apids() {
        for c in CHANNELS {
            let info = apid_info(c.apid).unwrap();
            assert!(
                c.offset + c.ty.size() <= info.payload_len,
                "{} overflows",
                c.name
            );
        }
    }
}
