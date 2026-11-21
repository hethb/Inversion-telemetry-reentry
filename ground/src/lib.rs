//! Ground segment core: frame ingest, link accounting, decommutation,
//! limit checking and the mission event log.
//!
//! Everything here is synchronous and takes the receive time as an argument,
//! so the same code runs live (fed by the UDP socket) and inside the
//! software-in-the-loop runner (fed directly by the simulator).

use proto::{APIDS, CHANNELS, Packet, Phase, flags};
use serde::Serialize;
use std::collections::HashMap;

pub mod archive;

/// Seconds without any valid frame before declaring loss of signal.
pub const LOS_TIMEOUT_S: f64 = 1.0;
/// Consecutive out-of-limit samples required before an alarm is raised.
/// Rejects single-sample sensor glitches without hiding real excursions.
pub const PERSISTENCE: u32 = 2;

pub const TOUCHDOWN_CHANNEL: &str = "derived.touchdown_vspeed";

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize)]
pub enum Level {
    Nominal,
    Yellow,
    Red,
}

#[derive(Debug, Clone, Serialize)]
pub struct Limit {
    pub channel: &'static str,
    pub yellow_lo: Option<f64>,
    pub yellow_hi: Option<f64>,
    pub red_lo: Option<f64>,
    pub red_hi: Option<f64>,
}

impl Limit {
    pub fn level(&self, v: f64) -> Level {
        let below = |x: Option<f64>| x.is_some_and(|l| v < l);
        let above = |x: Option<f64>| x.is_some_and(|l| v > l);
        if below(self.red_lo) || above(self.red_hi) {
            Level::Red
        } else if below(self.yellow_lo) || above(self.yellow_hi) {
            Level::Yellow
        } else {
            Level::Nominal
        }
    }
}

const fn hi(channel: &'static str, y: f64, r: f64) -> Limit {
    Limit {
        channel,
        yellow_lo: None,
        yellow_hi: Some(y),
        red_lo: None,
        red_hi: Some(r),
    }
}
const fn lo(channel: &'static str, y: f64, r: f64) -> Limit {
    Limit {
        channel,
        yellow_lo: Some(y),
        yellow_hi: None,
        red_lo: Some(r),
        red_hi: None,
    }
}

/// Flight rules. Thresholds sit above the nominal envelope of the simulated
/// vehicle (peak ~8.6 g, ~1850 °C shield, ~30 °C payload) with margin.
pub fn default_limits() -> Vec<Limit> {
    vec![
        hi("nav.accel", 10.0, 14.0),
        hi("thermal.heatshield", 2000.0, 2300.0),
        hi("thermal.backshell", 400.0, 500.0),
        hi("thermal.payload", 33.0, 38.0),
        lo("power.bus_v", 26.0, 24.5),
        hi("power.current", 26.0, 32.0),
        lo("power.soc", 30.0, 15.0),
        lo(TOUCHDOWN_CHANNEL, -9.0, -12.0),
    ]
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct LinkStats {
    pub apid: u16,
    pub name: &'static str,
    pub received: u64,
    /// Packets inferred missing from sequence-count gaps.
    pub missing: u64,
    pub last_seq: Option<u16>,
    pub last_rx_s: Option<f64>,
    /// Receive rate over the last second (Hz).
    pub rate_hz: f64,
    #[serde(skip)]
    window: Vec<f64>,
    #[serde(skip)]
    next_expected: u16,
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct FrameErrors {
    pub crc: u64,
    pub length: u64,
    pub header: u64,
    pub unknown_apid: u64,
}

impl FrameErrors {
    pub fn total(&self) -> u64 {
        self.crc + self.length + self.header + self.unknown_apid
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct Alarm {
    pub id: u64,
    pub channel: &'static str,
    pub label: &'static str,
    pub level: Level,
    pub value: f64,
    pub met: f64,
    pub cleared_met: Option<f64>,
    pub acked: bool,
}

#[derive(Debug, Clone, Serialize)]
pub struct Event {
    pub met: f64,
    pub kind: &'static str,
    pub text: String,
}

#[derive(Debug, Clone, Copy, Serialize)]
pub struct Sample {
    pub ch: u16,
    pub met: f64,
    pub v: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub enum LinkState {
    NoSignal,
    Aos,
    Los,
}

/// What one ingest call produced, for streaming to displays.
#[derive(Debug, Default)]
pub struct Delta {
    pub samples: Vec<Sample>,
    pub alarms: Vec<Alarm>,
    pub events: Vec<Event>,
}

impl Delta {
    pub fn merge(&mut self, other: Delta) {
        self.samples.extend(other.samples);
        self.alarms.extend(other.alarms);
        self.events.extend(other.events);
    }
    pub fn is_empty(&self) -> bool {
        self.samples.is_empty() && self.alarms.is_empty() && self.events.is_empty()
    }
}

struct LimitState {
    limit: Limit,
    level: Level,
    pending: Level,
    pending_count: u32,
    open_alarm: Option<usize>,
}

pub struct Ground {
    pub stats: Vec<LinkStats>,
    pub errors: FrameErrors,
    pub history: HashMap<u16, Vec<(f64, f64)>>,
    pub latest: HashMap<u16, (f64, f64)>,
    pub alarms: Vec<Alarm>,
    pub events: Vec<Event>,
    pub phase: Option<Phase>,
    pub met: f64,
    pub link: LinkState,
    pub touchdown_vspeed: Option<f64>,
    pub peaks: HashMap<u16, f64>,
    limits: HashMap<&'static str, LimitState>,
    last_any_rx_s: Option<f64>,
    last_flags: u8,
    next_alarm_id: u64,
}

fn channel_by_name(name: &str) -> Option<&'static proto::Channel> {
    CHANNELS.iter().find(|c| c.name == name)
}

impl Default for Ground {
    fn default() -> Self {
        Self::new(default_limits())
    }
}

impl Ground {
    pub fn new(limits: Vec<Limit>) -> Self {
        let stats = APIDS
            .iter()
            .map(|a| LinkStats {
                apid: a.apid,
                name: a.name,
                ..Default::default()
            })
            .collect();
        let limits = limits
            .into_iter()
            .map(|l| {
                (
                    l.channel,
                    LimitState {
                        limit: l,
                        level: Level::Nominal,
                        pending: Level::Nominal,
                        pending_count: 0,
                        open_alarm: None,
                    },
                )
            })
            .collect();
        Ground {
            stats,
            errors: FrameErrors::default(),
            history: HashMap::new(),
            latest: HashMap::new(),
            alarms: Vec::new(),
            events: Vec::new(),
            phase: None,
            met: 0.0,
            link: LinkState::NoSignal,
            touchdown_vspeed: None,
            peaks: HashMap::new(),
            limits,
            last_any_rx_s: None,
            last_flags: 0,
            next_alarm_id: 1,
        }
    }

    pub fn limits(&self) -> Vec<Limit> {
        let mut v: Vec<Limit> = self.limits.values().map(|s| s.limit.clone()).collect();
        v.sort_by_key(|l| l.channel);
        v
    }

    fn event(&mut self, delta: &mut Delta, kind: &'static str, text: String) {
        let e = Event {
            met: self.met,
            kind,
            text,
        };
        self.events.push(e.clone());
        delta.events.push(e);
    }

    /// Ingest one received frame. `now_s` is the receive time (wall clock
    /// live, simulated in SIL).
    pub fn ingest(&mut self, frame: &[u8], now_s: f64) -> Delta {
        let mut delta = Delta::default();
        let pkt = match Packet::decode(frame) {
            Ok(p) => p,
            Err(e) => {
                use proto::DecodeError::*;
                match e {
                    BadCrc { .. } => self.errors.crc += 1,
                    LengthMismatch { .. } | TooShort { .. } | PayloadSize { .. } => {
                        self.errors.length += 1
                    }
                    BadVersion(_) | NoSecondaryHeader => self.errors.header += 1,
                    UnknownApid(_) => self.errors.unknown_apid += 1,
                }
                return delta;
            }
        };

        if self.link != LinkState::Aos {
            let text = match self.link {
                LinkState::NoSignal => "First frame received: acquisition of signal".to_string(),
                _ => format!(
                    "Signal reacquired after {:.1} s",
                    now_s - self.last_any_rx_s.unwrap_or(now_s)
                ),
            };
            self.met = pkt.met_us as f64 / 1e6;
            self.link = LinkState::Aos;
            self.event(&mut delta, "AOS", text);
        }
        self.last_any_rx_s = Some(now_s);
        self.met = self.met.max(pkt.met_us as f64 / 1e6);

        if let Some(st) = self.stats.iter_mut().find(|s| s.apid == pkt.apid) {
            // Sequence accounting (14-bit counter, assumed to start at 0).
            let gap = pkt.seq.wrapping_sub(st.next_expected) % proto::SEQ_MODULO;
            if gap < proto::SEQ_MODULO / 2 {
                st.missing += gap as u64;
            }
            st.next_expected = (pkt.seq + 1) % proto::SEQ_MODULO;
            st.received += 1;
            st.last_seq = Some(pkt.seq);
            st.last_rx_s = Some(now_s);
            st.window.push(now_s);
            st.window.retain(|t| now_s - *t <= 1.0);
            st.rate_hz = st.window.len() as f64;
        }

        let met = pkt.met_us as f64 / 1e6;
        for (ch, v) in pkt.decom() {
            self.record(ch, met, v, &mut delta);
        }
        delta
    }

    fn record(&mut self, ch: u16, met: f64, v: f64, delta: &mut Delta) {
        self.history.entry(ch).or_default().push((met, v));
        self.latest.insert(ch, (met, v));
        let peak = self.peaks.entry(ch).or_insert(v);
        if v.abs() > peak.abs() {
            *peak = v;
        }
        delta.samples.push(Sample { ch, met, v });
        let c = &CHANNELS[ch as usize];

        match c.name {
            "status.phase" => {
                if let Some(p) = Phase::from_u8(v as u8)
                    && self.phase != Some(p) {
                        let text = match self.phase {
                            None => format!("Phase {}", p.name()),
                            Some(prev) => format!("{} → {}", prev.name(), p.name()),
                        };
                        self.phase = Some(p);
                        self.event(delta, "PHASE", text);
                        if p == Phase::Landed {
                            self.on_touchdown(met, delta);
                        }
                    }
            }
            "status.flags" => {
                let f = v as u8;
                let new = f & !self.last_flags;
                for (bit, text) in [
                    (flags::DROGUE_DEPLOYED, "Drogue parachute deployed"),
                    (flags::MAIN_DEPLOYED, "Main parachute deployed"),
                    (flags::BEACON_ON, "Recovery beacon on"),
                    (flags::TOUCHDOWN, "Touchdown detected"),
                ] {
                    if new & bit != 0 {
                        self.event(delta, "VEHICLE", text.to_string());
                    }
                }
                self.last_flags = f;
            }
            _ => {}
        }
        self.check_limit(c.name, c.label, met, v, delta);
    }

    fn on_touchdown(&mut self, met: f64, delta: &mut Delta) {
        // Touchdown rate = last descending vertical speed before landing.
        let vs = channel_by_name("nav.vspeed").map(|c| c.id);
        let td = vs.and_then(|id| {
            self.history
                .get(&id)
                .and_then(|h| h.iter().rev().map(|(_, v)| *v).find(|v| *v < -0.05))
        });
        if let Some(td) = td {
            self.touchdown_vspeed = Some(td);
            self.event(delta, "VEHICLE", format!("Touchdown at {:.1} m/s", -td));
            // A one-shot derived measurement: force it past persistence.
            for _ in 0..PERSISTENCE {
                self.check_limit(
                    TOUCHDOWN_CHANNEL,
                    "Touchdown vertical speed",
                    met,
                    td,
                    delta,
                );
            }
        }
    }

    fn check_limit(
        &mut self,
        name: &'static str,
        label: &'static str,
        met: f64,
        v: f64,
        delta: &mut Delta,
    ) {
        let Some(ls) = self.limits.get_mut(name) else {
            return;
        };
        let lvl = ls.limit.level(v);
        if lvl == ls.level {
            ls.pending_count = 0;
            if let Some(i) = ls.open_alarm {
                // Track the worst value while the excursion lasts.
                let a = &mut self.alarms[i];
                if (v - a.value).abs() > 0.0
                    && ls.limit.level(v) >= a.level
                    && worse(&ls.limit, v, a.value)
                {
                    a.value = v;
                }
            }
            return;
        }
        if lvl == ls.pending {
            ls.pending_count += 1;
        } else {
            ls.pending = lvl;
            ls.pending_count = 1;
        }
        if ls.pending_count < PERSISTENCE {
            return;
        }
        let prev = ls.level;
        ls.level = lvl;
        ls.pending_count = 0;
        if lvl > prev {
            // Escalation: close the lower alarm and open a new one.
            if let Some(i) = ls.open_alarm.take() {
                self.alarms[i].cleared_met = Some(met);
            }
            let a = Alarm {
                id: self.next_alarm_id,
                channel: name,
                label,
                level: lvl,
                value: v,
                met,
                cleared_met: None,
                acked: false,
            };
            self.next_alarm_id += 1;
            self.alarms.push(a.clone());
            ls.open_alarm = Some(self.alarms.len() - 1);
            delta.alarms.push(a);
        } else {
            // De-escalation: clear the open alarm; reopen at yellow if needed.
            if let Some(i) = ls.open_alarm.take() {
                self.alarms[i].cleared_met = Some(met);
                delta.alarms.push(self.alarms[i].clone());
            }
            if lvl == Level::Yellow {
                let a = Alarm {
                    id: self.next_alarm_id,
                    channel: name,
                    label,
                    level: lvl,
                    value: v,
                    met,
                    cleared_met: None,
                    acked: false,
                };
                self.next_alarm_id += 1;
                self.alarms.push(a.clone());
                ls.open_alarm = Some(self.alarms.len() - 1);
                delta.alarms.push(a);
            }
        }
    }

    /// Periodic housekeeping: detect loss of signal.
    pub fn tick(&mut self, now_s: f64) -> Delta {
        let mut delta = Delta::default();
        if self.link == LinkState::Aos
            && let Some(t) = self.last_any_rx_s
                && now_s - t > LOS_TIMEOUT_S {
                    self.link = LinkState::Los;
                    let in_entry = self.phase == Some(Phase::Entry);
                    let text = if in_entry {
                        "Loss of signal (expected: entry plasma blackout)".to_string()
                    } else {
                        "Loss of signal".to_string()
                    };
                    self.event(&mut delta, "LOS", text);
                }
        for s in &mut self.stats {
            s.window.retain(|t| now_s - *t <= 1.0);
            s.rate_hz = s.window.len() as f64;
        }
        delta
    }

    pub fn ack(&mut self, id: u64) -> Option<Alarm> {
        let a = self.alarms.iter_mut().find(|a| a.id == id)?;
        a.acked = true;
        Some(a.clone())
    }

    pub fn total_missing(&self) -> u64 {
        self.stats.iter().map(|s| s.missing).sum()
    }

    pub fn total_received(&self) -> u64 {
        self.stats.iter().map(|s| s.received).sum()
    }
}

fn worse(l: &Limit, v: f64, current: f64) -> bool {
    if l.red_lo.is_some() || l.yellow_lo.is_some() {
        v < current
    } else {
        v > current
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proto::{PayloadWriter, apid};

    fn thermal(seq: u16, met: f64, shield: f32) -> Vec<u8> {
        Packet {
            apid: apid::THERMAL,
            seq,
            met_us: (met * 1e6) as u64,
            payload: PayloadWriter::new(apid::THERMAL).f32(0, shield).finish(),
        }
        .encode()
    }

    #[test]
    fn counts_sequence_gaps() {
        let mut g = Ground::default();
        g.ingest(&thermal(0, 0.0, 20.0), 0.0);
        g.ingest(&thermal(1, 0.1, 20.0), 0.1);
        g.ingest(&thermal(5, 0.5, 20.0), 0.5);
        assert_eq!(g.total_missing(), 3);
    }

    #[test]
    fn rejects_corrupt_frames() {
        let mut g = Ground::default();
        let mut f = thermal(0, 0.0, 20.0);
        f[12] ^= 0x40;
        g.ingest(&f, 0.0);
        assert_eq!(g.errors.crc, 1);
        assert_eq!(g.total_received(), 0);
    }

    #[test]
    fn single_glitch_does_not_alarm_but_sustained_does() {
        let mut g = Ground::default();
        g.ingest(&thermal(0, 0.0, 1500.0), 0.0);
        g.ingest(&thermal(1, 0.1, 2500.0), 0.1); // one bad sample
        g.ingest(&thermal(2, 0.2, 1500.0), 0.2);
        assert!(g.alarms.is_empty());
        g.ingest(&thermal(3, 0.3, 2500.0), 0.3);
        g.ingest(&thermal(4, 0.4, 2500.0), 0.4);
        assert_eq!(g.alarms.len(), 1);
        assert_eq!(g.alarms[0].level, Level::Red);
    }

    #[test]
    fn declares_los_and_aos() {
        let mut g = Ground::default();
        g.ingest(&thermal(0, 0.0, 20.0), 0.0);
        g.tick(2.0);
        assert_eq!(g.link, LinkState::Los);
        g.ingest(&thermal(1, 3.0, 20.0), 3.0);
        assert_eq!(g.link, LinkState::Aos);
        assert!(g.events.iter().any(|e| e.kind == "LOS"));
    }
}
