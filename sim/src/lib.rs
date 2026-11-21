//! Flight simulator for a small reentry capsule.
//!
//! The on-orbit, deorbit and coast legs are scripted (and time-compressed);
//! from entry interface (120 km) down the capsule is integrated as a
//! point mass in a 2-D planar model with an exponential atmosphere,
//! Sutton-Graves stagnation heating and staged parachute drag.
//!
//! The simulator is deterministic for a given seed, so the SIL runner can
//! replay exactly the same flight with and without injected faults.

use proto::{Packet, PayloadWriter, Phase, apid, flags};

const R_EARTH: f64 = 6_371_000.0;
const G0: f64 = 9.806_65;
const RHO0: f64 = 1.225;
const SCALE_HEIGHT: f64 = 7_200.0;
const SIGMA: f64 = 5.670_374e-8;

/// Simple seeded PRNG (xorshift64*) so runs are reproducible.
#[derive(Debug, Clone)]
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1)
    }
    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    /// Uniform in [0, 1).
    pub fn f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
    /// Standard normal (Box-Muller).
    pub fn gauss(&mut self) -> f64 {
        let u1 = self.f64().max(1e-12);
        let u2 = self.f64();
        (-2.0 * u1.ln()).sqrt() * (std::f64::consts::TAU * u2).cos()
    }
}

/// Faults the simulator can inject, for exercising the ground segment.
#[derive(Debug, Clone, Default)]
pub struct Faults {
    /// Probability that any given packet is lost on the link.
    pub drop_rate: f64,
    /// Probability that a delivered packet has a bit flipped.
    pub corrupt_rate: f64,
    /// Main parachute fails to deploy.
    pub fail_main: bool,
    /// One-off heatshield sensor glitch at this MET (seconds).
    pub heatshield_spike_at: Option<f64>,
    /// Duration of the glitch in mission seconds (default 1.2 s).
    pub heatshield_spike_width: Option<f64>,
    /// Disable the physical plasma-blackout model (for link testing).
    pub no_blackout: bool,
}

#[derive(Debug, Clone)]
pub struct Config {
    pub seed: u64,
    pub faults: Faults,
    /// Ballistic coefficient m / (Cd*A) of the bare capsule, kg/m².
    pub beta: f64,
    /// Entry flight path angle at 120 km, degrees (negative = descending).
    pub entry_fpa_deg: f64,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            seed: 7,
            faults: Faults::default(),
            beta: 85.0,
            entry_fpa_deg: -1.6,
        }
    }
}

// Scripted timeline before entry interface (mission seconds).
const T_DEORBIT_START: f64 = 40.0;
const T_DEORBIT_END: f64 = 70.0;
const T_ENTRY_INTERFACE: f64 = 130.0;
const H_ORBIT: f64 = 400_000.0;
const H_EI: f64 = 120_000.0;
const V_ORBIT: f64 = 7_670.0;
const DEORBIT_ACCEL_G: f64 = 0.35;
const V_EI: f64 = 7_800.0;

const H_DROGUE: f64 = 14_000.0;
const H_MAIN: f64 = 2_500.0;
const BETA_DROGUE: f64 = 22.0;
const BETA_MAIN: f64 = 3.6;
const NOSE_RADIUS: f64 = 0.4;
const EMISSIVITY: f64 = 0.85;
const HEADING_DEG: f64 = 112.0;

#[derive(Debug, Clone)]
pub struct State {
    pub met: f64,
    pub phase: Phase,
    pub h: f64,
    pub v: f64,
    pub gamma: f64,
    pub lat: f64,
    pub lon: f64,
    pub sensed_g: f64,
    pub heat_flux: f64,
    pub t_shield: f64,
    pub t_backshell: f64,
    pub t_payload: f64,
    pub bus_v: f64,
    pub current: f64,
    pub soc: f64,
    pub flags: u8,
    pub beta: f64,
    pub pyro_until: f64,
    pub touchdown_vspeed: Option<f64>,
    pub met_landed: Option<f64>,
}

pub struct Capsule {
    pub cfg: Config,
    pub s: State,
    rng: Rng,
    link_rng: Rng,
    seq: [u16; 4],
    tick: u64,
    /// Ground truth link accounting, so tests can check the ground station.
    pub link: LinkTruth,
}

#[derive(Debug, Clone, Default)]
pub struct LinkTruth {
    pub generated: u64,
    pub blackout_lost: u64,
    pub dropped: u64,
    pub corrupted: u64,
}

fn rho(h: f64) -> f64 {
    RHO0 * (-h.max(0.0) / SCALE_HEIGHT).exp()
}

fn smoothstep(x: f64) -> f64 {
    let x = x.clamp(0.0, 1.0);
    x * x * (3.0 - 2.0 * x)
}

impl Capsule {
    pub fn new(cfg: Config) -> Self {
        let rng = Rng::new(cfg.seed);
        let link_rng = Rng::new(cfg.seed ^ 0xA5A5_5A5A);
        let beta = cfg.beta;
        Capsule {
            cfg,
            s: State {
                met: 0.0,
                phase: Phase::Orbit,
                h: H_ORBIT,
                v: V_ORBIT,
                gamma: 0.0,
                lat: -18.0,
                lon: 96.0,
                sensed_g: 0.0,
                heat_flux: 0.0,
                t_shield: -40.0,
                t_backshell: -25.0,
                t_payload: 21.0,
                bus_v: 28.6,
                current: 4.2,
                soc: 98.0,
                flags: 0,
                beta,
                pyro_until: -1.0,
                touchdown_vspeed: None,
                met_landed: None,
            },
            rng,
            link_rng,
            seq: [0; 4],
            tick: 0,
            link: LinkTruth::default(),
        }
    }

    pub fn landed(&self) -> bool {
        self.s.phase == Phase::Landed
    }

    /// Advance mission time by `dt` seconds (internally sub-stepped).
    pub fn step(&mut self, dt: f64) {
        let sub = 0.01;
        let n = (dt / sub).ceil().max(1.0) as usize;
        let h = dt / n as f64;
        for _ in 0..n {
            self.substep(h);
        }
    }

    fn substep(&mut self, dt: f64) {
        let s = &mut self.s;
        s.met += dt;
        let t = s.met;
        let mut extra_current = 0.0;

        match s.phase {
            Phase::Orbit | Phase::DeorbitBurn | Phase::Coast => {
                if t < T_DEORBIT_START {
                    s.phase = Phase::Orbit;
                    s.sensed_g = 0.0;
                } else if t < T_DEORBIT_END {
                    s.phase = Phase::DeorbitBurn;
                    s.v -= DEORBIT_ACCEL_G * G0 * dt;
                    s.sensed_g = DEORBIT_ACCEL_G;
                    extra_current = 3.1;
                } else if t < T_ENTRY_INTERFACE {
                    // Time-compressed coast from 400 km to entry interface.
                    s.phase = Phase::Coast;
                    let x = smoothstep((t - T_DEORBIT_END) / (T_ENTRY_INTERFACE - T_DEORBIT_END));
                    let v_start =
                        V_ORBIT - DEORBIT_ACCEL_G * G0 * (T_DEORBIT_END - T_DEORBIT_START);
                    s.h = H_ORBIT + (H_EI - H_ORBIT) * x;
                    s.v = v_start + (V_EI - v_start) * x;
                    s.gamma = (self.cfg.entry_fpa_deg * x).to_radians();
                    s.sensed_g = 0.0;
                } else {
                    s.phase = Phase::Entry;
                    s.h = H_EI;
                    s.v = V_EI;
                    s.gamma = self.cfg.entry_fpa_deg.to_radians();
                }
            }
            Phase::Entry | Phase::Drogue | Phase::Main => {
                // Parachute staging on (baro) altitude.
                if s.phase == Phase::Entry && s.h < H_DROGUE {
                    s.phase = Phase::Drogue;
                    s.flags |= flags::DROGUE_DEPLOYED;
                    s.pyro_until = t + 0.4;
                }
                if s.phase == Phase::Drogue && s.h < H_MAIN && !self.cfg.faults.fail_main {
                    s.phase = Phase::Main;
                    s.flags |= flags::MAIN_DEPLOYED | flags::BEACON_ON;
                    s.pyro_until = t + 0.4;
                }
                // Chutes inflate over ~2 s rather than instantly.
                let target_beta = match s.phase {
                    Phase::Drogue => BETA_DROGUE,
                    Phase::Main => BETA_MAIN,
                    _ => self.cfg.beta,
                };
                s.beta += (target_beta - s.beta) * (dt / 1.5).min(1.0);

                let g = G0 * (R_EARTH / (R_EARTH + s.h)).powi(2);
                let r = rho(s.h);
                let drag_acc = 0.5 * r * s.v * s.v / s.beta;
                let dv = -drag_acc - g * s.gamma.sin();
                let dgamma = (s.v / (R_EARTH + s.h) - g / s.v.max(1.0)) * s.gamma.cos();
                s.v = (s.v + dv * dt).max(0.5);
                s.gamma = (s.gamma + dgamma * dt).clamp(-std::f64::consts::FRAC_PI_2, 0.3);
                s.h += s.v * s.gamma.sin() * dt;
                s.sensed_g = drag_acc / G0;

                if s.h <= 0.0 {
                    s.h = 0.0;
                    s.touchdown_vspeed = Some(s.v * s.gamma.sin());
                    s.met_landed = Some(t);
                    s.phase = Phase::Landed;
                    s.flags |= flags::TOUCHDOWN | flags::BEACON_ON;
                    s.v = 0.0;
                    s.gamma = 0.0;
                    s.sensed_g = 1.0;
                }
            }
            Phase::Landed => {
                s.sensed_g = 1.0;
            }
        }

        // Ground track: move along a fixed heading at horizontal speed.
        if s.phase != Phase::Landed {
            let ground = s.v * s.gamma.cos() * R_EARTH / (R_EARTH + s.h);
            let hdg = HEADING_DEG.to_radians();
            let dlat = ground * hdg.cos() * dt / 111_320.0;
            let dlon = ground * hdg.sin() * dt / (111_320.0 * s.lat.to_radians().cos().max(0.1));
            s.lat += dlat;
            s.lon += dlon;
        }

        // Aerothermal: Sutton-Graves stagnation heating; surface temperature
        // lags toward radiative equilibrium.
        let in_atmo = matches!(s.phase, Phase::Entry | Phase::Drogue | Phase::Main);
        s.heat_flux = if in_atmo {
            1.7415e-4 * (rho(s.h) / NOSE_RADIUS).sqrt() * s.v.powi(3) // W/m²
        } else {
            0.0
        };
        let ambient_c = if s.h > 60_000.0 {
            -40.0
        } else {
            15.0 - 0.0065 * s.h.min(11_000.0)
        };
        let t_eq_c = ((s.heat_flux / (EMISSIVITY * SIGMA)).powf(0.25) - 273.15).max(ambient_c);
        let tau = if t_eq_c > s.t_shield { 6.0 } else { 45.0 };
        s.t_shield += (t_eq_c - s.t_shield) * (dt / tau);
        s.t_backshell += (0.18 * s.t_shield.max(ambient_c) - s.t_backshell) * (dt / 25.0);
        s.t_payload += (s.t_backshell - s.t_payload) * dt * 0.000_12;

        // Power: base load, burn and pyro spikes, beacon after main.
        if t < s.pyro_until {
            extra_current += 18.0;
        }
        if s.flags & flags::BEACON_ON != 0 {
            extra_current += 0.6;
        }
        s.current = 4.2 + extra_current;
        s.bus_v = 28.6 - 0.0009 * t - 0.11 * extra_current;
        s.soc -= s.bus_v * s.current * dt / 3600.0 / 420.0 * 100.0;
    }

    fn next_seq(&mut self, apid: u16) -> u16 {
        let i = match apid {
            apid::NAV => 0,
            apid::THERMAL => 1,
            apid::POWER => 2,
            _ => 3,
        };
        let v = self.seq[i];
        self.seq[i] = (v + 1) % proto::SEQ_MODULO;
        v
    }

    /// Build the packets due on this 50 Hz tick (before link effects).
    pub fn telemetry(&mut self) -> Vec<Packet> {
        let tick = self.tick;
        self.tick += 1;
        let met_us = (self.s.met * 1e6) as u64;
        let mut out = Vec::with_capacity(4);
        let n = |rng: &mut Rng, sd: f64| rng.gauss() * sd;

        let landed = self.landed();
        if !landed || tick.is_multiple_of(10) {
            let s = self.s.clone();
            let p = PayloadWriter::new(apid::NAV)
                .f32(0, (s.h / 1000.0 + n(&mut self.rng, 0.002)) as f32)
                .f32(4, (s.v + n(&mut self.rng, 0.4)) as f32)
                .f32(8, s.gamma.to_degrees() as f32)
                .f32(12, (s.sensed_g + n(&mut self.rng, 0.015)).max(0.0) as f32)
                .f32(16, s.lat as f32)
                .f32(20, s.lon as f32)
                .f32(24, (s.v * s.gamma.sin()) as f32)
                .finish();
            let seq = self.next_seq(apid::NAV);
            out.push(Packet {
                apid: apid::NAV,
                seq,
                met_us,
                payload: p,
            });
        }
        if tick.is_multiple_of(2) {
            let s = self.s.clone();
            let mut shield = s.t_shield + n(&mut self.rng, 1.5);
            if let Some(at) = self.cfg.faults.heatshield_spike_at {
                let half = self.cfg.faults.heatshield_spike_width.unwrap_or(1.2) / 2.0;
                if (s.met - at).abs() < half {
                    shield += 1400.0;
                }
            }
            let p = PayloadWriter::new(apid::THERMAL)
                .f32(0, shield as f32)
                .f32(4, (s.heat_flux / 1e4) as f32)
                .f32(8, (s.t_backshell + n(&mut self.rng, 0.5)) as f32)
                .f32(12, (s.t_payload + n(&mut self.rng, 0.05)) as f32)
                .finish();
            let seq = self.next_seq(apid::THERMAL);
            out.push(Packet {
                apid: apid::THERMAL,
                seq,
                met_us,
                payload: p,
            });
        }
        if tick.is_multiple_of(5) {
            let s = self.s.clone();
            let p = PayloadWriter::new(apid::POWER)
                .f32(0, (s.bus_v + n(&mut self.rng, 0.03)) as f32)
                .f32(4, (s.current + n(&mut self.rng, 0.05)) as f32)
                .f32(8, s.soc as f32)
                .finish();
            let seq = self.next_seq(apid::POWER);
            out.push(Packet {
                apid: apid::POWER,
                seq,
                met_us,
                payload: p,
            });
        }
        if tick.is_multiple_of(10) {
            let p = PayloadWriter::new(apid::STATUS)
                .u8(0, self.s.phase as u8)
                .u8(1, self.s.flags)
                .finish();
            let seq = self.next_seq(apid::STATUS);
            out.push(Packet {
                apid: apid::STATUS,
                seq,
                met_us,
                payload: p,
            });
        }
        out
    }

    /// Plasma blackout: ionised sheath blocks RF at high speed in the upper atmosphere.
    pub fn in_blackout(&self) -> bool {
        !self.cfg.faults.no_blackout
            && self.s.phase == Phase::Entry
            && self.s.v > 4_800.0
            && (42_000.0..88_000.0).contains(&self.s.h)
    }

    /// Encode packets and apply link effects (blackout, loss, bit errors).
    pub fn downlink(&mut self, packets: Vec<Packet>) -> Vec<Vec<u8>> {
        self.link.generated += packets.len() as u64;
        if self.in_blackout() {
            self.link.blackout_lost += packets.len() as u64;
            return Vec::new();
        }
        let f = self.cfg.faults.clone();
        packets
            .into_iter()
            .filter_map(|p| {
                if self.link_rng.f64() < f.drop_rate {
                    self.link.dropped += 1;
                    return None;
                }
                let mut frame = p.encode();
                if self.link_rng.f64() < f.corrupt_rate {
                    let i = (self.link_rng.next_u64() as usize) % frame.len();
                    frame[i] ^= 1 << (self.link_rng.next_u64() % 8);
                    self.link.corrupted += 1;
                }
                Some(frame)
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fly(cfg: Config) -> Capsule {
        let mut c = Capsule::new(cfg);
        let mut guard = 0;
        while !c.landed() && guard < 200_000 {
            c.step(0.2);
            guard += 1;
        }
        c
    }

    #[test]
    fn nominal_flight_lands_softly() {
        let c = fly(Config::default());
        assert!(c.landed());
        let vz = c.s.touchdown_vspeed.unwrap();
        assert!(vz > -10.0 && vz < -4.0, "touchdown {vz} m/s");
    }

    #[test]
    fn failed_main_lands_hard() {
        let mut cfg = Config::default();
        cfg.faults.fail_main = true;
        let c = fly(cfg);
        assert!(c.s.touchdown_vspeed.unwrap() < -15.0);
    }

    #[test]
    fn deterministic_for_seed() {
        let a = fly(Config::default());
        let b = fly(Config::default());
        assert_eq!(a.s.met_landed, b.s.met_landed);
    }
}
