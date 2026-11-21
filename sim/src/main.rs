//! `sim`: fly the capsule and stream telemetry frames over UDP.

use clap::Parser;
use sim::{Capsule, Config, Faults};
use std::net::UdpSocket;
use std::time::{Duration, Instant};

#[derive(Parser, Debug)]
#[command(about = "Reentry capsule flight simulator: streams CCSDS-style telemetry over UDP")]
struct Args {
    /// Ground station address to send telemetry to.
    #[arg(long, default_value = "127.0.0.1:7000")]
    target: String,
    /// Time compression: mission seconds per wall-clock second.
    #[arg(long, default_value_t = 10.0)]
    speed: f64,
    /// Random seed (same seed = same flight).
    #[arg(long, default_value_t = 7)]
    seed: u64,
    /// Fraction of packets lost on the link (0..1).
    #[arg(long, default_value_t = 0.0)]
    drop: f64,
    /// Fraction of delivered packets with a flipped bit (0..1).
    #[arg(long, default_value_t = 0.0)]
    corrupt: f64,
    /// Main parachute fails to deploy.
    #[arg(long)]
    fail_main: bool,
    /// Inject a one-off heatshield sensor spike at this MET (s).
    #[arg(long)]
    spike_at: Option<f64>,
    /// Turn off the plasma blackout model.
    #[arg(long)]
    no_blackout: bool,
    /// Keep transmitting this many wall seconds after touchdown.
    #[arg(long, default_value_t = 8.0)]
    linger: f64,
}

const TICK_HZ: f64 = 50.0;

fn main() -> std::io::Result<()> {
    let a = Args::parse();
    let cfg = Config {
        seed: a.seed,
        faults: Faults {
            drop_rate: a.drop,
            corrupt_rate: a.corrupt,
            fail_main: a.fail_main,
            heatshield_spike_at: a.spike_at,
            heatshield_spike_width: None,
            no_blackout: a.no_blackout,
        },
        ..Config::default()
    };
    let sock = UdpSocket::bind("0.0.0.0:0")?;
    sock.connect(&a.target)?;
    let mut cap = Capsule::new(cfg);
    let tick = Duration::from_secs_f64(1.0 / TICK_HZ);
    let mission_dt = a.speed / TICK_HZ;
    let start = Instant::now();
    let mut n_tick: u64 = 0;
    let mut landed_at: Option<Instant> = None;
    let mut last_phase = cap.s.phase;
    let (mut sent, mut frames) = (0u64, 0u64);

    eprintln!("sim → {}  speed {}x  seed {}", a.target, a.speed, a.seed);
    loop {
        cap.step(mission_dt);
        let pkts = cap.telemetry();
        frames += pkts.len() as u64;
        for f in cap.downlink(pkts) {
            // A ground station that is not up yet is fine; keep flying.
            if sock.send(&f).is_ok() {
                sent += 1;
            }
        }
        if cap.s.phase != last_phase {
            eprintln!(
                "T+{:7.1}s  {:<12}  alt {:7.2} km  v {:6.0} m/s",
                cap.s.met,
                cap.s.phase.name(),
                cap.s.h / 1000.0,
                cap.s.v
            );
            last_phase = cap.s.phase;
        }
        if cap.landed() {
            let t = *landed_at.get_or_insert_with(Instant::now);
            if t.elapsed().as_secs_f64() > a.linger {
                break;
            }
        }
        n_tick += 1;
        let next = start + tick * n_tick as u32;
        if let Some(d) = next.checked_duration_since(Instant::now()) {
            std::thread::sleep(d);
        }
    }
    eprintln!(
        "touchdown {:.2} m/s at T+{:.0}s; {} of {} frames delivered",
        cap.s.touchdown_vspeed.unwrap_or(0.0),
        cap.s.met_landed.unwrap_or(0.0),
        sent,
        frames
    );
    Ok(())
}
