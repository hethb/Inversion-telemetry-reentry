//! `sil`: software-in-the-loop test runner.
//!
//! Flies the simulator headless (as fast as the CPU allows) and feeds every
//! downlinked frame straight into the same ground segment code the live
//! station runs. Each scenario injects faults and then checks the ground
//! station's conclusions against the simulator's ground truth.
//!
//! Writes `runs/sil-report.md` and `runs/sil-report.json`; exits non-zero if
//! any check fails, so it can gate CI.

use ground::{Ground, Level, LinkState, TOUCHDOWN_CHANNEL};
use proto::Phase;
use serde::Serialize;
use sim::{Capsule, Config, Faults};
use std::time::Instant;

/// Same cadence as the live simulator at 10x: 50 Hz ticks, 0.2 s of mission per tick.
const TICK_HZ: f64 = 50.0;
const MISSION_DT: f64 = 0.2;

#[derive(Serialize)]
struct Check {
    name: String,
    pass: bool,
    detail: String,
}

#[derive(Serialize)]
struct ScenarioResult {
    name: &'static str,
    description: &'static str,
    pass: bool,
    checks: Vec<Check>,
    sim_ms: u128,
    mission_s: f64,
    frames_generated: u64,
    frames_received: u64,
    peak_g: f64,
    peak_heatshield_c: f64,
    touchdown_mps: Option<f64>,
}

struct Scenario {
    name: &'static str,
    description: &'static str,
    faults: Faults,
    verify: fn(&Run) -> Vec<Check>,
}

struct Run {
    cap: Capsule,
    g: Ground,
    los_during_entry: bool,
    aos_after_blackout: bool,
}

fn check(name: &str, pass: bool, detail: impl Into<String>) -> Check {
    Check {
        name: name.to_string(),
        pass,
        detail: detail.into(),
    }
}

fn fly(faults: Faults) -> Run {
    let mut cap = Capsule::new(Config {
        faults,
        ..Config::default()
    });
    let mut g = Ground::default();
    let mut tick: u64 = 0;
    let mut landed_tick: Option<u64> = None;
    let mut los_during_entry = false;
    let mut aos_after_blackout = false;
    loop {
        let now = tick as f64 / TICK_HZ;
        cap.step(MISSION_DT);
        let pkts = cap.telemetry();
        for f in cap.downlink(pkts) {
            let d = g.ingest(&f, now);
            if d.events.iter().any(|e| e.kind == "AOS") && los_during_entry {
                aos_after_blackout = true;
            }
        }
        if tick.is_multiple_of(5) {
            let d = g.tick(now);
            if d.events.iter().any(|e| e.kind == "LOS") && g.phase == Some(Phase::Entry) {
                los_during_entry = true;
            }
        }
        if cap.landed() {
            let t = *landed_tick.get_or_insert(tick);
            if tick - t > 100 {
                break;
            }
        }
        tick += 1;
        if tick > 2_000_000 {
            break;
        }
    }
    Run {
        cap,
        g,
        los_during_entry,
        aos_after_blackout,
    }
}

fn alarms_on(r: &Run, channel: &str, level: Level) -> usize {
    r.g.alarms
        .iter()
        .filter(|a| a.channel == channel && a.level == level)
        .count()
}

fn common(r: &Run) -> Vec<Check> {
    let truth = &r.cap.link;
    let lost = truth.blackout_lost + truth.dropped + truth.corrupted;
    vec![
        check(
            "vehicle landed and ground saw LANDED",
            r.cap.landed() && r.g.phase == Some(Phase::Landed),
            format!("ground phase {:?}", r.g.phase.map(|p| p.name())),
        ),
        check(
            "missing-packet count matches ground truth",
            r.g.total_missing() == lost,
            format!(
                "ground inferred {} missing; truth {} (blackout {} + dropped {} + corrupted {})",
                r.g.total_missing(),
                lost,
                truth.blackout_lost,
                truth.dropped,
                truth.corrupted
            ),
        ),
        check(
            "every corrupted frame rejected",
            r.g.errors.total() == truth.corrupted,
            format!(
                "rejected {} (crc {}, length {}, header {}); corrupted {}",
                r.g.errors.total(),
                r.g.errors.crc,
                r.g.errors.length,
                r.g.errors.header,
                truth.corrupted
            ),
        ),
        check(
            "plasma blackout detected as LOS during entry, then AOS",
            r.los_during_entry && r.aos_after_blackout,
            format!(
                "LOS in entry: {}, reacquired: {}",
                r.los_during_entry, r.aos_after_blackout
            ),
        ),
        check(
            "link ends in AOS",
            r.g.link == LinkState::Aos,
            format!("{:?}", r.g.link),
        ),
    ]
}

fn no_false_alarms(r: &Run) -> Check {
    let n = r.g.alarms.len();
    let list: Vec<String> =
        r.g.alarms
            .iter()
            .map(|a| format!("{} {:?}", a.channel, a.level))
            .collect();
    check(
        "no alarms raised on a nominal flight",
        n == 0,
        if n == 0 {
            "none".into()
        } else {
            list.join(", ")
        },
    )
}

fn touchdown_nominal(r: &Run) -> Check {
    let td = r.g.touchdown_vspeed;
    check(
        "touchdown rate within limits",
        td.is_some_and(|v| v > -9.0) && alarms_on(r, TOUCHDOWN_CHANNEL, Level::Red) == 0,
        format!(
            "{:.2} m/s (truth {:.2})",
            td.unwrap_or(f64::NAN),
            r.cap.s.touchdown_vspeed.unwrap_or(f64::NAN)
        ),
    )
}

fn scenarios() -> Vec<Scenario> {
    vec![
        Scenario {
            name: "nominal",
            description: "Clean link, nominal vehicle. Only the physical plasma blackout interrupts telemetry.",
            faults: Faults::default(),
            verify: |r| {
                let mut c = common(r);
                c.push(no_false_alarms(r));
                c.push(touchdown_nominal(r));
                c
            },
        },
        Scenario {
            name: "degraded-link",
            description: "5% packet loss and 2% bit errors on the downlink. Ground must account for every frame without false alarms.",
            faults: Faults {
                drop_rate: 0.05,
                corrupt_rate: 0.02,
                ..Faults::default()
            },
            verify: |r| {
                let mut c = common(r);
                c.push(no_false_alarms(r));
                c.push(touchdown_nominal(r));
                c
            },
        },
        Scenario {
            name: "sensor-glitch-filtered",
            description: "Single-sample heatshield sensor spike. Persistence filtering should reject it.",
            faults: Faults {
                heatshield_spike_at: Some(450.0),
                heatshield_spike_width: Some(0.3),
                ..Faults::default()
            },
            verify: |r| {
                let mut c = common(r);
                let n =
                    r.g.alarms
                        .iter()
                        .filter(|a| a.channel == "thermal.heatshield")
                        .count();
                c.push(check(
                    "one-sample spike raised no alarm",
                    n == 0,
                    format!("{n} heatshield alarms"),
                ));
                c
            },
        },
        Scenario {
            name: "sensor-excursion-alarmed",
            description: "Sustained 2 s heatshield over-temperature reading. Ground must raise a red alarm and clear it afterwards.",
            faults: Faults {
                heatshield_spike_at: Some(450.0),
                heatshield_spike_width: Some(2.0),
                ..Faults::default()
            },
            verify: |r| {
                let mut c = common(r);
                let a =
                    r.g.alarms
                        .iter()
                        .find(|a| a.channel == "thermal.heatshield" && a.level == Level::Red);
                c.push(check(
                    "red heatshield alarm raised within 1.5 s of onset",
                    a.is_some_and(|a| (a.met - 449.0).abs() < 1.5),
                    a.map(|a| format!("raised at T+{:.1}s, value {:.0} °C", a.met, a.value))
                        .unwrap_or("not raised".into()),
                ));
                c.push(check(
                    "alarm cleared once readings returned to nominal",
                    a.is_some_and(|a| a.cleared_met.is_some()),
                    a.and_then(|a| a.cleared_met)
                        .map(|t| format!("cleared at T+{t:.1}s"))
                        .unwrap_or("still active".into()),
                ));
                c
            },
        },
        Scenario {
            name: "main-chute-failure",
            description: "Main parachute fails to deploy. Ground must flag the hard landing as a red alarm.",
            faults: Faults {
                fail_main: true,
                ..Faults::default()
            },
            verify: |r| {
                let mut c = common(r);
                c.push(check(
                    "no main-deploy event reported",
                    !r.g.events.iter().any(|e| e.text.contains("Main parachute")),
                    "",
                ));
                c.push(check(
                    "hard touchdown raised a red alarm",
                    alarms_on(r, TOUCHDOWN_CHANNEL, Level::Red) == 1,
                    format!(
                        "touchdown {:.1} m/s",
                        r.g.touchdown_vspeed.unwrap_or(f64::NAN)
                    ),
                ));
                c
            },
        },
    ]
}

fn main() {
    let filter: Option<String> = std::env::args().nth(1);
    let mut results = Vec::new();
    for s in scenarios()
        .into_iter()
        .filter(|s| filter.as_deref().is_none_or(|f| s.name.contains(f)))
    {
        let t0 = Instant::now();
        let run = fly(s.faults.clone());
        let ms = t0.elapsed().as_millis();
        let checks = (s.verify)(&run);
        let pass = checks.iter().all(|c| c.pass);
        let peak = |name: &str| {
            proto::CHANNELS
                .iter()
                .find(|c| c.name == name)
                .and_then(|c| run.g.peaks.get(&c.id))
                .copied()
                .unwrap_or(f64::NAN)
        };
        println!(
            "{} {:<26} {:>5} ms",
            if pass { "PASS" } else { "FAIL" },
            s.name,
            ms
        );
        for c in &checks {
            println!(
                "     {} {}  {}",
                if c.pass { "ok  " } else { "FAIL" },
                c.name,
                c.detail
            );
        }
        results.push(ScenarioResult {
            name: s.name,
            description: s.description,
            pass,
            checks,
            sim_ms: ms,
            mission_s: run.cap.s.met,
            frames_generated: run.cap.link.generated,
            frames_received: run.g.total_received(),
            peak_g: peak("nav.accel"),
            peak_heatshield_c: peak("thermal.heatshield"),
            touchdown_mps: run.g.touchdown_vspeed,
        });
    }

    let all = results.iter().all(|r| r.pass);
    std::fs::create_dir_all("runs").ok();
    std::fs::write(
        "runs/sil-report.json",
        serde_json::to_string_pretty(&results).unwrap(),
    )
    .ok();
    std::fs::write("runs/sil-report.md", markdown(&results)).ok();
    println!(
        "\n{}/{} scenarios passed. Report: runs/sil-report.md",
        results.iter().filter(|r| r.pass).count(),
        results.len()
    );
    std::process::exit(if all { 0 } else { 1 });
}

fn markdown(results: &[ScenarioResult]) -> String {
    let mut s = String::from(
        "# SIL report\n\n| Scenario | Result | Frames rx / generated | Peak g | Peak heatshield | Touchdown | Runtime |\n|---|---|---|---|---|---|---|\n",
    );
    for r in results {
        s += &format!(
            "| {} | {} | {} / {} | {:.2} | {:.0} °C | {} | {} ms |\n",
            r.name,
            if r.pass { "PASS" } else { "FAIL" },
            r.frames_received,
            r.frames_generated,
            r.peak_g,
            r.peak_heatshield_c,
            r.touchdown_mps
                .map(|v| format!("{:.2} m/s", -v))
                .unwrap_or("-".into()),
            r.sim_ms
        );
    }
    for r in results {
        s += &format!("\n## {}\n\n{}\n\n", r.name, r.description);
        for c in &r.checks {
            s += &format!(
                "- [{}] {} ({})\n",
                if c.pass { "x" } else { " " },
                c.name,
                c.detail
            );
        }
    }
    s
}
