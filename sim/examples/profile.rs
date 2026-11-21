//! Print a flight profile summary: `cargo run -p sim --example profile`
use sim::{Capsule, Config};
fn main() {
    let mut c = Capsule::new(Config::default());
    let (mut pg, mut pq, mut pt, mut pb, mut pp) =
        (0.0f64, 0.0f64, -999.0f64, -999.0f64, -999.0f64);
    let mut bo: Option<(f64, f64)> = None;
    let mut last_phase = c.s.phase;
    while !c.landed() {
        c.step(0.1);
        pg = pg.max(c.s.sensed_g);
        pq = pq.max(c.s.heat_flux / 1e4);
        pt = pt.max(c.s.t_shield);
        pb = pb.max(c.s.t_backshell);
        pp = pp.max(c.s.t_payload);
        if c.in_blackout() {
            bo = Some(match bo {
                None => (c.s.met, c.s.met),
                Some((a, _)) => (a, c.s.met),
            });
        }
        if c.s.phase != last_phase {
            println!(
                "{:7.1}s  {:?}  h={:.1} km v={:.0} m/s",
                c.s.met,
                c.s.phase,
                c.s.h / 1000.0,
                c.s.v
            );
            last_phase = c.s.phase;
        }
    }
    println!(
        "peak g {pg:.2}, peak q {pq:.1} W/cm2, peak shield {pt:.0} C, backshell {pb:.0} C, payload {pp:.1} C"
    );
    println!(
        "blackout {:?}, touchdown {:.2} m/s at {:.0}s, soc {:.1}%",
        bo,
        c.s.touchdown_vspeed.unwrap(),
        c.s.met_landed.unwrap(),
        c.s.soc
    );
}
