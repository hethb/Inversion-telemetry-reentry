//! `ground`: live ground station. Receives telemetry over UDP, runs it through
//! the ground segment core, archives raw frames, and serves the operator
//! display (HTTP + WebSocket).

use axum::{
    extract::{
        ws::{Message, WebSocket, WebSocketUpgrade},
        Path, State,
    },
    response::{Html, IntoResponse},
    routing::{get, post},
    Json, Router,
};
use clap::Parser;
use ground::{archive, Delta, Ground};
use proto::{Phase, APIDS, CHANNELS};
use serde_json::{json, Value};
use std::{
    path::PathBuf,
    sync::Arc,
    time::{Duration, Instant},
};
use tokio::{
    net::UdpSocket,
    sync::{broadcast, Mutex},
};

const INDEX_HTML: &str = include_str!("../web/index.html");

#[derive(Parser, Debug)]
#[command(about = "Ground station: telemetry ingest, limit checking and operator display")]
struct Args {
    /// UDP address to receive telemetry on.
    #[arg(long, default_value = "0.0.0.0:7000")]
    udp: String,
    /// HTTP address for the operator display.
    #[arg(long, default_value = "127.0.0.1:8080")]
    http: String,
    /// Write every received frame to this archive (JSON lines).
    #[arg(long)]
    archive: Option<PathBuf>,
    /// Replay an archive instead of listening on UDP.
    #[arg(long)]
    replay: Option<PathBuf>,
    /// Replay speed multiplier.
    #[arg(long, default_value_t = 1.0)]
    replay_speed: f64,
}

struct App {
    ground: Mutex<Ground>,
    pending: Mutex<Delta>,
    tx: broadcast::Sender<String>,
    start: Instant,
}

impl App {
    fn now(&self) -> f64 {
        self.start.elapsed().as_secs_f64()
    }
}

fn meta(g: &Ground) -> Value {
    json!({
        "channels": CHANNELS,
        "apids": APIDS,
        "limits": g.limits(),
        "phases": Phase::ALL.iter().map(|p| json!({"id": *p as u8, "name": p.name()})).collect::<Vec<_>>(),
        "los_timeout_s": ground::LOS_TIMEOUT_S,
        "persistence": ground::PERSISTENCE,
    })
}

fn status(g: &Ground) -> Value {
    json!({
        "met": g.met,
        "phase": g.phase.map(|p| p as u8),
        "link": g.link,
        "stats": g.stats,
        "errors": g.errors,
        "touchdown_vspeed": g.touchdown_vspeed,
    })
}

fn snapshot(g: &Ground) -> Value {
    let history: serde_json::Map<String, Value> =
        g.history.iter().map(|(ch, h)| (ch.to_string(), json!(h))).collect();
    json!({
        "type": "snapshot",
        "meta": meta(g),
        "status": status(g),
        "history": history,
        "alarms": g.alarms,
        "events": g.events,
    })
}

async fn ingest_frame(app: &App, frame: &[u8], t: f64) {
    let d = app.ground.lock().await.ingest(frame, t);
    if !d.is_empty() {
        app.pending.lock().await.merge(d);
    }
}

#[tokio::main]
async fn main() -> anyhow_lite::Result<()> {
    let a = Args::parse();
    let (tx, _) = broadcast::channel(256);
    let app = Arc::new(App {
        ground: Mutex::new(Ground::default()),
        pending: Mutex::new(Delta::default()),
        tx,
        start: Instant::now(),
    });

    // Telemetry source: live UDP or archive replay.
    if let Some(path) = a.replay.clone() {
        let frames = archive::read(&path)?;
        eprintln!("replaying {} frames from {} at {}x", frames.len(), path.display(), a.replay_speed);
        let app2 = app.clone();
        let speed = a.replay_speed.max(0.01);
        tokio::spawn(async move {
            let t0 = frames.first().map(|f| f.0).unwrap_or(0.0);
            let start = Instant::now();
            for (t, f) in frames {
                let due = Duration::from_secs_f64(((t - t0) / speed).max(0.0));
                if let Some(wait) = due.checked_sub(start.elapsed()) {
                    tokio::time::sleep(wait).await;
                }
                let now = app2.now();
                ingest_frame(&app2, &f, now).await;
            }
            eprintln!("replay complete");
        });
    } else {
        let sock = UdpSocket::bind(&a.udp).await?;
        eprintln!("listening for telemetry on udp://{}", a.udp);
        let mut writer = match &a.archive {
            Some(p) => Some(archive::Writer::create(p)?),
            None => None,
        };
        let app2 = app.clone();
        tokio::spawn(async move {
            let mut buf = vec![0u8; 2048];
            let mut since_flush = 0u32;
            loop {
                let Ok((n, _)) = sock.recv_from(&mut buf).await else { continue };
                let t = app2.now();
                if let Some(w) = writer.as_mut() {
                    let _ = w.write(t, &buf[..n]);
                    since_flush += 1;
                    if since_flush >= 200 {
                        let _ = w.flush();
                        since_flush = 0;
                    }
                }
                ingest_frame(&app2, &buf[..n], t).await;
            }
        });
    }

    // 10 Hz display update: LOS check + batched deltas.
    {
        let app = app.clone();
        tokio::spawn(async move {
            let mut iv = tokio::time::interval(Duration::from_millis(100));
            loop {
                iv.tick().await;
                let now = app.now();
                let mut g = app.ground.lock().await;
                let d = g.tick(now);
                let mut pending = app.pending.lock().await;
                pending.merge(d);
                let batch = std::mem::take(&mut *pending);
                drop(pending);
                let samples: Vec<_> = batch.samples.iter().map(|s| json!([s.ch, s.met, s.v])).collect();
                let msg = json!({
                    "type": "batch",
                    "samples": samples,
                    "alarms": batch.alarms,
                    "events": batch.events,
                    "status": status(&g),
                });
                drop(g);
                let _ = app.tx.send(msg.to_string());
            }
        });
    }

    let router = Router::new()
        .route("/", get(|| async { Html(INDEX_HTML) }))
        .route("/ws", get(ws_handler))
        .route("/api/meta", get(api_meta))
        .route("/api/snapshot", get(api_snapshot))
        .route("/api/alarms/{id}/ack", post(api_ack))
        .with_state(app.clone());
    let listener = tokio::net::TcpListener::bind(&a.http).await?;
    eprintln!("operator display on http://{}", a.http);
    axum::serve(listener, router).await?;
    Ok(())
}

async fn api_meta(State(app): State<Arc<App>>) -> Json<Value> {
    Json(meta(&*app.ground.lock().await))
}

async fn api_snapshot(State(app): State<Arc<App>>) -> Json<Value> {
    Json(snapshot(&*app.ground.lock().await))
}

async fn api_ack(State(app): State<Arc<App>>, Path(id): Path<u64>) -> impl IntoResponse {
    let acked = app.ground.lock().await.ack(id);
    match acked {
        Some(a) => {
            app.pending.lock().await.alarms.push(a.clone());
            Json(json!({ "ok": true, "alarm": a }))
        }
        None => Json(json!({ "ok": false })),
    }
}

async fn ws_handler(ws: WebSocketUpgrade, State(app): State<Arc<App>>) -> impl IntoResponse {
    ws.on_upgrade(move |socket| client(socket, app))
}

async fn client(mut socket: WebSocket, app: Arc<App>) {
    let mut rx = app.tx.subscribe();
    let snap = snapshot(&*app.ground.lock().await).to_string();
    if socket.send(Message::Text(snap.into())).await.is_err() {
        return;
    }
    loop {
        tokio::select! {
            msg = rx.recv() => match msg {
                Ok(m) => if socket.send(Message::Text(m.into())).await.is_err() { break },
                Err(broadcast::error::RecvError::Lagged(_)) => {
                    // Slow client: resynchronise with a full snapshot.
                    let snap = snapshot(&*app.ground.lock().await).to_string();
                    if socket.send(Message::Text(snap.into())).await.is_err() { break }
                }
                Err(_) => break,
            },
            incoming = socket.recv() => match incoming {
                Some(Ok(Message::Close(_))) | None | Some(Err(_)) => break,
                _ => {}
            }
        }
    }
}

/// Minimal error plumbing without pulling in another crate.
mod anyhow_lite {
    pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error + Send + Sync>>;
}
