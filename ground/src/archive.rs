//! Raw frame archive: every received frame (good or bad) is written with its
//! receive time, so a pass can be replayed bit-for-bit through the ground
//! software later for post-test analysis.

use std::fs::File;
use std::io::{self, BufRead, BufReader, BufWriter, Write};
use std::path::Path;

pub struct Writer {
    out: BufWriter<File>,
}

impl Writer {
    pub fn create(path: &Path) -> io::Result<Self> {
        if let Some(dir) = path.parent() {
            std::fs::create_dir_all(dir)?;
        }
        Ok(Writer { out: BufWriter::new(File::create(path)?) })
    }

    pub fn write(&mut self, t_s: f64, frame: &[u8]) -> io::Result<()> {
        let hex: String = frame.iter().map(|b| format!("{b:02x}")).collect();
        writeln!(self.out, "{{\"t\":{t_s:.6},\"frame\":\"{hex}\"}}")
    }

    pub fn flush(&mut self) -> io::Result<()> {
        self.out.flush()
    }
}

/// Read an archive back as (receive time, frame bytes).
pub fn read(path: &Path) -> io::Result<Vec<(f64, Vec<u8>)>> {
    let mut out = Vec::new();
    for line in BufReader::new(File::open(path)?).lines() {
        let line = line?;
        let v: serde_json::Value = match serde_json::from_str(&line) {
            Ok(v) => v,
            Err(_) => continue,
        };
        let (Some(t), Some(hex)) = (v["t"].as_f64(), v["frame"].as_str()) else { continue };
        let bytes = (0..hex.len() / 2)
            .filter_map(|i| u8::from_str_radix(&hex[2 * i..2 * i + 2], 16).ok())
            .collect();
        out.push((t, bytes));
    }
    Ok(out)
}
