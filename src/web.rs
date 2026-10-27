//! The live map: a page on 127.0.0.1 that gets the app's estimates as server-sent events
//! (/events), the same ones the menu shows, pushed on every refresh; every past track
//! (/history.json); and every past drone (/drones.json) with its replay (/replay/<n>.json).

use std::io::{BufRead, BufReader, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::{Arc, Condvar, Mutex};
use std::time::Duration;

/// Fixed so the page survives app restarts; a second instance (a replay) takes any free port.
const PORT: u16 = 47820;
const PAGE: &str = include_str!("map.html");
/// A comment line this often keeps an idle connection from being dropped.
const KEEPALIVE: Duration = Duration::from_secs(15);

pub type State = Arc<Shared>;

/// The latest state JSON with a version number; event streams wait on `changed`.
#[derive(Default)]
pub struct Shared {
    json: Mutex<(u64, String)>,
    changed: Condvar,
    /// The channel history as the map shows it; None until first computed in the background,
    /// replaced when new posts come in.
    pub past: Mutex<Option<Arc<Past>>>,
}

pub struct Past {
    /// Every past track as JSON (/history.json).
    pub tracks: String,
    /// Every past drone as JSON (/drones.json).
    pub drones: String,
    /// Drone n of that list played back: the page's state every few simulated seconds.
    pub replay: Replayer,
}

pub type Replayer = Box<dyn Fn(usize) -> Option<String> + Send + Sync>;

impl Shared {
    pub fn set(&self, json: String) {
        if let Ok(mut s) = self.json.lock()
            && s.1 != json
        {
            *s = (s.0 + 1, json);
            self.changed.notify_all();
        }
    }
}

/// Starts serving; returns the page URL.
pub fn serve(state: State) -> std::io::Result<String> {
    let listener =
        TcpListener::bind(("127.0.0.1", PORT)).or_else(|_| TcpListener::bind(("127.0.0.1", 0)))?;
    let port = listener.local_addr()?.port();
    std::thread::spawn(move || {
        for stream in listener.incoming().flatten() {
            let state = state.clone();
            std::thread::spawn(move || {
                let _ = respond(stream, &state, port);
            });
        }
    });
    Ok(format!("http://127.0.0.1:{port}/"))
}

fn respond(stream: TcpStream, state: &Shared, port: u16) -> std::io::Result<()> {
    let mut reader = BufReader::new(&stream);
    let mut request = String::new();
    reader.read_line(&mut request)?;
    let mut host = String::new();
    loop {
        let mut line = String::new();
        if reader.read_line(&mut line)? == 0 || line.trim().is_empty() {
            break;
        }
        if let Some((name, value)) = line.split_once(':')
            && name.eq_ignore_ascii_case("host")
        {
            host = value.trim().to_owned();
        }
    }
    let path = request.split_whitespace().nth(1).unwrap_or("/");
    let mut stream = &stream;
    // Only our own origin: a page on another host name resolving here (DNS rebinding)
    // must not read the home point.
    let ours = [format!("127.0.0.1:{port}"), format!("localhost:{port}")];
    let not_ready = ("503 Service Unavailable", "text/plain", "not ready".to_owned());
    let past = state.past.lock().ok().and_then(|p| p.clone());
    let (status, kind, body) = if !ours.contains(&host) {
        ("403 Forbidden", "text/plain", "forbidden".to_owned())
    } else if path == "/" {
        ("200 OK", "text/html; charset=utf-8", PAGE.to_owned())
    } else if path == "/events" {
        return events(stream, state);
    } else if path == "/history.json" {
        match past {
            Some(p) => ("200 OK", "application/json", p.tracks.clone()),
            None => not_ready,
        }
    } else if path == "/drones.json" {
        match past {
            Some(p) => ("200 OK", "application/json", p.drones.clone()),
            None => not_ready,
        }
    } else if let Some(n) = path
        .strip_prefix("/replay/")
        .and_then(|p| p.strip_suffix(".json"))
        .and_then(|n| n.parse().ok())
    {
        match past.map(|p| (p.replay)(n)) {
            Some(Some(json)) => ("200 OK", "application/json", json),
            Some(None) => ("404 Not Found", "text/plain", "no such drone".to_owned()),
            None => not_ready,
        }
    } else {
        ("404 Not Found", "text/plain", "not found".to_owned())
    };
    write!(
        stream,
        "HTTP/1.1 {status}\r\nContent-Type: {kind}\r\nContent-Length: {}\r\n\
         Cache-Control: no-store\r\nConnection: close\r\n\r\n{body}",
        body.len()
    )
}

/// Streams every new state until the page goes away.
fn events(mut stream: &TcpStream, state: &Shared) -> std::io::Result<()> {
    stream.write_all(
        b"HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\nCache-Control: no-store\r\n\r\n",
    )?;
    let mut sent = 0;
    loop {
        let (version, json) = {
            let guard = state
                .json
                .lock()
                .map_err(|_| std::io::Error::other("poisoned"))?;
            let (guard, _) = state
                .changed
                .wait_timeout_while(guard, KEEPALIVE, |s| s.0 == sent)
                .map_err(|_| std::io::Error::other("poisoned"))?;
            guard.clone()
        };
        if version == sent {
            stream.write_all(b": keepalive\n\n")?;
        } else {
            write!(stream, "data: {json}\n\n")?;
            sent = version;
        }
    }
}
