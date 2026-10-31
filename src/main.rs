mod icon;
mod web;

use ultra_alert::{gazetteer, predict, telegram};

use chrono::{DateTime, Local, Utc};
use predict::{Basis, Model, Status, Tracker};
use regex::Regex;
use std::sync::LazyLock;
use std::time::{Duration, Instant};
use telegram::Post;
use tray_icon::{
    MouseButton, MouseButtonState, TrayIcon, TrayIconBuilder, TrayIconEvent,
    menu::{Menu, MenuEvent, MenuItem, PredefinedMenuItem},
};
use winit::{
    application::ApplicationHandler,
    event_loop::{ActiveEventLoop, ControlFlow, EventLoop},
    platform::macos::{ActivationPolicy, EventLoopBuilderExtMacOS},
    window::WindowId,
};

const CHANNEL: &str = "vinnytsiappo";
const POLL: Duration = Duration::from_secs(5);
const REFRESH: Duration = Duration::from_millis(250);
const STALE: Duration = Duration::from_secs(30);
/// Below these odds of a drone being sent here the menu bar shows the odds, from them on its
/// distance; a drone sent here shows the countdown.
const KM_FROM_ODDS: f64 = 0.3;
/// Posts older than this are history, not worth a lookup.
const LOOKUP_AGE: chrono::TimeDelta = chrono::TimeDelta::hours(3);
/// A post with one of these is about a drone and may name a place.
const LOOKUP_WORDS: &[&str] = &[
    "шахед", "бпла", "🛵", "реактив", "дрон", "укритт", "обережн", "курс",
];

#[derive(Debug)]
enum UserEvent {
    Posts(Vec<Post>),
    FetchError(String),
    /// Posts of a fallback channel (vinnytsiarealll, Захід Головне, …).
    FallbackPosts(Vec<Post>),
    Menu(MenuEvent),
    /// A click on the status item (a right click clears the situation).
    Tray(TrayIconEvent),
    /// Past drones' turns, from the history computed in the background.
    Routes(Vec<predict::RouteLeg>, Vec<predict::Passage>),
    /// Places the gazetteer did not know, found for a post just now: (spelling, place).
    Places(Vec<(String, gazetteer::Place)>),
}

struct App {
    tracker: Tracker,
    tray: Option<TrayIcon>,
    status_item: MenuItem,
    eta_item: MenuItem,
    post_item: MenuItem,
    if_item: MenuItem,
    model_item: MenuItem,
    map_item: MenuItem,
    source_item: MenuItem,
    dismiss_item: MenuItem,
    quit_item: MenuItem,
    title: String,
    blink: Option<Duration>,
    blink_visible: bool,
    next_blink: Instant,
    next_refresh: Instant,
    last_ok: Option<Instant>,
    last_error: Option<String>,
    logged: String,
    recorded: String,
    /// Alert already notified, and whether its districts notification went out too.
    notified: Option<(DateTime<Utc>, bool)>,
    replay: Option<Replay>,
    /// "від дому" with a home point configured, otherwise "від Вінниці".
    from_label: &'static str,
    /// What the map page shows, rewritten on every refresh.
    map_state: web::State,
    map_url: Option<String>,
    /// Posts to look up unknown places in (`lookup`), and the ones already sent.
    lookup: Option<std::sync::mpsc::Sender<String>>,
    looked_up: std::collections::HashSet<(String, u64)>,
    /// The channel's posts for the map's history; none in a replay.
    history: Option<std::sync::mpsc::Sender<Vec<Post>>>,
}

/// `--replay <UTC start>[,<UTC start>…] [--speed N] [--minutes M] [--stagger S]`: plays past
/// attacks from data/messages.jsonl on an accelerated clock, with the live UI and
/// notifications. Several starts overlay those attacks on one clock, each S seconds (default
/// 60) after the previous one, to see the tracker with many drones at once.
struct Replay {
    from: DateTime<Utc>,
    until: DateTime<Utc>,
    speed: f64,
    started: Instant,
    /// Each replayed attack's own start and how far it is moved to land on the replay clock.
    windows: Vec<(DateTime<Utc>, chrono::Duration)>,
}

impl Replay {
    fn now(&self) -> DateTime<Utc> {
        let sim = self.started.elapsed().as_secs_f64() * self.speed;
        self.from + chrono::Duration::milliseconds((sim * 1000.0) as i64)
    }

    fn from_args() -> Option<Self> {
        let args: Vec<String> = std::env::args().collect();
        let value = |name: &str| {
            args.iter()
                .position(|a| a == name)
                .and_then(|i| args.get(i + 1))
        };
        let starts: Vec<DateTime<Utc>> = value("--replay")?
            .split(',')
            .map(|s| {
                DateTime::parse_from_rfc3339(s)
                    .expect("--replay takes RFC 3339 times, e.g. 2026-10-21T15:20:00Z")
                    .with_timezone(&Utc)
            })
            .collect();
        let speed = value("--speed").map_or(10.0, |v| v.parse().expect("--speed number"));
        let minutes = value("--minutes").map_or(20, |v| v.parse().expect("--minutes number"));
        let stagger = value("--stagger").map_or(60, |v| v.parse().expect("--stagger seconds"));
        let from = starts[0];
        let windows = starts
            .iter()
            .zip(0..)
            .map(|(s, i)| (*s, from - *s + chrono::Duration::seconds(stagger * i)))
            .collect();
        Some(Self {
            from,
            until: from + chrono::Duration::minutes(minutes),
            speed,
            started: Instant::now(),
            windows,
        })
    }

    /// The channel's posts inside the replayed windows, from the scraped history. One attack
    /// also gets the hour before it; overlaid ones get only their own window and lose their
    /// "чисто" posts, which would wipe the other attacks' drones.
    fn posts(&self) -> Vec<Post> {
        static NOTE: LazyLock<Regex> =
            LazyLock::new(|| Regex::new(r"(\d{1,2}):(\d{2})(\s*[-–—])").unwrap());
        let history = std::fs::read_to_string("data/messages.jsonl")
            .expect("run --replay from the repository root (needs data/messages.jsonl)");
        let posts: Vec<serde_json::Value> = history
            .lines()
            .filter_map(|l| serde_json::from_str(l).ok())
            .collect();
        let mixed = self.windows.len() > 1;
        let span = self.until - self.from;
        let mut out = Vec::new();
        for (n, (start, shift)) in (0u64..).zip(&self.windows) {
            let since = if mixed {
                *start
            } else {
                *start - chrono::Duration::hours(1)
            };
            for v in &posts {
                let (Some(date), Some(id), Some(text)) =
                    (v["date"].as_str(), v["id"].as_u64(), v["text"].as_str())
                else {
                    continue;
                };
                let Ok(time) = DateTime::parse_from_rfc3339(date) else {
                    continue;
                };
                let time = time.with_timezone(&Utc);
                if time < since || time > *start + span {
                    continue;
                }
                let lower = text.to_lowercase();
                if mixed && (lower.contains("чисто") || lower.contains("відбій")) {
                    continue;
                }
                // "15:39 - У місті пролунав ВИБУХ": the note's local time moves with the post.
                let text = NOTE.replace_all(text, |c: &regex::Captures| {
                    let minutes = c[1].parse::<i64>().unwrap_or(0) * 60
                        + c[2].parse::<i64>().unwrap_or(0)
                        + shift.num_minutes();
                    let minutes = minutes.rem_euclid(24 * 60);
                    format!("{:02}:{:02}{}", minutes / 60, minutes % 60, &c[3])
                });
                out.push(Post {
                    channel: CHANNEL.into(),
                    id: n * 1_000_000 + id,
                    time: time + *shift,
                    text: text.into_owned(),
                });
            }
        }
        out
    }
}

/// The map's history from the channel's posts, with the places found while running: every
/// track, the drone list and each drone's replay; and the turns past drones made.
fn past(
    posts: Vec<Post>,
    home: Option<(f64, f64)>,
    from_label: &'static str,
) -> (web::Past, Vec<predict::RouteLeg>, Vec<predict::Passage>) {
    let mut gazetteer = gazetteer::Gazetteer::from_json(include_str!("../data/places.json"));
    for (spelling, place) in
        ultra_alert::lookup::Lookup::load(support_dir().join(PLACES_FOUND)).found()
    {
        gazetteer.add(&spelling, place);
    }
    let mut t = Tracker::new(
        gazetteer.clone(),
        Model::from_json(include_str!("../data/model.json")),
    );
    t.update(posts.clone());
    // The ways past drones came, so a drone known at one place only is not taken to turn back
    // (Tracker::turn); then the tracks again with them.
    let routes = t.route_legs();
    t.set_routes(routes.clone());
    let tracks = t.past_tracks();
    let drones = t.all_past_tracks();
    let passages = predict::passages(&drones);
    eprintln!(
        "history: {} posts, {} drones, {} turns",
        posts.len(),
        drones.len(),
        routes.len()
    );
    let past = web::Past {
        tracks: serde_json::to_string(&tracks).unwrap_or_default(),
        drones: drone_list(&drones, &gazetteer).to_string(),
        replay: {
            let routes = routes.clone();
            let passages = passages.clone();
            // From its first announcement until it could have flown on for a while after the
            // last.
            Box::new(move |n| {
                let d = drones.get(n)?;
                let last = d.times.last().copied().unwrap_or(d.start);
                let until = last + chrono::TimeDelta::minutes(10);
                Some(replay_frames(
                    &posts, &gazetteer, &routes, &passages, home, from_label, d.start, until,
                ))
            })
        },
    };
    (past, routes, passages)
}

/// Where places found while running are kept (`lookup`).
const PLACES_FOUND: &str = "places_found.json";
/// The channel's history on this Mac (ultra_alert::archive).
const HISTORY: &str = "history.bin";
/// Its seed built into the app (build.rs): empty unless built where the posts were scraped.
const SEED_HISTORY: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/seed_history.gz"));

/// The channel's posts from JSON lines like data/messages.jsonl.
fn history_posts(jsonl: &str) -> Vec<Post> {
    ultra_alert::archive::parse(jsonl, CHANNEL)
}

/// An explosion counts as a drone's when it came this soon after the drone's last
/// announcement and this close to where it was announced.
const HIT_AFTER: chrono::TimeDelta = chrono::TimeDelta::minutes(15);
const HIT_KM: f64 = 15.0;

/// Every drone in the channel history for the map's list, newest last: its path, when it
/// was announced first and last, and the explosion it ended in, if any.
fn drone_list(tracks: &[predict::PastTrack], gazetteer: &gazetteer::Gazetteer) -> serde_json::Value {
    let impacts: Vec<(DateTime<Utc>, String)> =
        serde_json::from_str::<Vec<serde_json::Value>>(include_str!("../data/impacts.json"))
            .unwrap_or_default()
            .iter()
            .filter_map(|i| {
                let t = DateTime::parse_from_rfc3339(i["t"].as_str()?).ok()?;
                Some((t.with_timezone(&Utc), i["place"].as_str()?.to_owned()))
            })
            .collect();
    tracks
        .iter()
        .map(|t| {
            let last = t.times.last().copied().unwrap_or(t.start);
            let (_, lat, lon) = t.path.last().cloned().unwrap_or_default();
            let hit = impacts.iter().find(|(at, place)| {
                *at >= t.start
                    && *at <= last + HIT_AFTER
                    && gazetteer
                        .place(place)
                        .is_some_and(|p| predict::haversine_km(p.lat, p.lon, lat, lon) <= HIT_KM)
            });
            // Districts after "на Вінницю" are the city again by name.
            let mut path: Vec<&String> = t.path.iter().map(|p| &p.0).collect();
            path.dedup();
            serde_json::json!({
                "start": t.start.to_rfc3339(),
                "last": last.to_rfc3339(),
                "slow": t.slow,
                "path": path,
                "line": t.line,
                "hit": hit.map(|(at, place)| serde_json::json!({ "t": at.to_rfc3339(), "place": place })),
            })
        })
        .collect()
}

/// A past drone played back: the map page's state every few simulated seconds, from a
/// tracker that sees only the posts up to each moment, as the app did live.
#[allow(clippy::too_many_arguments)]
fn replay_frames(
    posts: &[Post],
    gazetteer: &gazetteer::Gazetteer,
    routes: &[predict::RouteLeg],
    passages: &[predict::Passage],
    home: Option<(f64, f64)>,
    from_label: &str,
    start: DateTime<Utc>,
    end: DateTime<Utc>,
) -> String {
    let mut tracker = Tracker::new(
        gazetteer.clone(),
        Model::from_json(include_str!("../data/model.json")),
    );
    if let Some((lat, lon)) = home {
        tracker = tracker.with_home(lat, lon);
    }
    tracker.set_routes(routes.to_vec());
    // Only drones done hours before each moment count for its odds, as live: none of the
    // replayed attack.
    tracker.set_passages(passages.to_vec());
    tracker.update(
        posts
            .iter()
            // Six hours back, as the live app: what kind of drone the attack's posts said.
            .filter(|p| p.time >= start - chrono::TimeDelta::hours(6) && p.time <= end)
            .cloned(),
    );
    let (from, until) = (
        start - chrono::TimeDelta::seconds(30),
        end + chrono::TimeDelta::minutes(1),
    );
    // About a thousand frames at most; the page flies drones on between them.
    let step = ((until - from).num_seconds() / 1000).max(5);
    let frames: Vec<serde_json::Value> = (0..)
        .map(|i| from + chrono::TimeDelta::seconds(i * step))
        .take_while(|now| *now <= until)
        .map(|now| snapshot(&tracker, now, 1.0, from_label, &describe(&tracker, now, from_label)))
        .collect();
    serde_json::json!({ "step": step, "frames": frames }).to_string()
}

/// "Synth" from the iPhone-style alert tones macOS ships; that library is private, so if a macOS
/// update moves it the plain system sound plays instead.
const ALERT_SOUND: &str = "/System/Library/PrivateFrameworks/ToneLibrary.framework/Versions/A/Resources/AlertTones/Modern/Synth.m4r";
const FALLBACK_SOUND: &str = "/System/Library/Sounds/Sosumi.aiff";

/// Plays the alert sound and posts a notification; never blocks the UI.
fn alert(title: &str, text: &str) {
    eprintln!("  notification: {title} — {text}");
    let sound = if std::path::Path::new(ALERT_SOUND).exists() {
        ALERT_SOUND
    } else {
        FALLBACK_SOUND
    };
    let _ = std::process::Command::new("afplay").arg(sound).spawn();
    let script = format!(
        "display notification {} with title {}",
        applescript_string(text),
        applescript_string(title)
    );
    let _ = std::process::Command::new("osascript")
        .arg("-e")
        .arg(script)
        .spawn();
}

/// A quoted AppleScript string literal: text can never break out of it.
fn applescript_string(s: &str) -> String {
    format!("\"{}\"", s.replace('\\', "\\\\").replace('"', "\\\""))
}

fn replay_mode() -> bool {
    std::env::args().any(|a| a == "--replay")
}

fn support_dir() -> std::path::PathBuf {
    let home = std::env::var_os("HOME").unwrap_or_default();
    std::path::Path::new(&home).join("Library/Application Support/ultra-alert")
}

/// The alert already notified, so a restart mid-alert (an update, a crash) stays quiet.
/// When the situation was last cleared by hand, so a restart does not bring a stuck alert back.
fn dismissed_path() -> std::path::PathBuf {
    support_dir().join("dismissed.json")
}

fn load_dismissed() -> Option<DateTime<Utc>> {
    let v: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(dismissed_path()).ok()?).ok()?;
    Some(
        DateTime::parse_from_rfc3339(v["at"].as_str()?)
            .ok()?
            .with_timezone(&Utc),
    )
}

fn notified_path() -> std::path::PathBuf {
    support_dir().join("notified.json")
}

fn load_notified() -> Option<(DateTime<Utc>, bool)> {
    let v: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(notified_path()).ok()?).ok()?;
    let at = DateTime::parse_from_rfc3339(v["alert_time"].as_str()?).ok()?;
    Some((at.with_timezone(&Utc), v["districts"].as_bool()?))
}

fn predictions_path() -> std::path::PathBuf {
    support_dir().join("predictions.jsonl")
}

/// `{"home": [lat, lon]}` in the support dir, kept out of the repository: distances and ETAs
/// are measured to that point instead of the city centre.
fn home() -> Option<(f64, f64)> {
    let text = std::fs::read_to_string(support_dir().join("config.json")).ok()?;
    let v: serde_json::Value = serde_json::from_str(&text).ok()?;
    Some((v["home"][0].as_f64()?, v["home"][1].as_f64()?))
}

struct View {
    title: String,
    blink: Option<Duration>,
    status: String,
    eta: String,
}

/// Drones over the oblast, none sent here: when the nearest could arrive if it turned now.
fn if_city(tracker: &Tracker, now: DateTime<Utc>) -> String {
    let Status::Drones { .. } = tracker.status(now) else {
        return "Якщо на Вінницю: —".into();
    };
    // How likely, from the past drones that were where these are.
    let odds = tracker.city_odds(now).map_or_else(String::new, |(p, n)| {
        format!(" · ймовірність {:.0}% (з {n} схожих)", p * 100.0)
    });
    match tracker.if_heading(now) {
        Some((eta, from, km)) => {
            let left = (eta - now).num_seconds().max(0);
            format!(
                "Якщо на Вінницю: ≈ {} (через {}:{:02}; {from}, {km:.0} км){odds}",
                eta.with_timezone(&Local).format("%H:%M"),
                left / 60,
                left % 60
            )
        }
        None => format!("Якщо на Вінницю: —{odds}"),
    }
}

/// "Карта ↗", with how far the channel's history is while the first launch fetches it.
fn map_text(state: &web::State) -> String {
    let ready = state.past.lock().is_ok_and(|p| p.is_some());
    let progress: serde_json::Value = state
        .progress
        .lock()
        .ok()
        .and_then(|p| serde_json::from_str(&p).ok())
        .unwrap_or_default();
    match (ready, progress["stage"].as_str(), progress["p"].as_f64()) {
        (false, Some("fetch"), Some(p)) => format!("Карта ↗ (історія {:.0}%)", p * 100.0),
        (false, Some("build"), _) => "Карта ↗ (обробка історії…)".into(),
        _ => "Карта ↗".into(),
    }
}

/// The menu's text for the tracker's state at `now`.
fn describe(tracker: &Tracker, now: DateTime<Utc>, from_label: &str) -> View {
    let local =
        |t: chrono::DateTime<Utc>| t.with_timezone(&Local).format("%H:%M:%S").to_string();
    match tracker.status(now) {
        Status::Quiet => View {
            title: "—".into(),
            blink: None,
            status: "Загроз немає".into(),
            eta: String::new(),
        },
        Status::Drones { nearest } => {
            let (title, status) = match nearest {
                // Course known: live dead-reckoned distance, changes every ~1 s.
                Some(e) if e.course.is_some() => (
                    format!("{:.1}", e.km),
                    format!(
                        "Шахед → {}: ≈{:.1} км {}, {}",
                        e.heading_to,
                        e.km,
                        from_label,
                        if e.closing {
                            "наближається"
                        } else {
                            "віддаляється"
                        }
                    ),
                ),
                Some(e) => (
                    format!("{:.0}", e.km),
                    format!(
                        "Шахед → {} ({:.0} км {}), напрямок невідомий",
                        e.heading_to, e.km, from_label
                    ),
                ),
                None => ("·".into(), "Шахеди в області".into()),
            };
            // Unlikely to come here: the odds it does; likelier: how far it is.
            let title = match tracker.city_odds(now) {
                Some((p, _)) if p < KM_FROM_ODDS => format!("{:.0}%", p * 100.0),
                _ => title,
            };
            let tracked = tracker.drones(now).len();
            let status = if tracked > 1 {
                format!("{status} · у треках: {tracked}")
            } else {
                status
            };
            View {
                title,
                blink: None,
                status,
                eta: String::new(),
            }
        }
        Status::Threat(p) => {
            let left = (p.eta - now).num_seconds();
            let title = if left > 0 {
                format!("{}:{:02}", left / 60, left % 60)
            } else {
                "0:00".into()
            };
            let blink = Some(Duration::from_millis(match left {
                181.. => 750,
                61..=180 => 500,
                _ => 250,
            }));
            let from = match &p.basis {
                Basis::Waypoint(place, km) => format!("від {place}, {km:.0} км"),
                Basis::Districts => "оператор назвав райони міста".into(),
                Basis::Alert => "канал: «на Вінницю»".into(),
                Basis::Course { via, miss_km } => {
                    format!("каналу ще не було: курс через {via} проходить за {miss_km:.0} км")
                }
            };
            View {
                title,
                blink,
                status: format!("Шахед на Вінницю (тривога о {})", local(p.alert_time)),
                eta: format!(
                    "Можливий приліт ≈ {} ± {:.0} с ({from})",
                    local(p.eta),
                    p.sigma_s
                ),
            }
        }
        // Better a false alarm than a missed hit, but no countdown to a drone not sent here.
        Status::Possible { at } => View {
            title: "!".into(),
            blink: Some(Duration::from_millis(1000)),
            status: format!("Може піти на Вінницю (о {})", local(at)),
            eta: "Відліку немає: канал ще не написав «на Вінницю»".into(),
        },
        Status::Impact { at } => View {
            title: "!".into(),
            blink: None,
            status: format!("Вибух у Вінниці о {}", local(at)),
            eta: String::new(),
        },
    }
}

/// The map page's state: the menu's text plus every tracked drone with its path. `rate`:
/// simulated seconds per real second, so the page moves drones between updates.
fn snapshot(tracker: &Tracker, now: DateTime<Utc>, rate: f64, from_label: &str, v: &View) -> serde_json::Value {
    let status = tracker.status(now);
    let target = tracker.target();
    let drones: Vec<_> = tracker
        .drones(now)
        .into_iter()
        .map(|e| {
            serde_json::json!({
                "lat": e.lat, "lon": e.lon, "course": e.course, "km": e.km,
                "heading_to": e.heading_to, "closing": e.closing, "slow": e.slow,
                "speed_kmh": e.speed_kmh, "lead_km": e.lead_km, "path": e.path,
            })
        })
        .collect();
    serde_json::json!({
        "now": now.to_rfc3339(),
        "rate": rate,
        "home": {
            "lat": target.lat, "lon": target.lon,
            "label": if from_label == "від дому" { "Дім" } else { gazetteer::CITY },
            "from": from_label,
        },
        "title": v.title,
        "status": v.status,
        // Odds a drone over the oblast is sent here; with the "if it were" line under them.
        "odds": tracker.city_odds(now).map(|(p, n)| serde_json::json!({ "p": p, "n": n })),
        "if_city": if_city(tracker, now),
        "eta": v.eta,
        "speed_kmh": tracker.model.speed_kmh,
        "threat": match &status {
            Status::Threat(p) => serde_json::json!({
                "eta": p.eta.to_rfc3339(), "sigma_s": p.sigma_s,
            }),
            _ => serde_json::Value::Null,
        },
        "possible": match &status {
            Status::Possible { at } => at.to_rfc3339().into(),
            _ => serde_json::Value::Null,
        },
        "impact": match status {
            Status::Impact { at } => at.to_rfc3339().into(),
            _ => serde_json::Value::Null,
        },
        "drones": drones,
        "post": tracker.latest_post(now).map(|p| serde_json::json!({
            "time": p.time.to_rfc3339(),
            "text": p.text.replace('\n', " ").chars().take(200).collect::<String>(),
            "channel": (p.channel != CHANNEL).then(|| p.channel.clone()),
        })),
    })
}

impl App {
    fn now(&self) -> DateTime<Utc> {
        self.replay.as_ref().map_or_else(Utc::now, Replay::now)
    }

    fn view(&self) -> View {
        let mut v = describe(&self.tracker, self.now(), self.from_label);
        if self.last_ok.is_none_or(|t| t.elapsed() > STALE)
            && let Some(err) = &self.last_error
        {
            v.title = format!("{}?", v.title);
            v.status = format!("{} — немає даних: {err}", v.status);
        }
        v
    }


    /// Sends recent posts about drones to the lookup of unknown places, each once.
    fn look_up(&mut self, posts: &[Post]) {
        let Some(tx) = &self.lookup else {
            return;
        };
        let since = Utc::now() - LOOKUP_AGE;
        for p in posts {
            let lower = p.text.to_lowercase();
            if p.time >= since
                && LOOKUP_WORDS.iter().any(|w| lower.contains(w))
                && self.looked_up.insert((p.channel.clone(), p.id))
            {
                let _ = tx.send(p.text.clone());
            }
        }
    }

    /// Appends every new prediction to predictions.jsonl, which tools/daily.py scores against
    /// the impacts that followed. A threat is re-logged when its ETA moves by 5 s or more.
    fn record(&mut self) {
        if self.replay.is_some() {
            return;
        }
        let now = Utc::now();
        let mut rec = match self.tracker.status(now) {
            Status::Quiet => serde_json::json!({ "kind": "quiet" }),
            Status::Drones { nearest } => serde_json::json!({
                "kind": "drones",
                "heading_to": nearest.map(|e| e.heading_to),
            }),
            Status::Threat(p) => serde_json::json!({
                "kind": "threat",
                "eta": p.eta.to_rfc3339(),
                "eta_bucket": p.eta.timestamp() / 5,
                "sigma_s": p.sigma_s,
                "basis": format!("{:?}", p.basis),
                "alert_time": p.alert_time.to_rfc3339(),
            }),
            Status::Possible { at } => serde_json::json!({ "kind": "possible", "at": at.to_rfc3339() }),
            Status::Impact { at } => serde_json::json!({ "kind": "impact", "at": at.to_rfc3339() }),
        };
        let key = rec.to_string();
        if key == self.recorded {
            return;
        }
        self.recorded = key;
        rec["logged_at"] = now.to_rfc3339().into();
        rec["latest_post"] = self.tracker.latest_post(now).map(|p| p.id).into();
        let path = predictions_path();
        let line = format!("{rec}\n");
        let written = std::fs::create_dir_all(path.parent().expect("has parent")).and_then(|()| {
            use std::io::Write;
            std::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open(&path)?
                .write_all(line.as_bytes())
        });
        if let Err(err) = written {
            eprintln!("cannot write {}: {err}", path.display());
        }
    }

    /// Clears the situation now: a stuck alert (a post read wrong) goes, new posts count.
    fn dismiss(&mut self) {
        let at = self.now();
        self.tracker.dismiss(at);
        eprintln!(
            "{} situation cleared by hand",
            at.with_timezone(&Local).format("%H:%M:%S")
        );
        if self.replay.is_none() {
            let json = serde_json::json!({ "at": at.to_rfc3339() });
            if let Err(err) = std::fs::write(dismissed_path(), json.to_string()) {
                eprintln!("cannot write {}: {err}", dismissed_path().display());
            }
        }
        self.refresh();
    }

    fn set_notified(&mut self, alert_time: DateTime<Utc>, districts: bool) {
        self.notified = Some((alert_time, districts));
        if self.replay.is_some() {
            return;
        }
        let json = serde_json::json!({
            "alert_time": alert_time.to_rfc3339(),
            "districts": districts,
        });
        if let Err(err) = std::fs::write(notified_path(), json.to_string()) {
            eprintln!("cannot write {}: {err}", notified_path().display());
        }
    }

    /// Sound and a macOS notification when a threat to the city starts and when the
    /// operator names city districts; once each per alert. Also when drones may go for the
    /// city, once.
    fn notify(&mut self) {
        let p = match self.tracker.status(self.now()) {
            Status::Threat(p) => p,
            Status::Possible { at } => {
                if self.notified.as_ref().is_none_or(|(was, _)| *was != at) {
                    self.set_notified(at, false);
                    alert(
                        "Може піти на Вінницю",
                        "Відліку ще немає: канал не написав «на Вінницю»",
                    );
                }
                return;
            }
            _ => return,
        };
        let eta = p.eta.with_timezone(&Local).format("%H:%M:%S");
        let districts = p.basis == Basis::Districts;
        // A threat that starts with districts gets only the districts notification.
        if !districts
            && self
                .notified
                .as_ref()
                .is_none_or(|(at, _)| *at != p.alert_time)
        {
            self.set_notified(p.alert_time, false);
            alert(
                "Шахед на Вінницю",
                &format!("Можливий приліт ≈ {eta} ± {:.0} с", p.sigma_s),
            );
        }
        if districts
            && self
                .notified
                .as_ref()
                .is_none_or(|(at, d)| *at != p.alert_time || !d)
        {
            self.set_notified(p.alert_time, true);
            alert(
                "Шахед підлітає до Вінниці",
                &format!("Оператор назвав райони міста. Приліт ≈ {eta}"),
            );
        }
    }

    fn publish(&self, v: &View) {
        let rate = self.replay.as_ref().map_or(1.0, |r| r.speed);
        let state = snapshot(&self.tracker, self.now(), rate, self.from_label, v);
        self.map_state.set(state.to_string());
    }

    fn refresh(&mut self) {
        if self.last_ok.is_some() {
            self.record();
            self.notify();
        }
        let v = self.view();
        self.publish(&v);
        if v.blink != self.blink {
            self.blink = v.blink;
            self.blink_visible = true;
            self.next_blink = Instant::now() + v.blink.unwrap_or(Duration::from_secs(3600));
        }
        // Log when the situation changes, not on every countdown tick or 0.1 km of movement.
        let situation: String = v
            .status
            .chars()
            .filter(|c| !c.is_ascii_digit() && *c != '.')
            .collect();
        if situation != self.logged {
            eprintln!(
                "{} [{}] {} {} | {}",
                self.now().with_timezone(&Local).format("%H:%M:%S"),
                v.title,
                v.status,
                v.eta,
                if_city(&self.tracker, self.now())
            );
            self.logged = situation;
        }
        if v.title != self.title {
            self.title = v.title;
            if let Some(tray) = &self.tray {
                let _ = tray.set_icon(Some(icon::render(&self.title, self.blink_visible)));
            }
        }
        if let Some(tray) = &self.tray {
            let _ = tray.set_tooltip(Some(&v.status));
        }
        self.status_item.set_text(&v.status);
        self.eta_item.set_text(if v.eta.is_empty() {
            "Прогноз: —"
        } else {
            &v.eta
        });
        if let Some(post) = self.tracker.latest_post(self.now()) {
            let text: String = post.text.replace('\n', " ").chars().take(90).collect();
            // A fallback channel says which one it is.
            let from = if post.channel == CHANNEL {
                String::new()
            } else {
                format!("резерв @{} · ", post.channel)
            };
            self.post_item.set_text(format!(
                "{} · {from}{text}",
                post.time.with_timezone(&Local).format("%H:%M")
            ));
        }
        self.if_item.set_text(if_city(&self.tracker, self.now()));
        self.map_item.set_text(map_text(&self.map_state));
    }
}

impl ApplicationHandler<UserEvent> for App {
    fn resumed(&mut self, _event_loop: &ActiveEventLoop) {
        if self.tray.is_some() {
            return;
        }
        let menu = Menu::new();
        for item in [&self.status_item, &self.eta_item, &self.post_item, &self.if_item] {
            menu.append(item).expect("menu item");
        }
        menu.append(&PredefinedMenuItem::separator())
            .expect("separator");
        menu.append(&self.map_item).expect("map item");
        menu.append(&self.model_item).expect("model item");
        menu.append(&self.source_item).expect("source item");
        menu.append(&PredefinedMenuItem::separator())
            .expect("separator");
        menu.append(&self.dismiss_item).expect("dismiss item");
        menu.append(&self.quit_item).expect("quit item");
        self.tray = Some(
            TrayIconBuilder::new()
                .with_icon(icon::render(&self.title, true))
                .with_tooltip("Ultra Alert — Вінниця")
                .with_menu(Box::new(menu))
                // Left click: the menu; right click: clear the situation.
                .with_menu_on_right_click(false)
                .build()
                .expect("create macOS status item"),
        );
    }

    fn user_event(&mut self, event_loop: &ActiveEventLoop, event: UserEvent) {
        match event {
            UserEvent::Posts(posts) => {
                self.last_ok = Some(Instant::now());
                self.last_error = None;
                self.look_up(&posts);
                if let Some(tx) = &self.history {
                    let _ = tx.send(posts.clone());
                }
                self.tracker.update(posts);
                self.refresh();
            }
            UserEvent::FetchError(err) => self.last_error = Some(err),
            UserEvent::FallbackPosts(posts) => {
                self.look_up(&posts);
                if self.tracker.update(posts) {
                    self.refresh();
                }
            }
            UserEvent::Routes(routes, passages) => {
                self.tracker.set_routes(routes);
                self.tracker.set_passages(passages);
            }
            UserEvent::Places(places) => {
                for (spelling, place) in places {
                    eprintln!("found {spelling}: {} {:.4},{:.4}", place.name, place.lat, place.lon);
                    self.tracker.add_place(&spelling, place);
                }
                self.refresh();
            }
            UserEvent::Menu(e) if e.id == self.map_item.id() => {
                if let Some(url) = &self.map_url {
                    let _ = std::process::Command::new("open").arg(url).spawn();
                }
            }
            UserEvent::Menu(e) if e.id == self.source_item.id() => {
                let _ = std::process::Command::new("open")
                    .arg(format!("https://t.me/{CHANNEL}"))
                    .spawn();
            }
            UserEvent::Menu(e) if e.id == self.dismiss_item.id() => self.dismiss(),
            UserEvent::Tray(TrayIconEvent::Click {
                button: MouseButton::Right,
                button_state: MouseButtonState::Up,
                ..
            }) => self.dismiss(),
            UserEvent::Tray(_) => {}
            UserEvent::Menu(e) if e.id == self.quit_item.id() => event_loop.exit(),
            UserEvent::Menu(_) => {}
        }
    }

    fn window_event(&mut self, _: &ActiveEventLoop, _: WindowId, _: winit::event::WindowEvent) {}

    fn about_to_wait(&mut self, event_loop: &ActiveEventLoop) {
        if let Some(r) = &self.replay
            && r.now() > r.until
        {
            if !event_loop.exiting() {
                eprintln!("replay finished");
            }
            event_loop.exit();
            return;
        }
        let now = Instant::now();
        if now >= self.next_refresh {
            self.refresh();
            self.next_refresh = now + REFRESH;
        }
        if let Some(interval) = self.blink
            && now >= self.next_blink
        {
            self.blink_visible = !self.blink_visible;
            self.next_blink = now + interval;
            if let Some(tray) = &self.tray {
                let _ = tray.set_icon(Some(icon::render(&self.title, self.blink_visible)));
            }
        }
        let wake = if self.blink.is_some() {
            self.next_refresh.min(self.next_blink)
        } else {
            self.next_refresh
        };
        event_loop.set_control_flow(ControlFlow::WaitUntil(wake));
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Data pipeline subcommands (run from the repository root); no subcommand: the app.
    let args: Vec<String> = std::env::args().collect();
    if let Some(cmd) = args.get(1).filter(|a| !a.starts_with("--")) {
        ultra_alert::pipeline::cli(cmd, &args[2..])?;
        return Ok(());
    }
    // Accessory: menu-bar only, no Dock icon even when run as a bare binary.
    let event_loop = EventLoop::<UserEvent>::with_user_event()
        .with_activation_policy(ActivationPolicy::Accessory)
        .build()?;
    let proxy = event_loop.create_proxy();

    let routes_proxy = proxy.clone();
    let places_proxy = proxy.clone();
    let menu_proxy = proxy.clone();
    MenuEvent::set_event_handler(Some(move |event| {
        let _ = menu_proxy.send_event(UserEvent::Menu(event));
    }));
    let tray_proxy = proxy.clone();
    TrayIconEvent::set_event_handler(Some(move |event| {
        let _ = tray_proxy.send_event(UserEvent::Tray(event));
    }));

    let replay = Replay::from_args();
    // A replay feeds recorded posts instead of polling the channel.
    std::thread::spawn(move || {
        if replay_mode() {
            return;
        }
        let runtime = tokio::runtime::Runtime::new().expect("Tokio runtime");
        runtime.block_on(async move {
            let client = reqwest::Client::new();
            let mut ticker = tokio::time::interval(POLL);
            for tick in 0usize.. {
                ticker.tick().await;
                let event = match telegram::fetch(&client, CHANNEL).await {
                    Ok(posts) => UserEvent::Posts(posts),
                    Err(err) => UserEvent::FetchError(err.to_string()),
                };
                if proxy.send_event(event).is_err() {
                    break;
                }
                // One fallback channel per tick; the tracker reads them only while the main
                // one is silent, and their failures do not make the data stale.
                let fallback = predict::FALLBACK_CHANNELS[tick % predict::FALLBACK_CHANNELS.len()];
                if let Ok(posts) = telegram::fetch(&client, fallback).await
                    && proxy.send_event(UserEvent::FallbackPosts(posts)).is_err()
                {
                    break;
                }
            }
        });
    });

    let model = Model::from_json(include_str!("../data/model.json"));
    let model_text = format!(
        "Модель: {:.0} км/год, «на X» за {:.0} км + {:.2}·d до X, райони за {:.0} км",
        model.speed_kmh, model.lead_km, model.lead_per_km, model.district_lead_km
    );
    let dismissed = if replay.is_some() {
        None
    } else {
        load_dismissed()
    };
    let mut tracker = Tracker::new(
        gazetteer::Gazetteer::from_json(include_str!("../data/places.json")),
        model,
    );
    let home = home();
    if let Some((lat, lon)) = home {
        tracker = tracker.with_home(lat, lon);
    }
    if let Some(at) = dismissed {
        tracker.dismiss(at);
    }
    // The last hours of the channel from this Mac's history: the kind of drone the attack's
    // posts said and the drones posted before the newest page the first fetch brings.
    if !replay_mode() {
        let mut history = ultra_alert::archive::Archive::load(support_dir().join(HISTORY), CHANNEL);
        history.seed(SEED_HISTORY);
        let since = Utc::now() - chrono::TimeDelta::hours(6);
        tracker.update(history.posts().into_iter().filter(|p| p.time >= since));
    }
    if let Some(r) = &replay {
        tracker.update(r.posts());
        eprintln!(
            "replaying {} – {} at ×{}",
            r.from.with_timezone(&Local).format("%m-%d %H:%M"),
            r.until.with_timezone(&Local).format("%H:%M"),
            r.speed
        );
    }
    // Places found in earlier runs, and a worker that looks up the ones posts name next;
    // a replay reads the history with the gazetteer it was built with.
    let lookup = (!replay_mode()).then(|| {
        let mut found = ultra_alert::lookup::Lookup::load(support_dir().join(PLACES_FOUND));
        for (spelling, place) in found.found() {
            tracker.add_place(&spelling, place);
        }
        let mut gazetteer = tracker.gazetteer().clone();
        let (tx, rx) = std::sync::mpsc::channel::<String>();
        std::thread::spawn(move || {
            let runtime = tokio::runtime::Runtime::new().expect("Tokio runtime");
            for text in rx {
                let places = runtime.block_on(found.resolve(&mut gazetteer, &text));
                if !places.is_empty() && places_proxy.send_event(UserEvent::Places(places)).is_err() {
                    break;
                }
            }
        });
        tx
    });
    let from_label = if home.is_some() {
        "від дому"
    } else {
        "від Вінниці"
    };
    let map_state = web::State::default();
    // The map's history layer, the turns and the odds: every track in the channel's history on
    // this Mac (fetched on first launch, then the posts as they come), rebuilt a minute after
    // new ones.
    let history_state = map_state.clone();
    let (history_tx, history_rx) = std::sync::mpsc::channel::<Vec<Post>>();
    std::thread::spawn(move || {
        let mut archive =
            ultra_alert::archive::Archive::load(support_dir().join(HISTORY), CHANNEL);
        let seeded = archive.seed(SEED_HISTORY);
        if seeded > 0 {
            eprintln!("history: {seeded} posts from the app");
        }
        // What earlier versions kept beside the history built into them.
        if let Ok(text) = std::fs::read_to_string(support_dir().join("messages_live.jsonl")) {
            archive.add(history_posts(&text));
        }
        if !replay_mode() {
            let since = DateTime::parse_from_rfc3339(&format!("{}T00:00:00Z", ultra_alert::pipeline::PPO_SINCE))
                .expect("PPO_SINCE")
                .with_timezone(&Utc);
            let runtime = tokio::runtime::Runtime::new().expect("Tokio runtime");
            let progress_state = history_state.clone();
            let mut progress = |p: f64, oldest: DateTime<Utc>| {
                if let Ok(mut s) = progress_state.progress.lock() {
                    *s = serde_json::json!({ "stage": "fetch", "p": p, "oldest": oldest.to_rfc3339() })
                        .to_string();
                }
            };
            if let Ok(mut s) = history_state.progress.lock() {
                *s = serde_json::json!({ "stage": "fetch", "p": 0.0 }).to_string();
            }
            let fetched =
                runtime.block_on(archive.catch_up(&reqwest::Client::new(), since, &mut progress));
            eprintln!("history: {fetched} posts fetched");
            if let Err(err) = archive.save() {
                eprintln!("history not saved: {err}");
            }
        }
        loop {
            if let Ok(mut s) = history_state.progress.lock() {
                *s = serde_json::json!({ "stage": "build" }).to_string();
            }
            let (past, routes, passages) = past(archive.posts(), home, from_label);
            if let Ok(mut p) = history_state.past.lock() {
                *p = Some(std::sync::Arc::new(past));
            }
            // The live tracker flies drones on past their waypoints along these.
            if routes_proxy.send_event(UserEvent::Routes(routes, passages)).is_err() {
                break;
            }
            loop {
                let Ok(batch) = history_rx.recv() else {
                    return;
                };
                if archive.add(batch) {
                    break;
                }
            }
            std::thread::sleep(Duration::from_secs(60));
            while let Ok(batch) = history_rx.try_recv() {
                archive.add(batch);
            }
            if let Err(err) = archive.save() {
                eprintln!("history not saved: {err}");
            }
        }
    });
    let map_url = web::serve(map_state.clone())
        .inspect(|url| eprintln!("map: {url}"))
        .inspect_err(|err| eprintln!("map server: {err}"))
        .ok();
    let mut app = App {
        tracker,
        tray: None,
        status_item: MenuItem::new("Завантаження…", false, None),
        eta_item: MenuItem::new("Прогноз: —", false, None),
        post_item: MenuItem::new("—", false, None),
        if_item: MenuItem::new("Якщо на Вінницю: —", false, None),
        model_item: MenuItem::new(model_text, false, None),
        map_item: MenuItem::new("Карта ↗", map_url.is_some(), None),
        source_item: MenuItem::new(format!("Канал t.me/{CHANNEL} ↗"), true, None),
        dismiss_item: MenuItem::new("Скинути ситуацію (правий клік по значку)", true, None),
        quit_item: MenuItem::new("Вийти", true, None),
        title: "…".into(),
        blink: None,
        blink_visible: true,
        next_blink: Instant::now(),
        next_refresh: Instant::now(),
        last_ok: replay.as_ref().map(|_| Instant::now()),
        last_error: None,
        logged: String::new(),
        recorded: String::new(),
        notified: if replay.is_some() {
            None
        } else {
            load_notified()
        },
        replay,
        from_label,
        map_state,
        map_url,
        lookup,
        looked_up: std::collections::HashSet::new(),
        history: (!replay_mode()).then_some(history_tx),
    };
    event_loop.run_app(&mut app)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::applescript_string;

    #[test]
    fn applescript_string_escapes_quotes_and_backslashes() {
        assert_eq!(applescript_string("ok"), "\"ok\"");
        assert_eq!(
            applescript_string(r#"a" & do shell script "x" & "\"#),
            r#""a\" & do shell script \"x\" & \"\\""#
        );
    }
}
