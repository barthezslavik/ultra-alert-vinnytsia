//! Turns channel posts into live drone positions, a threat status and an impact ETA.
//!
//! One model for both: every post "на X" says the drone is L km short of X and heading
//! there, L = lead_km + lead_per_km · d(X, Vinnytsia) (the operator announces far places
//! earlier). Between posts the drone flies on at `speed_kmh`. The ETA at a target is when
//! that path reaches it: first "на X" + (L + d(X, target)) / v, or + L / v when X is the
//! target. Districts of Vinnytsia named in a post are a waypoint at the city with their own L.
//! Parameters are fitted on the channel history (tests `fit_trajectory`, `backtest`).

use crate::gazetteer::{CITY, Gazetteer, Mention, Place, Role};
use crate::telegram::Post;
use chrono::{DateTime, Duration, Local, NaiveTime, TimeZone, Utc};
use regex::Regex;
use serde::Deserialize;
use std::collections::HashMap;
use std::sync::LazyLock;

pub mod eval;
pub mod kalman;

/// Regular (propeller) Shahed ground speed, km/h.
const SLOW_SPEED_KMH: f64 = 180.0;
const HISTORY: Duration = Duration::minutes(60);
const ALERT_TTL: Duration = Duration::minutes(20);
const IMPACT_SHOWN: Duration = Duration::minutes(3);
/// A track not mentioned for this long has left, been shot down or merged into another.
const TRACK_TTL: Duration = Duration::minutes(20);
/// Dead reckoning past the last waypoint stops after this long.
const MAX_EXTRAPOLATION: Duration = Duration::minutes(15);
/// A new waypoint joins a track when the implied speed and turn are plausible.
const LINK_SPEED_KMH: std::ops::RangeInclusive<f64> = 150.0..=750.0;
/// Nor a waypoint farther than this from the track's last one: posts name the next town.
const MAX_LEG_KM: f64 = 100.0;
/// Distance allowed beyond the speed range: a couple of minutes of posting jitter, and posts name
/// where a drone heads next, not where it is ("на Гайсин", 5 min later "на Брацлав" 33 km on is
/// one slow drone; 5 km made it the one by Бар, 95 km back east).
const LINK_SLACK_KM: f64 = 15.0;
/// The same for a slow (ordinary) Shahed on either end of the link.
const SLOW_LINK_SPEED_KMH: std::ops::RangeInclusive<f64> = 60.0..=250.0;
const MAX_TURN_DEG: f64 = 100.0;
/// Turning for the city: posts name the city when the drone swings for it, the turn can be
/// sharper than between two towns ("на Вороновицю" from Тиврів, then the city: 101°).
const MAX_CITY_TURN_DEG: f64 = 135.0;
/// A course comes from a place at least this far behind the waypoint; closer ones (the city
/// and its districts) are not a bearing.
const COURSE_MIN_KM: f64 = 8.0;
/// A districts post or an explosion with no track heading for that place attaches to the
/// latest track whose waypoint is this close to it.
const NEAR_KM: f64 = 60.0;

#[derive(Debug, Clone, Deserialize)]
pub struct Model {
    /// Jet-Shahed ground speed, km/h.
    pub speed_kmh: f64,
    /// L at Vinnytsia itself, km.
    pub lead_km: f64,
    /// Extra L per km of the waypoint's distance from Vinnytsia.
    pub lead_per_km: f64,
    /// L for a post naming city districts, km.
    pub district_lead_km: f64,
    /// ETA uncertainty (robust σ of backtest errors) by what the ETA rests on, s.
    pub sigma_waypoint_s: f64,
    pub sigma_alert_s: f64,
    pub sigma_district_s: f64,
    /// Warn from the trajectory alone when a drone's course passes within this many km of the
    /// target, at most `course_range_km` away; 0 disables it.
    #[serde(default)]
    pub course_corridor_km: f64,
    #[serde(default)]
    pub course_range_km: f64,
    /// How much later than vinnytsiappo a channel posts the same thing, s: its posts find
    /// the drone that much closer, so L shrinks by the distance flown meanwhile.
    #[serde(default)]
    pub channel_delay_s: HashMap<String, f64>,
    /// Past the waypoint, fly on along the mean course of past drones that came through within
    /// `route_radius_km` of it on a course within `route_angle_deg`, when there were at least
    /// `route_min` of them; 0 km disables it.
    #[serde(default)]
    pub route_radius_km: f64,
    #[serde(default)]
    pub route_angle_deg: f64,
    #[serde(default)]
    pub route_min: usize,
    /// A fallback channel (vinnytsiarealll, Захід Головне, …) is read only after this long
    /// without a post from the main one, s.
    #[serde(default = "fallback_after_s")]
    pub fallback_after_s: f64,
    /// How much later than the explosion its post comes out, s: the backtest and the refit
    /// compare ETAs with the post time less this.
    #[serde(default)]
    pub impact_post_delay_s: f64,
    /// A city post joining a drone heading for another place is a turn: the drone keeps its
    /// dead-reckoned distance instead of jumping to L.
    #[serde(default)]
    pub city_turns: bool,
    /// The same for the first districts post.
    #[serde(default)]
    pub district_turns: bool,
    /// Track drones with the Kalman filter (`kalman.rs`) instead of jumping to each post.
    #[serde(default)]
    pub kalman: bool,
    /// Its noise: random acceleration, (km/h)² per hour; where a post puts the drone, σ along
    /// its way and across, km (districts: `kf_district_km`); its velocity, σ km/h.
    #[serde(default)]
    pub kf_accel: f64,
    #[serde(default)]
    pub kf_along_km: f64,
    #[serde(default)]
    pub kf_cross_km: f64,
    #[serde(default)]
    pub kf_district_km: f64,
    #[serde(default)]
    pub kf_velocity_kmh: f64,
}

fn fallback_after_s() -> f64 {
    1200.0
}

#[derive(Deserialize)]
struct ModelFile {
    trajectory: Model,
}

impl Model {
    pub fn from_json(json: &str) -> Self {
        serde_json::from_str::<ModelFile>(json)
            .expect("model.json")
            .trajectory
    }
}

#[derive(Debug, Clone, PartialEq)]
enum Signal {
    Clear,
    /// Where the explosion was heard; None when the post names no place.
    Explosion {
        place: Option<String>,
    },
    /// `district`: names parts of Vinnytsia, i.e. the drone is about to reach the city.
    /// `district_at`: the centre of the named districts when the list is short enough to say
    /// where in the city; None for "the whole city".
    /// `summary`: "Обстановка: ▪️1 на A; ▪️2 на B" lists several drones, one place each;
    /// `slow_each` is then the type of each mention, from its own line.
    /// `fresh`: "+1 на X", "ще один на X": a drone other than the ones already tracked.
    /// `maybe`: "можуть піти на ВІННИЦЮ": where it may go, not where it is sent.
    /// `early`: "залітає на Вінниччину, Вінниця вже може займати укриття": sent to the city
    /// while still at the oblast's edge.
    /// `passed`: "пролетів Жмеринку": a place it has flown past, not where it heads.
    /// `typed`: the post says what kind ("реактивний", "звичайний", "ударний"); "1 Шахед на X"
    /// does not, and is of the kind the attack's last typed post said.
    Drone {
        slow: bool,
        typed: bool,
        fresh: bool,
        maybe: bool,
        early: bool,
        passed: bool,
        route: Vec<Mention>,
        district: bool,
        district_at: Option<(f64, f64)>,
        summary: bool,
        slow_each: Vec<bool>,
    },
    Other,
}

#[derive(Debug, Clone)]
struct Event {
    time: DateTime<Utc>,
    signal: Signal,
    channel: String,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Basis {
    /// Waypoint on the approach and its distance to the target, km.
    Waypoint(String, f64),
    /// The operator named city districts.
    Districts,
    /// The last waypoint is the target itself ("на Вінницю").
    Alert,
    /// No post about the target yet: the drone's course passes `miss_km` from it.
    Course { via: String, miss_km: f64 },
}

#[derive(Debug, Clone, PartialEq)]
pub struct Prediction {
    pub eta: DateTime<Utc>,
    pub sigma_s: f64,
    pub basis: Basis,
    pub alert_time: DateTime<Utc>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Status {
    Quiet,
    /// Drones over the oblast, none announced for the city; the live position of the one
    /// closest to the city.
    Drones {
        nearest: Option<Estimate>,
    },
    Threat(Prediction),
    /// "Ще кілька реактивних можуть піти на ВІННИЦЮ": the city may be next, no drone sent to it
    /// yet, so no countdown.
    Possible {
        at: DateTime<Utc>,
    },
    Impact {
        at: DateTime<Utc>,
    },
}

/// One drone's whole tracked path, for drawing the history.
#[derive(Debug, Clone, serde::Serialize)]
pub struct PastTrack {
    pub start: DateTime<Utc>,
    pub slow: bool,
    /// Where it came from (a town, not a neighbouring oblast), then its waypoints: (name,
    /// lat, lon).
    pub path: Vec<(String, f64, f64)>,
    /// When each waypoint was first announced (the origin has none).
    pub times: Vec<DateTime<Utc>>,
    /// The flight as the model sees it, smoothed: (lat, lon).
    pub line: Vec<(f64, f64)>,
}

/// Estimated live position of one tracked drone.
#[derive(Debug, Clone, PartialEq)]
pub struct Estimate {
    /// Last waypoint announced for it ("на X").
    pub heading_to: String,
    pub lat: f64,
    pub lon: f64,
    /// From the estimated position to the target, km.
    pub km: f64,
    /// Degrees from north; None when only one waypoint and no origin are known, and the
    /// position is then just the waypoint itself.
    pub course: Option<f64>,
    pub closing: bool,
    pub slow: bool,
    pub speed_kmh: f64,
    /// How far short of its last waypoint it was when that was announced, km.
    pub lead_km: f64,
    /// Where it came from (a town, not a neighbouring oblast), then its waypoints in order:
    /// (name, lat, lon).
    pub path: Vec<(String, f64, f64)>,
}

#[derive(Debug, Clone)]
struct Waypoint {
    place: Place,
    first: DateTime<Utc>,
    slow: bool,
    /// Its post said the kind ("реактивний", "звичайний"), not taken from earlier posts.
    typed: bool,
    /// Posted as city districts ("Тяжилів / Хутори укриття"): the drone is at the city edge.
    district: bool,
    /// Posting delay of the channel it came from, s.
    delay_s: f64,
    /// "на Вінницю" right after another place: a turn of a drone still far out, not its position.
    turned: bool,
}

/// One drone: its waypoints in order, each at its first announcement.
#[derive(Debug, Clone)]
struct Track {
    points: Vec<Waypoint>,
    origin: Option<Place>,
    /// When a post said where it came from ("з Одещини").
    origin_at: Option<DateTime<Utc>>,
    last_seen: DateTime<Utc>,
    /// Every place its posts said it heads for ("на Вороновицю курсом на Вінницю").
    targets: Vec<String>,
}

impl Track {
    /// Its pace between its posts, km/h, once three of them span a few minutes: the way it
    /// flies tells its kind better than the words (2026-10-31: "1 Шахед на …" all evening).
    fn pace_kmh(&self) -> Option<f64> {
        let (first, last) = (self.points.first()?, self.points.last()?);
        let hours = (last.first - first.first).num_seconds() as f64 / 3600.0;
        if self.points.len() < 3 || hours < PACE_MIN.num_seconds() as f64 / 3600.0 {
            return None;
        }
        let km: f64 = self.points.windows(2).map(|w| km(&w[0].place, &w[1].place)).sum();
        Some(km / hours)
    }

    /// Slow or jet by its pace, past either way's margin of what the posts said: jets go some
    /// 430 km/h from post to post (most over 290), slow ones some 170 (most under 290). Only
    /// when no post of it said its kind: a jet named so stays one however its posts went (taken
    /// for slow by their pace, three Vinnytsia hits were 3-8 min later than forecast).
    fn kind_from_pace(&mut self) {
        if self.points.iter().any(|w| w.typed) {
            return;
        }
        let Some(pace) = self.pace_kmh() else {
            return;
        };
        let slow = match self.head().slow {
            false => pace < PACE_SLOW_BELOW_KMH,
            true => pace <= PACE_JET_ABOVE_KMH,
        };
        for w in &mut self.points {
            w.slow = slow;
        }
    }

    fn head(&self) -> &Waypoint {
        self.points.last().expect("track has a waypoint")
    }

    /// Where the drone came from before its current waypoint.
    fn tail(&self) -> Option<&Place> {
        match self.points.as_slice() {
            [.., prev, _] => Some(&prev.place),
            _ => self.origin.as_ref(),
        }
    }

    /// From the last place before the current waypoint far enough off to give a bearing:
    /// "на Вінницю" and then districts 3 km from the centre fly on from Немирів before them.
    fn course(&self) -> Option<f64> {
        let head = &self.head().place;
        self.points[..self.points.len() - 1]
            .iter()
            .rev()
            .map(|w| &w.place)
            .chain(self.origin.as_ref())
            .find(|p| km(p, head) >= COURSE_MIN_KM)
            .map(|t| bearing(t, head))
    }

    fn turn_to(&self, p: &Place) -> f64 {
        self.course()
            .map_or(0.0, |c| angle_between(c, bearing(&self.head().place, p)))
    }

    fn heads_for(&self, target: &Place) -> bool {
        self.head().place.name == target.name || self.targets.contains(&target.name)
    }
}

pub struct Tracker {
    gazetteer: Gazetteer,
    pub model: Model,
    posts: HashMap<(String, u64), Post>,
    city: Place,
    /// Turns past drones made at each waypoint, for flying on past one.
    routes: Vec<RouteLeg>,
    /// Where past drones were and whether they went for the city, for its odds.
    passages: Vec<Passage>,
    /// When the user cleared the situation by hand: as if "чисто" was posted then.
    dismissed: Option<DateTime<Utc>>,
}

/// A past drone at one of its places, and whether it went for the city from there.
#[derive(Debug, Clone)]
pub struct Passage {
    time: DateTime<Utc>,
    lat: f64,
    lon: f64,
    /// The course it came in on; None at its first place.
    came: Option<f64>,
    /// Whether it was sent to the city after.
    to_city: bool,
}

/// Every place past drones were posted at, with whether they went on to the city.
pub fn passages(tracks: &[PastTrack]) -> Vec<Passage> {
    let mut out = Vec::new();
    for t in tracks {
        let first = t.path.len() - t.times.len();
        for (k, time) in (first..t.path.len()).zip(&t.times) {
            let (name, lat, lon) = &t.path[k];
            if name == CITY {
                continue;
            }
            out.push(Passage {
                time: *time,
                lat: *lat,
                lon: *lon,
                came: k
                    .checked_sub(1)
                    .map(|j| bearing_ll(t.path[j].1, t.path[j].2, *lat, *lon)),
                to_city: t.path[k + 1..].iter().any(|p| p.0 == CITY),
            });
        }
    }
    out
}

/// A past drone at a waypoint: the course it came in on and the one it left on.
#[derive(Debug, Clone)]
pub struct RouteLeg {
    day: chrono::NaiveDate,
    lat: f64,
    lon: f64,
    came: f64,
    left: f64,
}

static DISTRICTS: LazyLock<Regex> = LazyLock::new(|| {
    Regex::new(r"тяжил|\bлука\b|\bлук[уиі][\s-]+мелешк|хутор|гпз|старе місто|немирівськ\w* шосе|замост|вишеньк|київськ[аій]\b|чехова|поділл|масложир|аеропорт|гуменн|сабар|\bцентр\b").unwrap()
});
/// A part of Vinnytsia ("Лука" is Лука-Мелешківська, not the village of Лука).
pub fn is_district(name: &str) -> bool {
    DISTRICTS.is_match(&name.to_lowercase())
}

/// Parts of the city with coordinates (`ultra-alert districts`), for where a districts post
/// puts the drone.
static DISTRICT_POINTS: LazyLock<Vec<(Regex, f64, f64)>> = LazyLock::new(|| {
    let v: Vec<serde_json::Value> =
        serde_json::from_str(include_str!("../data/districts.json")).expect("districts.json");
    v.iter()
        .map(|d| {
            (
                Regex::new(d["pattern"].as_str().expect("pattern")).expect("district pattern"),
                d["lat"].as_f64().expect("lat"),
                d["lon"].as_f64().expect("lon"),
            )
        })
        .collect()
});
/// Longer lists of districts mean "the whole city", not a place in it.
const SHORT_DISTRICT_LIST: usize = 5;
/// The aftermath of an attack ("Вночі ворог вдарив Реактивним Шахедом по «Епіцентру» у Вінниці",
/// "наслідки влучання … постраждали 6 людей"): news, not a drone on its way. Whole words, so
/// "з півночі" is not "вночі".
static AFTERMATH: LazyLock<Regex> = LazyLock::new(|| {
    Regex::new(r"(?i)\b(наслід\w*|внаслідок|вночі|вдарив|вдарили|постраждал\w*|загинул\w*|поранен\w*|зруйнован\w*|момент\s+влучанн\w*|відео\w*)\b").unwrap()
});
/// A drone's pace counts once its posts span this long.
const PACE_MIN: Duration = Duration::minutes(6);
/// Below this pace a drone posted as a jet is a slow one; above that one, one posted slow is a
/// jet (2026-10-31, the history's tracks of three posts or more: jets median 432 km/h, 10th
/// percentile 288; slow ones median 173, 90th percentile 286).
const PACE_SLOW_BELOW_KMH: f64 = 230.0;
const PACE_JET_ABOVE_KMH: f64 = 330.0;
/// How long a post's kind of drone stands for the untyped posts after it.
const KIND_KEPT: Duration = Duration::hours(6);
/// How much nearer than its rough centre a neighbouring oblast's edge facing us is, km.
const REGION_EDGE_KM: f64 = 70.0;
/// Odds a drone is sent to the city, from past drones posted before ODDS_SETTLED on a course
/// within ODDS_ANGLE_DEG of its own: the share of those at its place (within ODDS_PLACE_KM),
/// pulled towards the share of those around (within ODDS_RADIUS_KM) as if ODDS_PLACE_PULL more
/// were like them, which is pulled towards the share of all as if ODDS_PRIOR more were.
/// Fitted on 1218 places of 541 drones, each day's odds from the other days only
/// (2026-10-27): log loss 0.225 (0.306 within 60 km of the city) against 0.219 (0.314) for
/// the share around alone: a little worse far out, better where it matters. "На Вороновицю":
/// 12 of 23 went on for the city, 9 of 32 around it.
const ODDS_RADIUS_KM: f64 = 38.0;
const ODDS_PLACE_KM: f64 = 3.0;
const ODDS_ANGLE_DEG: f64 = 60.0;
const ODDS_SETTLED: Duration = Duration::hours(3);
const ODDS_PRIOR: f64 = 0.5;
const ODDS_PLACE_PULL: f64 = 16.0;
/// Whether the drone a text is about is a slow one: "звичайний", "ударний", "не реактивний";
/// None when it says neither.
fn drone_type(lower: &str) -> Option<bool> {
    if lower.contains("не реактив") {
        Some(true)
    } else if lower.contains("реактив") {
        Some(false)
    } else if lower.contains("звичайн") || lower.contains("ударн") {
        Some(true)
    } else {
        None
    }
}

/// A post cut where one thing it says ends and another begins: lines, sentences, clauses,
/// "+1 …".
fn parts(text: &str) -> Vec<&str> {
    let mut parts = Vec::new();
    let mut start = 0;
    for (i, c) in text.char_indices() {
        if matches!(c, '\n' | '.' | '!' | ';' | ',' | '+') {
            parts.push(&text[start..i]);
            start = if c == '+' { i } else { i + c.len_utf8() };
        }
    }
    parts.push(&text[start..]);
    parts
}

/// Districts of the oblast: (how posts name them, their town, name). Вінницький район is in
/// the gazetteer.
const RAIONS: &[(&[&str], &str, &str)] = &[
    (&["гайсинськ", "гайсинщин"], "Гайсин", "Гайсинський район"),
    (&["тульчинськ", "тульчинщин"], "Тульчин", "Тульчинський район"),
    (&["жмеринськ", "жмеринщин"], "Жмеринка", "Жмеринський район"),
    (
        // The town is "Могилів-Подільський" too: only with "район".
        &["подільський район", "подільського район", "подільському район", "поділський район", "подільский район"],
        "Могилів-Подільський",
        "Могилів-Подільський район",
    ),
    (&["хмільницьк", "хмільниччин"], "Хмільник", "Хмільницький район"),
];
/// The all-clear itself: "Відбій загрози Шахедів", "📡 Відбій на Вінниччині 🫡".
static ALL_CLEAR: LazyLock<Regex> = LazyLock::new(|| {
    Regex::new(r"відбій\s+(загрози|тривоги|на\s+вінниччині|по\s+(всій\s+)?області)").unwrap()
});
/// … not one waited for: "відбій буде дуже НЕскоро" (2026-10-31 19:55, mid-attack), "на відбій
/// не розраховуємо", "очікуємо відбій", "мають дати відбій".
static NOT_YET: LazyLock<Regex> = LazyLock::new(|| {
    Regex::new(r"очіку|буде|розрахов|мають дати|дадуть|незабаром|скоро|пролунає|навіщо|на відбій").unwrap()
});
/// May go there, not sent there: "Може піти на Вінницю!", "Ще кілька реактивних БПЛА можуть
/// піти на ВІННИЦЮ", "Можливо будуть повторні удари по ВІННИЦІ".
static MAYBE: LazyLock<Regex> = LazyLock::new(|| {
    Regex::new(r"\bмож(е|уть)\s+(піти|йти|повернути|залетіти)\b|\bможлив(о|і)\b|\bпотенційн").unwrap()
});
/// Flown past, not heading for: "Реактивний Шахед пролетів Жмеринку".
static PASSED: LazyLock<Regex> = LazyLock::new(|| {
    Regex::new(r"\bпроле?т(ів|іла|іли|ає|ають)\b|\bпролітає\b|\bпролітають\b|\bминув\b").unwrap()
});
static CITY_ALERT: LazyLock<Regex> = LazyLock::new(|| {
    Regex::new(r"\b(на|по) міст[оуі]\b|\bвсі\W+в укриття\b|\bв укриття\W+швидко").unwrap()
});
/// Another drone, not the one posted before: "+1 на Чечельник", "Ще один Реактив на Козятин",
/// "Новий Реактивний Шахед курсом на Оратів", "Ще ШАХЕД на місто!".
static FRESH: LazyLock<Regex> = LazyLock::new(|| {
    Regex::new(r"(?i)(\+\s*\d|\bще\s+(\d|один|одна|одн\w*|шахед|реактив|звичайн|ударн|дрон|бпла|🛵)|\bнов(ий|і|их)\s+(\d\s+)?(шахед|реактив|звичайн|ударн|дрон|бпла|🛵))").unwrap()
});
static NOTE: LazyLock<Regex> =
    LazyLock::new(|| Regex::new(r"(\d{1,2}):(\d{2})\s*[-–—]\s*").unwrap());
/// Channel signatures ("💪 Телеграм канал ППО Вінниця", "👉 Термінові новини Вінниці 👈"):
/// not mentions of the city.
static SIGNATURE: LazyLock<Regex> = LazyLock::new(|| {
    Regex::new(r"(?is)(💪?\s*(телеграм канал|підписатись на)\s+ппо вінниця|👉\s*термінові новини вінниці\s*👈).*$").unwrap()
});

impl Tracker {
    pub fn new(gazetteer: Gazetteer, model: Model) -> Self {
        let city = gazetteer.place(CITY).expect("Vinnytsia in gazetteer");
        Self {
            gazetteer,
            model,
            posts: HashMap::new(),
            city,
            routes: Vec::new(),
            passages: Vec::new(),
            dismissed: None,
        }
    }

    /// Measures distances and ETAs to `(lat, lon)` (the user's home) instead of the city
    /// centre. Posts still name the city, so it keeps the city's name.
    pub fn with_home(mut self, lat: f64, lon: f64) -> Self {
        self.city.lat = lat;
        self.city.lon = lon;
        self
    }

    /// The point distances and ETAs are measured to: home, or the city centre.
    pub fn gazetteer(&self) -> &Gazetteer {
        &self.gazetteer
    }

    /// A place found while the app runs; posts naming it are read anew with it.
    pub fn add_place(&mut self, spelling: &str, place: Place) {
        self.gazetteer.add(spelling, place);
    }

    pub fn target(&self) -> &Place {
        &self.city
    }

    /// Adds new posts and replaces edited ones. Returns true when anything changed.
    pub fn update(&mut self, posts: impl IntoIterator<Item = Post>) -> bool {
        let mut changed = false;
        for post in posts {
            let key = (post.channel.clone(), post.id);
            if self.posts.get(&key) != Some(&post) {
                self.posts.insert(key, post);
                changed = true;
            }
        }
        changed
    }

    /// The latest post the tracker reads: a fallback channel's only while the main one was
    /// silent, and a regional one's only when it is about Vinnytsia oblast.
    pub fn latest_post(&self, now: DateTime<Utc>) -> Option<&Post> {
        let covered = self.fallback_covered(now);
        self.posts
            .values()
            .filter(|p| p.time <= now && !covered(p))
            .filter(|p| {
                !FALLBACK_CHANNELS.contains(&p.channel.as_str()) || !self.classify(p).is_empty()
            })
            .max_by_key(|p| p.time)
    }

    /// Whether a fallback channel's post is to be ignored at `now`: the main channel has posted
    /// since shortly before it. A fallback fills the main channel's silence only while that
    /// lasts; once the main one speaks, it alone tells the story, so the same drone is not
    /// tracked twice from two accounts.
    fn fallback_covered(&self, now: DateTime<Utc>) -> impl Fn(&Post) -> bool + use<> {
        let silence = seconds(self.model.fallback_after_s);
        let last_own = self
            .posts
            .values()
            .filter(|p| !FALLBACK_CHANNELS.contains(&p.channel.as_str()) && p.time <= now)
            .map(|p| p.time)
            .max();
        move |p: &Post| {
            FALLBACK_CHANNELS.contains(&p.channel.as_str())
                && last_own.is_some_and(|t| t >= p.time - silence)
        }
    }

    fn events(&self, now: DateTime<Utc>) -> Vec<Event> {
        self.events_since(now - HISTORY, now)
    }

    fn events_since(&self, from: DateTime<Utc>, now: DateTime<Utc>) -> Vec<Event> {
        let covered = self.fallback_covered(now);
        let mut events: Vec<Event> = self
            .posts
            .values()
            .filter(|p| p.time > from && p.time <= now && !covered(p))
            .flat_map(|p| self.classify(p))
            .filter(|e| e.time <= now)
            .collect();
        if let Some(at) = self.dismissed.filter(|at| *at <= now) {
            events.push(Event {
                time: at,
                signal: Signal::Clear,
                channel: "dismissed".into(),
            });
        }
        events.sort_by_key(|e| e.time);
        events
    }

    /// One post can carry several events: the original text plus notes the operator appends
    /// by editing ("15:39 - У місті пролунав ВИБУХ").
    fn classify(&self, post: &Post) -> Vec<Event> {
        let regional;
        let text = if REGIONAL_CHANNELS.contains(&post.channel.as_str()) {
            regional = vinnytsia_part(&post.text);
            if regional.is_empty() {
                return Vec::new();
            }
            std::borrow::Cow::Borrowed(regional.as_str())
        } else {
            SIGNATURE.replace(&post.text, "")
        };
        let mut parts = Vec::new();
        let mut last = 0;
        let mut time = post.time;
        for m in NOTE.captures_iter(&text) {
            let whole = m.get(0).unwrap();
            parts.push((time, &text[last..whole.start()]));
            time = note_time(post.time, &m[1], &m[2]).unwrap_or(post.time);
            last = whole.end();
        }
        parts.push((time, &text[last..]));
        parts
            .into_iter()
            .map(|(time, text)| Event {
                time,
                signal: self.signal(text),
                channel: post.channel.clone(),
            })
            .collect()
    }

    fn signal(&self, text: &str) -> Signal {
        // "❗️Можливо": the emoji's variation selector is a mark, so a word character to the
        // patterns, and no word would start right after it.
        let plain = text.replace('\u{fe0f}', "");
        let text = plain.as_str();
        let lower = text.to_lowercase();
        let has = |words: &[&str]| words.iter().any(|w| lower.contains(w));
        let mut route = self.gazetteer.mentions(text);
        // "Відбій ракетної загрози" ends a missile alert, not the drones being tracked.
        if has(&[
            "чисто на вінниччині",
            "чисто по шахедах",
            "без шахедів",
            "без загроз",
        ]) || (ALL_CLEAR.is_match(&lower) && !NOT_YET.is_match(&lower) && !has(&["ракет", "балісти"]))
        {
            return Signal::Clear;
        }
        if AFTERMATH.is_match(&lower) {
            return Signal::Other;
        }
        if has(&["вибух", "гупає"]) && !has(&["збитт", "збиття", "можливі вибухи"])
        {
            // "ВИБУХ НА ВІННИЧЧИНІ" / "У ЖМЕРИНСЬКОМУ РАЙОНІ": somewhere in the oblast,
            // not the channel's own city.
            let place = route
                .iter()
                .find(|m| !m.place.region)
                .map(|m| m.place.name.clone())
                .or_else(|| has(&["вінниччин", "район", "област"]).then(|| "Вінниччина".into()));
            return Signal::Explosion { place };
        }
        // The channel's own city by no name: "Ще 2 Шахеди можуть піти на місто!", "Всі в
        // укриття!". Better a false alarm than a missed hit.
        if route.is_empty() && CITY_ALERT.is_match(&lower) {
            route.push(Mention {
                place: self.city.clone(),
                role: Role::Ahead,
            });
        }
        let district = DISTRICTS.is_match(&lower);
        if district
            || has(&["шахед", "бпла", "🛵", "реактив", "дрон", "мопед", "геран"])
            || (has(&["укритт", "обережн"]) && !route.is_empty())
        {
            let slow = drone_type(&lower).unwrap_or(false);
            let summary = text.contains('▪') || has(&["обстановка"]);
            let lines = self.drone_lines(text, slow);
            // "1 Реактивний Шахед на Клембівку / Могилів-Подільський\n🛵 2 Звичайних на Теплик":
            // a line each, like a summary.
            let summary = summary || lines.0.len() > 1;
            let (mut route, mut slow_each) = if lines.0.len() > 1 {
                lines
            } else if summary {
                self.summary_items(text, slow)
            } else {
                let each = vec![slow; route.len()];
                (route, each)
            };
            for m in self.raions(&lower) {
                route.push(m);
                slow_each.push(slow);
            }
            let named: Vec<(f64, f64)> = DISTRICT_POINTS
                .iter()
                .filter(|(re, _, _)| re.is_match(&lower))
                .map(|(_, lat, lon)| (*lat, *lon))
                .collect();
            let district_at = (1..=SHORT_DISTRICT_LIST).contains(&named.len()).then(|| {
                let n = named.len() as f64;
                (
                    named.iter().map(|p| p.0).sum::<f64>() / n,
                    named.iter().map(|p| p.1).sum::<f64>() / n,
                )
            });
            return Signal::Drone {
                slow,
                fresh: !summary && self.fresh(text, &route),
                maybe: self.maybe(text),
                early: lower.contains("може займати"),
                passed: !summary && PASSED.is_match(&lower),
                typed: drone_type(&lower).is_some(),
                route,
                district,
                district_at,
                summary,
                slow_each,
            };
        }
        Signal::Other
    }

    /// "+1", "ще один", "новий" about the drone the post sends somewhere, in its own part of the
    /// post: "1 Реактивний Шахед на Вендичани +1 новий на Вінниччину з Одещини" is the drone
    /// already tracked going on to Вендичани, and another one coming. A place no part names
    /// (the city by no name): the whole post.
    fn fresh(&self, text: &str, route: &[Mention]) -> bool {
        let Some(place) = ahead(route) else {
            return false;
        };
        let parts = parts(text);
        let own = self
            .part_naming(&parts, &place.name)
            .map_or_else(|| FRESH.is_match(text), |part| FRESH.is_match(part));
        // "Могилів-Подільський, в укриття / Ще один ударний Шахед на місто": a part naming no
        // place of its own is about the post's; "слідом за ним ще 2" are others behind it.
        own || parts.iter().any(|part| {
            let lower = part.to_lowercase();
            FRESH.is_match(part)
                && self.gazetteer.mentions(part).is_empty()
                && !["слідом", "за ним", "за нею"].iter().any(|w| lower.contains(w))
        })
    }

    /// "Може піти на Вінницю", "можуть піти на ВІННИЦЮ" in the part of the post that names the
    /// city: "на ВІННИЦЮ, можливо через Агрономічне" is sent there all the same. The city by no
    /// name ("можуть піти на місто"): the whole post.
    fn maybe(&self, text: &str) -> bool {
        let parts = parts(text);
        self.part_naming(&parts, CITY)
            .map_or_else(|| MAYBE.is_match(&text.to_lowercase()), |part| MAYBE.is_match(&part.to_lowercase()))
    }

    /// The part of a post that names this place.
    fn part_naming<'a>(&self, parts: &[&'a str], name: &str) -> Option<&'a str> {
        parts
            .iter()
            .find(|part| self.gazetteer.mentions(part).iter().any(|m| m.place.name == name))
            .copied()
    }

    /// "курсом на Гайсинський / Тульчинський райони", "тривога на Гайсинщині": districts of
    /// the oblast, each at its town, as areas (like a neighbouring oblast): never a waypoint,
    /// only roughly where the drones are.
    fn raions(&self, lower: &str) -> Vec<Mention> {
        RAIONS
            .iter()
            .filter(|(stems, _, _)| stems.iter().any(|s| lower.contains(s)))
            .filter_map(|(_, town, name)| {
                let p = self.gazetteer.place(town)?;
                Some(Mention {
                    place: Place {
                        name: (*name).into(),
                        region: true,
                        alt: Vec::new(),
                        ..p
                    },
                    role: Role::Ahead,
                })
            })
            .collect()
    }

    /// The lines of a post that each send a drone somewhere: the first place ahead in each, with
    /// its line's type.
    fn drone_lines(&self, text: &str, header_slow: bool) -> (Vec<Mention>, Vec<bool>) {
        let mut route = Vec::new();
        let mut slow_each = Vec::new();
        for line in text.split('\n') {
            let lower = line.to_lowercase();
            if !["🛵", "шахед", "реактив", "звичайн", "ударн", "бпла"].iter().any(|w| lower.contains(w)) {
                continue;
            }
            let mentions = self.gazetteer.mentions(line);
            if let Some(p) = ahead(&mentions) {
                route.push(Mention {
                    place: p.clone(),
                    role: Role::Ahead,
                });
                slow_each.push(drone_type(&lower).unwrap_or(header_slow));
            }
        }
        (route, slow_each)
    }

    /// A summary's mentions line by line, each with its line's drone type: "▪️2 звичайні на
    /// Немирів; ▪️2 реактивні на Жмеринку". A line that names no type takes the header's.
    fn summary_items(&self, text: &str, header_slow: bool) -> (Vec<Mention>, Vec<bool>) {
        let mut route = Vec::new();
        let mut slow_each = Vec::new();
        for line in text.split(['▪', ';', '\n']) {
            let lower = line.to_lowercase();
            let slow = drone_type(&lower).unwrap_or(header_slow);
            for m in self.gazetteer.mentions(line) {
                route.push(m);
                slow_each.push(slow);
            }
        }
        (route, slow_each)
    }

    pub fn status(&self, now: DateTime<Utc>) -> Status {
        self.status_for(&self.city, now)
    }

    /// Status for any town in the oblast; the app shows Vinnytsia, the backtest uses all towns.
    pub fn status_for(&self, target: &Place, now: DateTime<Utc>) -> Status {
        self.status_from(&self.events(now), target, now)
    }

    fn status_from(&self, events: &[Event], target: &Place, now: DateTime<Utc>) -> Status {
        let events = since_clear(events);

        let city_alert = events.iter().rev().find(|e| {
            e.time > now - ALERT_TTL
                && matches!(&e.signal, Signal::Drone { route, maybe: false, .. } if heads_to(route, target))
        });
        let maybe_alert = events.iter().rev().find(|e| {
            e.time > now - ALERT_TTL
                && matches!(&e.signal, Signal::Drone { route, maybe: true, .. } if heads_to(route, target))
        });
        // An explosion post without a place is about the channel's own city.
        let city_explosion = events.iter().rev().find(|e| match &e.signal {
            Signal::Explosion { place } => place.as_deref().unwrap_or(CITY) == target.name,
            _ => false,
        });

        if let Some(ex) = city_explosion
            && now - ex.time <= IMPACT_SHOWN
            && city_alert.is_none_or(|a| a.time <= ex.time)
        {
            return Status::Impact { at: ex.time };
        }
        if let Some(alert) = city_alert
            && city_explosion.is_none_or(|ex| ex.time < alert.time)
        {
            // The approach being tracked started after the last explosion at the target.
            let since =
                city_explosion.map_or(0, |ex| events.partition_point(|e| e.time <= ex.time));
            // Told to take shelter as it enters the oblast: no countdown until a town of ours is
            // posted for it (2026-10-25 04:09: the hit came 41 minutes later), then from there.
            if matches!(alert.signal, Signal::Drone { early: true, .. })
                && self
                    .tracks(&events[since..], now)
                    .iter()
                    .filter(|t| t.heads_for(target))
                    .all(placeless)
            {
                return Status::Possible { at: alert.time };
            }
            // Stays on past the ETA: only an explosion, "чисто" or ALERT_TTL ends it.
            return Status::Threat(self.predict(&events[since..], alert, target, now));
        }
        // Nothing posted about the target, but a drone's course leads there.
        let since = city_explosion.map_or(0, |ex| events.partition_point(|e| e.time <= ex.time));
        if let Some(p) = self.course_threat(&events[since..], target, now) {
            return Status::Threat(p);
        }
        if let Some(m) = maybe_alert
            && city_explosion.is_none_or(|ex| ex.time < m.time)
        {
            return Status::Possible { at: m.time };
        }
        if events
            .iter()
            .any(|e| matches!(e.signal, Signal::Drone { .. }))
        {
            let nearest = self
                .estimates(events, target, now)
                .into_iter()
                .min_by(|a, b| a.km.total_cmp(&b.km));
            Status::Drones { nearest }
        } else {
            Status::Quiet
        }
    }

    /// The turns of every past track, for flying drones on past their waypoints.
    pub fn route_legs(&self) -> Vec<RouteLeg> {
        let mut legs = Vec::new();
        for t in self.past_tracks() {
            let n = t.path.len() - t.times.len();
            let pts = &t.path[n..];
            for (w, time) in pts.windows(3).zip(&t.times[1..]) {
                let place = |p: &(String, f64, f64)| (p.1, p.2);
                let (a, b, c) = (place(&w[0]), place(&w[1]), place(&w[2]));
                legs.push(RouteLeg {
                    day: time.with_timezone(&Local).date_naive(),
                    lat: b.0,
                    lon: b.1,
                    came: bearing_ll(a.0, a.1, b.0, b.1),
                    left: bearing_ll(b.0, b.1, c.0, c.1),
                });
            }
        }
        legs
    }

    pub fn set_routes(&mut self, routes: Vec<RouteLeg>) {
        self.routes = routes;
    }

    /// Clears the situation at `at`, as if "чисто" was posted: a stuck alert from a post read
    /// wrong goes away, posts after it count as usual.
    pub fn dismiss(&mut self, at: DateTime<Utc>) {
        self.dismissed = Some(at);
    }

    /// The course past drones came to `at` on (their mean), when there were enough of them and
    /// they mostly agree (mean resultant length at least 0.5); never the same day's.
    /// How sharply the drone of `t` would turn to fly on to `p`: from its own course, or with
    /// only one place known, from the way past drones came to it (2026-10-08: "повз Турбів", then
    /// "поблизу Дашева" is another drone, not one turned back south-east).
    fn turn(&self, t: &Track, p: &Place, now: DateTime<Utc>) -> f64 {
        let head = &t.head().place;
        t.course()
            .or_else(|| self.approach_course(head, now))
            .map_or(0.0, |c| angle_between(c, bearing(head, p)))
    }

    fn approach_course(&self, at: &Place, now: DateTime<Utc>) -> Option<f64> {
        let m = &self.model;
        if m.route_radius_km <= 0.0 {
            return None;
        }
        let today = now.with_timezone(&Local).date_naive();
        let (mut s, mut c, mut n) = (0.0, 0.0, 0);
        for l in &self.routes {
            if l.day != today && haversine_km(l.lat, l.lon, at.lat, at.lon) <= m.route_radius_km {
                s += l.came.to_radians().sin();
                c += l.came.to_radians().cos();
                n += 1;
            }
        }
        let agree = n > 0 && s.hypot(c) / n as f64 >= 0.5;
        (n >= m.route_min.max(1) && agree).then(|| f64::atan2(s, c).to_degrees().rem_euclid(360.0))
    }

    /// The course past drones left `at` on, having come in on about `course`: their mean, when
    /// there were enough of them. Drones of the same day never count, so the backtest is
    /// honest; live, today's earlier drones are left out too.
    fn route_course(&self, at: &Place, course: f64, now: DateTime<Utc>) -> Option<f64> {
        let m = &self.model;
        if m.route_radius_km <= 0.0 {
            return None;
        }
        let today = now.with_timezone(&Local).date_naive();
        let (mut s, mut c, mut n) = (0.0, 0.0, 0);
        for l in &self.routes {
            // A past drone that went back the way it came is two drones joined, not a turn.
            if l.day != today
                && angle_between(l.came, course) <= m.route_angle_deg
                && angle_between(l.came, l.left) <= MAX_TURN_DEG
                && haversine_km(l.lat, l.lon, at.lat, at.lon) <= m.route_radius_km
            {
                s += l.left.to_radians().sin();
                c += l.left.to_radians().cos();
                n += 1;
            }
        }
        (n >= m.route_min.max(1)).then(|| f64::atan2(s, c).to_degrees().rem_euclid(360.0))
    }

    /// Every drone tracked over all the posts given, each in its last shape (for the map's
    /// history layer); tracks that never got past one place are left out.
    pub fn past_tracks(&self) -> Vec<PastTrack> {
        let mut out = self.all_past_tracks();
        out.retain(|t| t.path.len() > 1);
        out
    }

    /// Every drone in the history, also those announced at one place only.
    pub fn all_past_tracks(&self) -> Vec<PastTrack> {
        // One pass over the whole history, as the drones flew: a drone that went out of the
        // tracker's hour still links on by its whole course.
        let Some(now) = self.posts.values().map(|p| p.time).max() else {
            return Vec::new();
        };
        let mut found: Vec<Track> = Vec::new();
        let live = self.follow(
            &self.events_since(DateTime::<Utc>::MIN_UTC, now),
            Some(&mut found),
            None,
        );
        found.extend(live);
        let mut out: Vec<PastTrack> = found
            .iter()
            .map(|t| PastTrack {
                start: t.points[0].first,
                slow: t.head().slow,
                path: t
                    .origin
                    .iter()
                    .filter(|o| !o.region)
                    .chain(t.points.iter().map(|p| &p.place))
                    .map(|p| (p.name.clone(), p.lat, p.lon))
                    .collect(),
                times: t.points.iter().map(|p| p.first).collect(),
                line: self.flight_line(t),
            })
            .collect();
        out.sort_by_key(|t| t.start);
        out
    }

    /// Where the model had the drone at each announcement ("на X": L short of X on the course
    /// from the previous place), ending at the last X it flew through, joined by a smooth curve.
    fn flight_line(&self, t: &Track) -> Vec<(f64, f64)> {
        let mut pts: Vec<(f64, f64)> = t
            .origin
            .iter()
            .filter(|o| !o.region)
            .map(|o| (o.lat, o.lon))
            .collect();
        for (i, w) in t.points.iter().enumerate() {
            let prev = if i > 0 {
                Some(&t.points[i - 1].place)
            } else {
                t.origin.as_ref()
            };
            pts.push(match prev {
                Some(p) if km(p, &w.place) > 1.0 => {
                    // Never behind the previous place: L can exceed a short leg.
                    let back = self.lead_km(w).min(0.6 * km(p, &w.place));
                    destination(w.place.lat, w.place.lon, bearing(p, &w.place) + 180.0, back)
                }
                _ => (w.place.lat, w.place.lon),
            });
        }
        let last = &t.head().place;
        pts.push((last.lat, last.lon));
        pts.dedup_by(|a, b| haversine_km(a.0, a.1, b.0, b.1) < 0.5);
        catmull_rom(&pts, 8)
    }

    /// When the drone that could reach the target first would, were it sent there now: (when,
    /// from where, km). From its live position, else, for drones posted only by oblast or
    /// district ("летять з півдня, курсом на Гайсинський / Тульчинський"), from the nearest
    /// place the last drone post named. None with no drones.
    pub fn if_heading(&self, now: DateTime<Utc>) -> Option<(DateTime<Utc>, String, f64)> {
        let at = |km: f64, speed: f64| now + seconds(km / speed * 3600.0);
        if let Some(e) = self
            .drones(now)
            .into_iter()
            // A drone once sent to the city: no "if" about it.
            .filter(|e| e.heading_to != CITY)
            .min_by(|a, b| (a.km / a.speed_kmh).total_cmp(&(b.km / b.speed_kmh)))
        {
            return Some((at(e.km, e.speed_kmh), e.heading_to, e.km));
        }
        let target = self.target();
        since_clear(&self.events(now)).iter().rev().find_map(|e| {
            let Signal::Drone { route, slow, typed, .. } = &e.signal else {
                return None;
            };
            // "1 Шахед на Гайсинський район": of the kind the attack's posts said.
            let slow = &if *typed {
                *slow
            } else {
                self.kind_before(e.time).map_or(*slow, |(k, _)| k)
            };
            let m = route
                .iter()
                .filter(|m| m.role != Role::Behind && m.place.name != CITY)
                .min_by(|a, b| km(&a.place, target).total_cmp(&km(&b.place, target)))?;
            let d = km(&m.place, target);
            let speed = if *slow {
                SLOW_SPEED_KMH
            } else {
                self.model.speed_kmh
            };
            Some((at(d, speed), m.place.name.clone(), d))
        })
    }

    pub fn set_passages(&mut self, passages: Vec<Passage>) {
        self.passages = passages;
    }

    /// Odds that one of the drones now over the oblast is sent to the city, from the past
    /// drones that were where each is at `now`, on its course: (odds, fewest past drones it
    /// rests on).
    /// For drones posted only by area, from the places the last post named.
    pub fn city_odds(&self, now: DateTime<Utc>) -> Option<(f64, usize)> {
        let drones: Vec<Estimate> = self
            .drones(now)
            .into_iter()
            .filter(|e| e.heading_to != CITY)
            .collect();
        let each: Vec<(f64, usize)> = if drones.is_empty() {
            let places: Vec<Place> = since_clear(&self.events(now))
                .iter()
                .rev()
                .find_map(|e| match &e.signal {
                    Signal::Drone { route, .. } if !route.is_empty() => Some(
                        route
                            .iter()
                            .filter(|m| m.role != Role::Behind && m.place.name != CITY)
                            .map(|m| m.place.clone())
                            .collect(),
                    ),
                    _ => None,
                })
                .unwrap_or_default();
            // Several places in one post: one group of drones, as likely as its likeliest.
            places
                .iter()
                .filter_map(|p| self.odds_at(p.lat, p.lon, None, now))
                .max_by(|a, b| a.0.total_cmp(&b.0))
                .into_iter()
                .collect()
        } else {
            // At the place it was posted heading for, as the past drones are; once it has flown
            // past that with no new post, where it is now, so the odds move on with it.
            drones
                .iter()
                .filter_map(|e| {
                    let (_, lat, lon) = e.path.last()?;
                    let past = e.course.is_some_and(|c| {
                        haversine_km(*lat, *lon, e.lat, e.lon) > ODDS_PLACE_KM
                            && angle_between(bearing_ll(*lat, *lon, e.lat, e.lon), c) < 90.0
                    });
                    let (lat, lon) = if past { (e.lat, e.lon) } else { (*lat, *lon) };
                    self.odds_at(lat, lon, e.course, now)
                })
                .collect()
        };
        // No past drone was where one is now: the share of all of them is no estimate for it.
        let fewest = each.iter().map(|(_, n)| *n).min().filter(|n| *n > 0)?;
        let none: f64 = each.iter().map(|(p, _)| 1.0 - p).product();
        Some((1.0 - none, fewest))
    }

    /// The odds a drone at (lat, lon), on `course` when it is known, is sent to the city, from
    /// the past drones posted there and around it (see ODDS_RADIUS_KM); and how many there
    /// were around. Not today's last hours: their drones may still turn for the city.
    fn odds_at(&self, lat: f64, lon: f64, course: Option<f64>, now: DateTime<Utc>) -> Option<(f64, usize)> {
        let settled: Vec<&Passage> =
            self.passages.iter().filter(|p| p.time < now - ODDS_SETTLED).collect();
        if settled.is_empty() {
            return None;
        }
        let share = |ps: &[&Passage]| ps.iter().filter(|p| p.to_city).count() as f64;
        let all = share(&settled) / settled.len() as f64;
        let around: Vec<&Passage> = settled
            .into_iter()
            .filter(|p| {
                haversine_km(p.lat, p.lon, lat, lon) <= ODDS_RADIUS_KM
                    && match (course, p.came) {
                        (Some(c), Some(came)) => angle_between(c, came) <= ODDS_ANGLE_DEG,
                        (Some(_), None) => false,
                        (None, _) => true,
                    }
            })
            .collect();
        let here: Vec<&Passage> = around
            .iter()
            .copied()
            .filter(|p| haversine_km(p.lat, p.lon, lat, lon) <= ODDS_PLACE_KM)
            .collect();
        let near = (share(&around) + ODDS_PRIOR * all) / (around.len() as f64 + ODDS_PRIOR);
        let p = (share(&here) + ODDS_PLACE_PULL * near) / (here.len() as f64 + ODDS_PLACE_PULL);
        Some((p, around.len()))
    }

    /// Live position of every drone tracked since the last "чисто", relative to Vinnytsia.
    pub fn drones(&self, now: DateTime<Utc>) -> Vec<Estimate> {
        self.drones_for(&self.city, now)
    }

    fn drones_for(&self, target: &Place, now: DateTime<Utc>) -> Vec<Estimate> {
        self.estimates(since_clear(&self.events(now)), target, now)
    }

    fn estimates(&self, events: &[Event], target: &Place, now: DateTime<Utc>) -> Vec<Estimate> {
        self.tracks(events, now)
            .iter()
            // Sent to the city with no town on its way: the threat and its countdown stand; it is
            // drawn flying in along the way past drones came, and not at all if that is unknown
            // (rather than over the city). Its first town posted makes it a track.
            .filter(|t| !placeless(t) || self.approach_course(&self.city, now).is_some())
            .map(|t| self.estimate_with(t, target, now, true))
            .collect()
    }

    fn tracks(&self, events: &[Event], now: DateTime<Utc>) -> Vec<Track> {
        // The kind the attack's posts said before these, a "чисто" or the hour ago it starts.
        let kind = events.first().and_then(|e| self.kind_before(e.time));
        let mut tracks = self.follow(events, None, kind);
        tracks.retain(|t| now - t.last_seen <= TRACK_TTL);
        tracks
    }

    /// What kind of drone the last post that said so before `time` said, within KIND_KEPT and
    /// across a "чисто" (2026-10-31: "Ударний керований Шахед" at 15:58, "чисто" at 16:10, then
    /// "1 Шахед на Гайсин" all evening): (slow, when).
    fn kind_before(&self, time: DateTime<Utc>) -> Option<(bool, DateTime<Utc>)> {
        self.events_since(time - KIND_KEPT, time)
            .iter()
            .rev()
            .filter(|e| e.time < time)
            .find_map(|e| match e.signal {
                Signal::Drone { typed: true, slow, .. } => Some((slow, e.time)),
                _ => None,
            })
    }

    /// The drones `events` tell of, in one pass. With `ended`, the whole history: a drone
    /// not posted for TRACK_TTL, hit or cleared goes there, the rest are returned.
    fn follow(
        &self,
        events: &[Event],
        mut ended: Option<&mut Vec<Track>>,
        kind: Option<(bool, DateTime<Utc>)>,
    ) -> Vec<Track> {
        let mut tracks: Vec<Track> = Vec::new();
        // "підлітає з Одещини" without a waypoint: the origin of the next new track.
        let mut pending: Option<(Place, DateTime<Utc>)> = None;
        // The kind of drone the attack's last typed post said, and when.
        let mut kind = kind;
        for e in events {
            if let Some(ended) = ended.as_deref_mut() {
                if e.signal == Signal::Clear {
                    ended.append(&mut tracks);
                    continue;
                }
                let (gone, live) = std::mem::take(&mut tracks)
                    .into_iter()
                    .partition(|t| e.time - t.last_seen > TRACK_TTL);
                ended.extend::<Vec<Track>>(gone);
                tracks = live;
            }
            // An explosion ends the drone that was heading there.
            if let Signal::Explosion { place } = &e.signal {
                let at = match place.as_deref() {
                    None | Some(CITY) => Some(self.city.clone()),
                    Some(name) => self.gazetteer.place(name),
                };
                if let Some(i) = at.and_then(|at| nearest_track(&tracks, &at)) {
                    let hit = tracks.remove(i);
                    if let Some(ended) = ended.as_deref_mut() {
                        ended.push(hit);
                    }
                }
                continue;
            }
            let Signal::Drone {
                slow,
                typed,
                fresh,
                maybe,
                passed,
                route,
                summary,
                district,
                district_at,
                slow_each,
                ..
            } = &e.signal
            else {
                continue;
            };
            // Untyped posts are of the kind the attack's last typed post said (2026-10-31: "1 Шахед
            // на Тростянець" after "Ударний керований Шахед …", then "Він не реактивний").
            if *typed {
                kind = Some((*slow, e.time));
            }
            let own: Option<(bool, Vec<bool>)> = kind
                .filter(|(_, at)| !*typed && e.time - *at <= KIND_KEPT)
                .map(|(k, _)| (k, vec![k; slow_each.len()]));
            let (slow, slow_each) = match &own {
                Some((k, each)) => (k, each),
                None => (slow, slow_each),
            };
            let delay_s = self.delay_s(&e.channel);
            if *district {
                self.observe_district(&mut tracks, *district_at, *slow, e.time, delay_s);
                continue;
            }
            let origin = route
                .iter()
                .rfind(|m| m.role == Role::Behind)
                .map(|m| (m.place.clone(), e.time));
            // A summary lists one place per drone; any other post is one drone's route.
            let heads: Vec<(&Place, bool)> = if *summary {
                route
                    .iter()
                    .zip(slow_each)
                    .filter(|(m, _)| m.role != Role::Behind && !m.place.region)
                    .map(|(m, s)| (&m.place, *s))
                    .collect()
            } else {
                ahead(route).map(|p| (p, *slow)).into_iter().collect()
            };
            if heads.is_empty() {
                pending = origin;
                continue;
            }
            let origin = origin.or_else(|| {
                pending
                    .take()
                    .filter(|(_, t)| e.time - *t <= TRACK_TTL)
            });
            let targets: Vec<String> = if *summary {
                Vec::new()
            } else {
                route
                    .iter()
                    .filter(|m| m.role != Role::Behind && !m.place.region)
                    .map(|m| m.place.name.clone())
                    .collect()
            };
            // Where it may go is no waypoint: it is not flying there yet.
            let mut heads: Vec<(Place, bool)> = heads
                .into_iter()
                .filter(|(p, _)| !(*maybe && p.name == CITY))
                .map(|(p, slow)| (self.resolve(&tracks, p, e.time, slow), slow))
                .collect();
            let turn = |t: &Track, p: &Place| self.turn(t, p, e.time);
            // A summary's lines go to the drones together: those that fit a drone best first, no
            // drone taking two (2026-10-15: "на Жмеринку", then "▪️1 на Тульчин; ▪️1 на Бар" is
            // the one at Жмеринка going on to Бар and another at Тульчин).
            if heads.len() > 1 {
                let fit = |p: &Place, slow: bool| {
                    tracks
                        .iter()
                        .filter_map(|t| {
                            if t.head().place.name == p.name {
                                return Some(0.0);
                            }
                            links(t, p, e.time, slow, &turn).then(|| unlikeness(t, p, &turn))
                        })
                        .min_by(f64::total_cmp)
                        .unwrap_or(f64::INFINITY)
                };
                heads.sort_by(|a, b| fit(&a.0, a.1).total_cmp(&fit(&b.0, b.1)));
            }
            let mut taken = Vec::new();
            for (p, slow) in heads {
                taken.push(observe(
                    &mut tracks,
                    &p,
                    origin.clone(),
                    slow,
                    *typed,
                    *fresh,
                    *passed,
                    e.time,
                    delay_s,
                    &targets,
                    self.model.city_turns,
                    &taken,
                    &turn,
                ));
            }
        }
        tracks
    }

    /// A name several villages share ("Пеньківка"): the one nearest where some tracked drone
    /// is now; with no drone tracked, the first one found.
    fn resolve(&self, tracks: &[Track], p: &Place, time: DateTime<Utc>, slow: bool) -> Place {
        if p.alt.is_empty() {
            return p.clone();
        }
        let at = |(lat, lon)| Place {
            lat,
            lon,
            ..p.clone()
        };
        let candidates: Vec<Place> = std::iter::once((p.lat, p.lon))
            .chain(p.alt.iter().copied())
            .map(at)
            .collect();
        // As a post joins a drone: the latest one that could have flown to one of them, and of
        // those the one on its course (2026-10-27: "на Пеньківку" after Дашів → Немирів is the
        // one west of Немирів, not the one by Турбів an older drone near Козятин was closest to).
        let mut recent: Vec<&Track> = tracks.iter().filter(|t| !placeless(t)).collect();
        recent.sort_by_key(|t| std::cmp::Reverse(t.last_seen));
        for t in recent {
            let hours = (time - t.head().first).num_milliseconds() as f64 / 3.6e6;
            let on_course = candidates
                .iter()
                .filter(|c| {
                    hours > 0.0
                        && plausible_leg(km(&t.head().place, c), hours, slow || t.head().slow)
                        && t.turn_to(c) <= MAX_TURN_DEG
                })
                // No course yet: the nearest.
                .min_by(|a, b| {
                    t.turn_to(a)
                        .total_cmp(&t.turn_to(b))
                        .then(km(&t.head().place, a).total_cmp(&km(&t.head().place, b)))
                });
            if let Some(c) = on_course {
                return c.clone();
            }
        }
        candidates
            .into_iter()
            .filter_map(|c| {
                let d = tracks
                    .iter()
                    .map(|t| self.estimate(t, &c, time).km)
                    .min_by(f64::total_cmp)?;
                Some((d, c))
            })
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map_or_else(|| p.clone(), |(_, c)| c)
    }

    /// Dead reckoning: at the first "на X" the drone is L short of X on its course, then
    /// flies on until the next waypoint is posted.
    /// The drone by the model the ETA uses (most accurate on the history): at each post L short
    /// of its waypoint.
    fn estimate(&self, track: &Track, target: &Place, now: DateTime<Utc>) -> Estimate {
        self.estimate_with(track, target, now, false)
    }

    /// `smooth` (for the map and the menu): the same arrivals, but from where the drone was at the
    /// post rather than jumping to L short of the waypoint, so it never teleports. On the history
    /// that path as the ETA model was worse (area under the error curve 0.598 vs 0.614).
    fn estimate_with(
        &self,
        track: &Track,
        target: &Place,
        now: DateTime<Utc>,
        smooth: bool,
    ) -> Estimate {
        if self.model.kalman {
            return self.estimate_kalman(track, target, now);
        }
        let head = track.head();
        let speed = self.speed(head);
        let lead = self.head_lead_km(track);
        // Only one place known: assume it comes the way past drones came to it, so it flies in
        // from L out instead of sitting on the place.
        let course = track
            .course()
            .or_else(|| self.approach_course(&head.place, now));
        let turn = course.and_then(|c| self.route_course(&head.place, c, now));
        // A drone is one object in time: when a waypoint is posted it is where the previous leg
        // had it, not L short of the waypoint. It flies from there to the waypoint, reaching it
        // when the model says (L at its speed), so the catch-up is a change of pace, not a jump.
        let start = course.map(|c| {
            smooth
                .then(|| self.previous_position(track))
                .flatten()
                .unwrap_or_else(|| destination(head.place.lat, head.place.lon, c + 180.0, lead))
        });
        let reach_h = lead / speed;
        let at = |t: DateTime<Utc>| match (course, start) {
            (Some(c), Some(s0)) => {
                let flown = (t - head.first).clamp(Duration::zero(), MAX_EXTRAPOLATION);
                let h = flown.num_milliseconds() as f64 / 3.6e6;
                if h < reach_h {
                    let to = bearing_ll(s0.0, s0.1, head.place.lat, head.place.lon);
                    let left = haversine_km(s0.0, s0.1, head.place.lat, head.place.lon);
                    destination(s0.0, s0.1, to, left * h / reach_h)
                } else {
                    // Past the waypoint: on along the way past drones went.
                    let on = turn.unwrap_or(c);
                    destination(head.place.lat, head.place.lon, on, speed * (h - reach_h))
                }
            }
            _ => (head.place.lat, head.place.lon),
        };
        let (lat, lon) = at(now);
        let before = at(now - Duration::seconds(10));
        let dist = haversine_km(lat, lon, target.lat, target.lon);
        // The course it flies now: towards the waypoint, then the route's once past it.
        let passed = (now - head.first)
            .clamp(Duration::zero(), MAX_EXTRAPOLATION)
            .num_milliseconds() as f64
            / 3.6e6
            >= reach_h;
        let course = match (course, start) {
            (Some(c), _) if passed => Some(turn.unwrap_or(c)),
            (Some(_), Some(s0)) => Some(bearing_ll(s0.0, s0.1, head.place.lat, head.place.lon)),
            _ => course,
        };
        Estimate {
            heading_to: head.place.name.clone(),
            lat,
            lon,
            km: dist,
            course,
            closing: dist < haversine_km(before.0, before.1, target.lat, target.lon),
            slow: head.slow,
            speed_kmh: speed,
            lead_km: lead,
            path: track
                .origin
                .iter()
                .filter(|o| !o.region)
                .chain(track.points.iter().map(|p| &p.place))
                .map(|p| (p.name.clone(), p.lat, p.lon))
                .collect(),
        }
    }

    /// Local plane around the city, km: x east, y north.
    fn to_xy(&self, lat: f64, lon: f64) -> (f64, f64) {
        (
            (lon - self.city.lon) * 111.32 * self.city.lat.to_radians().cos(),
            (lat - self.city.lat) * 110.57,
        )
    }

    fn xy_to_lat_lon(&self, p: (f64, f64)) -> (f64, f64) {
        (
            self.city.lat + p.1 / 110.57,
            self.city.lon + p.0 / (111.32 * self.city.lat.to_radians().cos()),
        )
    }

    fn noise(&self) -> kalman::Noise {
        let m = &self.model;
        kalman::Noise {
            accel: m.kf_accel,
            along_km: m.kf_along_km,
            cross_km: m.kf_cross_km,
            velocity_kmh: m.kf_velocity_kmh,
        }
    }

    /// Where the filtered drone is at `now`: from its state at the last post it flies straight
    /// to the head waypoint at its own pace, then on along past drones' route.
    fn estimate_kalman(&self, track: &Track, target: &Place, now: DateTime<Utc>) -> Estimate {
        let head = track.head();
        let f = self.filtered(track, now);
        let (p, x) = (f.pos(), self.to_xy(head.place.lat, head.place.lon));
        let speed = self.kalman_speed(&f, head);
        let to_head = (x.0 - p.0).hypot(x.1 - p.1);
        let inbound = bearing_xy(p, x);
        let turn = self
            .route_course(&head.place, inbound, now)
            .unwrap_or(inbound);
        let at = |t: DateTime<Utc>| {
            let h = (t - head.first)
                .clamp(Duration::zero(), MAX_EXTRAPOLATION)
                .num_milliseconds() as f64
                / 3.6e6;
            let s = speed * h;
            if s < to_head {
                let (lat, lon) = self.xy_to_lat_lon(p);
                (destination(lat, lon, inbound, s), inbound)
            } else {
                (
                    destination(head.place.lat, head.place.lon, turn, s - to_head),
                    turn,
                )
            }
        };
        let ((lat, lon), course) = at(now);
        let (before, _) = at(now - Duration::seconds(10));
        let dist = haversine_km(lat, lon, target.lat, target.lon);
        Estimate {
            heading_to: head.place.name.clone(),
            lat,
            lon,
            km: dist,
            course: Some(course),
            closing: dist < haversine_km(before.0, before.1, target.lat, target.lon),
            slow: head.slow,
            speed_kmh: speed,
            lead_km: to_head,
            path: track
                .origin
                .iter()
                .filter(|o| !o.region)
                .chain(track.points.iter().map(|p| &p.place))
                .map(|p| (p.name.clone(), p.lat, p.lon))
                .collect(),
        }
    }

    /// The filtered ground speed, kept within plausible bounds of the cruise speed.
    fn kalman_speed(&self, f: &kalman::Filter, head: &Waypoint) -> f64 {
        let cruise = self.speed(head);
        let v = f.vel();
        v.0.hypot(v.1).clamp(0.6 * cruise, 1.5 * cruise)
    }

    /// The Kalman filter run over a track's posts, as it stands right after the last one.
    fn filtered(&self, track: &Track, now: DateTime<Utc>) -> kalman::Filter {
        let noise = self.noise();
        let unit = |a: (f64, f64)| {
            let n = a.0.hypot(a.1).max(1e-9);
            (a.0 / n, a.1 / n)
        };
        let first = &track.points[0];
        let x0 = self.to_xy(first.place.lat, first.place.lon);
        let speed = self.speed(first);
        // Which way it came: from its origin town, else the way past drones came here.
        let dir = track
            .origin
            .as_ref()
            .filter(|o| !o.region || km(o, &first.place) > 1.0)
            .map(|o| {
                let a = self.to_xy(o.lat, o.lon);
                unit((x0.0 - a.0, x0.1 - a.1))
            })
            .or_else(|| {
                self.approach_course(&first.place, now)
                    .map(|c| (c.to_radians().sin(), c.to_radians().cos()))
            });
        let lead = self.lead_km(first);
        let mut f = match dir {
            Some(u) => kalman::Filter::new(
                (x0.0 - u.0 * lead, x0.1 - u.1 * lead),
                (u.0 * speed, u.1 * speed),
                noise.along_km,
                noise.velocity_kmh,
            ),
            // Nothing known but the place: somewhere around L from it, any heading.
            None => kalman::Filter::new(x0, (0.0, 0.0), lead.max(noise.along_km), speed),
        };
        let mut t = first.first;
        for w in &track.points[1..] {
            f.predict((w.first - t).num_milliseconds() as f64 / 3.6e6, &noise);
            t = w.first;
            let x = self.to_xy(w.place.lat, w.place.lon);
            let p = f.pos();
            let u = if (x.0 - p.0).hypot(x.1 - p.1) > 0.5 {
                unit((x.0 - p.0, x.1 - p.1))
            } else {
                unit(f.vel())
            };
            let speed = self.speed(w);
            let vel = Some(((u.0 * speed, u.1 * speed), noise.velocity_kmh));
            if w.turned {
                // "на Вінницю" from far out: which way, not where.
                f.observe(p, u, 1e6, 1e6, vel);
            } else {
                let lead = self.lead_km(w);
                let (along, cross) = if w.district {
                    (self.model.kf_district_km, self.model.kf_district_km)
                } else {
                    (noise.along_km, noise.cross_km)
                };
                f.observe((x.0 - u.0 * lead, x.1 - u.1 * lead), u, along, cross, vel);
            }
        }
        f
    }

    /// Where the previous leg's dead reckoning had the drone when the head waypoint was posted.
    fn previous_position(&self, track: &Track) -> Option<(f64, f64)> {
        if track.points.len() < 2 {
            return None;
        }
        let previous = Track {
            points: track.points[..track.points.len() - 1].to_vec(),
            ..track.clone()
        };
        let head = track.head();
        // Without a course the previous leg says nothing about where it was, unless the head is a
        // turn: then it was at the previous place.
        if previous.course().is_none() && !head.turned {
            return None;
        }
        let e = self.estimate(&previous, &head.place, head.first);
        Some((e.lat, e.lon))
    }

    /// L for the track's head, but never farther from it than the drone already was by the
    /// dead reckoning of the previous leg: a new waypoint must not move the drone backwards.
    fn head_lead_km(&self, track: &Track) -> f64 {
        let lead = self.lead_km(track.head());
        if track.points.len() < 2 {
            return lead;
        }
        let previous = Track {
            points: track.points[..track.points.len() - 1].to_vec(),
            ..track.clone()
        };
        let head = track.head();
        let was = self.estimate(&previous, &head.place, head.first).km;
        // A turn of a drone still far out: it has all that way to fly.
        if head.turned {
            return was;
        }
        if previous.course().is_none() {
            return lead;
        }
        // Forward, though, it may move: dead reckoning drifts behind over long silences, and on
        // the history a cap on that pull (any speed from 500 to 1000 km/h) made ETAs worse.
        lead.min(was)
    }

    /// The ETA is when the tracked path of the drone heading for `target` reaches it;
    /// re-estimated on every post.
    fn predict(
        &self,
        events: &[Event],
        alert: &Event,
        target: &Place,
        now: DateTime<Utc>,
    ) -> Prediction {
        let first_alert = events
            .iter()
            .find(|e| matches!(&e.signal, Signal::Drone { route, maybe: false, .. } if heads_to(route, target)))
            .map_or(alert.time, |e| e.time);
        let tracks = self.tracks(events, now);
        let track = tracks
            .iter()
            .filter(|t| t.heads_for(target))
            .max_by_key(|t| t.last_seen);
        let mut filtered_sigma = None;
        let (mut eta, basis) = match track {
            Some(t) => {
                let head = t.head();
                let d = if head.place.name == target.name {
                    0.0
                } else {
                    km(&head.place, target)
                };
                let basis = if head.district {
                    Basis::Districts
                } else if d == 0.0 {
                    Basis::Alert
                } else {
                    Basis::Waypoint(head.place.name.clone(), d)
                };
                let eta = if self.model.kalman {
                    // The filtered drone at the last post, its way to the head and on to the
                    // target at its own pace; σ from what the filter is unsure of.
                    let f = self.filtered(t, now);
                    let (p, x) = (f.pos(), self.to_xy(head.place.lat, head.place.lon));
                    let to_head = (x.0 - p.0).hypot(x.1 - p.1);
                    let path = to_head + d;
                    let speed = self.kalman_speed(&f, head);
                    let u = if to_head > 0.5 {
                        ((x.0 - p.0) / to_head, (x.1 - p.1) / to_head)
                    } else {
                        (0.0, 1.0)
                    };
                    let sd = f.pos_var_along(u).sqrt();
                    let sv = f.vel_var_along(u).sqrt();
                    filtered_sigma = Some(
                        ((sd / speed).powi(2) + (path * sv / (speed * speed)).powi(2)).sqrt()
                            * 3600.0,
                    );
                    head.first + seconds(path / speed * 3600.0)
                } else {
                    head.first + seconds((self.head_lead_km(t) + d) / self.speed(head) * 3600.0)
                };
                (eta, basis)
            }
            None => (alert.time + self.alert_flight(target), Basis::Alert),
        };
        // Sent to the city from a neighbouring oblast with no town of ours yet ("повторні удари
        // по ВІННИЦІ, 2 нові … з Одещини"): it has that oblast's way to fly first, from its edge
        // at the nearest.
        if let Some(t) = track
            && placeless(t)
            && let Some(from) = t.origin.as_ref().filter(|o| o.region)
        {
            // From when that oblast was said: it has flown on since (2026-10-14: "з Одещини"
            // at 22:43, "на ВІННИЦЮ" at 23:01, the hit at 23:06).
            let since = t.origin_at.unwrap_or(t.points[0].first);
            let km_left = (km(from, target) - REGION_EDGE_KM).max(0.0);
            eta = eta.max(since + seconds(km_left / self.model.speed_kmh * 3600.0));
        }
        // A fresh "на <target>" after the predicted arrival: the drone is still on its way, but
        // once districts were named it is over the city ("ШАХЕД ПАДАЄ"), not a whole L out.
        if eta < alert.time {
            eta = alert.time
                + match basis {
                    Basis::Districts => {
                        seconds(self.model.district_lead_km / self.model.speed_kmh * 3600.0)
                    }
                    _ => self.alert_flight(target),
                };
        }
        let sigma_s = match basis {
            Basis::Waypoint(..) => self.model.sigma_waypoint_s,
            Basis::Districts => self.model.sigma_district_s,
            Basis::Alert => self.model.sigma_alert_s,
            Basis::Course { .. } => self.model.sigma_waypoint_s,
        };
        let sigma_s = filtered_sigma.unwrap_or(sigma_s);
        Prediction {
            eta,
            sigma_s,
            basis,
            alert_time: first_alert,
        }
    }

    /// The earliest arrival among drones whose dead-reckoned course passes within the
    /// corridor of `target`, ahead of them and within range.
    fn course_threat(
        &self,
        events: &[Event],
        target: &Place,
        now: DateTime<Utc>,
    ) -> Option<Prediction> {
        if self.model.course_corridor_km <= 0.0 {
            return None;
        }
        self.tracks(events, now)
            .iter()
            .filter(|t| now - t.head().first <= MAX_EXTRAPOLATION)
            .filter_map(|t| {
                let course = t.course()?;
                let e = self.estimate(t, target, now);
                let off = angle_between(course, bearing_ll(e.lat, e.lon, target.lat, target.lon))
                    .to_radians();
                let miss_km = e.km * off.sin();
                let along_km = e.km * off.cos();
                (along_km > 0.0
                    && e.km <= self.model.course_range_km
                    && miss_km <= self.model.course_corridor_km)
                    .then(|| Prediction {
                        eta: now + seconds(along_km / self.speed(t.head()) * 3600.0),
                        sigma_s: self.model.sigma_waypoint_s,
                        basis: Basis::Course {
                            via: t.head().place.name.clone(),
                            miss_km,
                        },
                        alert_time: t.head().first,
                    })
            })
            .min_by_key(|p| p.eta)
    }

    /// Flight time from the moment "на <target>" is posted.
    fn alert_flight(&self, target: &Place) -> Duration {
        seconds(self.lead_at(target) / self.model.speed_kmh * 3600.0)
    }

    fn speed(&self, w: &Waypoint) -> f64 {
        if w.slow {
            SLOW_SPEED_KMH
        } else {
            self.model.speed_kmh
        }
    }

    fn lead_km(&self, w: &Waypoint) -> f64 {
        let lead = if w.district {
            self.model.district_lead_km
        } else {
            self.lead_at(&w.place)
        };
        (lead - self.speed(w) * w.delay_s / 3600.0).max(0.0)
    }

    fn delay_s(&self, channel: &str) -> f64 {
        self.model
            .channel_delay_s
            .get(channel)
            .copied()
            .unwrap_or(0.0)
    }

    fn lead_at(&self, p: &Place) -> f64 {
        self.model.lead_km + self.model.lead_per_km * km(p, &self.city)
    }

    /// Districts named: a waypoint at the city on the drone heading for it.
    fn observe_district(
        &self,
        tracks: &mut Vec<Track>,
        at: Option<(f64, f64)>,
        slow: bool,
        time: DateTime<Utc>,
        delay_s: f64,
    ) {
        // Still the city by name: it is the target the drone was announced for.
        let mut place = self.city.clone();
        if let Some((lat, lon)) = at {
            (place.lat, place.lon) = (lat, lon);
        }
        let point = Waypoint {
            place,
            first: time,
            slow,
            typed: false,
            district: true,
            delay_s,
            turned: self.model.district_turns && !tracks.iter().any(|t| t.head().district),
        };
        let i = nearest_track(tracks, &self.city);
        match i.map(|i| &mut tracks[i]) {
            Some(t) if t.head().district => t.last_seen = time,
            Some(t) => {
                t.points.push(point);
                t.last_seen = time;
            }
            None => tracks.push(Track {
                points: vec![point],
                origin: None,
                origin_at: None,
                last_seen: time,
                targets: vec![self.city.name.clone()],
            }),
        }
    }
}

/// Could a drone have flown `km` in `hours`? Post times are a couple of minutes off, so short
/// legs get LINK_SLACK_KM either way; long ones are held to the speed range.
fn plausible_leg(km: f64, hours: f64, slow: bool) -> bool {
    let speeds = if slow {
        &SLOW_LINK_SPEED_KMH
    } else {
        &LINK_SPEED_KMH
    };
    km <= MAX_LEG_KM
        && km <= speeds.end() * hours + LINK_SLACK_KM
        && km >= speeds.start() * hours - LINK_SLACK_KM
}

/// A drone sent to the city with no town known on its way: only the city itself (not its
/// districts, which put the drone at the city edge) and at most an oblast it came from. The city
/// there is where it heads, not where it is.
fn placeless(t: &Track) -> bool {
    t.origin.as_ref().is_none_or(|o| o.region)
        && t.points.iter().all(|w| !w.district && w.place.name == CITY)
}

/// Adds a sighting of "на `p`" to the track it most plausibly belongs to.
#[allow(clippy::too_many_arguments)]
fn observe(
    tracks: &mut Vec<Track>,
    p: &Place,
    origin: Option<(Place, DateTime<Utc>)>,
    slow: bool,
    typed: bool,
    fresh: bool,
    passed: bool,
    time: DateTime<Utc>,
    delay_s: f64,
    targets: &[String],
    city_turns: bool,
    taken: &[usize],
    turn: &dyn Fn(&Track, &Place) -> f64,
) -> usize {
    let add_targets = |t: &mut Track| {
        for name in targets {
            if !t.targets.contains(name) {
                t.targets.push(name.clone());
            }
        }
    };
    let latest = |pick: &dyn Fn(&Track) -> bool| {
        (0..tracks.len())
            .filter(|&i| !taken.contains(&i) && pick(&tracks[i]))
            .max_by_key(|&i| tracks[i].last_seen)
    };
    // Same heading again: a reminder, not a new position; "ще один на X" is another drone. Not
    // of one gone quiet (2026-10-31: "на Козятин" at 19:43 is no reminder of 18:51's).
    if let Some(i) =
        latest(&|t| !fresh && t.head().place.name == p.name && time - t.last_seen <= TRACK_TTL)
    {
        let t = &mut tracks[i];
        t.last_seen = time;
        add_targets(t);
        return i;
    }
    // "Пролетів Жмеринку" after "на Бар": it came by Жмеринка on its way to Бар, not back to
    // it. Where it came from, when nothing earlier says so; else on the way it was flying.
    if passed
        && let Some(i) = latest(&|t| {
            !placeless(t)
                && time - t.last_seen <= TRACK_TTL
                && km(&t.head().place, p) <= MAX_LEG_KM
                && t.tail().is_none_or(|tail| {
                    km(tail, p) + km(p, &t.head().place) <= 1.25 * km(tail, &t.head().place)
                })
        })
    {
        let t = &mut tracks[i];
        if t.points.len() == 1 && t.origin.is_none() {
            t.origin = Some(p.clone());
            t.origin_at = Some(time);
        }
        t.last_seen = time;
        return i;
    }
    // The drone posted last that could be it: picking the nearest instead made the backtest
    // worse within 20 s (12 hits to 10).
    let plausible = latest(&|t| !fresh && links(t, p, time, slow, turn));
    // "на Вінницю" names where a drone turns, so it joins the drone posted last even when it could
    // not be L short of the city by now (14.10: "на Іллінці / Оратів", two minutes later "на
    // ВІННИЦЮ", hit eight minutes after that): that drone is still far out. Of several, the one
    // closest to the city (2026-10-25 03:49: the one at Вороновиця, not "+1 на Чечельник" 130 km
    // off posted a minute before it).
    let turning = || {
        (0..tracks.len())
            .filter(|&i| {
                let t = &tracks[i];
                !fresh
                    && !taken.contains(&i)
                    && p.name == CITY
                    && !t.head().district
                    && t.head().place.name != CITY
                    && time - t.last_seen <= Duration::minutes(10)
                    && km(&t.head().place, p) <= MAX_LEG_KM
                    && turn(t, p) <= MAX_CITY_TURN_DEG
            })
            .min_by(|&a, &b| km(&tracks[a].head().place, p).total_cmp(&km(&tracks[b].head().place, p)))
    };
    let linked = plausible.or_else(turning);
    // Joining a drone that was heading for another place: it turns for the city from where it
    // is, posts run ahead of it in a panic (2026-10-25: "на ВОРОНОВИЦЮ" 75 s after "на
    // Немирів", the city a minute later, the hit eight minutes after that).
    let turned = p.name == CITY
        && linked.is_some_and(|i| {
            (city_turns || plausible.is_none()) && tracks[i].head().place.name != CITY
        });
    let point = Waypoint {
        place: p.clone(),
        first: time,
        slow,
        typed,
        district: false,
        delay_s,
        turned,
    };
    // A drone sent to the city with no place known ("залітає на Вінниччину, Вінниця вже може
    // займати укриття"): its first town posted after is where it is; the city stays its target.
    let adopt = if linked.is_none() && p.name != CITY {
        latest(&|t| placeless(t) && time - t.last_seen <= TRACK_TTL)
    } else {
        None
    };
    if let Some(i) = adopt {
        let t = &mut tracks[i];
        t.points = vec![point];
        t.last_seen = time;
        add_targets(t);
        return i;
    }
    match linked {
        Some(i) => {
            let t = &mut tracks[i];
            t.points.push(point);
            t.last_seen = time;
            add_targets(t);
            t.kind_from_pace();
            i
        }
        None => {
            let (origin, origin_at) = origin.unzip();
            tracks.push(Track {
                points: vec![point],
                origin,
                origin_at,
                last_seen: time,
                targets: targets.to_vec(),
            });
            tracks.len() - 1
        }
    }
}

/// How badly `p` fits the drone of `t`: the turn it would make, degrees, plus the way to it,
/// km, so with no course to go by the nearer one wins.
fn unlikeness(t: &Track, p: &Place, turn: &dyn Fn(&Track, &Place) -> f64) -> f64 {
    turn(t, p) + km(&t.head().place, p)
}

/// Could the drone of `t` be the one posted at `p` now: still about, near enough for the time
/// since, and no sharper turn than drones make.
fn links(
    t: &Track,
    p: &Place,
    time: DateTime<Utc>,
    slow: bool,
    turn: &dyn Fn(&Track, &Place) -> f64,
) -> bool {
    let hours = (time - t.head().first).num_milliseconds() as f64 / 3.6e6;
    // A drone not posted for TRACK_TTL is gone (2026-10-31: "на Бар" at 19:40 is not the one
    // last posted at Брацлав at 18:53).
    hours > 0.0
        && time - t.last_seen <= TRACK_TTL
        && !placeless(t)
        && plausible_leg(km(&t.head().place, p), hours, slow || t.head().slow)
        && turn(t, p) <= MAX_TURN_DEG
}

/// The latest track heading for `place`, else the latest one whose waypoint is near it.
fn nearest_track(tracks: &[Track], place: &Place) -> Option<usize> {
    let latest = |pick: &dyn Fn(&Track) -> bool| {
        tracks
            .iter()
            .enumerate()
            .filter(|(_, t)| pick(t))
            .max_by_key(|(_, t)| t.last_seen)
            .map(|(i, _)| i)
    };
    latest(&|t| t.heads_for(place)).or_else(|| latest(&|t| km(&t.head().place, place) <= NEAR_KM))
}

fn since_clear(events: &[Event]) -> &[Event] {
    let start = events
        .iter()
        .rposition(|e| e.signal == Signal::Clear)
        .map_or(0, |i| i + 1);
    &events[start..]
}

fn heads_to(route: &[Mention], target: &Place) -> bool {
    route
        .iter()
        .any(|m| m.place.name == target.name && m.role != Role::Behind)
}

fn km(a: &Place, b: &Place) -> f64 {
    haversine_km(a.lat, a.lon, b.lat, b.lon)
}

/// First place ahead of the drone (a neighbouring oblast is never a waypoint).
fn ahead(route: &[Mention]) -> Option<&Place> {
    route
        .iter()
        .filter(|m| !m.place.region)
        .find(|m| m.role != Role::Behind)
        .map(|m| &m.place)
}

fn seconds(s: f64) -> Duration {
    Duration::milliseconds((s * 1000.0) as i64)
}

/// "HH:MM" written in local (Kyiv) time after the post was published.
fn note_time(posted: DateTime<Utc>, h: &str, m: &str) -> Option<DateTime<Utc>> {
    let t = NaiveTime::from_hms_opt(h.parse().ok()?, m.parse().ok()?, 30)?;
    let local = posted.with_timezone(&Local).date_naive();
    [local, local + Duration::days(1)]
        .into_iter()
        .find_map(|d| {
            let at = Local
                .from_local_datetime(&d.and_time(t))
                .earliest()?
                .with_timezone(&Utc);
            (at >= posted - Duration::minutes(1) && at - posted <= Duration::hours(6)).then_some(at)
        })
}

fn bearing(a: &Place, b: &Place) -> f64 {
    bearing_ll(a.lat, a.lon, b.lat, b.lon)
}

fn bearing_ll(lat_a: f64, lon_a: f64, lat_b: f64, lon_b: f64) -> f64 {
    let (lat1, lat2) = (lat_a.to_radians(), lat_b.to_radians());
    let dlon = (lon_b - lon_a).to_radians();
    let y = dlon.sin() * lat2.cos();
    let x = lat1.cos() * lat2.sin() - lat1.sin() * lat2.cos() * dlon.cos();
    (y.atan2(x).to_degrees() + 360.0) % 360.0
}

fn angle_between(a: f64, b: f64) -> f64 {
    ((a - b + 180.0).rem_euclid(360.0) - 180.0).abs()
}

/// Words that make a regional channel's item about a drone.
const DRONE_WORDS: &[&str] = &["геран", "шахед", "бпл", "дрон"];

/// Channels covering several oblasts ("❗️ Рівненщина: герань на Клевень ❗️ Вінниччина: герань
/// на Тростянець"): only their Vinnytsia part is read.
const REGIONAL_CHANNELS: &[&str] = &["zahid_golovne_ua", "povitryanatrivogaaa"];
/// Channels read only while vinnytsiappo (the main source) is silent.
pub const FALLBACK_CHANNELS: &[&str] =
    &["vinnytsiarealll", "zahid_golovne_ua", "povitryanatrivogaaa"];

/// The part of a regional channel's post about Vinnytsia oblast, "" if none. Several drones
/// ("2 герані на Крижопіль, ще 1 на Бершадь") come out as a ▪ summary, one place per drone.
/// Air-raid alerts are not drone posts; the all-clear of a drone ("жовта") alert that names
/// Vinnytsia is.
fn vinnytsia_part(text: &str) -> String {
    let ours = |s: &str| {
        let l = s.to_lowercase();
        l.contains("вінниц") || l.contains("вінниччин")
    };
    let mut items: Vec<String> = Vec::new();
    // "✈️Вінниччина: →Шаргород ✈️Одещина: …": a plane opens a drone item of its own.
    let text = text.replace('✈', "\n✈");
    for block in text.split(['❗', '⚠', '\n']) {
        let plane = block.trim_start().starts_with('✈');
        let block = block.trim_matches(|c: char| c.is_whitespace() || c == '\u{fe0f}' || c == '✈');
        let lower = block.to_lowercase();
        let drone = plane || DRONE_WORDS.iter().any(|w| lower.contains(w));
        if lower.contains("тривог") {
            if lower.contains("відбій") && lower.contains("жовт") && ours(block) {
                items.push("Чисто на Вінниччині".into());
            }
            continue;
        }
        if block.contains("щина)") || block.contains("ччина)") {
            // "Герань на Дубровицю (Рівненщина), Герань на Борівку (Вінниччина)"
            for item in block.split(',') {
                if let Some((what, _)) = item.split_once('(')
                    && item.to_lowercase().contains("(вінниччин")
                {
                    items.push(what.trim().into());
                }
            }
            continue;
        }
        if let Some((label, rest)) = block.split_once(':')
            && label.chars().count() < 20
            && !label.chars().any(|c| c.is_ascii_digit())
        {
            // "Вінниччина: по 1 герані на Чечельник, Клембівку та Ладижин"
            if ours(label) {
                for item in rest.split([',', ';']).flat_map(|s| s.split(" та ")) {
                    let item = item.trim();
                    // Every item stays a drone post on its own.
                    let named = DRONE_WORDS.iter().any(|w| item.to_lowercase().contains(w));
                    if drone && !named {
                        items.push(format!("герань на {item}"));
                    } else {
                        items.push(item.into());
                    }
                }
            }
            continue;
        }
        if ours(block) {
            // "✈️ БПЛA→Вінниця/р-н": the Latin A hides "бпла" from the drone vocabulary.
            items.push(if plane {
                format!("дрон {block}")
            } else {
                block.into()
            });
        }
    }
    items.retain(|i| !i.is_empty());
    items.join(" ▪ ")
}

/// Degrees from north of the way from `a` to `b` on the local plane.
fn bearing_xy(a: (f64, f64), b: (f64, f64)) -> f64 {
    (b.0 - a.0).atan2(b.1 - a.1).to_degrees().rem_euclid(360.0)
}

/// A smooth curve through `pts`, `steps` points per segment: centripetal Catmull–Rom, which
/// neither loops nor overshoots when the points are unevenly spaced.
fn catmull_rom(pts: &[(f64, f64)], steps: usize) -> Vec<(f64, f64)> {
    if pts.len() < 3 {
        return pts.to_vec();
    }
    // Work in a local plane where a degree of longitude is as long as one of latitude.
    let k = pts[0].0.to_radians().cos();
    let xy: Vec<(f64, f64)> = pts.iter().map(|p| (p.1 * k, p.0)).collect();
    let n = xy.len() as isize;
    let at = |i: isize| -> (f64, f64) {
        match i {
            // Mirror the end points so the curve starts and ends along its first and last legs.
            -1 => (2.0 * xy[0].0 - xy[1].0, 2.0 * xy[0].1 - xy[1].1),
            i if i == n => {
                let (a, b) = (xy[n as usize - 1], xy[n as usize - 2]);
                (2.0 * a.0 - b.0, 2.0 * a.1 - b.1)
            }
            i => xy[i as usize],
        }
    };
    let lerp = |a: (f64, f64), b: (f64, f64), ta: f64, tb: f64, t: f64| {
        let w = if tb > ta { (t - ta) / (tb - ta) } else { 0.0 };
        (a.0 + (b.0 - a.0) * w, a.1 + (b.1 - a.1) * w)
    };
    let knot = |a: (f64, f64), b: (f64, f64)| ((b.0 - a.0).hypot(b.1 - a.1)).sqrt().max(1e-9);
    let mut out = Vec::new();
    for i in 0..n - 1 {
        let (p0, p1, p2, p3) = (at(i - 1), at(i), at(i + 1), at(i + 2));
        let t0 = 0.0;
        let t1 = t0 + knot(p0, p1);
        let t2 = t1 + knot(p1, p2);
        let t3 = t2 + knot(p2, p3);
        for s in 0..steps {
            let t = t1 + (t2 - t1) * s as f64 / steps as f64;
            let a1 = lerp(p0, p1, t0, t1, t);
            let a2 = lerp(p1, p2, t1, t2, t);
            let a3 = lerp(p2, p3, t2, t3, t);
            let b1 = lerp(a1, a2, t0, t2, t);
            let b2 = lerp(a2, a3, t1, t3, t);
            let c = lerp(b1, b2, t1, t2, t);
            out.push((c.1, c.0 / k));
        }
    }
    out.push(pts[pts.len() - 1]);
    out
}

fn destination(lat: f64, lon: f64, bearing_deg: f64, distance_km: f64) -> (f64, f64) {
    let angular = distance_km / 6371.0;
    let bearing = bearing_deg.to_radians();
    let (lat1, lon1) = (lat.to_radians(), lon.to_radians());
    let lat2 = (lat1.sin() * angular.cos() + lat1.cos() * angular.sin() * bearing.cos()).asin();
    let lon2 = lon1
        + (bearing.sin() * angular.sin() * lat1.cos())
            .atan2(angular.cos() - lat1.sin() * lat2.sin());
    (
        lat2.to_degrees(),
        ((lon2.to_degrees() + 540.0) % 360.0) - 180.0,
    )
}

pub fn haversine_km(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let (lat1, lat2) = (lat1.to_radians(), lat2.to_radians());
    let dlat = lat2 - lat1;
    let dlon = (lon2 - lon1).to_radians();
    let a = (dlat / 2.0).sin().powi(2) + lat1.cos() * lat2.cos() * (dlon / 2.0).sin().powi(2);
    2.0 * 6371.0088 * a.sqrt().atan2((1.0 - a).sqrt())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tracker() -> Tracker {
        Tracker::new(
            Gazetteer::from_json(include_str!("../data/places.json")),
            Model::from_json(include_str!("../data/model.json")),
        )
    }

    fn post(id: u64, time: &str, text: &str) -> Post {
        Post {
            channel: "vinnytsiappo".into(),
            id,
            time: DateTime::parse_from_rfc3339(time)
                .unwrap()
                .with_timezone(&Utc),
            text: text.into(),
        }
    }

    fn at(time: &str) -> DateTime<Utc> {
        DateTime::parse_from_rfc3339(time)
            .unwrap()
            .with_timezone(&Utc)
    }

    /// 2026-10-21 attack: Немирів → Вороновиця → city alert → explosion at 15:33:08Z.
    fn attack_2026_10_21() -> Vec<Post> {
        vec![
            post(
                1329,
                "2026-10-21T15:25:18Z",
                "❗️Обстановка по Реактивних Шахедах над Вінниччиною: ▪️1 реактивний на Райгород; ▪️1 реактивний на Клембівку.",
            ),
            post(
                1330,
                "2026-10-21T15:27:38Z",
                "🛵 1 Реактивний Шахед на Немирів",
            ),
            post(
                1331,
                "2026-10-21T15:29:01Z",
                "🛵 1 Реактивний Шахед на Вороновицю",
            ),
            post(
                1332,
                "2026-10-21T15:30:15Z",
                "🛵 1 Реактивний Шахед на Вороновицю",
            ),
            post(
                1333,
                "2026-10-21T15:31:29Z",
                "ГПЗ / Тяжилів / Хутори / Немирівське шосе УКРИТТЯ",
            ),
            post(1334, "2026-10-21T15:33:08Z", "Вибух 💥"),
        ]
    }

    /// 2026-10-25 03:41: "ВІННИЦЯ, ШАХЕД ПАДАЄ" after the districts ETA had passed restarted a
    /// whole L of flight (impact at 01:43:43Z); the drone is over the city by then.
    #[test]
    fn city_post_after_districts_eta_is_short() {
        let mut t = tracker();
        t.update([
            post(1, "2026-10-25T01:37:21Z", "🛵 1 Реактивний Шахед на ВОРОНОВИЦЮ -> ВІННИЦЮ"),
            post(2, "2026-10-25T01:38:57Z", "❗️Лука Мелешківська / Бучми / Немирівське шосе / Старе місто / Хутори / Масложир, в УКРИТТЯ!"),
            post(3, "2026-10-25T01:41:39Z", "❗️ ВІННИЦЯ, ШАХЕД ПАДАЄ"),
        ]);
        let Status::Threat(p) = t.status(at("2026-10-25T01:41:45Z")) else {
            panic!("no threat");
        };
        let err = (p.eta - at("2026-10-25T01:43:43Z")).num_seconds().abs();
        assert!(err <= 60, "eta {} ({err} s off)", p.eta);
    }

    /// 2026-10-25 night: vinnytsiappo was silent, the fallback channels saw a drone on
    /// Бершадь; then vinnytsiappo woke up and followed the same drone to the city. One drone,
    /// not a fallback ghost next to it.
    #[test]
    fn fallback_only_while_main_is_silent() {
        let mut t = tracker();
        let from = |channel: &str, id, time, text| Post {
            channel: String::from(channel),
            ..post(id, time, text)
        };
        t.update([
            from("zahid_golovne_ua", 1, "2026-10-25T00:22:45Z", "❗️ Вінниччина: ще герань на Бершадь з Одещини"),
            from("povitryanatrivogaaa", 2, "2026-10-25T00:30:41Z", "✈️Чернігівщина: →Сновськ/Березна (2х). ✈️Вінниччина: →Бершадь/Тростянець. ✈️Одещина: →Чорноморськ/Одеса"),
            post(3, "2026-10-25T00:33:19Z", "🛵 1 Реактивний Шахед на Бершадь"),
            post(4, "2026-10-25T00:35:57Z", "🛵 1 Реактивний Шахед на Тростянець"),
            post(5, "2026-10-25T00:43:30Z", "🛵 1 Реактивний Шахед на Тиврів"),
            post(6, "2026-10-25T00:49:15Z", "❗️ВІННИЦЯ , в укриття ! Поки йде курсом на місто!"),
        ]);
        // Before vinnytsiappo spoke, the fallback was all there was.
        assert!(!t.drones(at("2026-10-25T00:32:00Z")).is_empty());
        let drones = t.drones(at("2026-10-25T00:49:30Z"));
        assert_eq!(drones.len(), 1, "{drones:?}");
        let path: Vec<&str> = drones[0].path.iter().map(|p| p.0.as_str()).collect();
        assert_eq!(path, ["Бершадь", "Тростянець", "Тиврів", "Вінниця"]);
    }

    /// Захід Головне covers several oblasts: only the Vinnytsia part counts, several drones
    /// become a summary, and alerts or other cities' explosions are not ours.
    #[test]
    fn regional_channel_posts() {
        let t = tracker();
        let signal = |text: &str| {
            let p = Post {
                channel: "zahid_golovne_ua".into(),
                ..post(1, "2026-10-25T00:00:00Z", text)
            };
            t.classify(&p)
                .into_iter()
                .map(|e| e.signal)
                .collect::<Vec<_>>()
        };
        let heads = |text: &str| -> Vec<String> {
            match signal(text).as_slice() {
                [Signal::Drone { route, .. }] => {
                    route.iter().map(|m| m.place.name.clone()).collect()
                }
                other => panic!("{text}: {other:?}"),
            }
        };
        assert_eq!(
            heads(
                "❗️ Рівненщина: герань на Клевень ❗️ Хмельниччина: герань на Базалію ❗️ Вінниччина: герань на Тростянець"
            ),
            ["Тростянець"]
        );
        assert_eq!(
            heads(
                "❗️ Житомирщина: герань на Малин ❗️ Вінниччина: 2 герані на Крижопіль, ще 1 на Бершадь"
            ),
            ["Крижопіль", "Бершадь"]
        );
        assert_eq!(
            heads(
                "❗️Вінниччина: по 1 герані на Чечельник, Клембівку та Ладижин Рівненщина дорозвідка"
            ),
            ["Чечельник", "Клембівка", "Ладижин"]
        );
        assert_eq!(
            heads(
                "❗️ Герань на Дубровицю (Рівненщина), Герань на Борівку (Вінниччина), грань на Овруч (Житомирщина)"
            ),
            ["Борівка"]
        );
        assert_eq!(heads("❗️Вінниця герань до вас"), ["Вінниця"]);
        assert!(matches!(
            signal("💥Вінниця вибух").as_slice(),
            [Signal::Explosion { place: Some(p) }] if p == "Вінниця"
        ));
        for other in [
            "💥Житомир вибух",
            "❗️Хмельницький дрон до вас летить",
            "🫡 Хмельницький — відбій жовтої тривоги!",
            "🔴🚨 ВІННИЦЯ — червона тривога!!",
            "🟡🚨 Вінниця — жовта тривога (дронова небезпека)!",
            "❗️ Превентивна тривога Буквині та Хмельниччині через герань на Вінниччині",
        ] {
            assert!(
                signal(other).iter().all(|s| matches!(s, Signal::Other)),
                "{other}: {:?}",
                signal(other)
            );
        }
        assert!(matches!(
            signal("🫡 Хмельницький, Вінниця та Житомир — відбій жовтої тривоги!").as_slice(),
            [Signal::Clear]
        ));
        // Карта повітряних тривог: every oblast after a plane.
        let heads_in = |channel: &str, text: &str| -> Vec<String> {
            let p = Post {
                channel: channel.into(),
                ..post(1, "2026-10-25T00:00:00Z", text)
            };
            match t
                .classify(&p)
                .into_iter()
                .map(|e| e.signal)
                .collect::<Vec<_>>()
                .as_slice()
            {
                [Signal::Drone { route, .. }] => route
                    .iter()
                    .filter(|m| !m.place.region)
                    .map(|m| m.place.name.clone())
                    .collect(),
                other => panic!("{text}: {other:?}"),
            }
        };
        let povit = "povitryanatrivogaaa";
        assert_eq!(
            heads_in(
                povit,
                "✈️Вінниччина: →Шаргород/Хмельниччина. ✈️Київщина: →Іванків/Житомирщина. ✈️Чернігівщина: →Десна/Київщина (2х)"
            ),
            ["Шаргород"]
        );
        assert_eq!(
            heads_in(
                povit,
                "✈️Чернігівщина: →Сновськ/Березна (2х). ✈️Київщина: →Сухолуччя. ✈️Вінниччина: →Вінниця/р-н. ✈️Одещина: →Любашівка"
            ),
            ["Вінниця"]
        );
        assert_eq!(heads_in(povit, "✈️ БПЛA→Вінниця/р-н"), ["Вінниця"]);
        assert!(
            heads_in(
                povit,
                "✈️Вінниччина: →Чечельник. ✈️Одещина: →Березівка/Миколаївка."
            )
            .contains(&"Чечельник".to_string())
        );
    }

    /// Short legs pass whatever the posting jitter makes of their speed (Немирів → Вороновиця,
    /// 19 km in 1.4 min); long jumps do not (Теплик → Козятин, 135 km in 10 min).
    #[test]
    fn plausible_legs() {
        assert!(plausible_leg(19.0, 1.4 / 60.0, false));
        assert!(!plausible_leg(135.0, 10.0 / 60.0, false));
        assert!(!plausible_leg(60.0, 3.0 / 60.0, false));
        // 600 km/h: a jet, not an ordinary Shahed.
        assert!(plausible_leg(60.0, 6.0 / 60.0, false));
        assert!(!plausible_leg(60.0, 6.0 / 60.0, true));
    }

    /// The smoothed flight goes through every point it was given, in order.
    #[test]
    fn smoothed_line_keeps_its_points() {
        let pts = [(48.5, 29.5), (48.8, 28.5), (48.9, 28.26), (49.0, 27.95)];
        let line = catmull_rom(&pts, 8);
        assert_eq!(line.len(), 3 * 8 + 1);
        for (i, p) in pts.iter().enumerate() {
            let q = line[i * 8];
            assert!(haversine_km(p.0, p.1, q.0, q.1) < 0.01, "{i}: {q:?}");
        }
    }

    /// Past the waypoint the drone turns the way past drones did there, but never using the
    /// same day's drones, and only with enough of them.
    #[test]
    fn route_course_from_other_days() {
        let mut t = tracker();
        (
            t.model.route_radius_km,
            t.model.route_angle_deg,
            t.model.route_min,
        ) = (20.0, 30.0, 3);
        let leg = |day: u32, left: f64| RouteLeg {
            day: chrono::NaiveDate::from_ymd_opt(2026, 10, day).unwrap(),
            lat: 48.78,
            lon: 28.48,
            came: 320.0,
            left,
        };
        let now = at("2026-10-24T18:20:00Z");
        let rakhny = t.gazetteer.place("Рахни-Лісові").unwrap();
        t.set_routes(vec![leg(20, 300.0), leg(21, 290.0), leg(22, 310.0)]);
        let c = t.route_course(&rakhny, 325.0, now).unwrap();
        assert!(angle_between(c, 300.0) < 1.0, "{c}");
        // Coming in from the other side: no route.
        assert!(t.route_course(&rakhny, 140.0, now).is_none());
        // Two of three from today: too few.
        t.set_routes(vec![leg(20, 300.0), leg(24, 290.0), leg(24, 310.0)]);
        assert!(t.route_course(&rakhny, 325.0, now).is_none());
        // Back the way they came: two drones joined, no route (2026-10-31, Гнівань).
        t.set_routes(vec![leg(20, 140.0), leg(21, 145.0), leg(22, 135.0), leg(23, 300.0)]);
        assert!(t.route_course(&rakhny, 325.0, now).is_none());
    }

    /// Only the all-clear itself ends the situation, not one waited for (2026-10-31 19:55,
    /// mid-attack: "відбій буде дуже НЕскоро").
    #[test]
    fn all_clear_is_said_not_awaited() {
        let t = tracker();
        for text in [
            "❗️Дуже багато Шахедів летять на Вінниччину, тому відбій буде дуже НЕскоро",
            "🛵 1 Реактивний Шахед на Теплик, тому на відбій не розраховуємо",
            "📡 Очікуємо відбій на Вінниччині",
            "Реактив вилітає на Хмельниччину і мають дати відбій по всій області",
        ] {
            assert_ne!(t.signal(text), Signal::Clear, "{text}");
        }
        for text in ["📡 Відбій загрози Шахедів 🫡", "📡 Відбій на Вінниччині 🫡", "Відбій тривоги в області."] {
            assert_eq!(t.signal(text), Signal::Clear, "{text}");
        }
        assert_ne!(t.signal("Відбій ракетної загрози"), Signal::Clear);
    }

    /// 2026-10-31: "Ударний керований Шахед" at 15:37-15:58, "чисто" at 16:10, then "1 Шахед на
    /// …" all evening: slow ones, live too, where the tracker looks back only an hour.
    #[test]
    fn untyped_posts_are_of_the_attacks_kind() {
        let mut t = tracker();
        t.update([
            post(1, "2026-10-31T13:37:00Z", "🛵 1 Ударний керований Шахед на Козятин"),
            post(2, "2026-10-31T14:10:00Z", "📡 Чисто на Вінниччині🫡"),
            post(3, "2026-10-31T15:02:00Z", "🛵 1 Шахед на Гайсин"),
        ]);
        let drones = t.drones(at("2026-10-31T15:03:00Z"));
        assert_eq!(drones.len(), 1, "{drones:?}");
        assert!(drones[0].slow, "{drones:?}");
        // Nothing said before: a jet, until its posts show its pace.
        let mut t = tracker();
        t.update([
            post(1, "2026-10-31T12:00:00Z", "🛵 1 Шахед на Немирів"),
            post(2, "2026-10-31T12:10:00Z", "🛵 1 Шахед на Вороновицю"),
        ]);
        assert!(!t.drones(at("2026-10-31T12:10:30Z"))[0].slow);
        t.update([post(3, "2026-10-31T12:20:00Z", "🛵 1 Шахед на Тиврів")]);
        let drones = t.drones(at("2026-10-31T12:20:30Z"));
        assert_eq!(drones.len(), 1, "{drones:?}");
        assert!(drones[0].slow, "40 km in 20 min is no jet: {drones:?}");
    }

    /// 2026-10-24: four villages are called Пеньківка; the drone coming up from Рахни-Лісові
    /// heads for the one by Станіславчик, not the first one geocoded, by Турбів.
    #[test]
    fn same_named_village_follows_the_track() {
        let mut t = tracker();
        t.update([
            post(
                1,
                "2026-10-24T18:11:41Z",
                "🛵 1 Реактивний Шахед на Вапнярку",
            ),
            post(
                2,
                "2026-10-24T18:16:47Z",
                "🛵 1 Реактивний Шахед на Рахни Лісові",
            ),
            post(
                3,
                "2026-10-24T18:21:44Z",
                "🛵 1 Реактивний Шахед на Пеньківку",
            ),
        ]);
        let drones = t.drones(at("2026-10-24T18:22:00Z"));
        assert_eq!(drones.len(), 1);
        let (name, lat, lon) = drones[0].path.last().unwrap();
        assert_eq!(name, "Пеньківка");
        assert!(
            haversine_km(*lat, *lon, 48.896, 28.262) < 3.0,
            "{lat}, {lon}"
        );
    }

    /// 2026-10-27: an older drone near Козятин must not pull "на Пеньківку" to the one by
    /// Турбів; the drone from Немирів heading west is the one it is about.
    #[test]
    fn same_named_village_on_the_course_not_by_another_drone() {
        let mut t = tracker();
        t.update([
            post(1, "2026-10-27T11:55:00Z", "🛵 1 Реактивний Шахед на Козятин"),
            post(2, "2026-10-27T12:01:04Z", "🛵 1 Реактивний Шахед на Дашів"),
            post(3, "2026-10-27T12:07:00Z", "🛵 1 Реактивний Шахед на Немирів"),
            post(4, "2026-10-27T12:14:00Z", "🛵 1 Реактивний Шахед на Пеньківку"),
        ]);
        let drones = t.drones(at("2026-10-27T12:14:30Z"));
        let penkivka = drones
            .iter()
            .find(|d| d.heading_to == "Пеньківка")
            .expect("a drone heading for Пеньківка");
        let (_, lat, lon) = penkivka.path.last().unwrap();
        assert!(haversine_km(*lat, *lon, 48.896, 28.262) < 3.0, "{lat}, {lon}");
        assert!(penkivka.path.iter().any(|p| p.0 == "Немирів"));
    }

    /// "на Вороновицю курсом на ВІННИЦЮ": the drone is L short of Вороновиця, then flies
    /// Вороновиця → city. Accuracy over many raids is the backtest's job.
    #[test]
    fn eta_through_waypoint_heading_for_city() {
        let mut t = tracker();
        let mut posts = attack_2026_10_21();
        posts[2].text = "🛵 1 Реактивний Шахед на Вороновицю курсом на ВІННИЦЮ".into();
        t.update(posts.into_iter().take(3));
        let Status::Threat(p) = t.status(at("2026-10-21T15:29:30Z")) else {
            panic!("expected threat")
        };
        let Basis::Waypoint(name, d) = &p.basis else {
            panic!("{:?}", p.basis)
        };
        assert_eq!(name, "Вороновиця");
        let flight = (t.model.lead_km + d) / t.model.speed_kmh * 3600.0;
        assert_eq!(p.eta, at("2026-10-21T15:29:01Z") + seconds(flight));
    }

    #[test]
    fn city_alert_is_a_waypoint_at_the_city() {
        let mut t = tracker();
        let mut posts = attack_2026_10_21();
        posts[4].text = "🛵 Реактивний Шахед на ВІННИЦЮ".into();
        t.update(posts.into_iter().take(5));
        let Status::Threat(p) = t.status(at("2026-10-21T15:31:40Z")) else {
            panic!("expected threat")
        };
        assert_eq!(p.basis, Basis::Alert);
        let flight = t.model.lead_km / t.model.speed_kmh * 3600.0;
        assert_eq!(p.eta, at("2026-10-21T15:31:29Z") + seconds(flight));
    }

    #[test]
    fn districts_override_waypoint() {
        let mut t = tracker();
        t.update(attack_2026_10_21());
        assert!(matches!(
            t.status(at("2026-10-21T15:31:00Z")),
            Status::Drones { .. }
        ));
        let Status::Threat(p) = t.status(at("2026-10-21T15:31:40Z")) else {
            panic!("expected threat")
        };
        assert_eq!(p.basis, Basis::Districts);
        let err = (p.eta - at("2026-10-21T15:33:08Z")).num_seconds().abs() as f64;
        assert!(err <= p.sigma_s, "eta {}", p.eta);
    }

    #[test]
    fn explosion_then_quiet() {
        let mut t = tracker();
        t.update(attack_2026_10_21());
        assert!(matches!(
            t.status(at("2026-10-21T15:34:00Z")),
            Status::Impact { .. }
        ));
        t.update([post(
            1340,
            "2026-10-21T15:40:00Z",
            "📡 Чисто на Вінниччині🫡",
        )]);
        assert_eq!(t.status(at("2026-10-21T15:41:00Z")), Status::Quiet);
    }

    #[test]
    fn drones_elsewhere_are_not_a_city_threat() {
        let mut t = tracker();
        t.update([post(
            1,
            "2026-10-22T15:08:21Z",
            "🛵 Перший Реактивний Шахед на Ладижин",
        )]);
        let Status::Drones { nearest: Some(e) } = t.status(at("2026-10-22T15:09:00Z")) else {
            panic!("expected drones")
        };
        assert_eq!(e.heading_to, "Ладижин");
        assert!(e.km > 60.0);
    }

    /// Drones posted only by district still get a time they could be here in.
    #[test]
    fn if_heading_from_districts_of_the_oblast() {
        let mut t = tracker();
        t.update([post(
            1,
            "2026-10-25T23:22:15Z",
            "🛵 3 Реактивні Шахеди летять в напрямку Вінниччини з півдня, курсом на Гайсинський / Тульчинський райони\n\n💪 Телеграм канал ППО Вінниця",
        )]);
        let now = at("2026-10-25T23:40:00Z");
        assert!(matches!(t.status(now), Status::Drones { .. }));
        let (eta, from, km) = t.if_heading(now).expect("a time");
        eprintln!("{from} {km:.0} km, {} s", (eta - now).num_seconds());
        assert!(eta > now);
    }

    /// The city by no name is the city: better a false alarm than a missed hit.
    #[test]
    fn unnamed_city_is_the_city() {
        let t = tracker();
        for text in [
            "Ще 2 Шахеди можуть піти на місто !",
            "Може йти на удар! Всі в укриття!",
            "В укриття швидко!",
        ] {
            let Signal::Drone { route, .. } = t.signal(text) else {
                panic!("{text}: not a drone")
            };
            assert_eq!(route[0].place.name, CITY, "{text}");
        }
        assert!(!matches!(
            t.signal("Перебувайте в укриттях в разі тривоги. Бережіть себе"),
            Signal::Drone { .. }
        ));
    }

    /// Today's raid: "з Одещини" then "на Чечельник" gives a course, and the live distance
    /// to the city keeps changing between posts.
    #[test]
    fn live_position_moves_between_posts() {
        let mut t = tracker();
        t.update([
            post(1579, "2026-10-23T13:46:31Z", "🛵 1 Реактивний Шахед підлітає з Одещини, тому скоро очікуємо тривогу на Гайсинщині та Тульчинщині"),
            post(1580, "2026-10-23T13:54:28Z", "🛵 1 Реактивний Шахед на Чечельник"),
        ]);
        let live = |at_: &str| match t.status(at(at_)) {
            Status::Drones { nearest: Some(e) } => e,
            other => panic!("expected drones, got {other:?}"),
        };
        let (a, b) = (live("2026-10-23T13:55:00Z"), live("2026-10-23T13:57:00Z"));
        assert!(a.course.is_some(), "course from Одещина to Чечельник");
        assert!((a.km - b.km).abs() > 5.0, "{} vs {}", a.km, b.km);
    }

    #[test]
    fn track_follows_successive_waypoints() {
        let mut t = tracker();
        t.update(attack_2026_10_21().into_iter().skip(1).take(3));
        let drones = t.drones(at("2026-10-21T15:30:30Z"));
        assert_eq!(drones.len(), 1, "{drones:?}");
        assert_eq!(drones[0].heading_to, "Вороновиця");
        assert!(drones[0].closing);
    }

    #[test]
    fn explosion_ends_the_track_that_hit() {
        let mut t = tracker();
        t.update(attack_2026_10_21());
        assert!(
            t.drones(at("2026-10-21T15:34:00Z"))
                .iter()
                .all(|d| d.heading_to != "Вінниця")
        );
    }

    /// A drone first heard of as "на Немирів" flies in along the way past drones came, from L
    /// out, rather than appearing over the town.
    #[test]
    fn one_place_drone_flies_in_along_past_approaches() {
        let mut t = tracker();
        (
            t.model.route_radius_km,
            t.model.route_angle_deg,
            t.model.route_min,
        ) = (20.0, 30.0, 3);
        let town = t.gazetteer.place("Немирів").unwrap();
        let leg = |day: u32, came: f64| RouteLeg {
            day: chrono::NaiveDate::from_ymd_opt(2026, 10, day).unwrap(),
            lat: town.lat,
            lon: town.lon,
            came,
            left: came,
        };
        t.set_routes(vec![leg(20, 300.0), leg(21, 310.0), leg(22, 320.0)]);
        t.update([post(
            1,
            "2026-10-25T02:30:00Z",
            "🛵 Реактивний Шахед на Немирів",
        )]);
        let d = &t.drones(at("2026-10-25T02:30:00Z"))[0];
        let off = haversine_km(d.lat, d.lon, town.lat, town.lon);
        assert!(
            (off - t.lead_at(&town)).abs() < 1.0,
            "{off} km from the town"
        );
        assert!(angle_between(d.course.unwrap(), 310.0) < 1.0);
        // From the south-east: the drone is south-east of the town.
        assert!(d.lat < town.lat && d.lon > town.lon);
    }

    /// 2026-10-25 05:12: "залітає на Вінниччину, Вінниця вже може займати укриття", then "на
    /// Крижопіль" and on west: one drone that starts at Крижопіль (not one flying out of the city
    /// south), still a threat to the city at first, its countdown from Крижопіль.
    #[test]
    fn city_alert_out_of_nowhere_takes_the_first_town() {
        let mut t = tracker();
        t.update([
            post(
                1,
                "2026-10-25T03:12:38Z",
                "🛵 1 Реактивний Шахед залітає на Вінниччину\n\nВінниця вже може займати укриття.",
            ),
            post(
                2,
                "2026-10-25T03:20:02Z",
                "🛵 1 Реактивний Шахед на Крижопіль",
            ),
        ]);
        let Status::Threat(p) = t.status(at("2026-10-25T03:20:30Z")) else {
            panic!("{:?}", t.status(at("2026-10-25T03:20:30Z")));
        };
        assert!(
            p.eta - at("2026-10-25T03:20:02Z") > Duration::minutes(10),
            "eta {}",
            p.eta
        );
        t.update([
            post(
                3,
                "2026-10-25T03:27:38Z",
                "🛵 1 Реактивний Шахед на Томашпіль",
            ),
            post(
                4,
                "2026-10-25T03:31:42Z",
                "🛵 1 Реактивний Шахед на Шаргород",
            ),
            post(
                5,
                "2026-10-25T03:35:22Z",
                "🛵 1 Реактивний Шахед на Копайгород",
            ),
        ]);
        let drones = t.drones(at("2026-10-25T03:36:00Z"));
        assert_eq!(drones.len(), 1, "{drones:?}");
        let path: Vec<&str> = drones[0].path.iter().map(|p| p.0.as_str()).collect();
        assert_eq!(path, ["Крижопіль", "Томашпіль", "Шаргород", "Копайгород"]);
    }

    /// 2026-10-25 04:09: "залітає на Вінниччину, Вінниця вже може займати укриття" with no town
    /// on the drone's way: better a threat too many than a hit missed (three real ones started
    /// like this), but the drone is not drawn over the city until a town on its way is posted.
    #[test]
    fn city_alert_out_of_nowhere_warns_but_is_not_drawn() {
        let mut t = tracker();
        t.update([post(
            1,
            "2026-10-25T02:09:59Z",
            "🛵 1 Реактивний Шахед залітає на Вінниччину\n\nВінниця вже може займати укриття.",
        )]);
        let now = at("2026-10-25T02:10:30Z");
        // An alarm, but no town to count down from yet.
        assert!(
            matches!(t.status(now), Status::Possible { .. }),
            "{:?}",
            t.status(now)
        );
        assert!(t.drones(now).is_empty());
        t.update([post(
            2,
            "2026-10-25T02:13:00Z",
            "🛵 1 Реактивний Шахед на Вороновицю курсом на Вінницю",
        )]);
        assert!(matches!(
            t.status(at("2026-10-25T02:13:30Z")),
            Status::Threat(_)
        ));
        assert_eq!(t.drones(at("2026-10-25T02:13:30Z")).len(), 1);
    }

    /// 2026-10-25 morning: news of the night's hits named a Shahed and the city and started an
    /// alert that blinked at 0:00; "з півночі" in a live post is not "вночі".
    #[test]
    fn aftermath_news_is_no_alert() {
        let mut t = tracker();
        t.update([
            post(1, "2026-10-25T10:38:10Z", "💥 Вночі ворог вдарив Реактивним Шахедом по «Епіцентру» у Вінниці 😔"),
            post(2, "2026-10-25T09:41:04Z", "💥 Страшні наслідки влучання Реактиву Шахеда по житловому будинку у Вінниці 😔 Внаслідок удару постраждали 6 людей"),
        ]);
        assert_eq!(t.status(at("2026-10-25T10:39:00Z")), Status::Quiet);
        t.update([post(
            3,
            "2026-10-25T10:45:00Z",
            "🛵 Новий Звичайний Шахед на Козятин з півночі",
        )]);
        assert!(matches!(
            t.status(at("2026-10-25T10:45:30Z")),
            Status::Drones { .. }
        ));
    }

    /// "+1 новий" in its own part of the post is another drone, not the one sent to Вендичани
    /// (2026-10-26 21:23); "Ще один … на місто" naming no place of its own is about the post's.
    #[test]
    fn fresh_only_in_its_own_part() {
        let t = tracker();
        let fresh = |text: &str| match t.signal(text) {
            Signal::Drone { fresh, .. } => fresh,
            other => panic!("{text}: {other:?}"),
        };
        assert!(!fresh("🛵 1 Реактивний Шахед на Вендичани\n\n+1 новий на Вінниччину з Одещини"));
        assert!(!fresh("🛵 1 Реактивний Шахед на Теплик\n\nЗа ним слідом ще 3."));
        assert!(fresh("🛵 +1 Реактивний Шахед на Чечельник"));
        assert!(fresh("🛵 Могилів-Подільський, в укриття\n\nЩе один ударний Шахед на місто❗️"));
    }

    /// Two drones in one post, a line each, each with its own type (2026-10-14 19:16).
    #[test]
    fn a_drone_per_line() {
        let t = tracker();
        let Signal::Drone { route, slow_each, .. } =
            t.signal("🛵 1 Реактивний Шахед на Клембівку / Могилів-Подільський\n\n🛵 2 Звичайних на Теплик")
        else {
            panic!("a drone post")
        };
        let names: Vec<&str> = route.iter().map(|m| m.place.name.as_str()).collect();
        assert_eq!(names, ["Клембівка", "Теплик"]);
        assert_eq!(slow_each, [false, true]);
        assert!(matches!(
            t.signal("🛵 1 Звичайний Шахед Калинівку\n\nВін не реактивний, тому й так повільно тягнеться"),
            Signal::Drone { slow: true, .. }
        ));
    }

    /// 2026-10-14 22:20: "на Жмеринку", then "▪️1 на Тульчин; ▪️1 на Бар": the drone at
    /// Жмеринка goes on west to Бар, Тульчин is another one (not one flying back south-east).
    #[test]
    fn summary_lines_go_to_the_drones_they_fit() {
        let mut t = tracker();
        t.update([
            post(1, "2026-10-14T22:20:40Z", "🛵 Звичайний Шахед на Жмеринку"),
            post(
                2,
                "2026-10-14T22:29:49Z",
                "🛵 2 Шахеди над Вінниччиною:\n\n▪️1 на Тульчин (Тульчинщина);\n▪️1 на Бар (Жмеринщина).",
            ),
        ]);
        let paths: Vec<Vec<String>> = t
            .drones(at("2026-10-14T22:30:00Z"))
            .into_iter()
            .map(|d| d.path.into_iter().map(|p| p.0).collect())
            .collect();
        assert!(paths.contains(&vec!["Жмеринка".to_owned(), "Бар".to_owned()]), "{paths:?}");
        assert!(paths.contains(&vec!["Тульчин".to_owned()]), "{paths:?}");
    }

    /// 2026-10-20 15:18 "на Бар", 15:26 "пролетів Жмеринку": it came by Жмеринка on its way
    /// to Бар, not back east to Жмеринка.
    #[test]
    fn a_place_flown_past_is_where_it_came_from() {
        let mut t = tracker();
        t.update([
            post(1, "2026-10-20T12:18:00Z", "🛵 1 Реактивний Шахед на Бар"),
            post(2, "2026-10-20T12:26:00Z", "🛵 Реактивний Шахед пролетів Жмеринку"),
        ]);
        let drones = t.drones(at("2026-10-20T12:26:30Z"));
        assert_eq!(drones.len(), 1, "{drones:?}");
        assert_eq!(drones[0].heading_to, "Бар");
        let path: Vec<&str> = drones[0].path.iter().map(|p| p.0.as_str()).collect();
        assert_eq!(path, ["Жмеринка", "Бар"]);
    }

    /// 2026-10-26 13:04: a repost of the morning's hit, "Епіцентр" no district "центр".
    #[test]
    fn hit_video_is_no_alert() {
        let mut t = tracker();
        t.update([post(
            1,
            "2026-10-26T11:04:00Z",
            "💥 😯☺️😁 Момент влучання Реактивного Шахеда по магазину «Епіцентр» у Вінниці 🤯\n\nВідео поширюють всеукраїнські ЗМІ.",
        )]);
        assert_eq!(t.status(at("2026-10-26T11:05:00Z")), Status::Quiet);
        assert!(!DISTRICTS.is_match("шахед на епіцентр"));
        assert!(DISTRICTS.is_match("центр в укриття"));
    }

    /// 2026-10-26 10:27: sent to the city while still in Odesa oblast: not minutes away.
    #[test]
    fn city_alert_from_another_oblast_is_far() {
        let mut t = tracker();
        t.update([post(
            1,
            "2026-10-26T08:27:40Z",
            "🛵 2 Реактивні Шахеди з Одещини на ВІННИЦЮ",
        )]);
        let now = at("2026-10-26T08:28:00Z");
        let Status::Threat(p) = t.status(now) else {
            panic!("a threat, better safe")
        };
        assert!(p.eta - now > Duration::minutes(10), "{:?}", p.eta - now);
    }

    /// "Можуть піти на ВІННИЦЮ": an alarm, no countdown, until "на ВІННИЦЮ" (2026-10-16 16:31,
    /// whose countdown ran out 20 minutes before the hit at 16:57).
    #[test]
    fn may_go_for_the_city_is_no_countdown() {
        let mut t = tracker();
        t.update([post(
            1,
            "2026-10-16T13:31:50Z",
            "❗️Перебувайте в безпечних місцях\n\nЩе кілька реактивних БПЛА можуть піти на ВІННИЦЮ",
        )]);
        assert!(matches!(
            t.status(at("2026-10-16T13:32:00Z")),
            Status::Possible { .. }
        ));
        t.update([post(
            2,
            "2026-10-26T08:27:40Z",
            "❗️Можливо будуть повторні удари по ВІННИЦІ\n\n🛵 2 нові ракети Герань-5 з Одещини на Вінниччину",
        )]);
        let s = t.status(at("2026-10-26T08:28:00Z"));
        assert!(matches!(s, Status::Possible { .. }), "{s:?} {:?}", t.signal("❗️Можливо будуть повторні удари по ВІННИЦІ\n\n🛵 2 нові ракети Герань-5 з Одещини на Вінниччину"));
        t.update([
            post(3, "2026-10-16T13:54:40Z", "🛵 Реактивний Шахед на Вороновицю курсом на Вінницю"),
            post(4, "2026-10-16T13:55:29Z", "🛵 Реактивний Шахед на Вінницю"),
        ]);
        let Status::Threat(p) = t.status(at("2026-10-16T13:55:40Z")) else {
            panic!("sent to the city: a countdown")
        };
        assert!(p.alert_time >= at("2026-10-16T13:54:40Z"), "{}", p.alert_time);
        let mut t = tracker();
        t.update([post(5, "2026-10-16T13:55:29Z", "🛵 Реактивний Шахед на ВІННИЦЮ, можливо через Агрономічне")]);
        assert!(matches!(t.status(at("2026-10-16T13:55:40Z")), Status::Threat(_)));
    }

    /// Clearing by hand ends a stuck alert like a "чисто" post would; posts after it count.
    #[test]
    fn dismiss_clears_the_situation() {
        let mut t = tracker();
        t.update([post(
            1,
            "2026-10-25T12:00:00Z",
            "🛵 Реактивний Шахед на ВІННИЦЮ",
        )]);
        assert!(matches!(
            t.status(at("2026-10-25T12:01:00Z")),
            Status::Threat(_)
        ));
        t.dismiss(at("2026-10-25T12:02:00Z"));
        assert_eq!(t.status(at("2026-10-25T12:02:30Z")), Status::Quiet);
        t.update([post(
            2,
            "2026-10-25T12:05:00Z",
            "🛵 Реактивний Шахед на ВІННИЦЮ",
        )]);
        assert!(matches!(
            t.status(at("2026-10-25T12:05:30Z")),
            Status::Threat(_)
        ));
    }

    #[test]
    fn channel_signature_is_not_a_city_alert() {
        let mut t = tracker();
        t.update([post(
            1,
            "2026-10-22T15:08:21Z",
            "🛵 Реактивний Шахед на Ладижин\n\n💪 Телеграм канал ППО Вінниця",
        )]);
        assert!(matches!(
            t.status(at("2026-10-22T15:09:00Z")),
            Status::Drones { .. }
        ));
    }

    #[test]
    fn edited_note_is_a_later_event() {
        let mut t = tracker();
        t.update([
            post(
                0,
                "2026-10-12T12:33:03Z",
                "🛵 Залетів новий Реактивний Шахед курсом від Погребища на Вінницю",
            ),
            post(
                1,
                "2026-10-12T12:35:02Z",
                "🛵 1 Реактивний Шахед на ВІННИЦЮ 💥 15:39 - У місті пролунав ВИБУХ",
            ),
        ]);
        assert!(matches!(
            t.status(at("2026-10-12T12:36:00Z")),
            Status::Threat(_)
        ));
    }
}

/// Tunes the trajectory-only warning: `cargo test --release course_warning -- --ignored --nocapture`
#[cfg(test)]
mod course_warning {
    use super::*;
    use serde_json::Value;
    use std::collections::BTreeSet;

    #[test]
    #[ignore]
    fn course_warning() {
        // The maintainer's scraped history, not in git.
        let posts: Vec<Post> = std::fs::read_to_string("data/messages.jsonl")
            .expect("data/messages.jsonl: run `ultra-alert scrape` first")
            .lines()
            .map(|l| serde_json::from_str::<Value>(l).unwrap())
            .filter(|v| v["date"].as_str().unwrap() >= "2026-10-05")
            .map(|v| Post {
                channel: "vinnytsiappo".into(),
                id: v["id"].as_u64().unwrap(),
                time: DateTime::parse_from_rfc3339(v["date"].as_str().unwrap())
                    .unwrap()
                    .with_timezone(&Utc),
                text: v["text"].as_str().unwrap().to_owned(),
            })
            .collect();
        let gz = Gazetteer::from_json(include_str!("../data/places.json"));
        let impacts: Vec<(DateTime<Utc>, Place)> =
            serde_json::from_str::<Vec<Value>>(include_str!("../data/impacts.json"))
                .unwrap()
                .iter()
                .filter_map(|v| {
                    let t = DateTime::parse_from_rfc3339(v["t"].as_str()?)
                        .ok()?
                        .with_timezone(&Utc);
                    Some((t, gz.place(v["place"].as_str()?)?))
                })
                .filter(|(t, _)| *t > posts[0].time)
                .collect();
        let mut t = Tracker::new(gz, Model::from_json(include_str!("../data/model.json")));
        // Classify every post once; status_from then works on time windows of these.
        let mut all: Vec<Event> = posts.iter().flat_map(|p| t.classify(p)).collect();
        all.sort_by_key(|e| e.time);
        let window = |now: DateTime<Utc>| {
            let a = all.partition_point(|e| e.time <= now - HISTORY);
            let b = all.partition_point(|e| e.time <= now);
            &all[a..b]
        };
        let city = t.city.clone();
        let is_threat = |s: &Status| matches!(s, Status::Threat(_));

        // Every 30 s for 20 min after any drone post.
        let mut samples = BTreeSet::new();
        for e in all
            .iter()
            .filter(|e| matches!(e.signal, Signal::Drone { .. }))
        {
            let base = e.time.timestamp() / 30 * 30;
            for k in 0..=40 {
                samples.insert(base + k * 30);
            }
        }
        let samples: Vec<DateTime<Utc>> = samples
            .into_iter()
            .filter_map(|s| DateTime::from_timestamp(s, 0))
            .collect();
        let city_impacts: Vec<DateTime<Utc>> = impacts
            .iter()
            .filter(|(_, p)| p.name == CITY)
            .map(|(t, _)| *t)
            .collect();

        t.model.course_corridor_km = 0.0;
        let missed: Vec<&(DateTime<Utc>, Place)> = impacts
            .iter()
            .filter(|(ex, p)| {
                !is_threat(&t.status_from(
                    window(*ex - Duration::seconds(1)),
                    p,
                    *ex - Duration::seconds(1),
                ))
            })
            .collect();
        let base_threat: Vec<bool> = samples
            .iter()
            .map(|s| is_threat(&t.status_from(window(*s), &city, *s)))
            .collect();
        let first_warning = |t: &Tracker, ex: DateTime<Utc>| {
            (0..=50)
                .rev()
                .map(|k| ex - Duration::seconds(30 * k))
                .find(|s| is_threat(&t.status_from(window(*s), &city, *s)))
        };
        let base_first: Vec<Option<DateTime<Utc>>> = city_impacts
            .iter()
            .map(|ex| first_warning(&t, *ex))
            .collect();
        println!(
            "impacts {} (Вінниця {}), missed by the channel: {}",
            impacts.len(),
            city_impacts.len(),
            missed.len()
        );

        for corridor in [5.0, 10.0, 15.0, 20.0] {
            for range in [30.0, 50.0, 80.0] {
                t.model.course_corridor_km = corridor;
                t.model.course_range_km = range;
                let caught: Vec<i64> = missed
                    .iter()
                    .filter_map(|(ex, p)| {
                        let now = *ex - Duration::seconds(1);
                        match t.status_from(window(now), p, now) {
                            Status::Threat(pr) => Some((pr.eta - *ex).num_seconds()),
                            _ => None,
                        }
                    })
                    .collect();
                // Vinnytsia: course-only threat moments grouped into episodes.
                let mut episodes: Vec<DateTime<Utc>> = Vec::new();
                let mut last: Option<DateTime<Utc>> = None;
                for (s, base) in samples.iter().zip(&base_threat) {
                    if *base || !is_threat(&t.status_from(window(*s), &city, *s)) {
                        continue;
                    }
                    if last.is_none_or(|l| *s - l > Duration::minutes(10)) {
                        episodes.push(*s);
                    }
                    last = Some(*s);
                }
                let false_alarms = episodes
                    .iter()
                    .filter(|a| {
                        !city_impacts
                            .iter()
                            .any(|ex| *ex >= **a && *ex - **a <= Duration::minutes(25))
                    })
                    .count();
                let mut gains: Vec<i64> = city_impacts
                    .iter()
                    .zip(&base_first)
                    .filter_map(|(ex, base)| {
                        let new = first_warning(&t, *ex)?;
                        Some((base.unwrap_or(*ex) - new).num_seconds().max(0))
                    })
                    .collect();
                gains.sort();
                let mut abs: Vec<i64> = caught.iter().map(|e| e.abs()).collect();
                abs.sort();
                println!(
                    "corridor {corridor:4.0} km range {range:3.0} km | caught {:2}/{} (median |err| {:4}s) | \
                     Вінниця: extra episodes {:2}, without impact {:2}, median earlier warning {:4}s",
                    caught.len(),
                    missed.len(),
                    abs.get(abs.len() / 2).copied().unwrap_or(0),
                    episodes.len(),
                    false_alarms,
                    gains.get(gains.len() / 2).copied().unwrap_or(0)
                );
            }
        }
    }
}
