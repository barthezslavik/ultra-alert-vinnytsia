//! Normalises place names from the labelled events and geocodes new ones with OSM Nominatim.
//! Every Nominatim request is cached by its query in data/nominatim_cache.json, so a query is
//! sent at most once ever; data/places.json holds the result per name (read by the gazetteer),
//! including "not found", so those are not looked up again either.

use super::{CHANNELS, PLACES, label};
use anyhow::Result;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::time::Duration;

const NOMINATIM_CACHE: &str = "data/nominatim_cache.json";

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Coord {
    pub lat: f64,
    pub lon: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub osm: Option<String>,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub region: bool,
    /// Other places in the oblast with the same name; None: not looked up yet.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub alt: Option<Vec<[f64; 2]>>,
}

/// Every spelling seen → canonical name; canonical name → coordinates (None: not found).
#[derive(Debug, Default, Serialize, Deserialize)]
pub struct Places {
    pub alias: BTreeMap<String, String>,
    pub coords: BTreeMap<String, Option<Coord>>,
}

impl Places {
    pub fn load() -> Result<Self> {
        Ok(match std::fs::read_to_string(PLACES) {
            Ok(text) => serde_json::from_str(&text)?,
            Err(_) => Self::default(),
        })
    }

    fn save(&self) -> Result<()> {
        Ok(std::fs::write(PLACES, serde_json::to_string_pretty(self)?)?)
    }

    pub fn canonical<'a>(&'a self, name: &'a str) -> &'a str {
        self.alias.get(name).map_or(name, String::as_str)
    }

    /// Coordinates of a town (not a neighbouring oblast).
    pub fn town(&self, name: &str) -> Option<(f64, f64)> {
        match self.coords.get(self.canonical(name)) {
            Some(Some(c)) if !c.region => Some((c.lat, c.lon)),
            _ => None,
        }
    }
}

/// Parts of Vinnytsia: all treated as the city itself.
const CITY_PARTS: &[&str] = &[
    "Вінниця",
    "Тяжилів",
    "Хутори",
    "ГПЗ",
    "Немирівське шосе",
    "Лука Мелешківська",
    "Лука-Мелешківська",
    "Старе місто",
    "Центр",
    "Київська",
    "Масложир",
    "Вишенька",
    "Чехова",
    "Замостя",
    "Аеропорт",
    "Поділля",
    "Вінницькі Хутори",
    "Гуменне",
    "Сабарів",
    "Пирогово",
];
/// Typos and spelling variants seen in the channels.
const ALIASES: &[(&str, &str)] = &[
    ("Копайгрод", "Копайгород"),
    ("Капайгород", "Копайгород"),
    ("Черепашенці", "Черепашинці"),
    ("Муровані-Курилівці", "Муровані Курилівці"),
    ("Могилів-подільський", "Могилів-Подільський"),
    ("Луку-Барську", "Лука-Барська"),
    ("Сустиски", "Сутиски"),
    ("Сутисики", "Сутиски"),
    ("Вапянрку", "Вапнярка"),
    ("Муровані-Курлівці", "Муровані Курилівці"),
    ("Могилів Подільський", "Могилів-Подільський"),
    ("Могилів-Подільский", "Могилів-Подільський"),
    ("Могилів-Поділський", "Могилів-Подільський"),
    ("Мог Подільський", "Могилів-Подільський"),
    ("Могилів", "Могилів-Подільський"),
    ("Житомирищну", "Житомирщина"),
    // "Козятинчата, займіть укриття": the people of Козятин.
    ("Козятинчата", "Козятин"),
    ("Нову Ушицю", "Нова Ушиця"),
    ("Перепільченці", "Перепільчинці"),
    ("Оридинці", "Ординці"),
    ("Бушу", "Буша"),
    ("Зарваниці", "Зарванці"),
    ("Пішанку", "Піщанка"),
    ("Стадницю", "Стадниця"),
];
/// Neighbouring oblasts: rough centroids, only used as "where the drone comes from".
const REGIONS: &[(&str, f64, f64)] = &[
    ("Хмельниччина", 49.42, 26.98),
    ("Житомирщина", 50.25, 28.66),
    ("Київщина", 50.2, 30.3),
    ("Черкащина", 49.2, 31.5),
    ("Одещина", 47.6, 29.9),
    ("Миколаївщина", 47.4, 31.7),
    ("Кіровоградщина", 48.5, 31.5),
    ("Буковина", 48.3, 25.9),
];
/// Not places: the oblast itself, directions, and common words codex took for names
/// ("на селище Крижопіль" -> "Селище").
const SKIP: &[&str] = &[
    "Селище",
    "Село",
    "Водоканал",
    "Вокзал",
    "Електромережа",
    "Корея",
    "Південь",
    "Вінниччина",
    "Вінницька область",
    "Україна",
    "південь",
    "схід",
    "Крим",
    "Київ",
    "Перемога",
    "Дружба",
    "Високе",
];
/// Vinnytsia oblast with a margin: (lat_min, lon_min, lat_max, lon_max).
const BBOX: (f64, f64, f64, f64) = (47.9, 26.9, 50.2, 30.4);

fn alias(name: &str) -> &str {
    ALIASES
        .iter()
        .find(|(a, _)| *a == name)
        .map_or(name, |(_, c)| c)
}

fn canonical(name: &str, known: &BTreeSet<String>) -> String {
    let name = alias(name.trim());
    if CITY_PARTS.contains(&name) {
        return "Вінниця".into();
    }
    // Accusative left over from regex parsing: Жмеринку -> Жмеринка, Вінницю -> Вінниця.
    for (acc, nom) in [('у', 'а'), ('ю', 'я')] {
        if let Some(stem) = name.strip_suffix(acc) {
            let nominative = format!("{stem}{nom}");
            if known.contains(&nominative) {
                return canonical(&nominative, known);
            }
        }
    }
    name.to_owned()
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct Hit {
    lat: String,
    lon: String,
    display_name: String,
    /// OSM class and type ("place"/"village", "waterway"/"river"); absent in old cache entries.
    #[serde(default)]
    class: String,
    #[serde(default, rename = "type")]
    kind: String,
}

/// Nominatim search results for `q`, from the cache when this query was ever sent before.
struct Nominatim {
    client: reqwest::Client,
    cache: BTreeMap<String, Vec<Hit>>,
}

impl Nominatim {
    fn new() -> Result<Self> {
        let cache = match std::fs::read_to_string(NOMINATIM_CACHE) {
            Ok(text) => serde_json::from_str(&text)?,
            Err(_) => BTreeMap::new(),
        };
        Ok(Self {
            client: reqwest::Client::new(),
            cache,
        })
    }

    async fn search(&mut self, q: &str) -> Result<Vec<Hit>> {
        if let Some(hits) = self.cache.get(q) {
            return Ok(hits.clone());
        }
        tokio::time::sleep(Duration::from_millis(1100)).await; // usage policy: 1 request/s
        let url = reqwest::Url::parse_with_params(
            "https://nominatim.openstreetmap.org/search",
            &[
                ("q", q),
                ("format", "json"),
                ("countrycodes", "ua"),
                ("limit", "10"),
                ("accept-language", "uk"),
            ],
        )?;
        let text = self
            .client
            .get(url)
            .header("User-Agent", "ultra-alert/0.1")
            .timeout(Duration::from_secs(20))
            .send()
            .await?
            .error_for_status()?
            .text()
            .await?;
        let hits: Vec<Hit> = serde_json::from_str(&text)?;
        self.cache.insert(q.to_owned(), hits.clone());
        std::fs::write(NOMINATIM_CACHE, serde_json::to_string_pretty(&self.cache)?)?;
        Ok(hits)
    }

    /// Other villages in Vinnytsia oblast named exactly `name`, at least 3 km from (lat, lon)
    /// and from each other: villages only, not rivers, stations or parts of towns. A hromada
    /// centre has none: "на Липовець" means the town, not the village of the same name.
    async fn namesakes(&mut self, name: &str, lat: f64, lon: f64) -> Result<Vec<[f64; 2]>> {
        let mut found: Vec<[f64; 2]> = vec![[lat, lon]];
        for h in self.search(&format!("{name}, Вінницька область")).await? {
            let own = h
                .display_name
                .split(',')
                .next()
                .is_some_and(|n| n.trim() == name);
            let village = matches!(
                (h.class.as_str(), h.kind.as_str()),
                ("place", "village" | "hamlet" | "town") | ("boundary", "administrative")
            );
            if !own || !h.display_name.contains("Вінницька область") || !village {
                continue;
            }
            let p = [h.lat.parse::<f64>()?, h.lon.parse::<f64>()?];
            let near = |q: &[f64; 2]| crate::predict::haversine_km(p[0], p[1], q[0], q[1]) <= 3.0;
            // "Липовець, Липовецька міська громада": the centre its hromada is named after.
            let stem: String = name.to_lowercase().chars().take(4).collect();
            let centre = h.display_name.split(',').nth(1).is_some_and(|g| {
                g.trim().to_lowercase().starts_with(&stem) && g.contains("громада")
            });
            if near(&found[0]) && centre {
                return Ok(Vec::new());
            }
            if !found.iter().any(near) {
                found.push(p);
            }
        }
        Ok(found.split_off(1))
    }

    async fn locate(&mut self, name: &str) -> Result<Option<Coord>> {
        if let Some((_, lat, lon)) = REGIONS.iter().find(|(r, _, _)| *r == name) {
            return Ok(Some(Coord {
                lat: *lat,
                lon: *lon,
                osm: None,
                region: true,
                alt: None,
            }));
        }
        for q in [format!("{name}, Вінницька область"), name.to_owned()] {
            for h in self.search(&q).await? {
                let (lat, lon) = (h.lat.parse::<f64>()?, h.lon.parse::<f64>()?);
                if (BBOX.0..=BBOX.2).contains(&lat) && (BBOX.1..=BBOX.3).contains(&lon) {
                    return Ok(Some(Coord {
                        lat,
                        lon,
                        osm: Some(h.display_name.chars().take(80).collect()),
                        region: false,
                        alt: None,
                    }));
                }
            }
        }
        Ok(None)
    }
}

/// Adds aliases for every name in the labelled events and coordinates for new ones.
pub async fn update() -> Result<()> {
    let mut names = BTreeSet::new();
    for ch in &CHANNELS {
        for post in label::load(ch)? {
            for e in post.events {
                names.extend(e.target.into_iter().chain(e.from_place).chain(e.via));
            }
        }
    }
    let known: BTreeSet<String> = names.iter().map(|n| alias(n).to_owned()).collect();
    let mut places = Places::load()?;
    let mut nominatim = Nominatim::new()?;
    let mut new = 0;
    for name in &names {
        let c = canonical(name, &known);
        places.alias.insert(name.clone(), c.clone());
        if SKIP.contains(&c.as_str()) || places.coords.contains_key(&c) {
            continue;
        }
        let found = nominatim.locate(&c).await?;
        eprintln!("{c}: {found:?}");
        places.coords.insert(c, found);
        new += 1;
    }
    // Same-named villages: one cached query per place, once.
    let names: Vec<String> = places.coords.keys().cloned().collect();
    for name in names {
        if let Some(Some(c)) = places.coords.get(&name)
            && c.alt.is_none()
            && !c.region
        {
            let (lat, lon) = (c.lat, c.lon);
            let alt = nominatim.namesakes(&name, lat, lon).await?;
            if !alt.is_empty() {
                eprintln!("{name}: {} more in the oblast", alt.len());
            }
            if let Some(Some(c)) = places.coords.get_mut(&name) {
                c.alt = Some(alt);
            }
        }
    }
    for skip in SKIP {
        places.coords.remove(*skip);
    }
    places.alias.retain(|_, c| !SKIP.contains(&c.as_str()));
    places.save()?;
    let missing: Vec<&String> = places
        .coords
        .iter()
        .filter(|(_, v)| v.is_none())
        .map(|(k, _)| k)
        .collect();
    eprintln!(
        "{} places ({new} new), not found: {missing:?}",
        places.coords.len()
    );
    Ok(())
}

/// Parts of Vinnytsia the operators name when a drone is about to cross the city: display
/// name, the pattern that finds it in a post, and Nominatim queries, the first hit inside the
/// city counting. Written with coordinates to data/districts.json.
const DISTRICTS: [(&str, &str, &[&str]); 16] = [
    ("Тяжилів", "тяжил", &["Тяжилів, Вінниця"]),
    (
        "Лука-Мелешківська",
        r"\bлука\b|\bлук[уиі][\s-]+мелешк",
        &["Лука-Мелешківська"],
    ),
    ("Вінницькі Хутори", "хутор", &["Вінницькі Хутори"]),
    (
        "ГПЗ",
        "гпз",
        &["ГПЗ, Вінниця", "Підшипниковий завод, Вінниця"],
    ),
    ("Старе місто", "старе місто", &["Старе місто, Вінниця"]),
    (
        "Немирівське шосе",
        r"немирівськ\w* шосе",
        &["Немирівське шосе, Вінниця"],
    ),
    ("Замостя", "замост", &["Замостя, Вінниця"]),
    ("Вишенька", "вишеньк", &["Вишенька, Вінниця"]),
    ("Київська", r"київськ[аій]\b", &["Київська вулиця, Вінниця"]),
    (
        "Чехова",
        "чехова",
        &["вулиця Чехова, Вінниця", "вулиця Антона Чехова, Вінниця"],
    ),
    ("Поділля", "поділл", &["Поділля, Вінниця"]),
    ("Масложир", "масложир", &["Масложир, Вінниця"]),
    (
        "Аеропорт",
        "аеропорт",
        &["Аеропорт Вінниця", "Міжнародний аеропорт Вінниця"],
    ),
    ("Гуменне", "гуменн", &["Гуменне"]),
    ("Сабарів", "сабар", &["Сабарів, Вінниця"]),
    ("Центр", r"\bцентр\b", &["Соборна вулиця, Вінниця"]),
];
const DISTRICTS_FILE: &str = "data/districts.json";
/// Vinnytsia with its suburbs: (south, west, north, east).
const CITY_BOX: (f64, f64, f64, f64) = (49.13, 28.33, 49.32, 28.75);

#[derive(Debug, Serialize)]
struct District {
    name: &'static str,
    pattern: &'static str,
    lat: f64,
    lon: f64,
    osm: String,
}

/// Geocodes the city districts (one cached query each) into data/districts.json.
pub async fn districts() -> Result<()> {
    let mut nominatim = Nominatim::new()?;
    let mut out = Vec::new();
    for (name, pattern, queries) in DISTRICTS {
        // Inside the city; the object named like the district first ("Замостя, …" rather than
        // the railway station in it).
        let mut hits = Vec::new();
        for q in queries {
            for h in nominatim.search(q).await? {
                let (lat, lon) = (h.lat.parse::<f64>()?, h.lon.parse::<f64>()?);
                if (CITY_BOX.0..=CITY_BOX.2).contains(&lat)
                    && (CITY_BOX.1..=CITY_BOX.3).contains(&lon)
                {
                    hits.push((lat, lon, h.display_name));
                }
            }
        }
        let named = |osm: &String| osm.split(',').next().is_some_and(|n| n.trim() == name);
        let found = hits
            .iter()
            .position(|h| named(&h.2))
            .or((!hits.is_empty()).then_some(0))
            .map(|i| hits.swap_remove(i))
            .map(|(lat, lon, osm)| (lat, lon, osm.chars().take(90).collect::<String>()));
        match found {
            Some((lat, lon, osm)) => {
                eprintln!("{name}: {lat:.4}, {lon:.4}  {osm}");
                out.push(District {
                    name,
                    pattern,
                    lat,
                    lon,
                    osm,
                });
            }
            None => eprintln!("{name}: NOT FOUND"),
        }
    }
    std::fs::write(DISTRICTS_FILE, serde_json::to_string_pretty(&out)?)?;
    Ok(())
}
