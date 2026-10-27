//! Places the gazetteer does not know yet, found while the app runs: a post names "Юзвин",
//! the app asks OSM Nominatim for "Юзвин, Вінницька область" and the tracker knows it from
//! then on, also with an accusative ending made nominative ("на Пішанку"). Every Nominatim query
//! and every place found is kept in one file, so nothing is asked twice and a restart keeps
//! what was found. Names it cannot place (typos) wait for the maintainer's daily job, whose
//! places come with the next update.

use crate::gazetteer::{Gazetteer, Place};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::path::PathBuf;
use std::time::Duration;

/// Vinnytsia oblast with a margin: (lat_min, lon_min, lat_max, lon_max).
const BBOX: (f64, f64, f64, f64) = (47.9, 26.9, 50.2, 30.4);
/// A district or oblast by its adjective or "-щина" name, not a settlement.
static AREA: std::sync::LazyLock<regex::Regex> = std::sync::LazyLock::new(|| {
    regex::Regex::new(r"(ськ|цьк|ск)(ий|ого|ому|ім|ій|ої)$|щин[аиіуо]$").unwrap()
});
/// OSM place types that are a town or village, not a part of one.
const SETTLEMENTS: &[&str] = &["city", "town", "village", "hamlet"];

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Found {
    pub name: String,
    pub lat: f64,
    pub lon: f64,
}

#[derive(Default, Serialize, Deserialize)]
struct Cache {
    /// Nominatim query → the settlement it found in the oblast, or none.
    nominatim: BTreeMap<String, Option<Found>>,
    /// Spelling in a post → the place.
    places: BTreeMap<String, Found>,
}

pub struct Lookup {
    path: PathBuf,
    cache: Cache,
    client: reqwest::Client,
}

#[derive(Deserialize)]
struct Hit {
    lat: String,
    lon: String,
    name: String,
    display_name: String,
    #[serde(default)]
    class: String,
    #[serde(default, rename = "type")]
    kind: String,
}

impl Lookup {
    pub fn load(path: PathBuf) -> Self {
        let cache = std::fs::read_to_string(&path)
            .ok()
            .and_then(|t| serde_json::from_str(&t).ok())
            .unwrap_or_default();
        Self {
            path,
            cache,
            client: reqwest::Client::new(),
        }
    }

    /// Every place found so far, by the spelling a post used.
    pub fn found(&self) -> Vec<(String, Place)> {
        self.cache
            .places
            .iter()
            .map(|(spelling, f)| (spelling.clone(), place(f)))
            .collect()
    }

    /// The unknown places a post names, found and added to `gazetteer`; also returned to add
    /// them elsewhere.
    pub async fn resolve(&mut self, gazetteer: &mut Gazetteer, text: &str) -> Vec<(String, Place)> {
        // Parts of the city are the tracker's, not places to look up; nor districts and oblasts
        // ("Гайсинському", "Миколаївській", "Гайсинщині") or words in Latin ("Monobank").
        let names: Vec<String> = gazetteer
            .unknown_names(text)
            .into_iter()
            .filter(|n| !crate::predict::is_district(n) && !AREA.is_match(&n.to_lowercase()))
            .filter(|n| n.to_lowercase().chars().any(|c| matches!(c, 'а'..='я' | 'і' | 'ї' | 'є' | 'ґ')))
            .collect();
        if names.is_empty() {
            return Vec::new();
        }
        let mut out = Vec::new();
        for name in names {
            if let Some(f) = self.settlement(&name).await {
                out.push((name, f));
            }
        }
        for (spelling, f) in &out {
            self.cache.places.insert(spelling.clone(), f.clone());
            gazetteer.add(spelling, place(f));
        }
        self.save();
        out.into_iter().map(|(s, f)| (s, place(&f))).collect()
    }

    /// The oblast's town or village of this name, as written or with an accusative ending
    /// made nominative ("Пішанку" → "Пішанка").
    async fn settlement(&mut self, name: &str) -> Option<Found> {
        let mut variants = vec![title(name)];
        for (acc, nom) in [('у', 'а'), ('ю', 'я')] {
            if let Some(stem) = variants[0].strip_suffix(acc) {
                variants.push(format!("{stem}{nom}"));
            }
        }
        for v in variants {
            if let Some(f) = self.nominatim(&format!("{v}, Вінницька область")).await {
                return Some(f);
            }
        }
        None
    }

    async fn nominatim(&mut self, q: &str) -> Option<Found> {
        if let Some(cached) = self.cache.nominatim.get(q) {
            return cached.clone();
        }
        // Usage policy: at most one request a second.
        tokio::time::sleep(Duration::from_millis(1100)).await;
        let url = reqwest::Url::parse_with_params(
            "https://nominatim.openstreetmap.org/search",
            &[
                ("q", q),
                ("format", "json"),
                ("countrycodes", "ua"),
                ("limit", "10"),
                ("accept-language", "uk"),
            ],
        )
        .ok()?;
        let text = self
            .client
            .get(url)
            .header("User-Agent", "ultra-alert/0.1")
            .timeout(Duration::from_secs(20))
            .send()
            .await
            .ok()?
            .error_for_status()
            .ok()?
            .text()
            .await
            .ok()?;
        let hits: Vec<Hit> = serde_json::from_str(&text).ok()?;
        let found = hits.into_iter().find_map(|h| {
            let (lat, lon) = (h.lat.parse::<f64>().ok()?, h.lon.parse::<f64>().ok()?);
            // A village is sometimes only its boundary in OSM ("Стадниця", Вінницька громада).
            let boundary = h.class == "boundary"
                && h.kind == "administrative"
                && !["район", "громада", "область"].iter().any(|w| h.name.contains(w));
            ((h.class == "place" && SETTLEMENTS.contains(&h.kind.as_str()) || boundary)
                && h.display_name.contains("Вінницька область")
                && (BBOX.0..=BBOX.2).contains(&lat)
                && (BBOX.1..=BBOX.3).contains(&lon))
            .then_some(Found {
                name: h.name,
                lat,
                lon,
            })
        });
        // Only answers are cached: a network error is asked again next time.
        self.cache.nominatim.insert(q.to_owned(), found.clone());
        self.save();
        found
    }

    fn save(&self) {
        if let Ok(json) = serde_json::to_string_pretty(&self.cache) {
            let _ = std::fs::write(&self.path, json);
        }
    }
}

fn place(f: &Found) -> Place {
    Place {
        name: f.name.clone(),
        lat: f.lat,
        lon: f.lon,
        region: false,
        alt: Vec::new(),
    }
}

/// "ЮЗВИН" → "Юзвин", "медвеже вушко" → "Медвеже Вушко".
fn title(name: &str) -> String {
    name.split(' ')
        .map(|w| {
            let lower = w.to_lowercase();
            let mut chars = lower.chars();
            chars
                .next()
                .map_or_else(String::new, |c| c.to_uppercase().chain(chars).collect())
        })
        .collect::<Vec<_>>()
        .join(" ")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn titles() {
        assert_eq!(title("ЮЗВИН"), "Юзвин");
        assert_eq!(title("медвеже вушко"), "Медвеже Вушко");
    }
}
