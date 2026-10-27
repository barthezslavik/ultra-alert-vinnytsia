//! Impacts in every town from both channels' labelled explosions: the ground truth for the
//! ETA backtest and the refit. Written to data/impacts.json.

use super::geocode::Places;
use super::label::{self, Labelled};
use super::{CHANNELS, IMPACTS};
use anyhow::Result;
use chrono::{DateTime, Duration, Local, NaiveTime, TimeZone, Utc};
use serde::Serialize;
use std::collections::BTreeSet;

/// Not an explosion of its own but news of one already reported.
static AFTER: std::sync::LazyLock<regex::Regex> = std::sync::LazyLock::new(|| {
    regex::Regex::new(r"момент|\bдим\w*|після вибух|звук\w* вибух|наслід|постраждал|\bзмі\b").unwrap()
});
/// Explosion posts at the same place this close together are one impact.
const MERGE: Duration = Duration::minutes(4);
/// Only impacts with a jet Shahed reported this shortly before: older attacks and other
/// weapons are not what the model predicts.
const JET_WINDOW: Duration = Duration::minutes(20);

#[derive(Debug, Serialize)]
pub struct Impact {
    pub t: DateTime<Utc>,
    pub place: String,
    pub src: BTreeSet<&'static str>,
}

fn post_time(p: &Labelled) -> Option<DateTime<Utc>> {
    Some(
        DateTime::parse_from_rfc3339(&p.date)
            .ok()?
            .with_timezone(&Utc),
    )
}

/// "15:39 - У місті пролунав ВИБУХ" added by an edit: the real time, local (Kyiv).
fn note_time(p: &Labelled, posted: DateTime<Utc>) -> Option<DateTime<Utc>> {
    p.events
        .iter()
        .flat_map(|e| &e.times)
        .filter(|t| t.kind == "explosion")
        .find_map(|t| {
            let (h, m) = t.hhmm.split_once(':')?;
            let time = NaiveTime::from_hms_opt(h.parse().ok()?, m.parse().ok()?, 30)?;
            let day = posted.with_timezone(&Local).date_naive();
            let at = Local
                .from_local_datetime(&day.and_time(time))
                .earliest()?
                .with_timezone(&Utc);
            (at >= posted && at - posted <= Duration::hours(3)).then_some(at)
        })
}

pub fn build(places: &Places) -> Result<Vec<Impact>> {
    let mut raw = Vec::new();
    let mut jets = Vec::new();
    for ch in &CHANNELS {
        for p in label::load(ch)? {
            let Some(posted) = post_time(&p) else {
                continue;
            };
            // "МОМЕНТ СИЛЬНОГО ВИБУХУ У ВІННИЦІ": a video of an explosion already reported, posted
            // 10-30 min later; on its own it would be a second, made-up impact. So are the smoke
            // after it, the sound "the whole city heard", the casualties, the local media.
            let lower = p.text.to_lowercase();
            // "ВИБУХ у Вінницькому районі", "біля ЛІТИНА / ХМІЛЬНИКА": somewhere in a district or
            // one of two towns, not a hit on a town to time an ETA against.
            let explosions = p.events.iter().filter(|e| e.kind == "explosion").count();
            let video = AFTER.is_match(&lower) || lower.contains("район") || explosions > 1;
            for e in &p.events {
                let target = e.target.as_deref().map(|n| places.canonical(n));
                match e.kind.as_str() {
                    "explosion" => {
                        if video {
                            continue;
                        }
                        if let Some(t) = target.filter(|t| places.town(t).is_some()) {
                            raw.push((
                                note_time(&p, posted).unwrap_or(posted),
                                t.to_owned(),
                                ch.name,
                            ));
                        }
                    }
                    "track" | "shelter" if e.jet == Some(true) => {
                        let has_place = target
                            .into_iter()
                            .chain(e.via.iter().map(String::as_str))
                            .any(|n| places.town(n).is_some());
                        if has_place {
                            jets.push(posted);
                        }
                    }
                    _ => {}
                }
            }
        }
    }
    raw.sort_by_key(|r| r.0);
    let mut merged: Vec<Impact> = Vec::new();
    for (t, place, ch) in raw {
        match merged
            .iter_mut()
            .find(|m| m.place == place && (t - m.t).abs() <= MERGE)
        {
            Some(m) => {
                m.t = m.t.min(t);
                m.src.insert(ch);
            }
            None => merged.push(Impact {
                t,
                place,
                src: BTreeSet::from([ch]),
            }),
        }
    }
    merged.retain(|m| jets.iter().any(|j| *j >= m.t - JET_WINDOW && *j < m.t));
    Ok(merged)
}

/// Rebuilds data/impacts.json; returns how many impacts there are.
pub fn update() -> Result<usize> {
    let impacts = build(&Places::load()?)?;
    std::fs::write(IMPACTS, serde_json::to_string_pretty(&impacts)?)?;
    Ok(impacts.len())
}
