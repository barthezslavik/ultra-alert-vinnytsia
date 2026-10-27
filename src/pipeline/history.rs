//! Channel history from the public t.me/s web preview, stored as JSON lines
//! `{"id", "date", "text"}` sorted by id.

use super::Channel;
use crate::telegram::{self, Post};
use anyhow::{Context, Result, bail};
use chrono::{DateTime, Utc};
use std::collections::BTreeMap;
use std::time::Duration;

/// Pages re-read inside known history on an incremental update: the operator edits recent
/// posts ("15:39 - вибух").
const OVERLAP_PAGES: usize = 3;

pub fn load(ch: &Channel, since: &str) -> Result<Vec<Post>> {
    let text = super::read(ch.history)?;
    let mut posts = Vec::new();
    for line in text.lines().filter(|l| !l.trim().is_empty()) {
        let v: serde_json::Value = serde_json::from_str(line)?;
        let date = v["date"].as_str().context("date")?;
        if date < since {
            continue;
        }
        posts.push(Post {
            channel: ch.name.to_owned(),
            id: v["id"].as_u64().context("id")?,
            time: DateTime::parse_from_rfc3339(date)?.with_timezone(&Utc),
            text: v["text"].as_str().unwrap_or_default().to_owned(),
        });
    }
    Ok(posts)
}

fn save(ch: &Channel, posts: &BTreeMap<u64, Post>) -> Result<()> {
    let mut out = String::new();
    for p in posts.values() {
        out += &serde_json::json!({ "id": p.id, "date": p.time.to_rfc3339(), "text": p.text })
            .to_string();
        out.push('\n');
    }
    std::fs::write(ch.history, out).with_context(|| format!("write {}", ch.history))
}

async fn page(client: &reqwest::Client, ch: &Channel, before: Option<u64>) -> Result<Vec<Post>> {
    for attempt in 0..5u64 {
        match telegram::fetch_page(client, ch.name, before).await {
            Ok(posts) => return Ok(posts),
            Err(err) => {
                eprintln!("retry {} before={before:?}: {err}", ch.name);
                tokio::time::sleep(Duration::from_secs(2 + attempt * 3)).await;
            }
        }
    }
    bail!("{}: page before={before:?} failed 5 times", ch.name)
}

/// Walks back from the newest post. Incremental: stops a few pages into known history; full:
/// rereads all of it. Posts the channel has since deleted stay as they were stored.
/// Returns (stored, new).
pub async fn update(ch: &Channel, incremental: bool) -> Result<(usize, usize)> {
    let mut known: BTreeMap<u64, Post> = BTreeMap::new();
    if std::path::Path::new(ch.history).exists() {
        known = load(ch, "")?.into_iter().map(|p| (p.id, p)).collect();
    }
    let known_max = if incremental {
        known.keys().next_back().copied().unwrap_or(0)
    } else {
        0
    };
    let client = reqwest::Client::new();
    let (mut before, mut overlap, mut new) = (None, 0, 0);
    let mut seen = std::collections::HashSet::new();
    loop {
        let batch = page(&client, ch, before).await?;
        let oldest = batch.iter().map(|p| p.id).min();
        // Channels kept only since a date are not followed back to their first post.
        let reached_since = !ch.fetch_since.is_empty()
            && batch
                .iter()
                .map(|p| p.time.to_rfc3339())
                .min()
                .is_some_and(|t| t.as_str() < ch.fetch_since);
        let mut fresh = false;
        for p in batch {
            if seen.insert(p.id) {
                fresh = true;
                new += usize::from(!known.contains_key(&p.id));
                known.insert(p.id, p);
            }
        }
        let Some(oldest) = oldest.filter(|_| fresh) else {
            break;
        };
        before = Some(oldest);
        if reached_since {
            break;
        }
        if known_max > 0 && oldest <= known_max {
            overlap += 1;
            if overlap >= OVERLAP_PAGES {
                break;
            }
        }
        if oldest <= 1 {
            break;
        }
        tokio::time::sleep(Duration::from_millis(700)).await;
    }
    save(ch, &known)?;
    eprintln!(
        "{}: {} posts ({new} new) -> {}",
        ch.name,
        known.len(),
        ch.history
    );
    Ok((known.len(), new))
}
