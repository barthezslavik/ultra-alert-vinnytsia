//! The channel's history on this Mac, for the map, the replays and the odds: its posts since
//! PPO_SINCE in ~/Library/Application Support/ultra-alert/history.bin (gzipped JSON lines),
//! fetched from the public t.me/s preview on first launch, then kept up with the posts the
//! app reads and their edits. Not in git: every Mac fetches its own.

use crate::telegram::{self, Post};
use chrono::{DateTime, Utc};
use flate2::Compression;
use flate2::read::GzDecoder;
use flate2::write::GzEncoder;
use std::collections::BTreeMap;
use std::io::{Read, Write};
use std::path::PathBuf;
use std::time::Duration;

/// Pages re-read inside known history when catching up: the operator edits recent posts
/// ("15:39 - вибух").
const OVERLAP_PAGES: usize = 3;

pub struct Archive {
    path: PathBuf,
    channel: String,
    posts: BTreeMap<u64, Post>,
}

impl Archive {
    /// The stored history, empty when there is none yet.
    pub fn load(path: PathBuf, channel: &str) -> Self {
        let mut text = String::new();
        let read = std::fs::File::open(&path)
            .map(|f| GzDecoder::new(f).read_to_string(&mut text).is_ok())
            .unwrap_or(false);
        let posts = if read { parse(&text, channel) } else { Vec::new() };
        Self {
            path,
            channel: channel.to_owned(),
            posts: posts.into_iter().map(|p| (p.id, p)).collect(),
        }
    }

    /// The posts built into the app (build.rs), where this Mac has none of its own yet: a
    /// released app then fetches only what came after it was built.
    pub fn seed(&mut self, gz: &[u8]) -> usize {
        let mut text = String::new();
        if GzDecoder::new(gz).read_to_string(&mut text).is_err() {
            return 0;
        }
        let mut added = 0;
        for p in parse(&text, &self.channel) {
            if let std::collections::btree_map::Entry::Vacant(e) = self.posts.entry(p.id) {
                e.insert(p);
                added += 1;
            }
        }
        added
    }

    pub fn posts(&self) -> Vec<Post> {
        self.posts.values().cloned().collect()
    }

    /// Takes the channel's new or edited posts; true when there were any.
    pub fn add(&mut self, posts: impl IntoIterator<Item = Post>) -> bool {
        let mut changed = false;
        for p in posts.into_iter().filter(|p| p.channel == self.channel) {
            if self.posts.get(&p.id).map(|q| &q.text) != Some(&p.text) {
                changed = true;
                self.posts.insert(p.id, p);
            }
        }
        changed
    }

    pub fn save(&self) -> std::io::Result<()> {
        let mut gz = GzEncoder::new(Vec::new(), Compression::default());
        for p in self.posts.values() {
            let line = serde_json::json!({ "date": p.time.to_rfc3339(), "id": p.id, "text": p.text });
            writeln!(gz, "{line}")?;
        }
        std::fs::write(&self.path, gz.finish()?)
    }

    /// Walks back from the newest post until `since`, or until a few pages into what is
    /// already stored: the first launch fetches it all (some 100 pages), later ones the gap
    /// while the app was not running. Returns how many posts were new or edited. `progress`
    /// gets after each page how far back it has got, 0 to 1, and the date it reached.
    pub async fn catch_up(
        &mut self,
        client: &reqwest::Client,
        since: DateTime<Utc>,
        progress: &mut dyn FnMut(f64, DateTime<Utc>),
    ) -> usize {
        let now = Utc::now();
        let (mut before, mut known_pages, mut changed) = (None, 0, 0);
        loop {
            let Some(page) = self.page(client, before).await else {
                break;
            };
            let Some(oldest) = page.iter().min_by_key(|p| p.id).cloned() else {
                break;
            };
            let known = page.iter().all(|p| self.posts.contains_key(&p.id));
            changed += page
                .into_iter()
                .filter(|p| p.time >= since)
                .filter(|p| self.add([p.clone()]))
                .count();
            known_pages = if known { known_pages + 1 } else { 0 };
            let span = (now - since).num_seconds().max(1) as f64;
            progress(((now - oldest.time).num_seconds() as f64 / span).clamp(0.0, 1.0), oldest.time);
            if oldest.time < since || known_pages >= OVERLAP_PAGES {
                break;
            }
            before = Some(oldest.id);
            // Politely: one page at a time, like a reader scrolling back.
            tokio::time::sleep(Duration::from_millis(500)).await;
        }
        changed
    }

    async fn page(&self, client: &reqwest::Client, before: Option<u64>) -> Option<Vec<Post>> {
        for attempt in 0..5u64 {
            match telegram::fetch_page(client, &self.channel, before).await {
                Ok(posts) => return Some(posts),
                Err(err) => {
                    eprintln!("history: page before {before:?}: {err}");
                    tokio::time::sleep(Duration::from_secs(2 + attempt * 3)).await;
                }
            }
        }
        None
    }
}

/// Posts from JSON lines `{"date", "id", "text"}`.
pub fn parse(jsonl: &str, channel: &str) -> Vec<Post> {
    jsonl
        .lines()
        .filter_map(|l| serde_json::from_str::<serde_json::Value>(l).ok())
        .filter_map(|v| {
            Some(Post {
                channel: channel.to_owned(),
                id: v["id"].as_u64()?,
                time: DateTime::parse_from_rfc3339(v["date"].as_str()?)
                    .ok()?
                    .with_timezone(&Utc),
                text: v["text"].as_str()?.to_owned(),
            })
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn saved_history_reads_back() {
        let path = std::env::temp_dir().join(format!("ultra-alert-archive-{}.bin", std::process::id()));
        let mut a = Archive::load(path.clone(), "vinnytsiappo");
        let post = |id, text: &str| Post {
            channel: "vinnytsiappo".into(),
            id,
            time: DateTime::parse_from_rfc3339("2026-10-26T08:02:05Z").unwrap().with_timezone(&Utc),
            text: text.into(),
        };
        assert!(a.add([post(1, "🛵 на Тиврів"), post(2, "ВІННИЦЯ, займіть укриття")]));
        assert!(!a.add([post(1, "🛵 на Тиврів")]));
        assert!(a.add([post(1, "🛵 на Тиврів\n\n10:09 - вибух")]));
        a.save().unwrap();
        let b = Archive::load(path.clone(), "vinnytsiappo");
        let _ = std::fs::remove_file(&path);
        assert_eq!(b.posts(), a.posts());
        // The seed adds what is missing and leaves an edited post as this Mac has it.
        let mut gz = GzEncoder::new(Vec::new(), Compression::default());
        writeln!(gz, "{}", serde_json::json!({ "date": "2026-10-26T08:00:00Z", "id": 1, "text": "🛵 на Тиврів" })).unwrap();
        writeln!(gz, "{}", serde_json::json!({ "date": "2026-10-26T07:00:00Z", "id": 0, "text": "🛵 на Немирів" })).unwrap();
        let mut c = Archive::load(path.clone(), "vinnytsiappo");
        c.add(a.posts());
        assert_eq!(c.seed(&gz.finish().unwrap()), 1);
        assert_eq!(c.posts().len(), 3);
        assert!(c.posts().iter().any(|p| p.text.contains("10:09")));
        assert!(!std::fs::read(&path).is_ok());
    }
}
