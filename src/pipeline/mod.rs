//! Data pipeline that keeps the model trained: channel history → labelled events →
//! geocoded places → impacts → refit. Run from the repository root; everything lives in data/.
//!
//! `ultra-alert daily` runs it all (the LaunchAgent does so at 09:00); the steps are also
//! subcommands: scrape, label, geocode, impacts, fit, backtest.

pub mod daily;
pub mod geocode;
pub mod history;
pub mod impacts;
pub mod label;

use anyhow::{Context, Result, bail};

pub const MODEL: &str = "data/model.json";
pub const PLACES: &str = "data/places.json";
pub const IMPACTS: &str = "data/impacts.json";

/// A channel whose history we keep.
pub struct Channel {
    pub name: &'static str,
    pub history: &'static str,
    pub events: &'static str,
    /// Older posts are not labelled (before jet Shaheds there is nothing to learn from).
    pub since: &'static str,
    /// History is not fetched further back than this ("": back to the first post).
    pub fetch_since: &'static str,
}

/// vinnytsiappo is the live source; vinnytsiarealll adds impacts and a comparison.
pub const CHANNELS: [Channel; 2] = [
    Channel {
        name: "vinnytsiappo",
        history: "data/messages.jsonl",
        events: "data/events.jsonl",
        since: "",
        fetch_since: "",
    },
    Channel {
        name: "vinnytsiarealll",
        history: "data/vinnytsiarealll.jsonl",
        events: "data/events_vinnytsiarealll.jsonl",
        since: "2026-07-01",
        fetch_since: "",
    },
];

/// Channels covering several oblasts, Vinnytsia among them: a fallback the tracker reads only
/// while vinnytsiappo is silent. Not labelled.
pub const REGIONAL: [Channel; 2] = [
    Channel {
        name: "zahid_golovne_ua",
        history: "data/zahid.jsonl",
        events: "",
        since: "2026-10-05",
        fetch_since: "2026-10-05",
    },
    Channel {
        name: "povitryanatrivogaaa",
        history: "data/povitryana.jsonl",
        events: "",
        since: "2026-10-05",
        fetch_since: "2026-10-05",
    },
];

/// Since when vinnytsiappo covers attacks (its post 1 is a 2024 placeholder).
pub const PPO_SINCE: &str = "2026-10-05";

pub fn read(path: &str) -> Result<String> {
    std::fs::read_to_string(path)
        .with_context(|| format!("read {path} (run from the repository root)"))
}

pub fn cli(command: &str, args: &[String]) -> Result<()> {
    let flag = |name: &str| args.iter().any(|a| a == name);
    match command {
        "scrape" => {
            let runtime = tokio::runtime::Runtime::new()?;
            for ch in &CHANNELS {
                runtime.block_on(history::update(ch, !flag("--full")))?;
            }
            for ch in &REGIONAL {
                runtime.block_on(history::update(ch, !flag("--full")))?;
            }
        }
        "label" => {
            for ch in &CHANNELS {
                label::label(ch, flag("--dry"))?;
            }
        }
        "geocode" => tokio::runtime::Runtime::new()?.block_on(geocode::update())?,
        "districts" => tokio::runtime::Runtime::new()?.block_on(geocode::districts())?,
        // The app's lookup of unknown places for one post text, with the app's cache.
        "lookup" => {
            let text = args.first().context("lookup \"<post text>\"")?;
            let path = std::path::PathBuf::from(std::env::var("HOME")?)
                .join("Library/Application Support/ultra-alert/places_found.json");
            let mut lookup = crate::lookup::Lookup::load(path);
            let mut gazetteer = crate::gazetteer::Gazetteer::from_json(&read(PLACES)?);
            for (spelling, place) in lookup.found() {
                gazetteer.add(&spelling, place);
            }
            println!("unknown: {:?}", gazetteer.unknown_names(text));
            let found = tokio::runtime::Runtime::new()?.block_on(lookup.resolve(&mut gazetteer, text));
            for (spelling, p) in found {
                println!("{spelling}: {} {:.5},{:.5}", p.name, p.lat, p.lon);
            }
        }
        "impacts" => {
            let n = impacts::update()?;
            eprintln!("{n} impacts -> {IMPACTS}");
        }
        "fit" => {
            let fit = daily::fit_current(&mut |line| println!("{line}"))?;
            println!("{}", serde_json::to_string_pretty(&fit)?);
        }
        "backtest" => {
            for b in daily::backtests(true)? {
                println!("{}", b.lines.join("\n"));
            }
        }
        "positions" => println!("{}", daily::positions()?.join("\n")),
        "learn-positions" => println!("{}", daily::learn_positions()?.join("\n")),
        "learn-routes" => println!("{}", daily::learn_routes()?.join("\n")),
        "jumps" => println!("{}", daily::jumps()?.join("\n")),
        "vacuum" => println!("{}", daily::vacuum()?.join("\n")),
        "learn-eta" => println!("{}", daily::learn_eta()?.join("\n")),
        "corridors" => println!("{}", daily::corridors()?.join("\n")),
        "daily" => daily::run(!flag("--no-install"))?,
        other => bail!("unknown command {other}"),
    }
    Ok(())
}
