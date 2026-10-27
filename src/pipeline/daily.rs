//! `ultra-alert daily`: fetch new posts, label them, geocode, rebuild impacts, refit the
//! model, score the app's own predictions against new impacts, rebuild and restart the app.
//! The report is appended to data/daily_report.md.

use super::geocode::Places;
use super::{CHANNELS, IMPACTS, MODEL, PLACES, PPO_SINCE, geocode, history, impacts, label, read};
use crate::gazetteer::CITY;
use crate::predict::Model;
use crate::predict::eval::{self, Backtest, Fit};
use crate::telegram::Post;
use anyhow::{Context, Result, bail};
use chrono::{DateTime, Duration, Local, Utc};
use std::path::PathBuf;

const REPORT: &str = "data/daily_report.md";
/// A threat shown this long before an impact counts as a warning for it.
const WARN_WINDOW: Duration = Duration::minutes(25);
/// How often and how long the app's reinstall waits for no drones over the oblast: 12 hours.
const QUIET_CHECK_EVERY: std::time::Duration = std::time::Duration::from_secs(5 * 60);
const QUIET_CHECKS: usize = 144;

/// Impacts at the time the explosion was heard: its post, less the posting delay.
fn impacts_list() -> Result<Vec<(DateTime<Utc>, String)>> {
    let v: Vec<serde_json::Value> = serde_json::from_str(&read(IMPACTS)?)?;
    let delay = Duration::milliseconds((model()?.impact_post_delay_s * 1000.0) as i64);
    v.iter()
        .map(|m| {
            let t = DateTime::parse_from_rfc3339(m["t"].as_str().context("t")?)?
                .with_timezone(&Utc)
                - delay;
            Ok((t, m["place"].as_str().context("place")?.to_owned()))
        })
        .collect()
}

fn ppo_posts() -> Result<Vec<Post>> {
    history::load(&CHANNELS[0], PPO_SINCE)
}

fn model() -> Result<Model> {
    Ok(Model::from_json(&read(MODEL)?))
}

/// Refit on the current data, logging progress.
pub fn fit_current(log: &mut dyn FnMut(String)) -> Result<Fit> {
    Ok(eval::fit(
        &read(PLACES)?,
        &model()?,
        ppo_posts()?,
        &impacts_list()?,
        log,
    ))
}

/// vinnytsiappo alone, vinnytsiarealll alone, and vinnytsiappo with fallback channels (read only
/// while it is silent): vinnytsiarealll, the regional ones, and all of them (the app).
pub fn backtests(verbose: bool) -> Result<Vec<Backtest>> {
    let (places, model, impacts) = (read(PLACES)?, model()?, impacts_list()?);
    let realll = |since| history::load(&CHANNELS[1], since);
    Ok(vec![
        eval::backtest(
            "vinnytsiappo",
            &places,
            &model,
            ppo_posts()?,
            &impacts,
            None,
            verbose,
        ),
        eval::backtest(
            "vinnytsiarealll",
            &places,
            &model,
            realll(CHANNELS[1].since)?,
            &impacts,
            None,
            verbose,
        ),
        // Merged as an equal source vinnytsiarealll made the ETA worse (2026-10-23); as a
        // fallback it only fills vinnytsiappo's silences.
        eval::backtest(
            "vinnytsiappo, fallback vinnytsiarealll",
            &places,
            &model,
            ppo_posts()?.into_iter().chain(realll(PPO_SINCE)?).collect(),
            &impacts,
            None,
            false,
        ),
        eval::backtest(
            "vinnytsiappo, fallback regional",
            &places,
            &model,
            ppo_posts()?.into_iter().chain(regional_posts()?).collect(),
            &impacts,
            None,
            false,
        ),
        eval::backtest(
            "vinnytsiappo, fallback all",
            &places,
            &model,
            ppo_posts()?
                .into_iter()
                .chain(realll(PPO_SINCE)?)
                .chain(regional_posts()?)
                .collect(),
            &impacts,
            None,
            verbose,
        ),
    ])
}

/// Every regional channel's posts since vinnytsiappo covers attacks (missing files: none).
fn regional_posts() -> Result<Vec<Post>> {
    let mut posts = Vec::new();
    for ch in &super::REGIONAL {
        if std::path::Path::new(ch.history).exists() {
            posts.extend(history::load(ch, PPO_SINCE)?);
        }
    }
    Ok(posts)
}

/// Accept the refit only if the ETA backtest on all history is not worse than today's model.
fn refit() -> Result<(Fit, bool, Vec<String>)> {
    let fit = fit_current(&mut |line| eprintln!("{line}"))?;
    // The refit chooses among parameter sets by their ETA at these very impacts, so judge it
    // by its leave-one-out error, not the in-sample one.
    let accepted = fit.eta_loo_median_s <= fit.current_eta_median_s * 1.05
        && fit.eta_within_60 + 1 >= fit.current_eta_within_60;
    let mut json: serde_json::Value = serde_json::from_str(&read(MODEL)?)?;
    let fitted = serde_json::to_value(&fit)?;
    let mut changed = Vec::new();
    for key in [
        "speed_kmh",
        "lead_km",
        "lead_per_km",
        "district_lead_km",
        "sigma_alert_s",
        "sigma_waypoint_s",
        "sigma_district_s",
    ] {
        let (old, new) = (&json["trajectory"][key], &fitted[key]);
        if old.as_f64() != new.as_f64() {
            changed.push(format!("{key} {old}→{new}"));
        }
    }
    if accepted && !changed.is_empty() {
        for key in fitted.as_object().context("fit")?.keys() {
            if json["trajectory"].get(key).is_some() {
                json["trajectory"][key] = fitted[key].clone();
            }
        }
        std::fs::write(MODEL, serde_json::to_string_pretty(&json)?)?;
    }
    Ok((fit, accepted, changed))
}

fn support_dir() -> PathBuf {
    PathBuf::from(std::env::var_os("HOME").unwrap_or_default())
        .join("Library/Application Support/ultra-alert")
}

/// Did vinnytsiappo name Vinnytsia (or its districts) in [from, to)?
fn channel_named_city(
    posts: &[label::Labelled],
    places: &Places,
    from: DateTime<Utc>,
    to: DateTime<Utc>,
) -> bool {
    posts.iter().any(|p| {
        let Ok(t) = DateTime::parse_from_rfc3339(&p.date) else {
            return false;
        };
        let t = t.with_timezone(&Utc);
        t >= from
            && t < to
            && p.events.iter().any(|e| {
                matches!(e.kind.as_str(), "track" | "shelter")
                    && e.target
                        .iter()
                        .chain(&e.via)
                        .any(|n| places.canonical(n) == CITY)
            })
    })
}

/// The app's last logged threat before each Vinnytsia impact since `since`.
fn live_scores(since: DateTime<Utc>) -> Result<Vec<String>> {
    let Ok(text) = std::fs::read_to_string(support_dir().join("predictions.jsonl")) else {
        return Ok(Vec::new());
    };
    let recs: Vec<(DateTime<Utc>, serde_json::Value)> = text
        .lines()
        .filter_map(|l| serde_json::from_str::<serde_json::Value>(l).ok())
        .filter_map(|r| {
            let at = DateTime::parse_from_rfc3339(r["logged_at"].as_str()?)
                .ok()?
                .with_timezone(&Utc);
            Some((at, r))
        })
        .collect();
    let Some(first) = recs.first().map(|r| r.0) else {
        return Ok(Vec::new());
    };
    let (places, labelled) = (Places::load()?, label::load(&CHANNELS[0])?);
    let start = since.max(first);
    let mut lines = Vec::new();
    for (ex, place) in impacts_list()? {
        if place != CITY || ex <= start {
            continue;
        }
        let at = ex.with_timezone(&Local).format("%m-%d %H:%M:%S");
        let window: Vec<&(DateTime<Utc>, serde_json::Value)> = recs
            .iter()
            .filter(|(t, r)| *t >= ex - WARN_WINDOW && *t < ex && r["kind"] == "threat")
            .collect();
        let (Some(first), Some(last)) = (window.first(), window.last()) else {
            lines.push(if channel_named_city(&labelled, &places, ex - WARN_WINDOW, ex) {
                format!("  - {at} NOT WARNED although the channel named the city: app/model error, check")
            } else {
                format!("  - {at} not warned: the channel did not name the city (source gap, not a model error)")
            });
            continue;
        };
        let eta = DateTime::parse_from_rfc3339(last.1["eta"].as_str().unwrap_or_default())?;
        lines.push(format!(
            "  - {at} ETA error {:+} s, warned {} s ahead, {}",
            (eta.with_timezone(&Utc) - ex).num_seconds(),
            (ex - first.0).num_seconds(),
            last.1["basis"].as_str().unwrap_or_default()
        ));
    }
    Ok(lines)
}

fn last_report_time() -> DateTime<Utc> {
    std::fs::read_to_string(REPORT)
        .ok()
        .and_then(|text| {
            text.lines()
                .filter_map(|l| l.strip_prefix("## "))
                .filter_map(|t| DateTime::parse_from_rfc3339(t.trim()).ok())
                .next_back()
                .map(|t| t.with_timezone(&Utc))
        })
        .unwrap_or_else(|| Utc::now() - Duration::days(1))
}

/// Commits data/ only, then pushes; other changes in the working tree are left alone.
fn commit_data(now: DateTime<Utc>) -> Result<()> {
    let git = |args: &[&str]| std::process::Command::new("git").args(args).status();
    if !git(&["add", "--", "data"])?.success() {
        bail!("git add data failed");
    }
    if git(&["diff", "--cached", "--quiet", "--", "data"])?.success() {
        eprintln!("no data changes to commit");
        // Earlier days' data may still be waiting.
        return push_data();
    }
    let message = format!(
        "daily: data update {}",
        now.with_timezone(&Local).format("%Y-%m-%d")
    );
    if !git(&["commit", "-q", "-m", &message, "--", "data"])?.success() {
        bail!("git commit failed");
    }
    eprintln!("committed data: {message}");
    push_data()
}

/// Pushes main when everything not yet on origin is daily data commits; other unpushed work
/// is left for the user to push.
fn push_data() -> Result<()> {
    let out = |args: &[&str]| -> Result<String> {
        let o = std::process::Command::new("git").args(args).output()?;
        if !o.status.success() {
            bail!(
                "git {}: {}",
                args.join(" "),
                String::from_utf8_lossy(&o.stderr).trim()
            );
        }
        Ok(String::from_utf8_lossy(&o.stdout).trim().to_owned())
    };
    if out(&["rev-parse", "--abbrev-ref", "HEAD"])? != "main" {
        eprintln!("not on main: data not pushed");
        return Ok(());
    }
    out(&["fetch", "-q", "origin", "main"])?;
    let unpushed = out(&["log", "--format=%s", "origin/main..HEAD"])?;
    if let Some(other) = unpushed.lines().find(|s| !s.starts_with("daily:")) {
        eprintln!("unpushed non-data commit \"{other}\": data not pushed");
        return Ok(());
    }
    out(&["push", "-q", "origin", "main"])?;
    eprintln!("pushed data");
    Ok(())
}

pub fn run(install: bool) -> Result<()> {
    let runtime = tokio::runtime::Runtime::new()?;
    // Anyone else's Mac: the maintainer's data and model come with the code.
    if !maintainer() {
        let pulled = std::process::Command::new("git")
            .args(["pull", "-q", "--ff-only"])
            .status()?;
        if !pulled.success() {
            bail!("git pull failed");
        }
        return if install { install_when_quiet(&runtime) } else { Ok(()) };
    }
    let since = last_report_time();
    let now = Utc::now();
    for ch in &CHANNELS {
        runtime.block_on(history::update(ch, true))?;
    }
    for ch in &super::REGIONAL {
        runtime.block_on(history::update(ch, true))?;
    }
    for ch in &CHANNELS {
        label::label(ch, false)?;
    }
    runtime.block_on(geocode::update())?;
    impacts::update()?;
    let (fit, accepted, changed) = refit()?;
    let backtests = backtests(false)?;

    let mut lines = vec![format!("## {}", now.to_rfc3339()), String::new()];
    lines.push(format!(
        "- data: {} track transitions, {} impacts with vinnytsiappo coverage",
        fit.transitions, fit.impacts
    ));
    let verdict = match (accepted, changed.is_empty()) {
        (true, false) => format!("accepted {}", changed.join(", ")),
        (true, true) => "no change".into(),
        (false, _) => "rejected".into(),
    };
    lines.push(format!(
        "- refit: ETA median {:.0} s in-sample, {:.0} s leave-one-out, ≤60 s {}/{} \
         (current model {:.0} s, {}) -> {verdict}",
        fit.eta_median_s,
        fit.eta_loo_median_s,
        fit.eta_within_60,
        fit.eta_n,
        fit.current_eta_median_s,
        fit.current_eta_within_60
    ));
    for b in &backtests {
        lines.extend(b.lines.iter().map(|l| format!("- backtest: {}", l.trim())));
    }
    let live = live_scores(since)?;
    if live.is_empty() {
        lines.push("- no Vinnytsia impacts with app predictions since the last report".into());
    } else {
        lines.push("- app's live predictions for Vinnytsia impacts since the last report:".into());
        lines.extend(live);
    }
    let report = lines.join("\n");
    let mut all = std::fs::read_to_string(REPORT).unwrap_or_default();
    all += &report;
    all += "\n\n";
    std::fs::write(REPORT, all)?;
    println!("{report}");
    if let Err(err) = commit_data(now) {
        eprintln!("data not committed: {err}");
    }

    if !install {
        return Ok(());
    }
    install_when_quiet(&runtime)
}

/// Whether this checkout scrapes, labels and trains: the maintainer's, with the channels'
/// history and its Codex labels (both kept out of git) beside the code.
fn maintainer() -> bool {
    std::path::Path::new(label::CACHE).exists()
}

/// A restart in the middle of an attack would drop what the app saw before its first page:
/// waits for the oblast to be quiet, checking the channel every few minutes, then reinstalls.
fn install_when_quiet(runtime: &tokio::runtime::Runtime) -> Result<()> {
    for _ in 0..QUIET_CHECKS {
        let Some(status) = runtime.block_on(active_now())? else {
            let status = std::process::Command::new("tools/install.sh").status()?;
            if !status.success() {
                bail!("tools/install.sh failed");
            }
            return Ok(());
        };
        eprintln!("{} not reinstalled yet: {status}", Local::now().format("%H:%M"));
        std::thread::sleep(QUIET_CHECK_EVERY);
    }
    eprintln!("not reinstalled: never quiet; tomorrow");
    Ok(())
}

/// What the app shows right now if it is not "Загроз немає", from the channel's newest posts.
async fn active_now() -> Result<Option<String>> {
    let mut tracker = crate::predict::Tracker::new(
        crate::gazetteer::Gazetteer::from_json(&read(PLACES)?),
        model()?,
    );
    let client = reqwest::Client::new();
    let newest = crate::telegram::fetch(&client, CHANNELS[0].name).await?;
    let before = newest.iter().map(|p| p.id).min();
    let older = crate::telegram::fetch_page(&client, CHANNELS[0].name, before).await?;
    tracker.update(newest.into_iter().chain(older));
    Ok(match tracker.status(Utc::now()) {
        crate::predict::Status::Quiet => None,
        crate::predict::Status::Drones { .. } => Some("drones over the oblast".into()),
        crate::predict::Status::Threat(_) => Some("a threat to the city".into()),
        crate::predict::Status::Possible { .. } => Some("drones that may go for the city".into()),
        crate::predict::Status::Impact { .. } => Some("an impact just now".into()),
    })
}

/// `ultra-alert corridors`: do named districts lie along the incoming course? Per channel.
pub fn corridors() -> Result<Vec<String>> {
    let districts: Vec<eval::District> = serde_json::from_str(&read("data/districts.json")?)?;
    let (places, model) = (read(PLACES)?, model()?);
    let mut lines = Vec::new();
    for (ch, since) in [(&CHANNELS[0], PPO_SINCE), (&CHANNELS[1], CHANNELS[1].since)] {
        lines.push(format!("=== {}", ch.name));
        lines.extend(eval::corridors(
            &places,
            &model,
            history::load(ch, since)?,
            &districts,
        ));
    }
    lines.push("=== both channels".into());
    let both: Vec<Post> = ppo_posts()?
        .into_iter()
        .chain(history::load(&CHANNELS[1], PPO_SINCE)?)
        .collect();
    lines.extend(eval::corridors(&places, &model, both, &districts));
    Ok(lines)
}

/// `ultra-alert positions`: where the position error comes from (vinnytsiappo).
pub fn positions() -> Result<Vec<String>> {
    Ok(eval::positions(&read(PLACES)?, &model()?, ppo_posts()?))
}

/// `ultra-alert learn-positions`: can over/undershoot be predicted (vinnytsiappo)?
pub fn learn_positions() -> Result<Vec<String>> {
    Ok(eval::learn_positions(
        &read(PLACES)?,
        &model()?,
        ppo_posts()?,
    ))
}

/// `ultra-alert learn-routes`: do past tracks predict turns (vinnytsiappo)?
pub fn learn_routes() -> Result<Vec<String>> {
    Ok(eval::learn_routes(&read(PLACES)?, &model()?, ppo_posts()?))
}

/// `ultra-alert jumps`: where a new waypoint moved the drone forward (vinnytsiappo).
pub fn jumps() -> Result<Vec<String>> {
    Ok(eval::jumps(&read(PLACES)?, &model()?, ppo_posts()?))
}

/// `ultra-alert vacuum`: city alerts while no drone was tracked (vinnytsiappo).
pub fn vacuum() -> Result<Vec<String>> {
    Ok(eval::vacuum_alerts(
        &read(PLACES)?,
        &model()?,
        ppo_posts()?,
        &impacts_list()?,
    ))
}

/// `ultra-alert learn-eta`: can ML learn how far to trust the last post (vinnytsiappo)?
pub fn learn_eta() -> Result<Vec<String>> {
    Ok(eval::learn_eta(
        &read(PLACES)?,
        &model()?,
        ppo_posts()?,
        &impacts_list()?,
    ))
}
