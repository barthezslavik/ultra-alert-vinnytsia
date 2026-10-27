//! Backtest and refit of the model on the channel history (`ultra-alert backtest` / `fit`,
//! and the daily job).

use super::*;
use chrono::Timelike;
use serde::Serialize;
use std::collections::BTreeSet;

/// Parameter sets whose median position error is within this of the best count as equally
/// good on positions.
const POSITION_TOLERANCE_KM: f64 = 0.5;

fn median_i(mut v: Vec<i64>) -> i64 {
    v.sort();
    v.get(v.len() / 2).copied().unwrap_or(0)
}

fn median(mut v: Vec<f64>) -> f64 {
    v.sort_by(f64::total_cmp);
    v.get(v.len() / 2).copied().unwrap_or(f64::NAN)
}

fn robust_sigma(v: &[f64]) -> f64 {
    let m = median(v.to_vec());
    median(v.iter().map(|x| (x - m).abs()).collect()) * 1.4826
}

fn resolve(
    gz: &Gazetteer,
    impacts: &[(DateTime<Utc>, String)],
    since: DateTime<Utc>,
) -> Vec<(DateTime<Utc>, Place)> {
    impacts
        .iter()
        .filter(|(t, _)| *t > since)
        .filter_map(|(t, name)| Some((*t, gz.place(name)?)))
        .collect()
}

pub struct Backtest {
    pub lines: Vec<String>,
    pub median_before_s: i64,
    pub within_60: usize,
    pub predicted: usize,
    pub median_first_alert_s: i64,
    pub missed: usize,
}

/// ETA at every impact (all towns) after the first post, measured just before the impact
/// and at the first alert. `realll_delay_s` sets vinnytsiarealll's posting delay.
pub fn backtest(
    label: &str,
    places_json: &str,
    model: &Model,
    posts: Vec<Post>,
    impacts: &[(DateTime<Utc>, String)],
    realll_delay_s: Option<f64>,
    verbose: bool,
) -> Backtest {
    let gz = Gazetteer::from_json(places_json);
    let start = posts.iter().map(|p| p.time).min().unwrap_or_default();
    let impacts = resolve(&gz, impacts, start);
    let mut t = Tracker::new(gz, model.clone());
    if let Some(d) = realll_delay_s {
        t.model.channel_delay_s.insert("vinnytsiarealll".into(), d);
    }
    t.update(posts);
    // Past routes for flying on past waypoints (same-day drones never count).
    t.routes = t.route_legs();

    let mut lines = vec![format!(
        "=== {label}: {} impacts in all towns",
        impacts.len()
    )];
    let mut groups: [(&str, Vec<i64>); 4] = [
        ("waypoint", vec![]),
        ("districts", vec![]),
        ("alert only", vec![]),
        ("course", vec![]),
    ];
    let (mut all, mut firsts, mut missed) = (vec![], vec![], vec![]);
    for (ex, place) in &impacts {
        let Status::Threat(p) = t.status_for(place, *ex - Duration::seconds(1)) else {
            missed.push(place.name.clone());
            continue;
        };
        let err = (p.eta - *ex).num_seconds();
        let first = match t.status_for(place, p.alert_time + Duration::seconds(1)) {
            Status::Threat(f) => (f.eta - *ex).num_seconds(),
            _ => err,
        };
        let g = match &p.basis {
            Basis::Waypoint(..) => 0,
            Basis::Districts => 1,
            Basis::Alert => 2,
            Basis::Course { .. } => 3,
        };
        groups[g].1.push(err.abs());
        all.push(err.abs());
        firsts.push(first.abs());
        if verbose {
            lines.push(format!(
                "{} {:<20} err {err:+5}s  first {first:+5}s  warning {:4}s  {:?}",
                ex.with_timezone(&Local).format("%m-%d %H:%M:%S"),
                place.name,
                (*ex - p.alert_time).num_seconds(),
                p.basis
            ));
        }
    }
    for (name, errs) in &groups {
        lines.push(format!(
            "  {name:<10} n={:2} median |err| {:4}s  <=60s: {}",
            errs.len(),
            median_i(errs.clone()),
            errs.iter().filter(|e| **e <= 60).count()
        ));
    }
    let b = Backtest {
        median_before_s: median_i(all.clone()),
        within_60: all.iter().filter(|e| **e <= 60).count(),
        predicted: all.len(),
        median_first_alert_s: median_i(firsts),
        missed: missed.len(),
        lines: Vec::new(),
    };
    lines.push(format!(
        "  ALL n={} median |err| before impact {}s (≤60s: {}) | at first alert {}s | not warned {}",
        b.predicted, b.median_before_s, b.within_60, b.median_first_alert_s, b.missed
    ));
    if verbose {
        lines.push(format!("  not warned: {missed:?}"));
    }
    Backtest { lines, ..b }
}

/// Result of a refit, with the ETA backtest of the fitted and of the current parameters.
#[derive(Debug, Serialize)]
pub struct Fit {
    pub speed_kmh: f64,
    pub lead_km: f64,
    pub lead_per_km: f64,
    pub district_lead_km: f64,
    pub sigma_alert_s: f64,
    pub sigma_waypoint_s: f64,
    pub sigma_district_s: f64,
    pub eta_median_s: f64,
    /// ETA median error of this fitting procedure on impacts it did not see: what the refit
    /// is really worth, as the choice itself is made on the impacts.
    pub eta_loo_median_s: f64,
    pub eta_within_60: usize,
    pub eta_n: usize,
    pub current_eta_median_s: f64,
    pub current_eta_within_60: usize,
    pub transitions: usize,
    pub impacts: usize,
}

/// 1. speed and L = a + b·d(X, Vinnytsia) on track transitions (positions only);
/// 2. district L on impacts, leave-one-out;
/// 3. ETA σ per basis from the errors at impacts in every town.
pub fn fit(
    places_json: &str,
    model: &Model,
    posts: Vec<Post>,
    impacts: &[(DateTime<Utc>, String)],
    log: &mut dyn FnMut(String),
) -> Fit {
    let gz = Gazetteer::from_json(places_json);
    let start = posts.iter().map(|p| p.time).min().unwrap_or_default();
    let impacts = resolve(&gz, impacts, start);
    let mut t = Tracker::new(gz, model.clone());
    t.update(posts.clone());
    // Past routes for flying on past waypoints (same-day drones never count).
    t.routes = t.route_legs();
    let tracks_at = |t: &Tracker, now: DateTime<Utc>| t.tracks(since_clear(&t.events(now)), now);

    // Track linking does not depend on the parameters: collect the cases once.
    let mut transitions: Vec<(Track, DateTime<Utc>, Place)> = Vec::new();
    for p in &posts {
        let before = tracks_at(&t, p.time - Duration::milliseconds(1));
        for tr in tracks_at(&t, p.time) {
            let same = |b: &&Track| {
                b.points[0].first == tr.points[0].first && b.points[0].place == tr.points[0].place
            };
            if let Some(b) = before.iter().find(same)
                && tr.points.len() > b.points.len()
                && b.course().is_some()
                && !tr.head().district
            {
                transitions.push((b.clone(), tr.head().first, tr.head().place.clone()));
            }
        }
    }
    let at_impact: Vec<(Vec<Event>, DateTime<Utc>, Place)> = impacts
        .iter()
        .map(|(ex, place)| (t.events(*ex - Duration::seconds(1)), *ex, place.clone()))
        .collect();
    log(format!(
        "transitions: {} | impacts: {}",
        transitions.len(),
        at_impact.len()
    ));

    let eta_errors = |t: &Tracker| -> Vec<(Basis, f64)> {
        at_impact
            .iter()
            .filter_map(|(events, ex, place)| {
                match t.status_from(events, place, *ex - Duration::seconds(1)) {
                    Status::Threat(p) => Some((p.basis, (p.eta - *ex).num_seconds() as f64)),
                    _ => None,
                }
            })
            .collect()
    };
    let summary = |errs: &[(Basis, f64)]| {
        let abs: Vec<f64> = errs.iter().map(|(_, e)| e.abs()).collect();
        (
            median(abs.clone()),
            abs.iter().filter(|x| **x <= 60.0).count(),
            abs.len(),
        )
    };
    let current = summary(&eta_errors(&t));

    // 1. Positions: when "на X" is posted the drone should be L(X) short of X.
    let mut rows = Vec::new();
    for v in (0..=6).map(|i| 375.0 + i as f64 * 25.0) {
        for a in (0..=16).map(|i| i as f64 * 2.5) {
            for b in (0..=10).map(|i| i as f64 * 0.05) {
                t.model.speed_kmh = v;
                t.model.lead_km = a;
                t.model.lead_per_km = b;
                let errs: Vec<f64> = transitions
                    .iter()
                    .map(|(tr, time, place)| {
                        (t.estimate(tr, place, *time).km - t.lead_at(place)).abs()
                    })
                    .collect();
                rows.push((v, a, b, median(errs)));
            }
        }
    }
    rows.sort_by(|x, y| x.3.total_cmp(&y.3));
    for (v, a, b, e) in rows.iter().take(3) {
        log(format!(
            "positions: v {v:.0} L = {a:.1} + {b:.2}·d -> median {e:.1} km"
        ));
    }

    // 2. Positions leave a plateau of near-equal parameter sets. Within it, with the district
    // L, pick the set with the best ETA at impacts; leave-one-out shows what that choice is
    // worth on an impact it did not see.
    let plateau: Vec<(f64, f64, f64, f64)> = rows
        .iter()
        .filter(|r| r.3 <= rows[0].3 + POSITION_TOLERANCE_KM)
        .copied()
        .collect();
    let leads: Vec<f64> = (0..=40).map(|i| i as f64 * 0.75).collect();
    // errors[c][l][k]: ETA error at impact k for plateau set c and district lead l.
    let errors: Vec<Vec<Vec<Option<f64>>>> = plateau
        .iter()
        .map(|(v, a, b, _)| {
            t.model.speed_kmh = *v;
            t.model.lead_km = *a;
            t.model.lead_per_km = *b;
            leads
                .iter()
                .map(|l| {
                    t.model.district_lead_km = *l;
                    at_impact
                        .iter()
                        .map(|(events, ex, place)| {
                            match t.status_from(events, place, *ex - Duration::seconds(1)) {
                                Status::Threat(p) => Some((p.eta - *ex).num_seconds() as f64),
                                _ => None,
                            }
                        })
                        .collect()
                })
                .collect()
        })
        .collect();
    let score = |c: usize, l: usize, skip: Option<usize>| {
        median(
            errors[c][l]
                .iter()
                .enumerate()
                .filter(|(k, _)| Some(*k) != skip)
                .filter_map(|(_, e)| e.map(f64::abs))
                .collect(),
        )
    };
    // Ties go to the better position error (plateau is sorted by it).
    let pick = |skip: Option<usize>| {
        (0..plateau.len())
            .flat_map(|c| (0..leads.len()).map(move |l| (c, l)))
            .min_by(|x, y| score(x.0, x.1, skip).total_cmp(&score(y.0, y.1, skip)))
            .unwrap_or((0, 0))
    };
    let (c, l) = pick(None);
    let loo: Vec<f64> = (0..at_impact.len())
        .filter_map(|k| {
            let (c, l) = pick(Some(k));
            errors[c][l][k].map(f64::abs)
        })
        .collect();
    let loo_median = median(loo.clone());
    let (v, a, b, pos) = plateau[c];
    let district_lead = leads[l];
    log(format!(
        "{} sets within {POSITION_TOLERANCE_KM} km of the best position error; chosen v {v:.0} L {a:.1} + {b:.2}·d \
         ({pos:.1} km), district L {district_lead:.1} km: ETA median {:.0} s, leave-one-out {:.0} s",
        plateau.len(),
        score(c, l, None),
        median(loo)
    ));
    t.model.speed_kmh = v;
    t.model.lead_km = a;
    t.model.lead_per_km = b;
    t.model.district_lead_km = district_lead;

    // 3. ETA errors by basis, and σ for each.
    let errs = eta_errors(&t);
    let sigma = |pick: fn(&Basis) -> bool| -> Option<f64> {
        let e: Vec<f64> = errs
            .iter()
            .filter(|(b, _)| pick(b))
            .map(|(_, e)| *e)
            .collect();
        (e.len() >= 3).then(|| robust_sigma(&e).round())
    };
    let fitted = summary(&errs);
    let alert = sigma(|b| *b == Basis::Alert).unwrap_or(model.sigma_alert_s);
    log(format!(
        "ETA n={} median |err| {:.0} s, ≤60 s {} (current model: {:.0} s, {})",
        fitted.2, fitted.0, fitted.1, current.0, current.1
    ));
    Fit {
        speed_kmh: v,
        lead_km: a,
        lead_per_km: b,
        district_lead_km: district_lead,
        sigma_alert_s: alert,
        sigma_waypoint_s: sigma(|b| matches!(b, Basis::Waypoint(..))).unwrap_or(alert),
        sigma_district_s: sigma(|b| *b == Basis::Districts).unwrap_or(model.sigma_district_s),
        eta_median_s: fitted.0,
        eta_loo_median_s: loo_median,
        eta_within_60: fitted.1,
        eta_n: fitted.2,
        current_eta_median_s: current.0,
        current_eta_within_60: current.1,
        transitions: transitions.len(),
        impacts: at_impact.len(),
    }
}

/// A part of Vinnytsia from data/districts.json.
#[derive(serde::Deserialize)]
pub struct District {
    pub name: String,
    pub pattern: String,
    pub lat: f64,
    pub lon: f64,
}

/// Do the districts a post names lie along the drone's course through the city? For every
/// districts post: the incoming course of the track it attaches to against the main axis of
/// the named districts (undirected, 0–90°; random would be 45°), how line-like they are, and
/// which side of the centre they lie on relative to where the drone comes from (0° = the
/// near side, 180° = the far side; random would be 90°).
pub fn corridors(
    places_json: &str,
    model: &Model,
    posts: Vec<Post>,
    districts: &[District],
) -> Vec<String> {
    let patterns: Vec<(Regex, &District)> = districts
        .iter()
        .map(|d| (Regex::new(&d.pattern).expect("district pattern"), d))
        .collect();
    let mut t = Tracker::new(Gazetteer::from_json(places_json), model.clone());
    let mut posts = posts;
    posts.sort_by_key(|p| p.time);
    t.update(posts.clone());
    let centre = t.city.clone();
    // Local plane around the centre, km: x east, y north.
    let xy = |lat: f64, lon: f64| {
        (
            (lon - centre.lon) * 111.32 * centre.lat.to_radians().cos(),
            (lat - centre.lat) * 110.57,
        )
    };
    let mut lines = Vec::new();
    let (mut axis_diffs, mut sides, mut no_track) = (Vec::new(), Vec::new(), 0);
    for p in &posts {
        let text = SIGNATURE.replace(&p.text, "").to_lowercase();
        let named: Vec<&District> = patterns
            .iter()
            .filter(|(re, _)| re.is_match(&text))
            .map(|(_, d)| *d)
            .collect();
        if named.len() < 2 {
            continue;
        }
        let all = t.events(p.time - Duration::seconds(1));
        let before = &all[..];
        let tracks = t.tracks(since_clear(before), p.time);
        let Some(track) = nearest_track(&tracks, &centre).map(|i| &tracks[i]) else {
            no_track += 1;
            continue;
        };
        // The leg into the city: from the last place before it to the city.
        let real: Vec<&Waypoint> = track.points.iter().filter(|w| !w.district).collect();
        let from = match real.as_slice() {
            [.., w] if w.place.name != centre.name => Some(&w.place),
            [.., w, _] => Some(&w.place),
            _ => track.origin.as_ref(),
        };
        let Some(from) = from.filter(|f| km(f, &centre) > 5.0) else {
            no_track += 1;
            continue;
        };
        let course = bearing(from, &centre);
        let pts: Vec<(f64, f64)> = named.iter().map(|d| xy(d.lat, d.lon)).collect();
        let n = pts.len() as f64;
        let (mx, my) = (
            pts.iter().map(|p| p.0).sum::<f64>() / n,
            pts.iter().map(|p| p.1).sum::<f64>() / n,
        );
        let (mut sxx, mut syy, mut sxy) = (0.0, 0.0, 0.0);
        for (x, y) in &pts {
            sxx += (x - mx) * (x - mx);
            syy += (y - my) * (y - my);
            sxy += (x - mx) * (y - my);
        }
        let angle = 0.5 * (2.0 * sxy).atan2(sxx - syy); // from east, counter-clockwise
        let axis = (90.0 - angle.to_degrees()).rem_euclid(180.0);
        let tr = sxx + syy;
        let det = sxx * syy - sxy * sxy;
        let l1 = tr / 2.0 + (tr * tr / 4.0 - det).max(0.0).sqrt();
        let l2 = (tr - l1).max(1e-9);
        let elongation = (l1 / l2).sqrt();
        let d = (axis - course.rem_euclid(180.0)).abs();
        let diff = d.min(180.0 - d);
        let offset = (mx * mx + my * my).sqrt();
        let side_bearing = mx.atan2(my).to_degrees().rem_euclid(360.0);
        let side = angle_between(side_bearing, course + 180.0);
        if elongation >= 1.5 {
            axis_diffs.push(diff);
        }
        if offset >= 2.0 {
            sides.push(side);
        }
        lines.push(format!(
            "{} {:<15} from {:<14} course {:>3.0}° | {:>2} districts axis {:>3.0}° ×{:<4.1} diff {:>2.0}° | centroid {:>4.1} km at {:>3.0}°, {:>3.0}° from the near side  [{}]",
            p.time.with_timezone(&chrono::Local).format("%m-%d %H:%M"),
            p.channel,
            from.name,
            course.rem_euclid(360.0),
            named.len(),
            axis,
            elongation.min(99.0),
            diff,
            offset,
            side_bearing,
            side,
            named.iter().map(|d| d.name.as_str()).collect::<Vec<_>>().join(", "),
        ));
    }
    let share = |v: &[f64], max: f64| v.iter().filter(|x| **x <= max).count();
    lines.push(format!(
        "axis vs course (elongated lists): n={} median {:.0}° (random 45°), ≤30°: {} of {} (random 1/3)",
        axis_diffs.len(),
        median(axis_diffs.clone()),
        share(&axis_diffs, 30.0),
        axis_diffs.len()
    ));
    lines.push(format!(
        "centroid side (≥2 km off centre): n={} median {:.0}° (random 90°), near half (≤90°): {} of {}",
        sides.len(),
        median(sides.clone()),
        share(&sides, 90.0),
        sides.len()
    ));
    lines.push(format!("posts with no incoming track: {no_track}"));
    lines
}

/// One transition: blind minutes, turn °, signed error km, error with the previous leg's speed.
type Row = (f64, f64, f64, Option<f64>);
/// Trains on (features, targets) and predicts one row.
type Predictor<'a> = dyn Fn(&[Vec<f64>], &[f64], &[f64]) -> f64 + 'a;

/// Where the position error comes from: at every track transition ("на X" joins a track
/// with a known course), the dead-reckoned distance to X minus L, split by how long the
/// drone flew blind, how sharply it turned, and against an oracle that knows each drone's
/// own speed from its previous leg.
pub fn positions(places_json: &str, model: &Model, posts: Vec<Post>) -> Vec<String> {
    let mut t = Tracker::new(Gazetteer::from_json(places_json), model.clone());
    t.update(posts.clone());
    // Past routes for flying on past waypoints (same-day drones never count).
    t.routes = t.route_legs();
    let tracks_at = |t: &Tracker, now: DateTime<Utc>| t.tracks(since_clear(&t.events(now)), now);
    // Tracks known by one place only: flown in along past drones' approach vs sitting on it.
    let (mut prior, mut sitting) = (Vec::new(), Vec::new());
    let mut rows: Vec<Row> = Vec::new();
    for p in &posts {
        let before = tracks_at(&t, p.time - Duration::milliseconds(1));
        for tr in tracks_at(&t, p.time) {
            let same = |b: &&Track| {
                b.points[0].first == tr.points[0].first && b.points[0].place == tr.points[0].place
            };
            let Some(b) = before.iter().find(same) else {
                continue;
            };
            if tr.points.len() > b.points.len()
                && b.course().is_none()
                && !tr.head().district
                && t.approach_course(&b.head().place, tr.head().first)
                    .is_some()
            {
                let (place, time) = (&tr.head().place, tr.head().first);
                prior.push(t.estimate(b, place, time).km - t.lead_at(place));
                sitting.push(km(&b.head().place, place) - t.lead_at(place));
            }
            if tr.points.len() <= b.points.len() || b.course().is_none() || tr.head().district {
                continue;
            }
            let (head, place, time) = (b.head(), &tr.head().place, tr.head().first);
            let blind = (time - head.first).num_seconds() as f64 / 60.0;
            let turn = b.turn_to(place);
            let err = t.estimate(b, place, time).km - t.lead_at(place);
            // The previous leg's own speed, when the track has one.
            let own = match b.points.as_slice() {
                [.., prev, last] if !prev.district && !last.district => {
                    let h = (last.first - prev.first).num_seconds() as f64 / 3600.0;
                    let v = km(&prev.place, &last.place) / h;
                    (h > 0.0 && (150.0..=900.0).contains(&v)).then(|| {
                        let mut m = t.model.clone();
                        m.speed_kmh = v;
                        let saved = std::mem::replace(&mut t.model, m);
                        let e = t.estimate(b, place, time).km - t.lead_at(place);
                        t.model = saved;
                        e
                    })
                }
                _ => None,
            };
            if std::env::var_os("POSITIONS_DUMP").is_some() {
                eprintln!("{}\t{}\t{err:.2}", time.to_rfc3339(), place.name);
            }
            rows.push((blind, turn, err, own));
        }
    }
    let abs = |v: Vec<f64>| median(v.iter().map(|x| x.abs()).collect());
    let mut lines = vec![format!(
        "{} transitions: median |err| {:.1} km, median signed {:+.1} km (+ = drone placed short of X)",
        rows.len(),
        abs(rows.iter().map(|r| r.2).collect()),
        median(rows.iter().map(|r| r.2).collect())
    )];
    let group = |lines: &mut Vec<String>, label: &str, pick: &dyn Fn(&Row) -> bool| {
        let g: Vec<f64> = rows.iter().filter(|r| pick(r)).map(|r| r.2).collect();
        lines.push(format!(
            "  {label:<22} n={:>3} median |err| {:>5.1} km, signed {:+5.1} km",
            g.len(),
            abs(g.clone()),
            median(g)
        ));
    };
    lines.push("by minutes since the previous waypoint:".into());
    for (lo, hi) in [(0.0, 2.0), (2.0, 4.0), (4.0, 7.0), (7.0, 12.0), (12.0, 1e9)] {
        group(&mut lines, &format!("{lo:.0}–{hi:.0} min"), &|r| {
            r.0 >= lo && r.0 < hi
        });
    }
    lines.push("by turn at the new waypoint:".into());
    for (lo, hi) in [(0.0, 15.0), (15.0, 40.0), (40.0, 70.0), (70.0, 181.0)] {
        group(&mut lines, &format!("{lo:.0}–{hi:.0}°"), &|r| {
            r.1 >= lo && r.1 < hi
        });
    }
    lines.push(format!(
        "one-place tracks with a past approach (n={}): median |err| {:.1} km flown in along it, {:.1} km sitting on the place",
        prior.len(),
        abs(prior),
        abs(sitting)
    ));
    let both: Vec<&Row> = rows.iter().filter(|r| r.3.is_some()).collect();
    lines.push(format!(
        "own previous-leg speed (n={}): median |err| {:.1} km vs {:.1} km with the common speed",
        both.len(),
        abs(both.iter().filter_map(|r| r.3).collect()),
        abs(both.iter().map(|r| r.2).collect())
    ));
    lines
}

/// Solves (XᵀX + λI)β = Xᵀy; the first column (intercept) is not shrunk.
fn ridge(x: &[Vec<f64>], y: &[f64], lambda: f64) -> Vec<f64> {
    let k = x[0].len();
    let mut a = vec![vec![0.0; k + 1]; k];
    for (row, t) in x.iter().zip(y) {
        for i in 0..k {
            for j in 0..k {
                a[i][j] += row[i] * row[j];
            }
            a[i][k] += row[i] * t;
        }
    }
    for (i, r) in a.iter_mut().enumerate().skip(1) {
        r[i] += lambda;
    }
    for c in 0..k {
        let p = (c..k)
            .max_by(|&i, &j| a[i][c].abs().total_cmp(&a[j][c].abs()))
            .unwrap();
        a.swap(c, p);
        let pivot = a[c][c];
        if pivot.abs() < 1e-12 {
            continue;
        }
        for v in &mut a[c][c..] {
            *v /= pivot;
        }
        let row = a[c].clone();
        for (i, r) in a.iter_mut().enumerate() {
            if i != c {
                let f = r[c];
                for (v, p) in r[c..].iter_mut().zip(&row[c..]) {
                    *v -= f * p;
                }
            }
        }
    }
    a.iter().map(|r| r[k]).collect()
}

/// Can overshoot/undershoot be predicted? At every transition, features known when "на X" is
/// posted → the signed error (dead-reckoned distance to X minus L). Ridge regression and
/// k-nearest neighbours, scored leaving out one attack day at a time, against no correction
/// and a constant shift.
pub fn learn_positions(places_json: &str, model: &Model, posts: Vec<Post>) -> Vec<String> {
    let mut t = Tracker::new(Gazetteer::from_json(places_json), model.clone());
    t.update(posts.clone());
    // Past routes for flying on past waypoints (same-day drones never count).
    t.routes = t.route_legs();
    let city = t.city.clone();
    let tracks_at = |t: &Tracker, now: DateTime<Utc>| t.tracks(since_clear(&t.events(now)), now);
    let names = [
        "1",
        "blind min",
        "prev leg/100km",
        "head–city/100km",
        "course sin",
        "course cos",
        "hour sin",
        "hour cos",
        "drones/5",
        "points",
    ];
    let (mut x, mut y, mut day) = (Vec::new(), Vec::new(), Vec::new());
    for p in &posts {
        let before = tracks_at(&t, p.time - Duration::milliseconds(1));
        let now = tracks_at(&t, p.time);
        for tr in &now {
            let same = |b: &&Track| {
                b.points[0].first == tr.points[0].first && b.points[0].place == tr.points[0].place
            };
            let Some(b) = before.iter().find(same) else {
                continue;
            };
            if tr.points.len() <= b.points.len() || b.course().is_none() || tr.head().district {
                continue;
            }
            let (head, place, time) = (b.head(), &tr.head().place, tr.head().first);
            let err = t.estimate(b, place, time).km - t.lead_at(place);
            let course = b.course().unwrap().to_radians();
            let hour = time.with_timezone(&Local).num_seconds_from_midnight() as f64 / 86400.0
                * std::f64::consts::TAU;
            x.push(vec![
                1.0,
                ((time - head.first).num_seconds() as f64 / 60.0).min(15.0),
                b.tail().map_or(0.0, |t| km(t, &head.place)) / 100.0,
                km(&head.place, &city) / 100.0,
                course.sin(),
                course.cos(),
                hour.sin(),
                hour.cos(),
                now.len() as f64 / 5.0,
                b.points.len() as f64,
            ]);
            // Errors past ±40 km are lost tracks, not over/undershoot: clip so they do not
            // dominate the fit.
            y.push(err.clamp(-40.0, 40.0));
            day.push(time.with_timezone(&Local).date_naive());
        }
    }
    let days: BTreeSet<_> = day.iter().copied().collect();
    let abs_med = |v: Vec<f64>| median(v.iter().map(|e| e.abs()).collect());
    // Held-out residuals of a predictor trained on the other days.
    let held_out = |fit: &Predictor| -> Vec<f64> {
        let mut res = Vec::new();
        for d in &days {
            let train: Vec<usize> = (0..y.len()).filter(|&i| day[i] != *d).collect();
            let tx: Vec<Vec<f64>> = train.iter().map(|&i| x[i].clone()).collect();
            let ty: Vec<f64> = train.iter().map(|&i| y[i]).collect();
            for i in (0..y.len()).filter(|&i| day[i] == *d) {
                res.push(y[i] - fit(&tx, &ty, &x[i]));
            }
        }
        res
    };
    // Standardise features on the training part only, inside each fit.
    let scale = |tx: &[Vec<f64>]| -> Vec<(f64, f64)> {
        (0..tx[0].len())
            .map(|j| {
                if j == 0 {
                    return (0.0, 1.0);
                }
                let col: Vec<f64> = tx.iter().map(|r| r[j]).collect();
                let m = col.iter().sum::<f64>() / col.len() as f64;
                let s = (col.iter().map(|v| (v - m) * (v - m)).sum::<f64>() / col.len() as f64)
                    .sqrt()
                    .max(1e-9);
                (m, s)
            })
            .collect()
    };
    let z = |r: &[f64], sc: &[(f64, f64)]| -> Vec<f64> {
        r.iter().zip(sc).map(|(v, (m, s))| (v - m) / s).collect()
    };
    let mut lines = vec![format!(
        "{} transitions on {} days; features known before the post; target: signed error, km (+ = drone placed short of X)",
        y.len(),
        days.len()
    )];
    lines.push(format!(
        "  no correction        median |err| {:.1} km",
        abs_med(y.clone())
    ));
    let shift = held_out(&|_, ty, _| median(ty.to_vec()));
    lines.push(format!(
        "  constant shift       median |err| {:.1} km",
        abs_med(shift)
    ));
    for lambda in [1.0, 10.0, 100.0] {
        let r = held_out(&|tx, ty, row| {
            let sc = scale(tx);
            let zx: Vec<Vec<f64>> = tx.iter().map(|r| z(r, &sc)).collect();
            let beta = ridge(&zx, ty, lambda);
            z(row, &sc).iter().zip(&beta).map(|(a, b)| a * b).sum()
        });
        lines.push(format!(
            "  ridge λ={lambda:<5}       median |err| {:.1} km",
            abs_med(r)
        ));
    }
    for k in [10, 25, 50] {
        let r = held_out(&|tx, ty, row| {
            let sc = scale(tx);
            let q = z(row, &sc);
            let mut d: Vec<(f64, f64)> = tx
                .iter()
                .zip(ty)
                .map(|(r, t)| {
                    let zr = z(r, &sc);
                    (
                        zr.iter()
                            .zip(&q)
                            .skip(1)
                            .map(|(a, b)| (a - b) * (a - b))
                            .sum(),
                        *t,
                    )
                })
                .collect();
            d.sort_by(|a, b| a.0.total_cmp(&b.0));
            median(d.iter().take(k).map(|p| p.1).collect())
        });
        lines.push(format!(
            "  kNN k={k:<3}            median |err| {:.1} km",
            abs_med(r)
        ));
    }
    // What the full-data ridge leans on (standardised coefficients, km per σ).
    let sc = scale(&x);
    let zx: Vec<Vec<f64>> = x.iter().map(|r| z(r, &sc)).collect();
    let beta = ridge(&zx, &y, 10.0);
    let mut coef: Vec<(&str, f64)> = names.iter().copied().zip(beta).skip(1).collect();
    coef.sort_by(|a, b| b.1.abs().total_cmp(&a.1.abs()));
    lines.push(format!(
        "  ridge coefficients, km per σ: {}",
        coef.iter()
            .map(|(n, b)| format!("{n} {b:+.1}"))
            .collect::<Vec<_>>()
            .join(", ")
    ));
    lines
}

/// Do past tracks predict where a drone turns? For every (previous → current → next) in the
/// history: the course after the current place predicted from other days' drones that came
/// through within 10 km of it on a course within 30° (their mean outgoing course), against
/// flying straight on. Scored on held-out days.
pub fn learn_routes(places_json: &str, model: &Model, posts: Vec<Post>) -> Vec<String> {
    let mut t = Tracker::new(Gazetteer::from_json(places_json), model.clone());
    t.update(posts);
    // (day, current place, incoming course, outgoing course, next place)
    let mut legs = Vec::new();
    for tr in t.past_tracks() {
        let n = tr.path.len() - tr.times.len();
        let pts: Vec<Place> = tr.path[n..]
            .iter()
            .map(|(name, lat, lon)| Place {
                name: name.clone(),
                lat: *lat,
                lon: *lon,
                region: false,
                alt: Vec::new(),
            })
            .collect();
        for w in pts.windows(3) {
            let day = tr.start.with_timezone(&Local).date_naive();
            legs.push((
                day,
                w[1].clone(),
                bearing(&w[0], &w[1]),
                bearing(&w[1], &w[2]),
                w[2].clone(),
            ));
        }
    }
    let mean_course = |cs: &[f64]| {
        let (s, c) = cs.iter().fold((0.0, 0.0), |(s, c), b| {
            (s + b.to_radians().sin(), c + b.to_radians().cos())
        });
        s.atan2(c).to_degrees().rem_euclid(360.0)
    };
    let (mut straight, mut learned, mut learned_hits, mut straight_hits) = (vec![], vec![], 0, 0);
    let mut covered = 0;
    let (mut cov_straight, mut cov_learned, mut cov_s_hits, mut cov_l_hits) =
        (vec![], vec![], 0, 0);
    for (day, cur, inc, out, next) in &legs {
        let similar: Vec<&(chrono::NaiveDate, Place, f64, f64, Place)> = legs
            .iter()
            .filter(|l| l.0 != *day && km(&l.1, cur) <= 10.0 && angle_between(l.2, *inc) <= 30.0)
            .collect();
        let leg = km(cur, next);
        let lands = |course: f64| {
            let (lat, lon) = destination(cur.lat, cur.lon, course, leg);
            haversine_km(lat, lon, next.lat, next.lon) <= 15.0
        };
        straight.push(angle_between(*inc, *out));
        straight_hits += usize::from(lands(*inc));
        if similar.len() >= 3 {
            covered += 1;
            let c = mean_course(&similar.iter().map(|l| l.3).collect::<Vec<_>>());
            learned.push(angle_between(c, *out));
            learned_hits += usize::from(lands(c));
            cov_learned.push(angle_between(c, *out));
            cov_l_hits += usize::from(lands(c));
            cov_straight.push(angle_between(*inc, *out));
            cov_s_hits += usize::from(lands(*inc));
        } else {
            learned.push(angle_between(*inc, *out));
            learned_hits += usize::from(lands(*inc));
        }
    }
    vec![
        format!(
            "{} turns on {} days; {} with ≥3 similar drones on other days",
            legs.len(),
            legs.iter().map(|l| l.0).collect::<BTreeSet<_>>().len(),
            covered
        ),
        format!(
            "  straight on:   median course error {:.0}°, next place within 15 km {} of {}",
            median(straight.clone()),
            straight_hits,
            legs.len()
        ),
        format!(
            "  past routes:   median course error {:.0}°, next place within 15 km {} of {}",
            median(learned.clone()),
            learned_hits,
            legs.len()
        ),
        format!(
            "  where past routes exist ({covered}): straight {:.0}°, {} hits; past routes {:.0}°, {} hits",
            median(cov_straight),
            cov_s_hits,
            median(cov_learned),
            cov_l_hits
        ),
    ]
}

/// Where a new waypoint moved the drone forward: at "на X" it is put L short of X, though by
/// the dead reckoning of its previous leg it was still farther. Every such jump over 3 km.
pub fn jumps(places_json: &str, model: &Model, posts: Vec<Post>) -> Vec<String> {
    let mut t = Tracker::new(Gazetteer::from_json(places_json), model.clone());
    t.update(posts.clone());
    t.routes = t.route_legs();
    let tracks_at = |t: &Tracker, now: DateTime<Utc>| t.tracks(since_clear(&t.events(now)), now);
    let mut lines = Vec::new();
    let (mut all, mut city) = (Vec::new(), Vec::new());
    for p in &posts {
        let before = tracks_at(&t, p.time - Duration::milliseconds(1));
        for tr in tracks_at(&t, p.time) {
            let same = |b: &&Track| {
                b.points[0].first == tr.points[0].first && b.points[0].place == tr.points[0].place
            };
            let Some(b) = before.iter().find(same) else {
                continue;
            };
            if tr.points.len() <= b.points.len() || b.course().is_none() {
                continue;
            }
            let (head, time) = (tr.head(), tr.head().first);
            let was = t.estimate(b, &head.place, time).km;
            let put = t.head_lead_km(&tr);
            let jump = was - put;
            all.push(jump);
            if head.place.name == CITY {
                city.push(jump);
            }
            if jump > 3.0 {
                lines.push(format!(
                    "{} {} → {}{}: was {was:.0} km from it, put at {put:.0} km: jumped {jump:.0} km ({:.1} min at {:.0} km/h)",
                    time.with_timezone(&Local).format("%m-%d %H:%M"),
                    b.head().place.name,
                    head.place.name,
                    if head.district { " (districts)" } else { "" },
                    jump / t.speed(head) * 60.0,
                    t.speed(head),
                ));
            }
        }
    }
    let over = |v: &[f64], km: f64| v.iter().filter(|j| **j > km).count();
    lines.push(format!(
        "{} new waypoints: forward jumps >3 km {}, >10 km {}; median {:.1} km",
        all.len(),
        over(&all, 3.0),
        over(&all, 10.0),
        median(all.clone())
    ));
    lines.push(format!(
        "to the city ({}): >3 km {}, >10 km {}; median {:.1} km",
        city.len(),
        over(&city, 3.0),
        over(&city, 10.0),
        median(city.clone())
    ));
    lines
}

/// City alerts out of nowhere: a post sends a drone to the city while no drone with a place
/// on its way was tracked. Each one, and whether the city was hit within 20 min after.
pub fn vacuum_alerts(
    places_json: &str,
    model: &Model,
    posts: Vec<Post>,
    impacts: &[(DateTime<Utc>, String)],
) -> Vec<String> {
    let mut t = Tracker::new(Gazetteer::from_json(places_json), model.clone());
    t.update(posts.clone());
    t.routes = t.route_legs();
    let city = t.city.clone();
    let hits: Vec<DateTime<Utc>> = impacts
        .iter()
        .filter(|(_, p)| p == CITY)
        .map(|(at, _)| *at)
        .collect();
    let (mut lines, mut vacuum, mut tracked) = (Vec::new(), (0, 0), (0, 0));
    for p in &posts {
        let events = t.classify(p);
        let to_city = events.iter().any(|e| {
            matches!(&e.signal, Signal::Drone { route, district: false, .. } if heads_to(route, &city))
        });
        if !to_city {
            continue;
        }
        let before = t.tracks(
            since_clear(&t.events(p.time - Duration::milliseconds(1))),
            p.time,
        );
        // Only the first post of an approach: the city had no alert in the last 20 min.
        let earlier = t.events(p.time - Duration::milliseconds(1));
        if earlier.iter().any(|e| {
            e.time > p.time - Duration::minutes(20)
                && matches!(&e.signal, Signal::Drone { route, .. } if heads_to(route, &city))
        }) {
            continue;
        }
        let hit = hits
            .iter()
            .find(|h| **h > p.time && **h - p.time <= Duration::minutes(20))
            .map(|h| (*h - p.time).num_seconds());
        let slot = if before.is_empty() {
            &mut vacuum
        } else {
            &mut tracked
        };
        slot.0 += 1;
        slot.1 += usize::from(hit.is_some());
        lines.push(format!(
            "{} {:<8} {} | {}",
            p.time.with_timezone(&Local).format("%m-%d %H:%M"),
            if before.is_empty() {
                "VACUUM"
            } else {
                "tracked"
            },
            hit.map_or("no hit within 20 min".into(), |s| format!(
                "hit after {s} s"
            )),
            p.text
                .replace('\n', " ")
                .chars()
                .take(80)
                .collect::<String>()
        ));
    }
    lines.push(format!(
        "first city alerts out of nowhere: {} (followed by a city hit: {}); with a drone tracked: {} (hit: {})",
        vacuum.0, vacuum.1, tracked.0, tracked.1
    ));
    lines
}

/// Can ML learn how far to trust the last post? At each impact with a threat, from the last
/// post before it: the physics ETA (posts taken as positions), the dead-reckoning ETA (posts
/// only turn the drone), how many posts came in the 5 min before (a panic runs ahead of the
/// drone), minutes since the post before, and whether districts were named. Every learner
/// that makes sense on a few dozen rows, on the time left and on the correction to physics,
/// scored leaving out one day at a time against physics alone.
pub fn learn_eta(
    places_json: &str,
    model: &Model,
    posts: Vec<Post>,
    impacts: &[(DateTime<Utc>, String)],
) -> Vec<String> {
    let build = |city_turns: bool, kalman: bool| {
        let mut m = model.clone();
        (m.city_turns, m.district_turns) = (city_turns, city_turns);
        if kalman {
            // The best noise on the history (2026-10-25 grid): Kalman as a third expert.
            (m.kalman, m.kf_accel, m.kf_along_km, m.kf_cross_km) = (true, 3e5, 25.0, 5.0);
            (m.kf_district_km, m.kf_velocity_kmh) = (8.0, 200.0);
        }
        let mut t = Tracker::new(Gazetteer::from_json(places_json), m);
        t.update(posts.clone());
        t.routes = t.route_legs();
        t
    };
    let (phys, dr, kal) = (build(false, false), build(true, false), build(false, true));
    let gz = Gazetteer::from_json(places_json);
    let start = posts.iter().map(|p| p.time).min().unwrap_or_default();
    let mut times: Vec<DateTime<Utc>> = posts.iter().map(|p| p.time).collect();
    times.sort();
    // Features: physics ETA, reckoning ETA (s after the last post), posts in 5 min, minutes
    // since the post before, districts named.
    let mut kalman_left = Vec::new();
    let (mut x, mut y, mut day) = (Vec::new(), Vec::new(), Vec::new());
    for (ex, place) in resolve(&gz, impacts, start) {
        let now = ex - Duration::seconds(1);
        let (Status::Threat(a), Status::Threat(b), Status::Threat(c)) = (
            phys.status_for(&place, now),
            dr.status_for(&place, now),
            kal.status_for(&place, now),
        ) else {
            continue;
        };
        let i = times.partition_point(|t| *t <= now);
        if i == 0 {
            continue;
        }
        let last = times[i - 1];
        let gap = if i >= 2 {
            (last - times[i - 2]).num_seconds() as f64 / 60.0
        } else {
            30.0
        };
        let rate = times[..i]
            .iter()
            .filter(|t| **t >= last - Duration::minutes(5))
            .count();
        let left = |eta: DateTime<Utc>| (eta - last).num_seconds() as f64;
        x.push(vec![
            left(a.eta),
            left(b.eta),
            rate as f64,
            gap.min(30.0),
            f64::from(u8::from(a.basis == Basis::Districts)),
        ]);
        kalman_left.push(left(c.eta));
        y.push((ex - last).num_seconds() as f64);
        day.push(ex.with_timezone(&Local).date_naive());
    }
    let days: BTreeSet<_> = day.iter().copied().collect();
    // Held-out residuals: train on all other days, predict this one. `correction`: the learner
    // predicts y - physics and physics is added back.
    let held_out = |learner: &Learner, correction: bool| -> Vec<f64> {
        let mut res = Vec::new();
        for d in &days {
            let train: Vec<usize> = (0..y.len()).filter(|&i| day[i] != *d).collect();
            let tx: Vec<Vec<f64>> = train.iter().map(|&i| x[i].clone()).collect();
            let ty: Vec<f64> = train
                .iter()
                .map(|&i| if correction { y[i] - x[i][0] } else { y[i] })
                .collect();
            let predict = learner(&tx, &ty);
            for i in (0..y.len()).filter(|&i| day[i] == *d) {
                let base = if correction { x[i][0] } else { 0.0 };
                res.push(y[i] - base - predict(&x[i]));
            }
        }
        res
    };
    let score = |res: &[f64]| {
        let abs: Vec<f64> = res.iter().map(|e| e.abs()).collect();
        let late = res.iter().filter(|e| **e < -60.0).count();
        (
            median(abs.clone()),
            abs.iter().filter(|e| **e <= 60.0).count(),
            late,
        )
    };
    let mut lines = vec![format!(
        "{} impacts on {} days; target: seconds from the last post to the impact; held out by day",
        y.len(),
        days.len()
    )];
    lines.push(format!(
        "  {:<40} {:<36}|{}",
        "",
        "within 10, 20, … 200 s:",
        (1..=20)
            .map(|k| format!("{:>3}", 10 * k))
            .collect::<String>()
    ));
    // How many impacts the ETA got within 10, 20, … 200 s, and the mean share over those
    // thresholds (the area under that curve; 1 = all within 10 s).
    let curve = |res: &[f64]| -> (Vec<usize>, f64) {
        let counts: Vec<usize> = (1..=20)
            .map(|k| res.iter().filter(|e| e.abs() <= 10.0 * k as f64).count())
            .collect();
        let area = counts.iter().sum::<usize>() as f64 / (20.0 * res.len() as f64);
        (counts, area)
    };
    let row = |name: &str, res: &[f64]| {
        let (m, _, late) = score(res);
        let (counts, area) = curve(res);
        format!(
            "  {name:<40} median {m:>4.0} s late>1min {late:>2} area {area:.3} |{}",
            counts.iter().map(|c| format!("{c:>3}")).collect::<String>()
        )
    };
    let phys_err: Vec<f64> = (0..y.len()).map(|i| y[i] - x[i][0]).collect();
    let dr_err: Vec<f64> = (0..y.len()).map(|i| y[i] - x[i][1]).collect();
    lines.push(row("physics (no learning)", &phys_err));
    lines.push(row("dead reckoning (no learning)", &dr_err));
    let kal_err: Vec<f64> = (0..y.len()).map(|i| y[i] - kalman_left[i]).collect();
    lines.push(row("Kalman (no learning)", &kal_err));
    // Several physics, one per case: the best possible pick, then simple rules for the pick
    // ("many posts in 5 min: a panic, trust reckoning"), the threshold set on the other days.
    let experts: Vec<Vec<f64>> = (0..y.len())
        .map(|i| vec![x[i][0], x[i][1], kalman_left[i]])
        .collect();
    let oracle: Vec<f64> = (0..y.len())
        .map(|i| {
            experts[i]
                .iter()
                .map(|e| y[i] - e)
                .min_by(|a, b| a.abs().total_cmp(&b.abs()))
                .unwrap()
        })
        .collect();
    lines.push(row("best physics per case (oracle, not usable)", &oracle));
    // Weights for all three, non-negative and summing to one (two free numbers), chosen on
    // the other days by median error in steps of 0.05.
    let mut res = Vec::new();
    let mut chosen = Vec::new();
    for d in &days {
        let train: Vec<usize> = (0..y.len()).filter(|&i| day[i] != *d).collect();
        let mut best = (f64::INFINITY, [1.0, 0.0, 0.0]);
        for a in 0..=20 {
            for b in 0..=(20 - a) {
                let w = [a as f64 / 20.0, b as f64 / 20.0, (20 - a - b) as f64 / 20.0];
                let errs: Vec<f64> = train
                    .iter()
                    .map(|&i| (y[i] - (0..3).map(|k| w[k] * experts[i][k]).sum::<f64>()).abs())
                    .collect();
                let m = median(errs);
                if m < best.0 {
                    best = (m, w);
                }
            }
        }
        chosen.push(best.1);
        for i in (0..y.len()).filter(|&i| day[i] == *d) {
            res.push(y[i] - (0..3).map(|k| best.1[k] * experts[i][k]).sum::<f64>());
        }
    }
    lines.push(row("weighted physics + reckoning + Kalman", &res));
    let mean_w: Vec<f64> = (0..3)
        .map(|k| chosen.iter().map(|w| w[k]).sum::<f64>() / chosen.len() as f64)
        .collect();
    lines.push(format!(
        "    weights chosen (mean over days): physics {:.2}, reckoning {:.2}, Kalman {:.2}",
        mean_w[0], mean_w[1], mean_w[2]
    ));
    let names = ["physics", "reckoning", "Kalman"];
    for (feature, fname) in [
        (2, "posts in 5 min"),
        (3, "minutes since previous"),
        (4, "districts"),
    ] {
        for (alt, aname) in [(1, names[1]), (2, names[2])] {
            let mut res = Vec::new();
            for d in &days {
                let train: Vec<usize> = (0..y.len()).filter(|&i| day[i] != *d).collect();
                let mut cuts: Vec<f64> = train.iter().map(|&i| x[i][feature]).collect();
                cuts.sort_by(f64::total_cmp);
                cuts.dedup();
                // "feature >= cut (or < cut when `below`): the alternative, else physics".
                let mut best = (f64::INFINITY, f64::INFINITY, false);
                for &cut in &cuts {
                    for below in [false, true] {
                        let pick = |i: usize| (x[i][feature] >= cut) != below;
                        let errs: Vec<f64> = train
                            .iter()
                            .map(|&i| {
                                (y[i]
                                    - if pick(i) {
                                        experts[i][alt]
                                    } else {
                                        experts[i][0]
                                    })
                                .abs()
                            })
                            .collect();
                        let m = median(errs);
                        if m < best.0 {
                            best = (m, cut, below);
                        }
                    }
                }
                for i in (0..y.len()).filter(|&i| day[i] == *d) {
                    let pick = (x[i][feature] >= best.1) != best.2;
                    res.push(y[i] - if pick { experts[i][alt] } else { experts[i][0] });
                }
            }
            lines.push(row(&format!("pick {aname} by {fname}"), &res));
        }
    }
    let learners: Vec<(&str, Box<Learner>)> = vec![
        (
            "linear (least squares)",
            Box::new(|tx, ty| linear(tx, ty, 1e-6)),
        ),
        ("ridge λ=1", Box::new(|tx, ty| linear(tx, ty, 1.0))),
        ("ridge λ=10", Box::new(|tx, ty| linear(tx, ty, 10.0))),
        (
            "least absolute deviations",
            Box::new(|tx, ty| robust(tx, ty, 0.0)),
        ),
        ("Huber (δ = 60 s)", Box::new(|tx, ty| robust(tx, ty, 60.0))),
        (
            "k nearest neighbours k=3",
            Box::new(|tx, ty| knn(tx, ty, 3)),
        ),
        (
            "k nearest neighbours k=5",
            Box::new(|tx, ty| knn(tx, ty, 5)),
        ),
        (
            "k nearest neighbours k=8",
            Box::new(|tx, ty| knn(tx, ty, 8)),
        ),
        (
            "decision tree depth 2",
            Box::new(|tx, ty| forest(tx, ty, 1, 2, 3, false, false)),
        ),
        (
            "decision tree depth 3",
            Box::new(|tx, ty| forest(tx, ty, 1, 3, 3, false, false)),
        ),
        (
            "random forest depth 3",
            Box::new(|tx, ty| forest(tx, ty, 300, 3, 3, true, false)),
        ),
        (
            "extra trees depth 3",
            Box::new(|tx, ty| forest(tx, ty, 300, 3, 3, false, true)),
        ),
        (
            "gradient boosting depth 1",
            Box::new(|tx, ty| boosting(tx, ty, 1)),
        ),
        (
            "gradient boosting depth 2",
            Box::new(|tx, ty| boosting(tx, ty, 2)),
        ),
        (
            "kernel ridge (RBF, γ=0.1)",
            Box::new(|tx, ty| kernel_ridge(tx, ty, 0.1, 1.0)),
        ),
        (
            "kernel ridge (RBF, γ=0.5)",
            Box::new(|tx, ty| kernel_ridge(tx, ty, 0.5, 1.0)),
        ),
        (
            "neural net (1 hidden layer, 8 tanh)",
            Box::new(|tx, ty| mlp(tx, ty, 8)),
        ),
    ];
    for correction in [false, true] {
        lines.push(if correction {
            "learning the correction to physics:".into()
        } else {
            "learning the time left:".into()
        });
        for (name, learner) in &learners {
            lines.push(row(name, &held_out(learner.as_ref(), correction)));
        }
    }
    lines
}

/// A trained model: the prediction for a row.
type Predict = Box<dyn Fn(&[f64]) -> f64>;
/// Trains on rows and targets.
type Learner = dyn Fn(&[Vec<f64>], &[f64]) -> Predict;

/// Column means and standard deviations of the training rows.
fn standardiser(x: &[Vec<f64>]) -> impl Fn(&[f64]) -> Vec<f64> + Clone + 'static {
    let k = x[0].len();
    let stats: Vec<(f64, f64)> = (0..k)
        .map(|j| {
            let n = x.len() as f64;
            let m = x.iter().map(|r| r[j]).sum::<f64>() / n;
            let s = (x.iter().map(|r| (r[j] - m).powi(2)).sum::<f64>() / n)
                .sqrt()
                .max(1e-9);
            (m, s)
        })
        .collect();
    move |r: &[f64]| {
        r.iter()
            .zip(&stats)
            .map(|(v, (m, s))| (v - m) / s)
            .collect()
    }
}

/// Least squares on standardised features with an intercept; `lambda` > 0 is ridge.
fn linear(x: &[Vec<f64>], y: &[f64], lambda: f64) -> Predict {
    let z = standardiser(x);
    let rows: Vec<Vec<f64>> = x.iter().map(|r| [vec![1.0], z(r)].concat()).collect();
    let beta = ridge(&rows, y, lambda);
    Box::new(move |r| {
        [vec![1.0], z(r)]
            .concat()
            .iter()
            .zip(&beta)
            .map(|(a, b)| a * b)
            .sum()
    })
}

/// Iteratively reweighted least squares: `delta` = 0 for least absolute deviations, else Huber.
fn robust(x: &[Vec<f64>], y: &[f64], delta: f64) -> Predict {
    let z = standardiser(x);
    let rows: Vec<Vec<f64>> = x.iter().map(|r| [vec![1.0], z(r)].concat()).collect();
    let mut beta = ridge(&rows, y, 1e-6);
    for _ in 0..50 {
        let w: Vec<f64> = rows
            .iter()
            .zip(y)
            .map(|(r, t)| {
                let e = (t - r.iter().zip(&beta).map(|(a, b)| a * b).sum::<f64>()).abs();
                if delta > 0.0 && e <= delta {
                    1.0
                } else {
                    (delta.max(1.0)) / e.max(1.0)
                }
            })
            .collect();
        let wr: Vec<Vec<f64>> = rows
            .iter()
            .zip(&w)
            .map(|(r, w)| r.iter().map(|v| v * w.sqrt()).collect())
            .collect();
        let wy: Vec<f64> = y.iter().zip(&w).map(|(t, w)| t * w.sqrt()).collect();
        beta = ridge(&wr, &wy, 1e-6);
    }
    Box::new(move |r| {
        [vec![1.0], z(r)]
            .concat()
            .iter()
            .zip(&beta)
            .map(|(a, b)| a * b)
            .sum()
    })
}

/// The median target of the k nearest training rows (standardised distance).
fn knn(x: &[Vec<f64>], y: &[f64], k: usize) -> Predict {
    let z = standardiser(x);
    let train: Vec<(Vec<f64>, f64)> = x.iter().map(|r| z(r)).zip(y.iter().copied()).collect();
    Box::new(move |r| {
        let q = z(r);
        let mut d: Vec<(f64, f64)> = train
            .iter()
            .map(|(p, t)| (p.iter().zip(&q).map(|(a, b)| (a - b).powi(2)).sum(), *t))
            .collect();
        d.sort_by(|a, b| a.0.total_cmp(&b.0));
        median(d.iter().take(k).map(|p| p.1).collect())
    })
}

/// Trees: one plain tree (`trees` = 1, no bagging), a random forest (`bagging`), or extra trees
/// (`random_cuts`: one random threshold per tried feature, no bootstrap).
fn forest(
    x: &[Vec<f64>],
    y: &[f64],
    trees: usize,
    depth: usize,
    min_leaf: usize,
    bagging: bool,
    random_cuts: bool,
) -> Predict {
    let f = Forest::fit(x, y, trees, depth, min_leaf, bagging, random_cuts);
    Box::new(move |r| f.predict(r))
}

/// Gradient boosting of shallow trees on squared error: 150 rounds, learning rate 0.05.
fn boosting(x: &[Vec<f64>], y: &[f64], depth: usize) -> Predict {
    let base = y.iter().sum::<f64>() / y.len() as f64;
    let mut pred = vec![base; y.len()];
    let mut stages = Vec::new();
    for _ in 0..150 {
        let resid: Vec<f64> = y.iter().zip(&pred).map(|(t, p)| t - p).collect();
        let tree = Forest::fit(x, &resid, 1, depth, 3, false, false);
        for (p, r) in pred.iter_mut().zip(x) {
            *p += 0.05 * tree.predict(r);
        }
        stages.push(tree);
    }
    Box::new(move |r| base + stages.iter().map(|t| 0.05 * t.predict(r)).sum::<f64>())
}

/// Kernel ridge regression with an RBF kernel on standardised features (a Gaussian process
/// mean with noise `lambda`), around the training mean.
fn kernel_ridge(x: &[Vec<f64>], y: &[f64], gamma: f64, lambda: f64) -> Predict {
    let z = standardiser(x);
    let pts: Vec<Vec<f64>> = x.iter().map(|r| z(r)).collect();
    let mean = y.iter().sum::<f64>() / y.len() as f64;
    let kern = move |a: &[f64], b: &[f64]| {
        (-gamma * a.iter().zip(b).map(|(p, q)| (p - q).powi(2)).sum::<f64>()).exp()
    };
    let n = pts.len();
    // Solve (K + λI) α = y - mean via the ridge solver on the identity design: K is SPD.
    let mut a: Vec<Vec<f64>> = (0..n)
        .map(|i| {
            let mut row: Vec<f64> = (0..n).map(|j| kern(&pts[i], &pts[j])).collect();
            row[i] += lambda;
            row.push(y[i] - mean);
            row
        })
        .collect();
    for c in 0..n {
        let p = (c..n)
            .max_by(|&i, &j| a[i][c].abs().total_cmp(&a[j][c].abs()))
            .unwrap();
        a.swap(c, p);
        let pivot = a[c][c];
        for v in &mut a[c][c..] {
            *v /= pivot;
        }
        let row = a[c].clone();
        for (i, r) in a.iter_mut().enumerate() {
            if i != c {
                let f = r[c];
                for (v, p) in r[c..].iter_mut().zip(&row[c..]) {
                    *v -= f * p;
                }
            }
        }
    }
    let alpha: Vec<f64> = a.iter().map(|r| r[n]).collect();
    Box::new(move |r| {
        let q = z(r);
        mean + pts
            .iter()
            .zip(&alpha)
            .map(|(p, a)| a * kern(p, &q))
            .sum::<f64>()
    })
}

/// A one-hidden-layer tanh network on standardised features and target, full-batch gradient
/// descent with weight decay; deterministic initialisation.
fn mlp(x: &[Vec<f64>], y: &[f64], hidden: usize) -> Predict {
    let z = standardiser(x);
    let xs: Vec<Vec<f64>> = x.iter().map(|r| z(r)).collect();
    let (ym, ys) = {
        let n = y.len() as f64;
        let m = y.iter().sum::<f64>() / n;
        (
            m,
            (y.iter().map(|t| (t - m).powi(2)).sum::<f64>() / n)
                .sqrt()
                .max(1e-9),
        )
    };
    let ts: Vec<f64> = y.iter().map(|t| (t - ym) / ys).collect();
    let k = xs[0].len();
    let mut seed = 0x2545_f491_4f6c_dd1du64;
    let mut rand = || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed % 2000) as f64 / 1000.0 - 1.0
    };
    let mut w1: Vec<Vec<f64>> = (0..hidden)
        .map(|_| (0..=k).map(|_| rand() * 0.5).collect())
        .collect();
    let mut w2: Vec<f64> = (0..=hidden).map(|_| rand() * 0.5).collect();
    let (rate, decay) = (0.05, 1e-3);
    for _ in 0..3000 {
        let mut g1 = vec![vec![0.0; k + 1]; hidden];
        let mut g2 = vec![0.0; hidden + 1];
        for (r, t) in xs.iter().zip(&ts) {
            let h: Vec<f64> = w1
                .iter()
                .map(|w| (w[k] + w[..k].iter().zip(r).map(|(a, b)| a * b).sum::<f64>()).tanh())
                .collect();
            let out = w2[hidden] + w2[..hidden].iter().zip(&h).map(|(a, b)| a * b).sum::<f64>();
            let e = out - t;
            g2[hidden] += e;
            for j in 0..hidden {
                g2[j] += e * h[j];
                let back = e * w2[j] * (1.0 - h[j] * h[j]);
                g1[j][k] += back;
                for (g, v) in g1[j][..k].iter_mut().zip(r) {
                    *g += back * v;
                }
            }
        }
        let n = xs.len() as f64;
        for j in 0..=hidden {
            w2[j] -= rate * (g2[j] / n + decay * w2[j]);
        }
        for (w, g) in w1.iter_mut().zip(&g1) {
            for (v, d) in w.iter_mut().zip(g) {
                *v -= rate * (d / n + decay * *v);
            }
        }
    }
    Box::new(move |r| {
        let q = z(r);
        let h: Vec<f64> = w1
            .iter()
            .map(|w| (w[k] + w[..k].iter().zip(&q).map(|(a, b)| a * b).sum::<f64>()).tanh())
            .collect();
        ym + ys * (w2[hidden] + w2[..hidden].iter().zip(&h).map(|(a, b)| a * b).sum::<f64>())
    })
}

/// Regression trees on squared error: one plain tree, a random forest (bootstrap samples, a
/// random third of the features at each split) or extra trees (one random cut per tried
/// feature). Deterministic (a fixed xorshift seed).
struct Forest {
    trees: Vec<Node>,
}

enum Node {
    Leaf(f64),
    Split {
        feature: usize,
        at: f64,
        left: Box<Node>,
        right: Box<Node>,
    },
}

/// How trees are grown.
struct Grow {
    depth: usize,
    min_leaf: usize,
    /// Features tried at each split.
    tried: usize,
    random_cuts: bool,
}

impl Forest {
    fn fit(
        x: &[Vec<f64>],
        y: &[f64],
        trees: usize,
        depth: usize,
        min_leaf: usize,
        bagging: bool,
        random_cuts: bool,
    ) -> Self {
        let mut seed = 0x9e37_79b9_7f4a_7c15u64;
        let mut rand = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            seed
        };
        let k = x[0].len();
        let how = Grow {
            depth,
            min_leaf,
            tried: if trees == 1 { k } else { k.div_ceil(3) },
            random_cuts,
        };
        let trees = (0..trees)
            .map(|_| {
                let idx: Vec<usize> = if bagging {
                    (0..y.len()).map(|_| rand() as usize % y.len()).collect()
                } else {
                    (0..y.len()).collect()
                };
                Self::grow(x, y, &idx, how.depth, &how, &mut rand)
            })
            .collect();
        Self { trees }
    }

    fn grow(
        x: &[Vec<f64>],
        y: &[f64],
        idx: &[usize],
        depth: usize,
        how: &Grow,
        rand: &mut dyn FnMut() -> u64,
    ) -> Node {
        let mean = idx.iter().map(|&i| y[i]).sum::<f64>() / idx.len() as f64;
        if depth == 0 || idx.len() < 2 * how.min_leaf {
            return Node::Leaf(mean);
        }
        let sse = |s: &[usize]| {
            let m = s.iter().map(|&i| y[i]).sum::<f64>() / s.len() as f64;
            s.iter().map(|&i| (y[i] - m).powi(2)).sum::<f64>()
        };
        let k = x[0].len();
        let mut best: Option<(f64, usize, f64)> = None;
        for t in 0..how.tried {
            // A plain tree tries every feature in turn; the others pick at random.
            let f = if how.tried == k {
                t
            } else {
                rand() as usize % k
            };
            let mut vals: Vec<f64> = idx.iter().map(|&i| x[i][f]).collect();
            vals.sort_by(f64::total_cmp);
            vals.dedup();
            if vals.len() < 2 {
                continue;
            }
            let cuts: Vec<f64> = if how.random_cuts {
                let u = (rand() % 1_000_000) as f64 / 1_000_000.0;
                vec![vals[0] + u * (vals[vals.len() - 1] - vals[0])]
            } else {
                vals.windows(2).map(|w| (w[0] + w[1]) / 2.0).collect()
            };
            for at in cuts {
                let (l, r): (Vec<usize>, Vec<usize>) = idx.iter().partition(|&&i| x[i][f] <= at);
                if l.len() < how.min_leaf || r.len() < how.min_leaf {
                    continue;
                }
                let cost = sse(&l) + sse(&r);
                if best.is_none_or(|b| cost < b.0) {
                    best = Some((cost, f, at));
                }
            }
        }
        let Some((_, feature, at)) = best else {
            return Node::Leaf(mean);
        };
        let (l, r): (Vec<usize>, Vec<usize>) = idx.iter().partition(|&&i| x[i][feature] <= at);
        Node::Split {
            feature,
            at,
            left: Box::new(Self::grow(x, y, &l, depth - 1, how, rand)),
            right: Box::new(Self::grow(x, y, &r, depth - 1, how, rand)),
        }
    }

    fn predict(&self, row: &[f64]) -> f64 {
        let one = |mut n: &Node| loop {
            match n {
                Node::Leaf(v) => return *v,
                Node::Split {
                    feature,
                    at,
                    left,
                    right,
                } => {
                    n = if row[*feature] <= *at { left } else { right };
                }
            }
        };
        self.trees.iter().map(one).sum::<f64>() / self.trees.len() as f64
    }
}
