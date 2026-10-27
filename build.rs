//! The seed of the channel's history built into the app: the maintainer's scraped posts
//! (data/messages.jsonl, kept out of git) since PPO_SINCE, gzipped, so a released app fetches
//! only what came after it was built. Without that file (anyone else's build) the seed is
//! empty and the app fetches the whole history on first launch.

use std::io::Write;

/// The same as `pipeline::PPO_SINCE`.
const SINCE: &str = "2026-10-05";

fn main() {
    println!("cargo:rerun-if-changed=data/messages.jsonl");
    let posts = std::fs::read_to_string("data/messages.jsonl").unwrap_or_default();
    let kept: String = posts
        .lines()
        .filter(|l| {
            l.split_once("\"date\":\"")
                .is_some_and(|(_, rest)| rest.get(..SINCE.len()).is_some_and(|d| d >= SINCE))
        })
        .flat_map(|l| [l, "\n"])
        .collect();
    let mut gz = flate2::write::GzEncoder::new(Vec::new(), flate2::Compression::best());
    gz.write_all(kept.as_bytes()).expect("gzip");
    let out = std::path::Path::new(&std::env::var("OUT_DIR").expect("OUT_DIR")).join("seed_history.gz");
    std::fs::write(out, gz.finish().expect("gzip")).expect("write the seed");
}
