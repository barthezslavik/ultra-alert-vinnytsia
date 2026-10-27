//! Turns channel posts into structured events for the offline pipeline.
//!
//! Template posts are parsed with regexes, posts without threat words are "other", and the
//! rest go to `codex exec` in large batches (one request per BATCH unique texts), cached in
//! data/codex_cache.json so a text is never sent twice.

use super::{Channel, history};
use anyhow::{Context, Result, bail};
use regex::Regex;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashMap};
use std::io::Write;
use std::process::{Command, Stdio};
use std::sync::LazyLock;

pub const CACHE: &str = "data/codex_cache.json";
const BATCH: usize = 120;

/// One event in a post. Place names are Ukrainian, nominative case ("Вінниця", "Козятин").
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Event {
    pub kind: String,
    pub count: Option<i64>,
    pub jet: Option<bool>,
    pub target: Option<String>,
    #[serde(default)]
    pub via: Vec<String>,
    pub from_place: Option<String>,
    #[serde(default)]
    pub leaving: bool,
    #[serde(default)]
    pub times: Vec<NoteTime>,
}

/// "15:39 - вибух" written into a post: local time and what happened then.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NoteTime {
    pub hhmm: String,
    pub kind: String,
}

/// A labelled post as stored in data/events*.jsonl.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Labelled {
    pub id: u64,
    pub date: String,
    pub text: String,
    pub events: Vec<Event>,
}

impl Event {
    fn of(kind: &str) -> Self {
        Self {
            kind: kind.into(),
            ..Self::default()
        }
    }
}

pub fn load(ch: &Channel) -> Result<Vec<Labelled>> {
    super::read(ch.events)?
        .lines()
        .filter(|l| !l.trim().is_empty())
        .map(|l| Ok(serde_json::from_str(l)?))
        .collect()
}

static FOOTER: LazyLock<Regex> = LazyLock::new(|| {
    Regex::new(r"(?s)(💪\s*(Телеграм канал|Підписатись на) ППО Вінниця|👉\s*Термінові новини Вінниці\s*👈|Подробиці .* Інстаграм).*").unwrap()
});
/// vinnytsiarealll prefixes every post with a mood marker like "🔴 😗🙂😞😟".
static MOOD: LazyLock<Regex> = LazyLock::new(|| {
    Regex::new(r"^(?:[🔴💥]\s*)?(?:[\x{1F600}-\x{1F64F}\x{1F910}-\x{1F97F}\x{1FAE0}-\x{1FAFF}☹☺\x{FE0F}]\s*){3,}").unwrap()
});
static SPACE: LazyLock<Regex> = LazyLock::new(|| Regex::new(r"\s+").unwrap());
/// Posts without any of these words are news or ads.
static THREAT: LazyLock<Regex> = LazyLock::new(|| {
    Regex::new(r"(?i)шахед|бпла|дрон|мопед|реактив|вибух|гупа|гуркіт|ракет|калібр|балісти|кинджал|укритт|чисто|відбій|тривог|збит|знищ|курс|летить|летять|підліта|залітає|вилітає|атаку|приліт|влучан|ППО|сил[аи] ППО|почу[вл]|гучн|💥").unwrap()
});
static CLEAR: LazyLock<Regex> =
    LazyLock::new(|| Regex::new(r"^📡 (Чисто|Без Шахедів) на Вінниччині ?🫡$").unwrap());
static RECON: LazyLock<Regex> =
    LazyLock::new(|| Regex::new(r"^📡 Дорозвідка на Вінниччині ?🫡?$").unwrap());
static BALLISTIC: LazyLock<Regex> =
    LazyLock::new(|| Regex::new(r"^❗️\s*Загроза балістики на Вінниччині.*$").unwrap());
static EXPLOSION: LazyLock<Regex> = LazyLock::new(|| {
    Regex::new(r"^💥?\s*(Знову\s+)?(ВИБУХ|Вибух|Вибухи)\s*(у|в)?\s*(ВІННИЦІ|Вінниці)?\s*[!❗️💥]*$")
        .unwrap()
});
static TRACK: LazyLock<Regex> = LazyLock::new(|| {
    Regex::new(concat!(
        r"^🛵\s*(?:(?P<n>\d+|\+\d+|Один|Два|Три|Чотири|Новий|Нові|Ще один|Перший|Другий|Третій|Останній)\s+(?:нов\w+\s+)?)?",
        r"(?P<jet>Реактивн\w*|Ударн\w*|звичайн\w*)?\s*Шахед\w*\s+(?:летить\s+|летять\s+)?на\s+(?:селище\s+|село\s+|місто\s+)?",
        r"(?P<to>[А-ЯІЇЄҐ][\w'ʼ\-]+(?:[ -][А-ЯІЇЄҐ][\w'ʼ\-]+)?)(?:\s+\w+ського району)?!?$"
    ))
    .unwrap()
});
static SHELTER: LazyLock<Regex> = LazyLock::new(|| {
    Regex::new(r"(?i)^❗️\s*(?P<to>[А-ЯІЇЄҐ][А-ЯІЇЄҐA-Z'ʼ\-]+(?:-[А-ЯІЇЄҐ][\w'ʼ]+)?),\s*(?:займіть\s+укриття|в\s+укриття)").unwrap()
});

pub fn norm(text: &str) -> String {
    let text = FOOTER.replace_all(text, "");
    let text = SPACE.replace_all(&text, " ");
    MOOD.replace(text.trim(), "").trim().to_owned()
}

/// "ВІННИЦЮ" → "Вінницю"; mixed case stays as written.
fn capitalize_if_upper(s: &str) -> String {
    let all_upper = s.chars().any(char::is_alphabetic)
        && s.chars()
            .filter(|c| c.is_alphabetic())
            .all(char::is_uppercase);
    if !all_upper {
        return s.to_owned();
    }
    capitalize(s)
}

fn capitalize(s: &str) -> String {
    let mut chars = s.chars();
    chars.next().map_or_else(String::new, |first| {
        first
            .to_uppercase()
            .chain(chars.flat_map(char::to_lowercase))
            .collect()
    })
}

fn regex_label(t: &str) -> Option<Event> {
    if CLEAR.is_match(t) {
        return Some(Event::of("clear"));
    }
    if RECON.is_match(t) {
        return Some(Event::of("recon"));
    }
    if BALLISTIC.is_match(t) {
        return Some(Event::of("ballistic"));
    }
    if EXPLOSION.is_match(t) {
        return Some(Event {
            target: Some("Вінниця".into()),
            ..Event::of("explosion")
        });
    }
    if let Some(m) = TRACK.captures(t) {
        let n = m.name("n").map_or("1", |n| n.as_str()).to_lowercase();
        let n = n.trim_start_matches('+');
        let count = n.parse().unwrap_or(match n {
            "два" => 2,
            "три" => 3,
            "чотири" => 4,
            _ => 1,
        });
        let jet = m
            .name("jet")
            .is_some_and(|j| j.as_str().to_lowercase().starts_with("реактив"));
        return Some(Event {
            count: Some(count),
            jet: Some(jet),
            target: Some(capitalize_if_upper(&m["to"])),
            ..Event::of("track")
        });
    }
    if let Some(m) = SHELTER.captures(t) {
        return Some(Event {
            target: Some(capitalize(&m["to"])),
            jet: t.contains("еактив").then_some(true),
            ..Event::of("shelter")
        });
    }
    None
}

const PROMPT: &str = r#"Ти розмічаєш пости вінницьких телеграм-каналів про повітряні загрози.
Пости, що не стосуються повітряних загроз (новини, реклама, світло/вода), - kind "other", без деталей.
Події поза Вінницькою областю (інші області, Київ) - kind "other".
"Вибух на Вінниччині" без міста: explosion з target "Вінниччина"; "у Жмеринському районі": target "Жмеринка".
"Гупає"/"звуки, схожі на вибухи" у/біля міста: explosion для цього міста.
Для КОЖНОГО поста поверни список подій (events). Один пост може містити кілька подій
(наприклад «Обстановка: ▪️1 реактивний на Жмеринку; ▪️2 звичайні на Немирів» = 2 події track).

kind:
- track: шахед(и) летять на/в напрямку/біля населеного пункту або району
- shelter: заклик конкретному місту/району міста в укриття
- explosion: вибух(и) у місті; target = місто (якщо не вказано і пост про Вінницю - "Вінниця"); якщо вибух через збиття - shootdown
- shootdown: збито/знищено
- clear: чисто / без шахедів в області; all_clear: відбій тривоги
- recon: дорозвідка; ballistic: загроза балістики; missile: ракети / крилаті ракети
- other: донати, новини, загальні попередження, решта

Поля:
- count: кількість дронів (null якщо невідомо, "+1"/"новий"/"ще один" = 1)
- jet: true для реактивних шахедів, false для звичайних/ударних, null якщо не сказано
- target: населений пункт, на який/до якого летить або де подія, у НАЗИВНОМУ відмінку
  українською ("на Вінницю" -> "Вінниця", "на Калинівку" -> "Калинівка", "Козятина" -> "Козятин").
  Якщо вказаний лише район (напр. "Гайсинський район") - назва районного центру ("Гайсин").
  Для частин міста Вінниці (Тяжилів, Лука Мелешківська, Старе місто, ГПЗ, Хутори, Немирівське шосе) target = "Вінниця", а ці назви поклади у via.
- via: проміжні пункти маршруту в порядку польоту, у називному відмінку.
  Якщо цілі перелічені як альтернативи через "/" або "-" ("на Погребище / Оратів"), target = перша, решта у via.
- from_place: звідки летить, якщо вказано (називний відмінок; для областей - назва області, напр. "Хмельниччина")
- leaving: true якщо дрон вилітає з Вінницької області
- times: часи в тексті у форматі HH:MM з типом події: explosion, shootdown, leaving, all_clear, lost, other

Поверни JSON з items: [{i, events}], по одному item на кожен вхідний пост, i - номер поста.
"#;

fn schema() -> serde_json::Value {
    let nullable = |t: &str| serde_json::json!({ "type": [t, "null"] });
    serde_json::json!({
        "type": "object", "additionalProperties": false, "required": ["items"],
        "properties": {"items": {"type": "array", "items": {
            "type": "object", "additionalProperties": false, "required": ["i", "events"],
            "properties": {"i": {"type": "integer"}, "events": {"type": "array", "items": {
                "type": "object", "additionalProperties": false,
                "required": ["kind", "count", "jet", "target", "via", "from_place", "leaving", "times"],
                "properties": {
                    "kind": {"type": "string", "enum": ["track", "shelter", "explosion", "shootdown",
                             "clear", "recon", "ballistic", "missile", "all_clear", "other"]},
                    "count": nullable("integer"),
                    "jet": nullable("boolean"),
                    "target": nullable("string"),
                    "via": {"type": "array", "items": {"type": "string"}},
                    "from_place": nullable("string"),
                    "leaving": {"type": "boolean"},
                    "times": {"type": "array", "items": {
                        "type": "object", "additionalProperties": false, "required": ["hhmm", "kind"],
                        "properties": {"hhmm": {"type": "string"}, "kind": {"type": "string"}}}},
                }}}}}}},
    })
}

/// One `codex exec` request for a whole batch of (index, text).
fn codex(batch: &[(usize, String)], tag: usize) -> Result<HashMap<usize, Vec<Event>>> {
    let dir = std::env::temp_dir().join(format!("ultra-alert-codex-{}-{tag}", std::process::id()));
    std::fs::create_dir_all(&dir)?;
    let (schema_path, out_path) = (dir.join("schema.json"), dir.join("out.json"));
    std::fs::write(&schema_path, schema().to_string())?;
    let body: Vec<String> = batch.iter().map(|(i, t)| format!("[{i}] {t}")).collect();
    let mut child = Command::new("codex")
        .args([
            "exec",
            "--ephemeral",
            "--skip-git-repo-check",
            "-s",
            "read-only",
            "-C",
        ])
        .arg(&dir)
        .arg("--output-schema")
        .arg(&schema_path)
        .arg("-o")
        .arg(&out_path)
        .arg("-")
        .stdin(Stdio::piped())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .context("run codex (is the Codex CLI installed and logged in?)")?;
    child
        .stdin
        .take()
        .context("codex stdin")?
        .write_all(format!("{PROMPT}\nПости:\n{}", body.join("\n")).as_bytes())?;
    if !child.wait()?.success() {
        bail!("codex exec failed");
    }
    #[derive(Deserialize)]
    struct Out {
        items: Vec<Item>,
    }
    #[derive(Deserialize)]
    struct Item {
        i: usize,
        events: Vec<Event>,
    }
    let out: Out = serde_json::from_str(&std::fs::read_to_string(&out_path)?)?;
    let _ = std::fs::remove_dir_all(&dir);
    Ok(out.items.into_iter().map(|it| (it.i, it.events)).collect())
}

/// Labels a channel's posts since `ch.since` into `ch.events`; only unseen texts reach codex.
pub fn label(ch: &Channel, dry: bool) -> Result<()> {
    let posts = history::load(ch, ch.since)?;
    let mut cache: BTreeMap<String, Vec<Event>> = match std::fs::read_to_string(CACHE) {
        Ok(text) => serde_json::from_str(&text)?,
        Err(_) => BTreeMap::new(),
    };
    let mut labels: HashMap<String, Vec<Event>> = HashMap::new();
    let mut todo: Vec<String> = Vec::new();
    for p in &posts {
        let t = norm(&p.text);
        if labels.contains_key(&t) || todo.contains(&t) {
            continue;
        }
        if let Some(cached) = cache.get(&t) {
            labels.insert(t, cached.clone());
        } else if let Some(ev) = regex_label(&t) {
            labels.insert(t, vec![ev]);
        } else if !THREAT.is_match(&t) {
            labels.insert(t, vec![Event::of("other")]);
        } else {
            todo.push(t);
        }
    }
    eprintln!(
        "{}: {} posts, {} need codex -> {} requests",
        ch.name,
        posts.len(),
        todo.len(),
        todo.len().div_ceil(BATCH)
    );
    if dry {
        todo.iter()
            .take(40)
            .for_each(|t| eprintln!("   {}", t.chars().take(140).collect::<String>()));
        return Ok(());
    }
    let chunks: Vec<Vec<(usize, String)>> = todo
        .chunks(BATCH)
        .map(|c| c.iter().cloned().enumerate().collect())
        .collect();
    let results: Vec<Result<HashMap<usize, Vec<Event>>>> = std::thread::scope(|s| {
        let handles: Vec<_> = chunks
            .iter()
            .enumerate()
            .map(|(k, chunk)| s.spawn(move || codex(chunk, k)))
            .collect();
        handles
            .into_iter()
            .map(|h| h.join().expect("codex thread"))
            .collect()
    });
    for (chunk, got) in chunks.iter().zip(results) {
        let got = got?;
        eprintln!("codex batch: {}/{}", got.len(), chunk.len());
        for (i, t) in chunk {
            if let Some(events) = got.get(i) {
                cache.insert(t.clone(), events.clone());
                labels.insert(t.clone(), events.clone());
            }
        }
    }
    std::fs::write(CACHE, serde_json::to_string_pretty(&cache)?)?;
    let mut out = String::new();
    for p in &posts {
        let text = norm(&p.text);
        let events = labels.get(&text).cloned().unwrap_or_default();
        let rec = Labelled {
            id: p.id,
            date: p.time.to_rfc3339(),
            text,
            events,
        };
        out += &serde_json::to_string(&rec)?;
        out.push('\n');
    }
    std::fs::write(ch.events, out).with_context(|| format!("write {}", ch.events))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn templates() {
        let t = regex_label("🛵 1 Реактивний Шахед на ВІННИЦЮ").unwrap();
        assert_eq!(
            (t.kind.as_str(), t.count, t.jet),
            ("track", Some(1), Some(true))
        );
        assert_eq!(t.target.as_deref(), Some("Вінницю"));
        let s =
            regex_label("❗️КОЗЯТИН, займіть укриття Реактивний Шахед в напрямку міста.").unwrap();
        assert_eq!(
            (s.kind.as_str(), s.target.as_deref(), s.jet),
            ("shelter", Some("Козятин"), Some(true))
        );
        assert_eq!(
            regex_label("📡 Чисто на Вінниччині 🫡").unwrap().kind,
            "clear"
        );
        assert_eq!(regex_label("💥Вибух у ВІННИЦІ").unwrap().kind, "explosion");
        assert!(regex_label("Друзі, дякуємо за донати").is_none());
    }

    #[test]
    fn norm_strips_signature_and_mood() {
        assert_eq!(
            norm("🛵 Шахед на Ладижин\n\n💪 Телеграм канал ППО Вінниця"),
            "🛵 Шахед на Ладижин"
        );
        assert_eq!(
            norm("🔴 😗🙂😞😟 РЕАКТИВНИЙ ШАХЕД ЛЕТИТЬ НА ВІННИЦЮ 👉 Термінові новини Вінниці 👈"),
            "РЕАКТИВНИЙ ШАХЕД ЛЕТИТЬ НА ВІННИЦЮ"
        );
    }
}
