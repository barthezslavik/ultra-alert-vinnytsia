//! Finds Vinnytsia-oblast place names in post text, in any grammatical case.
//!
//! Built from data/places.json (produced by `ultra-alert geocode`): every spelling seen in the
//! channel maps to a canonical name with coordinates.

use serde::Deserialize;
use std::collections::HashMap;

pub const CITY: &str = "Вінниця";

#[derive(Debug, Clone, PartialEq)]
pub struct Place {
    pub name: String,
    pub lat: f64,
    pub lon: f64,
    /// A neighbouring oblast ("з Одещини"): only tells where a drone comes from.
    pub region: bool,
    /// Other villages in the oblast with the same name, (lat, lon): the tracker picks the one
    /// that fits the drone's path.
    pub alt: Vec<(f64, f64)>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Role {
    /// "на X", "курсом на X": the drone is heading there.
    Ahead,
    /// "через X": the drone will pass X before the other places in the post.
    Via,
    /// "з X", "від X": the drone has already passed X.
    Behind,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Mention {
    pub place: Place,
    pub role: Role,
}

#[derive(Deserialize)]
struct PlacesFile {
    alias: HashMap<String, String>,
    coords: HashMap<String, Option<Coord>>,
}

#[derive(Deserialize)]
struct Coord {
    lat: f64,
    lon: f64,
    #[serde(default)]
    region: bool,
    #[serde(default)]
    alt: Option<Vec<[f64; 2]>>,
}

#[derive(Clone)]
pub struct Gazetteer {
    patterns: Vec<(Vec<Stem>, Place)>,
}

/// One word of a place name: matches a word that starts with one of `bases` followed by at
/// most 3 more letters (the case ending).
#[derive(Debug, Clone, PartialEq)]
struct Stem {
    bases: Vec<String>,
}

impl Stem {
    fn matches(&self, word: &str) -> bool {
        self.bases.iter().any(|b| {
            word.strip_prefix(b.as_str())
                .is_some_and(|rest| rest.chars().count() <= 3)
        })
    }
}

impl Gazetteer {
    pub fn from_json(json: &str) -> Self {
        let file: PlacesFile = serde_json::from_str(json).expect("places.json");
        let mut spellings: HashMap<String, String> = file.alias;
        for name in file.coords.keys() {
            spellings
                .entry(name.clone())
                .or_insert_with(|| name.clone());
        }
        let mut seen = std::collections::HashSet::new();
        let mut patterns = Vec::new();
        for (spelling, canonical) in spellings {
            let Some(Some(c)) = file.coords.get(&canonical) else {
                continue;
            };
            let Some(pattern) = stems(&spelling) else {
                continue;
            };
            if !seen.insert(format!("{pattern:?}")) {
                continue;
            }
            let place = Place {
                name: canonical,
                lat: c.lat,
                lon: c.lon,
                region: c.region,
                alt: c
                    .alt
                    .iter()
                    .flatten()
                    .map(|[lat, lon]| (*lat, *lon))
                    .collect(),
            };
            patterns.push((pattern, place));
        }
        // Multi-word names first so "Немирівське шосе" wins over "Немирів".
        patterns.sort_by_key(|(words, _)| std::cmp::Reverse(words.len()));
        Self { patterns }
    }

    /// A place found while the app runs (`lookup`), under the spelling a post used and its own
    /// name.
    pub fn add(&mut self, spelling: &str, place: Place) {
        for s in [spelling, place.name.as_str()] {
            if let Some(pattern) = stems(s)
                && !self.patterns.iter().any(|(p, _)| *p == pattern)
            {
                self.patterns.push((pattern, place.clone()));
            }
        }
        self.patterns
            .sort_by_key(|(words, _)| std::cmp::Reverse(words.len()));
    }

    /// Names in a post that look like places but are not known: capitalised words after
    /// "на", "повз", "через", "біля", "до" or in a "/" list ("Агрономічне / Юзвин, обережно").
    pub fn unknown_names(&self, text: &str) -> Vec<String> {
        // The signature is not part of the post ("💪 Телеграм канал ППО Вінниця").
        let text = ["💪", "👉", "Телеграм канал"]
            .iter()
            .filter_map(|s| text.find(s))
            .min()
            .map_or(text, |i| &text[..i]);
        let mut out: Vec<String> = Vec::new();
        let separators = |c: char| "/,;\n!.:()▪".contains(c) || (!c.is_alphanumeric() && !" -'ʼ’".contains(c));
        let mut rest = text;
        while !rest.is_empty() {
            let end = rest.find(separators).unwrap_or(rest.len());
            let segment = &rest[..end];
            let before = text[..text.len() - rest.len()].trim_end().ends_with('/');
            rest = &rest[end..];
            let sep = rest.chars().next();
            rest = sep.map_or(rest, |c| &rest[c.len_utf8()..]);
            let listed = before || sep == Some('/');
            let words: Vec<&str> = segment.split_whitespace().collect();
            let mut i = 0;
            while i < words.len() {
                let capital = |w: &str| {
                    w.chars().next().is_some_and(char::is_uppercase)
                        && w.chars().filter(|c| c.is_alphabetic()).count() >= 3
                        && !NOT_NAMES.contains(&w.to_lowercase().as_str())
                };
                if !capital(words[i]) {
                    i += 1;
                    continue;
                }
                let start = i;
                while i < words.len() && i - start < 3 && capital(words[i]) {
                    i += 1;
                }
                let after = start
                    .checked_sub(1)
                    .is_some_and(|k| PREPOSITIONS.contains(&words[k].to_lowercase().as_str()));
                let name = words[start..i].join(" ");
                if (after || (listed && start == 0 && i == words.len()))
                    && self.mentions(&name).is_empty()
                    && !out.contains(&name)
                {
                    out.push(name);
                }
            }
        }
        out
    }

    /// The place by its canonical name, else by any spelling ("Рахни-Лісові" and "Рахни Лісові"
    /// are one place, whichever spelling became its name).
    pub fn place(&self, name: &str) -> Option<Place> {
        self.patterns
            .iter()
            .map(|(_, p)| p)
            .find(|p| p.name == name)
            .cloned()
            .or_else(|| self.mentions(name).into_iter().next().map(|m| m.place))
    }

    /// Places in the order they appear, with "через X" moved ahead of the rest and
    /// consecutive repeats of the same place collapsed.
    pub fn mentions(&self, text: &str) -> Vec<Mention> {
        // Words split like place names are ("Рахни-Лісові" is two words); emoji and their
        // invisible marks (U+FE0F in "❗️ВІННИЦЯ") are separators.
        let lower = text.to_lowercase();
        let words: Vec<&str> = lower
            .split(|c: char| !(c.is_alphanumeric() || matches!(c, '\'' | 'ʼ' | '’')))
            .filter(|w| !w.is_empty())
            .collect();
        let mut taken = vec![false; words.len()];
        let mut found: Vec<(usize, &Place)> = Vec::new();
        for (stems, place) in &self.patterns {
            for start in 0..words.len().saturating_sub(stems.len() - 1) {
                let span = start..start + stems.len();
                if taken[span.clone()].iter().any(|t| *t)
                    || !stems
                        .iter()
                        .zip(&words[span.clone()])
                        .all(|(s, w)| s.matches(w))
                {
                    continue;
                }
                // "Вендичани Могилів-Подільського району": the district, not the town, nor
                // "Могилів" in it.
                let district = words
                    .get(span.end)
                    .is_some_and(|w| w.starts_with("район") || w.starts_with("громад"));
                taken[span].fill(true);
                if district {
                    continue;
                }
                found.push((start, place));
            }
        }
        found.sort_by_key(|&(start, _)| start);
        let mut out: Vec<Mention> = found
            .into_iter()
            .map(|(start, place)| {
                let prev = start.checked_sub(1).map_or("", |i| words[i]);
                let role = match prev {
                    "через" | "повз" => Role::Via,
                    "з" | "із" | "від" => Role::Behind,
                    _ => Role::Ahead,
                };
                Mention {
                    place: place.clone(),
                    role,
                }
            })
            .collect();
        out.dedup_by(|a, b| a.place.name == b.place.name);
        out.sort_by_key(|m| match m.role {
            Role::Behind => 0,
            Role::Via => 1,
            Role::Ahead => 2,
        });
        out
    }
}

/// Words before a place name in a post.
const PREPOSITIONS: &[&str] = &["на", "повз", "через", "біля", "до", "над", "у", "в"];
/// Capitalised words that are not places.
const NOT_NAMES: &[&str] = &[
    "реактивний", "реактивних", "реактивні", "реактивного", "реактив", "шахед", "шахеди",
    "шахедів", "шахеда", "керований", "керованих", "новий", "нові", "перший", "другий",
    "третій", "звичайний", "звичайні", "ударний", "обстановка", "увага", "уважно", "укриття",
    "укриттях", "обережно", "вінниччина", "вінниччину", "вінниччині", "область", "області",
    "район", "районі", "місто", "місті", "чисто", "тривога", "відбій", "вибух", "будьте",
    "займіть", "негайно", "ппо", "бпла", "село", "селі", "села", "селище", "селищі", "селища", "смт",
];

/// Stems matching a Ukrainian place name in any case: "Вінниця" → "вінниц" + ≤3 letters, so
/// "Вінницю"/"Вінниці" match but "Вінницький" and "Вінниччина" do not; "Немирів" also
/// matches "Немирова" (і/о alternation).
fn stems(name: &str) -> Option<Vec<Stem>> {
    let lower = name.to_lowercase();
    if lower.chars().count() < 3 {
        return None;
    }
    let words: Vec<Stem> = lower
        .split([' ', '-'])
        .filter(|w| !w.is_empty())
        .map(|w| {
            let chars: Vec<char> = w.chars().collect();
            let n = chars.len();
            let head = |k: usize| chars[..n - k].iter().collect::<String>();
            let bases = if w.ends_with("ів") && n > 4 {
                // "Немирова", "Дашева", "Бердичева".
                vec![
                    format!("{}ів", head(2)),
                    format!("{}ов", head(2)),
                    format!("{}ев", head(2)),
                ]
            } else if w.ends_with("іль") && n > 5 {
                // "Ямполя", "Томашполя".
                vec![head(1), format!("{}ол", head(3))]
            } else if (w.ends_with("ий") || w.ends_with("ій")) && n > 5 {
                // "Могилів-Подільського", "Подільському".
                vec![head(2)]
            } else if n > 4 && "аяеєиіїоуюьй".contains(chars[n - 1]) {
                vec![head(1)]
            } else {
                vec![w.to_owned()]
            };
            Stem { bases }
        })
        .collect();
    (!words.is_empty()).then_some(words)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn gz() -> Gazetteer {
        Gazetteer::from_json(include_str!("../data/places.json"))
    }

    #[test]
    fn genitive_and_typos() {
        assert_eq!(names("Шахед поблизу Дашева"), ["Дашів"]);
        assert_eq!(names("в напрямку Ямполя"), ["Ямпіль"]);
        assert_eq!(names("з Бердичева"), ["Бердичів"]);
        assert_eq!(names("в напрямку Могилів-Подільського"), ["Могилів-Подільський"]);
        assert_eq!(names("на Сустиски / Тиврів"), ["Сутиски", "Тиврів"]);
        assert_eq!(names("на Вендичани / Мог. Подільський"), ["Вендичани", "Могилів-Подільський"]);
    }

    #[test]
    fn district_of_a_town_is_not_the_town() {
        assert_eq!(
            names("на селище Вендичани Могилів-Подільського району"),
            ["Вендичани"]
        );
    }

    fn names(text: &str) -> Vec<String> {
        gz().mentions(text)
            .into_iter()
            .map(|m| m.place.name)
            .collect()
    }

    #[test]
    fn matches_cases_but_not_adjectives() {
        assert_eq!(names("🛵 Реактивний Шахед на ВІННИЦЮ"), ["Вінниця"]);
        assert_eq!(names("Вибух у Вінниці"), ["Вінниця"]);
        assert!(names("Тривога по всій Вінницькій області").is_empty());
        assert!(names("📡 Чисто на Вінниччині").is_empty());
        assert_eq!(names("Реактивний Шахед на Немирів"), ["Немирів"]);
        assert_eq!(names("з Немирова"), ["Немирів"]);
    }

    #[test]
    fn unknown_names_in_lists_and_after_prepositions() {
        let g = gz();
        assert_eq!(
            g.unknown_names("❗️Агрономічне / Небувалівка / Бохоники, обережно\n\n💪 Телеграм канал ППО Вінниця"),
            ["Небувалівка"]
        );
        assert_eq!(g.unknown_names("🛵 1 Реактивний Шахед на Небувалку"), ["Небувалку"]);
        assert_eq!(
            g.unknown_names("Керований Реактивний через / повз вас курсом на Небувалівку"),
            ["Небувалівку"]
        );
        assert!(g.unknown_names("🛵 1 Реактивний Шахед на ВІННИЦЮ").is_empty());
        assert!(g.unknown_names("📡 Чисто на Вінниччині🫡").is_empty());
    }

    #[test]
    fn added_place_is_found() {
        let mut g = gz();
        let place = Place {
            name: "Небувалівка".into(),
            lat: 49.0,
            lon: 28.0,
            region: false,
            alt: Vec::new(),
        };
        g.add("Небувалівку", place);
        assert_eq!(names_in(&g, "Шахед на Небувалівку"), ["Небувалівка"]);
        assert_eq!(names_in(&g, "повз Небувалівку"), ["Небувалівка"]);
    }

    fn names_in(g: &Gazetteer, text: &str) -> Vec<String> {
        g.mentions(text).into_iter().map(|m| m.place.name).collect()
    }

    #[test]
    fn emoji_before_name() {
        assert_eq!(names("❗️ВІННИЦЯ, займіть укриття"), ["Вінниця"]);
    }

    #[test]
    fn city_districts_map_to_city() {
        assert_eq!(
            names("Немирівське шосе / Лука Мелешківська / Тяжилів в укриття!"),
            ["Вінниця"]
        );
    }

    #[test]
    fn route_order_follows_roles() {
        assert_eq!(
            names("2 Реактивні Шахеди на Вінницю через Липовець"),
            ["Липовець", "Вінниця"]
        );
        let m = gz().mentions("Реактивний Шахед з Бердичева на Хмільник");
        assert_eq!(m.last().unwrap().place.name, "Хмільник");
        assert_eq!(m.last().unwrap().role, Role::Ahead);
        assert!(
            m.iter()
                .all(|x| x.place.name != "Бердичів" || x.role == Role::Behind)
        );
        assert_eq!(
            names("Полетів далі на ВОРОНОВИЦЮ курсом на ВІННИЦЮ"),
            ["Вороновиця", "Вінниця"]
        );
    }
}
