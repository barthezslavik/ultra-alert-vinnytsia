//! Reads the public web preview of a Telegram channel (t.me/s/<channel>), no account needed.

use chrono::{DateTime, Utc};
use regex::Regex;
use std::sync::LazyLock;
use std::time::Duration;

#[derive(Debug, Clone, PartialEq)]
pub struct Post {
    pub channel: String,
    pub id: u64,
    pub time: DateTime<Utc>,
    pub text: String,
}

static POST_ID: LazyLock<Regex> =
    LazyLock::new(|| Regex::new(r#"data-post="[^/"]+/(\d+)""#).unwrap());
static TIME: LazyLock<Regex> = LazyLock::new(|| Regex::new(r#"<time datetime="([^"]+)""#).unwrap());
/// The post's own text, not the quote of the post it replies to (`js-message_reply_text`).
static TEXT: LazyLock<Regex> = LazyLock::new(|| {
    Regex::new(r#"(?s)tgme_widget_message_text js-message_text[^>]*>(.*?)</div>"#).unwrap()
});
static BR: LazyLock<Regex> = LazyLock::new(|| Regex::new(r"<br\s*/?>").unwrap());
static TAG: LazyLock<Regex> = LazyLock::new(|| Regex::new(r"<[^>]+>").unwrap());
static ENTITY: LazyLock<Regex> =
    LazyLock::new(|| Regex::new(r"&(#x[0-9a-fA-F]+|#\d+|[a-z]+);").unwrap());

/// The newest page of posts.
pub async fn fetch(client: &reqwest::Client, channel: &str) -> Result<Vec<Post>, reqwest::Error> {
    fetch_page(client, channel, None).await
}

/// One page of about 20 posts, those older than `before` when given.
pub async fn fetch_page(
    client: &reqwest::Client,
    channel: &str,
    before: Option<u64>,
) -> Result<Vec<Post>, reqwest::Error> {
    let url = match before {
        Some(id) => format!("https://t.me/s/{channel}?before={id}"),
        None => format!("https://t.me/s/{channel}"),
    };
    let html = client
        .get(url)
        .header("User-Agent", "Mozilla/5.0")
        .timeout(Duration::from_secs(10))
        .send()
        .await?
        .error_for_status()?
        .text()
        .await?;
    Ok(parse(&html, channel))
}

pub fn parse(html: &str, channel: &str) -> Vec<Post> {
    html.split("class=\"tgme_widget_message_wrap")
        .skip(1)
        .filter_map(|block| {
            let id = POST_ID.captures(block)?[1].parse().ok()?;
            let time = DateTime::parse_from_rfc3339(&TIME.captures(block)?[1])
                .ok()?
                .with_timezone(&Utc);
            let text = TEXT
                .captures(block)
                .map(|c| {
                    unescape(&TAG.replace_all(&BR.replace_all(&c[1], "\n"), ""))
                        .trim()
                        .to_owned()
                })
                .unwrap_or_default();
            Some(Post {
                channel: channel.to_owned(),
                id,
                time,
                text,
            })
        })
        .collect()
}

fn unescape(s: &str) -> String {
    ENTITY
        .replace_all(s, |c: &regex::Captures| {
            let e = &c[1];
            let code = if let Some(hex) = e.strip_prefix("#x") {
                u32::from_str_radix(hex, 16).ok()
            } else if let Some(dec) = e.strip_prefix('#') {
                dec.parse().ok()
            } else {
                match e {
                    "amp" => Some('&' as u32),
                    "lt" => Some('<' as u32),
                    "gt" => Some('>' as u32),
                    "quot" => Some('"' as u32),
                    "apos" => Some('\'' as u32),
                    "nbsp" => Some(' ' as u32),
                    _ => None,
                }
            };
            code.and_then(char::from_u32)
                .map_or_else(|| c[0].to_owned(), String::from)
        })
        .into_owned()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_preview_block() {
        let html = r#"<div class="tgme_widget_message_wrap js-widget_message_wrap"><div class="tgme_widget_message" data-post="vinnytsiappo/1501">
            <div class="tgme_widget_message_text js-message_text" dir="auto">💥Вибух у ВІННИЦІ<br/><br/>&quot;x&quot; &#39;y&#39;</div>
            <a class="tgme_widget_message_date"><time datetime="2026-10-22T15:53:33+00:00" class="time">18:53</time></a></div></div>"#;
        let posts = parse(html, "vinnytsiappo");
        assert_eq!(posts.len(), 1);
        assert_eq!(posts[0].id, 1501);
        assert_eq!(posts[0].text, "💥Вибух у ВІННИЦІ\n\n\"x\" 'y'");
        assert_eq!(posts[0].time.to_rfc3339(), "2026-10-22T15:53:33+00:00");
    }

    /// A reply carries the quoted post first; the text is the reply's own.
    #[test]
    fn reply_text_is_its_own() {
        let html = r#"<div class="tgme_widget_message_wrap js-widget_message_wrap"><div class="tgme_widget_message" data-post="vinnytsiappo/1758">
            <a class="tgme_widget_message_reply"><div class="tgme_widget_message_text js-message_reply_text" dir="auto">🛵 1 Реактивний Шахед на Піщанку</div></a>
            <div class="tgme_widget_message_text js-message_text" dir="auto">🛵 1 Реактивний Шахед на Клембівку</div>
            <a class="tgme_widget_message_date"><time datetime="2026-10-25T12:26:38+00:00" class="time">15:26</time></a></div></div>"#;
        assert_eq!(
            parse(html, "vinnytsiappo")[0].text,
            "🛵 1 Реактивний Шахед на Клембівку"
        );
    }
}
