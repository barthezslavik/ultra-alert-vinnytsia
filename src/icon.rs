//! Menu-bar icon: a solid rounded pill with the text cut out, so the menu-bar colour shows through.

use fontdue::{Font, FontSettings};
use std::sync::LazyLock;
use tray_icon::Icon;

/// SF Rounded where macOS has it, else the next of these it has: read when the app starts,
/// not built in, so the app builds and runs on any Mac and macOS version.
const FONTS: &[&str] = &[
    "/System/Library/Fonts/SFNSRounded.ttf",
    "/System/Library/Fonts/SFNS.ttf",
    "/System/Library/Fonts/SFNSText.ttf",
    "/System/Library/Fonts/Supplemental/Arial.ttf",
    "/Library/Fonts/Arial.ttf",
    "/System/Library/Fonts/Helvetica.ttc",
];

static FONT: LazyLock<Font> = LazyLock::new(|| {
    FONTS
        .iter()
        .filter_map(|path| std::fs::read(path).ok())
        .find_map(|bytes| Font::from_bytes(bytes, FontSettings::default()).ok())
        .expect("a system font for the menu-bar icon")
});

const HEIGHT: usize = 18;
const FONT_SIZE: f32 = 13.0;

pub fn render(text: &str, visible: bool) -> Icon {
    let (width, mut alpha) = pixels(text);
    if !visible {
        alpha.fill(0);
    }
    let rgba: Vec<u8> = alpha.into_iter().flat_map(|a| [255, 255, 255, a]).collect();
    Icon::from_rgba(rgba, width as u32, HEIGHT as u32).expect("valid tray icon")
}

/// Width and per-pixel alpha of the pill with `text` cut out.
fn pixels(text: &str) -> (usize, Vec<u8>) {
    // Tabular digits: every digit takes the widest digit's cell, so the icon keeps its width
    // while the number changes ("62.5" → "61.1"). Other characters keep their own advance.
    let digit_cell = ('0'..='9')
        .map(|d| FONT.metrics(d, FONT_SIZE).advance_width.ceil() as usize)
        .max()
        .unwrap_or(8);
    let glyphs: Vec<_> = text
        .chars()
        .map(|ch| {
            let (metrics, bitmap) = FONT.rasterize(ch, FONT_SIZE);
            let cell = if ch.is_ascii_digit() {
                digit_cell
            } else {
                metrics.advance_width.ceil() as usize
            };
            (ch, cell, metrics, bitmap)
        })
        .collect();
    let text_width: usize = glyphs.iter().map(|(_, cell, _, _)| cell).sum();
    let width = (text_width + 12).max(24);
    let mut alpha = vec![0_u8; width * HEIGHT];

    for y in 0..HEIGHT {
        for x in 0..width {
            let mut covered = 0_u16;
            for sy in 0..4 {
                for sx in 0..4 {
                    let px = x as f32 + (sx as f32 + 0.5) / 4.0;
                    let py = y as f32 + (sy as f32 + 0.5) / 4.0;
                    if in_rounded_rect(px, py, width as f32, HEIGHT as f32, 5.0) {
                        covered += 1;
                    }
                }
            }
            alpha[y * width + x] = (covered * 255 / 16) as u8;
        }
    }

    // One baseline for all glyphs, with the digits centred vertically: "." sits on the
    // baseline instead of every glyph being centred on its own.
    let digit = FONT.metrics('0', FONT_SIZE);
    let baseline = (HEIGHT as isize + digit.height as isize) / 2 + digit.ymin as isize;
    let mut pen_x = (width - text_width) / 2;
    for (ch, cell, metrics, bitmap) in glyphs {
        let top = (baseline - metrics.height as isize - metrics.ymin as isize).max(0) as usize;
        let left = if ch.is_ascii_digit() {
            pen_x + cell.saturating_sub(metrics.width) / 2
        } else {
            (pen_x as isize + metrics.xmin as isize).max(0) as usize
        };
        for gy in 0..metrics.height {
            for gx in 0..metrics.width {
                let (x, y) = (left + gx, top + gy);
                if x < width && y < HEIGHT {
                    let index = y * width + x;
                    alpha[index] = alpha[index].saturating_sub(bitmap[gy * metrics.width + gx]);
                }
            }
        }
        pen_x += cell;
    }
    (width, alpha)
}

fn in_rounded_rect(x: f32, y: f32, width: f32, height: f32, radius: f32) -> bool {
    if x < 0.0 || y < 0.0 || x >= width || y >= height {
        return false;
    }
    let cx = x.clamp(radius, width - radius);
    let cy = y.clamp(radius, height - radius);
    (x - cx).powi(2) + (y - cy).powi(2) <= radius.powi(2)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn digits_are_tabular() {
        let widths: Vec<usize> = ["62.5", "61.1", "11.1", "88.8", "10.7"]
            .iter()
            .map(|t| pixels(t).0)
            .collect();
        assert!(widths.windows(2).all(|w| w[0] == w[1]), "{widths:?}");
    }

    /// The pixels "." cuts out of the pill are in its lower half, on the digits' baseline.
    #[test]
    fn dot_sits_on_baseline() {
        let (width, with_dot) = pixels(".");
        let (empty_width, without) = pixels("");
        assert_eq!(width, empty_width);
        let rows: Vec<usize> = (0..HEIGHT)
            .filter(|y| (0..width).any(|x| with_dot[y * width + x] < without[y * width + x]))
            .collect();
        assert!(
            !rows.is_empty() && rows.iter().all(|y| *y > HEIGHT / 2),
            "{rows:?}"
        );
    }
}
