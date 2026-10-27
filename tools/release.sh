#!/bin/bash
# Build "Ultra Alert.app" for Apple silicon and Intel in one, zip it and publish it as a GitHub
# release, for those who do not build it themselves.
# usage: tools/release.sh v0.1.0
set -euo pipefail
cd "$(dirname "$0")/.."
tag=${1:?usage: tools/release.sh <tag>}

target=$(cargo metadata --format-version 1 --no-deps | sed -E 's/.*"target_directory":"([^"]*)".*/\1/')
for t in aarch64-apple-darwin x86_64-apple-darwin; do
  rustup target add "$t" >/dev/null
  cargo build --release --target "$t"
done
out="$target/release-app"
rm -rf "$out"
mkdir -p "$out"
lipo -create -output "$out/ultra-alert" \
  "$target/aarch64-apple-darwin/release/ultra-alert" \
  "$target/x86_64-apple-darwin/release/ultra-alert"
BIN="$out/ultra-alert" tools/bundle.sh "$out" >/dev/null
app="$out/Ultra Alert.app"
# Ad-hoc: Apple silicon runs nothing unsigned; Gatekeeper still asks once (no Developer ID).
codesign --force --sign - "$app"
zip="$out/Ultra-Alert-Vinnytsia-$tag.zip"
ditto -c -k --keepParent "$app" "$zip"
lipo -archs "$app/Contents/MacOS/ultra-alert"

gh release create "$tag" "$zip" --title "Ultra Alert — Вінниця $tag" --notes "Готова програма для Mac (Apple silicon і Intel). Як встановити — у README, розділ «Готова програма»."
