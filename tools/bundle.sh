#!/bin/bash
# Build "Ultra Alert.app" (menu-bar only, LSUIElement) from a release build.
# usage: tools/bundle.sh [install-dir]   (default: the cargo target directory)
# BIN=<binary> bundles that one (tools/release.sh: Apple silicon + Intel) instead of building.
set -euo pipefail
cd "$(dirname "$0")/.."
target=$(cargo metadata --format-version 1 --no-deps | sed -E 's/.*"target_directory":"([^"]*)".*/\1/')
if [ -z "${BIN:-}" ]; then
  cargo build --release
  BIN="$target/release/ultra-alert"
fi
version=$(sed -nE 's/^version = "(.*)"/\1/p' Cargo.toml | head -1)
app="${1:-$target/release}/Ultra Alert.app"
rm -rf "$app"
mkdir -p "$app/Contents/MacOS"
cp "$BIN" "$app/Contents/MacOS/ultra-alert"
cat > "$app/Contents/Info.plist" <<PLIST
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>CFBundleName</key><string>Ultra Alert</string>
  <key>CFBundleIdentifier</key><string>com.barthezslavik.ultra-alert</string>
  <key>CFBundleExecutable</key><string>ultra-alert</string>
  <key>CFBundlePackageType</key><string>APPL</string>
  <key>CFBundleShortVersionString</key><string>$version</string>
  <key>LSUIElement</key><true/>
</dict>
</plist>
PLIST
echo "$app"
