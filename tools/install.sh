#!/bin/bash
# Install "Ultra Alert.app" into ~/Applications and start it at login via a per-user LaunchAgent.
# Also installs a daily 09:00 job (`ultra-alert daily`): a `git pull` of the maintainer's
# data and model, or on the maintainer's Mac new posts, labels, refit, report and a push;
# then a reinstall once no drones are over the oblast.
# Re-run after pulling changes to rebuild and restart. Uninstall:
#   launchctl bootout gui/$(id -u)/com.barthezslavik.ultra-alert
#   launchctl bootout gui/$(id -u)/com.barthezslavik.ultra-alert.daily
#   rm ~/Library/LaunchAgents/com.barthezslavik.ultra-alert{,.daily}.plist
#   rm -rf ~/Applications/"Ultra Alert.app"
set -euo pipefail
cd "$(dirname "$0")/.."

LABEL=com.barthezslavik.ultra-alert
PLIST="$HOME/Library/LaunchAgents/$LABEL.plist"
APP="$HOME/Applications/Ultra Alert.app"
LOG="$HOME/Library/Logs/ultra-alert.log"

launchctl bootout "gui/$(id -u)/$LABEL" 2>/dev/null || true
mkdir -p "$HOME/Applications"
tools/bundle.sh "$HOME/Applications" >/dev/null

mkdir -p "$(dirname "$PLIST")"
cat > "$PLIST" <<PLIST
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>Label</key><string>$LABEL</string>
  <key>ProgramArguments</key>
  <array><string>$APP/Contents/MacOS/ultra-alert</string></array>
  <key>RunAtLoad</key><true/>
  <!-- Restart after a crash, but not after "Вийти" from the menu (clean exit). -->
  <key>KeepAlive</key><dict><key>SuccessfulExit</key><false/></dict>
  <key>ProcessType</key><string>Interactive</string>
  <key>StandardErrorPath</key><string>$LOG</string>
  <key>StandardOutPath</key><string>$LOG</string>
</dict>
</plist>
PLIST

launchctl bootstrap "gui/$(id -u)" "$PLIST"
echo "installed: $APP"
echo "agent:     $PLIST"
echo "log:       $LOG"

# Daily update. Not re-registered when the daily job itself runs this script.
DAILY=$LABEL.daily
DAILY_PLIST="$HOME/Library/LaunchAgents/$DAILY.plist"
if [ -z "${ULTRA_ALERT_DAILY:-}" ]; then
  launchctl bootout "gui/$(id -u)/$DAILY" 2>/dev/null || true
  cat > "$DAILY_PLIST" <<PLIST
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>Label</key><string>$DAILY</string>
  <key>ProgramArguments</key>
  <array><string>$APP/Contents/MacOS/ultra-alert</string><string>daily</string></array>
  <key>WorkingDirectory</key><string>$PWD</string>
  <key>StartCalendarInterval</key><dict><key>Hour</key><integer>9</integer><key>Minute</key><integer>0</integer></dict>
  <key>EnvironmentVariables</key>
  <dict>
    <key>PATH</key><string>$HOME/.cargo/bin:$HOME/.local/bin:/opt/homebrew/bin:/usr/local/bin:/usr/bin:/bin</string>
    <key>ULTRA_ALERT_DAILY</key><string>1</string>
  </dict>
  <key>StandardErrorPath</key><string>$HOME/Library/Logs/ultra-alert-daily.log</string>
  <key>StandardOutPath</key><string>$HOME/Library/Logs/ultra-alert-daily.log</string>
</dict>
</plist>
PLIST
  launchctl bootstrap "gui/$(id -u)" "$DAILY_PLIST"
  echo "daily:     $DAILY_PLIST (09:00, log ~/Library/Logs/ultra-alert-daily.log)"
fi
