#!/bin/bash
# run_azahar.sh '<settings.txt contents>' [secs]: boot build/3ds/mm.3ds in Azahar, capture its window to
# build/debug/shot.png, report whether Azahar survived, its critical errors, and the tail of boot.log.
REPO="$(cd "$(dirname "$0")/../../.." && pwd)"
OUT="$REPO/build/debug"; mkdir -p "$OUT"
[ -x "$OUT/winid" ] || swiftc -O "$REPO/tools/port/debug/winid.swift" -o "$OUT/winid"
SD="$HOME/Library/Application Support/Azahar/sdmc/3ds/mm"
LOG="$HOME/Library/Application Support/Azahar/log/azahar_log.txt"
printf "$1" > "$SD/settings.txt"
pkill -9 -f "MacOS/azahar"; sleep 1; rm -f "$SD/boot.log"
APP=$(ls -d /Applications/*[Aa]zahar*/Azahar.app | head -1)
open -n -a "$APP" --args "$REPO/build/3ds/mm.3ds"
sleep "${2:-25}"
id=$("$OUT/winid" Azahar | sort -k2 -n -r | head -1 | awk '{print $1}')
[ -n "$id" ] && screencapture -x -o -l "$id" "$OUT/shot.png" && echo "captured window $id"
pgrep -f "MacOS/azahar" >/dev/null && echo "azahar alive" || echo "azahar GONE"
pkill -TERM -f "MacOS/azahar"; sleep 2; pkill -9 -f "MacOS/azahar"
grep -a "Critical\|unknown GPU" "$LOG" | tail -3 | cut -c1-160
tail -4 "$SD/boot.log" | cut -c1-120
