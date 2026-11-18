#!/bin/bash
# an_gif.sh <in.mp4> <out.gif> [max_colors] [start_s] [dur_s] - GIF for analysis/README.md: 960 px wide, 12 fps, palette per clip.
# Runs ffmpeg at nice 19 with one thread. Prints size in bytes.
set -e
IN="$1"; OUT="$2"; NC="${3:-128}"; SS="${4:-0}"; DUR="${5:-}"
T=(); [ -n "$DUR" ] && T=(-t "$DUR")
PAL="$(mktemp --suffix=.png)"
F="fps=12,scale=960:-1:flags=lanczos"
nice -n 19 ffmpeg -v error -y -threads 1 -ss "$SS" "${T[@]}" -i "$IN" -vf "$F,palettegen=max_colors=$NC:stats_mode=diff" -threads 1 "$PAL"
nice -n 19 ffmpeg -v error -y -threads 1 -ss "$SS" "${T[@]}" -i "$IN" -i "$PAL" -lavfi "$F[x];[x][1:v]paletteuse=dither=bayer:bayer_scale=5:diff_mode=rectangle" -threads 1 "$OUT"
rm -f "$PAL"; stat -c %s "$OUT"
