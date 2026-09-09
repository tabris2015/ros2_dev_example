#!/usr/bin/env bash
# Scaffold a lesson package from the templates next to this script.
#
#   bash .claude/skills/lesson/scaffold.sh <NN> <topic>
#
# Placeholders replaced in file contents and paths:
#   PACKAGE_NAME  tut<NN>_<topic>
#   LESSON_NN     two-digit lesson number
#   LESSON_TITLE  topic in Title Case
#   TOPIC         the topic, also the starter node name
#   YEAR          current year
# The template directory named __PKG__ becomes the Python package directory.
set -euo pipefail

if [ $# -ne 2 ]; then
  echo "usage: $0 <NN> <topic>" >&2
  exit 2
fi

nn=$(printf '%02d' "$((10#$1))")
topic=$2
if ! [[ "$topic" =~ ^[a-z][a-z0-9_]*$ ]]; then
  echo "topic must be snake_case (got '$topic')" >&2
  exit 2
fi

root=$(git rev-parse --show-toplevel)
pkg="tut${nn}_${topic}"
dest="$root/src/$pkg"
templates="$(cd "$(dirname "$0")" && pwd)/templates"
title=$(echo "$topic" | sed 's/_/ /g; s/\b\(.\)/\u\1/g')
year=$(date +%Y)

if [ -e "$dest" ]; then
  echo "refusing to overwrite $dest" >&2
  exit 1
fi

while IFS= read -r rel; do
  out="$dest/$rel"
  out="${out//__PKG__/$pkg}"
  out="${out//TOPIC/$topic}"
  mkdir -p "$(dirname "$out")"
  sed -e "s/PACKAGE_NAME/$pkg/g" \
      -e "s/LESSON_NN/$nn/g" \
      -e "s/LESSON_TITLE/$title/g" \
      -e "s/TOPIC/$topic/g" \
      -e "s/YEAR/$year/g" \
      "$templates/$rel" > "$out"
done < <(cd "$templates" && find . -type f | sed 's|^\./||' | sort)

chmod +x "$dest"/scripts/* "$dest"/solution/scripts/* 2>/dev/null || true

echo "created $dest"
(cd "$root" && find "src/$pkg" -type f | sort)
