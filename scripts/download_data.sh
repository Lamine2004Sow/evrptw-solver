#!/usr/bin/env bash
# Downloads the E-VRPTW instances of Schneider, Stenger & Goeke (2014) from Mendeley Data
# (https://data.mendeley.com/datasets/h3mrm5dhxw/1) into data/instances/.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DEST="$ROOT/data/instances"
URL="https://data.mendeley.com/public-files/datasets/h3mrm5dhxw/files/771dcb44-af89-4dbc-986b-e0c76724f444/file_downloaded"
SHA256="54e305a03cc0aa8aba085d3e7fefc4e3357aba34357e0cd1f84c5b36c6c1df02"

if [ -f "$DEST/r101_21.txt" ]; then
  echo "instances already present in $DEST"
  exit 0
fi

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
curl -sSL -o "$TMP/evrptw.zip" "$URL"
echo "$SHA256  $TMP/evrptw.zip" | sha256sum -c -
unzip -q "$TMP/evrptw.zip" -d "$TMP"
mkdir -p "$DEST"
cp "$TMP"/evrptw_instances/*.txt "$DEST"/
echo "$(ls "$DEST" | grep -c '\.txt$') files in $DEST"
