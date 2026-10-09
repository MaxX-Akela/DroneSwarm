#!/bin/bash
# Build the drone-swarm .deb: packaging/build-deb.sh <version> [out_dir]
set -euo pipefail

VERSION="${1:?usage: build-deb.sh <version> [out_dir]}"
OUT_DIR="${2:-$PWD/dist}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$HERE")"
STAGE="$(mktemp -d)"
trap 'rm -rf "$STAGE"' EXIT
chmod 755 "$STAGE"

install -d "$STAGE/usr/bin" "$STAGE/DEBIAN" "$STAGE/opt/droneswarm" "$STAGE/lib/systemd/system" "$STAGE/usr/share/droneswarm"

tar -C "$ROOT" --exclude='__pycache__' --exclude='*.pyc' --exclude='drone.ini' --exclude='animation.csv' \
    -cf - drone | tar -C "$STAGE/opt/droneswarm" -xf -
install -m 644 "$HERE/droneswarm.service" "$STAGE/lib/systemd/system/droneswarm.service"
install -m 755 "$HERE/drone-setup" "$STAGE/usr/bin/drone-setup"
install -m 644 "$ROOT/builder/assets/chrony-drone.conf" "$STAGE/usr/share/droneswarm/chrony-drone.conf"

sed "s/@VERSION@/$VERSION/" "$HERE/control" > "$STAGE/DEBIAN/control"
for s in postinst prerm postrm; do install -m 755 "$HERE/$s" "$STAGE/DEBIAN/$s"; done

mkdir -p "$OUT_DIR"
dpkg-deb --root-owner-group --build "$STAGE" "$OUT_DIR/drone-swarm_${VERSION}_all.deb"
