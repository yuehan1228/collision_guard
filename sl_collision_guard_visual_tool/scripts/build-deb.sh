#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
CONTROL_FILE="$ROOT_DIR/packaging/debian/control"

if ! command -v dpkg-deb >/dev/null 2>&1; then
    echo "dpkg-deb is required" >&2
    exit 1
fi

PACKAGE_NAME="$(awk '/^Package:/ {print $2}' "$CONTROL_FILE")"
VERSION="$(awk '/^Version:/ {print $2}' "$CONTROL_FILE")"
ARCH="$(dpkg --print-architecture)"
BUILD_ROOT="$ROOT_DIR/build/deb/${PACKAGE_NAME}_${VERSION}_${ARCH}"
OUTPUT_DEB="$ROOT_DIR/build/${PACKAGE_NAME}_${VERSION}_${ARCH}.deb"
PACKAGE_ROOT="/opt/collision-guard-visual-debugger/current"
UNIT_NAME="collision-guard-visual-debugger"

rm -rf "$BUILD_ROOT"
mkdir -p "$BUILD_ROOT/DEBIAN"
mkdir -p "$BUILD_ROOT$PACKAGE_ROOT"
mkdir -p "$BUILD_ROOT/etc/default"
mkdir -p "$BUILD_ROOT/lib/systemd/system"
mkdir -p "$BUILD_ROOT/usr/bin"
mkdir -p "$BUILD_ROOT/usr/share/applications"
mkdir -p "$BUILD_ROOT/usr/share/icons/hicolor/scalable/apps"

install -m 0644 "$CONTROL_FILE" "$BUILD_ROOT/DEBIAN/control"
install -m 0755 "$ROOT_DIR/packaging/debian/postinst" "$BUILD_ROOT/DEBIAN/postinst"
install -m 0755 "$ROOT_DIR/packaging/debian/prerm" "$BUILD_ROOT/DEBIAN/prerm"
install -m 0755 "$ROOT_DIR/packaging/debian/postrm" "$BUILD_ROOT/DEBIAN/postrm"

cp "$ROOT_DIR/app.py" "$BUILD_ROOT$PACKAGE_ROOT/app.py"
cp -r "$ROOT_DIR/static" "$BUILD_ROOT$PACKAGE_ROOT/static"
cp -r "$ROOT_DIR/config" "$BUILD_ROOT$PACKAGE_ROOT/config"

install -m 0644 "$ROOT_DIR/systemd/collision-guard-visual-debugger.service" "$BUILD_ROOT/lib/systemd/system/$UNIT_NAME.service"
install -m 0644 "$ROOT_DIR/systemd/collision-guard-visual-debugger.env" "$BUILD_ROOT/etc/default/$UNIT_NAME"
install -m 0755 "$ROOT_DIR/systemd/collision-guard-visual-debugger-open" "$BUILD_ROOT/usr/bin/collision-guard-visual-debugger-open"
install -m 0644 "$ROOT_DIR/systemd/collision-guard-visual-debugger.desktop" "$BUILD_ROOT/usr/share/applications/$UNIT_NAME.desktop"
install -m 0644 "$ROOT_DIR/static/collision-guard-visual-debugger.svg" "$BUILD_ROOT/usr/share/icons/hicolor/scalable/apps/$UNIT_NAME.svg"

dpkg-deb --build "$BUILD_ROOT" "$OUTPUT_DEB"
echo "$OUTPUT_DEB"
