#!/bin/sh
# Create the generated symlinks a PlatformIO build of the Saleae app needs:
# the shared firmware in common/, and the library itself. Same idea as
# extras/scripts/build-pio-dirs.sh, kept local to this harness.
set -e
ROOT=$(cd "$(dirname "$0")/../../../.." && pwd)
APP="$ROOT/extras/tests/saleae_based/apps/arduino"
COMMON="$ROOT/extras/tests/saleae_based/common"

mkdir -p "$APP/src" "$APP/FastAccelStepper"
ln -sfn "$ROOT/src" "$APP/FastAccelStepper/src"

for f in "$COMMON"/*.cpp "$COMMON"/*.h; do
	ln -sfn "$f" "$APP/src/$(basename "$f")"
done

echo "linked $(ls "$APP/src" | wc -l | tr -d ' ') files into $APP/src"
