set -euo pipefail
TIMELINE="test.json"
> "$TIMELINE"
LAST_DISARM="$((grep -v '^Bot	' "$TIMELINE" || true) | cut -f2 | sort | tail -1)"
echo "Success: $LAST_DISARM"
