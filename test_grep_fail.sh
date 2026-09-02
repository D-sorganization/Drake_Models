set -e
TIMELINE="test.json"
echo "Bot	something" > "$TIMELINE"
LAST_DISARM="$(grep -v '^Bot	' "$TIMELINE" | cut -f2 | sort | tail -1)"
echo "survived grep"
