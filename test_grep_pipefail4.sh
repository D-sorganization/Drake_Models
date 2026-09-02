set -eo pipefail
if false; then
  echo "false"
elif printf '%s' "hello" | grep -qiE 'world'; then
  echo "world"
fi
echo "survived elif"
