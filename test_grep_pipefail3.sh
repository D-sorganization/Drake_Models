set -eo pipefail
has_label() { printf '%s' "a" | grep -qiF "$1"; }
has_label "b" && echo "found"
echo "survived has_label"
