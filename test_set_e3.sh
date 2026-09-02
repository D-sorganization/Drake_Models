set -e
has_label() { printf '%s' ",a,b," | grep -qiF ",$1,"; }
add_reason() { echo "reason: $1"; }
for hold in c d; do
  has_label "$hold" && add_reason "$hold"
done
echo "survived 3"
