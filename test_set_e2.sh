set -e
has_label() { printf '%s' ",a,b," | grep -qiF ",$1,"; }
has_label "c" && echo "found c"
echo "survived 2"
