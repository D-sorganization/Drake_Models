set -e
has_label() {
  printf '%s' "a,b" | grep -q "c"
}
has_label && echo "yes"
echo "survived 5"
