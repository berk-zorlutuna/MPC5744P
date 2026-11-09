#!/bin/sh
# Traceability consistency check:
#   SYS (docs/01) -> SWR (docs/02) -> SVC (docs/03) <-> test registry (tests/vtest.c)
# Exit status 0 if consistent, 1 otherwise.
set -u
cd "$(dirname "$0")/.." || exit 1

SYS_DOC=docs/01_system_requirements.md
SWR_DOC=docs/02_software_requirements.md
SVC_DOC=docs/03_software_verification_cases.md
REG=tests/vtest.c
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
ERR=0

fail() { echo "ERROR: $*"; ERR=1; }

# 1. Definitions
grep -oE '^\| SYS-[0-9]{3} \|' "$SYS_DOC" | grep -oE 'SYS-[0-9]{3}' | sort > "$TMP/sys_def"
grep -oE '^\| SWR-[A-Z]+-[0-9]{3} \|' "$SWR_DOC" | grep -oE 'SWR-[A-Z]+-[0-9]{3}' | sort > "$TMP/swr_def"
sort -c -u "$TMP/sys_def" 2>/dev/null || fail "duplicate SYS IDs in $SYS_DOC"
sort -c -u "$TMP/swr_def" 2>/dev/null || fail "duplicate SWR IDs in $SWR_DOC"

# 2. SYS <-> SWR (column "Traces to" in docs/02)
grep -E '^\| SWR-' "$SWR_DOC" | grep -oE 'SYS-[0-9]{3}' | sort -u > "$TMP/sys_used"
for id in $(comm -23 "$TMP/sys_def" "$TMP/sys_used"); do fail "$id is not refined by any software requirement"; done
for id in $(comm -13 "$TMP/sys_def" "$TMP/sys_used"); do fail "$id referenced in $SWR_DOC but not defined in $SYS_DOC"; done

# 3. Case -> requirement pairs from the case document
awk '
  /^### SVC-/ { cur = $2; next }
  /^\| Verifies \|/ && cur != "" {
      line = $0; sub(/^\| Verifies \| */, "", line); sub(/ *\|$/, "", line)
      n = split(line, a, /, */); for (i = 1; i <= n; i++) print cur, a[i]; cur = "" }
' "$SVC_DOC" | sort > "$TMP/pairs_doc"
grep -E '^### SVC-' "$SVC_DOC" | sed -E 's/^### //' | sort > "$TMP/titles_doc"

# 4. Same from the test registry
grep -E '^ *\{ "SVC-' "$REG" | sed -E 's/^ *\{ "([^"]+)", "([^"]+)", "([^"]+)".*/\1|\2|\3/' > "$TMP/reg"
awk -F'|' '{ n = split($3, a, /, */); for (i = 1; i <= n; i++) print $1, a[i] }' "$TMP/reg" | sort > "$TMP/pairs_reg"
awk -F'|' '{ print $1 " " $2 }' "$TMP/reg" | sort > "$TMP/titles_reg"

if ! diff "$TMP/pairs_doc" "$TMP/pairs_reg" > "$TMP/d1"; then
    fail "'Verifies' differs between $SVC_DOC (<) and $REG (>):"; cat "$TMP/d1"
fi
if ! diff "$TMP/titles_doc" "$TMP/titles_reg" > "$TMP/d2"; then
    fail "case titles differ between $SVC_DOC (<) and $REG (>):"; cat "$TMP/d2"
fi

# 5. Every SWR verified; every verified SWR defined
awk '{ print $2 }' "$TMP/pairs_doc" | sort -u > "$TMP/swr_ver"
for id in $(comm -23 "$TMP/swr_def" "$TMP/swr_ver"); do fail "$id has no verification case"; done
for id in $(comm -13 "$TMP/swr_def" "$TMP/swr_ver"); do fail "$id is verified but not defined in $SWR_DOC"; done

# 6. Traceability matrix (section 11) matches the case definitions
sed -n '/^## 11\./,$p' "$SVC_DOC" | grep -E '^\| SWR-' | awk -F'|' '
  { swr = $2; gsub(/ /, "", swr); n = split($3, a, /,/);
    for (i = 1; i <= n; i++) { c = a[i]; gsub(/ /, "", c); print c, swr } }' | sort > "$TMP/pairs_mx"
if ! diff "$TMP/pairs_doc" "$TMP/pairs_mx" > "$TMP/d3"; then
    fail "traceability matrix differs from case definitions (< cases, > matrix):"; cat "$TMP/d3"
fi

echo "System requirements:   $(wc -l < "$TMP/sys_def")"
echo "Software requirements: $(wc -l < "$TMP/swr_def")"
echo "Verification cases:    $(wc -l < "$TMP/titles_doc")"
echo "Case/requirement links: $(wc -l < "$TMP/pairs_doc")"
if [ "$ERR" -eq 0 ]; then echo "Traceability: OK"; else echo "Traceability: FAILED"; fi
exit "$ERR"
