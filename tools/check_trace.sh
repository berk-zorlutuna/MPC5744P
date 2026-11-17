#!/bin/sh
# Traceability consistency check.
#   Software: SYS (01) -> SWR (02) -> SVC (03) <-> test registry (tests/vtest.c)
#   Hardware: SYS (01) -> HWR (05) -> HVC (07) <-> analysis registry (tools/hw_analysis.py)
# Exit status 0 if consistent, 1 otherwise.
set -u
cd "$(dirname "$0")/.." || exit 1

SYS_DOC=docs/01_system_requirements.md
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
ERR=0
fail() { echo "ERROR: $*"; ERR=1; }

grep -oE '^\| SYS-[0-9]{3} \|' "$SYS_DOC" | grep -oE 'SYS-[0-9]{3}' | sort > "$TMP/sys_def"
sort -c -u "$TMP/sys_def" 2>/dev/null || fail "duplicate SYS IDs in $SYS_DOC"
: > "$TMP/sys_used"

# check_chain NAME REQ_DOC REQ_PREFIX CASE_DOC CASE_PREFIX REGISTRY_FILE(id|title|verifies)
check_chain() {
    name=$1; req_doc=$2; rp=$3; case_doc=$4; cp=$5; reg=$6
    grep -oE "^\| $rp-[A-Z]+-[0-9]{3} \|" "$req_doc" | grep -oE "$rp-[A-Z]+-[0-9]{3}" | sort > "$TMP/req_def"
    sort -c -u "$TMP/req_def" 2>/dev/null || fail "duplicate $rp IDs in $req_doc"
    grep -E "^\| $rp-" "$req_doc" | grep -oE 'SYS-[0-9]{3}' | sort -u > "$TMP/sys_this"
    for id in $(comm -13 "$TMP/sys_def" "$TMP/sys_this"); do fail "$id referenced in $req_doc but not defined"; done
    cat "$TMP/sys_this" >> "$TMP/sys_used"

    awk '
      /^### / && $2 ~ /^[A-Z]+-/ { cur = $2; next }
      /^\| Verifies \|/ && cur != "" {
          line = $0; sub(/^\| Verifies \| */, "", line); sub(/ *\|$/, "", line)
          n = split(line, a, /, */); for (i = 1; i <= n; i++) print cur, a[i]; cur = "" }
    ' "$case_doc" | sort > "$TMP/pairs_doc"
    grep -E "^### $cp-" "$case_doc" | sed -E 's/^### //' | sort > "$TMP/titles_doc"
    awk -F'|' '{ n = split($3, a, /, */); for (i = 1; i <= n; i++) print $1, a[i] }' "$reg" | sort > "$TMP/pairs_reg"
    awk -F'|' '{ print $1 " " $2 }' "$reg" | sort > "$TMP/titles_reg"

    diff "$TMP/pairs_doc" "$TMP/pairs_reg" > "$TMP/d1" || { fail "$name: 'Verifies' differs, $case_doc (<) vs registry (>)"; cat "$TMP/d1"; }
    diff "$TMP/titles_doc" "$TMP/titles_reg" > "$TMP/d2" || { fail "$name: case titles differ, $case_doc (<) vs registry (>)"; cat "$TMP/d2"; }

    awk '{ print $2 }' "$TMP/pairs_doc" | sort -u > "$TMP/req_ver"
    for id in $(comm -23 "$TMP/req_def" "$TMP/req_ver"); do fail "$id has no verification case"; done
    for id in $(comm -13 "$TMP/req_def" "$TMP/req_ver"); do fail "$id is verified but not defined in $req_doc"; done

    sed -n '/^## .*[Tt]raceability matrix/,$p' "$case_doc" | grep -E "^\| $rp-" | awk -F'|' '
      { r = $2; gsub(/ /, "", r); n = split($3, a, /,/);
        for (i = 1; i <= n; i++) { c = a[i]; gsub(/ /, "", c); print c, r } }' | sort > "$TMP/pairs_mx"
    diff "$TMP/pairs_doc" "$TMP/pairs_mx" > "$TMP/d3" || { fail "$name: traceability matrix differs from case definitions (< cases, > matrix)"; cat "$TMP/d3"; }

    printf '%-9s requirements: %3d  cases: %3d  links: %3d\n' "$name" \
        "$(wc -l < "$TMP/req_def")" "$(wc -l < "$TMP/titles_doc")" "$(wc -l < "$TMP/pairs_doc")"
}

grep -E '^ *\{ "SVC-' tests/vtest.c | sed -E 's/^ *\{ "([^"]+)", "([^"]+)", "([^"]+)".*/\1|\2|\3/' > "$TMP/reg_sw"
check_chain Software docs/02_software_requirements.md SWR docs/03_software_verification_cases.md SVC "$TMP/reg_sw"

python3 tools/hw_analysis.py --list > "$TMP/reg_hw" || fail "cannot list hardware cases"
check_chain Hardware docs/05_hardware_requirements.md HWR docs/07_hardware_verification_cases.md HVC "$TMP/reg_hw"

sort -u "$TMP/sys_used" -o "$TMP/sys_used"
for id in $(comm -23 "$TMP/sys_def" "$TMP/sys_used"); do fail "$id is not refined by any software or hardware requirement"; done
echo "System requirements: $(wc -l < "$TMP/sys_def")"
if [ "$ERR" -eq 0 ]; then echo "Traceability: OK"; else echo "Traceability: FAILED"; fi
exit "$ERR"
