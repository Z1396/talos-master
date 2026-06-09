#!/usr/bin/env bash

set -Eeuo pipefail

if [[ $# -ne 1 ]]; then
    echo "usage: $0 <sysroot>" >&2
    exit 1
fi

sysroot="$1"

if [[ ! -d "$sysroot" ]]; then
    echo "sysroot does not exist: $sysroot" >&2
    exit 1
fi

sysroot="$(cd "$sysroot" && pwd)"

fixed=0
scanned=0

while IFS= read -r -d '' link; do
    ((scanned += 1))

    target="$(readlink "$link")"
    [[ "$target" == /* ]] || continue

    candidate="${sysroot}${target}"
    [[ -e "$candidate" ]] || continue

    link_dir="$(dirname "$link")"
    relative_target="$(realpath --relative-to="$link_dir" "$candidate")"

    rm "$link"
    ln -s "$relative_target" "$link"
    ((fixed += 1))

    printf 'fixed %s -> %s\n' "$link" "$relative_target"
done < <(find "$sysroot" \( -path "$sysroot/proc" -o -path "$sysroot/sys" -o -path "$sysroot/dev" \) -prune -o -type l -print0)

printf 'scanned symlinks: %d\n' "$scanned"
printf 'rewritten symlinks: %d\n' "$fixed"
