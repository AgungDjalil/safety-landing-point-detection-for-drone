#!/usr/bin/env bash
# Pasang aset simulasi yang dimodifikasi ke sebuah checkout PX4-Autopilot.
#
# Pemakaian:
#   ./px4_assets/install.sh [PX4_DIR] [--dry-run]
#
# PX4_DIR baku: $PX4_SOURCE_DIR bila diset, selain itu ~/PX4-Autopilot.
# Setiap berkas yang ditimpa dicadangkan lebih dulu menjadi <nama>.orig
# (pencadangan hanya dilakukan sekali, sehingga menjalankan ulang skrip ini
# tidak akan menimpa cadangan aslinya).
set -euo pipefail

SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

DRY=0
PX4_DIR=""
for arg in "$@"; do
    case "$arg" in
        --dry-run) DRY=1 ;;
        -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
        *) PX4_DIR="$arg" ;;
    esac
done
: "${PX4_DIR:=${PX4_SOURCE_DIR:-$HOME/PX4-Autopilot}}"
PX4_DIR="${PX4_DIR/#\~/$HOME}"

if [ ! -f "$PX4_DIR/Makefile" ] || [ ! -d "$PX4_DIR/boards" ]; then
    echo "install.sh: '$PX4_DIR' tidak terlihat seperti akar PX4-Autopilot." >&2
    echo "  Berikan lokasinya:  ./px4_assets/install.sh /path/ke/PX4-Autopilot" >&2
    exit 1
fi

echo "Memasang aset simulasi ke: $PX4_DIR"
[ "$DRY" = 1 ] && echo "(dry-run: tidak ada berkas yang ditulis)"

count=0
while IFS= read -r -d '' src; do
    rel="${src#"$SRC"/}"
    case "$rel" in install.sh|README.md) continue ;; esac
    dst="$PX4_DIR/$rel"

    if [ "$DRY" = 1 ]; then
        if [ -f "$dst" ] && cmp -s "$src" "$dst"; then
            echo "  sama    $rel"
        elif [ -f "$dst" ]; then
            echo "  TIMPA   $rel"
        else
            echo "  BARU    $rel"
        fi
    else
        mkdir -p "$(dirname "$dst")"
        if [ -f "$dst" ] && [ ! -f "$dst.orig" ] && ! cmp -s "$src" "$dst"; then
            cp -p "$dst" "$dst.orig"
            echo "  cadangan dibuat: $rel.orig"
        fi
        cp -p "$src" "$dst"
        echo "  terpasang $rel"
    fi
    count=$((count + 1))
done < <(find "$SRC" -type f ! -name install.sh ! -name README.md -print0)

echo "Selesai: $count berkas."
if [ "$DRY" != 1 ]; then
    echo
    echo "Langkah berikutnya: bangun ulang PX4 SITL supaya airframe yang baru terbaca,"
    echo "  cd $PX4_DIR && make px4_sitl"
fi
