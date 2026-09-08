#!/usr/bin/env python3
"""
Pembacaan CSV metrik dan palet gambar, dipakai bersama seluruh skrip di sini.

Tiga skrip membaca berkas yang sama dengan aturan yang sama — memisahkan
prefiks run dari nama berkas, mengembalikan kolom `extra`, membatasi ke jendela
udara. Menyalin aturan itu tiga kali berarti tiga tempat yang bisa berbeda
diam-diam, dan perbedaan seperti itu tidak akan muncul sebagai error melainkan
sebagai dua grafik yang tidak cocok tanpa ada yang tahu mana yang benar.

Bukan paket ROS: tidak ikut `colcon build`, cukup `python3 -m pytest scripts/`.
"""

import csv
import json
from collections import defaultdict
from pathlib import Path


# ── Palet ───────────────────────────────────────────────────────────────────
# Satu ramp biru untuk besaran kontinu, slot kategorikal berurutan untuk
# identitas. Teks selalu memakai tinta, tidak pernah warna seri.
SURFACE = '#fcfcfb'
INK = '#0b0b0b'
INK_MUTED = '#52514e'
GRID = '#e3e2de'

SEQ_BLUE = ['#cde2fb', '#9ec5f4', '#6da7ec', '#3987e5', '#256abf', '#104281']

# Dipakai berurutan, TIDAK PERNAH didaur: seri kesembilan bukan warna baru,
# melainkan tanda bahwa grafiknya perlu dipecah.
CATEGORICAL = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100',
               '#e87ba4', '#008300', '#4a3aa7', '#e34948']

TRACK = '#8a8983'   # jejak/konteks: bukan seri, jadi tidak memakai slot


def style(ax):
    """Sumbu dan grid yang mundur ke belakang, bukan bersaing dengan data."""
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=0.8, zorder=0)
    ax.set_axisbelow(True)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=INK_MUTED, labelsize=9)
    ax.xaxis.label.set_color(INK_MUTED)
    ax.yaxis.label.set_color(INK_MUTED)


# ── Pembacaan ───────────────────────────────────────────────────────────────

def load(path):
    """
    Baca CSV, dan gabungkan kembali kolom `extra` ke barisnya.

    `extra` berisi field yang muncul setelah header berkas itu terbentuk.
    Mengabaikannya akan mengulang kegagalan yang perekamnya sudah perbaiki:
    datanya ada di berkas, tapi tidak pernah sampai ke grafik.
    """
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            extra = r.pop('extra', '')
            if extra:
                try:
                    r.update(json.loads(extra))
                except ValueError:
                    pass
            rows.append(r)
    return rows


def split_name(path):
    """
    `(run_key, kind)` dari nama berkas, atau `(None, None)` bila bukan milik kita.

    Nama berkas: `<run_id>_<YYYYMMDD>_<HHMMSS>_<jenis>`. Baik `run_id` maupun
    `jenis` boleh memuat garis bawah (`mission_summary`), jadi pemisahnya
    dicari lewat pasangan tanggal+jam, bukan lewat posisi.

    `run_key` memuat stempel waktunya, BUKAN hanya `run_id`. Dua penerbangan
    dengan `run_id` yang sama memang terjadi — dan dengan kunci `run_id` saja,
    yang lebih lama tertimpa tanpa satu pun peringatan.
    """
    parts = Path(path).stem.split('_')
    for i in range(len(parts) - 1):
        if (len(parts[i]) == 8 and parts[i].isdigit()
                and len(parts[i + 1]) == 6 and parts[i + 1].isdigit()):
            if i == 0 or i + 2 >= len(parts):
                return (None, None)
            return ('_'.join(parts[:i + 2]), '_'.join(parts[i + 2:]))
    return (None, None)


def group_runs(root):
    """{run_key: {jenis: baris}} untuk seluruh CSV di satu direktori."""
    runs = defaultdict(dict)
    for p in sorted(Path(root).expanduser().glob('*.csv')):
        run_key, kind = split_name(p)
        if run_key is None:
            continue
        rows = load(p)
        if rows:
            runs[run_key][kind] = rows
    return dict(runs)


# ── Angka ───────────────────────────────────────────────────────────────────

def to_float(row, key, default=None):
    """Satu sel sebagai float, atau `default` bila kosong / bukan angka."""
    v = row.get(key, '')
    if v in ('', None):
        return default
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def nums(rows, key):
    """Satu kolom sebagai daftar float; sel kosong dan non-angka dilewati."""
    out = []
    for r in rows:
        v = to_float(r, key)
        if v is not None:
            out.append(v)
    return out


def numeric_columns(rows, sample=200):
    """
    Kolom yang isinya benar-benar angka, terurut sesuai urutan di berkas.

    Dipakai `--list` dan pesan galat: menawarkan `to_state` sebagai pilihan
    plot hanya memindahkan kegagalan ke traceback beberapa detik kemudian.
    """
    if not rows:
        return []
    skip = {'run_id', 'frame_id', 'source', 'kind', 'state'}
    out = []
    for key in rows[0]:
        if key in skip:
            continue
        vals = [to_float(r, key) for r in rows[:sample]]
        if any(v is not None for v in vals):
            out.append(key)
    return out


def seconds_from(stamps):
    """Ubah stempel nanosekon menjadi detik sejak sampel pertama."""
    if not stamps:
        return []
    t0 = min(stamps)
    return [(s - t0) / 1e9 for s in stamps]


# ── Jendela udara ───────────────────────────────────────────────────────────

def airborne_window(kinds):
    """
    (mulai_ns, selesai_ns) saat drone benar-benar di udara, atau None.

    Tanpa ini, statistik dan grafik mencakup SELURUH rekaman — termasuk
    menit-menit sebelum lepas landas dan sesudah mendarat, saat kamera hanya
    beberapa sentimeter dari tanah dan awan titiknya nyaris kosong. Terukur
    pada satu penerbangan: 60% frame berisi kurang dari 50 titik, dan median
    `downsampled_points` jatuh dari 1.845 ke 8. Angka seperti itu tidak salah
    membaca berkasnya — ia salah menjawab pertanyaannya.

    Batasnya diambil dari `recv_ns`, bukan `t_s`, karena `recv_ns` dicap oleh
    perekam yang sama untuk semua topik: tidak ada offset antar jam yang perlu
    ditebak.
    """
    tr = kinds.get('mission_transition') or kinds.get('transition') or []
    if not tr:
        return None

    def recv(pred):
        for r in tr:
            if pred(r):
                v = to_float(r, 'recv_ns')
                if v is not None:
                    return int(v)
        return None

    start = recv(lambda r: r.get('to_state') == 'TAKEOFF')
    # LANDED bila misinya mendarat; kalau menyerah ke HOLD, pakai transisi
    # terakhir — penerbangan yang gagal tetap punya jendela udara yang sah.
    end = recv(lambda r: r.get('to_state') == 'LANDED')
    if end is None:
        tail = [to_float(r, 'recv_ns') for r in tr]
        tail = [v for v in tail if v is not None]
        end = int(tail[-1]) if tail else None

    if start is None or end is None or end <= start:
        return None
    return (start, end)


def within(rows, window):
    """Hanya baris yang `recv_ns`-nya jatuh di dalam jendela (inklusif)."""
    if window is None:
        return rows
    lo, hi = window
    out = []
    for r in rows:
        v = to_float(r, 'recv_ns')
        if v is not None and lo <= int(v) <= hi:
            out.append(r)
    return out
