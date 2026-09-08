#!/usr/bin/env python3
r"""
Gambar lintasan drone dalam tiga dimensi, sebagai PNG.

    python3 scripts/plot_trajectory3d.py ~/ros2_logs terbang_pertama
    python3 scripts/plot_trajectory3d.py ~/ros2_logs terbang_kedua \\
        --azim 60 --elev 20 --out ~/lintasan.png

Proyeksinya dihitung sendiri dengan matplotlib 2D, TIDAK memakai Axes3D.
Bukan pilihan gaya: di mesin ini `mpl_toolkits` berasal dari paket apt untuk
matplotlib 3.5.1 sedangkan matplotlib-nya versi pip 3.10.5, dan yang lama
mengimpor `matplotlib.docstring` yang sudah tidak ada. Penyebabnya bukan urutan
sys.path melainkan aturan impor Python: versi pip adalah namespace package
(tanpa __init__.py) sedangkan versi apt paket biasa, dan paket biasa selalu
menang. Menggambar proyeksinya sendiri melewati seluruh persoalan itu.

Gambar diam tidak bisa diputar, jadi kedalamannya harus digambar: ada bayangan
lintasan di lantai, garis jatuh berkala, dan kisi lantai. Tanpa ketiganya,
proyeksi ortografik statis gampang terbaca sebagai kurva 2D biasa.
"""

import argparse
import json
import math
import sys
import warnings
from pathlib import Path

# Lihat catatan di plot_stats.py: peringatan Axes3D tidak relevan di sini,
# justru karena berkas ini sengaja tidak memakainya.
warnings.filterwarnings('ignore', message='.*Axes3D.*')

import matplotlib                          # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt            # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap, Normalize  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from metrics_io import (GRID, INK, INK_MUTED, SEQ_BLUE,  # noqa: E402
                        SURFACE, TRACK, airborne_window, group_runs,
                        to_float, within)

CHOSEN = '#eb6834'      # slot kategorikal 2 — oranye
TOUCHDOWN = '#1baf7a'   # slot kategorikal 3 — aqua
START = '#4a3aa7'       # slot kategorikal 7 — violet


def project(x, y, z, azim_deg, elev_deg):
    """
    Proyeksi ortografik satu titik (x, y, z) ke layar (u, v).

    Putar dulu terhadap sumbu z sebesar `azim`, lalu miringkan pandangan
    sebesar `elev`. Ortografik, bukan perspektif: jarak yang sama panjangnya
    tergambar sama panjang di mana pun ia berada, sehingga gambar ini boleh
    diukur — perspektif akan membuat bagian yang jauh tampak lebih pendek.
    """
    a = math.radians(azim_deg)
    e = math.radians(elev_deg)
    xr = x * math.cos(a) + y * math.sin(a)
    yr = -x * math.sin(a) + y * math.cos(a)
    return (xr, yr * math.sin(e) + z * math.cos(e))


def equalise(xs, ys, zs):
    """
    Pusatkan ketiga sumbu dan bagi dengan rentang TERBESAR di antaranya.

    Skalanya harus sama untuk ketiganya. Pada penerbangan yang diuji, x hanya
    membentang 1,70 m sedangkan y 15,59 m dan z 14,03 m — meregangkan tiap
    sumbu agar memenuhi kanvas akan menggambar gerakan menyamping selebar
    gerakan maju, padahal drone praktis terbang di satu bidang tegak.

    Membagi dengan rentang terbesar menjaga perbandingannya: sumbu yang
    rentangnya separuh tetap tergambar separuh.
    """
    out = []
    spans = []
    for v in (xs, ys, zs):
        lo, hi = min(v), max(v)
        spans.append(hi - lo)
        out.append((lo + hi) / 2.0)
    scale = max(spans)
    if scale <= 0:
        scale = 1.0
    return out, scale


class View:
    """Satu sudut pandang: memusatkan, menyamakan skala, lalu memproyeksikan."""

    def __init__(self, xs, ys, zs, azim, elev):
        """Simpan pusat dan skala bersama dari awan titik yang diberikan."""
        (self.cx, self.cy, self.cz), self.scale = equalise(xs, ys, zs)
        self.azim = azim
        self.elev = elev

    def __call__(self, x, y, z):
        """Ubah koordinat dunia (meter) menjadi koordinat layar."""
        return project((x - self.cx) / self.scale,
                       (y - self.cy) / self.scale,
                       (z - self.cz) / self.scale,
                       self.azim, self.elev)

    def many(self, xs, ys, zs):
        """Versi banyak titik; mengembalikan dua daftar (u, v)."""
        pts = [self(x, y, z) for x, y, z in zip(xs, ys, zs)]
        return [p[0] for p in pts], [p[1] for p in pts]


def draw_floor(ax, view, xs, ys, z_floor, step=5.0):
    """Kisi lantai pada bidang x-y, ikut diproyeksikan.

    Kisi inilah yang membawa skala gambarnya: sumbu 2D-nya disembunyikan
    karena koordinat u/v hasil proyeksi tidak punya arti fisik, dan mencetak
    angkanya justru mengundang salah baca.
    """
    lo_x, hi_x = math.floor(min(xs) / step) * step, math.ceil(max(xs) / step) * step
    lo_y, hi_y = math.floor(min(ys) / step) * step, math.ceil(max(ys) / step) * step

    n = 0
    gx = lo_x
    while gx <= hi_x + 1e-9:
        u, v = view.many([gx, gx], [lo_y, hi_y], [z_floor, z_floor])
        ax.plot(u, v, color=GRID, linewidth=0.8, zorder=1)
        gx += step
        n += 1
    gy = lo_y
    while gy <= hi_y + 1e-9:
        u, v = view.many([lo_x, hi_x], [gy, gy], [z_floor, z_floor])
        ax.plot(u, v, color=GRID, linewidth=0.8, zorder=1)
        gy += step
    return step, n


def draw_path(ax, view, xs, ys, zs):
    """Lintasan berwarna menurut waktu, terang di awal, gelap di akhir."""
    u, v = view.many(xs, ys, zs)
    pts = list(zip(u, v))
    segs = [[pts[i], pts[i + 1]] for i in range(len(pts) - 1)]
    if not segs:
        return None

    cmap = LinearSegmentedColormap.from_list('waktu', SEQ_BLUE)
    lc = LineCollection(segs, cmap=cmap,
                        norm=Normalize(vmin=0, vmax=max(1, len(segs) - 1)),
                        linewidth=2.2, zorder=5)
    lc.set_array(range(len(segs)))
    ax.add_collection(lc)
    return lc


def draw_shadow(ax, view, xs, ys, zs, z_floor, every=20):
    """Bayangan lintasan di lantai, plus garis jatuh berkala.

    Dua petunjuk kedalaman yang paling murah. Tanpa keduanya, kurva
    melengkung di ruang tidak bisa dibedakan dari kurva yang sama di bidang
    datar — dan gambar diam tidak bisa diputar untuk memastikannya.
    """
    fz = [z_floor] * len(xs)
    su, sv = view.many(xs, ys, fz)
    ax.plot(su, sv, color=TRACK, linewidth=1.4, alpha=0.5, zorder=2)

    for i in range(0, len(xs), max(1, every)):
        u0, v0 = view(xs[i], ys[i], zs[i])
        u1, v1 = view(xs[i], ys[i], z_floor)
        ax.plot([u0, u1], [v0, v1], color=TRACK, linewidth=0.7,
                alpha=0.35, zorder=3)


def draw_marker(ax, view, p, color, label, marker='o', size=11, hollow=False):
    """Satu penanda titik penting, digambar di atas lintasan."""
    if not p:
        return
    u, v = view(p[0], p[1], p[2])
    ax.plot([u], [v], marker=marker, markersize=size,
            markerfacecolor='none' if hollow else color,
            markeredgecolor=color, markeredgewidth=2.2 if hollow else 1.4,
            linestyle='none', zorder=7, label=label)


def draw_scale_bar(ax, view, xs, ys, z_floor, metres):
    """Batang sepanjang `metres` di lantai, sebagai acuan ukuran."""
    x0, y0 = min(xs), min(ys)
    u, v = view.many([x0, x0 + metres], [y0, y0], [z_floor, z_floor])
    ax.plot(u, v, color=INK_MUTED, linewidth=2.5, zorder=6)
    ax.annotate('%g m' % metres, xy=((u[0] + u[1]) / 2, (v[0] + v[1]) / 2),
                xytext=(0, -14), textcoords='offset points',
                color=INK_MUTED, fontsize=9, ha='center')


def parse_args(argv):
    """Argumen baris perintah."""
    p = argparse.ArgumentParser(
        description='Gambar lintasan drone dalam 3D sebagai PNG.')
    p.add_argument('log_dir', help='direktori CSV, mis. ~/ros2_logs')
    p.add_argument('run', help='run_id atau stempel waktunya')
    p.add_argument('--azim', type=float, default=45.0,
                   help='putaran terhadap sumbu z, derajat (default 45)')
    p.add_argument('--elev', type=float, default=25.0,
                   help='kemiringan pandangan, derajat (default 25)')
    p.add_argument('--all-frames', dest='all_frames', action='store_true',
                   help='jangan batasi ke jendela udara')
    p.add_argument('--out', default=None, help='berkas PNG keluaran')
    return p.parse_args(argv)


def pick_run(runs, name):
    """Run yang ditunjuk, atau None bila tidak ada / ambigu."""
    hits = [k for k in runs
            if k == name or k.startswith(name + '_') or name in k]
    if len(hits) == 1:
        return hits[0]
    if not hits:
        print("Tidak ada run '%s'. Yang ada:\n  %s"
              % (name, '\n  '.join(sorted(runs))))
    else:
        print("'%s' menunjuk lebih dari satu run:\n  %s"
              % (name, '\n  '.join(sorted(hits))))
    return None


def main(argv=None):
    """Baca satu run, proyeksikan lintasannya, tulis PNG."""
    args = parse_args(sys.argv[1:] if argv is None else argv)
    root = Path(args.log_dir).expanduser()
    runs = group_runs(root)
    if not runs:
        print('Tidak ada CSV metrik di %s' % root)
        return 1

    key = pick_run(runs, args.run)
    if key is None:
        return 1

    kinds = runs[key]
    window = None if args.all_frames else airborne_window(kinds)
    rows = within(kinds.get('trajectory', []), window)
    if len(rows) < 2:
        print("Tidak ada data lintasan untuk '%s'.\n"
              'Apakah logger_stats berjalan (dan TF map -> base_link hidup) '
              'selama penerbangan itu?' % key)
        return 1

    xs = [to_float(r, 'x') for r in rows]
    ys = [to_float(r, 'y') for r in rows]
    zs = [to_float(r, 'z') for r in rows]
    keep = [i for i in range(len(xs))
            if None not in (xs[i], ys[i], zs[i])]
    xs = [xs[i] for i in keep]
    ys = [ys[i] for i in keep]
    zs = [zs[i] for i in keep]

    summary = (kinds.get('mission_summary') or [{}])[0]
    chosen = (json.loads(summary['chosen_point'])
              if summary.get('chosen_point') else None)
    touch = (json.loads(summary['touchdown'])
             if summary.get('touchdown') else None)

    z_floor = min(zs)
    view = View(xs, ys, zs, args.azim, args.elev)

    fig, ax = plt.subplots(figsize=(10, 8), facecolor=SURFACE)
    ax.set_facecolor(SURFACE)
    ax.set_aspect('equal')
    ax.axis('off')

    step, _ = draw_floor(ax, view, xs, ys, z_floor)
    draw_shadow(ax, view, xs, ys, zs, z_floor)
    lc = draw_path(ax, view, xs, ys, zs)
    draw_scale_bar(ax, view, xs, ys, z_floor, step)

    draw_marker(ax, view, (xs[0], ys[0], zs[0]), START, 'lepas landas',
                marker='s', size=9)
    draw_marker(ax, view, chosen, CHOSEN, 'titik terpilih', size=15,
                hollow=True)
    draw_marker(ax, view, touch, TOUCHDOWN, 'titik mendarat', marker='X',
                size=11)

    ax.autoscale_view()
    leg = ax.legend(loc='upper left', frameon=False, fontsize=9)
    for t in leg.get_texts():
        t.set_color(INK_MUTED)

    if lc is not None:
        bar = fig.colorbar(lc, ax=ax, fraction=0.03, pad=0.02)
        bar.set_label('urutan waktu (awal → akhir)', color=INK_MUTED,
                      fontsize=9)
        bar.set_ticks([])
        bar.outline.set_edgecolor(GRID)

    err = to_float(summary, 'landing_error_m')
    sub = 'x %.1f m   y %.1f m   z %.1f m' % (
        max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
    if err is not None:
        sub += '   galat %.3f m' % err
    fig.suptitle('Lintasan %s  —  %s\n%s'
                 % (key, summary.get('outcome', 'tidak diketahui'), sub),
                 color=INK, fontsize=12, x=0.06, ha='left')

    out = (Path(args.out).expanduser() if args.out
           else root / ('%s_trajectory3d.png' % key))
    fig.savefig(out, dpi=150, facecolor=SURFACE, bbox_inches='tight')
    print('Tersimpan: %s' % out)
    print('  %d sampel, sudut pandang azim=%g elev=%g, kisi %g m'
          % (len(xs), args.azim, args.elev, step))
    return 0


if __name__ == '__main__':
    sys.exit(main())
