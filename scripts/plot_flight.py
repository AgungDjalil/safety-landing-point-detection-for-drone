#!/usr/bin/env python3
"""
Gambar satu penerbangan: lintasan drone dan profil ketinggiannya.

Pemakaian:
    python3 scripts/plot_flight.py ~/ros2_logs gng

Menghasilkan `<run>_<stamp>_flight.png` di direktori yang sama dengan CSV-nya.

Dua panel, dua pertanyaan berbeda:

  TAMPAK ATAS   ke mana drone pergi, di mana kandidat terkumpul, dan seberapa
                layak masing-masing menurut skornya.

  KETINGGIAN    bagaimana bentuk turunnya terhadap waktu, dengan batas antar
                state ditandai supaya SCAN yang menggantung atau DESCEND yang
                terpotong abort langsung terlihat.

Warna dipakai untuk SATU pekerjaan di tiap panel. Di tampak atas, kanal warna
dipegang oleh `score` kandidat — besaran kontinu, jadi satu hue dari terang ke
gelap, bukan pelangi. Lintasannya sendiri digambar abu-abu: ia konteks, bukan
besaran, dan mewarnainya akan merebut kanal yang sedang dipakai menjawab
pertanyaan lain.

Hanya butuh matplotlib dan pustaka standar.
"""

import json
import sys
from pathlib import Path

import warnings

# Lihat catatan yang sama di plot_stats.py: peringatan Axes3D tidak relevan
# untuk gambar 2D, tapi terbaca seperti kegagalan.
warnings.filterwarnings('ignore', message='.*Axes3D.*')

import matplotlib                          # noqa: E402
matplotlib.use('Agg')                      # tanpa display; tulis PNG saja
import matplotlib.pyplot as plt            # noqa: E402
from matplotlib.colors import LinearSegmentedColormap, Normalize  # noqa: E402


sys.path.insert(0, str(Path(__file__).resolve().parent))
from metrics_io import (GRID, INK, INK_MUTED, SEQ_BLUE,  # noqa: E402
                        SURFACE, TRACK, load, seconds_from, style,
                        to_float)

CHOSEN = '#eb6834'      # slot kategorikal 2 — oranye
TOUCHDOWN = '#1baf7a'   # slot kategorikal 3 — aqua


def find(root, run_id, kind):
    """Berkas `<run>_<stamp>_<kind>.csv` terbaru untuk satu run."""
    hits = sorted(Path(root).glob('%s_*_%s.csv' % (run_id, kind)))
    for p in reversed(hits):
        rows = load(p)
        if rows:
            return p, rows
    return (hits[-1] if hits else None), []


def plot_top_view(ax, traj, cands, summary):
    """Tampak atas: lintasan, kandidat berwarna menurut skor, dua penanda."""
    ax.set_title('Tampak atas — lintasan dan kandidat',
                 color=INK, fontsize=11, loc='left', pad=10)
    ax.set_xlabel('x map (m, timur)')
    ax.set_ylabel('y map (m, utara)')

    # Kandidat: satu titik per kandidat unik, memakai skor TERAKHIR yang
    # tercatat untuknya. Memplot tiap baris apa adanya akan menumpuk ribuan
    # titik di posisi yang sama dan membuat kepadatan gambar mencerminkan
    # jumlah frame, bukan jumlah kandidat.
    latest = {}
    for r in cands:
        x, y, sc = to_float(r, 'x'), to_float(r, 'y'), to_float(r, 'score')
        if x is None or y is None or sc is None:
            continue
        latest[(round(x, 2), round(y, 2))] = sc

    if latest:
        cmap = LinearSegmentedColormap.from_list('score_blue', SEQ_BLUE)
        xs = [k[0] for k in latest]
        ys = [k[1] for k in latest]
        cs = list(latest.values())
        # Skalanya mengikuti rentang skor yang BENAR-BENAR muncul, bukan
        # 0..1 penuh. Skor lapangan berkerumun di pita sempit (terukur
        # 0,22-0,74), dan memaksakan sumbu penuh membuat seluruh kandidat
        # tampak sewarna -- kanal warnanya terpakai tapi tidak mengatakan
        # apa-apa. Batasnya dicetak di label supaya tidak ada yang mengira
        # biru tergelap berarti skor sempurna.
        lo, hi = min(cs), max(cs)
        if hi - lo < 1e-6:
            lo, hi = lo - 0.05, hi + 0.05
        sc = ax.scatter(xs, ys, c=cs, cmap=cmap,
                        norm=Normalize(vmin=lo, vmax=hi),
                        s=46, linewidths=0.8, edgecolors=SURFACE,
                        zorder=3, label='kandidat (n=%d)' % len(latest))
        bar = ax.figure.colorbar(sc, ax=ax, fraction=0.046, pad=0.02)
        bar.set_label('score  (%.2f - %.2f)' % (lo, hi),
                      color=INK_MUTED, fontsize=9)
        bar.ax.tick_params(colors=INK_MUTED, labelsize=8)
        bar.outline.set_edgecolor(GRID)

    if traj:
        tx = [to_float(r, 'x') for r in traj]
        ty = [to_float(r, 'y') for r in traj]
        ax.plot(tx, ty, color=TRACK, linewidth=2.0, zorder=2,
                label='lintasan drone')

    chosen = json.loads(summary['chosen_point']) if summary.get(
        'chosen_point') else None
    touch = json.loads(summary['touchdown']) if summary.get(
        'touchdown') else None

    # Titik terpilih digambar sebagai CINCIN, bukan cakram penuh: pendaratan
    # yang berhasil menaruh kedua penanda berjarak beberapa sentimeter, dan
    # cakram penuh akan tertutup rapat oleh penanda mendarat di atasnya --
    # gambar yang paling ingin dilihat justru jadi gambar yang menyembunyikan
    # separuh isinya.
    if chosen:
        ax.plot(chosen[0], chosen[1], marker='o', markersize=16,
                markerfacecolor='none', markeredgecolor=CHOSEN,
                markeredgewidth=2.5, linestyle='none', zorder=5,
                label='titik terpilih')
    if touch:
        ax.plot(touch[0], touch[1], marker='X', markersize=11,
                color=TOUCHDOWN, markeredgecolor=SURFACE, markeredgewidth=1.2,
                linestyle='none', zorder=6, label='titik mendarat')

    # Galat pendaratan diberi label langsung, bukan disembunyikan di legenda:
    # inilah satu angka yang paling menentukan penilaian misi.
    err = to_float(summary, 'landing_error_m')
    if chosen and touch and err is not None:
        ax.annotate('galat %.3f m' % err,
                    xy=(touch[0], touch[1]),
                    xytext=(10, -16), textcoords='offset points',
                    color=INK, fontsize=9)

    ax.set_aspect('equal', adjustable='datalim')
    leg = ax.legend(loc='best', frameon=False, fontsize=9)
    for text in leg.get_texts():
        text.set_color(INK_MUTED)


def plot_altitude(ax, traj, transitions):
    """Ketinggian terhadap waktu, dengan batas antar state ditandai."""
    ax.set_title('Ketinggian terhadap waktu', color=INK, fontsize=11,
                 loc='left', pad=10)
    ax.set_xlabel('waktu sejak sampel pertama (s)')
    ax.set_ylabel('z map (m, atas)')

    stamps = [int(to_float(r, 'stamp_ns', 0)) for r in traj]
    ts = seconds_from(stamps)
    zs = [to_float(r, 'z') for r in traj]

    # Satu seri saja: judulnya sudah menamainya, jadi tidak perlu legenda.
    ax.plot(ts, zs, color=SEQ_BLUE[3], linewidth=2.0, zorder=3)

    # Batas state dipetakan dari t_s misi ke sumbu waktu lintasan. Keduanya
    # berawal pada nol yang berbeda -- t_s dihitung sejak node misi menyala,
    # sedangkan lintasan sejak sampel TF pertama -- jadi yang disejajarkan
    # adalah SELISIHNYA, bukan nilai mutlaknya.
    if transitions and ts:
        t_end = ts[-1]
        offs = [to_float(r, 't_s') for r in transitions if to_float(r, 't_s') is not None]
        if offs:
            shift = t_end - max(offs)
            # Transisi yang terjadi nyaris bersamaan (ARMING lalu TAKEOFF
            # berselang 0,1 detik) menumpuk labelnya jadi coretan yang tidak
            # terbaca. Garisnya tetap digambar semuanya -- yang dilewati hanya
            # teksnya, karena garis yang hilang akan menyembunyikan sebuah
            # peristiwa, sedangkan label yang hilang hanya menyembunyikan
            # namanya.
            min_gap = max(t_end * 0.03, 0.5)
            last_label_x = None
            for r in transitions:
                t = to_float(r, 't_s')
                if t is None:
                    continue
                x = t + shift
                if not (ts[0] <= x <= t_end):
                    continue
                ax.axvline(x, color=GRID, linewidth=1.0, zorder=1)

                if last_label_x is not None and x - last_label_x < min_gap:
                    continue
                last_label_x = x
                ax.annotate(r.get('to_state', ''), xy=(x, 1.0),
                            xycoords=('data', 'axes fraction'),
                            xytext=(3, -12), textcoords='offset points',
                            color=INK_MUTED, fontsize=8, rotation=90,
                            va='top')


def main():
    """Baca CSV satu run, gambar dua panel, tulis PNG."""
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    root, run_id = sys.argv[1], sys.argv[2]

    traj_path, traj = find(root, run_id, 'trajectory')
    _, cands = find(root, run_id, 'candidates')
    _, summaries = find(root, run_id, 'mission_summary')
    _, transitions = find(root, run_id, 'mission_transition')

    if not traj:
        print("Tidak ada data lintasan untuk run '%s' di %s.\n"
              "Apakah logger_stats berjalan (dan TF map -> base_link hidup) "
              "selama penerbangan itu?" % (run_id, root))
        return 1

    summary = summaries[0] if summaries else {}

    fig, (ax_top, ax_alt) = plt.subplots(
        2, 1, figsize=(9, 11), facecolor=SURFACE,
        gridspec_kw={'height_ratios': [3, 2], 'hspace': 0.3})

    outcome = summary.get('outcome', 'tidak diketahui')
    fig.suptitle('Penerbangan %s — %s' % (run_id, outcome),
                 color=INK, fontsize=13, x=0.09, ha='left')

    style(ax_top)
    style(ax_alt)
    plot_top_view(ax_top, traj, cands, summary)
    plot_altitude(ax_alt, traj, transitions)

    out = Path(traj_path).with_name(
        Path(traj_path).stem.replace('_trajectory', '_flight') + '.png')
    fig.savefig(out, dpi=150, facecolor=SURFACE, bbox_inches='tight')
    print('Tersimpan: %s' % out)
    print('  %d sampel lintasan, %d baris kandidat' % (len(traj), len(cands)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
