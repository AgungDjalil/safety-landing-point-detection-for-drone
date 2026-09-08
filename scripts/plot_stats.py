#!/usr/bin/env python3
r"""
Plot kolom apa pun dari CSV metrik penerbangan.

    # apa saja yang tersedia
    python3 scripts/plot_stats.py ~/ros2_logs --list
    python3 scripts/plot_stats.py ~/ros2_logs --list landing

    # satu variabel
    python3 scripts/plot_stats.py ~/ros2_logs landing computation_time_ms

    # beberapa variabel -> beberapa panel bertumpuk
    python3 scripts/plot_stats.py ~/ros2_logs landing \\
        computation_time_ms num_candidates selected_score

    # sebaran
    python3 scripts/plot_stats.py ~/ros2_logs candidates roughness_m --hist

    # satu besaran terhadap besaran lain
    python3 scripts/plot_stats.py ~/ros2_logs landing obstacle_points \\
        --x target_alt_m

    # bandingkan dua penerbangan
    python3 scripts/plot_stats.py ~/ros2_logs gng computation_time_ms \\
        --runs 074807 081836

Dua aturan gambar yang tidak bisa dimatikan lewat argumen, karena keduanya
soal benar atau salah, bukan selera:

  BEBERAPA VARIABEL JADI BEBERAPA PANEL, bukan dua sumbu y. Waktu komputasi
  (puluhan milidetik) dan jumlah kandidat (puluhan buah) tidak sesatuan;
  menumpuknya di satu sumbu ganda membuat bentuk kurvanya bisa diatur
  mengatakan apa saja, tergantung skala mana yang dipilih.

  WARNA HANYA MEMBEDAKAN RUN. Slot dipakai berurutan dan tidak pernah didaur;
  penerbangan kesembilan bukan warna baru, melainkan tanda bahwa grafiknya
  perlu dipecah.
"""

import argparse
import bisect
import statistics as st
import sys
from pathlib import Path

import warnings

# Peringatan ini muncul karena matplotlib terpasang dua kali di mesin ini (apt
# dan pip), dan proyeksi 3D-nya tidak tersedia. Seluruh gambar di sini 2D, jadi
# ia tidak berarti apa-apa -- tapi ia dicetak sebelum baris hasil dan terbaca
# seperti kegagalan. Menyembunyikannya di sini lebih jujur daripada membiarkan
# tiap pemakaian tampak error.
warnings.filterwarnings('ignore', message='.*Axes3D.*')

import matplotlib                          # noqa: E402
matplotlib.use('Agg')                      # tanpa display; tulis PNG saja
import matplotlib.pyplot as plt            # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from metrics_io import (CATEGORICAL, GRID, INK, INK_MUTED, SURFACE,  # noqa: E402
                        airborne_window, group_runs, numeric_columns,
                        seconds_from, style, to_float, within)


def parse_args(argv):
    """Argumen baris perintah; `kind` dan kolom bersifat posisional."""
    p = argparse.ArgumentParser(
        description='Plot kolom apa pun dari CSV metrik penerbangan.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    p.add_argument('log_dir', help='direktori CSV, mis. ~/ros2_logs')
    p.add_argument('kind', nargs='?',
                   help='jenis berkas: gng, ransac, landing, candidates, '
                        'trajectory, center, mission_transition, ...')
    p.add_argument('columns', nargs='*', help='satu atau lebih nama kolom')
    # nargs='?' supaya `--list` sendirian mendaftar run, dan `--list landing`
    # mendaftar kolomnya. Sebagai store_true, `landing` sesudahnya tidak
    # terserap ke posisional `kind` dan argparse menolaknya.
    p.add_argument('--list', dest='do_list', nargs='?', const='',
                   default=None, metavar='JENIS',
                   help='daftar run, atau kolom bila jenis diberi')
    p.add_argument('--runs', nargs='+', default=None,
                   help='saring run lewat substring, mis. 081836')
    p.add_argument('--x', default='time',
                   help="'time' (default), 'stamp_ns', atau nama kolom")
    p.add_argument('--x-kind', dest='x_kind', default=None, metavar='JENIS',
                   help='berkas asal --x bila beda dari yang di sumbu y; '
                        'barisnya dijodohkan menurut waktu')
    p.add_argument('--right', default=None, metavar='KOLOM',
                   help='kolom kedua pada sumbu y KANAN (skala terpisah)')
    p.add_argument('--right-kind', dest='right_kind', default=None,
                   metavar='JENIS',
                   help='berkas asal --right bila beda dari yang kiri')
    p.add_argument('--trend', nargs='?', const=20, type=int, default=None,
                   metavar='N',
                   help='garis tren: bin sumbu x jadi N bagian (default 20), '
                        'gambar median per bin plus pita p10-p90')
    p.add_argument('--hist', action='store_true', help='gambar sebaran')
    p.add_argument('--bins', type=int, default=30, help='bin histogram')
    p.add_argument('--events', action='store_true',
                   help='tandai transisi state pada sumbu waktu')
    p.add_argument('--all-frames', dest='all_frames', action='store_true',
                   help='jangan batasi ke jendela udara')
    p.add_argument('--out', default=None, help='berkas PNG keluaran')
    return p.parse_args(argv)


def matches(run_key, f):
    """
    Apakah satu filter menunjuk run ini, dengan aturan yang bisa ditebak.

    Cocok bila filter sama persis dengan kunci penuh, sama persis dengan
    run_id-nya (bagian sebelum tanggal), atau merupakan bagian dari stempel
    waktunya.

    Pencocokan substring polos ditinggalkan karena menyesatkan: `--runs freeze`
    ikut memilih `nofreeze_...`, dan pesan galatnya hanya menyebut "2 terpilih"
    tanpa memberi tahu yang mana -- penyebabnya jadi tidak kelihatan sama
    sekali.
    """
    if f == run_key:
        return True
    parts = run_key.split('_')
    if len(parts) >= 3:
        run_id = '_'.join(parts[:-2])
        stamp = '_'.join(parts[-2:])
        if f == run_id or f in stamp:
            return True
    return False


def select_runs(runs, filters):
    """Run yang ditunjuk salah satu filter; semuanya bila tidak ada filter."""
    if not filters:
        return dict(runs)

    out = {k: v for k, v in runs.items()
           if any(matches(k, f) for f in filters)}
    if out:
        return out

    # Tidak ada yang cocok persis: jatuh ke substring, tapi KATAKAN. Diam-diam
    # melebar adalah bagaimana `freeze` berubah menjadi dua run tanpa ada yang
    # menyadarinya.
    loose = {k: v for k, v in runs.items()
             if any(f in k for f in filters)}
    if loose:
        print('Tidak ada run yang cocok persis; memakai pencocokan longgar: %s'
              % ', '.join(sorted(loose)))
    return loose


def do_list(runs, kind):
    """Cetak isi direktori: run dan jenis berkas, atau kolom satu jenis."""
    if not runs:
        print('Tidak ada CSV metrik di direktori itu.')
        return 1

    if kind is None:
        print('Run yang ditemukan:\n')
        for run_key in sorted(runs):
            kinds = runs[run_key]
            print('  %s' % run_key)
            for k in sorted(kinds):
                print('      %-22s %6d baris' % (k, len(kinds[k])))
        print('\nUntuk melihat kolomnya:')
        print('  plot_stats.py <dir> --list <jenis>')
        return 0

    for run_key in sorted(runs):
        rows = runs[run_key].get(kind)
        if not rows:
            continue
        cols = numeric_columns(rows)
        print('Kolom numerik di %s (%s, %d baris):\n'
              % (kind, run_key, len(rows)))
        for c in cols:
            print('  %s' % c)
        return 0

    print("Tidak ada berkas '%s'. Yang ada: %s"
          % (kind, ', '.join(sorted({k for r in runs.values() for k in r}))))
    return 1


def is_long_format(rows):
    """
    Kenali format panjang: satu stempel memuat banyak baris.

    Menyambungkan titik-titik yang berbagi satu waktu menghasilkan zig-zag
    vertikal yang tidak berarti apa pun, jadi bentuk datanya menentukan
    bentuk marknya — bukan sebaliknya.
    """
    stamps = [r.get('stamp_ns') for r in rows[:400] if r.get('stamp_ns')]
    return bool(stamps) and len(set(stamps)) < len(stamps) * 0.8


def time_origin(kinds, window, rows):
    """
    Nanosekon yang dianggap t=0 untuk satu run.

    Lepas landas bila diketahui, sehingga sumbunya terbaca "detik sejak lepas
    landas" dan dua penerbangan bisa ditumpuk dengan arti yang sama.

    Origin ini dipakai deret data DAN penanda peristiwa. Memakai `stamp_ns`
    untuk deret dan `recv_ns` untuk peristiwa — seperti versi pertama skrip
    ini — menggeser garis TAKEOFF ke t=0 padahal `landing.csv` baru mulai saat
    drone TIBA, delapan belas detik kemudian. Garisnya tergambar rapi di tempat
    yang salah, dan tidak ada yang terlihat rusak.
    """
    if window is not None:
        return window[0]
    vals = [to_float(r, 'recv_ns') for r in rows]
    vals = [v for v in vals if v is not None]
    return min(vals) if vals else None


def nearest_lookup(rows, key, max_gap_s=0.5):
    """
    Fungsi yang mencari nilai `key` paling dekat menurut waktu, atau None.

    Dipakai saat sumbu x diambil dari berkas lain: `computation_time_ms` ada di
    gng.csv sedangkan ketinggian ada di trajectory.csv, dan keduanya disampel
    pada laju yang berbeda. Yang dijodohkan `recv_ns`, satu-satunya jam yang
    dicap perekam yang sama untuk semua topik.

    Pasangan yang terpaut lebih dari `max_gap_s` DITOLAK, bukan dipaksakan.
    Menjodohkan sampel yang berjarak beberapa detik akan menghasilkan titik
    yang terlihat rapi tapi memasangkan ketinggian dengan frame yang bukan
    miliknya.
    """
    pts = []
    for r in rows:
        t = to_float(r, 'recv_ns')
        v = to_float(r, key)
        if t is not None and v is not None:
            pts.append((t, v))
    if not pts:
        return None
    pts.sort()
    times = [p[0] for p in pts]
    vals = [p[1] for p in pts]
    gap_ns = max_gap_s * 1e9

    def look(t):
        i = bisect.bisect_left(times, t)
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(times):
                d = abs(times[j] - t)
                if d <= gap_ns and (best is None or d < best[0]):
                    best = (d, vals[j])
        return best[1] if best else None

    return look


def binned_trend(pts, nbins=20, min_per_bin=3):
    """
    (pusat, p10, median, p90) per bin, untuk daftar pasangan (x, y).

    Menyambungkan titik mentah apa adanya bukan jawaban: banyak frame berbagi
    nilai x yang berdekatan, sehingga garisnya menjadi zig-zag vertikal yang
    tidak berarti apa pun. Agregat per bin yang menjawab "bagaimana y menskala
    terhadap x".

    Pita p10-p90 ikut dikembalikan karena median telanjang akan menyatakan
    ketepatan yang tidak dimiliki datanya. Terukur pada data GNG: di bin
    6.476-7.546 titik, p10 dan p90 berjarak 72 sampai 283 ms.

    Bin dengan kurang dari `min_per_bin` sampel DILEWATI. Satu frame pencilan
    tidak boleh menjadi sebuah tren.
    """
    if not pts:
        return ([], [], [], [])

    xs = [p[0] for p in pts]
    lo, hi = min(xs), max(xs)
    if hi <= lo:
        return ([], [], [], [])

    buckets = [[] for _ in range(max(1, nbins))]
    width = (hi - lo) / len(buckets)
    for x, y in pts:
        i = min(len(buckets) - 1, int((x - lo) / width))
        buckets[i].append(y)

    cx, p10, med, p90 = [], [], [], []
    for i, vals in enumerate(buckets):
        if len(vals) < min_per_bin:
            continue
        vals.sort()
        cx.append(lo + width * (i + 0.5))
        p10.append(vals[int(len(vals) * 0.1)])
        med.append(st.median(vals))
        p90.append(vals[min(len(vals) - 1, int(len(vals) * 0.9))])
    return (cx, p10, med, p90)


def series_for(rows, x_key, y_key, origin_ns=None, x_lookup=None):
    """Pasangan (x, y) yang keduanya sah, terurut menurut x."""
    pts = []
    for r in rows:
        y = to_float(r, y_key)
        if y is None:
            continue
        if x_key == 'time':
            # `recv_ns` ada di SETIAP baris setiap berkas, termasuk peristiwa
            # misi yang tidak punya `stamp_ns` sama sekali. Itulah satu-satunya
            # jam yang bisa menyejajarkan semuanya.
            x = to_float(r, 'recv_ns')
        elif x_lookup is not None:
            t = to_float(r, 'recv_ns')
            x = x_lookup(t) if t is not None else None
        else:
            x = to_float(r, x_key)
        if x is None:
            continue
        pts.append((x, y))
    pts.sort()
    if x_key == 'time':
        if origin_ns is None:
            xs = seconds_from([p[0] for p in pts])
        else:
            xs = [(p[0] - origin_ns) / 1e9 for p in pts]
    else:
        xs = [p[0] for p in pts]
    return xs, [p[1] for p in pts]


def mark_events(ax, kinds, x_key, origin_ns):
    """Garis vertikal di tiap transisi state, memakai origin waktu run itu."""
    if x_key != 'time' or origin_ns is None:
        return
    # TIDAK disaring jendela udara: batas jendela itu sendiri adalah dua
    # peristiwa yang paling ingin dilihat.
    tr = kinds.get('mission_transition', [])
    if not tr:
        return
    base = origin_ns

    lo, hi = ax.get_xlim()
    min_gap = max((hi - lo) * 0.04, 0.5)
    last = None
    for r in tr:
        v = to_float(r, 'recv_ns')
        if v is None:
            continue
        x = (v - base) / 1e9
        if not (lo <= x <= hi):
            continue
        ax.axvline(x, color=GRID, linewidth=1.0, zorder=1)
        # Garisnya selalu digambar; hanya labelnya yang dilewati saat
        # berdempetan. Garis yang hilang menyembunyikan sebuah peristiwa,
        # label yang hilang hanya menyembunyikan namanya.
        if last is not None and x - last < min_gap:
            continue
        last = x
        ax.annotate(r.get('to_state', ''), xy=(x, 1.0),
                    xycoords=('data', 'axes fraction'),
                    xytext=(3, -12), textcoords='offset points',
                    color=INK_MUTED, fontsize=8, rotation=90, va='top')


def shared_bins(chosen, kind, col, args):
    """
    Batas bin yang sama untuk seluruh run, atau None bila tak ada data.

    matplotlib menghitung bin per pemanggilan, jadi dua run yang digambar
    bertumpuk mendapat batas yang berbeda dan batangnya bergeser satu sama
    lain. Yang terlihat lalu bukan perbedaan sebarannya, melainkan perbedaan
    binning-nya.
    """
    vals = []
    for kinds in chosen.values():
        rows = kinds.get(kind)
        if not rows:
            continue
        window = None if args.all_frames else airborne_window(kinds)
        for r in within(rows, window):
            v = to_float(r, col)
            if v is not None:
                vals.append(v)
    if not vals:
        return None
    lo, hi = min(vals), max(vals)
    if hi <= lo:
        lo, hi = lo - 0.5, hi + 0.5
    step = (hi - lo) / args.bins
    return [lo + i * step for i in range(args.bins + 1)]


def add_right_axis(ax, chosen, args, origins, is_bottom):
    """
    Sumbu y kedua di kanan, dengan skalanya sendiri.

    SUMBU GANDA. Kedua skala berdiri sendiri, jadi di mana kedua kurva
    berpotongan atau seberapa jauh jaraknya TIDAK berarti apa-apa — geser
    salah satu batas sumbu dan hubungan yang tampak ikut berubah. Yang tetap
    sahih hanyalah bentuk masing-masing kurva terhadap waktu.

    Pengamannya: tiap kurva diwarnai sama dengan sumbunya, dan yang kanan
    digaris putus-putus supaya kepemilikannya tidak bergantung pada warna
    saja — pembaca yang buta warna tetap bisa memasangkannya.
    """
    if not args.right or args.hist or len(origins) != 1:
        return None

    run_key = next(iter(origins))
    kinds = chosen[run_key]
    kind = args.right_kind or args.kind
    rows = kinds.get(kind)
    if not rows:
        return None

    window = None if args.all_frames else airborne_window(kinds)
    rows = within(rows, window)
    xs, ys = series_for(rows, args.x, args.right, origins[run_key])
    if not xs:
        return None

    color = CATEGORICAL[1]
    ax2 = ax.twinx()
    ax2.plot(xs, ys, color=color, linewidth=2.0, linestyle='--', zorder=2,
             label=args.right)
    ax2.set_ylabel(args.right, color=color)
    ax2.tick_params(axis='y', colors=color, labelsize=9)
    ax2.spines['top'].set_visible(False)
    ax2.spines['left'].set_visible(False)
    ax2.spines['right'].set_color(color)
    # Satu grid saja: dua kisi yang tidak sejajar membuat keduanya sulit
    # dibaca dan menyarankan kesejajaran yang tidak ada.
    ax2.grid(False)
    return ax2


def draw(ax, chosen, kind, col, args, is_bottom, show_legend):
    """Satu panel: satu kolom, satu garis/sebar per run."""
    style(ax)
    ax.set_ylabel(col)

    bins = shared_bins(chosen, kind, col, args) if args.hist else None
    drawn = 0
    origins = {}
    for i, (run_key, kinds) in enumerate(sorted(chosen.items())):
        rows = kinds.get(kind)
        if not rows:
            continue
        window = None if args.all_frames else airborne_window(kinds)
        rows = within(rows, window)
        if not rows:
            continue
        origins[run_key] = time_origin(kinds, window, rows)

        # Sumbu x dari berkas lain: jodohkan barisnya menurut waktu.
        x_lookup = None
        if args.x_kind and args.x not in ('time', 'stamp_ns'):
            x_rows = within(kinds.get(args.x_kind, []), window)
            x_lookup = nearest_lookup(x_rows, args.x)

        color = CATEGORICAL[i % len(CATEGORICAL)]
        label = run_key

        if args.hist:
            vals = [to_float(r, col) for r in rows]
            vals = [v for v in vals if v is not None]
            if not vals:
                continue
            ax.hist(vals, bins=bins or args.bins, color=color, alpha=0.7,
                    edgecolor=SURFACE, linewidth=0.6, label=label, zorder=3)
            ax.set_xlabel(col)
            ax.set_ylabel('jumlah')
        else:
            xs, ys = series_for(rows, args.x, col, origins[run_key],
                                x_lookup=x_lookup)
            if args.x_kind and len(xs) < len(rows):
                print('%s: %d dari %d baris terjodohkan dengan %s '
                      '(sisanya tidak punya pasangan dalam 0,5 detik).'
                      % (run_key, len(xs), len(rows), args.x_kind))
            if not xs:
                continue
            if args.trend:
                cx, p10, med, p90 = binned_trend(list(zip(xs, ys)),
                                                 nbins=args.trend)
                if not cx:
                    continue
                # Pita lebih dulu, garis di atasnya: pitanya konteks, garisnya
                # yang dibaca.
                ax.fill_between(cx, p10, p90, color=color, alpha=0.18,
                                linewidth=0, zorder=2)
                ax.plot(cx, med, color=color, linewidth=2.0, label=label,
                        zorder=4)
                skipped = args.trend - len(cx)
                print('%s: %d bin terpakai%s (median + pita p10-p90).'
                      % (run_key, len(cx),
                         ', %d dilewati karena <3 sampel' % skipped
                         if skipped else ''))
            elif is_long_format(rows) or args.x not in ('time', 'stamp_ns'):
                ax.plot(xs, ys, linestyle='none', marker='o', markersize=4,
                        color=color, alpha=0.45, label=label, zorder=3)
            else:
                ax.plot(xs, ys, color=color, linewidth=2.0, label=label,
                        zorder=3)
            if is_bottom:
                if args.x == 'time':
                    ax.set_xlabel('detik sejak lepas landas')
                elif args.x_kind:
                    ax.set_xlabel('%s  (dari %s)' % (args.x, args.x_kind))
                else:
                    ax.set_xlabel(args.x)
        drawn += 1

    if drawn == 0:
        ax.text(0.5, 0.5, "tidak ada nilai untuk '%s'" % col,
                transform=ax.transAxes, ha='center', color=INK_MUTED)
        return

    ax2 = add_right_axis(ax, chosen, args, origins, is_bottom)
    if ax2 is not None:
        # Label sumbu kiri ikut diwarnai supaya pasangan kurva-sumbu terbaca
        # tanpa harus menebak.
        ax.set_ylabel(col, color=CATEGORICAL[0])
        ax.tick_params(axis='y', colors=CATEGORICAL[0])

    # Legenda sekali saja, di panel teratas: seluruh panel memakai run yang
    # sama, jadi mengulanginya tiap panel hanya menutupi data.
    if show_legend and (drawn > 1 or ax2 is not None):
        handles, labels = ax.get_legend_handles_labels()
        if ax2 is not None:
            h2, l2 = ax2.get_legend_handles_labels()
            handles, labels = handles + h2, labels + l2
            if drawn == 1:
                labels[0] = col          # satu run: namai kurvanya, bukan run
        leg = ax.legend(handles, labels, loc='best', frameon=False, fontsize=9)
        for t in leg.get_texts():
            t.set_color(INK_MUTED)

    # Penanda peristiwa hanya bila SATU run digambar. Dua penerbangan punya
    # garis waktu masing-masing; menggambar milik salah satunya di atas
    # keduanya memberi tanggal yang salah pada separuh data.
    if args.events and not args.hist and len(origins) == 1:
        run_key = next(iter(origins))
        mark_events(ax, chosen[run_key], args.x, origins[run_key])


def report_window(chosen, kind, all_frames):
    """Beri tahu berapa frame dibuang, supaya penyaringannya tidak diam-diam."""
    if all_frames:
        print('Jendela udara DIMATIKAN: seluruh rekaman ikut digambar.')
        return
    for run_key, kinds in sorted(chosen.items()):
        rows = kinds.get(kind)
        if not rows:
            continue
        window = airborne_window(kinds)
        if window is None:
            print('%s: tidak ada transisi misi, jendela udara dilewati.'
                  % run_key)
            continue
        kept = len(within(rows, window))
        print('%s: %d dari %d frame di dalam jendela udara.'
              % (run_key, kept, len(rows)))


def main(argv=None):
    """Baca argumen, gambar tiap kolom sebagai panelnya sendiri, tulis PNG."""
    args = parse_args(sys.argv[1:] if argv is None else argv)
    root = Path(args.log_dir).expanduser()
    runs = group_runs(root)

    if args.do_list is not None:
        kind = args.do_list or args.kind
        return do_list(select_runs(runs, args.runs), kind or None)

    if not args.kind or not args.columns:
        print("Sebutkan jenis berkas dan minimal satu kolom.\n"
              "Contoh: plot_stats.py %s landing computation_time_ms\n"
              "Lihat yang tersedia: plot_stats.py %s --list"
              % (args.log_dir, args.log_dir))
        return 2

    chosen = select_runs(runs, args.runs)
    if not chosen:
        print('Tidak ada run yang cocok dengan %s. Yang ada: %s'
              % (args.runs, ', '.join(sorted(runs)) or '(kosong)'))
        return 1

    sample = None
    for kinds in chosen.values():
        if kinds.get(args.kind):
            sample = kinds[args.kind]
            break
    if sample is None:
        print("Tidak ada berkas '%s' pada run terpilih. Yang ada: %s"
              % (args.kind,
                 ', '.join(sorted({k for r in chosen.values() for k in r}))))
        return 1

    # Kolom yang bukan angka ditolak dengan daftar yang sah, bukan dengan
    # traceback beberapa detik kemudian.
    valid = numeric_columns(sample)
    bad = [c for c in args.columns if c not in valid]
    if bad:
        print('Bukan kolom numerik di %s: %s\n\nYang bisa diplot:\n  %s'
              % (args.kind, ', '.join(bad), '\n  '.join(valid)))
        return 1
    if args.x_kind:
        x_sample = None
        for kinds in chosen.values():
            if kinds.get(args.x_kind):
                x_sample = kinds[args.x_kind]
                break
        if x_sample is None:
            print("Tidak ada berkas '%s' untuk --x-kind. Yang ada: %s"
                  % (args.x_kind,
                     ', '.join(sorted({k for r in chosen.values() for k in r}))))
            return 1
        x_valid = numeric_columns(x_sample)
        if args.x not in x_valid:
            print('--x %r bukan kolom numerik di %s.\n\nYang bisa dipakai:\n'
                  '  %s' % (args.x, args.x_kind, '\n  '.join(x_valid)))
            return 1
    elif args.x not in ('time', 'stamp_ns') and args.x not in valid:
        print('Sumbu x %r bukan kolom numerik di %s.\n\nYang bisa dipakai:\n'
              "  time, stamp_ns\n  %s" % (args.x, args.kind,
                                          '\n  '.join(valid)))
        return 1

    if args.right:
        right_kind = args.right_kind or args.kind
        r_sample = None
        for kinds in chosen.values():
            if kinds.get(right_kind):
                r_sample = kinds[right_kind]
                break
        if r_sample is None:
            print("Tidak ada berkas '%s' untuk --right." % right_kind)
            return 1
        r_valid = numeric_columns(r_sample)
        if args.right not in r_valid:
            print('--right %r bukan kolom numerik di %s.\n\nYang bisa '
                  'dipakai:\n  %s'
                  % (args.right, right_kind, '\n  '.join(r_valid)))
            return 1
        if len(chosen) != 1:
            print('--right butuh tepat satu run, tapi %d terpilih:\n  %s\n'
                  'Persempit dengan --runs, mis. stempel waktunya.'
                  % (len(chosen), '\n  '.join(sorted(chosen))))
            return 1
        if args.hist:
            print('--right tidak berlaku untuk histogram.')
            return 1

        # Dikatakan sekali, jelas, tiap kali dipakai. Sumbu ganda adalah
        # bentuk grafik yang paling mudah menipu pembacanya sendiri, dan
        # peringatan yang hanya ada di README tidak menolong siapa pun yang
        # sedang menatap gambarnya.
        print('CATATAN: sumbu kiri dan kanan berskala sendiri-sendiri. Di '
              'mana kedua kurva berpotongan\n'
              '         atau seberapa jauh jaraknya tidak berarti apa-apa; '
              'yang sahih hanya bentuk\n'
              '         masing-masing kurva terhadap sumbu x.')

    if args.trend and args.x in ('time', 'stamp_ns'):
        print('--trend hanya berlaku saat sumbu x sebuah KOLOM, bukan waktu.\n'
              'Deret waktu sudah berupa garis; membin ulangnya hanya '
              'menghaluskan\nsesuatu yang sudah benar. Pakai mis. '
              '--x downsampled_points.')
        return 1
    if args.trend and args.hist:
        print('--trend dan --hist saling menggantikan; pilih salah satu.')
        return 1

    report_window(chosen, args.kind, args.all_frames)

    n = len(args.columns)
    fig, axes = plt.subplots(
        n, 1, figsize=(10, 3.1 * n + 1.2), facecolor=SURFACE, squeeze=False,
        sharex=not args.hist,
        gridspec_kw={'hspace': 0.45 if args.hist else 0.25})

    if args.hist:
        what = 'sebaran'
    elif args.x == 'time':
        what = 'deret waktu'
    else:
        what = 'terhadap %s' % args.x
    runs_txt = ('%d penerbangan' % len(chosen)) if len(chosen) > 1 \
        else next(iter(chosen))
    fig.suptitle('%s — %s  (%s)' % (args.kind, what, runs_txt),
                 color=INK, fontsize=13, x=0.08, ha='left')

    col_axes = list(axes[:, 0])

    # Legenda hanya dipasang bila ia benar-benar menerangkan seluruh gambar.
    #
    # Dengan beberapa panel dan sumbu kanan, legenda di panel teratas akan
    # menamai garis kirinya `computation_time_ms` -- padahal garis kiri di
    # panel bawahnya adalah variabel lain dengan warna yang sama. Label sumbu
    # yang sudah diwarnai senada sudah menerangkan tiap panel dengan benar,
    # dan ia tidak pernah salah karena ada di panelnya sendiri.
    legend_panel = 0 if (len(chosen) > 1 or len(args.columns) == 1) else -1
    for i, (ax, col) in enumerate(zip(col_axes, args.columns)):
        draw(ax, chosen, args.kind, col, args,
             is_bottom=(i == len(col_axes) - 1),
             show_legend=(i == legend_panel))

    if args.events and len(chosen) > 1:
        print('Penanda peristiwa dilewati: hanya digambar bila satu run '
              'dipilih (pakai --runs).')

    # Nama berkas menyebutkan setiap pilihan yang membentuk gambarnya, supaya
    # dua PNG dari perintah berbeda tidak saling menimpa.
    parts = [args.kind, '_'.join(args.columns)]
    if args.x not in ('time', 'stamp_ns'):
        parts.append('over_%s' % args.x)
    if args.right:
        parts.append('vs_%s' % args.right)
    if args.trend:
        parts.append('trend')
    if args.hist:
        parts.append('hist')
    out = (Path(args.out).expanduser() if args.out
           else root / ('%s.png' % '_'.join(parts)))
    fig.savefig(out, dpi=150, facecolor=SURFACE, bbox_inches='tight')
    print('Tersimpan: %s' % out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
