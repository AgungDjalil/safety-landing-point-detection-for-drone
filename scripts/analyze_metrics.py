#!/usr/bin/env python3
"""
Ringkas CSV metrik menjadi tabel perbandingan.

Pemakaian:
    python3 scripts/analyze_metrics.py ~/ros2_logs

Membaca seluruh berkas `<run>_<tanggal>_<jam>_<jenis>.csv` di direktori itu,
mengelompokkannya per `run_id`, lalu mencetak tiga tabel: segmentasi,
pencarian titik pendaratan, dan hasil misi.

Bukan paket ROS dan tidak ikut `colcon build`. Ini alat pelaporan di meja,
bukan kode yang terbang — tapi tetap di dalam repo, karena versi sebelumnya
hidup di direktori sementara dan terhapus bersama sesinya, membawa serta
satu-satunya cara membaca CSV ini menjadi tabel.

Hanya butuh pustaka standar; tidak ada pandas, supaya bisa dijalankan di mana
saja tanpa menyiapkan environment.
"""

import statistics as st
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from metrics_io import (airborne_window, group_runs, nums,  # noqa: E402
                        within)


def summarise(vals):
    """Ringkas jadi median / p90 / maks, atau None bila kosong."""
    if not vals:
        return None
    s = sorted(vals)
    p90 = s[min(len(s) - 1, int(len(s) * 0.9))]
    return (st.median(s), p90, s[-1])


def fmt(t, unit=''):
    """Satu sel tabel berisi median / p90 / maks."""
    if t is None:
        return '%-28s' % '(tidak ada data)'
    return '%8.2f %-6s %8.2f %8.2f' % (t[0], unit, t[1], t[2])


def rate_hz(rows):
    """Laju terbit dari selisih stamp_ns — waktu simulasi, bukan jam dinding."""
    stamps = sorted(int(float(r['stamp_ns'])) for r in rows
                    if r.get('stamp_ns'))
    stamps = [s for s in stamps if s > 0]
    if len(stamps) < 2:
        return None
    span_s = (stamps[-1] - stamps[0]) / 1e9
    return (len(stamps) - 1) / span_s if span_s > 0 else None


def table_segmentation(runs):
    """Waktu komputasi, latensi, laju, dan beban titik per backend."""
    print('\n== SEGMENTASI ==')
    print('%-10s %-8s %-28s %-28s %8s %12s %8s'
          % ('run', 'backend', 'comp_ms (med/p90/maks)',
             'latency_ms (med/p90/maks)', 'Hz', 'in->ds', 'plane'))
    for run, kinds in sorted(runs.items()):
        window = airborne_window(kinds)
        for backend in ('gng', 'ransac'):
            all_rows = kinds.get(backend)
            if not all_rows:
                continue
            rows = within(all_rows, window)
            if not rows:
                continue
            dropped = len(all_rows) - len(rows)
            hz = rate_hz(rows)
            ds = summarise(nums(rows, 'downsampled_points'))
            inp = summarise(nums(rows, 'input_points'))
            pl = summarise(nums(rows, 'plane_size'))
            print('%-10s %-8s %-28s %-28s %8s %12s %8s'
                  % (run, backend,
                     fmt(summarise(nums(rows, 'computation_time_ms'))),
                     fmt(summarise(nums(rows, 'latency_ms'))),
                     '%.2f' % hz if hz else '-',
                     '%d->%d' % (inp[0], ds[0]) if (inp and ds) else '-',
                     '%d' % pl[0] if pl else '-'))
            if dropped:
                print('%-19s (%d frame di luar jendela udara diabaikan)'
                      % ('', dropped))
    print('\n  Hanya frame ANTARA lepas landas dan mendarat yang dihitung.')
    print('  Frame saat drone di tanah nyaris kosong dan akan menarik')
    print('  mediannya ke bawah tanpa mengatakan apa pun tentang algoritmanya.')
    print('  Kolom in->ds adalah median titik masuk -> titik sesudah voxel.')
    print('  latency_ms kosong berarti node berjalan tanpa use_sim_time —')
    print('  angkanya sengaja tidak diterbitkan daripada salah.')


def table_landing(runs):
    """Jumlah kandidat, sebaran roughness dan skor, pemicuan penjaga."""
    print('\n== PENCARI TITIK PENDARATAN ==')
    for run, kinds in sorted(runs.items()):
        window = airborne_window(kinds)
        rows = within(kinds.get('landing', []), window)
        if not rows:
            continue
        cands = within(kinds.get('candidates', []), window)
        committed = [r for r in rows if r.get('committed') == '1']

        print('\n  run: %s' % run)
        print('    frame                : %d (%d sesudah commit)'
              % (len(rows), len(committed)))
        print('    comp_ms  med/p90/maks: %s'
              % fmt(summarise(nums(rows, 'computation_time_ms'))))
        print('    kandidat med/p90/maks: %s'
              % fmt(summarise(nums(rows, 'num_candidates'))))
        blocked = nums(rows, 'blocked_candidates')
        print('    terhalang     (median): %s'
              % ('%.0f' % st.median(blocked) if blocked else '-'))

        rough = [v for v in nums(cands, 'roughness_m') if v > 0]
        if rough:
            s = sorted(rough)
            print('    roughness_m          : min %.4f  median %.4f  '
                  'maks %.4f  (%.1fx)'
                  % (s[0], st.median(s), s[-1], s[-1] / max(1e-9, s[0])))
        scores = nums(cands, 'score')
        if scores:
            print('    score                : min %.3f  median %.3f  maks %.3f'
                  % (min(scores), st.median(scores), max(scores)))

        if cands:
            unmeasured = sum(1 for r in cands if r.get('rough_n') == '0')
            print('    roughness tak terukur: %d dari %d baris (%.1f%%)'
                  % (unmeasured, len(cands), 100.0 * unmeasured / len(cands)))

        obs = nums(rows, 'obstacle_points')
        if obs:
            print('    obstacle_points maks : %.0f' % max(obs))

        traj = within(kinds.get('trajectory', []), window)
        if traj:
            print('    sampel lintasan      : %d' % len(traj))


def table_mission(runs):
    """Galat pendaratan, waktu keputusan, abort, durasi tiap state."""
    print('\n== HASIL MISI ==')
    any_row = False
    for run, kinds in sorted(runs.items()):
        rows = kinds.get('mission_summary', [])
        for r in rows:
            any_row = True
            print('\n  run: %s   outcome: %s' % (run, r.get('outcome', '?')))
            for label, key, unit in (
                    ('galat pendaratan', 'landing_error_m', 'm'),
                    ('waktu ke commit', 'time_to_commit_s', 's'),
                    ('waktu ke waypoint', 'time_to_arrive_s', 's'),
                    ('total', 'total_time_s', 's'),
                    ('percobaan scan', 'scan_attempts', ''),
                    ('abort', 'abort_count', '')):
                v = r.get(key, '')
                print('    %-20s %s %s' % (label, v if v != '' else '-', unit))
            durs = {k[len('state_durations_s_'):]: v
                    for k, v in r.items()
                    if k.startswith('state_durations_s_')
                    and v not in ('', None)}
            if durs:
                print('    durasi state         %s'
                      % '  '.join('%s=%ss' % (k, v) for k, v in durs.items()))
    if not any_row:
        print('  (tidak ada peristiwa summary — apakah /mission_events '
              'terekam?)')


def main():
    """Cetak ketiga tabel untuk seluruh run di satu direktori."""
    root = sys.argv[1] if len(sys.argv) > 1 else str(Path.home() / 'ros2_logs')
    runs = group_runs(root)
    if not runs:
        print('Tidak ada CSV berisi data di %s' % root)
        return 1

    print('Direktori: %s' % root)
    print('Run ditemukan: %s' % ', '.join(sorted(runs)))
    table_segmentation(runs)
    table_landing(runs)
    table_mission(runs)
    return 0


if __name__ == '__main__':
    sys.exit(main())
