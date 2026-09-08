"""
Unit test untuk pembacaan CSV metrik.

    python3 -m pytest scripts/

`scripts/` bukan paket ROS, jadi colcon tidak menyentuhnya — tapi kesalahan di
sini tidak menghasilkan error, melainkan grafik dan tabel yang keliru tanpa
ada yang tahu. Dua kegagalan senyap sudah terjadi di jalur ini: parser regex
yang berhenti menemukan kandidat, dan kunci run yang membuat satu penerbangan
menimpa penerbangan lain. Keduanya diam sampai seseorang kebetulan
memeriksanya.
"""

import csv
import json

import metrics_io as mio


def write_csv(path, rows, header=None):
    """Tulis CSV bergaya perekam, lengkap dengan kolom `extra`."""
    header = header or (list(rows[0]) + ['extra'])
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=header)
        w.writeheader()
        for r in rows:
            row = dict(r)
            row.setdefault('extra', '')
            w.writerow(row)


# ── split_name ──────────────────────────────────────────────────────────────

def test_split_name_handles_an_underscore_inside_the_kind():
    """`mission_summary` mengandung garis bawah dan tidak boleh terpotong."""
    assert mio.split_name('gng_20260828_081836_mission_summary.csv') == (
        'gng_20260828_081836', 'mission_summary')


def test_split_name_handles_an_underscore_inside_the_run_id():
    """run_id boleh memuat garis bawah; pemisahnya tanggal+jam."""
    assert mio.split_name('uji_coba_20260828_081836_landing.csv') == (
        'uji_coba_20260828_081836', 'landing')


def test_split_name_keeps_the_timestamp_in_the_run_key():
    """
    Kunci run memuat stempel waktunya, bukan hanya run_id.

    Ini regresi langsung: dengan kunci `run_id` saja, dua penerbangan
    ber-run_id sama saling menimpa dan yang lebih lama hilang dari tabel tanpa
    satu pun peringatan.
    """
    a, _ = mio.split_name('gng_20260828_074807_landing.csv')
    b, _ = mio.split_name('gng_20260828_081836_landing.csv')
    assert a != b


def test_split_name_rejects_a_foreign_file():
    """Berkas tanpa pasangan tanggal+jam bukan milik perekam ini."""
    assert mio.split_name('catatan.csv') == (None, None)
    assert mio.split_name('20260828_081836_landing.csv') == (None, None)


# ── load ────────────────────────────────────────────────────────────────────

def test_load_merges_the_extra_column_back_into_the_row(tmp_path):
    """
    Field di kolom `extra` harus kembali ke barisnya.

    `extra` menampung field yang muncul setelah header terbentuk. Mengabaikan
    kolom itu akan membuang justru data terbaru — termasuk seluruh isi
    ringkasan misi.
    """
    p = tmp_path / 'gng_20260828_081836_landing.csv'
    write_csv(p, [{'run_id': 'gng', 'recv_ns': '10',
                   'extra': json.dumps({'landing_error_m': 0.061})}],
              header=['run_id', 'recv_ns', 'extra'])

    row = mio.load(p)[0]
    assert row['landing_error_m'] == 0.061
    assert 'extra' not in row


def test_load_survives_a_corrupt_extra_cell(tmp_path):
    """Satu sel rusak tidak boleh menjatuhkan seluruh pembacaan."""
    p = tmp_path / 'gng_20260828_081836_landing.csv'
    write_csv(p, [{'run_id': 'gng', 'extra': '{bukan json'}],
              header=['run_id', 'extra'])
    assert mio.load(p)[0]['run_id'] == 'gng'


# ── group_runs ──────────────────────────────────────────────────────────────

def test_two_flights_with_the_same_run_id_stay_separate(tmp_path):
    """
    Dua penerbangan ber-run_id sama menghasilkan dua kunci.

    Inilah bug yang menghilangkan satu penerbangan dari tabel: kuncinya
    `run_id`, jadi berkas yang lebih baru menimpa entri yang lebih lama.
    """
    for stamp, val in (('074807', '1'), ('081836', '2')):
        write_csv(tmp_path / ('gng_20260828_%s_landing.csv' % stamp),
                  [{'run_id': 'gng', 'recv_ns': val}],
                  header=['run_id', 'recv_ns', 'extra'])

    runs = mio.group_runs(tmp_path)
    assert set(runs) == {'gng_20260828_074807', 'gng_20260828_081836'}
    assert runs['gng_20260828_074807']['landing'][0]['recv_ns'] == '1'
    assert runs['gng_20260828_081836']['landing'][0]['recv_ns'] == '2'


def test_group_runs_ignores_files_that_are_not_ours(tmp_path):
    """CSV asing di direktori yang sama tidak boleh ikut terbaca."""
    write_csv(tmp_path / 'catatan.csv', [{'a': '1'}], header=['a', 'extra'])
    assert mio.group_runs(tmp_path) == {}


def test_group_runs_skips_a_file_with_only_a_header(tmp_path):
    """`ransac.csv` kosong ada di tiap penerbangan GNG; jangan dihitung."""
    p = tmp_path / 'gng_20260828_081836_ransac.csv'
    p.write_text('run_id,recv_ns,extra\n')
    assert mio.group_runs(tmp_path) == {}


# ── angka ───────────────────────────────────────────────────────────────────

def test_to_float_returns_the_default_for_an_empty_cell():
    """Sel kosong dan teks menghasilkan default, bukan ValueError."""
    assert mio.to_float({'a': ''}, 'a', default=-1) == -1
    assert mio.to_float({'a': 'DESCEND'}, 'a') is None


def test_to_float_keeps_zero_rather_than_treating_it_as_missing():
    """
    Nol adalah pengukuran, bukan ketiadaan pengukuran.

    `obstacle_points: 0` berarti penjaga penghalang melihat dan tidak
    menemukan apa-apa — baris yang justru membuktikan penjaganya bekerja.
    """
    assert mio.to_float({'a': '0'}, 'a') == 0.0
    assert mio.nums([{'a': '0'}, {'a': '0.0'}], 'a') == [0.0, 0.0]


def test_numeric_columns_leaves_out_the_text_ones():
    """Hanya kolom yang benar-benar angka yang boleh ditawarkan."""
    rows = [{'run_id': 'gng', 'to_state': 'DESCEND', 't_s': '1.5',
             'recv_ns': '900'}]
    cols = mio.numeric_columns(rows)
    assert 't_s' in cols and 'recv_ns' in cols
    assert 'to_state' not in cols and 'run_id' not in cols


def test_seconds_from_counts_from_the_earliest_sample():
    """Nol adalah sampel paling awal, bukan sampel pertama di daftar."""
    assert mio.seconds_from([3_000_000_000, 1_000_000_000]) == [2.0, 0.0]


# ── jendela udara ───────────────────────────────────────────────────────────

def transitions(*pairs):
    """Baris transisi ringkas: (to_state, recv_ns)."""
    return [{'to_state': s, 'recv_ns': str(ns)} for s, ns in pairs]


def test_airborne_window_spans_takeoff_to_landed():
    """Jendela udara membentang dari TAKEOFF sampai LANDED."""
    kinds = {'mission_transition': transitions(
        ('ARMING', 100), ('TAKEOFF', 200), ('SCAN', 500), ('LANDED', 900))}
    assert mio.airborne_window(kinds) == (200, 900)


def test_airborne_window_falls_back_to_the_last_event_when_it_gave_up():
    """Penerbangan yang berakhir HOLD tetap punya jendela udara yang sah."""
    kinds = {'mission_transition': transitions(
        ('TAKEOFF', 200), ('SCAN', 500), ('HOLD', 800))}
    assert mio.airborne_window(kinds) == (200, 800)


def test_airborne_window_is_none_without_transitions():
    """Tanpa transisi tidak ada jendela; jangan menebak."""
    assert mio.airborne_window({}) is None
    assert mio.airborne_window({'mission_transition': []}) is None


def test_airborne_window_is_none_when_takeoff_is_missing():
    """Tanpa TAKEOFF, awal jendelanya tidak diketahui."""
    kinds = {'mission_transition': transitions(('LANDED', 900))}
    assert mio.airborne_window(kinds) is None


# ── within ──────────────────────────────────────────────────────────────────

def test_within_keeps_the_boundaries_and_drops_the_outside():
    """Batasnya inklusif di kedua ujung."""
    rows = [{'recv_ns': str(v)} for v in (100, 200, 500, 900, 1000)]
    kept = [r['recv_ns'] for r in mio.within(rows, (200, 900))]
    assert kept == ['200', '500', '900']


def test_within_without_a_window_changes_nothing():
    """Tanpa jendela, barisnya diteruskan apa adanya."""
    rows = [{'recv_ns': '1'}]
    assert mio.within(rows, None) is rows


def test_within_drops_a_row_that_has_no_timestamp():
    """Baris tanpa recv_ns tidak bisa ditempatkan, jadi tidak bisa diklaim."""
    rows = [{'recv_ns': '300'}, {'recv_ns': ''}]
    assert len(mio.within(rows, (200, 900))) == 1
