"""
Unit test untuk perekam metrik.

Berkas ini ada karena sebuah kegagalan senyap. Versi sebelumnya mengurai teks
berformat dengan regex; regex itu patah saat format hulunya berubah, baris CSV
berhenti ditulis sama sekali, dan tidak ada satu pun pesan galat — berkasnya
kosong berbulan-bulan tanpa ada yang tahu. Tidak ada tes yang menjaganya.

Jadi yang diuji di sini bukan "apakah CSV-nya bisa ditulis", melainkan
sifat-sifat yang seharusnya membuat kegagalan itu mustahil terulang:

  - muatan sungguhan dari tiap penerbit terurai utuh, tanpa kolom yang hilang;
  - field yang MUNCUL BELAKANGAN tidak pernah lenyap diam-diam;
  - muatan rusak menghasilkan laporan, bukan berkas kosong yang membisu.

Tidak butuh rclpy berjalan: yang diuji fungsi murni dan penulis berkasnya.
"""

import csv
import json
import math

import pytest

from logger_stats.stats_logger_node import (CsvSink, StatsLoggerNode,
                                            yaw_deg_from_quaternion)


flatten = StatsLoggerNode._flatten


# ── Muatan contoh, disalin dari bentuk yang benar-benar diterbitkan ──────────

GNG_PAYLOAD = {
    "source": "gng", "stamp_ns": 1787806799260505384,
    "computation_time_ms": 13.55, "latency_ms": 82.4,
    "input_points": 19200, "valid_points": 6561, "valid_percentage": 34.17,
    "downsampled_points": 3013, "plane_size": 1834, "outlier_size": 214,
    "voxel_leaf_m": 0.15,
}

LANDING_PAYLOAD = {
    "source": "landing_circle", "stamp_ns": 1787806799260505384,
    "computation_time_ms": 13.55, "latency_ms": 91.2,
    "input_points": 3013, "valid_points": 3013, "valid_percentage": 100.0,
    "safe_size": 2, "registry_size": 28, "num_candidates": 28,
    "blocked_candidates": 5, "committed": True, "collect_elapsed_s": 10.0,
    "frozen_by_motion": False, "speed_mps": 0.02, "target_blocked_s": 0.0,
    "obstacle_points": 0, "target_alt_m": 13.04, "has_selection": True,
    "selected_distance_m": 13.78, "selected_score": 0.685,
    "selected_x": -3.25, "selected_y": 11.73, "selected_z": -2.64,
    "grid_cell_m": 0.6, "min_pts_per_cell": 2,
    "candidates": [
        {"x": -3.25, "y": 11.73, "z": -2.64, "hits": 12, "blocked_s": 0.0,
         "clearance_m": 1.2, "fill_ratio": 1.0, "roughness_m": 0.0082,
         "rough_n": 12, "score": 0.685, "selectable": True},
        {"x": -3.92, "y": 10.93, "z": -2.61, "hits": 9, "blocked_s": 0.0,
         "clearance_m": 0.9, "fill_ratio": 1.0, "roughness_m": 0.0148,
         "rough_n": 9, "score": 0.650, "selectable": True},
    ],
}

MISSION_SUMMARY = {
    "run_id": "gng", "kind": "summary", "t_s": 66.8, "state": "LANDING",
    "outcome": "landed", "target": [-0.17, 8.6, 11.0],
    "chosen_point": [-3.25, 11.73, -2.64], "touchdown": [-3.21, 11.6, -2.64],
    "landing_error_m": 0.1304, "time_to_commit_s": 10.3,
    "time_to_arrive_s": 14.0, "total_time_s": 66.8,
    "scan_attempts": 1, "abort_count": 0,
    "state_durations_s": {"GOTO": 5.9, "SCAN": 10.3, "DESCEND": 32.7},
}


# ── flatten ─────────────────────────────────────────────────────────────────

def test_flatten_keeps_every_scalar_field():
    """Tiap field skalar muatan harus selamat sampai ke baris CSV."""
    out = flatten(GNG_PAYLOAD)
    for key in GNG_PAYLOAD:
        assert key in out, 'field %r hilang saat didatarkan' % key


def test_flatten_expands_nested_dict_into_prefixed_columns():
    """Dict bersarang jadi kolom berprefiks, bukan satu sel JSON."""
    out = flatten(MISSION_SUMMARY)
    assert out['state_durations_s_SCAN'] == 10.3
    assert out['state_durations_s_DESCEND'] == 32.7
    assert 'state_durations_s' not in out


def test_flatten_serialises_lists_instead_of_dropping_them():
    """Koordinat berbentuk list tetap terbaca, bukan hilang."""
    out = flatten(MISSION_SUMMARY)
    assert json.loads(out['touchdown']) == [-3.21, 11.6, -2.64]


def test_flatten_writes_booleans_as_zero_or_one():
    """Boolean jadi 0/1 supaya bisa dijumlahkan langsung di analisis."""
    out = flatten(LANDING_PAYLOAD)
    assert out['committed'] == 1
    assert out['frozen_by_motion'] == 0


def test_flatten_keeps_zero_and_false_rather_than_dropping_them():
    """
    Nol adalah pengukuran, bukan ketiadaan pengukuran.

    `obstacle_points: 0` berarti penjaga penghalang melihat dan tidak
    menemukan apa-apa. Membuangnya karena 'kosong' akan menghapus justru
    baris-baris yang membuktikan penjaganya bekerja.
    """
    out = flatten(LANDING_PAYLOAD)
    assert out['obstacle_points'] == 0
    assert out['target_blocked_s'] == 0.0
    assert out['has_selection'] == 1


# ── CsvSink ─────────────────────────────────────────────────────────────────

def test_sink_writes_header_and_one_row_per_call(tmp_path):
    """Satu pesan, satu baris -- tanpa dedup, tanpa penggabungan."""
    s = CsvSink(tmp_path / 'a.csv')
    s.write({'a': 1, 'b': 2})
    s.write({'a': 3, 'b': 4})
    s.close()

    rows = list(csv.reader(open(tmp_path / 'a.csv')))
    assert rows[0] == ['a', 'b', 'extra']
    assert len(rows) == 3, 'satu header + dua baris data'
    assert s.rows == 2


def test_sink_never_silently_drops_a_field_that_appears_later(tmp_path):
    """
    Field baru di hulu harus muncul, bukan lenyap.

    Inilah bentuk kegagalan yang dulu terjadi: format berubah, perekamnya
    tidak, dan datanya hilang tanpa suara.
    """
    s = CsvSink(tmp_path / 'b.csv')
    s.write({'a': 1})
    s.write({'a': 2, 'roughness_m': 0.0082})
    s.close()

    rows = list(csv.reader(open(tmp_path / 'b.csv')))
    assert json.loads(rows[2][-1]) == {'roughness_m': 0.0082}


def test_sink_leaves_extra_empty_when_nothing_is_extra(tmp_path):
    """Kolom `extra` kosong bila memang tidak ada yang berlebih."""
    s = CsvSink(tmp_path / 'c.csv')
    s.write({'a': 1})
    s.close()
    assert list(csv.reader(open(tmp_path / 'c.csv')))[1] == ['1', '']


# ── muatan sungguhan, ujung ke ujung ────────────────────────────────────────

def test_landing_payload_produces_one_row_per_candidate(tmp_path):
    """
    Kandidat dicatat dalam format panjang, satu baris masing-masing.

    Cara lama membentangkannya jadi kolom lc_1_x, lc_2_x, ... yang membuat
    sebaran roughness dan skor praktis tidak bisa diplot.
    """
    payload = json.loads(json.dumps(LANDING_PAYLOAD))
    cands = payload.pop('candidates')

    sink = CsvSink(tmp_path / 'cand.csv')
    for i, c in enumerate(cands):
        row = {'stamp_ns': payload['stamp_ns'], 'candidate_index': i + 1}
        row.update(flatten(c))
        sink.write(row)
    sink.close()

    rows = list(csv.DictReader(open(tmp_path / 'cand.csv')))
    assert len(rows) == 2
    assert float(rows[0]['roughness_m']) == 0.0082
    assert float(rows[1]['score']) == 0.650
    assert rows[0]['candidate_index'] == '1'


def test_every_landing_scalar_survives_to_the_csv(tmp_path):
    """
    Regresi langsung terhadap kegagalan senyap itu.

    Sebelas field — selected_score, roughness, committed, obstacle_points dan
    lainnya — hilang tanpa jejak dari CSV lama. Tes ini menuntut setiap field
    skalar muatan sungguhan muncul sebagai kolom yang terisi.
    """
    payload = json.loads(json.dumps(LANDING_PAYLOAD))
    payload.pop('candidates')

    sink = CsvSink(tmp_path / 'land.csv')
    sink.write(flatten(payload))
    sink.close()

    row = list(csv.DictReader(open(tmp_path / 'land.csv')))[0]
    for key in payload:
        assert key in row, 'kolom %r tidak ada di CSV' % key
        assert row[key] != '', 'kolom %r ada tapi kosong' % key


def test_mission_summary_keeps_the_headline_number(tmp_path):
    """landing_error_m adalah angka utama penilaian; ia tidak boleh hilang."""
    sink = CsvSink(tmp_path / 'mission.csv')
    sink.write(flatten(MISSION_SUMMARY))
    sink.close()

    row = list(csv.DictReader(open(tmp_path / 'mission.csv')))[0]
    assert float(row['landing_error_m']) == 0.1304
    assert float(row['time_to_commit_s']) == 10.3
    assert row['abort_count'] == '0'


# ── lintasan drone ──────────────────────────────────────────────────────────

class _Q:
    """Kuaternion sederhana, cukup untuk yaw."""

    def __init__(self, w=1.0, x=0.0, y=0.0, z=0.0):
        self.w, self.x, self.y, self.z = w, x, y, z


def test_yaw_is_zero_for_the_identity_quaternion():
    """Drone menghadap +x ENU tanpa rotasi."""
    assert yaw_deg_from_quaternion(_Q()) == pytest.approx(0.0)


def test_yaw_reads_ninety_degrees_for_a_quarter_turn():
    """Seperempat putaran berlawanan jarum jam."""
    a = math.pi / 4
    assert yaw_deg_from_quaternion(
        _Q(w=math.cos(a), z=math.sin(a))) == pytest.approx(90.0)


def test_yaw_is_signed_not_wrapped_to_positive():
    """
    Belok ke kanan terbaca negatif, bukan 270 derajat.

    Menyimpan 270 akan membuat grafik heading melompat sepanjang lintasan
    setiap kali drone melewati arah nol.
    """
    a = -math.pi / 4
    assert yaw_deg_from_quaternion(
        _Q(w=math.cos(a), z=math.sin(a))) == pytest.approx(-90.0)


def test_yaw_handles_the_half_turn_boundary():
    assert abs(yaw_deg_from_quaternion(_Q(w=0.0, z=1.0))) == pytest.approx(180.0)


def test_trajectory_row_carries_every_column(tmp_path):
    """
    Baris lintasan harus memuat posisi, arah hadap, dan stempelnya.

    `stamp_ns` yang membuat lintasan bisa disejajarkan dengan
    <run>_candidates.csv dan <run>_landing.csv dari penerbangan yang sama;
    tanpanya berkas ini hanya bisa dibaca sendirian.
    """
    row = {
        'run_id': 'gng', 'recv_ns': 12345, 'stamp_ns': 1787806799260505384,
        'frame_id': 'map', 'x': -1.62, 'y': 14.36, 'z': 11.0, 'yaw_deg': 73.01,
    }
    sink = CsvSink(tmp_path / 'traj.csv')
    sink.write(row)
    sink.close()

    got = list(csv.DictReader(open(tmp_path / 'traj.csv')))[0]
    for key in row:
        assert got[key] != '', 'kolom %r kosong' % key
    assert float(got['x']) == -1.62
    assert float(got['yaw_deg']) == 73.01


def test_trajectory_keeps_a_zero_coordinate(tmp_path):
    """
    Nol adalah posisi, bukan ketiadaan posisi.

    Drone yang tepat berada di origin `map` menghasilkan x=0.0, dan baris itu
    harus terekam seperti baris lain.
    """
    sink = CsvSink(tmp_path / 'traj0.csv')
    sink.write({'stamp_ns': 1, 'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw_deg': 0.0})
    sink.close()

    got = list(csv.DictReader(open(tmp_path / 'traj0.csv')))[0]
    assert float(got['x']) == 0.0
    assert float(got['z']) == 0.0
