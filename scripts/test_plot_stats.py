"""
Unit test untuk agregasi tren di plot_stats.

    python3 -m pytest scripts/

Yang diuji `binned_trend`: fungsi murni yang mengubah awan titik (x, y) menjadi
garis median beserta pita p10-p90. Ia bisa salah tanpa menghasilkan error —
garisnya tetap tergambar, hanya bentuknya yang berbohong — jadi sifat-sifatnya
dikunci di sini alih-alih dipercaya dari melihat gambarnya.
"""

import pytest

from plot_stats import binned_trend


def linear(n=200, slope=2.0):
    """Bangun n pasangan (x, y) yang naik linear tanpa derau."""
    return [(float(i), slope * i) for i in range(n)]


# ── bentuk keluaran ─────────────────────────────────────────────────────────

def test_returns_four_lists_of_equal_length():
    """Pusat, p10, median, dan p90 harus sejajar satu sama lain."""
    cx, p10, med, p90 = binned_trend(linear(), nbins=10)
    assert len(cx) == len(p10) == len(med) == len(p90)
    assert len(cx) > 0


def test_empty_input_returns_empty_lists():
    """Daftar kosong bukan alasan melempar pengecualian."""
    assert binned_trend([]) == ([], [], [], [])


def test_a_single_x_value_does_not_divide_by_zero():
    """
    Rentang x nol berarti tidak ada tren untuk digambar.

    Terjadi nyata ketika kolom yang dipilih ternyata konstan — mis.
    `input_points`, yang selalu 19.200 karena itu ukuran kisi citra depth.
    """
    assert binned_trend([(5.0, 1.0)] * 20) == ([], [], [], [])


# ── nilai ───────────────────────────────────────────────────────────────────

def test_median_rises_monotonically_for_rising_data():
    """Data yang naik linear harus menghasilkan median yang naik."""
    _, _, med, _ = binned_trend(linear(), nbins=10)
    assert med == sorted(med)


def test_percentiles_bracket_the_median():
    """Persentilnya mengapit median di tiap bin, tanpa kecuali."""
    pts = [(float(i), float((i * 37) % 100)) for i in range(400)]
    _, p10, med, p90 = binned_trend(pts, nbins=8)
    for lo, m, hi in zip(p10, med, p90):
        assert lo <= m <= hi


def test_bin_centres_lie_inside_the_data_range():
    """Pusat bin selalu di dalam rentang datanya, bukan di tepinya."""
    cx, _, _, _ = binned_trend(linear(n=100), nbins=5)
    assert min(cx) > 0.0
    assert max(cx) < 99.0


def test_a_flat_relationship_gives_a_flat_line():
    """Hubungan datar memberi garis datar; tren tidak mengarang lereng."""
    pts = [(float(i), 42.0) for i in range(100)]
    _, _, med, _ = binned_trend(pts, nbins=10)
    assert all(m == 42.0 for m in med)


# ── bin yang terlalu tipis ──────────────────────────────────────────────────

def test_bins_with_too_few_samples_are_skipped():
    """
    Satu frame pencilan tidak boleh menjadi sebuah tren.

    Di sini seluruh data padat di x kecil kecuali satu titik jauh di ujung;
    bin yang hanya berisi titik itu harus hilang, bukan digambar sebagai
    lonjakan.
    """
    pts = [(float(i % 10), 1.0) for i in range(100)] + [(1000.0, 999.0)]
    cx, _, med, _ = binned_trend(pts, nbins=10, min_per_bin=3)
    assert 999.0 not in med
    assert all(x < 500.0 for x in cx)


def test_min_per_bin_of_one_keeps_everything():
    """Ambang satu sampel berarti tidak ada bin yang dibuang."""
    pts = [(float(i), float(i)) for i in range(10)]
    cx, _, _, _ = binned_trend(pts, nbins=10, min_per_bin=1)
    assert len(cx) == 10


def test_more_bins_than_samples_is_safe():
    """Meminta 500 bin atas 10 sampel tidak boleh menjatuhkan skripnya."""
    cx, _, _, _ = binned_trend(linear(n=10), nbins=500)
    assert isinstance(cx, list)


def test_every_sample_lands_in_some_bin():
    """
    Nilai x terbesar tidak boleh jatuh di luar bin terakhir.

    Pembulatan `int((x - lo) / width)` menghasilkan indeks satu di luar
    jangkauan untuk x maksimum; kalau tidak diklem, sampel paling ekstrem —
    justru yang paling menarik — akan hilang diam-diam.
    """
    pts = [(float(i), 1.0) for i in range(30)]
    _, _, med, _ = binned_trend(pts, nbins=3, min_per_bin=1)
    assert len(med) == 3


def test_percentiles_widen_with_spread():
    """Pita harus melebar saat sebarannya melebar, bukan tetap."""
    tight = [(float(i), 50.0 + (i % 3)) for i in range(300)]
    wide = [(float(i), 50.0 + (i % 100)) for i in range(300)]
    _, lo_t, _, hi_t = binned_trend(tight, nbins=5)
    _, lo_w, _, hi_w = binned_trend(wide, nbins=5)
    span_t = sum(h - lo for lo, h in zip(lo_t, hi_t)) / len(lo_t)
    span_w = sum(h - lo for lo, h in zip(lo_w, hi_w)) / len(lo_w)
    assert span_w > span_t


def test_rejects_nothing_when_data_is_dense_everywhere():
    """Data padat merata harus memakai seluruh bin yang diminta."""
    pts = [(float(i % 50), float(i)) for i in range(500)]
    cx, _, _, _ = binned_trend(pts, nbins=10, min_per_bin=3)
    assert len(cx) == 10


def test_negative_x_values_are_handled():
    """
    Sumbu x boleh negatif.

    `z` dari trajectory turun sampai -2 m, karena tanah di titik pendaratan
    berada di bawah origin frame `map`.
    """
    pts = [(float(i) - 50.0, float(i)) for i in range(100)]
    cx, _, med, _ = binned_trend(pts, nbins=5)
    assert min(cx) < 0.0
    assert med == sorted(med)


@pytest.mark.parametrize('nbins', [1, 2, 7, 20, 64])
def test_bin_count_is_respected_within_the_skip_rule(nbins):
    """Jumlah bin yang diminta dihormati saat datanya padat merata."""
    cx, _, _, _ = binned_trend(linear(n=1000), nbins=nbins, min_per_bin=1)
    assert len(cx) == nbins
