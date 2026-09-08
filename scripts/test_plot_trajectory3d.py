"""
Unit test untuk proyeksi lintasan 3D.

    python3 -m pytest scripts/

`project` dan `equalise` murni, dan justru di sanalah kesalahan tidak akan
memunculkan error — gambarnya tetap keluar, hanya bentuknya yang berbohong.
Sudut pandang yang tertukar atau skala yang diregangkan menghasilkan lintasan
yang tampak masuk akal padahal menggambarkan gerakan yang tidak pernah
terjadi.
"""

import math

import pytest

from plot_trajectory3d import equalise, project


# ── project ─────────────────────────────────────────────────────────────────

def test_top_down_view_shows_the_ground_plane():
    """
    Pada elev=90 gambarnya adalah tampak atas: v mengikuti y, bukan z.

    Ketinggian harus lenyap dari gambar; kalau ia masih muncul, sumbu
    tegaknya tertukar dengan sumbu utara.
    """
    u, v = project(3.0, 7.0, 99.0, azim_deg=0, elev_deg=90)
    assert u == pytest.approx(3.0)
    assert v == pytest.approx(7.0)


def test_side_view_shows_altitude_directly():
    """Pada elev=0 sumbu tegak layar adalah z apa adanya."""
    u, v = project(3.0, 7.0, 5.0, azim_deg=0, elev_deg=0)
    assert u == pytest.approx(3.0)
    assert v == pytest.approx(5.0)


def test_azimuth_ninety_swaps_the_horizontal_axes():
    """Putaran seperempat menukar peran x dan y."""
    u, _ = project(1.0, 0.0, 0.0, azim_deg=90, elev_deg=0)
    assert u == pytest.approx(0.0, abs=1e-9)
    u2, _ = project(0.0, 1.0, 0.0, azim_deg=90, elev_deg=0)
    assert u2 == pytest.approx(1.0)


def test_origin_stays_at_the_origin_for_any_view():
    """Titik asal tidak boleh bergeser oleh sudut pandang."""
    for azim in (0, 37, 90, 180, 305):
        for elev in (0, 25, 90):
            u, v = project(0.0, 0.0, 0.0, azim, elev)
            assert u == pytest.approx(0.0, abs=1e-12)
            assert v == pytest.approx(0.0, abs=1e-12)


def test_a_full_turn_returns_to_the_same_place():
    """Memutar 360 derajat mengembalikan koordinat semula."""
    a = project(2.0, -3.0, 4.0, azim_deg=17, elev_deg=30)
    b = project(2.0, -3.0, 4.0, azim_deg=17 + 360, elev_deg=30)
    assert a[0] == pytest.approx(b[0])
    assert a[1] == pytest.approx(b[1])


def test_projection_is_orthographic_not_perspective():
    """
    Ruas yang sama panjang tergambar sama panjang, di mana pun letaknya.

    Perspektif akan memendekkan yang jauh, dan gambar ini dimaksudkan untuk
    diukur terhadap batang skalanya.
    """
    def length(x0, x1):
        a = project(x0, 0.0, 0.0, 45, 25)
        b = project(x1, 0.0, 0.0, 45, 25)
        return math.hypot(b[0] - a[0], b[1] - a[1])

    assert length(0.0, 1.0) == pytest.approx(length(100.0, 101.0))


# ── equalise ────────────────────────────────────────────────────────────────

def test_centre_is_the_midpoint_of_each_axis():
    """Pusat tiap sumbu adalah titik tengah rentangnya."""
    (cx, cy, cz), _ = equalise([0.0, 10.0], [-4.0, 4.0], [2.0, 6.0])
    assert (cx, cy, cz) == (5.0, 0.0, 4.0)


def test_scale_is_the_largest_span():
    """Skalanya diambil dari sumbu terpanjang, bukan per sumbu."""
    _, scale = equalise([0.0, 2.0], [0.0, 20.0], [0.0, 5.0])
    assert scale == pytest.approx(20.0)


def test_a_narrow_axis_stays_narrow():
    """
    Sumbu sempit tetap sempit sesudah dinormalisasi.

    Inilah alasan `equalise` ada: pada penerbangan yang diuji, x membentang
    1,70 m sedangkan y 15,59 m. Menskalakan tiap sumbu sendiri-sendiri akan
    menggambar goyangan menyamping 1,7 m selebar perjalanan 15 m — gerakan
    yang tidak pernah terjadi.
    """
    xs, ys, zs = [0.0, 1.7], [0.0, 15.6], [0.0, 14.0]
    (cx, cy, _), scale = equalise(xs, ys, zs)
    span_x = (max(xs) - min(xs)) / scale
    span_y = (max(ys) - min(ys)) / scale
    assert span_y == pytest.approx(1.0)
    assert span_x == pytest.approx(1.7 / 15.6, rel=1e-6)
    assert span_x < 0.15


def test_a_single_point_does_not_divide_by_zero():
    """Lintasan yang tidak bergerak sama sekali tetap boleh digambar."""
    _, scale = equalise([1.0], [1.0], [1.0])
    assert scale == 1.0


def test_scale_is_never_zero_even_for_a_flat_hover():
    """Drone yang melayang diam sempurna punya rentang nol di ketiga sumbu."""
    _, scale = equalise([2.0, 2.0], [3.0, 3.0], [4.0, 4.0])
    assert scale > 0
