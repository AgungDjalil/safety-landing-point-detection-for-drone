"""
Unit test untuk saklar `perception` di depth_bridge_launch.py.

    colcon test --packages-select gz_bridge_ros2

Yang diuji dua hal. Pertama tabel mode: nilai `perception` mana menyalakan
node yang mana. Kedua, dan yang lebih mudah rusak diam-diam, kesamaan awan
masukan kedua front-end — kalau ukuran voxel keduanya lepas satu sama lain,
angka waktu komputasinya tetap terbit dan tetap terlihat wajar, hanya saja
tidak lagi membandingkan algoritma.

Semuanya kondisi dan substitution yang baru dievaluasi saat launch berjalan,
jadi salah di sini tidak menghasilkan error — melainkan launch yang tampak
sehat sambil tidak menyalakan apa pun, atau menyalakan node yang berlangganan
topik yang tak pernah terbit. Dua-duanya sudah pernah terjadi di rantai ini.
"""

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.substitutions import LaunchConfiguration


def _load():
    """
    Impor berkas launch langsung dari sumbernya.

    Berkas launch bukan modul Python yang terpasang — colcon menyalinnya ke
    share/, bukan ke site-packages — jadi ia dimuat lewat lokasinya.
    """
    path = Path(__file__).resolve().parent.parent / 'launch' / 'depth_bridge_launch.py'
    spec = importlib.util.spec_from_file_location('depth_bridge_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


bridge = _load()

PERCEPTION = LaunchConfiguration('perception')
PERCEPTION_INPUT = LaunchConfiguration('perception_input')
VOXEL_LEAF = LaunchConfiguration('voxel_leaf')

RAW = '/depth_camera/points'
ZED = '/zed/zed_node/point_cloud/cloud_registered'


def context(perception, perception_input=RAW, voxel_leaf=None):
    """Bangun LaunchContext dengan ketiga argumen sudah terisi."""
    ctx = LaunchContext()
    ctx.launch_configurations.update({
        'perception': perception,
        'perception_input': perception_input,
        'voxel_leaf': (bridge.VOXEL_LEAF_DEFAULT
                       if voxel_leaf is None else voxel_leaf),
    })
    return ctx


def started(perception):
    """Himpunan nama node yang benar-benar akan jalan pada mode ini."""
    nodes = bridge.perception_nodes(PERCEPTION, PERCEPTION_INPUT, VOXEL_LEAF)
    ctx = context(perception)
    return {name for name, node in nodes.items() if node.condition.evaluate(ctx)}


def resolve(value, ctx):
    """Substitution -> string; nilai biasa diteruskan apa adanya."""
    return value.perform(ctx) if hasattr(value, 'perform') else value


# -- tabel mode -------------------------------------------------------------

@pytest.mark.parametrize('perception, expected', [
    ('none', set()),
    ('gng_cpu', {'dbl_gng_cpu'}),
    ('gng_gpu', {'dbl_gng'}),
    ('ransac', {'plane_segmentation_ransac'}),
])
def test_each_mode_starts_exactly_its_own_nodes(perception, expected):
    """Tiap mode menyalakan persis himpunan node miliknya, tidak lebih."""
    assert started(perception) == expected


def test_the_cylinder_crop_is_gone_entirely():
    """
    Tidak ada lagi node cylinder_crop yang bisa dinyalakan launch ini.

    Ia memotong silinder pada sumbu x-y, padahal di `camera_link` justru
    x-lah sumbu KEDALAMAN — /depth_camera/points membentang x 0,21..14,81 m
    (median 12,51) di ketinggian pindai 11 m, sementara bidang tanahnya ada
    di y-z (karena itu landing_circle dikonfigurasi plane_axes=yz). Hasilnya
    19.200 titik masuk, 4 keluar, RANSAC berhenti menerbitkan apa pun, dan
    misi kehabisan waktu menunggu safety_point.
    """
    nodes = bridge.perception_nodes(PERCEPTION, PERCEPTION_INPUT, VOXEL_LEAF)
    assert 'cylinder_crop' not in nodes


def test_gng_and_ransac_are_mutually_exclusive():
    """
    Tidak ada nilai `perception` yang menyalakan kedua front-end.

    Inilah alasan satu argumen `perception` dipakai, bukan dua argumen
    terpisah: dua front-end sekaligus berarti dua kali beban CPU tanpa satu
    pun peringatan, di mesin yang sudah kewalahan menjalankan Gazebo.
    """
    gng = {'dbl_gng_cpu', 'dbl_gng'}
    ransac = {'plane_segmentation_ransac'}
    for mode in bridge.PERCEPTION_MODES:
        running = started(mode)
        assert not (running & gng and running & ransac)


def test_an_unknown_mode_starts_nothing():
    """
    Mode yang salah ketik tidak menyalakan apa pun.

    Yang membenarkan adanya LogInfo peringatan: tanpa itu, `perception:=gng`
    terlihat persis seperti launch yang sehat sampai misi kehabisan waktu
    menunggu titik pendaratan yang tidak akan pernah datang.
    """
    assert started('gng') == set()
    assert started('RANSAC') == set()
    assert started('') == set()


def test_the_gng_node_name_matches_the_missions_duplicate_guard():
    """
    Nama node GNG di sini harus sama dengan yang dipakai waypoint_node.

    PerceptionSupervisor melewatkan node yang sudah hidup dengan mencocokkan
    NAMA, dan default `perception_commands` di waypoint_node menyebut
    `dbl_gng_cpu`. Kalau nama di sini berbeda, misi akan menyalakan GNG kedua
    di samping yang sudah jalan.
    """
    assert 'dbl_gng_cpu' in bridge.perception_nodes(
        PERCEPTION, PERCEPTION_INPUT, VOXEL_LEAF)


# -- kesamaan awan masukan --------------------------------------------------

@pytest.mark.parametrize('leaf', ['0.15', '0.07', '0.3', '0.05'])
def test_both_front_ends_get_the_same_voxel_size(leaf):
    """
    Nilai leaf yang sampai ke GNG dan ke RANSAC selalu sama.

    Invarian yang membuat perbandingannya sah. Keduanya menamainya berbeda —
    GNG `voxel_leaf`, RANSAC `leaf_size` — sehingga menyetel salah satunya
    saja adalah kesalahan yang mudah dilakukan dan tidak menghasilkan gejala
    apa pun: angkanya tetap terbit dan tetap terlihat wajar. Terukur sebelum
    disamakan: RANSAC 19200->6445, GNG 19200->~2000.
    """
    ctx = context('ransac', voxel_leaf=leaf)
    gng = bridge.gng_params(PERCEPTION_INPUT, VOXEL_LEAF, True)
    ransac = bridge.ransac_params(PERCEPTION_INPUT, VOXEL_LEAF)
    assert resolve(gng['voxel_leaf'], ctx) == leaf
    assert resolve(ransac['leaf_size'], ctx) == leaf


def test_the_default_leaf_is_the_one_gng_already_used():
    """Tanpa argumen apa pun, yang didapat adalah masukan sebesar milik GNG."""
    assert bridge.VOXEL_LEAF_DEFAULT == '0.15'


def test_the_gpu_backend_is_not_given_a_voxel_it_cannot_read():
    """
    Node GPU tidak dioperi `voxel_leaf`.

    `dbl_gng_node` tidak punya tahap downsampling dan tidak mendeklarasikan
    parameter itu. Override untuk parameter yang tidak pernah dideklarasikan
    dibuang tanpa sepatah kata, jadi mengopernya ke sana akan membuat seolah
    kedua backend sudah disamakan padahal hanya satu yang disetel.
    """
    assert 'voxel_leaf' not in bridge.gng_params(PERCEPTION_INPUT, VOXEL_LEAF, False)
    assert 'voxel_leaf' in bridge.gng_params(PERCEPTION_INPUT, VOXEL_LEAF, True)


def test_the_ransac_passthrough_is_disabled():
    """
    Batas z RANSAC dibuka lebar, karena GNG tidak punya filter serupa.

    Di `camera_link` z adalah sumbu lateral yang membentang -8,15..7,40 m di
    ketinggian pindai 11 m, jadi batas bawaan [-5, 5] memotong pita luar
    medan pandang yang GNG tetap lihat. Itu filter masukan, bukan bagian
    algoritmanya.
    """
    params = bridge.ransac_params(PERCEPTION_INPUT, VOXEL_LEAF)
    assert params['z_min'] <= -100.0
    assert params['z_max'] >= 100.0


def test_both_front_ends_read_the_same_topic():
    """
    GNG dan RANSAC berlangganan topik yang sama, tanpa perantara.

    Tidak ada lagi topik antara: `perception_input` langsung, apa adanya.
    """
    ctx = context('ransac')
    gng = bridge.gng_params(PERCEPTION_INPUT, VOXEL_LEAF, True)
    ransac = bridge.ransac_params(PERCEPTION_INPUT, VOXEL_LEAF)
    assert resolve(gng['pointcloud_topic'], ctx) == RAW
    assert resolve(ransac['input_topic'], ctx) == RAW


def test_a_custom_input_topic_reaches_both():
    """Kamera ZED asli tetap bisa dipakai lewat perception_input."""
    ctx = context('ransac', perception_input=ZED)
    gng = bridge.gng_params(PERCEPTION_INPUT, VOXEL_LEAF, True)
    ransac = bridge.ransac_params(PERCEPTION_INPUT, VOXEL_LEAF)
    assert resolve(gng['pointcloud_topic'], ctx) == ZED
    assert resolve(ransac['input_topic'], ctx) == ZED


def test_ransac_is_never_left_on_its_zed_default():
    """
    input_topic selalu dioper eksplisit.

    Default node-nya adalah topik ZED yang tidak pernah ada di simulasi.
    Node yang berlangganan ke sana tetap hidup dan bisu selamanya — kegagalan
    senyap yang sudah pernah menimpa berkas launch RANSAC yang lama.
    """
    params = bridge.ransac_params(PERCEPTION_INPUT, VOXEL_LEAF)
    assert 'input_topic' in params
    resolved = resolve(params['input_topic'], context('ransac'))
    assert resolved and resolved != ZED


def test_both_front_ends_run_on_the_simulation_clock():
    """
    use_sim_time wajib, sama seperti node lain di launch ini.

    Tanpanya stempel keluarannya memakai jam dinding sementara TF memakai jam
    gz sim, dan MessageFilter RViz2 membuang tiap pesan.
    """
    assert bridge.ransac_params(PERCEPTION_INPUT, VOXEL_LEAF)['use_sim_time'] is True
    for with_voxel in (True, False):
        assert bridge.gng_params(
            PERCEPTION_INPUT, VOXEL_LEAF, with_voxel)['use_sim_time'] is True
