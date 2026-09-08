# Safety Landing Pipeline

Dokumen ini menjelaskan alur perhitungan sistem *safety landing* dari point
cloud sampai titik pendaratan dipublikasikan dan digunakan oleh misi drone.
Penjelasan mengikuti implementasi aktual pada source code, termasuk urutan
operasi, persamaan, ambang batas, dan nilai parameter default.

Jalur utama yang dijelaskan adalah:

```text
point cloud kamera
    -> preprocessing
    -> segmentasi GNG CPU
    -> pemetaan permukaan dan obstacle
    -> pencarian cakram landing
    -> penggabungan kandidat antar-frame
    -> penilaian dan penguncian kandidat
    -> titik landing global
```

Jalur RANSAC dibahas sebagai alternatif karena menghasilkan keluaran bidang
yang berbeda dari jalur GNG CPU.

## 1. Input Sensor dan Sistem Koordinat

Depth camera menghasilkan sekumpulan titik tiga dimensi. Setiap titik dapat
ditulis sebagai:

```text
p_i = (x_i, y_i, z_i)
```

Pada simulasi, point cloud dikirim ke ROS 2 melalui bridge dengan bentuk:

```text
sensor_msgs/msg/PointCloud2
```

dan frame kamera:

```text
camera_link
```

Posisi drone berasal dari odometri PX4. Odometri PX4 memakai konvensi NED,
sedangkan pemrosesan ROS memakai ENU. Konversinya adalah:

```text
x_ENU = y_NED
y_ENU = x_NED
z_ENU = -z_NED
```

Transformasi yang tersedia adalah:

```text
map -> odom -> base_link -> camera_link
```

Dengan transformasi ini, titik yang awalnya diukur relatif terhadap kamera
dapat dipindahkan ke frame global `map`. Langkah ini penting karena kamera
bergerak bersama drone. Tanpa frame global, satu bagian tanah yang sama akan
terlihat sebagai kandidat yang berpindah pada setiap frame.

Semua node perception memakai waktu simulasi yang sama dengan point cloud dan
TF. Ketidaksesuaian waktu dapat menyebabkan transformasi pada timestamp cloud
tidak ditemukan.

## 2. Preprocessing Point Cloud untuk GNG CPU

### 2.1 Membaca dan membersihkan titik

Node GNG CPU membaca hanya koordinat `x`, `y`, dan `z`. Titik NaN dilewati.
Setelah itu data disusun menjadi matriks:

```text
X = [x_1, x_2, ..., x_B]
```

dengan setiap baris:

```text
x_b = (x_b, y_b, z_b)
```

Jika jumlah titik kurang dari tiga, frame tidak diproses.

### 2.2 Voxel downsampling

Jumlah titik dikurangi menggunakan voxel grid. Untuk ukuran voxel `l`, indeks
voxel sebuah titik dihitung dengan:

```text
v(p) = floor(p / l)
```

Artinya:

```text
v_x = floor(x / l)
v_y = floor(y / l)
v_z = floor(z / l)
```

Titik-titik dengan indeks voxel yang sama dianggap berada pada satu sel. Kode
mempertahankan titik pertama yang masuk ke setiap voxel, bukan menghitung
rata-rata titik dalam voxel.

Nilai default:

```text
l = 0.15 m
```

Point cloud hasil downsampling menjadi masukan untuk GNG. Header dan timestamp
point cloud asli dipertahankan untuk keluaran dan pencocokan TF.

## 3. Segmentasi Bidang dengan GNG CPU

GNG digunakan untuk merangkum bentuk point cloud menjadi graph yang terdiri
dari node dan edge. Setiap node memiliki posisi tiga dimensi:

```text
W_i = (W_ix, W_iy, W_iz)
```

Konfigurasi yang dipakai oleh node CPU adalah:

```text
jumlah maksimum node       = 550
alpha                      = 0.5
beta                       = 0.03
delta                      = 0.7
rho                        = 0.7
epsilon                    = 0.0001
minimum cluster            = 20 node
radius tetangga normal     = 3
sumbu normal acuan         = (1, 0, 0)
batas sudut normal         = 10 derajat
batas planarity            = 0.0001
```

### 3.1 Inisialisasi node

Pada awal pemrosesan, data diacak dan dipakai untuk membentuk sepuluh pasang
node awal. Untuk setiap pasangan:

1. Satu titik dipilih sebagai node pertama.
2. Titik lain yang relatif jauh dari node tersebut dipilih sebagai node kedua.
3. Kedua node dihubungkan dengan satu edge.

Setelah inisialisasi, posisi node disimpan dalam matriks `W`, edge disimpan
sebagai pasangan indeks node, dan error setiap node diatur ke nol.

### 3.2 Jarak titik ke node

Untuk setiap titik `x_b` dan node `W_i`, jarak dihitung sebagai:

```text
d_bi = sqrt(||x_b - W_i||^2 + epsilon)
```

Dalam bentuk koordinat:

```text
d_bi = sqrt(
    (x_b - W_ix)^2
  + (y_b - W_iy)^2
  + (z_b - W_iz)^2
  + epsilon
)
```

Untuk setiap titik ditentukan:

```text
s1(b) = indeks node dengan jarak terkecil
s2(b) = indeks node dengan jarak terkecil kedua
```

`s1` adalah node pemenang pertama atau BMU (*best matching unit*), sedangkan
`s2` adalah node pemenang kedua.

### 3.3 Pengumpulan error dan perpindahan node

Untuk node `i`, error dikumpulkan dari titik-titik yang memilih node tersebut
sebagai BMU:

```text
E_i <- E_i + alpha * sum(d_bi)
```

Perpindahan node pemenang pertama dihitung dari selisih antara jumlah titik
yang ditugaskan dan posisi node saat ini:

```text
DeltaW1_i <- DeltaW1_i
           + alpha * (
               sum(x_b untuk s1(b)=i)
               - n1_i * W_i
             )
```

dengan `n1_i` adalah jumlah titik yang memilih node `i` sebagai BMU.

Node yang terhubung dengan BMU menerima pembaruan kedua:

```text
DeltaW2_i <- DeltaW2_i
           + beta * (
               sum(x_b untuk s1(b) bertetangga dengan i)
               - n2_i * W_i
             )
```

dengan `n2_i` adalah jumlah kontribusi titik dari node-node tetangga.

Setelah seluruh titik dalam satu batch diproses, posisi node diperbarui:

```text
W_i <- W_i
     + DeltaW1_i / (A1_i + epsilon)
     + DeltaW2_i / (A2_i + epsilon)
```

`A1_i` dan `A2_i` adalah jumlah aktivasi BMU dan aktivasi tetangga.

### 3.4 Pembentukan koneksi graph

Untuk setiap titik, node BMU pertama dan BMU kedua dianggap sebagai pasangan
yang aktif. Matriks koneksi batch diperbarui apabila pasangan node tersebut
muncul pada data:

```text
S_ij <- S_ij + 1
```

Setelah batch selesai, edge graph dibuat dari semua pasangan dengan nilai
koneksi lebih besar dari nol. Node yang tidak memiliki edge dapat dihapus.

Error node kemudian diluruhkan:

```text
E_i <- delta * E_i
```

Node yang tidak aktif juga dapat dihapus secara berkala. Jika jumlah node
belum mencapai batas maksimum, node baru dapat ditambahkan pada area dengan
error tinggi. Nilai threshold error ditentukan dari kuantil ke-85:

```text
threshold = quantile(E, 0.85)
```

Untuk edge antara node `q1` dan `q2` dengan error tinggi, node baru `q3`
ditempatkan di tengah:

```text
W_q3 = (W_q1 + W_q2) / 2
```

Error node lama dikurangi dan error node baru diatur sebagai:

```text
E_q1 <- rho * E_q1
E_q2 <- rho * E_q2
E_q3 <- (E_q1 + E_q2) / 2
```

Edge lama `q1-q2` diganti menjadi dua edge:

```text
q1-q3 dan q2-q3
```

### 3.5 Estimasi planarity dan normal

Graph dibagi menjadi connected component. Hanya component dengan minimal 20
node yang dipertimbangkan sebagai calon permukaan.

Untuk satu component, centroid node dihitung:

```text
mu = (1 / N) * sum(W_i)
```

Setiap node dipusatkan terhadap centroid:

```text
d_i = W_i - mu
```

Kemudian dibentuk matriks kovarians:

```text
C = covariance({d_i})
```

Eigenvalue matriks kovarians diurutkan:

```text
lambda_0 <= lambda_1 <= lambda_2
```

Eigenvector yang berhubungan dengan `lambda_0` dipakai sebagai estimasi normal
permukaan. Nilai planarity dihitung dengan:

```text
planarity = lambda_0 / (lambda_0 + lambda_1 + lambda_2)
```

Component dianggap datar jika:

```text
planarity < 0.0001
```

Selain datar, arah normal harus sesuai dengan sumbu acuan `a = (1, 0, 0)`.
Pengecekan sudut dilakukan menggunakan:

```text
abs(n dot a) >= cos(10 derajat)
```

atau secara ekuivalen:

```text
theta = arccos(abs(n dot a))
theta <= 10 derajat
```

Kode juga menghitung normal lokal setiap node dengan tetangga graph sampai
radius tiga. Sebuah node dipertahankan sebagai node datar jika node tersebut
berada dalam component datar atau normal lokalnya memenuhi batas sudut.

### 3.6 Pembentukan cloud bidang dan outlier

Setiap titik input telah dipetakan ke BMU. Semua titik yang terhubung ke node
datar dikumpulkan menjadi cloud bidang:

```text
plane = titik yang BMU-nya node datar
```

Cloud outlier adalah komplemennya:

```text
outlier = semua titik - plane
```

Jalur GNG CPU menerbitkan:

```text
/plane_cpu
/outlier_cpu
```

Cloud bidang menjadi masukan pencarian area landing. Cloud outlier dipakai
kembali setelah target landing terkunci untuk mencari obstacle yang berdiri di
atasnya.

## 4. Alternatif Segmentasi RANSAC

RANSAC adalah jalur alternatif untuk memisahkan bidang dan outlier. Ia tidak
memilih titik landing; ia hanya menggantikan tahap segmentasi bidang.

Urutan RANSAC adalah:

```text
point cloud
    -> hitung jumlah titik valid
    -> PassThrough pada sumbu z
    -> voxel downsampling
    -> estimasi normal
    -> SAC-RANSAC normal plane
    -> ekstraksi inlier dan outlier
```

Konfigurasi RANSAC yang dipakai launch saat ini adalah:

```text
voxel leaf              = 0.15 m
PassThrough             = -1000 sampai 1000 m
jumlah tetangga normal  = 30
model                   = SACMODEL_NORMAL_PLANE
normal distance weight  = 0.17
distance threshold      = 0.097 m
maximum iteration       = 650
sumbu bidang            = (1, 0, 0)
batas sudut normal     = 5 derajat
```

RANSAC menerbitkan:

```text
/plane
/outlier
```

Sedangkan konfigurasi default pencari landing membaca keluaran GNG CPU,
yaitu `/plane_cpu` dan `/outlier_cpu`. Oleh karena itu, jika RANSAC digunakan,
input pencari landing harus diarahkan secara eksplisit ke `/plane` dan
`/outlier`.

## 5. Transformasi Cloud ke Bidang dan Peta

Node pencari landing menerima cloud bidang. Untuk setiap frame dilakukan dua
transformasi utama.

Pertama, cloud dipindahkan dari frame asalnya ke frame bidang `camera_link`:

```text
p_plane = T_plane_from_cloud * p_cloud
```

Kedua, centroid kandidat dipindahkan dari frame bidang ke frame global:

```text
p_map = T_map_from_plane * p_plane
```

Transformasi dicari pada timestamp milik point cloud. Jika transformasi tidak
tersedia, frame tidak digunakan untuk membuat keputusan landing.

### 5.1 Proyeksi 3D ke 2D

Pencarian cakram dilakukan pada grid dua dimensi. Dengan pilihan bidang
default `yz`, proyeksinya adalah:

```text
u = y_plane
v = z_plane
```

Pilihan lain yang tersedia adalah:

```text
plane xy: u = x_plane, v = y_plane
plane xz: u = x_plane, v = z_plane
```

Untuk kamera yang sumbu kedalamannya adalah `x`, bidang tanah berada pada
`y-z`, sehingga pilihan default adalah `yz`.

## 6. Pembuatan Grid dan Cakram Landing

### 6.1 Batas grid

Rentang `u` dan `v` diambil dari seluruh titik cloud. Sebelum grid dibuat,
masing-masing batas diperluas dengan margin:

```text
margin = 0.25 m
```

Jika rentangnya `[u_min, u_max]` dan `[v_min, v_max]`, maka:

```text
u_min <- u_min - 0.25
u_max <- u_max + 0.25
v_min <- v_min - 0.25
v_max <- v_max + 0.25
```

Ukuran satu sel grid adalah:

```text
cell = 0.6 m
```

Jumlah kolom dan baris dihitung dengan:

```text
cols = max(1, ceil((u_max - u_min) / cell))
rows = max(1, ceil((v_max - v_min) / cell))
```

Jika jumlah sel melebihi 10.000.000, frame dihentikan untuk mencegah alokasi
grid yang terlalu besar.

### 6.2 Histogram titik per sel

Indeks sel untuk satu titik `(u, v)` dihitung sebagai:

```text
i_x = floor((u - u_min) / cell)
i_y = floor((v - v_min) / cell)
```

Jumlah titik dalam sel disimpan pada:

```text
count_raw[i_x, i_y]
```

### 6.3 Menutup gap kecil

Jika fitur penutupan gap aktif, sebuah sel yang kosong dapat diisi sementara
jika minimal lima dari delapan tetangganya pada jendela 3 x 3 terisi:

```text
count[i_x, i_y] = 1
```

Syaratnya:

```text
count_raw[i_x, i_y] = 0
dan jumlah tetangga terisi >= 5
```

Hasil penutupan gap dipakai untuk mencari pusat cakram. Akan tetapi, peta
obstacle dan pengujian cakupan tetap menggunakan `count_raw`, karena sel yang
diisi oleh gap closing bukan bukti langsung dari sensor.

### 6.4 Mask cakram

Diameter area landing ditetapkan sebesar:

```text
D = 1.3 m
```

Radius fisiknya adalah:

```text
r_safe = D / 2 = 0.65 m
```

Radius dalam satuan sel dihitung dengan:

```text
R_safe_cells = max(1, round(r_safe / cell))
```

Dengan nilai default:

```text
R_safe_cells = max(1, round(0.65 / 0.6)) = 1
```

Mask disk berisi semua offset sel `(dx, dy)` yang memenuhi:

```text
dx^2 + dy^2 <= R_safe_cells^2
```

Untuk `R_safe_cells = 1`, mask terdiri dari lima sel: pusat dan empat sel
ortogonal.

## 7. Validasi Geometri Kandidat

### 7.1 Fill ratio

Untuk setiap kemungkinan pusat `(i_x, i_y)`, program menghitung berapa sel
dalam mask cakram yang memiliki minimal dua titik:

```text
filled_cells = jumlah sel disk dengan count[i_x, i_y] >= 2
```

Rasio pengisian dihitung sebagai:

```text
fill_ratio = filled_cells / disk_area
```

Sebuah pusat lolos jika:

```text
fill_ratio >= 0.95
```

Pusat yang terlalu dekat dengan tepi grid dilewati agar seluruh cakram masih
dapat dievaluasi.

### 7.2 Peta obstacle

Peta obstacle dibangun dari hitungan titik mentah. Sel dianggap obstacle atau
tidak cukup teramati jika:

```text
count_raw[i_x, i_y] < 2
```

Obstacle kemudian diperbesar menggunakan dilasi 8-neighbor. Dengan satu kali
iterasi, sel kosong yang memiliki obstacle pada salah satu dari delapan arah
tetangganya ikut ditandai sebagai obstacle.

Nilai default inflasi:

```text
inflate_cells = 1
```

### 7.3 Distance transform dan clearance

Semua sel obstacle dimasukkan sebagai sumber dengan jarak nol. Jarak ke sel
lain dihitung menggunakan BFS 8-neighbor:

```text
dist[i_x, i_y] = jumlah langkah grid menuju obstacle terdekat
```

Jika tidak ada obstacle sama sekali, jarak awal dibuat cukup besar untuk
menganggap seluruh grid bebas.

Clearance fisik sebuah pusat dihitung sebagai:

```text
clearance_m = dist[i_x, i_y] * cell - r_safe
```

Sebuah kandidat hanya diterima jika seluruh sel dalam mask cakramnya berada di
luar peta obstacle.

### 7.4 Non-maximum suppression

Kandidat yang lolos diurutkan dari clearance terbesar ke terkecil. Kandidat
diterima satu per satu. Kandidat berikutnya dibuang jika terlalu dekat dengan
kandidat yang sudah diterima:

```text
(u_a - u_b)^2 + (v_a - v_b)^2 < s^2
```

dengan:

```text
s = 1.3 m
```

Jumlah kandidat per frame dibatasi:

```text
maksimum = 10 kandidat
```

Tahap ini menghasilkan beberapa area landing yang berbeda, bukan hanya satu
pemenang.

## 8. Centroid dan Kekasaran Permukaan

### 8.1 Centroid cakram

Untuk setiap kandidat, semua titik bidang yang jaraknya pada bidang proyeksi
kurang dari `r_safe` dikumpulkan:

```text
(u_i - u_c)^2 + (v_i - v_c)^2 <= r_safe^2
```

Jika terdapat `n` titik, centroid tiga dimensinya dihitung dengan:

```text
c_plane = (1 / n) * sum(p_i)
```

Centroid kemudian dipindahkan ke frame global:

```text
c_map = T_map_from_plane * c_plane
```

Kandidat tanpa satu pun titik dalam cakram dianggap tidak valid.

### 8.2 Roughness

Roughness diukur dari titik-titik dalam cakram yang sama. Minimal empat titik
diperlukan. Tiga titik selalu dapat membentuk satu bidang secara tepat, sehingga
kurang dari empat titik tidak memberikan pengukuran yang bermakna.

Rata-rata titik:

```text
mu = (1 / n) * sum(p_i)
```

Kovarians:

```text
C = (1 / n) * sum((p_i - mu)(p_i - mu)^T)
```

Eigenvalue kovarians diurutkan:

```text
lambda_0 <= lambda_1 <= lambda_2
```

Roughness dihitung dengan:

```text
roughness_m = sqrt(max(0, lambda_0))
```

Nilai ini mengukur seberapa jauh titik-titik menyimpang dari bidang terbaik.
Permukaan miring tetapi halus tetap dapat mempunyai roughness kecil, karena
kemiringan tidak dicampur ke dalam pengukuran roughness.

## 9. Registry Kandidat Antar-Frame

Kandidat dari satu frame disimpan dalam registry global agar keputusan tidak
bergantung pada satu pengukuran.

### 9.1 Pencocokan kandidat

Satu kandidat baru `c` dicocokkan dengan entri lama `e` jika jarak 3D memenuhi:

```text
||e.center - c.center||^2 <= merge_dist_m^2
```

Nilai default:

```text
merge_dist_m = 1.0 m
```

Jika cocok, metrik geometri terbaru disalin ke entri lama. Posisi entri
dibekukan secara default. Jika pembaruan posisi diaktifkan, posisi dapat
diperbarui dengan exponential moving average:

```text
center_new = (1 - ema) * center_old + ema * center_observed
```

Kapasitas registry dibatasi sampai 50 entri. Jika kapasitas terlampaui, entri
yang paling lama tidak terlihat dihapus, kecuali target yang sedang dipakai
sedang dipin.

### 9.2 Penggabungan roughness

Roughness yang valid dirata-ratakan antar-frame. Jika `r_old` telah memiliki
`n_old` sampel dan pengukuran baru adalah `r_new`, maka:

```text
r_average = (r_old * n_old + r_new) / (n_old + 1)
```

Pengukuran yang tidak valid tidak dimasukkan sebagai nol. Dengan demikian,
permukaan yang belum terukur tidak dianggap lebih rata daripada permukaan yang
sudah terukur.

### 9.3 Status blocked dan pemulihan

Entri lama hanya dihukum jika dua kondisi terpenuhi:

1. Kandidat seharusnya berada dalam jangkauan pandang frame tersebut.
2. Cakramnya benar-benar tersentuh data sensor.

Cakupan cakram dihitung dari jumlah sel mentah yang memiliki minimal satu titik:

```text
coverage = jumlah sel disk dengan count_raw > 0 / disk_area
```

Frame dianggap memiliki bukti yang cukup untuk mengevaluasi kandidat jika:

```text
coverage >= 0.6
```

Jika kandidat tidak terlihat, tetapi memiliki bukti observasi yang cukup, waktu
blocked bertambah:

```text
blocked_s <- blocked_s + dt
```

Kandidat tidak boleh dipilih ketika:

```text
blocked_s >= 3.0 s
```

Jika kandidat kembali terlihat, waktu blocked dikurangi dengan laju:

```text
recover_rate = block_after_s / recover_s
blocked_s <- max(0, blocked_s - dt * recover_rate)
```

Dengan nilai default, `recover_rate = 3.0` sehingga satu detik pengamatan
aman dapat memulihkan tiga detik status blocked.

Jika blocked berlangsung sampai 30 detik, entri dihapus permanen:

```text
blocked_s >= 30.0 s -> hapus entri
```

Saat drone sedang bergerak translasi, `dt` untuk registry dibuat nol. Tujuannya
agar pergeseran grid akibat gerakan kamera tidak dianggap sebagai bukti bahwa
permukaan menghilang atau menjadi obstacle.

## 10. Penilaian dan Pemilihan Kandidat

### 10.1 Batas jarak area pencarian

Jarak terhadap pusat area pencarian dihitung secara horizontal, bukan jarak
3D:

```text
d_horizontal = sqrt(
    (c_x - ref_x)^2
  + (c_y - ref_y)^2
)
```

Kandidat hanya boleh dipilih jika:

```text
d_horizontal <= 8.0 m
```

Batas ini adalah filter keras. Kandidat di luar area tidak masuk ke tahap
scoring.

### 10.2 Suku jarak

Suku jarak dinormalisasi ke rentang 0 sampai 1:

```text
distance_term = clamp(1 - d_horizontal / distance_reference, 0, 1)
```

Karena radius pencarian aktif, `distance_reference` menggunakan 8 meter.
Kandidat tepat di pusat area bernilai 1, sedangkan kandidat di tepi area
bernilai 0.

### 10.3 Suku roughness

Jika roughness telah terukur, sukunya adalah:

```text
roughness_term = clamp(1 - roughness_m / 0.10, 0, 1)
```

Jika roughness belum pernah terukur, maka:

```text
roughness_term = 0
```

### 10.4 Skor akhir

Skor kandidat dihitung dengan bobot default yang sama:

```text
score = 0.5 * distance_term
      + 0.5 * roughness_term
```

Skor hanya digunakan untuk mengurutkan kandidat yang sudah lolos seluruh
syarat keselamatan. Kandidat blocked, kandidat di luar radius, dan kandidat
yang baru ditolak tidak ikut dinilai.

Sebelum target dikunci, kandidat dengan skor tertinggi dipilih. Setelah target
dikunci dan mode sticky aktif, target lama dipertahankan selama masih valid.
Target hanya boleh diganti jika:

```text
target lama blocked
target lama keluar radius
target lama ditolak oleh misi
target lama sudah tidak ada di registry
```

Jika mode sticky tidak aktif, kandidat baru harus mengungguli kandidat lama
dengan selisih skor lebih besar dari:

```text
score_hysteresis = 0.05
```

## 11. Commit Gate dan Titik Final

Kandidat yang baru ditemukan belum langsung diumumkan sebagai titik landing.
Program mengumpulkan pengamatan kandidat yang dapat dipilih selama:

```text
commit_after_s = 10 s
```

Waktu commit hanya bertambah saat kandidat valid benar-benar tersedia:

```text
elapsed <- elapsed + dt
```

Jika tidak ada kandidat valid, waktu tidak bertambah. Jika drone sedang
bergerak, `dt` untuk proses pengumpulan dibuat nol.

Setelah:

```text
elapsed >= 10 s
```

dan masih ada kandidat valid, keputusan dianggap committed.

### 11.1 Perilaku sebelum commit

Sebelum commit:

```text
TF titik landing final       belum diterbitkan
koordinat pusat final        belum diterbitkan
safe cloud final             kosong
marker kandidat              tetap diterbitkan
```

Marker tetap diterbitkan supaya operator dapat melihat proses pengumpulan
kandidat.

### 11.2 Perilaku setelah commit

Setelah commit dan kandidat terpilih tersedia, program menerbitkan:

```text
map -> safety_point
```

Translasi frame tersebut adalah:

```text
(selected_x, selected_y, selected_z)
```

Pada frame yang sama, pusat target diproyeksikan kembali ke bidang kamera.
Semua titik dalam radius `0.65 m` dari pusat tersebut dikumpulkan dan
diterbitkan sebagai cloud area aman.

Setelah commit, pencarian kandidat baru dapat dibekukan. Target lama tetap
dipantau karena frame `safety_point` harus terus diperbarui agar konsumen tidak
menganggap target basi.

## 12. Pemeriksaan Obstacle pada Target Terkunci

Cloud outlier terakhir disimpan dan digunakan oleh pencari landing. Cloud
tersebut hanya dipakai jika usianya tidak lebih dari:

```text
outlier_max_age = 1.0 s
```

Outlier dipindahkan ke frame `map`. Sebuah outlier dihitung sebagai obstacle di
atas target jika berada dalam silinder vertikal:

```text
horizontal_distance <= 0.65 m
0.20 m <= p_z - center_z <= 3.00 m
```

Secara matematis:

```text
h = p_z - center_z

obstacle_point jika:
    0.20 <= h <= 3.00
dan
    (p_x - center_x)^2 + (p_y - center_y)^2 <= 0.65^2
```

Jumlah titik yang memenuhi syarat disebut `N_obstacle`. Target dianggap
terhalang jika:

```text
N_obstacle >= 3
```

Saat terdeteksi, akumulator waktu diperbarui:

```text
target_blocked_s <- target_blocked_s + dt
```

Jika pada frame berikutnya tidak ditemukan obstacle, akumulator dikurangi:

```text
target_blocked_s <- max(0, target_blocked_s - dt)
```

Target dilepas jika obstacle bertahan selama:

```text
target_blocked_s >= 1.5 s
```

Ketika target dilepas pada mode freeze, seluruh registry dikosongkan dan proses
commit dimulai kembali dari nol. Hal ini mencegah program langsung memilih
kandidat lama yang dikumpulkan dari kondisi ketinggian atau sudut pandang yang
berbeda.

## 13. Publikasi Hasil

### 13.1 Kandidat untuk visualisasi

Semua entri registry diterbitkan sebagai silinder tipis berdiameter `1.3 m`.

```text
hijau   = kandidat masih boleh dipilih
oranye  = kandidat sedang blocked
```

Teks di atas kandidat memuat skor dan roughness. Kandidat yang sedang dipilih
tidak memakai warna khusus; posisi target final ditunjukkan oleh frame
`safety_point`.

### 13.2 Cloud area terpilih

Cloud titik dalam cakram target diterbitkan sebagai cloud berwarna hijau.
Cloud ini hanya berisi titik-titik yang memenuhi:

```text
(u_i - u_selected)^2 + (v_i - v_selected)^2 <= 0.65^2
```

### 13.3 Koordinat dan frame target

Titik pusat diterbitkan dalam bentuk koordinat tiga dimensi pada frame `map`.
Frame dinamis `map -> safety_point` adalah keluaran yang dibaca oleh misi.

Statistik per frame juga diterbitkan, termasuk:

```text
jumlah input dan titik valid
jumlah titik dalam cloud aman
jumlah kandidat dalam registry
jumlah kandidat blocked
status commit
skor kandidat terpilih
roughness kandidat
jumlah obstacle pada target
waktu target berada dalam kondisi blocked
```

## 14. Penggunaan Titik oleh Misi Drone

Pencarian landing dimulai ketika drone telah sampai di waypoint. Misi
menerbitkan posisi waypoint sebagai pusat area pencarian, kemudian menunggu
frame `safety_point`.

Urutannya adalah:

```text
terbang ke waypoint
    -> mulai pengamatan landing
    -> tunggu commit gate
    -> baca map -> safety_point
    -> kunci koordinat target
    -> terbang horizontal ke atas target
    -> turun secara bertahap
    -> kirim NAV_LAND ke PX4
```

Target dianggap hilang jika frame `safety_point` tidak ada atau sudah lebih tua
dari:

```text
3.5 s
```

Saat masih cukup tinggi, target yang hilang atau berpindah lebih dari `1.0 m`
dapat menyebabkan misi membatalkan pendekatan dan kembali mencari. Setelah
drone berada di bawah ketinggian komit sekitar `4.0 m` di atas permukaan dan
masih mengikuti setpoint, misi menyelesaikan descent agar tidak terus-menerus
membatalkan pendaratan ketika kamera sudah tidak dapat melihat area secara
lengkap.

Kecepatan penurunan setpoint adalah:

```text
0.4 m/s
```

Drone diturunkan sampai sekitar:

```text
1.0 m di atas permukaan target
```

Kemudian misi mengirim perintah `NAV_LAND`. Setelah perintah tersebut diterima,
misi berhenti mengirim setpoint posisi agar tidak melawan pengendali landing
PX4.

## 15. Ringkasan Persamaan Utama

Persamaan yang menentukan hasil akhir adalah:

### Downsampling

```text
voxel(p) = floor(p / voxel_leaf)
```

### Jarak GNG

```text
d(x, W_i) = sqrt(||x - W_i||^2 + epsilon)
```

### Planarity

```text
planarity = lambda_min / sum(lambda)
```

### Pusat cakram

```text
centroid = (1 / n) * sum(point_i)
```

### Roughness

```text
roughness = sqrt(lambda_min(covariance))
```

### Fill ratio

```text
fill_ratio = filled_cells / disk_area
```

### Clearance

```text
clearance = distance_transform * grid_cell - safe_radius
```

### Suku jarak

```text
distance_term = clamp(1 - d_horizontal / distance_reference, 0, 1)
```

### Suku roughness

```text
roughness_term = clamp(1 - roughness / roughness_max, 0, 1)
```

### Skor kandidat

```text
score = w_distance * distance_term
      + w_roughness * roughness_term
```

### Obstacle di atas target

```text
horizontal_distance <= safe_radius
min_height <= p_z - center_z <= max_height
```

## 16. Referensi Implementasi

Bagian-bagian utama pipeline berada pada file berikut:

```text
src/gz_bridge_ros2/launch/depth_bridge_launch.py
src/gz_bridge_ros2/src/drone_kinematic.cpp
src/gng_node/gng_node/dbl_gng_cpu_node.py
src/gng_node/gng_node/dbl_gng_cpu.py
src/segmentation_node/src/plane_segmentation_ransac.cpp
src/segmentation_node/src/landing_circle.cpp
src/segmentation_node/include/segmentation_node/grid_coverage.hpp
src/segmentation_node/include/segmentation_node/plane_roughness.hpp
src/segmentation_node/include/segmentation_node/obstacle_probe.hpp
src/segmentation_node/include/segmentation_node/landing_score.hpp
src/segmentation_node/include/segmentation_node/landing_registry.hpp
src/segmentation_node/src/landing_registry.cpp
src/segmentation_node/include/segmentation_node/commit_gate.hpp
src/offboard_mission/offboard_mission/waypoint_node.py
```
