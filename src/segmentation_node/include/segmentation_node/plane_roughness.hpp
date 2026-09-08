// file: plane_roughness.hpp
//
// Seberapa KASAR permukaan di dalam satu cakram pendaratan.
//
// Diukur sebagai RMS jarak titik-titik ke bidang yang paling pas melewatinya,
// yaitu akar nilai eigen TERKECIL dari matriks kovarians 3x3 titik-titik itu.
// Untuk bidang terbaik yang melewati centroid, rata-rata kuadrat residualnya
// persis sama dengan nilai eigen terkecil bila kovariansnya dibagi n — jadi
// roughness = sqrt(lambda0), bersatuan meter.
//
// Sengaja INVARIAN TERHADAP KEMIRINGAN: permukaan miring tapi mulus terbaca
// mulus. Kemiringan adalah pertanyaan lain, dengan ambang lain, dan
// menggabungkan keduanya ke satu angka akan membuat keduanya tidak bisa
// disetel sendiri-sendiri.
//
// Butuh minimal EMPAT titik. Tiga titik menentukan sebuah bidang secara
// persis, sehingga residualnya selalu nol — angka yang terlihat sempurna
// justru karena tidak ada informasi di dalamnya. Cakram yang lebih miskin dari
// itu menghasilkan sampel TIDAK SAH, bukan sampel bernilai nol; membedakan
// keduanya penting karena "belum terukur" tidak boleh mengalahkan
// "terukur dan mulus".
//
// Bebas ROS dan PCL supaya bisa diuji tanpa menyalakan node.

#pragma once

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <vector>

struct RoughnessResult {
  float roughness_m = 0.f;
  bool  valid       = false;
};

// `pts` boleh berada di frame mana pun: roughness tidak berubah oleh rotasi
// maupun translasi, jadi tidak ada keharusan memindahkannya ke `map` dulu.
inline RoughnessResult planeRoughness(const std::vector<Eigen::Vector3f>& pts,
                                      int min_points = 4)
{
  RoughnessResult out;

  // Empat adalah lantai matematis, bukan pilihan selera — di bawah itu
  // hasilnya nol tanpa memandang bentuk permukaannya.
  const int floor_pts = std::max(4, min_points);
  const int n         = static_cast<int>(pts.size());
  if (n < floor_pts) return out;

  // Akumulasi dalam double: koordinat `map` bisa berorde puluhan meter
  // sedangkan residual yang dicari berorde sentimeter, dan kovarians
  // mengkuadratkan selisih itu.
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto& p : pts) mean += p.cast<double>();
  mean /= static_cast<double>(n);

  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (const auto& p : pts) {
    const Eigen::Vector3d d = p.cast<double>() - mean;
    cov += d * d.transpose();
  }
  cov /= static_cast<double>(n);   // dibagi n, bukan n-1: yang dicari RMS

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
  if (es.info() != Eigen::Success) return out;

  // eigenvalues() terurut menaik; yang terkecil adalah arah normal bidang.
  // Diklem di nol karena galat pembulatan bisa membuatnya sedikit negatif
  // untuk permukaan yang benar-benar datar.
  const double lambda0 = std::max(0.0, es.eigenvalues()(0));

  out.roughness_m = static_cast<float>(std::sqrt(lambda0));
  out.valid       = true;
  return out;
}
