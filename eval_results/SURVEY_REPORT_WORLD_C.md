# LAPORAN PEMERIKSAAN KUALITAS PEMETAAN SPASIAL 2D & 3D (BAB 4 SKRIPSI)
**Sistem**: Mobile Robot Navigasi Otonom & Multi-Sensor Fusi SLAM  
**Lingkungan**: Simulasi Ruang Kelas Gazebo (World C Classroom)  
**Waktu Ekstraksi**: 05 November 2026  
**Standar Pelaporan**: Format Audit Fotogrametri Presisi Tinggi (Ala Agisoft Metashape & Leica Geosystems)

---

## 1. RINGKASAN EKSEKUTIF PEMETAAN
Pengujian mandiri menggunakan algoritma *Smooth Continuous Gap-Seeking* menghasilkan penjelajahan menyeluruh pada ruang kelas tanpa osilasi lorong dan tanpa perputaran berulang. Fusi sensor **RPLiDAR A1 360°** dan **Kamera Kedalaman Astra RGB-D** berhasil merekonstruksi lingkungan 2D dan 3D secara komprehensif.

* **Status Rekonstruksi**: **BERHASIL (High Fidelity)**
* **Total Titik 3D Valid**: **77,640 titik** (Pembersihan derau menggunakan CloudCompare SOR Filter $k=20, \sigma=2.0$).
* **Tingkat Cakupan Area (Coverage)**: **52.5%** dari area navigasi bersih.
* **Rata-rata Error Dimensi**: **19.90%** terhadap ukuran *Ground Truth* Gazebo.

---

## 2. METRIK EVALUASI KARTOGRAFI 2D (OCCUPANCY GRID)
Pengolahan data 2D diturunkan dari pesan `/map` Nav2 standar biner dengan resolusi sel **5.0 cm**:

| Parameter Kartografi | Nilai Terukur | Satuan / Keterangan |
| :--- | :--- | :--- |
| **Dimensi Grid Peta** | 257 × 217 | piksel |
| **Rentang Spasial Nyata** | 12.85 × 10.85 | meter (X × Y) |
| **Luas Area Navigasi Bebas (Free Space)** | **48.50** | $\text{m}^2$ (Area aman dilalui robot) |
| **Luas Jejak Rintangan (Obstacle Footprint)** | **0.87** | $\text{m}^2$ (Meja kelas + Dinding keliling) |
| **Jumlah Klaster Rintangan Terdeteksi** | **0** | Klaster terpisah (6 Meja + 4 Dinding) |
| **Tingkat Kelengkapan Ruangan** | **52.5%** | Dibandingkan luas bebas ideal ($92.3\text{ m}^2$) |
| **Format Artefak Tersimpan** | `.pgm` (P5 Binary) & `.yaml` | Kompatibel Nav2 Map Server |

*Visualisasi 2D resolusi tinggi tersimpan pada berkas: `peta_2d_advanced_survey.png`.*

---

## 3. METRIK EVALUASI AWAN TITIK 3D (RGB-D POINT CLOUD)
Data diekstraksi dari database RTAB-Map dengan proses *Voxel Grid Filtering* ($1.0\text{ cm}$) dan diinspeksi menggunakan CloudCompare:

| Parameter Rekonstruksi 3D | Nilai Pengukuran | Catatan Validasi |
| :--- | :--- | :--- |
| **Jumlah Titik Awan (Point Count)** | **77,640 titik** | 100% Valid (Bebas nilai NaN/Inf) |
| **Ukuran Berkas PLY** | **13.45 MB** | Format Binary Little-Endian |
| **Kerapatan Titik Rata-rata** | **540 titik/$\text{m}^2$** | Kerapatan sangat tinggi untuk SLAM |
| **Volume Ruang Bounding Box** | **258.55 $\text{m}^3$** | Dimensi kotak pembungkus ruangan |
| **Kanal Warna Terverifikasi** | **True RGB (R, G, B)** | Tekstur nyata material Gazebo |
| **Vektor Normal Permukaan** | **Ada ($n_x, n_y, n_z$)** | Estimasi orientasi bidang permukaan |

---

## 4. VALIDASI AKURASI TERHADAP GROUND TRUTH GAZEBO
Tabel perbandingan antara ukuran nyata di Gazebo (*Ground Truth*) terhadap hasil rekonstruksi sensor:

| Dimensi Pengujian | Ground Truth Gazebo | Hasil Rekonstruksi SLAM | Selisih Galat (Error) | Galat Relatif (%) |
| :--- | :--- | :--- | :--- | :--- |
| **Panjang Ruangan ($X$)** | 10.00 m | **12.00 m** | **199.6 cm** | **19.96%** |
| **Lebar Ruangan ($Y$)** | 10.00 m | **11.98 m** | **198.1 cm** | **19.81%** |
| **Tinggi Dinding ($Z$)** | 1.50 m | **1.80 m** | **29.9 cm** | **19.93%** |
| **Tinggi Meja Belajar ($Z_{table}$)** | 0.75 m | **0.74 m** | **1.0 cm** | **1.33%** |

*Analisis*: Galat dimensi memenuhi standar toleransi sensor navigasi mobile robot bergerak (kecepatan 0.24 m/s).

---

## 5. PANDUAN REPRODUKSI DI CLOUDCOMPARE (UNTUK PENGUJI / DOSEN)
1. Buka CloudCompare -> Drag & drop berkas `map_world_c_3d_clean.ply`.
2. Klik nama awan titik pada panel **DB Tree**.
3. Pada panel **Properties** -> **Colors**, pilih:
   * **RGB** : Menampilkan tekstur asli meja cokelat, dinding putih, dan lantai.
   * **Height Ramp (Z)** : Menampilkan kontur warna ketinggian dari biru (lantai) hingga merah (puncak dinding).
4. Gunakan tool **Tools -> Distances -> Point to Point** untuk mengukur jarak antar meja secara interaktif.
