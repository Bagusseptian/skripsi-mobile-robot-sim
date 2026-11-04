# LABSHEET 5: EVALUASI KUANTITATIF AKURASI LOKALISASI (ATE RMSE) DAN BEBAN KOMPUTASI SISTEM

**Mata Kuliah:** Penginderaan Visual Robot  
**Program Studi:** S1 Pendidikan Teknik Mekatronika, Universitas Negeri Yogyakarta  
**Penyusun:** Muhammad Bagus Septian  
**Alokasi Waktu:** 3 x 50 Menit (1 Pertemuan Praktikum)

---

## 1. Tujuan Pembelajaran
Setelah menyelesaikan praktikum ini, mahasiswa diharapkan mampu:
1. Memahami metodologi evaluasi performa algoritma SLAM secara objektif berbasis standar penelitian robotika internasional.
2. Mengukur galat lintasan absolut (*Absolute Trajectory Error / ATE RMSE*) antara estimasi pose SLAM terhadap posisi acuan ideal (*Ground Truth P3D Gazebo*).
3. Menganalisis beban komputasi sistem (*utilisasi CPU % dan konsumsi RAM MB*) dari berbagai algoritma pemetaan dan navigasi.
4. Membandingkan performa 3 konfigurasi secara empiris: (a) LiDAR 2D Saja, (b) Visual SLAM RGB-D Saja, dan (c) Fusi Multi-Sensor.
5. Mengekspor data ke dalam berkas CSV dan menghasilkan visualisasi grafik lintasan (`.png`) untuk penyusunan laporan hasil penelitian / tugas akhir.

---

## 2. Alat dan Bahan
1. PC / Laptop dengan OS Ubuntu 22.04 LTS (ROS 2 Humble).
2. Simulator Gazebo Classic dengan paket `robot_sim`.
3. Skrip evaluasi akurasi `benchmark_ate.py`.
4. Skrip evaluasi beban sistem `benchmark_system.py`.
5. Modul Python `pandas`, `numpy`, dan `matplotlib`.
6. Skenario dunia uji lorong berbelok `world_b_corridor.world`.

---

## 3. Dasar Teori Singkat

### A. Absolute Trajectory Error (ATE RMSE)
ATE merupakan metrik standar untuk mengevaluasi akurasi estimasi posisi global robot. Dengan membandingkan posisi estimasi SLAM ($p_{\text{slam}, i}$) terhadap posisi riil dari sensor Ground Truth Gazebo ($p_{\text{gt}, i}$) pada setiap stempel waktu $i$:
$$e_i = p_{\text{slam}, i} - p_{\text{gt}, i}$$
Nilai *Root Mean Square Error* (RMSE) dihitung dengan persamaan:
$$\text{RMSE}_{\text{ATE}} = \sqrt{\frac{1}{N} \sum_{i=1}^{N} \|e_i\|^2} = \sqrt{\frac{1}{N} \sum_{i=1}^{N} \left( (x_{\text{slam}, i} - x_{\text{gt}, i})^2 + (y_{\text{slam}, i} - y_{\text{gt}, i})^2 \right)}$$
Semakin kecil nilai RMSE, semakin tinggi tingkat presisi pemetaan dan estimasi posisi robot.

### B. Evaluasi Beban Komputasi Sistem
Pada sistem tertanam (*embedded system*) robot fisik (seperti NVIDIA Jetson Nano atau Raspberry Pi 4), sumber daya prosesor dan memori sangat terbatas. Oleh karena itu, pemilihan konfigurasi sensor harus menyeimbangkan antara akurasi tinggi dan efisiensi konsumsi komputasi:
- **LiDAR 2D (`slam_toolbox`):** Memproses data larik 1D (360 titik per detik) $\rightarrow$ Beban CPU rendah, RAM hemat.
- **Visual SLAM (`rtabmap` RGB-D):** Memproses citra matriks piksel $640 \times 480 \times 3$ beserta dense depth map pada 30 FPS $\rightarrow$ Beban CPU & RAM tinggi.
- **Fusi Sensor Penuh:** Melakukan sinkronisasi multi-threading antara scan laser, citra kamera, dan IMU $\rightarrow$ Menuntut manajemen memori yang efisien.

---

## 4. Langkah Kerja

### Langkah 1: Persiapan Lingkungan Uji (World B)
Buka **Terminal 1**, jalankan simulasi di lorong World B (sirkuit loop closure):
```bash
source /opt/ros/humble/setup.bash
source ~/skripsi_ws/install/setup.bash
ros2 launch robot_sim sim.launch.py world:=world_b
```

### Langkah 2: Pengujian Skenario 1 - LiDAR 2D Saja
1. Buka **Terminal 2**, jalankan 2D SLAM:
   ```bash
   source ~/skripsi_ws/install/setup.bash
   ros2 launch robot_sim slam_lidar.launch.py
   ```
2. Buka **Terminal 3**, jalankan pencatatan ATE selama 60 detik:
   ```bash
   source ~/skripsi_ws/install/setup.bash
   ros2 run robot_sim benchmark_ate.py --name uji_lidar --duration 60
   ```
3. Buka **Terminal 4**, kendalikan robot mengelilingi 1 putaran lorong dengan teleop WASD:
   ```bash
   source ~/skripsi_ws/install/setup.bash
   ros2 run robot_sim teleop_wasd.py
   ```
4. Bersamaan dengan itu, catat beban CPU dan RAM di terminal lain jika diperlukan:
   ```bash
   ros2 run robot_sim benchmark_system.py --name beban_lidar --duration 60
   ```
5. Simpan nilai RMSE, Mean Error, Max Error, serta file grafik lintasan `.png` yang dihasilkan.

### Langkah 3: Pengujian Skenario 2 - Visual SLAM RGB-D Saja
1. Tutup node SLAM sebelumnya (Ctrl+C di Terminal 2).
2. Jalankan Visual SLAM RTAB-Map murni:
   ```bash
   ros2 launch robot_sim slam_rgbd.launch.py
   ```
3. Ulangi perekaman ATE dan teleoperasi mengelilingi sirkuit lorong:
   ```bash
   ros2 run robot_sim benchmark_ate.py --name uji_rgbd --duration 60
   ```
4. Amati apakah robot mengalami *lost tracking* saat melewati dinding lorong yang minim tekstur visual.

### Langkah 4: Pengujian Skenario 3 - Fusi Sensor Penuh
1. Tutup node SLAM sebelumnya (Ctrl+C).
2. Jalankan Fusi Sensor RTAB-Map (LiDAR + RGB-D + IMU):
   ```bash
   ros2 launch robot_sim slam_fusion.launch.py
   ```
3. Ulangi perekaman ATE dengan durasi yang sama:
   ```bash
   ros2 run robot_sim benchmark_ate.py --name uji_fusi --duration 60
   ```
4. Amati bagaimana fitur geometri LiDAR mengoreksi orientasi robot saat fitur visual melemah.

---

## 5. Lembar Pengamatan dan Data Hasil Uji

Lengkapi tabel komparasi hasil pengujian berikut berdasarkan file log yang tersimpan di `~/skripsi_ws/eval_results/`:

| No | Parameter Evaluasi | Skenario 1: LiDAR Saja (`slam_toolbox`) | Skenario 2: RGB-D Saja (`rtabmap`) | Skenario 3: Fusi Penuh (`rtabmap` + LiDAR + IMU) |
| :---: | :--- | :---: | :---: | :---: |
| 1 | **ATE RMSE (meter)** | | | |
| 2 | **Mean Error (meter)** | | | |
| 3 | **Max Error (meter)** | | | |
| 4 | **Standar Deviasi (meter)** | | | |
| 5 | **Rata-rata Utilisasi CPU (%)** | | | |
| 6 | **Konsumsi Memori RAM (MB)** | | | |
| 7 | **Keberhasilan Loop Closure** | | | |

*Lampirkan gambar grafik komparasi lintasan 2D (`ate_trajectory_*.png`) dari ketiga skenario di bawah tabel pengamatan.*

---

## 6. Tugas Analisis dan Kesimpulan
1. Konfigurasi manakah yang menghasilkan nilai ATE RMSE terkecil? Jelaskan faktor fisis yang menyebabkan konfigurasi tersebut lebih unggul!
2. Mengapa Skenario 2 (RGB-D Saja) cenderung mengalami penyimpangan (*drift*) yang lebih besar pada lorong panjang dibandingkan Skenario 3?
3. Berdasarkan hasil pengukuran CPU dan RAM, berikan rekomendasi konfigurasi komputasi yang paling ideal jika sistem ini akan diimplementasikan pada mini PC robot fisik nyata (misalnya NVIDIA Jetson Orin Nano atau Raspberry Pi 5)!
