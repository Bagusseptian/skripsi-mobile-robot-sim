# LABSHEET 1: PENGENALAN LINGKUNGAN SIMULASI DAN 2D SLAM BERBASIS LIDAR 360°

**Mata Kuliah:** Penginderaan Visual Robot  
**Program Studi:** S1 Pendidikan Teknik Mekatronika, Universitas Negeri Yogyakarta  
**Penyusun:** Muhammad Bagus Septian  
**Alokasi Waktu:** 3 x 50 Menit (1 Pertemuan Praktikum)

---

## 1. Tujuan Pembelajaran
Setelah menyelesaikan praktikum ini, mahasiswa diharapkan mampu:
1. Menjalankan simulasi mobile robot 4WD di lingkungan Gazebo simulator menggunakan ROS 2 Humble.
2. Menganalisis pohon koordinat (*TF Tree*) serta hierarki frame koordinat robot (`odom`, `base_footprint`, `base_link`, `laser_link`).
3. Memvisualisasikan dan menginterpretasikan data pengukuran sensor jarak laser 360° (*LaserScan*) pada RViz2.
4. Melakukan pemetaan 2D (*2D Occupancy Grid Mapping*) secara interaktif menggunakan algoritma `slam_toolbox`.
5. Menyimpan hasil peta digital 2D (`.yaml` dan `.pgm`) untuk digunakan pada tahap navigasi otonom.

---

## 2. Alat dan Bahan
1. Laptop / Komputer dengan OS Linux Ubuntu 22.04 LTS (Native atau WSL2 dengan WSLg).
2. Perangkat Lunak ROS 2 Humble Hawksbill.
3. Gazebo Simulator 11 (Gazebo Classic).
4. Paket ROS 2 `robot_sim` (dilengkapi model URDF, sensor RPLIDAR A1 virtual, dan dunia simulasi).
5. Tool visualisasi `rviz2`.
6. Node teleoperasi `teleop_wasd` atau `teleop_twist_keyboard`.

---

## 3. Dasar Teori Singkat

### A. Prinsip Kerja LiDAR 2D 360°
LiDAR (*Light Detection and Ranging*) 2D bekerja memancarkan pulsa sinar laser inframerah sambil berputar 360 derajat secara horizontal. Berdasarkan waktu pantul sinar laser kembali ke receiver (*Time-of-Flight*) atau triangulasi optik, jarak objek $d$ dihitung dengan rumus:
$$d = \frac{c \times \Delta t}{2}$$
Data yang dihasilkan berupa larik jarak (*array of ranges*) dengan sudut tertentu dalam format pesan ROS 2 `sensor_msgs/msg/LaserScan`.

### B. Konsep Transformasi Koordinat (TF2)
Dalam robotika bergerak, setiap komponen memiliki koordinat lokal masing-masing:
- `world` / `map`: Frame acuan global yang bersifat statis.
- `odom`: Frame estimasi pergerakan berbasis integrasi roda (mengalami *drift* kumulatif).
- `base_footprint`: Proyeksi 2D posisi robot di lantai.
- `laser_link`: Posisi fisik sensor LiDAR terhadap titik pusat robot.

### C. Pemetaan Grid Okupansi (Occupancy Grid Mapping) & Graph SLAM
`slam_toolbox` menggunakan pendekatan *Pose Graph SLAM*. Ruangan dibagi menjadi sel-sel berukuran tertentu (resolusi $0.05\text{ m} \times 0.05\text{ m}$). Tiap sel memiliki probabilitas okupansi:
- **0 (Putih):** Ruang kosong (*Free space*).
- **100 (Hitam):** Ruang terhalang rintangan (*Occupied*).
- **-1 (Abu-abu):** Area belum terpetakan (*Unknown*).

---

## 4. Langkah Kerja

### Langkah 1: Membuka Lingkungan Simulasi Robot
1. Buka Terminal pertama di Ubuntu / WSL2.
2. Source workspace skripsi dan jalankan simulasi Dunia A (Arena Dasar):
   ```bash
   source ~/skripsi_ws/install/setup.bash
   ros2 launch robot_sim sim.launch.py world:=world_a
   ```
3. Pastikan jendela Gazebo terbuka dan robot virtual berwarna abu-abu gelap dengan silinder LiDAR merah di atasnya telah muncul.

### Langkah 2: Memeriksa Topik dan Struktur TF
1. Buka Terminal kedua, periksa daftar topik aktif:
   ```bash
   ros2 topic list
   ```
2. Periksa frekuensi data sensor LiDAR (`/scan`):
   ```bash
   ros2 topic hz /scan
   ```
   *(Catat frekuensi yang tertera, idealnya mendekati 10.0 Hz)*.
3. Tampilkan struktur pohon transformasi (TF Tree):
   ```bash
   ros2 run tf2_tools view_frames
   evince frames.pdf &
   ```
   *(Amati hubungan antara `odom` -> `base_footprint` -> `base_link` -> `laser_link`)*.

### Langkah 3: Menjalankan Pemetaan 2D (SLAM Toolbox)
1. Buka Terminal ketiga, jalankan node pemetaan LiDAR:
   ```bash
   source ~/skripsi_ws/install/setup.bash
   ros2 launch robot_sim slam_lidar.launch.py
   ```
2. Jendela RViz2 akan terbuka menampilkan grid peta hitam-putih dan titik-titik pancaran laser merah.

### Langkah 4: Mengemudikan Robot (Teleoperasi)
1. Buka Terminal keempat, jalankan node pengemudi robot:
   ```bash
   source ~/skripsi_ws/install/setup.bash
   ros2 run robot_sim teleop_wasd.py
   ```
2. Gunakan tombol keyboard:
   - `W`: Maju lurus
   - `S`: Mundur lurus
   - `A`: Putar ke kiri
   - `D`: Putar ke kanan
   - `Spasi`: Berhenti
3. Kemudikan robot secara perlahan mengelilingi ruangan World A hingga seluruh batas dinding dan rintangan silinder/kotak terpetakan jelas.

### Langkah 5: Menyimpan Hasil Peta
1. Setelah peta di RViz2 terlihat tertutup rapat dan rapi, simpan peta dengan perintah:
   ```bash
   cd ~/skripsi_ws/src/robot_sim/maps
   ros2 run nav2_map_server map_saver_cli -f world_a_map
   ```
2. Periksa berkas peta yang terbentuk: `world_a_map.yaml` dan `world_a_map.pgm`.

---

## 5. Lembar Pengamatan Mahasiswa

### Tabel 1.1: Pengamatan Spesifikasi dan Karakteristik Topik ROS 2
| Nama Topik | Tipe Pesan (*Message Type*) | Frekuensi Aktual (Hz) | Frame ID Terkait |
| :--- | :--- | :--- | :--- |
| `/scan` | `sensor_msgs/msg/LaserScan` | ........... Hz | ........... |
| `/odom` | `nav_msgs/msg/Odometry` | ........... Hz | ........... |
| `/map` | `nav_msgs/msg/OccupancyGrid` | ........... Hz | ........... |

### Tabel 1.2: Pengamatan Parameter `sensor_msgs/msg/LaserScan`
*(Gunakan perintah `ros2 topic echo /scan --once`)*
| Parameter | Nilai Terbaca | Satuan | Analisis Keterangan |
| :--- | :--- | :--- | :--- |
| `angle_min` | | Radian | Sudut awal sapuan laser |
| `angle_max` | | Radian | Sudut akhir sapuan laser |
| `angle_increment` | | Radian | Resolusi sudut antar titik sinar |
| `range_min` | | Meter | Jarak buta minimum sensor |
| `range_max` | | Meter | Jangkauan deteksi maksimum sensor |

---

## 6. Pertanyaan Analisis dan Diskusi
1. Mengapa nilai `range_min` pada sensor LiDAR harus diatur lebih besar dari jari-jari badan robot itu sendiri? Apa bahayanya jika `range_min` diatur bernilai 0.0 meter?
2. Ketika robot berputar di tempat dengan kecepatan sudut tinggi, mengapa bentuk peta 2D pada RViz2 terkadang mengalami distorsi atau pergeseran dinding (*drift*) sebelum *loop closure* terjadi?
3. Jelaskan peran transformasi koordinat (*TF*) antara frame `odom` dan `map` yang dipublikasikan oleh `slam_toolbox`! Mengapa nilai transformasi tersebut selalu berubah-ubah saat robot bergerak?

---

## 7. Rubrik Penilaian Praktikum
| Aspek Penilaian | Skor Maksimal | Kriteria Pencapaian |
| :--- | :---: | :--- |
| **Persiapan & Setup Lingkungan** | 20 | Mampu mengaktifkan simulasi Gazebo, robot_state_publisher, dan memeriksa TF tree tanpa kendala. |
| **Pelaksanaan Pemetaan 2D** | 30 | Berhasil mengemudikan robot hingga menghasilkan peta ruangan 2D yang utuh, tanpa distorsi besar. |
| **Penyimpanan & Analisis Data** | 25 | Mengisi tabel pengamatan secara akurat dan berhasil mengekspor peta digital `.yaml`/`.pgm`. |
| **Jawaban Pertanyaan Diskusi** | 25 | Mampu menjelaskan konsep teknis Time-of-Flight, TF drift, dan koreksi Pose Graph secara komprehensif. |
| **Total Skor** | **100** | |
