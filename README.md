# Media Pembelajaran Mobile Robot: Visual SLAM RGB-D & LiDAR 360° (ROS 2 Humble)

**Penyusun:** Muhammad Bagus Septian  
**Program Studi:** S1 Pendidikan Teknik Mekatronika, Fakultas Teknik, Universitas Negeri Yogyakarta  
**Mata Kuliah:** Penginderaan Visual Robot  
**Versi:** 2.0.0 (Gazebo Classic 11 & ROS 2 Humble)

Repositori ini berisi media pembelajaran dan perangkat simulasi mobile robot 4WD (skid-steer) lengkap dengan fusi sensor **360° LiDAR RPLIDAR A1**, **Kamera RGB-D Orbbec Astra Pro**, dan **IMU**, terintegrasi dengan **RTAB-Map Visual SLAM**, **Nav2**, modul eksplorasi mandiri (*autonomous wanderer*), penyimpanan peta 2D/3D terpadu, instrumen evaluasi objektif (ATE & beban sistem), serta 6 modul labsheet praktikum terstruktur.

---

## Struktur Direktori Proyek

```
robot_sim/
├── CMakeLists.txt                    # Konfigurasi build ament_cmake
├── package.xml                       # Manifest paket ROS 2 Humble
├── urdf/
│   └── robot.urdf.xacro              # Model robot 4WD, RPLIDAR A1, Astra Pro RGB-D, IMU, Ground Truth P3D
├── worlds/
│   ├── world_a_basic.world           # Dunia A: Ruang latihan dasar 8x8m dengan rintangan geometri
│   ├── world_b_corridor.world        # Dunia B: Lorong sirkuit 14x10m dengan belokan 90° & pintu
│   └── world_c_classroom.world       # Dunia C: Ruang lab/kelas 12x10m dengan meja, monitor, papan tulis
├── config/
│   ├── slam_toolbox_params.yaml      # Parameter optimal untuk 2D LiDAR SLAM (slam_toolbox)
│   └── nav2_params.yaml              # Parameter Nav2 (Costmap 2D/3D Voxel, DWB Planner, NavFn)
├── launch/
│   ├── sim.launch.py                 # Peluncur simulasi Gazebo Classic (world:=world_a|world_b|world_c)
│   ├── slam_fusion.launch.py         # Peluncur Fusi Sensor Penuh (RTAB-Map: LiDAR + RGB-D + IMU) + rtabmap_viz
│   ├── slam_lidar.launch.py          # Peluncur 2D LiDAR SLAM (slam_toolbox) + RViz2
│   ├── slam_rgbd.launch.py           # Peluncur Visual SLAM RGB-D Murni (RTAB-Map tanpa LiDAR)
│   ├── nav2.launch.py                # Peluncur Navigasi Otonom Nav2 (Costmap 2D + Voxel Layer 3D)
│   ├── rviz.launch.py                # Peluncur visualisasi RViz2 terpisah (LiDAR, Kamera, PointCloud, Peta)
│   ├── rviz_sim.launch.py            # Visualisasi inspeksi sensor di RViz2
│   └── view_sensors.launch.py        # Visualisasi seluruh aliran sensor di RViz2
├── rviz/
│   ├── nav2_view.rviz                # Profil tampilan navigasi Nav2
│   ├── slam_fusion.rviz              # Profil tampilan Fusi SLAM (LiDAR, PointCloud, Peta 2D/3D)
│   ├── slam_lidar.rviz               # Profil tampilan RViz untuk LiDAR SLAM
│   └── view_all_sensors.rviz         # Profil inspeksi seluruh aliran sensor
├── scripts/
│   ├── autonomous_wanderer.py        # Robot berjalan otonom cerdas (gap-seeking & corridor centering)
│   ├── teleop_wasd.py                # Kontrol robot interaktif via keyboard (WASD + steering halus)
│   ├── map_saver_unified.py          # Penyimpan peta 2D (PGM/YAML) & 3D Dense (PCD/PLY) sekali klik
│   ├── generate_advanced_survey_report.py # Generator laporan survei 3D geospasial ala Agisoft/CloudCompare
│   ├── pid_velocity_controller.py    # Kontroler PID kecepatan halus & anti-oversteering
│   ├── labsheet0_rgbd_opencv.py      # Node ROS 2 untuk modul praktikum Labsheet 0
│   ├── rgbd_spatial_interactive.py   # Visualisasi spasial interaktif RGB-D
│   ├── benchmark_ate.py              # Evaluasi Absolute Trajectory Error (Ground Truth vs SLAM)
│   ├── benchmark_system.py           # Monitoring beban CPU % dan RAM MB per proses
│   └── nav2_goal_tester.py           # Penguji otomatis navigasi otonom 5 waypoint Nav2
├── maps/                             # Koleksi dataset peta hasil pemetaan 2D & 3D dense point cloud
├── eval_results/                     # Log hasil pengujian empiris (ATE, CPU/RAM, Survey Report)
├── labsheets/
│   ├── Labsheet_0_Jembatan_OpenCV_ke_Kamera_RGBD.md # Praktikum 0: Jembatan OpenCV & Kamera RGB-D
│   ├── Labsheet_0_Jembatan_OpenCV_ke_Kamera_RGBD.docx
│   ├── Labsheet_1_Instalasi_dan_LiDAR_360.md        # Praktikum 1: Setup ROS 2, LiDAR 360 & 2D Mapping
│   ├── Labsheet_2_Kamera_RGBD_dan_Visual_SLAM.md    # Praktikum 2: Visual Odometry & Visual SLAM RGB-D
│   ├── Labsheet_3_Fusi_Sensor_dan_Navigasi_Otonom.md # Praktikum 3: Fusi Sensor RTAB-Map & Nav2
│   ├── Labsheet_4_Navigasi_Otonom_dan_3D_Voxel_Layer.md # Praktikum 4: Navigasi Otonom & Voxel 3D
│   ├── Labsheet_5_Evaluasi_Objektif_ATE_dan_Beban_Komputasi.md # Praktikum 5: Pengujian ATE & Beban
│   ├── Kunci_Jawaban_dan_Rubrik_Penilaian.md         # Rubrik evaluasi dosen dan kunci praktikum
│   └── gambar_labsheet0/             # 18 berkas ilustrasi teknis untuk modul pembelajaran
└── docs/
    ├── Instrumen_Penelitian.md       # Angket ahli materi, ahli media, dan respons mahasiswa
    ├── Panduan_Instalasi_Mahasiswa.md# Panduan instalasi langkah demi langkah untuk praktikan
    └── Panduan_Transisi_Simulasi_ke_Hardware.md # Blueprint transisi ke robot fisik nyata
```

---

## Panduan Penggunaan Langkah demi Langkah (100% Berjalan)

Pastikan lingkungan ROS 2 Humble dan Gazebo Classic 11 sudah terpasang di Ubuntu 22.04 LTS (atau WSL2).

### 1. Kompilasi Workspace
Buka terminal dan lakukan build:
```bash
cd ~/skripsi_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select robot_sim
source install/setup.bash
```

---

### 2. Skenario Pemetaan Otonom Penuh (Rekomendasi Utama)

Jalankan 3 terminal berdampingan untuk melihat robot memetakan ruang kelas secara otomatis:

#### **Terminal 1: Jalankan Simulator Gazebo**
```bash
cd ~/skripsi_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_sim sim.launch.py world:=world_c
```
> **Pilihan Dunia (`world`):**
> * `world_c` : Ruang kelas/lab lengkap dengan meja, kursi, dan monitor (*Default*)
> * `world_b` : Lorong sirkuit panjang dengan sudut 90°
> * `world_a` : Arena uji dasar 8x8 meter

#### **Terminal 2: Jalankan Fusi Visual SLAM (RTAB-Map)**
```bash
cd ~/skripsi_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_sim slam_fusion.launch.py
```
*(Jendela bawaan `rtabmap_viz` akan terbuka menampilkan rekonstruksi 3D point cloud dan grid peta 2D secara real-time seperti awal).*

#### **Terminal 3 (Opsional Terpisah): Buka RViz2**
Jika ingin membuka visualisasi **RViz2** secara terpisah di jendela tersendiri:
```bash
cd ~/skripsi_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_sim rviz.launch.py
```
*(Jendela **RViz2** terbuka terpisah menampilkan model robot 3D, pindaian 360° LiDAR hijau, point cloud warna 3D kamera, occupancy grid `/map`, dan 3D dense map).*

#### **Terminal 4: Jalankan Robot Berjalan Otonom**
```bash
cd ~/skripsi_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run robot_sim autonomous_wanderer.py
```
*(Robot akan meluncur halus di tengah lorong meja kelas, menghindari rintangan secara cerdas, dan menjelajahi seluruh ruangan tanpa oleng).*

---

### 3. Kontrol Robot Manual (Teleoperasi WASD)

Jika ingin menyetir robot sendiri (sebagai pengganti robot berjalan otonom):
```bash
cd ~/skripsi_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run robot_sim teleop_wasd.py
```

| Tombol | Aksi |
| :---: | :--- |
| **`W` / `S`** | Maju / Mundur lurus |
| **`A` / `D`** | Putar kiri / Putar kanan di tempat |
| **`Q` / `E`** | Maju serong kiri / serong kanan |
| **`Z` / `C`** | Mundur serong kiri / serong kanan |
| **`Spasi` / `X`** | **Rem berhenti seketika** |
| **`+` / `-`** | Tambah / kurangi batas kecepatan |

---

### 4. Menyimpan Peta 2D & 3D (Sekali Klik)

Setelah ruangan selesai dipetakan, buka terminal baru untuk mengekspor peta 2D dan 3D dense point cloud:
```bash
cd ~/skripsi_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run robot_sim map_saver_unified.py --prefix map_world_c
```
File peta akan otomatis disimpan ke folder `maps/`:
* `map_world_c_2d.pgm` & `.yaml` : 2D Occupancy Grid Map standar ROS
* `map_world_c_3d.pcd` & `.ply` : 3D Dense Color Point Cloud (>500.000 titik)

---

### 5. Membuat Laporan Survei 3D (Geospatial Survey Report)

Untuk menghasilkan laporan evaluasi kerapatan titik 3D dan filtering noise CloudCompare SOR:
```bash
cd ~/skripsi_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 src/robot_sim/scripts/generate_advanced_survey_report.py
```
Laporan lengkap, statistik deviasi standar, dan grafik perbandingan akan dibuat di folder `eval_results/`.

---

## Tiga Konfigurasi SLAM yang Dibandingkan

1. **Skenario 1 — 2D LiDAR SLAM Murni (`slam_toolbox`):**
   ```bash
   ros2 launch robot_sim slam_lidar.launch.py
   ```
2. **Skenario 2 — Visual SLAM RGB-D Murni (`rtabmap` tanpa LiDAR):**
   ```bash
   ros2 launch robot_sim slam_rgbd.launch.py
   ```
3. **Skenario 3 — Fusi Sensor Penuh (`rtabmap` LiDAR 360° + RGB-D + IMU):**
   ```bash
   ros2 launch robot_sim slam_fusion.launch.py
   ```

---

## Navigasi Otonom (Nav2)
Setelah peta ruangan tersimpan:
```bash
ros2 launch robot_sim nav2.launch.py
```
Tentukan titik tujuan (*Goal Pose*) di RViz2. Local costmap dilengkapi **Voxel Layer 3D** yang membaca point cloud kamera depth (`/camera/points`) untuk menghindari rintangan 3D di luar jangkauan tembak laser 2D.

---

## Perangkat Evaluasi Objektif Skripsi (Bab IV)

### 1. Uji Akurasi Trayektori (Absolute Trajectory Error / ATE)
```bash
ros2 run robot_sim benchmark_ate.py --name fusi_sensor --duration 60
```
Merekam ground truth absolut dari Gazebo (`/ground_truth/odom`) dan membandingkannya dengan estimasi pose SLAM untuk menghitung RMSE linier, rotasi, dan plot lintasan.

### 2. Uji Beban Komputasi (CPU & RAM)
```bash
ros2 run robot_sim benchmark_system.py --name uji_fusi --duration 60
```
Merekam persentase utilisasi CPU (%) dan konsumsi RAM (MB) untuk Gazebo, RTAB-Map, Nav2, dan RViz2.

### 3. Pengujian Otomatis Nav2 (5 Waypoint Navigasi)
```bash
ros2 run robot_sim nav2_goal_tester.py --world world_c --name uji_nav_c
```

---

## Modul Pembelajaran (Labsheet Praktikum)

| Kode | Judul Praktikum | Berkas Modul |
| :--- | :--- | :--- |
| **Labsheet 0** | Jembatan Pemrograman OpenCV ke Sensor Kamera RGB-D | [`Labsheet_0_Jembatan_OpenCV_ke_Kamera_RGBD.md`](file:///C:/Users/Bagusseptian/Skripsi_Simulasi_Robot/labsheets/Labsheet_0_Jembatan_OpenCV_ke_Kamera_RGBD.md) |
| **Labsheet 1** | Pengenalan Ekosistem ROS 2, Sensor LiDAR 360°, dan 2D Mapping | [`Labsheet_1_Instalasi_dan_LiDAR_360.md`](file:///C:/Users/Bagusseptian/Skripsi_Simulasi_Robot/labsheets/Labsheet_1_Instalasi_dan_LiDAR_360.md) |
| **Labsheet 2** | Penginderaan Kamera RGB-D Astra Pro dan Rekonstruksi 3D | [`Labsheet_2_Kamera_RGBD_dan_Visual_SLAM.md`](file:///C:/Users/Bagusseptian/Skripsi_Simulasi_Robot/labsheets/Labsheet_2_Kamera_RGBD_dan_Visual_SLAM.md) |
| **Labsheet 3** | Fusi Sensor LiDAR-Visual SLAM dan Navigasi Otonom Sederhana | [`Labsheet_3_Fusi_Sensor_dan_Navigasi_Otonom.md`](file:///C:/Users/Bagusseptian/Skripsi_Simulasi_Robot/labsheets/Labsheet_3_Fusi_Sensor_dan_Navigasi_Otonom.md) |
| **Labsheet 4** | Perencanaan Jalur Otonom Nav2 dan Deteksi Rintangan 3D Voxel | [`Labsheet_4_Navigasi_Otonom_dan_3D_Voxel_Layer.md`](file:///C:/Users/Bagusseptian/Skripsi_Simulasi_Robot/labsheets/Labsheet_4_Navigasi_Otonom_dan_3D_Voxel_Layer.md) |
| **Labsheet 5** | Evaluasi Objektif Kinerja SLAM (ATE) dan Analisis Beban Komputasi | [`Labsheet_5_Evaluasi_Objektif_ATE_dan_Beban_Komputasi.md`](file:///C:/Users/Bagusseptian/Skripsi_Simulasi_Robot/labsheets/Labsheet_5_Evaluasi_Objektif_ATE_dan_Beban_Komputasi.md) |
| **Rubrik** | Kunci Jawaban Praktikum dan Rubrik Penilaian Dosen | [`Kunci_Jawaban_dan_Rubrik_Penilaian.md`](file:///C:/Users/Bagusseptian/Skripsi_Simulasi_Robot/labsheets/Kunci_Jawaban_dan_Rubrik_Penilaian.md) |

---

## Lisensi
Proyek ini dikembangkan di bawah lisensi **Apache License 2.0** untuk kepentingan penelitian skripsi dan media pembelajaran akademik Universitas Negeri Yogyakarta.
