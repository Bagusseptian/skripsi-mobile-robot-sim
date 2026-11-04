# Media Pembelajaran Mobile Robot: Visual SLAM RGB-D & LiDAR 360° (ROS 2 Humble)

**Penyusun:** Muhammad Bagus Septian  
**Program Studi:** S1 Pendidikan Teknik Mekatronika, Fakultas Teknik, Universitas Negeri Yogyakarta  
**Mata Kuliah:** Penginderaan Visual Robot  

Proyek ini adalah implementasi lengkap dari dokumen **Project Plan Tahap Simulasi**, mencakup pemodelan virtual robot sekelas ROSbot, tiga variasi dunia simulasi Gazebo, pipeline lengkap untuk 3 konfigurasi SLAM (LiDAR saja, RGB-D saja, dan Fusi Sensor), navigasi otonom Nav2, skrip evaluasi metrik objektif (ATE & beban sistem), tiga labsheet pembelajaran beserta kunci jawaban, serta panduan transisi ke robot fisik nyata.

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
│   └── world_c_classroom.world       # Dunia C: Ruang lab/kelas 12x10m dengan meja, kursi, tekstur kaya
├── config/
│   ├── slam_toolbox_params.yaml      # Parameter optimal untuk 2D LiDAR SLAM (slam_toolbox)
│   └── nav2_params.yaml              # Parameter Nav2 (Costmap 2D/3D Voxel, DWB Planner, NavFn)
├── launch/
│   ├── sim.launch.py                 # Peluncur simulasi Gazebo (pilihan: world:=world_a|world_b|world_c)
│   ├── slam_lidar.launch.py          # Peluncur 2D LiDAR SLAM (slam_toolbox) + RViz2
│   ├── slam_rgbd.launch.py           # Peluncur Visual SLAM RGB-D Murni (RTAB-Map tanpa LiDAR)
│   ├── slam_fusion.launch.py         # Peluncur Fusi Sensor Penuh (RTAB-Map: LiDAR + RGB-D + IMU)
│   ├── nav2.launch.py                # Peluncur Navigasi Otonom Nav2 (Costmap 2D + Voxel Layer 3D)
│   ├── rviz_sim.launch.py            # Visualisasi inspeksi sensor di RViz2
│   └── webcam_rgbd.launch.py         # Node opsional jembatan webcam
├── rviz/
│   ├── slam_lidar.rviz               # Profil tampilan RViz untuk LiDAR SLAM
│   └── view_all_sensors.rviz         # Profil inspeksi seluruh aliran sensor
├── scripts/
│   ├── teleop_wasd.py                # Kontrol robot interaktif via keyboard (WASD)
│   ├── benchmark_ate.py              # Evaluasi Absolute Trajectory Error (RMSE ground truth vs SLAM)
│   ├── benchmark_system.py           # Monitoring beban CPU % dan RAM MB per node
│   ├── nav2_goal_tester.py           # Penguji navigasi otonom otomatis (5 waypoint, waktu & sukses)
│   ├── dummy_robot.py                # Simulator kinematik ringan tanpa Gazebo
│   └── intelligent_explorer.py       # Eksplorasi frontier otonom
├── labsheets/
│   ├── Labsheet_1_Instalasi_dan_LiDAR_360.md       # Modul Praktikum 1: LiDAR & 2D Mapping
│   ├── Labsheet_2_Kamera_RGBD_dan_Visual_SLAM.md   # Modul Praktikum 2: RGB-D & Visual SLAM
│   ├── Labsheet_3_Fusi_Sensor_dan_Navigasi_Otonom.md# Modul Praktikum 3: Fusi Sensor & Nav2
│   └── Kunci_Jawaban_dan_Rubrik_Penilaian.md        # Panduan Dosen/Asisten & Rubrik Evaluasi
└── docs/
    └── Panduan_Transisi_Simulasi_ke_Hardware.md    # Dokumen transisi hardware (ESP32, driver riil, kalibrasi)
```

---

## Panduan Penggunaan Cepat

### 1. Build Workspace
```bash
cd ~/skripsi_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. Menjalankan Simulasi Gazebo
Pilih salah satu dari 3 dunia:
```bash
# Dunia A (Arena Dasar):
ros2 launch robot_sim sim.launch.py world:=world_a

# Dunia B (Lorong Sirkuit):
ros2 launch robot_sim sim.launch.py world:=world_b

# Dunia C (Ruang Kelas Bertekstur):
ros2 launch robot_sim sim.launch.py world:=world_c
```

### 3. Mengemudikan Robot (Teleoperasi WASD)
Buka terminal baru:
```bash
source ~/skripsi_ws/install/setup.bash
ros2 run robot_sim teleop_wasd.py
```
*(Gunakan W/A/S/D untuk menggerakkan robot, Spasi untuk berhenti)*.

---

## Tiga Konfigurasi SLAM yang Dibandingkan

### Skenario 1: LiDAR Saja (`slam_toolbox`)
```bash
ros2 launch robot_sim slam_lidar.launch.py
```
- Memetakan denah 2D menggunakan sensor 360° LiDAR (`/scan`).
- Menghasilkan 2D Occupancy Grid Map dengan cepat dan stabil.

### Skenario 2: Visual SLAM RGB-D Saja (`rtabmap`)
```bash
ros2 launch robot_sim slam_rgbd.launch.py
```
- Memetakan lingkungan murni menggunakan citra warna dan kedalaman kamera RGB-D (`/camera/*`).
- Merekonstruksi peta 3D point cloud dan mendeteksi visual loop closure.

### Skenario 3: Fusi Sensor Penuh (`rtabmap` LiDAR + RGB-D + IMU)
```bash
ros2 launch robot_sim slam_fusion.launch.py
```
- Menggabungkan batas geometri 2D LiDAR dengan fitur visual tekstur 3D kamera RGB-D dan kestabilan IMU.

---

## Navigasi Otonom (Nav2)
Setelah SLAM berjalan atau dengan peta tersimpan:
```bash
ros2 launch robot_sim nav2.launch.py
```
- Menentukan titik tujuan dengan tool **"Nav2 Goal"** di RViz2.
- Local costmap dilengkapi lapisan **Voxel Layer 3D** yang membaca point cloud kamera depth (`/camera/points`) untuk mendeteksi rintangan 3D di luar bidang iris laser 2D.

---

## Perangkat Evaluasi dan Pengujian Metrik (Bab Hasil)

### 1. Pengukuran Akurasi Lokalisasi (Absolute Trajectory Error / ATE)
```bash
ros2 run robot_sim benchmark_ate.py --name fusi_sensor --duration 60
```
- Merekam data Ground Truth absolut dari Gazebo (`/ground_truth/odom`) dan membandingkannya dengan estimasi pose SLAM.
- Menghitung RMSE, Mean Error, Max Error, StdDev, mengekspor dataset CSV, dan menghasilkan plot lintasan 2D (PNG).

### 2. Pengukuran Beban Komputasi (CPU & RAM)
```bash
ros2 run robot_sim benchmark_system.py --name uji_fusi --duration 60
```
- Merekam penggunaan CPU (%) dan RAM (MB) untuk Gazebo, RTAB-Map, Nav2, dan RViz2.
- Menghasilkan ringkasan rata-rata dan beban puncak.

### 3. Pengujian Otomatis Keberhasilan Navigasi Nav2
```bash
ros2 run robot_sim nav2_goal_tester.py --world world_b --name uji_nav_b
```
- Mengirim 5 titik waypoint berurutan via `BasicNavigator`.
- Menghitung persentase keberhasilan (*success rate %*) dan rata-rata waktu tempuh (detik).
