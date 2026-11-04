# LABSHEET 3: FUSI SENSOR (LIDAR + RGB-D + IMU) DAN NAVIGASI OTONOM (NAV2)

**Mata Kuliah:** Penginderaan Visual Robot  
**Program Studi:** S1 Pendidikan Teknik Mekatronika, Universitas Negeri Yogyakarta  
**Penyusun:** Muhammad Bagus Septian  
**Alokasi Waktu:** 3 x 50 Menit (1 Pertemuan Praktikum)

---

## 1. Tujuan Pembelajaran
Setelah menyelesaikan praktikum ini, mahasiswa diharapkan mampu:
1. Mengintegrasikan data multi-sensor (*360° LiDAR*, *Kamera RGB-D*, dan *IMU*) ke dalam arsitektur fusi sensor RTAB-Map.
2. Menjelaskan keunggulan dan kelemahan masing-masing modalitas sensor ketika digunakan secara mandiri maupun terfusi.
3. Mengonfigurasi dan mengoperasikan tumpukan navigasi otonom *Navigation 2 (Nav2)*.
4. Menganalisis peran *Global Costmap* dan *Local Costmap* (termasuk lapisan *Voxel Layer 3D* dari kamera *depth*) dalam mendeteksi dan menghindari rintangan statis maupun dinamis.
5. Melakukan pengujian navigasi titik tunggal (*Single Goal Pose*) serta multi-waypoint otomatis.

---

## 2. Alat dan Bahan
1. Laptop / PC dengan Ubuntu 22.04 LTS (ROS 2 Humble terinstal lengkap).
2. Simulator Gazebo dengan paket `robot_sim`.
3. Tumpukan navigasi `nav2_bringup` dan pengontrol `dwb_controller`.
4. Skrip pengujian otomatis `benchmark_ate.py` dan `nav2_goal_tester.py`.
5. Dunia simulasi World B (`world_b_corridor.world`) dan World C (`world_c_classroom.world`).

---

## 3. Dasar Teori Singkat

### A. Urgensi Fusi Sensor pada Mobile Robot
- **LiDAR 360° Saja:** Memiliki sudut pandang keliling penuh dan akurasi geometri dinding yang sangat tinggi, namun buta terhadap rintangan yang berada di bawah atau di atas bidang tembak sinar laser 2D (misal: kolong meja, rintangan menggantung, atau anak tangga). Selain itu, LiDAR tidak memiliki informasi warna/tekstur untuk membedakan ruangan berpenampang simetris panjang (*corridor ambiguity*).
- **Kamera RGB-D Saja:** Memiliki informasi visual kaya warna dan dense 3D point cloud, namun memiliki *Field of View* (FOV) horizontal terbatas ($\pm 87^\circ$), jangkauan efektif terbatas ($0.3 - 6\text{ m}$), dan rentan kehilangan jejak (*lost tracking*) saat menghadap dinding polos atau melakukan putaran cepat.
- **Fusi Sensor (LiDAR + RGB-D + IMU):** Mengombinasikan batasan geometri 360° LiDAR untuk kestabilan peta 2D dengan *visual loop closure* kamera RGB-D untuk relokalisasi akurat, didukung orientasi cepat dari akselerometer/giroskop IMU.

### B. Arsitektur Navigasi Otonom (Nav2)
Tumpukan Nav2 terdiri dari sub-sistem utama:
1. **BT Navigator (*Behavior Tree*):** Mengatur alur logika navigasi, urutan perencanaan, eksekusi kontrol, dan strategi pemulihan (*Recovery Behaviors: spin, backup, wait*).
2. **Planner Server (Global Planner):** Merencanakan lintasan bebas rintangan dari titik awal ke titik tujuan di atas *Global Costmap* menggunakan algoritma pencarian rute (A* atau NavFn Dijkstra).
3. **Controller Server (Local Planner / DWB):** Menghitung perintah kecepatan linier dan sudut (`/cmd_vel`) secara lokal (20 Hz) untuk mengikuti lintasan global sambil menghindari rintangan mendadak di atas *Local Costmap*.
4. **Costmap 2D & Voxel 3D:** Merepresentasikan tingkat bahaya ruang:
   - *Static Layer:* Berasal dari peta SLAM yang telah disimpan.
   - *Obstacle Layer:* Berasal dari sapuan laser LiDAR 2D real-time.
   - *Voxel Layer:* Memproyeksikan point cloud 3D dari kamera depth untuk mendeteksi rintangan 3D di lantai atau setinggi badan robot.
   - *Inflation Layer:* Memberikan zona penyangga keselamatan di sekeliling badan robot.

---

## 4. Langkah Kerja

### Langkah 1: Membuka Simulasi Dunia Lorong (World B)
1. Buka Terminal 1, jalankan simulasi:
   ```bash
   source ~/skripsi_ws/install/setup.bash
   ros2 launch robot_sim sim.launch.py world:=world_b
   ```
2. Pastikan robot berada di dalam lorong sirkuit World B.

### Langkah 2: Menjalankan Fusi Sensor Penuh (SLAM Fusion)
1. Buka Terminal 2, jalankan RTAB-Map dengan mode fusi sensor:
   ```bash
   source ~/skripsi_ws/install/setup.bash
   ros2 launch robot_sim slam_fusion.launch.py
   ```
2. Amati bahwa RTAB-Map kini menerima dan memproses data dari kedua sensor secara serempak:
   - Topik `/scan` menghasilkan 2D Occupancy Grid.
   - Topik `/camera/*` menghasilkan visual keypoints dan 3D dense cloud.
3. Kemudikan robot mengitari sirkuit lorong World B hingga robot kembali ke posisi awal. Amati ketahanan pemetaan terhadap drift odometri.

### Langkah 3: Menjalankan Navigasi Otonom (Nav2)
1. Buka Terminal 3, jalankan Nav2:
   ```bash
   source ~/skripsi_ws/install/setup.bash
   ros2 launch robot_sim nav2.launch.py
   ```
2. Jendela RViz2 akan terbuka dengan tampilan Costmap:
   - Garis hijau / biru: Rencana lintasan global (*Global Plan*).
   - Area merah / kuning di sekitar dinding: Lapisan inflasi rintangan (*Costmap Inflation*).
   - Kotak ungu / titik 3D: Proyeksi voxel rintangan dari kamera depth.

### Langkah 4: Pengujian Navigasi Titik Tunggal (Single Goal Pose)
1. Pada bilah atas RViz2, klik tombol **"Nav2 Goal"**.
2. Klik dan tarik mouse pada posisi ujung lorong lain di peta untuk menentukan posisi dan orientasi tujuan robot.
3. Amati pergerakan robot:
   - Global planner langsung menggambar garis rute.
   - Robot berputar dan melaju secara mandiri mengikuti jalur.
   - Ketika mendekati rintangan kotak di lorong timur, robot otomatis berbelok menghindarinya.

### Langkah 5: Pengujian Multi-Waypoint Otomatis dan Pengukuran Metrik
1. Buka Terminal 4, jalankan skrip evaluasi otomatis:
   ```bash
   source ~/skripsi_ws/install/setup.bash
   ros2 run robot_sim nav2_goal_tester.py --world world_b --name fusi_sensor_b
   ```
2. Skrip akan mengirim 5 titik tujuan secara berurutan dan mencatat waktu tempuh serta tingkat keberhasilan ke dalam berkas CSV.

---

## 5. Lembar Pengamatan Mahasiswa

### Tabel 3.1: Perbandingan Karakteristik Ketiga Metode SLAM
| Aspek Evaluasi | LiDAR Saja (`slam_toolbox`) | RGB-D Saja (`rtabmap`) | Fusi Sensor (`rtabmap`) |
| :--- | :---: | :---: | :---: |
| Cakupan Sudut Pandang Sensor | ............ derajat | ............ derajat | ............ derajat |
| Kemampuan Deteksi Rintangan 3D | Rendah / Sedang / Tinggi | Rendah / Sedang / Tinggi | Rendah / Sedang / Tinggi |
| Ketahanan di Lorong Simetris Polos | Rendah / Sedang / Tinggi | Rendah / Sedang / Tinggi | Rendah / Sedang / Tinggi |
| Beban Komputasi CPU (%) | ............ % | ............ % | ............ % |

### Tabel 3.2: Hasil Pengujian Navigasi Otonom Nav2 (World B)
| No. Titik Target | Posisi Target $(X, Y)$ | Status Navigasi (Succeeded / Failed) | Waktu Tempuh (detik) | Keterangan Perilaku Robot |
| :---: | :---: | :---: | :---: | :--- |
| 1 | $(5.8, 0.0)$ | | | |
| 2 | $(0.0, 3.8)$ | | | |
| 3 | $(-5.8, 0.0)$ | | | |
| 4 | $(0.0, -3.8)$ | | | |
| 5 | $(0.0, -1.0)$ | | | |

---

## 6. Pertanyaan Analisis dan Diskusi
1. Mengapa keberadaan lapisan *Voxel Layer 3D* pada *Local Costmap* sangat krusial bagi robot yang beroperasi di lingkungan kantor atau laboratorium? Berikan contoh rintangan nyata yang tidak dapat dideteksi oleh LiDAR 2D setinggi 15 cm tetapi berhasil dideteksi oleh kamera depth!
2. Jika sebuah rintangan statis diletakkan tepat menghalangi pintu lorong selebar $1.5\text{ m}$ sehingga celah tersisa hanya $0.3\text{ m}$, apa reaksi yang dilakukan oleh DWB local controller dan behavior recovery Nav2 saat robot berusaha melewatinya? (Kaitkan dengan dimensi lebar robot dan *inflation radius*).
3. Berdasarkan data pengamatan pada Tabel 3.1, simpulkan mengapa metode fusi sensor merupakan pilihan paling ideal untuk implementasi pada robot fisik yang akan dirakit pada tahap selanjutnya!

---

## 7. Rubrik Penilaian Praktikum
| Aspek Penilaian | Skor Maksimal | Kriteria Pencapaian |
| :--- | :---: | :--- |
| **Integrasi Fusi Sensor** | 25 | Berhasil menjalankan simulasi fusi LiDAR 360 + RGB-D + IMU secara sinkron pada RTAB-Map. |
| **Operasional Navigasi Otonom** | 25 | Mampu mengoperasikan Nav2, menentukan goal pose interaktif, dan menjalankan skrip multi-waypoint. |
| **Pengujian & Pengumpulan Metrik** | 25 | Mengisi tabel komparasi metode dan tabel navigasi berdasarkan data pengujian aktual. |
| **Jawaban Pertanyaan Diskusi** | 25 | Mampu menganalisis fungsi Voxel Layer, interaksi Costmap dengan geometri robot, dan justifikasi fusi sensor. |
| **Total Skor** | **100** | |
