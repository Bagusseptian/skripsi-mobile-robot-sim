# LABSHEET 4: NAVIGASI OTONOM (NAV2) DAN 3D VOXEL OBSTACLE LAYER DARI KAMERA RGB-D

**Mata Kuliah:** Penginderaan Visual Robot  
**Program Studi:** S1 Pendidikan Teknik Mekatronika, Universitas Negeri Yogyakarta  
**Penyusun:** Muhammad Bagus Septian  
**Alokasi Waktu:** 3 x 50 Menit (1 Pertemuan Praktikum)

---

## 1. Tujuan Pembelajaran
Setelah menyelesaikan praktikum ini, mahasiswa diharapkan mampu:
1. Mengonfigurasi arsitektur navigasi otonom *Navigation 2 (Nav2)* pada robot bergerak beroda empat (*4WD Mobile Robot*).
2. Membedakan fungsi dan cara kerja *Global Costmap* (perencanaan rute makro) dan *Local Costmap* (penghindaran rintangan reaktif).
3. Menganalisis integrasi sensor multi-dimensi: *2D Obstacle Layer* (LiDAR) dan *3D Voxel Layer* (Point Cloud kamera RGB-D Astra).
4. Melakukan *tuning* pengontrol pergerakan robot *Regulated Pure Pursuit Controller* serta *PID Velocity Filter* untuk mencegah *oversteering* dan selip.
5. Menjalankan pengujian navigasi otonom multi-waypoint otomatis dan menghitung persentase tingkat keberhasilan (*success rate*).

---

## 2. Alat dan Bahan
1. Laptop / PC dengan OS Ubuntu 22.04 LTS (ROS 2 Humble terinstalasi lengkap).
2. Simulator Gazebo dengan paket simulasi `robot_sim`.
3. Tumpukan navigasi `nav2_bringup`, `nav2_regulated_pure_pursuit_controller`, dan `nav2_costmap_2d`.
4. Profil visualisasi RViz2 `nav2_view.rviz`.
5. Skrip otomatisasi navigasi `nav2_goal_tester.py`.
6. Dunia simulasi ruang kelas `world_c_classroom.world` dan lorong `world_b_corridor.world`.

---

## 3. Dasar Teori Singkat

### A. Konsep Costmap 2D dan Lapisan Voxel 3D
Costmap merepresentasikan tingkat keamanan navigasi robot dalam bentuk kisi probabilitas bahaya:
1. **Static Layer:** Peta lingkungan global yang dihasilkan dari proses SLAM.
2. **Obstacle Layer (2D LaserScan):** Bidang potong laser horizontal 360° untuk mendeteksi dinding dan kaki furnitur.
3. **Voxel Layer (3D Point Cloud):** Kisi 3D beresolusi volumetrik ($0.05\text{ m}$) yang diproyeksikan dari citra kedalaman Orbbec Astra Pro (`/camera/points`). Voxel Layer mampu mendeteksi rintangan yang menggantung atau berada di luar jangkauan ketinggian sinar laser horizontal (misalnya permukaan daun meja komputer, monitor, tempat sampah, atau objek melayang).
4. **Inflation Layer:** Menambahkan radius penyangga keselamatan di sekeliling batas fisik robot ($r_{\text{inflation}} = 0.35\text{ m}$) agar badan robot tidak menyenggol rintangan.

### B. Kontroler Regulated Pure Pursuit & Filter PID
*Regulated Pure Pursuit* bekerja dengan menghitung titik acuan (*lookahead point*) pada jalur target kemudian memerintahkan kecepatan sudut ($\omega$) yang proporsional terhadap kurvatur belokan:
$$\kappa = \frac{2 \sin(\alpha)}{L}$$
Di mana $\alpha$ adalah sudut antara orientasi robot dengan titik acuan, dan $L$ adalah jarak lookahead ($0.50\text{ m}$). 

Pada robot virtual 4WD skid-steer, perputaran tempat dapat memicu lonjakan gaya gesek dan selip (*oversteering*). Untuk itu, paket `robot_sim` menyematkan node filter PID (`pid_velocity_controller.py`) yang menghaluskan transisi percepatan sudut ($\Delta \omega$) sehingga manuver robot stabil dan presisi.

---

## 4. Langkah Kerja

### Langkah 1: Menjalankan Simulasi Ruang Kelas (World C)
Buka **Terminal 1**, jalankan Gazebo simulasi:
```bash
source /opt/ros/humble/setup.bash
source ~/skripsi_ws/install/setup.bash
ros2 launch robot_sim sim.launch.py world:=world_c
```
*Perhatikan:* Robot akan di-spawn di lorong tengah kelas bebas halangan. Filter PID velocity controller otomatis aktif di latar belakang.

### Langkah 2: Menjalankan Fusi Sensor Penuh (SLAM Fusion)
Buka **Terminal 2**, jalankan RTAB-Map dengan fusi sensor:
```bash
source ~/skripsi_ws/install/setup.bash
ros2 launch robot_sim slam_fusion.launch.py
```
*Amati:* RTAB-Map memetakan grid okupansi 2D dan point cloud 3D secara bersamaan pada topik `/map`.

### Langkah 3: Menjalankan Navigasi Otonom (Nav2)
Buka **Terminal 3**, jalankan tumpukan Nav2 beserta RViz2:
```bash
source ~/skripsi_ws/install/setup.bash
ros2 launch robot_sim nav2.launch.py
```
*Amati tampilan RViz2:*
- Lapisan *Global Costmap* berwarna cyan/ungu membungkus dinding kelas.
- Jendela *Image* di pojok kiri bawah menampilkan pandangan kamera RGB-D real-time.
- Titik laser berwarna hijau neon dan point cloud 3D penuh warna menyelimuti meja lab.

### Langkah 4: Uji Coba Navigasi Interaktif (Nav2 Goal)
1. Pada bilah atas RViz2, klik tombol **Nav2 Goal**.
2. Klik kiri pada area bebas di depan papan tulis kelas, lalu seret panah hijau untuk menentukan orientasi hadap robot.
3. Amati jalur lintasan hijau (*Global Plan*) terbentuk, robot mulai bergerak maju, melintasi celah antar meja, dan berhenti presisi di titik tujuan.

### Langkah 5: Pengujian Otomatis 5 Titik Waypoint (Benchmarking)
Buka **Terminal 4**, jalankan skrip penguji otonom:
```bash
source ~/skripsi_ws/install/setup.bash
ros2 run robot_sim nav2_goal_tester.py --world world_c --name uji_nav_kelas
```
Skrip akan menguji 5 titik waypoint berurutan di ruang kelas dan menghasilkan file log evaluasi di folder `~/skripsi_ws/eval_results/`.

---

## 5. Lembar Pengamatan Praktikum

Catat hasil pengujian 5 waypoint pada tabel berikut:

| No | Nama Target Waypoint | Koordinat Target $(x, y)$ | Waktu Tempuh (detik) | Status Hasil (Succeeded / Aborted) | Catatan Observasi Pergerakan Robot |
| :---: | :--- | :---: | :---: | :---: | :--- |
| 1 | Goal 1 (Depan Papan Tulis) | $(0.0, 3.5)$ | | | |
| 2 | Goal 2 (Meja Lab Kanan 1) | $(4.2, 1.0)$ | | | |
| 3 | Goal 3 (Lorong Belakang Kanan)| $(4.2, -3.5)$ | | | |
| 4 | Goal 4 (Lorong Belakang Kiri) | $(-4.2, -3.5)$ | | | |
| 5 | Goal 5 (Kembali Depan Kelas) | $(0.0, 1.5)$ | | | |

* **Tingkat Keberhasilan Navigasi (%):** .................... %  
* **Rata-rata Waktu Capai per Goal:** .................... detik

---

## 6. Tugas Analisis dan Diskusi
1. Mengapa keberadaan lapisan *3D Voxel Layer* sangat krusial saat robot bernavigasi di lingkungan laboratorium yang memiliki banyak meja komputer berkaki?
2. Bagaimana pengaruh parameter `inflation_radius` terhadap kemampuan robot melintasi lorong sempit antar-meja? Jelaskan apa yang terjadi jika radius diatur terlalu besar ($>0.5\text{ m}$) atau terlalu kecil ($<0.2\text{ m}$).
3. Jelaskan peranan node `pid_velocity_controller.py` dalam meredam fenomena *oversteering* dan getaran putaran saat robot melakukan belokan tajam di sudut lorong!
