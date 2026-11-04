# LABSHEET 2: EKSPLORASI KAMERA RGB-D, POINT CLOUD 3D, DAN VISUAL SLAM (RTAB-MAP)

**Mata Kuliah:** Penginderaan Visual Robot  
**Program Studi:** S1 Pendidikan Teknik Mekatronika, Universitas Negeri Yogyakarta  
**Penyusun:** Muhammad Bagus Septian  
**Alokasi Waktu:** 3 x 50 Menit (1 Pertemuan Praktikum)

---

## 1. Tujuan Pembelajaran
Setelah menyelesaikan praktikum ini, mahasiswa diharapkan mampu:
1. Memahami prinsip kerja sensor kamera RGB-D (kombinasi citra warna RGB dan peta kedalaman *Depth Map*).
2. Memvisualisasikan dan merekonstruksi awan titik 3D (*3D Point Cloud*) menggunakan pesan `sensor_msgs/msg/PointCloud2` pada RViz2.
3. Menjalankan algoritma Visual SLAM berbasis *Real-Time Appearance-Based Mapping* (RTAB-Map) tanpa menggunakan sensor laser LiDAR.
4. Menganalisis mekanisme deteksi *Loop Closure* berbasis pencocokan fitur visual (*Visual Bag-of-Words*).
5. Mengeksplorasi struktur basis data grafis RTAB-Map (`.db`) menggunakan perangkat visualisasi basis data.

---

## 2. Alat dan Bahan
1. Komputer / Laptop dengan OS Ubuntu 22.04 LTS (Native / WSL2).
2. ROS 2 Humble dengan paket simulator `robot_sim`.
3. Paket `rtabmap_ros` dan `rtabmap_launch`.
4. Dunia simulasi World C (`world_c_classroom.world`) yang kaya akan objek dan tekstur visual.
5. Node kemudi `teleop_wasd.py`.

---

## 3. Dasar Teori Singkat

### A. Prinsip Kamera RGB-D
Kamera RGB-D (seperti Orbbec Astra Pro, Intel RealSense D435) menghasilkan dua jenis data tersinkronisasi:
1. **Citra Warna (RGB):** Resolusi $640 \times 480$, format matriks piksel warna 24-bit.
2. **Peta Kedalaman (Depth Map):** Matriks piksel berformat 16-bit (*16UC1*) atau floating point 32-bit (*32FC1*) di mana tiap nilai piksel menyatakan jarak fisik objek dari bidang lensa kamera (dalam milimeter atau meter).

Dengan matriks intrinsik kamera $K$:
$$K = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}$$
Setiap piksel $(u, v)$ dengan nilai kedalaman $Z$ diproyeksikan menjadi koordinat 3D $(X, Y, Z)$ di ruang nyata:
$$X = \frac{(u - c_x) \times Z}{f_x}, \quad Y = \frac{(v - c_y) \times Z}{f_y}$$

### B. Visual SLAM dan RTAB-Map
RTAB-Map adalah pendekatan *Graph-Based SLAM* yang didesain untuk pemetaan jangka panjang (*long-term mapping*). Komponen utamanya:
- **Short-Term Memory (STM):** Menyimpan lokasi-lokasi pose terkini.
- **Working Memory (WM):** Menyeleksi node-node representatif berdasarkan bobot untuk menjaga komputasi real-time tetap ringan.
- **Long-Term Memory (LTM):** Menyimpan memori visual yang jarang dikunjungi ke dalam database SQLite.
- **Loop Closure Detection:** Mencocokkan deskriptor fitur visual (ORB/SURF/BRIEF) citra kamera saat ini dengan citra pada memori masa lalu untuk mengeliminasi akumulasi drift odometri.

---

## 4. Langkah Kerja

### Langkah 1: Membuka Simulasi Dunia Kelas Bertekstur (World C)
1. Buka Terminal 1, jalankan simulasi pada World C:
   ```bash
   source ~/skripsi_ws/install/setup.bash
   ros2 launch robot_sim sim.launch.py world:=world_c
   ```
2. Amati bahwa ruangan World C memiliki meja komputer, papan tulis, poster dinding berwarna-warni, dan monitor yang menyediakan kontras visual tinggi.

### Langkah 2: Inspeksi Data RGB, Depth, dan Point Cloud
1. Buka Terminal 2, periksa topik kamera aktif:
   ```bash
   ros2 topic list | grep camera
   ```
2. Periksa frekuensi aliran data kedalaman dan citra warna:
   ```bash
   ros2 topic hz /camera/image_raw
   ros2 topic hz /camera/depth/image_raw
   ros2 topic hz /camera/points
   ```
3. Buka jendela visualisasi sensor untuk melihat citra dan point cloud secara berdampingan:
   ```bash
   ros2 launch robot_sim rviz_sim.launch.py
   ```
   *(Amati kerapatan titik 3D pada objek meja dan dinding)*.

### Langkah 3: Menjalankan Visual SLAM RGB-D Murni
1. Buka Terminal 3, aktifkan RTAB-Map dalam mode Visual SLAM murni (LiDAR dinonaktifkan):
   ```bash
   source ~/skripsi_ws/install/setup.bash
   ros2 launch robot_sim slam_rgbd.launch.py
   ```
2. Antarmuka grafis `rtabmap_viz` akan muncul, menampilkan:
   - Jendela kiri atas: Citra RGB dengan titik-titik fitur visual berwarna (keypoints).
   - Jendela kanan: Rekonstruksi awan titik 3D global dan graf lintasan (*pose graph*).

### Langkah 4: Eksplorasi dan Pengujian Loop Closure
1. Buka Terminal 4, jalankan pengendali keyboard:
   ```bash
   source ~/skripsi_ws/install/setup.bash
   ros2 run robot_sim teleop_wasd.py
   ```
2. Kemudikan robot menyusuri lorong antar meja di World C, lalu putari barisan meja hingga robot kembali menghadap ke titik start awal.
3. Amati layar `rtabmap_viz`:
   - Ketika robot melihat kembali pemandangan yang sama, bilah bawah akan berubah warna menjadi hijau dengan tulisan **"Loop Closure Detected!"**.
   - Perhatikan bagaimana graf lintasan (*pose graph*) seketika mengoreksi posisi robot agar klop dengan posisi awal.

### Langkah 5: Menghentikan dan Menyimpan Database
1. Tekan `Ctrl+C` pada Terminal peluncur RTAB-Map.
2. Basis data graf visual otomatis tersimpan pada direktori `~/.ros/rtabmap.db`.

---

## 5. Lembar Pengamatan Mahasiswa

### Tabel 2.1: Analisis Karakteristik Citra RGB-D
| Parameter | Citra RGB (`/camera/image_raw`) | Peta Depth (`/camera/depth/image_raw`) |
| :--- | :--- | :--- |
| Tipe Encoding Piksel | ......................... | ......................... |
| Resolusi Lebar x Tinggi | ......................... | ......................... |
| Jarak Terukur Minimum | - | ........... meter |
| Jarak Terukur Maksimum | - | ........... meter |

### Tabel 2.2: Pengamatan Visual Features dan Loop Closure pada RTAB-Map
| Posisi Robot | Jumlah Fitur Terdeteksi (*Inliers*) | Status Loop Closure (Ya / Tidak) | Nilai Pose Error Teramati |
| :--- | :---: | :---: | :---: |
| Titik Awal (Start) | | | |
| Depan Papan Tulis | | | |
| Lorong Belakang | | | |
| Kembali Menghadap Start | | | |

---

## 6. Pertanyaan Analisis dan Diskusi
1. Mengapa kamera RGB-D sangat membutuhkan lingkungan dengan pencahayaan yang cukup dan dinding yang kaya akan tekstur visual, sedangkan sensor LiDAR 2D dapat bekerja sempurna di ruangan gelap gulita dengan dinding polos tanpa tekstur?
2. Apa yang terjadi pada nilai kedalaman (*depth value*) jika objek yang dilihat robot berada lebih dekat dari $0.3\text{ meter}$ (*blind zone*) atau terbuat dari kaca transparan/permukaan cermin?
3. Jelaskan perbedaan mendasar antara *Visual Odometry* dengan *Visual SLAM*! Mengapa *Visual Odometry* saja tidak cukup untuk pemetaan jangka panjang tanpa mekanisme *Loop Closure*?

---

## 7. Rubrik Penilaian Praktikum
| Aspek Penilaian | Skor Maksimal | Kriteria Pencapaian |
| :--- | :---: | :--- |
| **Konfigurasi Sensor RGB-D** | 20 | Mampu mengidentifikasi topik, format encoding citra RGB & depth, dan merekonstruksi 3D Point Cloud. |
| **Operasional Visual SLAM** | 30 | Berhasil menjalankan RTAB-Map visual murni dan mengendalikan robot hingga tercipta visual map 3D. |
| **Analisis Loop Closure** | 25 | Mampu menunjukkan dan mencatat terjadinya loop closure saat robot kembali ke posisi awal. |
| **Jawaban Pertanyaan Diskusi** | 25 | Memberikan analisis mendalam mengenai keterbatasan optik, blind zone sensor, dan perbedaan odometri vs SLAM. |
| **Total Skor** | **100** | |
