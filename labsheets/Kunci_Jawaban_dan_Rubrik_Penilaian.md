# PANDUAN DOSEN & ASISTEN: KUNCI JAWABAN DAN RUBRIK PENILAIAN LABSHEET

**Mata Kuliah:** Penginderaan Visual Robot  
**Program Studi:** S1 Pendidikan Teknik Mekatronika, Universitas Negeri Yogyakarta  
**Penyusun:** Muhammad Bagus Septian  
**Sasaran Pengguna:** Dosen Pengampu & Asisten Laboratorium Robotika / Penginderaan Visual

---

## I. KUNCI JAWABAN LABSHEET 1: INSTALASI DAN LIDAR 360°

### Kunci Tabel Pengamatan 1.1: Karakteristik Topik ROS 2
- `/scan`: Tipe `sensor_msgs/msg/LaserScan`, Frekuensi aktual $\approx 10.0\text{ Hz}$, Frame ID: `laser_link`.
- `/odom`: Tipe `nav_msgs/msg/Odometry`, Frekuensi aktual $\approx 20.0 - 50.0\text{ Hz}$, Frame ID: `odom` (Child: `base_footprint`).
- `/map`: Tipe `nav_msgs/msg/OccupancyGrid`, Frekuensi aktual $\approx 0.5 - 2.0\text{ Hz}$ (latched / periodik saat ada pembaruan), Frame ID: `map`.

### Kunci Tabel Pengamatan 1.2: Parameter LaserScan
- `angle_min`: $-3.14159\text{ rad}$ ($-180^\circ$)
- `angle_max`: $+3.14159\text{ rad}$ ($+180^\circ$)
- `angle_increment`: $\approx 0.01745\text{ rad}$ ($1.0^\circ$ per sampel, total 360 sampel)
- `range_min`: $0.15\text{ meter}$ (zona buta fisik sensor)
- `range_max`: $12.0\text{ meter}$ (jangkauan tembak maksimum laser)

### Kunci Pertanyaan Analisis & Diskusi Labsheet 1
1. **Mengapa nilai `range_min` harus lebih besar dari jari-jari badan robot?**  
   *Jawaban:* Badan robot memiliki dimensi fisik (panjang 36 cm, lebar 26 cm, radius ekivalen sekitar 22 cm). Sensor LiDAR diletakkan di atas sasis. Jika `range_min = 0.0\text{ m}`, berkas sinar laser yang mengarah ke bawah atau terpantul oleh casing, baut, tiang penyangga sensor, atau kabel robot akan dianggap sebagai rintangan nyata (*self-collision artifact*). Akibatnya, peta grid okupansi pada posisi robot langsung terisi sel hitam (*occupied*), yang membuat algoritma path planning mengira robot terjebak rintangan dan menolak bergerak (*planning failure*).
2. **Mengapa terjadi distorsi peta (drift) saat robot berputar cepat sebelum loop closure?**  
   *Jawaban:* Ketika robot berputar cepat, terjadi slip gesek antara roda dan lantai (*wheel slip*), sehingga estimasi odometri roda mengakumulasi kesalahan sudut (*angular drift*). Selain itu, scan matching LiDAR memiliki latensi sampling (10 Hz). Jika pergeseran sudut antar dua frame scan terlalu besar, algoritma optimasi non-linear (*Ceres solver*) gagal menemukan konvergensi global dan terjebak pada *local minima*, menghasilkan dinding berganda (*ghost walls*) hingga robot kembali ke area yang telah dipetakan untuk memicu *loop closure*.
3. **Jelaskan peran transformasi TF antara `odom` dan `map`! Mengapa selalu berubah?**  
   *Jawaban:* Frame `odom` mengukur pergerakan lokal berbasis putaran roda yang terus menerus mengalami akumulasi kesalahan pergeseran (*cumulative drift*). Frame `map` adalah koordinat global dunia yang sebenarnya (statis). Tugas `slam_toolbox` adalah membandingkan sapuan laser saat ini dengan peta global, lalu menghitung koreksi offset pose. Offset koreksi ini dipublikasikan sebagai transformasi dinamis dari `map` ke `odom`. Dengan demikian, formula pose absolut robot adalah:
   $$T_{\text{map} \to \text{base}} = T_{\text{map} \to \text{odom}} \times T_{\text{odom} \to \text{base}}$$
   Karena drift pada `odom` terus bertambah, maka $T_{\text{map} \to \text{odom}}$ harus diperbarui secara berkala oleh SLAM untuk mengkompensasi drift tersebut.

---

## II. KUNCI JAWABAN LABSHEET 2: KAMERA RGB-D DAN VISUAL SLAM

### Kunci Tabel Pengamatan 2.1: Analisis Karakteristik Citra RGB-D
- **Citra RGB (`/camera/image_raw`):** Encoding `rgb8` (atau `bgr8`), resolusi $640 \times 480$ piksel, 3 saluran (Red, Green, Blue, 8-bit per saluran).
- **Peta Depth (`/camera/depth/image_raw`):** Encoding `32FC1` (floating point 32-bit dalam meter) atau `16UC1` (unsigned int 16-bit dalam milimeter), resolusi $640 \times 480$ piksel, 1 saluran.
- **Jarak Terukur:** Minimum $0.3\text{ meter}$, Maksimum $6.0\text{ meter}$.

### Kunci Tabel Pengamatan 2.2: Visual Features dan Loop Closure
- Jumlah fitur terdeteksi berkisar antara **150 hingga 400 keypoints** per frame di area kaya tekstur World C.
- Di depan papan tulis putih polos yang minim tekstur, jumlah fitur menurun tajam (< 40 keypoints).
- Ketika robot kembali ke posisi awal, status *Loop Closure Detected* bernilai **YA (Hijau)**, ditandai dengan reduksi mendadak pose error dan penyesuaian visualisasi awan titik 3D.

### Kunci Pertanyaan Analisis & Diskusi Labsheet 2
1. **Mengapa kamera RGB-D butuh tekstur dan cahaya, sedangkan LiDAR 2D tidak?**  
   *Jawaban:* LiDAR 2D adalah sensor aktif monokromatik yang memancarkan sumber pulsa lasernya sendiri dan hanya mengukur waktu pantulan pada satu bidang iris horizontal. Ia tidak peduli apakah dinding memiliki gambar atau polos. Sebaliknya, kamera RGB-D mengandalkan deteksi pola visual (seperti gradien kecerahan sudut, tepian, dan tekstur warna menggunakan algoritma ORB/FAST/BRIEF) untuk mengenali lokasi dan membedakan ruangan. Jika ruangan gelap gulita atau dinding putih polos sempurna tanpa tekstur, tidak ada fitur gradien (*keypoints*) yang dapat diekstraksi, sehingga Visual SLAM kehilangan referensi (*tracking lost*).
2. **Apa yang terjadi pada nilai kedalaman jika objek lebih dekat dari 0.3 m atau berupa kaca transparan?**  
   *Jawaban:* Pada jarak < 0.3 m (*blind zone*), sinar proyeksi inframerah kamera tidak dapat membentuk paralaks triangulasi yang valid pada sensor penerima, menghasilkan nilai piksel kedalaman `NaN` (*Not a Number*) atau 0. Pada permukaan kaca/cermin transparan, berkas inframerah akan menembus kaca atau terpantul ke arah lain (*specular reflection*), sehingga jarak yang terbaca adalah jarak objek di balik kaca atau tidak terdeteksi sama sekali.
3. **Perbedaan Visual Odometry dengan Visual SLAM:**  
   *Jawaban:* *Visual Odometry* (VO) hanya menghitung estimasi perpindahan posisi robot secara lokal dari frame kamera $k-1$ ke frame $k$ (incremental). VO tidak mengingat riwayat masa lalu dan kesalahan posisi akan terus terakumulasi tanpa batas (*unbounded drift*). Sebaliknya, *Visual SLAM* membangun peta global (*global map*) dan graf lokasi (*pose graph*). Ketika robot mengunjungi kembali tempat yang pernah dilihatnya beberapa menit atau jam lalu, Visual SLAM mendeteksi *Loop Closure*, menghitung galat penutupan loop, dan mengoptimasi seluruh graf secara serempak (*bundle adjustment / pose-graph optimization*), sehingga galat akumulatif kembali ke mendekati nol.

---

## III. KUNCI JAWABAN LABSHEET 3: FUSI SENSOR DAN NAVIGASI OTONOM

### Kunci Tabel Pengamatan 3.1: Perbandingan Karakteristik Metode SLAM
| Aspek Evaluasi | LiDAR Saja (`slam_toolbox`) | RGB-D Saja (`rtabmap`) | Fusi Sensor (`rtabmap`) |
| :--- | :---: | :---: | :---: |
| Cakupan Sudut Pandang Sensor | $360^\circ$ Penuh | Terbatas ($\approx 87^\circ$) | $360^\circ$ Penuh + $87^\circ$ 3D |
| Kemampuan Deteksi Rintangan 3D | Rendah (hanya 1 bidang iris 2D) | Tinggi (Volume 3D kerucut) | Sangat Tinggi (Komprehensif) |
| Ketahanan di Lorong Simetris Polos | Rendah (bisa tertukar karena dinding simetris) | Rendah (minim tekstur) | Tinggi (saling melengkapi) |
| Beban Komputasi CPU (%) | Rendah ($\approx 15-25\%$) | Menengah-Tinggi ($\approx 35-50\%$) | Tinggi ($\approx 45-65\%$) |

### Kunci Pertanyaan Analisis & Diskusi Labsheet 3
1. **Mengapa Voxel Layer 3D sangat krusial di lingkungan kantor/laboratorium? Berikan contoh rintangan nyata!**  
   *Jawaban:* Sensor LiDAR 2D dipasang pada ketinggian tetap (misal setinggi 13 cm dari lantai). Objek-objek seperti kaki meja dengan kolong kosong di atas 15 cm, papan meja yang menonjol keluar, monitor yang tergantung, tempat sampah pendek, atau kardus di lantai di bawah jangkauan berkas laser 2D tidak akan tampak pada `/scan`. Tanpa *Voxel Layer*, robot akan menabrak kolong meja atau rintangan tersebut. Kamera RGB-D memproyeksikan kerucut penglihatan 3D (*point cloud*), sehingga rintangan berketinggian berapa pun di dalam FOV kamera langsung diubah menjadi sel rintangan (*occupied voxels*) pada *Local Costmap*.
2. **Reaksi DWB controller dan recovery behaviors jika celah terhalang sisa 0.3 m:**  
   *Jawaban:* Robot memiliki lebar sasis 0.26 m dan jari-jari aman inflasi 0.40 m. Total footprint + inflasi membutuhkan koridor minimal $\approx 0.60 - 0.80\text{ m}$. Celah 0.3 m sepenuhnya tertutup oleh warna merah/kuning (*lethal / inscribed cost*) pada costmap. 
   - Local planner DWB mendeteksi bahwa semua trajektori kecepatan menuju celah tersebut bernilai biaya tak terhingga (*cost violation*), sehingga robot berhenti.
   - BT Navigator mengaktifkan *Recovery Behaviors*:
     1. Menunggu beberapa detik (*Wait behavior*).
     2. Menghapus costmap dan memindai ulang sensor.
     3. Melakukan putaran di tempat (*Spin behavior*) untuk mencari alternatif jalur keluar.
     4. Jika tetap tidak ada jalur bebas rintangan, Nav2 membatalkan tugas (*Navigation Aborted*).
3. **Kesimpulan mengapa Fusi Sensor paling ideal untuk robot fisik:**  
   *Jawaban:* Pada robot fisik nyata, lingkungan tidak pernah ideal: lantai memiliki slip, dinding bisa memiliki area kaca/polos, dan ruangan memiliki rintangan bertingkat 3D. Fusi sensor menggabungkan keandalan geometri 360 derajat LiDAR (mencegah tabrakan saat robot berputar dan menjaga konsistensi dinding) dengan kecerdasan visual kamera RGB-D (mendeteksi rintangan 3D di luar bidang laser dan memberikan verifikasi loop closure berbasis tekstur visual). Kelemahan masing-masing sensor saling ditutupi oleh keunggulan sensor pasangannya.

---

## IV. STANDAR RUBRIK PENILAIAN PRAKTIKUM (SKALA 100)

| Rentang Nilai | Predikat | Kriteria Deskriptif Kualitatif |
| :---: | :---: | :--- |
| **86 - 100** | **Amat Baik (A)** | Mahasiswa menguasai seluruh prosedur mandiri; simulasi Gazebo, SLAM, dan Nav2 berjalan mulus; seluruh tabel pengamatan terisi dengan data riil dan akurat; analisis komparatif mendalam dan mengaitkan konsep teoritis dengan fenomena fisis robotik. |
| **76 - 85** | **Baik (B)** | Mahasiswa mampu menjalankan simulasi dan menyelesaikan pemetaan serta navigasi dengan sedikit bantuan; tabel pengamatan terisi lengkap; analisis pertanyaan benar secara garis besar namun kurang detail pada aspek matematis/TF. |
| **60 - 75** | **Cukup (C)** | Mahasiswa mengalami kendala saat konfigurasi atau teleoperasi; peta yang dihasilkan mengalami sedikit cacat/distorsi; pengamatan terisi sebagian; jawaban analisis bersifat dangkal atau sekadar mengulang teori tanpa mengaitkan ke data pengujian. |
| **< 60** | **Kurang (D/E)** | Mahasiswa tidak mampu menyelesaikan pemetaan atau navigasi; tidak mengisi lembar kerja; jawaban analisis tidak relevan atau tidak mengumpulkan laporan praktikum. |
