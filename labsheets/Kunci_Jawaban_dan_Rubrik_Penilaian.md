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

## IV. KUNCI JAWABAN LABSHEET 4: NAVIGASI OTONOM DAN 3D VOXEL LAYER

### Kunci Pertanyaan Analisis & Diskusi Labsheet 4
1. **Mengapa keberadaan 3D Voxel Layer sangat krusial di laboratorium komputer?**  
   *Jawaban:* Meja laboratorium memiliki kolong berketinggian 70 cm dengan permukaan daun meja setinggi pinggang dan monitor komputer di atasnya. Berkas sinar LiDAR 2D setinggi 13 cm hanya melihat kaki meja yang kecil berbentuk silinder/kotak tipis, mengira area di antara kaki-kaki meja adalah jalan kosong terbuka. Akibatnya, robot tanpa 3D Voxel Layer akan mencoba menerobos masuk ke bawah meja dan menabrak rangka atau kabel meja. Voxel Layer memproyeksikan point cloud 3D Orbbec Astra, menandai volume udara di bawah dan setinggi meja sebagai zona bahaya occupied pada local costmap, sehingga robot merencanakan jalur memutar di lorong bebas.
2. **Pengaruh parameter `inflation_radius` terhadap navigasi lorong sempit:**  
   *Jawaban:* `inflation_radius` menentukan ketebalan lapisan penyangga keselamatan di sekitar rintangan. Jika diatur terlalu besar ($>0.5\text{ m}$), zona bahaya dari meja kiri dan meja kanan akan saling bertumpuk (*overlap*), menutup total lorong tengah selebar 1.0 m sehingga robot mengira jalan buntu dan membatalkan misi (*aborted*). Sebaliknya, jika diatur terlalu kecil ($<0.2\text{ m}$), batas inflasi lebih kecil dari radius robot fisik (0.22 m), sehingga controller dapat mengarahkan robot terlalu dekat dengan sudut meja dan memicu tabrakan fisik (*collision*) saat berbelok. Nilai optimal $0.35\text{ m}$ memberikan clearance yang pas untuk lorong lab.
3. **Peran `pid_velocity_controller.py` dalam meredam oversteering:**  
   *Jawaban:* Pada kinematika skid-steer 4WD di simulator Gazebo, percepatan sudut impulsif dari Nav2 memicu slip gesek dinamik dan osilasi (*hunting behavior*). Node `pid_velocity_controller` mengimplementasikan low-pass filter dan kontrol proporsional-integral pada perintah `/cmd_vel_nav`, membatasi lonjakan akselerasi sudut ($\dot{\omega} \le 0.8\text{ rad/s}^2$) dan memperhalus profil kecepatan saat robot memasuki dan keluar dari tikungan tajam, menghasilkan lintasan mulus tanpa getaran.

---

## V. KUNCI JAWABAN LABSHEET 5: EVALUASI KUANTITATIF ATE DAN BEBAN SISTEM

### Kunci Pertanyaan Analisis & Diskusi Labsheet 5
1. **Konfigurasi dengan ATE RMSE terkecil dan faktor fisisnya:**  
   *Jawaban:* Skenario 3 (Fusi Multi-Sensor) menghasilkan nilai ATE RMSE terkecil ($\approx 0.04 - 0.08\text{ m}$), jauh lebih presisi dibandingkan Visual SLAM murni ($\approx 0.25 - 0.45\text{ m}$) dan LiDAR murni ($\approx 0.10 - 0.15\text{ m}$). Keunggulan ini disebabkan oleh mekanisme komplementer: 360° LiDAR mengunci geometri dinding agar odometri tidak mengalami angular drift, sementara kamera RGB-D mengekstrak ratusan visual keypoints untuk mendeteksi loop closure fotometrik yang mengeliminasi akumulasi kesalahan translasi saat robot menyelesaikan sirkuit.
2. **Penyebab drift besar pada Skenario 2 (RGB-D Saja) di lorong panjang:**  
   *Jawaban:* Di lorong panjang simetris, dinding samping kiri dan kanan seringkali berwarna monokrom/polos tanpa tekstur warna atau poster. Karena kamera RGB-D hanya memiliki FOV $87^\circ$ ke depan, saat robot melaju lurus di sepanjang lorong, fitur visual yang dapat diekstraksi sangat sedikit dan terkonsentrasi di ujung lorong yang jauh. Estimasi kedalaman pada jarak jauh ($>4\text{ m}$) memiliki varian noise kuadratik ($\sigma_z \propto z^2$), menyebabkan estimasi translasi maju mengalami skala drift (*scale drift*).
3. **Rekomendasi implementasi pada embedded hardware nyata (Jetson Orin Nano / RPi 5):**  
   *Jawaban:* Pada perangkat seperti NVIDIA Jetson Orin Nano (6-core ARM CPU, GPU Ampere), Fusi Sensor Penuh dapat dijalankan dengan memanfaatkan akselerasi CUDA pada RTAB-Map dan OpenCV. Namun, jika menggunakan prosesor tanpa GPU bertenaga (seperti Raspberry Pi 4/5), rekomendasi terbaik adalah menurunkan resolusi depth kamera ke $320 \times 240$ pada 15 FPS, atau menggunakan mode 2D LiDAR SLAM (`slam_toolbox`) sebagai estimator posisi utama yang hemat daya, dengan kamera RGB-D dikhususkan sebagai sensor deteksi rintangan 3D lokal di Nav2.

---

## VI. RUBRIK PENILAIAN KOMPREHENSIF TIGA RANAH (STANDAR OBE FT UNY)

### 1. Ranah Psikomotorik / Keterampilan (Bobot 50%)
* **Skor 41-50:** Mahasiswa terampil mengoperasikan terminal ROS 2, mampu mengatasi kendala sistem mandiri, peta yang dibangun simetris dan tajam tanpa distorsi, navigasi 5 waypoint 100% sukses.
* **Skor 31-40:** Mahasiswa mampu menjalankan seluruh simulasi dan merekam data dengan panduan minimal asisten; peta memiliki sedikit noise yang tidak mengganggu navigasi.
* **Skor 21-30:** Mahasiswa membutuhkan bantuan teknis intensif; pemetaan mengalami kegagalan loop closure atau robot terjebak rintangan berulang kali.
* **Skor < 20:** Mahasiswa tidak mampu menyelesaikan prosedur praktikum.

### 2. Ranah Kognitif / Pengetahuan (Bobot 35%)
* **Skor 31-35:** Analisis lembar diskusi mendalam, mengaitkan rumus matematis ATE RMSE, matriks TF, dan kinematika skid-steer dengan data empiris pengujian.
* **Skor 25-30:** Jawaban benar secara konseptual, namun penjelasan matematis atau korelasi dengan log data kurang mendalam.
* **Skor 18-24:** Jawaban singkat dan bersifat teoritis umum tanpa mengaitkan ke hasil observasi laboratorium.
* **Skor < 18:** Jawaban salah atau tidak menjawab pertanyaan analisis.

### 3. Ranah Afektif / Sikap Kerja (Bobot 15%)
* **Skor 13-15:** Disiplin waktu, menjaga kebersihan dan ketertiban laboratorium, menunjukkan etos kerja ilmiah yang jujur dalam mencatat data log numerik tanpa manipulasi.
* **Skor 10-12:** Tertib dan mengumpulkan laporan tepat waktu, namun kurang proaktif dalam diskusi praktikum.
* **Skor < 10:** Terlambat mengumpulkan tugas atau melakukan plagiasi data pengamatan teman sekelompok.
