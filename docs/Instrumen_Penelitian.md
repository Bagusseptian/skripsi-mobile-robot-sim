# INSTRUMEN PENELITIAN
## Pengembangan Media Pembelajaran Simulasi Mobile Robot dengan Visual SLAM Berbasis Kamera RGB-D dan LiDAR pada Mata Kuliah Penginderaan Visual Robot

**Penyusun:** Muhammad Bagus Septian  
**Status:** Draf. Kisi-kisi dan butir disusun peneliti dari isi produk; aspek penilaian perlu dirujuk ke teori evaluasi media di Bab II dan dikonsultasikan dengan dosen pembimbing sebelum dipakai.

Dokumen ini berisi tiga instrumen:
- **Instrumen A** — Lembar validasi ahli materi (rumusan masalah: kelayakan menurut ahli materi)
- **Instrumen B** — Lembar validasi ahli media (rumusan masalah: kelayakan menurut ahli media)
- **Instrumen C** — Angket respons mahasiswa (rumusan masalah: kepraktisan)

Uji unjuk kerja simulasi tidak memakai angket. Datanya berasal dari skrip pengujian (`benchmark_ate.py`, `benchmark_system.py`, `nav2_goal_tester.py`).

---

## Skala Penilaian
Ketiga instrumen memakai skala Likert empat tingkat tanpa pilihan netral:

| Skor | Instrumen A dan B | Instrumen C |
| :---: | :--- | :--- |
| 4 | Sangat Baik | Sangat Setuju |
| 3 | Baik | Setuju |
| 2 | Kurang | Tidak Setuju |
| 1 | Sangat Kurang | Sangat Tidak Setuju |

Skala empat tingkat dipilih agar responden tidak memilih nilai tengah. Jika pembimbing menghendaki skala lima tingkat, ubah tabel di atas dan rumus skor maksimum.

---

## INSTRUMEN A — LEMBAR VALIDASI AHLI MATERI

**Identitas Validator**  
Nama: ........................................................  
NIP/NIDN: ........................................................  
Bidang keahlian: ........................................................  
Tanggal: ........................................................

**Petunjuk**
1. Bapak/Ibu dimohon menilai materi pada paket media: Labsheet 0–5, kunci jawaban, dan rubrik penilaian.
2. Beri tanda centang (✓) pada kolom skor yang sesuai.
3. Saran dan komentar ditulis pada bagian akhir lembar ini.

### Kisi-kisi
| Aspek | Indikator | Nomor Butir |
| :--- | :--- | :---: |
| Kesesuaian dengan kurikulum | Kesesuaian dengan capaian mata kuliah, tujuan pembelajaran, dan posisi sebagai materi lanjutan | 1–4 |
| Kebenaran konsep | Ketepatan konsep sensor, SLAM, fusi sensor, dan navigasi | 5–9 |
| Kelengkapan dan urutan materi | Cakupan materi dan urutan dari mudah ke sulit | 10–13 |
| Kualitas labsheet | Kejelasan langkah kerja, lembar pengamatan, pertanyaan diskusi, dan rubrik | 14–18 |
| Bahasa | Kejelasan dan ketepatan bahasa serta istilah | 19–21 |

### Butir Penilaian
| No | Pernyataan | 4 | 3 | 2 | 1 |
| :---: | :--- | :---: | :---: | :---: | :---: |
| 1 | Tujuan pembelajaran setiap labsheet sesuai dengan capaian mata kuliah Penginderaan Visual Robot | | | | |
| 2 | Materi melanjutkan materi pengolahan citra dengan OpenCV yang sudah dipelajari sebelumnya | | | | |
| 3 | Labsheet 0 menghubungkan materi OpenCV (piksel, histogram, filter, morfologi, fitur) dengan citra kedalaman | | | | |
| 4 | Materi sesuai dengan perkembangan teknologi robot bergerak | | | | |
| 5 | Konsep prinsip kerja LiDAR 360° disajikan dengan benar | | | | |
| 6 | Konsep kamera RGB-D dan citra kedalaman disajikan dengan benar | | | | |
| 7 | Konsep SLAM, pemetaan grid okupansi, dan *loop closure* disajikan dengan benar | | | | |
| 8 | Konsep fusi LiDAR dan kamera RGB-D disajikan dengan benar | | | | |
| 9 | Konsep evaluasi akurasi lokalisasi (ATE) dan beban komputasi disajikan dengan benar | | | | |
| 10 | Materi mencakup sensor, pemetaan, fusi sensor, navigasi, dan evaluasi | | | | |
| 11 | Urutan labsheet bergerak dari materi sederhana ke materi yang lebih kompleks | | | | |
| 12 | Dasar teori pada setiap labsheet cukup untuk mengerjakan langkah kerja | | | | |
| 13 | Contoh dan lingkungan uji (tiga dunia simulasi) mendukung pemahaman materi | | | | |
| 14 | Langkah kerja jelas dan dapat diikuti secara berurutan | | | | |
| 15 | Lembar pengamatan sesuai dengan data yang dihasilkan langkah kerja | | | | |
| 16 | Pertanyaan diskusi mendorong mahasiswa menganalisis, bukan hanya mengingat | | | | |
| 17 | Rubrik penilaian jelas dan sesuai dengan tujuan pembelajaran | | | | |
| 18 | Kunci jawaban benar dan membantu dosen/asisten menilai | | | | |
| 19 | Bahasa yang digunakan jelas dan mudah dipahami mahasiswa | | | | |
| 20 | Istilah teknis digunakan secara tepat dan konsisten | | | | |
| 21 | Penulisan sesuai kaidah Bahasa Indonesia | | | | |

**Saran dan komentar:**  
...........................................................................................................  
...........................................................................................................

**Kesimpulan** (lingkari salah satu):  
1. Layak digunakan tanpa revisi  
2. Layak digunakan dengan revisi sesuai saran  
3. Tidak layak digunakan

Yogyakarta, ..............................  
Validator,

(...........................................)

---

## INSTRUMEN B — LEMBAR VALIDASI AHLI MEDIA

**Identitas Validator**  
Nama: ........................................................  
NIP/NIDN: ........................................................  
Bidang keahlian: ........................................................  
Tanggal: ........................................................

**Petunjuk**
1. Bapak/Ibu dimohon mencoba simulasi dengan mengikuti *Panduan Instalasi untuk Mahasiswa* dan minimal satu labsheet.
2. Beri tanda centang (✓) pada kolom skor yang sesuai.
3. Saran dan komentar ditulis pada bagian akhir lembar ini.

### Kisi-kisi
| Aspek | Indikator | Nomor Butir |
| :--- | :--- | :---: |
| Fungsi dan kinerja | Simulasi, sensor virtual, SLAM, dan navigasi berjalan sesuai fungsi | 1–6 |
| Kemudahan penggunaan | Instalasi, menjalankan program, dan alur penggunaan | 7–11 |
| Tampilan visual | Tampilan Gazebo, RViz2, peta, dan gambar hasil | 12–15 |
| Kelengkapan dan dokumentasi | Panduan instalasi, labsheet, dan dokumentasi pendukung | 16–19 |
| Kemanfaatan media | Fleksibilitas dan kemungkinan pengembangan | 20–22 |

### Butir Penilaian
| No | Pernyataan | 4 | 3 | 2 | 1 |
| :---: | :--- | :---: | :---: | :---: | :---: |
| 1 | Simulasi Gazebo berjalan stabil tanpa *crash* | | | | |
| 2 | Sensor LiDAR 360° virtual menghasilkan data yang sesuai dengan lingkungan | | | | |
| 3 | Kamera RGB-D virtual menghasilkan citra warna dan citra kedalaman yang sesuai | | | | |
| 4 | Ketiga konfigurasi SLAM menghasilkan peta lingkungan | | | | |
| 5 | Navigasi otonom (Nav2) membawa robot ke titik tujuan | | | | |
| 6 | Skrip evaluasi (ATE, beban komputasi, navigasi) menghasilkan data dan grafik | | | | |
| 7 | Langkah instalasi mudah diikuti | | | | |
| 8 | Program dapat dijalankan dengan perintah yang sederhana | | | | |
| 9 | Pengendalian robot (teleoperasi) mudah dilakukan | | | | |
| 10 | Pesan kesalahan dan solusi masalah pada panduan membantu ketika terjadi kendala | | | | |
| 11 | Media dapat dijalankan pada laptop dengan spesifikasi minimum yang dicantumkan | | | | |
| 12 | Model robot dan lingkungan simulasi tampil jelas | | | | |
| 13 | Visualisasi data sensor di RViz2 mudah diamati | | | | |
| 14 | Peta 2D dan *point cloud* 3D yang dihasilkan jelas | | | | |
| 15 | Gambar dan grafik hasil skrip mudah dibaca | | | | |
| 16 | Panduan instalasi lengkap dan sesuai dengan kondisi sebenarnya | | | | |
| 17 | Tata letak labsheet rapi dan konsisten | | | | |
| 18 | Setiap labsheet memuat tujuan, alat, dasar teori, langkah kerja, pengamatan, diskusi, dan rubrik | | | | |
| 19 | Dokumentasi pendukung (README, panduan transisi ke perangkat keras) memadai | | | | |
| 20 | Media dapat dipakai mahasiswa secara mandiri di laptop masing-masing | | | | |
| 21 | Lingkungan dan parameter simulasi dapat diubah untuk percobaan lain | | | | |
| 22 | Media dapat dikembangkan ke robot fisik pada penelitian selanjutnya | | | | |

**Saran dan komentar:**  
...........................................................................................................  
...........................................................................................................

**Kesimpulan** (lingkari salah satu):  
1. Layak digunakan tanpa revisi  
2. Layak digunakan dengan revisi sesuai saran  
3. Tidak layak digunakan

Yogyakarta, ..............................  
Validator,

(...........................................)

---

## INSTRUMEN C — ANGKET RESPONS MAHASISWA (KEPRAKTISAN)

**Identitas Responden**  
Nama/NIM: ........................................................  
Kelas: ........................................................  
Sistem operasi laptop: Windows + WSL2 / Ubuntu *native* (lingkari)

**Petunjuk**
1. Isi angket setelah mengerjakan Labsheet 0 sampai Labsheet yang ditentukan dosen.
2. Beri tanda centang (✓) pada kolom yang paling sesuai dengan pengalaman Anda.
3. Jawaban tidak memengaruhi nilai mata kuliah.

### Kisi-kisi
| Aspek | Indikator | Nomor Butir |
| :--- | :--- | :---: |
| Kemudahan penggunaan | Instalasi, menjalankan simulasi, dan mengikuti labsheet | 1–5 |
| Kejelasan materi | Pemahaman dasar teori dan hubungan dengan materi sebelumnya | 6–9 |
| Kebermanfaatan | Pemahaman SLAM, sensor, dan relevansi dengan industri | 10–13 |
| Ketertarikan | Minat dan motivasi belajar | 14–16 |
| Efisiensi waktu | Kesesuaian alokasi waktu praktikum | 17–18 |

### Butir Pernyataan
| No | Pernyataan | 4 | 3 | 2 | 1 |
| :---: | :--- | :---: | :---: | :---: | :---: |
| 1 | Saya dapat memasang media ini dengan mengikuti panduan instalasi | | | | |
| 2 | Saya dapat menjalankan simulasi tanpa bantuan asisten | | | | |
| 3 | Langkah kerja pada labsheet mudah diikuti | | | | |
| 4 | Perintah-perintah yang digunakan mudah dipahami | | | | |
| 5 | Simulasi berjalan lancar di laptop saya | | | | |
| 6 | Dasar teori pada labsheet mudah dipahami | | | | |
| 7 | Labsheet 0 membantu saya menghubungkan materi OpenCV dengan kamera RGB-D | | | | |
| 8 | Saya memahami perbedaan data LiDAR dan kamera RGB-D setelah praktikum | | | | |
| 9 | Pertanyaan diskusi membantu saya memahami materi lebih dalam | | | | |
| 10 | Media ini membantu saya memahami cara kerja SLAM | | | | |
| 11 | Visualisasi peta dan lintasan robot membantu saya memahami hubungan data sensor, posisi robot, dan peta | | | | |
| 12 | Saya dapat membandingkan hasil pemetaan dari konfigurasi sensor yang berbeda | | | | |
| 13 | Materi ini bermanfaat untuk bekal saya di dunia kerja atau penelitian | | | | |
| 14 | Belajar dengan simulasi robot lebih menarik daripada hanya membaca teori | | | | |
| 15 | Media ini membuat saya ingin mempelajari robotika lebih lanjut | | | | |
| 16 | Saya dapat bereksperimen tanpa takut merusak alat | | | | |
| 17 | Waktu praktikum cukup untuk menyelesaikan setiap labsheet | | | | |
| 18 | Saya dapat mengulang praktikum di luar jam kuliah | | | | |

**Kesan dan saran:**  
...........................................................................................................

---

## Teknik Analisis Data

### 1. Persentase skor
Untuk setiap validator atau responden, dan untuk keseluruhan:

$$P = \frac{\sum \text{skor yang diperoleh}}{\sum \text{skor maksimum}} \times 100\%$$

Skor maksimum = jumlah butir × 4 × jumlah responden. Contoh: Instrumen A dengan 21 butir dan 2 validator memiliki skor maksimum $21 \times 4 \times 2 = 168$.

### 2. Kategori
Contoh pembagian kategori dengan empat interval yang sama untuk skala 1–4:

| Persentase | Kategori Kelayakan (A, B) | Kategori Kepraktisan (C) |
| :---: | :--- | :--- |
| 81,25% – 100% | Sangat Layak | Sangat Praktis |
| 62,50% – < 81,25% | Layak | Praktis |
| 43,75% – < 62,50% | Kurang Layak | Kurang Praktis |
| 25,00% – < 43,75% | Tidak Layak | Tidak Praktis |

Batas interval diperoleh dari rentang 25%–100% (skor minimum 1/4 hingga maksimum 4/4) yang dibagi empat sama besar. Pembagian kategori ini perlu dirujuk ke sumber metodologi yang dipakai di Bab III, atau disesuaikan dengan panduan pembimbing.

### 3. Reliabilitas angket mahasiswa
Reliabilitas Instrumen C dapat diuji dengan koefisien *Cronbach's Alpha* setelah data terkumpul. Validitas isi Instrumen A–C diperoleh dari penilaian ahli (*expert judgment*) sebelum instrumen dipakai.
