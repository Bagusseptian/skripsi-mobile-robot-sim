# PANDUAN INSTALASI UNTUK MAHASISWA
## Media Pembelajaran Simulasi Mobile Robot dengan Visual SLAM

**Mata Kuliah:** Penginderaan Visual Robot  
**Program Studi:** S1 Pendidikan Teknik Mekatronika, Universitas Negeri Yogyakarta  
**Penyusun:** Muhammad Bagus Septian

Panduan ini dipakai satu kali sebelum Labsheet 0. Ikuti langkah sesuai urutan. Perkiraan waktu: 60–90 menit, sebagian besar untuk mengunduh paket.

---

## 1. Spesifikasi Laptop

| Komponen | Minimum | Disarankan |
| :--- | :--- | :--- |
| Sistem operasi | Windows 10 versi 21H2 / Windows 11 (dengan WSL2), atau Ubuntu 22.04 | Windows 11 dengan WSL2, atau Ubuntu 22.04 *native* |
| Prosesor | 4 inti (*core*) | 8 inti atau lebih |
| RAM | 8 GB | 16 GB |
| Ruang kosong | 30 GB | 50 GB |
| Kartu grafis | Grafis terintegrasi | GPU terpisah (NVIDIA/AMD) |

Dasar angka di atas: pada pengujian proyek ini, Gazebo memakai sekitar 0,9 GB RAM dan 1,1–1,4 inti CPU, sedangkan RTAB-Map memakai 0,9–2,3 GB RAM dan hingga 1,7 inti CPU. Ditambah RViz2 dan sistem operasi, laptop dengan RAM 8 GB masih bisa dipakai tetapi akan lambat bila semua program dibuka bersamaan.

---

## 2. Memasang WSL2 dan Ubuntu 22.04 (khusus pengguna Windows)
Lewati bagian ini jika laptop sudah memakai Ubuntu 22.04.

1. Buka **PowerShell** sebagai *Administrator*, lalu jalankan:
   ```powershell
   wsl --install -d Ubuntu-22.04
   ```
2. *Restart* laptop jika diminta.
3. Buka aplikasi **Ubuntu 22.04** dari menu Start. Buat *username* dan *password* Linux (catat, akan sering dipakai untuk `sudo`).
4. Pastikan versi WSL adalah 2:
   ```powershell
   wsl -l -v
   ```
   Kolom `VERSION` harus bernilai `2`.
5. Perbarui driver kartu grafis di Windows. Jendela Gazebo dan RViz2 tampil melalui WSLg, yang sudah tersedia di Windows 11 dan Windows 10 versi terbaru.

---

## 3. Memasang ROS 2 Humble dan Paket Pendukung
Semua perintah berikut dijalankan di terminal Ubuntu.

### Langkah 1: Repositori ROS 2
```bash
sudo apt update && sudo apt install -y software-properties-common curl git
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Langkah 2: ROS 2, Gazebo, SLAM, dan Navigasi
```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-rtabmap-ros \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-xacro \
  ros-humble-teleop-twist-keyboard \
  python3-colcon-common-extensions \
  python3-psutil \
  python3-matplotlib \
  evince
```

### Langkah 3: Aktifkan ROS 2 otomatis setiap membuka terminal
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 4. Memasang Paket Media Pembelajaran (`robot_sim`)

1. Buat *workspace* dan salin paket `robot_sim` ke dalamnya. Paket dibagikan oleh dosen/asisten dalam bentuk folder atau *repository*:
   ```bash
   mkdir -p ~/skripsi_ws/src
   cd ~/skripsi_ws/src
   # salin folder robot_sim ke sini, contoh dari flashdisk/Windows:
   # cp -r /mnt/c/Users/<nama_user>/Downloads/robot_sim .
   ```
2. *Build* paket:
   ```bash
   cd ~/skripsi_ws
   colcon build --symlink-install
   echo "source ~/skripsi_ws/install/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```
3. Pastikan *build* selesai dengan pesan `Summary: 1 package finished`.

---

## 5. Uji Coba Instalasi
Jalankan ketiga uji berikut. Jika semuanya berhasil, laptop siap dipakai untuk Labsheet 0–5.

| No | Perintah | Hasil yang Benar |
| :---: | :--- | :--- |
| 1 | `ros2 launch robot_sim sim.launch.py world:=world_a` | Jendela Gazebo terbuka dan robot muncul di tengah arena |
| 2 | (terminal baru) `ros2 topic hz /scan` | Frekuensi sekitar 10 Hz |
| 3 | (terminal baru) `ros2 run robot_sim labsheet0_rgbd_opencv.py --mode info` | Muncul ukuran citra RGB (480, 640, 3) dan citra *depth* (480, 640) |

Hentikan program dengan menekan `Ctrl + C` di setiap terminal.

---

## 6. Masalah yang Sering Terjadi

| Gejala | Penyebab | Solusi |
| :--- | :--- | :--- |
| `ros2: command not found` | ROS 2 belum di-*source* | Jalankan `source /opt/ros/humble/setup.bash`, atau ulangi Bagian 3 Langkah 3 |
| `Package 'robot_sim' not found` | *Workspace* belum di-*build* atau belum di-*source* | Ulangi Bagian 4 Langkah 2 |
| Jendela Gazebo tidak muncul di WSL | WSLg belum aktif atau driver grafis lama | Jalankan `wsl --update` di PowerShell, perbarui driver grafis, lalu *restart* |
| Gazebo sangat lambat atau macet | RAM/CPU tidak cukup | Tutup aplikasi lain; jalankan Gazebo tanpa jendela dengan `gui:=false` dan amati hasil di RViz2 saja |
| Robot tidak muncul / simulasi lama berhenti | Proses Gazebo sebelumnya masih berjalan | Jalankan `pkill -f gzserver; pkill -f gzclient`, lalu ulangi |
| Skrip Labsheet 0: "Data kamera tidak diterima" | Simulasi belum berjalan | Jalankan simulasi terlebih dahulu (uji nomor 1) |
