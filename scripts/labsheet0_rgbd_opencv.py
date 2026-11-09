#!/usr/bin/env python3
"""
labsheet0_rgbd_opencv.py
========================
Skrip praktikum Labsheet 0: jembatan dari materi OpenCV (citra 2D) ke kamera RGB-D.

Setiap mode memakai materi buku "Langkah Mudah Pemrograman OpenCV & Python"
(Abdul Kadir), tetapi datanya diambil dari kamera RGB-D robot simulasi:

  info       Bab 4  : citra = matriks piksel (ukuran, tipe data, encoding)
  depth      Bab 4  : membaca nilai piksel depth (jarak dalam meter)
  histogram  Bab 5  : histogram citra depth (sebaran jarak di depan robot)
  filter     Bab 7  : median blur untuk mengurangi derau depth
  obstacle   Bab 4,8: citra biner (threshold jarak) + morfologi + kontur rintangan
  features   Bab 9  : titik fitur GFTT pada citra RGB (detektor yang dipakai RTAB-Map)
  point      ---    : proyeksi piksel (u, v, Z) ke titik 3D (X, Y, Z)

Contoh:
  ros2 run robot_sim labsheet0_rgbd_opencv.py --mode depth
  ros2 run robot_sim labsheet0_rgbd_opencv.py --mode obstacle --jarak 1.0

Hasil gambar disimpan di ~/labsheet0_hasil/. Tambahkan --show untuk menampilkan
jendela OpenCV (butuh tampilan grafis, misalnya WSLg).
"""

import argparse
import os
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

TOPIK_RGB = '/camera/image_raw'
TOPIK_DEPTH = '/camera/depth/image_raw'
TOPIK_INFO = '/camera/camera_info'
FOLDER_HASIL = os.path.expanduser('~/labsheet0_hasil')


class Labsheet0(Node):
    def __init__(self, args):
        super().__init__('labsheet0_rgbd_opencv')
        self.args = args
        self.bridge = CvBridge()
        self.rgb = None
        self.depth = None
        self.K = None
        self.create_subscription(Image, TOPIK_RGB, self.cb_rgb, 10)
        self.create_subscription(Image, TOPIK_DEPTH, self.cb_depth, 10)
        self.create_subscription(CameraInfo, TOPIK_INFO, self.cb_info, 10)

    def cb_rgb(self, msg):
        self.rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def cb_depth(self, msg):
        if '32F' in msg.encoding:
            self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1').copy()
        else:  # 16UC1 dalam milimeter (kamera fisik)
            self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough').astype(np.float32) / 1000.0
        self.depth_encoding = msg.encoding

    def cb_info(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)

    def siap(self):
        return self.rgb is not None and self.depth is not None and self.K is not None


def depth_ke_warna(depth, maks=6.0):
    """Ubah depth (meter) menjadi citra berwarna agar mudah dilihat."""
    d = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
    d8 = np.clip(d / maks * 255.0, 0, 255).astype(np.uint8)
    warna = cv2.applyColorMap(d8, cv2.COLORMAP_JET)
    warna[d <= 0] = (0, 0, 0)  # piksel tidak valid (di luar jangkauan 0,3-6,0 m) ditampilkan hitam
    return warna


def simpan(nama, img, show):
    os.makedirs(FOLDER_HASIL, exist_ok=True)
    path = os.path.join(FOLDER_HASIL, nama)
    cv2.imwrite(path, img)
    print(f'  Gambar disimpan: {path}')
    if show:
        cv2.imshow(nama, img)


def valid(depth):
    """Masker piksel depth yang valid (bukan NaN/inf/0)."""
    return np.isfinite(depth) & (depth > 0)


def mode_info(n):
    print('=== MODE INFO: citra adalah matriks piksel (Bab 4) ===')
    print(f'  RGB   : shape={n.rgb.shape}, dtype={n.rgb.dtype}')
    print(f'  Depth : shape={n.depth.shape}, dtype={n.depth.dtype}, encoding={n.depth_encoding}')
    print('  Perbedaan: piksel RGB berisi 3 angka 0-255 (B, G, R),')
    print('             piksel depth berisi 1 angka pecahan = jarak dalam meter.')
    simpan('info_rgb.png', n.rgb, n.args.show)
    simpan('info_depth_warna.png', depth_ke_warna(n.depth), n.args.show)


def mode_depth(n):
    print('=== MODE DEPTH: membaca nilai piksel (Bab 4) ===')
    h, w = n.depth.shape
    titik = [(w // 2, h // 2, 'tengah'), (w // 4, h // 2, 'kiri'), (3 * w // 4, h // 2, 'kanan'),
             (w // 2, h // 4, 'atas'), (w // 2, 3 * h // 4, 'bawah')]
    vis = depth_ke_warna(n.depth)
    for u, v, nama in titik:
        z = n.depth[v, u]
        b, g, r = n.rgb[v, u]
        teks = f'{z:.3f} m' if np.isfinite(z) else 'tidak valid'
        print(f'  Piksel {nama:7s} (u={u:3d}, v={v:3d}) -> depth = {teks:12s} | RGB = ({r}, {g}, {b})')
        cv2.circle(vis, (u, v), 6, (255, 255, 255), 2)
        cv2.putText(vis, teks, (min(u + 8, w - 90), v - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    m = valid(n.depth)
    print(f'  Piksel valid: {m.sum()} dari {m.size} ({100 * m.mean():.1f}%)')
    simpan('depth_titik.png', vis, n.args.show)


def mode_histogram(n):
    print('=== MODE HISTOGRAM: sebaran jarak (Bab 5) ===')
    d = n.depth[valid(n.depth)]
    hist, tepi = np.histogram(d, bins=12, range=(0.0, 6.0))
    for jumlah, a, b in zip(hist, tepi[:-1], tepi[1:]):
        bar = '#' * int(50 * jumlah / max(hist.max(), 1))
        print(f'  {a:4.1f}-{b:4.1f} m | {jumlah:7d} | {bar}')
    print(f'  Jarak minimum = {d.min():.3f} m, rata-rata = {d.mean():.3f} m, maksimum = {d.max():.3f} m')
    # Gambar histogram dengan OpenCV (tanpa matplotlib)
    img = np.full((300, 600, 3), 255, np.uint8)
    lebar = 600 // len(hist)
    for i, jumlah in enumerate(hist):
        tinggi = int(260 * jumlah / max(hist.max(), 1))
        cv2.rectangle(img, (i * lebar + 5, 280 - tinggi), ((i + 1) * lebar - 5, 280), (200, 100, 0), -1)
        cv2.putText(img, f'{tepi[i]:.1f}', (i * lebar + 8, 295), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0), 1)
    simpan('histogram_depth.png', img, n.args.show)


def mode_filter(n):
    print('=== MODE FILTER: mengurangi derau depth (Bab 7) ===')
    d = np.nan_to_num(n.depth, nan=0.0, posinf=0.0, neginf=0.0).astype(np.float32)
    if n.args.derau > 0:
        # Citra depth dari simulator hampir bebas derau, berbeda dengan kamera fisik.
        # Derau Gaussian ditambahkan agar efek filter terlihat (bagian dari sim-to-real gap).
        rng = np.random.default_rng(0)
        d = np.where(d > 0, d + rng.normal(0.0, n.args.derau, d.shape).astype(np.float32), 0.0).astype(np.float32)
        print(f'  Derau Gaussian ditambahkan: simpangan baku {n.args.derau * 1000:.0f} mm')
    else:
        print('  Tanpa derau tambahan (citra depth asli simulator)')
    k = n.args.kernel
    halus = cv2.medianBlur(d, k)  # medianBlur menerima float32 untuk kernel 3 atau 5

    def derau(img):
        # Simpangan baku lokal 5x5 di setiap piksel, lalu diambil mediannya.
        # Median dipakai supaya tepi benda (lompatan jarak besar) tidak ikut terhitung sebagai derau.
        rata = cv2.blur(img, (5, 5))
        rata2 = cv2.blur(img * img, (5, 5))
        sd = np.sqrt(np.maximum(rata2 - rata * rata, 0))
        return float(np.median(sd[(img > 0) & (halus > 0)])) * 1000.0

    print(f'  Kernel median = {k}x{k}')
    print(f'  Derau lokal (median simpangan baku 5x5) sebelum filter : {derau(d):.2f} mm')
    print(f'  Derau lokal (median simpangan baku 5x5) sesudah filter : {derau(halus):.2f} mm')
    simpan('filter_sebelum.png', depth_ke_warna(d), n.args.show)
    simpan('filter_sesudah.png', depth_ke_warna(halus), n.args.show)

    # Perbesaran (zoom) 160x160 piksel di sekitar (u, v) dengan skala warna sempit (±30 mm),
    # karena derau beberapa milimeter tidak terlihat pada skala warna 0-6 m.
    h, w = d.shape
    u = n.args.u if n.args.u is not None else w // 2
    v = n.args.v if n.args.v is not None else h // 2
    u0, v0 = min(max(u - 80, 0), w - 160), min(max(v - 80, 0), h - 160)
    potong = (slice(v0, v0 + 160), slice(u0, u0 + 160))
    acuan = halus[potong][halus[potong] > 0]
    if acuan.size == 0:
        print(f'  Area zoom di sekitar ({u}, {v}) tidak berisi depth valid. Pilih --u/--v lain.')
        return
    tengah = float(np.median(acuan))

    def zoom(img):
        z = np.clip((img[potong] - (tengah - 0.03)) / 0.06 * 255.0, 0, 255).astype(np.uint8)
        z = cv2.applyColorMap(z, cv2.COLORMAP_JET)
        z[img[potong] <= 0] = (0, 0, 0)
        return cv2.resize(z, (480, 480), interpolation=cv2.INTER_NEAREST)

    print(f'  Zoom: area 160x160 di sekitar (u={u}, v={v}), skala warna {tengah - 0.03:.2f}-{tengah + 0.03:.2f} m')
    simpan('filter_zoom_sebelum.png', zoom(d), n.args.show)
    simpan('filter_zoom_sesudah.png', zoom(halus), n.args.show)


def mode_obstacle(n):
    print('=== MODE OBSTACLE: citra biner + morfologi + kontur (Bab 4 & 8) ===')
    batas = n.args.jarak
    m = valid(n.depth) & (n.depth < batas)
    if not n.args.dengan_lantai:
        # Pada frame optik kamera, sumbu Y mengarah ke bawah. Titik lantai berada
        # sekitar setinggi kamera di bawah sumbu optik: Y = (v - cy) * Z / fy ≈ tinggi kamera.
        fy, cy = n.K[1, 1], n.K[1, 2]
        v = np.arange(n.depth.shape[0], dtype=np.float32).reshape(-1, 1)
        Y = (v - cy) * np.nan_to_num(n.depth, nan=0.0, posinf=0.0, neginf=0.0) / fy
        m &= Y < (n.args.tinggi_kamera - 0.03)
        print(f'  Lantai diabaikan (titik dengan Y >= {n.args.tinggi_kamera - 0.03:.2f} m dibuang)')
    biner = (m * 255).astype(np.uint8)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    bersih = cv2.morphologyEx(biner, cv2.MORPH_OPEN, kernel)  # buang bintik kecil
    kontur, _ = cv2.findContours(bersih, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    kontur = [c for c in kontur if cv2.contourArea(c) > 300]
    vis = n.rgb.copy()
    print(f'  Batas jarak = {batas:.2f} m -> {len(kontur)} rintangan terdeteksi')
    for i, c in enumerate(kontur, 1):
        x, y, w, h = cv2.boundingRect(c)
        jarak = float(np.median(n.depth[y:y + h, x:x + w][m[y:y + h, x:x + w]]))
        print(f'  Rintangan {i}: kotak (x={x}, y={y}, w={w}, h={h}), jarak median = {jarak:.2f} m')
        cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 0, 255), 2)
        tx = min(x, vis.shape[1] - 70)  # jaga label tetap di dalam gambar
        cv2.putText(vis, f'{jarak:.2f} m', (tx, max(y - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    simpan('obstacle_biner.png', bersih, n.args.show)
    simpan('obstacle_hasil.png', vis, n.args.show)


def mode_features(n):
    print('=== MODE FEATURES: titik fitur GFTT (Bab 9, dipakai RTAB-Map) ===')
    abu = cv2.cvtColor(n.rgb, cv2.COLOR_BGR2GRAY)
    titik = cv2.goodFeaturesToTrack(abu, maxCorners=n.args.maks_fitur, qualityLevel=0.01, minDistance=7)
    jumlah = 0 if titik is None else len(titik)
    vis = n.rgb.copy()
    punya_depth = 0
    if titik is not None:
        for p in titik.reshape(-1, 2).astype(int):
            u, v = int(p[0]), int(p[1])
            ada = np.isfinite(n.depth[v, u]) and n.depth[v, u] > 0
            punya_depth += int(ada)
            cv2.circle(vis, (u, v), 3, (0, 255, 0) if ada else (0, 0, 255), -1)
    print(f'  Jumlah fitur terdeteksi : {jumlah}')
    print(f'  Fitur dengan depth valid: {punya_depth} (hijau). Merah = tanpa depth.')
    print('  RTAB-Map hanya bisa memakai fitur yang punya nilai depth untuk membentuk titik 3D.')
    simpan('features_gftt.png', vis, n.args.show)


def baca_point_cloud(node, u, v, batas_waktu=5.0):
    """Ambil titik (u, v) dari point cloud terorganisasi /camera/points sebagai pembanding."""
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    hasil = {}

    def cb(msg):
        if 'p' in hasil:
            return
        if msg.height <= 1:
            hasil['p'] = None  # point cloud tidak terorganisasi
            return
        idx = v * msg.width + u
        titik = list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=False))
        hasil['p'] = tuple(float(c) for c in titik[idx]) if idx < len(titik) else None
        hasil['frame'] = msg.header.frame_id

    sub = node.create_subscription(PointCloud2, '/camera/points', cb, 5)
    waktu = 0.0
    while 'p' not in hasil and waktu < batas_waktu:
        rclpy.spin_once(node, timeout_sec=0.1)
        waktu += 0.1
    node.destroy_subscription(sub)
    return hasil.get('p'), hasil.get('frame')


def mode_point(n):
    print('=== MODE POINT: piksel (u, v) + depth Z -> titik 3D ===')
    fx, fy, cx, cy = n.K[0, 0], n.K[1, 1], n.K[0, 2], n.K[1, 2]
    print(f'  Parameter intrinsik dari /camera/camera_info: fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}')
    h, w = n.depth.shape
    u = n.args.u if n.args.u is not None else w // 2
    v = n.args.v if n.args.v is not None else h // 2
    if not (0 <= u < w and 0 <= v < h):
        print(f'  Piksel ({u}, {v}) di luar citra. u harus 0-{w - 1}, v harus 0-{h - 1}.')
        return
    z = float(n.depth[v, u])
    if not np.isfinite(z) or z <= 0:
        print(f'  Depth di piksel ({u}, {v}) tidak valid (hitam pada depth_titik.png). Pilih piksel lain.')
        return
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    print(f'  Piksel terpilih: u={u}, v={v}, Z (nilai piksel depth) = {z:.3f} m')
    print(f'  X = (u - cx) * Z / fx = ({u} - {cx:.1f}) * {z:.3f} / {fx:.1f} = {x:.3f} m')
    print(f'  Y = (v - cy) * Z / fy = ({v} - {cy:.1f}) * {z:.3f} / {fy:.1f} = {y:.3f} m')
    print(f'  Hasil rumus        : (X, Y, Z) = ({x:.3f}, {y:.3f}, {z:.3f}) m')
    print('  Arah sumbu frame optik kamera: X ke kanan, Y ke bawah, Z ke depan.')
    pc, frame = baca_point_cloud(n, u, v)
    if pc is None or not all(np.isfinite(pc)):
        print('  Pembanding /camera/points: tidak tersedia untuk piksel ini.')
    else:
        print(f'  Pembanding /camera/points (frame {frame}): ({pc[0]:.3f}, {pc[1]:.3f}, {pc[2]:.3f}) m')
        selisih = float(np.linalg.norm(np.array([x, y, z]) - np.array(pc)))
        print(f'  Selisih jarak hasil rumus dan point cloud: {selisih * 1000:.1f} mm')
    vis = n.rgb.copy()
    cv2.drawMarker(vis, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 24, 2)
    cv2.circle(vis, (int(round(cx)), int(round(cy))), 4, (255, 255, 0), -1)
    teks = f'(u={u}, v={v}) -> X={x:.2f} Y={y:.2f} Z={z:.2f} m'
    cv2.rectangle(vis, (0, h - 36), (w, h), (40, 40, 40), -1)
    cv2.putText(vis, teks, (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(vis, 'pusat citra (cx, cy)', (int(round(cx)) + 8, int(round(cy)) + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1, cv2.LINE_AA)
    simpan(f'point_u{u}_v{v}.png', vis, n.args.show)


MODE = {'info': mode_info, 'depth': mode_depth, 'histogram': mode_histogram, 'filter': mode_filter,
        'obstacle': mode_obstacle, 'features': mode_features, 'point': mode_point}


def main():
    p = argparse.ArgumentParser(description='Labsheet 0: OpenCV untuk kamera RGB-D')
    p.add_argument('--mode', choices=MODE.keys(), default='info')
    p.add_argument('--jarak', type=float, default=1.0, help='batas jarak rintangan (meter), mode obstacle')
    p.add_argument('--dengan-lantai', dest='dengan_lantai', action='store_true',
                   help='mode obstacle: jangan buang titik lantai (untuk melihat masalahnya)')
    p.add_argument('--tinggi-kamera', dest='tinggi_kamera', type=float, default=0.14,
                   help='tinggi kamera dari lantai (meter), mode obstacle')
    p.add_argument('--derau', type=float, default=0.01,
                   help='mode filter: simpangan baku derau Gaussian tambahan (meter), 0 = tanpa derau')
    p.add_argument('--kernel', type=int, choices=[3, 5], default=5, help='ukuran kernel median, mode filter')
    p.add_argument('--maks-fitur', dest='maks_fitur', type=int, default=500, help='mode features')
    p.add_argument('--u', type=int, help='kolom piksel, mode point dan pusat zoom mode filter')
    p.add_argument('--v', type=int, help='baris piksel, mode point dan pusat zoom mode filter')
    p.add_argument('--show', action='store_true', help='tampilkan jendela OpenCV')
    args = p.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = Labsheet0(args)
    print('Menunggu data kamera RGB-D (pastikan simulasi sudah berjalan)...')
    waktu = 0.0
    while rclpy.ok() and not node.siap() and waktu < 15.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        waktu += 0.1
    if not node.siap():
        print('Data kamera tidak diterima dalam 15 detik. Cek: ros2 topic list | grep camera')
    else:
        MODE[args.mode](node)
        if args.show:
            print('Tekan sembarang tombol pada jendela gambar untuk keluar.')
            cv2.waitKey(0)
            cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
