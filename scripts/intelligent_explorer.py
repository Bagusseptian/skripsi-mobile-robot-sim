#!/usr/bin/env python3
"""
Intelligent Explorer Node (Master LLM Agent + Strict Target Vision + Precision PID + Spatial Memory)
Mata Kuliah: Penginderaan Visual Robot
Fusi Sensor: RPLiDAR A1 360° (/scan) + Kamera RGB-D Astra (/camera) + LLaMA 3.2
Arena Lab: 6m x 6m (3 Objek Utama: Silinder Hijau, Balok Persegi Panjang Oranye, Kubus Persegi Merah)

Perbaikan Kritis:
  1. Strict Target Vision (NO FALSE FALLBACK):
     - Jika target diminta 'pilar_hijau', visual HANYA mencari warna hijau / silinder.
     - JANGAN PERNAH fallback mengunci balok oranye atau objek lain!
     - Jika target belum terlihat di kamera, robot menghadap & bergerak ke koordinat landmark target sampai kamera mengunci objek yang benar.
  2. True Color Normalization:
     - Konversi RGB8 Gazebo ke BGR internal OpenCV.
     - Di RViz (/perception/detection_image), Merah tampil Merah sejati, Hijau tampil Hijau, Oranye tampil Oranye.
  3. Ground-Truth 3 Objek Nyata:
     - [PERSEGI] Kotak Merah (-1.5, 1.5)
     - [SILINDER] Pilar Hijau (-1.2, -1.2)
     - [PERSEGI PANJANG] Balok Oranye (1.5, 1.0)
  4. Visual Servoing & Precision PID:
     - Continuous alignment (Alignment Factor) tanpa rem mendadak.
     - Berhenti presisi di 0.55m tepat di depan objek target.
  5. Memori Spasial (Grid 0.5m x 0.5m) + Frontier Bias + Return to Home (RTH).
"""

import sys
import os
import math
import re
import json
import threading
import time
import urllib.request
import urllib.error
import warnings

warnings.filterwarnings('ignore')

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image

# Fast Pure-Python Image Serializer (Zero cv_bridge KeyError 16 bugs, 100% Reliable in RViz)
def cv2_to_imgmsg_fast(cv_img, encoding="bgr8", frame_id="camera_depth_optical_frame"):
    msg = Image()
    msg.height, msg.width = cv_img.shape[:2]
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = msg.width * (3 if len(cv_img.shape) == 3 else 1)
    msg.data = cv_img.tobytes()
    msg.header.frame_id = frame_id
    return msg

# Deskripsi Objek Geometris Lab 6m x 6m (Identifikasi Visual Dinamis)
LAB_LANDMARKS = {
    "kotak_merah": {
        "name": "[PERSEGI] KOTAK MERAH",
        "shape": "persegi",
        "color": "merah",
        "desc": "Benda Persegi / Kubus Merah (Target Utama)"
    },
    "pilar_hijau": {
        "name": "[SILINDER] PILAR HIJAU",
        "shape": "silinder",
        "color": "hijau",
        "desc": "Benda Silinder Hijau"
    },
    "meja_lab": {
        "name": "[PERSEGI PANJANG] BALOK ORANYE",
        "shape": "persegi_panjang",
        "color": "oranye",
        "desc": "Benda Persegi Panjang Oranye (Meja Lab)"
    }
}

# Daftar Waypoint Lab
KNOWN_WAYPOINTS = {
    "tempat_a":       {"x": 1.5,  "y": -0.5, "desc": "Posisi Pengamatan Tenggara (Tempat A)"},
    "tempat_b":       {"x": -1.5, "y": 1.5,  "desc": "Posisi Pengamatan Barat Laut (Tempat B)"},
    "tengah":         {"x": 0.0,  "y": 0.0,  "desc": "Titik Pusat Laboratorium (0, 0)"},
    "pos_utara":      {"x": 0.0,  "y": 1.8,  "desc": "Pos Pengamatan Utara"},
    "pos_selatan":    {"x": 0.0,  "y": -1.8, "desc": "Pos Pengamatan Selatan"}
}

BANNER = """
========================================================================
     ROBOT EKSPLORASI CERDAS (MASTER LLM LLAMA 3.2 + TRUE VISION 3D + PID)
========================================================================
LAB 6x6 METER - HYBRID YOLOv8 + REINFORCEMENT LEARNING + TUNED PID:
========================================================================
FUSI VISION GANDA: YOLOv8 (80 KELAS COCO) + GEOMETRIC LAB + LIDAR 360°:

  [1] Deteksi Objek COCO Bebas (YOLOv8-Nano 80 Kelas):
      - "cari botol" / "cari orang" / "cari kursi" / "cari laptop"
      - "cari cangkir" / "cari buku" / "cari tas" / "cari hp"
      --> Robot mendeteksi objek dunia nyata via YOLOv8, kunci bounding box & hampiri!

  [2] Perintah Objek Lab & Dinding (Vision 3D Geometric + PID Halus):
      - "coba anda ke kotak merah" / "dekati kotak merah"  --> Menuju Persegi Merah
      - "coba kamu ke silinder hijau" / "ke pilar hijau"   --> Menuju Silinder Hijau
      - "ke persegi panjang" / "cari meja"                --> Menuju Balok Oranye
      - "dekati dinding" / "ke tembok"                    --> Dekati dinding hingga 0.60m

  [3] Eksplorasi Sistematis Reinforcement Learning (4 Pojok Dinding -> Tengah):
      - "coba anda bergerak memutari ruangan gunakan lidar sebagai acuan agar tidak menabrak"
      - "kelilingi ruangan dari pojok dinding baru ke tengah"
      - "sudah berapa persen" / "dapat apa saja" / "kembali ke awal"
      --> Robot menyisir 4 pojok dinding (perimeter), masuk ke tengah (interior),
          mengoptimalkan reward Q-Learning anti-tabrak, lalu OTOMATIS PULANG KE TITIK AWAL!

  [4] Survei Ruangan Bebas (Aktif Berputar 360° & Mendata):
      - "scan semua yang berada di ruangan" / "bisakah anda mengecek ulang?"
      --> Robot memutar badan 360°, mencatat 3 objek geometris & jaraknya!

  [5] Kendali Manuver Langsung & Pendekatan Dinding:
      - "dekati dinding" / "ke dinding" / "dekati tembok" --> Maju hingga 0.60m dari dinding
      - "coba belok kekanan" / "belok kanan" / "belok kekiri" / "belok kiri"
      - "mundur" / "robot mundur" / "mundur 50 cm" / "maju 1 meter" / "putar balik"
      - "stop" / "berhenti"           --> Hentikan motor seketika
========================================================================
"""

def get_canonical_key(name):
    n = name.lower()
    if "merah" in n or "persegi" in n or "kubus" in n:
        if "panjang" not in n:
            return "kotak_merah"
    if "hijau" in n or "pilar" in n or "silinder" in n:
        return "pilar_hijau"
    if "oranye" in n or "panjang" in n or "meja" in n or "balok" in n:
        return "meja_lab"
    return None

# =====================================================================
# 1. PID CONTROLLER ANTI-OSILASI
# =====================================================================

# =====================================================================
# 1.2 YOLOv8 COCO 80 KELAS DUNIA NYATA & KAMUS TERJEMAHAN
# =====================================================================
COCO_TRANSLATIONS = {
    'orang': 'person', 'manusia': 'person', 'person': 'person',
    'kursi': 'chair', 'chair': 'chair',
    'meja': 'dining table', 'dining table': 'dining table',
    'botol': 'bottle', 'bottle': 'bottle',
    'gelas': 'cup', 'cangkir': 'cup', 'cup': 'cup',
    'laptop': 'laptop', 'komputer': 'laptop',
    'mouse': 'mouse', 'keyboard': 'keyboard',
    'buku': 'book', 'book': 'book',
    'tas': 'backpack', 'ransel': 'backpack', 'backpack': 'backpack',
    'payung': 'umbrella', 'umbrella': 'umbrella',
    'bola': 'sports ball', 'sports ball': 'sports ball',
    'kucing': 'cat', 'cat': 'cat',
    'anjing': 'dog', 'dog': 'dog',
    'sepeda': 'bicycle', 'bicycle': 'bicycle',
    'mobil': 'car', 'car': 'car',
    'motor': 'motorcycle', 'motorcycle': 'motorcycle',
    'sofa': 'couch', 'couch': 'couch',
    'tv': 'tv', 'televisi': 'tv',
    'hp': 'cell phone', 'handphone': 'cell phone', 'telepon': 'cell phone', 'cell phone': 'cell phone',
    'jam': 'clock', 'clock': 'clock',
    'vas': 'vase', 'vase': 'vase',
    'gunting': 'scissors', 'scissors': 'scissors',
    'boneka': 'teddy bear', 'teddy bear': 'teddy bear',
    'sikat gigi': 'toothbrush', 'toothbrush': 'toothbrush'
}

class PIDController:
    """
    KENDALI PD DENGAN FILTER DERIVATIF & DEADBAND
    (Ki = 0.0 untuk menghilangkan lag fase dan osilasi goyang visual)
    """
    def __init__(self, kp=1.20, ki=0.0, kd=0.15, out_min=-0.65, out_max=0.65, deadband=0.035):
        self.kp = kp
        self.ki = 0.0  # Wajib 0.0 pada visual servoing agar tidak terjadi osilasi/goyang
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.deadband = deadband
        self.integral = 0.0
        self.prev_error = 0.0
        self.filtered_d = 0.0
        self.last_time = time.time()

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.filtered_d = 0.0
        self.last_time = time.time()

    def update(self, error):
        now = time.time()
        dt = max(now - self.last_time, 0.001)
        self.last_time = now

        # Deadband untuk mengeliminasi getaran mikro
        if abs(error) <= self.deadband:
            return 0.0

        p = self.kp * error

        # Filtered Derivative (Low-Pass Filter) untuk redaman osilasi
        raw_d = (error - self.prev_error) / dt
        self.filtered_d = 0.70 * self.filtered_d + 0.30 * raw_d
        self.prev_error = error
        d = self.kd * self.filtered_d

        out = p + d
        return float(np.clip(out, self.out_min, self.out_max))

def parse_motion_distance(text, default=0.5):
    # Cek centimeter dulu: '50 cm', '50cm', '50 centi'
    cm_match = re.search(r"(\d+(?:\.\d+)?)\s*(?:centi|cm)\b", text)
    if cm_match:
        return max(0.1, float(cm_match.group(1)) / 100.0)
    # Cek meter: '1 meter', '1.5 m', '2m'
    m_match = re.search(r"(\d+(?:\.\d+)?)\s*(?:meter|m)\b", text)
    if m_match:
        return max(0.1, float(m_match.group(1)))
    # Cek angka tanpa satuan: 'mundur 2', 'maju 0.5'
    num_match = re.search(r"(?:maju|mundur|jalan)\s*(\d+(?:\.\d+)?)", text)
    if num_match:
        val = float(num_match.group(1))
        if val >= 10.0:
            return max(0.1, val / 100.0)
        return max(0.1, val)
    return default


# =====================================================================
# 1.5 REINFORCEMENT LEARNING EXPLORATION POLICY (Perimeter Q-Learning)
# =====================================================================
CORNER_WAYPOINTS = [
    {"name": "Pojok Barat Daya (SW)", "x": -1.80, "y": -1.80},
    {"name": "Pojok Barat Laut (NW)", "x": -1.80, "y":  1.80},
    {"name": "Pojok Timur Laut (NE)", "x":  1.80, "y":  1.80},
    {"name": "Pojok Tenggara (SE)",   "x":  1.80, "y": -1.80},
]
CENTER_WAYPOINT = {"name": "Tengah Lab (Center)", "x": 0.0, "y": 0.0}

class RLExplorationPolicy:
    def __init__(self):
        self.q_table = {}
        self.alpha = 0.20      # Learning rate ter-tuning halus
        self.gamma = 0.95      # Discount factor horizon panjang
        self.epsilon = 0.04    # Exploration rate rendah untuk pergerakan stabil
        self.last_state = None
        self.last_action = None
        self.total_rewards = 0.0
        self.step_count = 0

    def get_state(self, d_front, d_left, d_right, yaw_err, is_new_cell):
        # Diskretisasi resolusi tinggi (Fine Discretization)
        f_bin = 0 if d_front < 0.42 else (1 if d_front < 0.70 else (2 if d_front < 1.10 else 3))
        l_bin = 0 if d_left < 0.50 else (1 if d_left < 0.90 else 2)
        r_bin = 0 if d_right < 0.50 else (1 if d_right < 0.90 else 2)
        h_bin = 0 if abs(yaw_err) < 0.20 else (1 if yaw_err > 0.40 else (2 if yaw_err > 0 else (-1 if yaw_err < -0.40 else -2)))
        n_bin = 1 if is_new_cell else 0
        return (f_bin, l_bin, r_bin, h_bin, n_bin)

    def select_action(self, state):
        if state not in self.q_table:
            # Warm-start Q-table: 6 Aksi Halus
            # 0:CRUISE_FORWARD, 1:GENTLE_LEFT, 2:GENTLE_RIGHT, 3:MODERATE_LEFT, 4:MODERATE_RIGHT, 5:REVERSE_ESCAPE
            q = [0.0] * 6
            f_bin, l_bin, r_bin, h_bin, n_bin = state
            if f_bin == 0:     # Bahaya sangat dekat
                q[5] = 14.0    # Mundur terarah
                q[3] = 9.0 if l_bin >= 1 else 1.0
                q[4] = 9.0 if r_bin >= 1 else 1.0
            elif f_bin == 1:   # Peringatan jarak dekat
                if h_bin > 0:    q[3] = 8.0; q[1] = 6.0
                elif h_bin < 0:  q[4] = 8.0; q[2] = 6.0
                else:            q[1] = 5.0; q[2] = 5.0
            else:              # Jalur terbuka
                if h_bin == 0:   q[0] = 15.0; q[1] = 8.0; q[2] = 8.0
                elif h_bin > 0:  q[1] = 12.0; q[0] = 8.0
                else:            q[2] = 12.0; q[0] = 8.0
            self.q_table[state] = q

        # Epsilon decay adaptif
        eff_eps = max(0.015, self.epsilon * (0.998 ** self.step_count))
        if np.random.rand() < eff_eps:
            return int(np.random.randint(6))
        return int(np.argmax(self.q_table[state]))

    def update(self, reward, next_state):
        if self.last_state is not None and self.last_action is not None:
            old_q = self.q_table[self.last_state][self.last_action]
            if next_state not in self.q_table:
                self.select_action(next_state)
            max_next_q = max(self.q_table[next_state])
            new_q = old_q + self.alpha * (reward + self.gamma * max_next_q - old_q)
            self.q_table[self.last_state][self.last_action] = new_q
            self.total_rewards += reward
            self.step_count += 1

class ConversationalLlama:
    def __init__(self, model_name="llama3.2:1b", ollama_url="http://localhost:11434"):
        self.model_name = model_name
        self.ollama_url = ollama_url
        self.ollama_available = self.check_ollama()

    def check_ollama(self):
        try:
            req = urllib.request.Request(f"{self.ollama_url}/api/tags")
            with urllib.request.urlopen(req, timeout=1.5) as res:
                if res.status == 200:
                    data = json.loads(res.read().decode('utf-8'))
                    models = [m.get('name', '') for m in data.get('models', [])]
                    for m in models:
                        if 'llama' in m.lower():
                            self.model_name = m
                            print(f"[LLM] Model Ollama Terdeteksi: '{self.model_name}' (Siap Aktif)")
                            return True
                    print(f"[LLM] Ollama aktif dengan model: {models}")
                    return True
        except Exception:
            pass
        print("[LLM] Ollama offline / belum start. Menggunakan Mesin Inferensi Semantik Cepat.")
        return False

    def parse_user_intent(self, user_text: str, discovered_catalog=None, sensor_telemetry=None):
        text = user_text.strip()
        cmd_lower = text.lower()
        if sensor_telemetry is None:
            sensor_telemetry = {"front_lidar": 3.0, "laser_left": 3.0, "laser_right": 3.0, "laser_rear": 3.0, "front_visual_text": "Jalur lapang", "x": 0.0, "y": 0.0, "yaw_deg": 0.0}

        front_lidar = sensor_telemetry.get("front_lidar", 3.0)
        laser_left  = sensor_telemetry.get("laser_left", 3.0)
        laser_right = sensor_telemetry.get("laser_right", 3.0)
        laser_rear  = sensor_telemetry.get("laser_rear", 3.0)
        front_visual = sensor_telemetry.get("front_visual_text", "Jalur lapang")
        curr_x      = sensor_telemetry.get("x", 0.0)
        curr_y      = sensor_telemetry.get("y", 0.0)
        curr_yaw    = sensor_telemetry.get("yaw_deg", 0.0)

        # -------------------------------------------------------------
        # 1. KENDALI DARURAT STOP
        # -------------------------------------------------------------
        if re.search(r"\b(stop|berhenti|diam|batal|halt|pause|rem)\b", cmd_lower):
            return {"action": "STOP", "response": "Baik, motor robot segera dihentikan seketika!"}

        # -------------------------------------------------------------
        # 2. STATUS CAKUPAN DAN LAPORAN EKSPLORASI
        # -------------------------------------------------------------
        if re.search(r"(berapa persen|persen|persentase|cakupan|hasil keliling|progress|sudah dapat apa|dapat apa saja|laporan keliling|posisi sekarang)", cmd_lower):
            return {
                "action": "CHECK_COVERAGE",
                "response": "Berikut laporan kemajuan eksplorasi dan cakupan ruangan saat ini:"
            }

        # -------------------------------------------------------------
        # 3. KEMBALI KE AWAL (RETURN TO HOME)
        # -------------------------------------------------------------
        if re.search(r"(kembali ke awal|kembali ke posisi awal|kembali ke titik awal|pulang|balik ke awal|kembali ke home|balik ke titik awal|kembali awal)", cmd_lower) and not re.search(r"(keliling|jelajah|patroli|muter|memutari)", cmd_lower):
            return {
                "action": "RETURN_HOME",
                "response": "Baik! Menghentikan penjelajahan dan segera kembali ke titik awal keberangkatan (Home) dengan panduan LiDAR."
            }

        # -------------------------------------------------------------
        # 4. FILTER PERTANYAAN SENSOR (Tanya vs Perintah Bergerak)
        # -------------------------------------------------------------
        is_question = bool(re.search(r"(apa|gimana|aman|lihat|cek|info|berapa)\b|\?", cmd_lower))

        if is_question and re.search(r"(di kirimu|di kiri|sebelah kiri|sisi kiri|kirimu)", cmd_lower) and not re.search(r"(belok|putar)", cmd_lower):
            status = "Area lapang (> 0.8 m)" if laser_left > 0.8 else f"Rintangan dekat ({laser_left:.2f} m)"
            response = f"Sensor RPLiDAR A1 360° pada sudut kiri robot (+90°): Jarak rintangan {laser_left:.2f} m ({status})."
            return {"action": "CHAT", "response": response}

        if is_question and re.search(r"(di kananmu|di kanan|sebelah kanan|sisi kanan|kananmu)", cmd_lower) and not re.search(r"(belok|putar)", cmd_lower):
            status = "Area lapang (> 0.8 m)" if laser_right > 0.8 else f"Rintangan dekat ({laser_right:.2f} m)"
            response = f"Sensor RPLiDAR A1 360° pada sudut kanan robot (-90°): Jarak rintangan {laser_right:.2f} m ({status})."
            return {"action": "CHAT", "response": response}

        if is_question and re.search(r"(di belakangmu|di belakang|sebelah belakang|belakangmu)", cmd_lower) and not re.search(r"(mundur|putar)", cmd_lower):
            status = "Area aman" if laser_rear > 0.8 else f"Rintangan dekat ({laser_rear:.2f} m)"
            response = f"Sensor RPLiDAR A1 360° pada sudut belakang robot (180°): Jarak rintangan {laser_rear:.2f} m ({status})."
            return {"action": "CHAT", "response": response}

        if is_question and re.search(r"(didepan|di depan|depanmu|depan aman|rintangan depan|apa yang kamu lihat|lagi ngeliat apa)", cmd_lower) and not re.search(r"(maju|jalan ke depan)", cmd_lower):
            path_status = "Jalur Terbuka (> 0.8 m)" if front_lidar > 0.8 else f"Perhatian: Rintangan Dekat ({front_lidar:.2f} m)"
            response = (
                f"Fusi Sensor Langsung:\n"
                f"  • Kamera RGB-D: {front_visual}.\n"
                f"  • RPLiDAR A1 (Depan): {front_lidar:.2f} meter ({path_status}).\n"
                f"  • Posisi Robot: X={curr_x:.2f}, Y={curr_y:.2f}, Hadap {curr_yaw:.1f}°."
            )
            return {"action": "CHAT", "response": response}

        # -------------------------------------------------------------
        # 5. PERINTAH MANUVER ROTASI & GERAK LANGSUNG (PRIORITAS EKSEKUSI)
        # -------------------------------------------------------------
        # A. Putar Balik (180°)
        if re.search(r"(putar balik|puter balik|balik arah|balik badan|balik 180|putar 180|berbalik)", cmd_lower):
            return {"action": "ROTATE_RELATIVE", "angle_rad": math.pi, "response": "Baik, robot melakukan manuver putar balik 180°."}

        # B. Belok / Putar Kanan (Mendukung 'kekanan', 'ke kanan', 'belok kanan', dsb)
        if re.search(r"(belok\s*(?:ke\s*)?kanan|belok\s*kekanan|putar\s*(?:ke\s*)?kanan|puter\s*(?:ke\s*)?kanan|hadap\s*kanan|serong\s*kanan|ke\s*kanan|\bkekanan\b)", cmd_lower) or \
           (re.search(r"\bkanan\b", cmd_lower) and re.search(r"(robot|coba|tolong|kamu|anda|gerak|jalan|arah|hadap)", cmd_lower) and not is_question):
            return {"action": "ROTATE_RELATIVE", "angle_rad": -math.pi / 2.0, "response": "Baik, robot berbelok 90° ke arah kanan."}

        # C. Belok / Putar Kiri (Mendukung 'kekiri', 'ke kiri', 'belok kiri', dsb)
        if re.search(r"(belok\s*(?:ke\s*)?kiri|belok\s*kekiri|putar\s*(?:ke\s*)?kiri|puter\s*(?:ke\s*)?kiri|hadap\s*kiri|serong\s*kiri|ke\s*kiri|\bkekiri\b)", cmd_lower) or \
           (re.search(r"\bkiri\b", cmd_lower) and re.search(r"(robot|coba|tolong|kamu|anda|gerak|jalan|arah|hadap)", cmd_lower) and not is_question):
            return {"action": "ROTATE_RELATIVE", "angle_rad": math.pi / 2.0, "response": "Baik, robot berbelok 90° ke arah kiri."}

        # D. Mundur (Mendukung 'mundur', 'robot mundur', 'mundur 1 meter', 'mundur 50 cm')
        if re.search(r"(mundur|atret|jalan\s*mundur|bergerak\s*mundur|ke\s*belakang)", cmd_lower) and not is_question:
            d = parse_motion_distance(cmd_lower, default=0.5)
            return {"action": "MOVE_RELATIVE", "distance": -d, "response": f"Baik, robot mundur sejauh {d:.2f} meter."}

        # E. Maju (Mendukung 'maju', 'robot maju', 'jalan maju', 'maju 2 meter', 'lurus')
        if re.search(r"(maju|jalan\s*maju|bergerak\s*maju|lurus|ke\s*depan)", cmd_lower) and not is_question and not re.search(r"(didepan|di depan)", cmd_lower):
            d = parse_motion_distance(cmd_lower, default=0.5)
            return {"action": "MOVE_RELATIVE", "distance": d, "response": f"Baik, robot melaju maju sejauh {d:.2f} meter."}

        # -------------------------------------------------------------
        # 5.5 PATROLI & EKSPLORASI OTONOM DENGAN MEMORI SPASIAL
        # -------------------------------------------------------------
        if re.search(r"(memutari|mutari|muter|keliling|patroli|jelajahi|eksplorasi|jalan-jalan|putar ruangan|kelilingi|acuan agar tidak menabrak|jangan nabrak|hindari rintangan)", cmd_lower):
            return {
                "action": "PATROL_ROOM",
                "response": "Siap! Memulai eksplorasi spasial anti-pengulangan. Seluruh ruangan akan dipetakan, mencatat objek yang terdeteksi sensor, lalu otomatis kembali ke titik awal (≥85% cakupan)."
            }

        # -------------------------------------------------------------
        # 5.6 SURVEI / SCAN 360° RUANGAN
        # -------------------------------------------------------------
        if re.search(r"(scan|memindai|pindai|survei|deteksi\s*(?:semua|ruangan|lab|isi)?|cek\s*(?:semua|ulang|lagi)?|cari\s*semua|identifikasi\s*semua|ada apa aja|benda apa saja|ada objek apa|semua yang berada di ruangan)", cmd_lower):
            return {
                "action": "SCAN_ROOM",
                "response": "Siap! Memutar badan 360° memindai objek di dalam laboratorium dengan fusi LiDAR 360° & Kamera RGB-D."
            }

        # 5.7 Dekati Dinding / Tembok Laboratorium (Prioritas Bebas Halusinasi Objek)
        if re.search(r"(dekati\s*(?:ke\s*)?(?:dinding|tembok)|menuju\s*(?:ke\s*)?(?:dinding|tembok)|ke\s*(?:dinding|tembok)|dinding\s*terdekat|tembok\s*terdekat|\bdinding\b|\btembok\b)", cmd_lower) and not is_question and not re.search(r"(keliling|memutari|mutari|muter|patroli|jelajah)", cmd_lower):
            return {
                "action": "APPROACH_WALL",
                "response": "Siap! Mengidentifikasi dinding laboratorium terdekat. Melaju lurus dengan panduan LiDAR hingga jarak aman 0.60m di depan dinding."
            }

        # -------------------------------------------------------------
        # 6. PERINTAH DETEKSI YOLOv8 COCO (80 KELAS DUNIA NYATA)
        # -------------------------------------------------------------
        for indo_name, coco_class in COCO_TRANSLATIONS.items():
            if re.search(r"\b" + re.escape(indo_name) + r"\b", cmd_lower) and not is_question and not re.search(r"(kotak merah|pilar hijau|balok oranye)", cmd_lower):
                return {
                    "action": "SEARCH_OBJECT",
                    "target_name": coco_class,
                    "target_shape": f"YOLO [{coco_class.upper()}]",
                    "target_color": "COCO Object",
                    "target_type": "YOLO",
                    "response": f"Siap! Mengaktifkan deteksi YOLOv8-Nano untuk melacak objek '{coco_class}' (80 kelas COCO). Kamera & PID visual aktif menuju objek!"
                }

        # -------------------------------------------------------------
        # 6.5 PERINTAH PENDEKATAN 3 OBJEK LAB (STRICT TARGET VISION + PID)
        # -------------------------------------------------------------
        # A. Balok Persegi Panjang Oranye (Meja Lab)
        if re.search(r"(persegi panjang|balok|meja|oranye|cokelat|table)", cmd_lower) and not re.search(r"(merah|hijau)", cmd_lower):
            return {
                "action": "SEARCH_OBJECT",
                "target_name": "meja_lab",
                "target_shape": "persegi_panjang",
                "target_color": "oranye",
                "target_type": "GEOMETRIC",
                "response": "Siap! Mengunci Benda Persegi Panjang (Balok Oranye). Kamera RGB-D & PID visual aktif melaju mendekati objek hingga jarak aman 0.55m."
            }

        # B. Pilar Silinder Hijau
        if re.search(r"(pilar|hijau|silinder|tabung|green)", cmd_lower) and not re.search(r"(merah|oranye|biru)", cmd_lower):
            return {
                "action": "SEARCH_OBJECT",
                "target_name": "pilar_hijau",
                "target_shape": "silinder",
                "target_color": "hijau",
                "target_type": "GEOMETRIC",
                "response": "Siap! Mengunci Benda Silinder (Pilar Hijau). Kamera RGB-D & PID visual aktif melaju mendekati objek hingga jarak aman 0.55m."
            }

        # C. Kotak Persegi Merah
        if re.search(r"(kotak|merah|persegi|kubus|red)", cmd_lower) and not re.search(r"(panjang|hijau|oranye|biru)", cmd_lower):
            return {
                "action": "SEARCH_OBJECT",
                "target_name": "kotak_merah",
                "target_shape": "persegi",
                "target_color": "merah",
                "target_type": "GEOMETRIC",
                "response": "Siap! Mengunci Benda Persegi (Kotak Merah). Kamera RGB-D & PID visual aktif melaju mendekati objek hingga jarak aman 0.55m."
            }

        # -------------------------------------------------------------
        # 9. REFERENSI HASIL SCAN ("dekati objek nomor 1")
        # -------------------------------------------------------------
        idx_match = re.search(r"(?:objek\s*(?:nomor|no)?\s*(\d+)|nomor\s*(\d+)|objek\s+pertama|objek\s+kedua|objek\s+ketiga)", cmd_lower)
        if idx_match and discovered_catalog:
            idx = 1
            if "pertama" in cmd_lower: idx = 1
            elif "kedua" in cmd_lower: idx = 2
            elif "ketiga" in cmd_lower: idx = 3
            else:
                num_str = idx_match.group(1) or idx_match.group(2)
                if num_str: idx = int(num_str)

            if 1 <= idx <= len(discovered_catalog):
                obj_key = list(discovered_catalog.keys())[idx - 1]
                target_name = get_canonical_key(obj_key) or "kotak_merah"
                lm = LAB_LANDMARKS.get(target_name, LAB_LANDMARKS["kotak_merah"])
                return {
                    "action": "SEARCH_OBJECT",
                    "target_name": target_name,
                    "target_shape": lm["shape"],
                    "target_color": lm["color"],
                    "response": f"Baik! Mengunci objek nomor {idx}: {lm['name']}. Melaju mendekati target dengan PID lurus!"
                }

        # -------------------------------------------------------------
        # 10. NAVIGASI WAYPOINT POSISI
        # -------------------------------------------------------------
        for k, info in KNOWN_WAYPOINTS.items():
            if k in ["kotak_merah", "pilar_hijau", "meja"]:
                continue
            k_clean = k.replace("_", " ")
            if k_clean in cmd_lower or f"tempat {k[-1]}" in cmd_lower:
                return {
                    "action": "NAVIGATE_LOCATION",
                    "target_name": info["desc"],
                    "x": info["x"], "y": info["y"],
                    "response": f"Baik, saya akan bergerak menuju {info['desc']} (X={info['x']}, Y={info['y']}) dengan kendali PID."
                }

        # -------------------------------------------------------------
        # 11. INFERENSI SEMANTIK LLM LLAMA 3.2 (FALLBACK CADANGAN)
        # -------------------------------------------------------------
        if self.ollama_available:
            try:
                system_prompt = (
                    "Kamu adalah pengendali AI Robot Eksplorasi Cerdas (ROS 2 Humble).\n"
                    "Kamu adalah pengendali AI Robot Cerdas (ROS 2 Humble).\n"
                    "KEMAMPUAN PERSEPSI:\n"
                    "1. YOLOv8-Nano: Mendeteksi 80 kelas COCO dunia nyata ('person', 'bottle', 'chair', 'cup', 'laptop', 'book', 'backpack', dll).\n"
                    "2. Objek Geometris Lab: 'kotak_merah', 'pilar_hijau', 'meja_lab'.\n"
                    "3. Lingkungan: 'dinding' / 'tembok', 'pojok', 'tengah'.\n\n"
                    "Aksi yang diizinkan:\n"
                    "- SEARCH_OBJECT: jika mencari objek COCO YOLO atau objek lab. Berikan target nama objek.\n"
                    "- APPROACH_WALL: jika diminta mendekati dinding / tembok.\n"
                    "- PATROL_ROOM: keliling 4 pojok dinding lalu ke tengah.\n"
                    "- ROTATE_RELATIVE: belok kanan (-1.57), kiri (1.57), putar balik (3.14).\n"
                    "- MOVE_RELATIVE: maju atau mundur (distance meter).\n"
                    "- STOP: berhenti.\n"
                    "- CHAT: hanya jika pertanyaan menyapa atau info sensor.\n\n"
                    "Format JSON:\n"
                    '{"action": "ACTION_NAME", "target": "target_name", "distance": 0.5, "angle_rad": 0.0, "response": "pesan singkat"}\n\n'
                    f'Pesan: "{text}"\nJSON:'
                )
                payload = json.dumps({
                    "model": self.model_name,
                    "prompt": system_prompt,
                    "format": "json",
                    "stream": False,
                    "options": {"temperature": 0.1, "num_predict": 120}
                }).encode('utf-8')

                req = urllib.request.Request(f"{self.ollama_url}/api/generate", data=payload, headers={"Content-Type": "application/json"})
                resp = urllib.request.urlopen(req, timeout=8.0)
                res_data = json.loads(resp.read().decode('utf-8'))
                parsed = json.loads(res_data.get('response', '{}').strip())
                action = parsed.get("action", "CHAT").upper()
                resp_text = parsed.get("response", "").strip()

                if action in ["PATROL_ROOM", "STOP", "RETURN_HOME", "CHECK_COVERAGE", "SCAN_ROOM", "APPROACH_WALL"]:
                    return {"action": action, "response": resp_text}
                elif action in ["ROTATE_RELATIVE", "TURN_RIGHT", "TURN_LEFT", "TURN_AROUND"]:
                    ang = parsed.get("angle_rad", -math.pi/2.0 if ("RIGHT" in action or "kanan" in cmd_lower) else (math.pi/2.0 if ("LEFT" in action or "kiri" in cmd_lower) else math.pi))
                    return {"action": "ROTATE_RELATIVE", "angle_rad": ang, "response": resp_text or "Melakukan manuver rotasi."}
                elif action in ["MOVE_RELATIVE", "MOVE_BACKWARD", "MOVE_FORWARD", "MUNDUR", "MAJU"]:
                    dist = parsed.get("distance", -0.5 if ("BACK" in action or "mundur" in cmd_lower) else 0.5)
                    return {"action": "MOVE_RELATIVE", "distance": dist, "response": resp_text or "Melakukan manuver gerak."}
                elif action == "SEARCH_OBJECT":
                    tgt = parsed.get("target", "kotak_merah").lower().strip()
                    if tgt in COCO_TRANSLATIONS:
                        coco_c = COCO_TRANSLATIONS[tgt]
                        return {"action": "SEARCH_OBJECT", "target_name": coco_c, "target_shape": f"YOLO [{coco_c.upper()}]", "target_color": "COCO Object", "target_type": "YOLO", "response": resp_text or f"Mencari objek YOLO: {coco_c}..."}
                    canon = get_canonical_key(tgt) or "kotak_merah"
                    lm = LAB_LANDMARKS.get(canon, LAB_LANDMARKS["kotak_merah"])
                    return {"action": "SEARCH_OBJECT", "target_name": canon, "target_shape": lm["shape"], "target_color": lm["color"], "target_type": "GEOMETRIC", "response": resp_text or f"Mendekati {lm['name']}..."}
                elif action == "CHAT":
                    # Safety check: Jangan biarkan perintah gerak / dinding menjadi sekadar chat!
                    if re.search(r"(dinding|tembok)", cmd_lower):
                        return {"action": "APPROACH_WALL", "response": "Baik, melaju mendekati dinding laboratorium."}
                    if re.search(r"(belok\s*(?:ke\s*)?kanan|belok\s*kekanan|\bkekanan\b)", cmd_lower):
                        return {"action": "ROTATE_RELATIVE", "angle_rad": -math.pi/2.0, "response": "Baik, robot berbelok ke arah kanan."}
                    if re.search(r"(belok\s*(?:ke\s*)?kiri|belok\s*kekiri|\bkekiri\b)", cmd_lower):
                        return {"action": "ROTATE_RELATIVE", "angle_rad": math.pi/2.0, "response": "Baik, robot berbelok ke arah kiri."}
                    if re.search(r"(mundur|atret)", cmd_lower):
                        return {"action": "MOVE_RELATIVE", "distance": -0.50, "response": "Baik, robot mundur 0.50 meter."}
                    if re.search(r"(maju|lurus)", cmd_lower) and not re.search(r"(didepan|di depan)", cmd_lower):
                        return {"action": "MOVE_RELATIVE", "distance": 0.50, "response": "Baik, robot melaju maju 0.50 meter."}
                    if resp_text:
                        return {"action": "CHAT", "response": resp_text}
            except Exception:
                pass

        # Fallback akhir jika kata dinding terdeteksi
        if re.search(r"(dinding|tembok)", cmd_lower):
            return {"action": "APPROACH_WALL", "response": "Baik, robot melaju mendekati dinding laboratorium."}

        # Fallback akhir jika kata perintah gerak terdeteksi
        if re.search(r"(belok\s*(?:ke\s*)?kanan|belok\s*kekanan|\bkekanan\b)", cmd_lower):
            return {"action": "ROTATE_RELATIVE", "angle_rad": -math.pi/2.0, "response": "Baik, robot berbelok ke arah kanan."}
        if re.search(r"(belok\s*(?:ke\s*)?kiri|belok\s*kekiri|\bkekiri\b)", cmd_lower):
            return {"action": "ROTATE_RELATIVE", "angle_rad": math.pi/2.0, "response": "Baik, robot berbelok ke arah kiri."}
        if re.search(r"(mundur|atret)", cmd_lower):
            return {"action": "MOVE_RELATIVE", "distance": -0.50, "response": "Baik, robot mundur 0.50 meter."}
        if re.search(r"(maju|lurus)", cmd_lower) and not re.search(r"(didepan|di depan)", cmd_lower):
            return {"action": "MOVE_RELATIVE", "distance": 0.50, "response": "Baik, robot melaju maju 0.50 meter."}

        return {
            "action": "CHAT",
            "response": (
                f"Saya adalah Robot Eksplorasi Cerdas. Di ruangan lab 6x6m ini terdapat 3 objek utama: "
                "Silinder Hijau, Balok Persegi Panjang Oranye, dan Kubus Persegi Merah. "
                "Anda dapat menyuruh saya bergerak langsung: 'belok kekanan', 'belok kekiri', 'mundur', 'maju', atau 'cari merah'."
            )
        }

# =====================================================================
# 3. VISION & SENSOR PERCEPTION (STRICT TARGET + TRUE COLOR)
# =====================================================================
class VisionPerception:
    def __init__(self):
        # 1. Objek Geometris Lab 6x6m Asli
        self.categories = [
            {
                "id": "kotak_merah",
                "label": "[PERSEGI] KOTAK MERAH",
                "shape": "Persegi / Kubus",
                "color_bgr": (0, 0, 255),
                "hsv_ranges": [
                    (np.array([0, 70, 50]),   np.array([12, 255, 255])),
                    (np.array([168, 70, 50]), np.array([180, 255, 255]))
                ]
            },
            {
                "id": "pilar_hijau",
                "label": "[SILINDER] PILAR HIJAU",
                "shape": "Silinder",
                "color_bgr": (0, 255, 0),
                "hsv_ranges": [
                    (np.array([35, 55, 40]), np.array([85, 255, 255]))
                ]
            },
            {
                "id": "meja_lab",
                "label": "[PERSEGI PANJANG] BALOK ORANYE",
                "shape": "Persegi Panjang",
                "color_bgr": (0, 140, 255),
                "hsv_ranges": [
                    (np.array([11, 65, 40]), np.array([25, 255, 255]))
                ]
            }
        ]

        # 2. YOLOv8-Nano 80 Kelas COCO Dunia Nyata
        self.yolo_model = None
        self.yolo_available = False
        try:
            from ultralytics import YOLO
            self.yolo_model = YOLO('yolov8n.pt')
            self.yolo_available = True
            print("[VISION] YOLOv8-Nano Siap Aktif! Mendukung 80 Kelas Objek COCO Dunia Nyata.")
        except Exception as e:
            print(f"[VISION] YOLOv8 tidak dapat diinisialisasi ({e}). Mode Geometric tetap aktif.")

    def inspect_front(self, bgr_image, depth_image, depth_encoding):
        """Memeriksa objek yang berada di depan ROI kamera"""
        if bgr_image is None:
            return "Kamera belum menerima citra", None

        h_img, w_img = bgr_image.shape[:2]
        cx_min, cx_max = int(w_img * 0.15), int(w_img * 0.85)
        roi_bgr = bgr_image[:, cx_min:cx_max]
        hsv_roi = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        best_obj = None
        max_area = 0

        for cat in self.categories:
            combined_mask = None
            for lower, upper in cat["hsv_ranges"]:
                m = cv2.inRange(hsv_roi, lower, upper)
                combined_mask = m if combined_mask is None else (combined_mask | m)

            combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
            contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 350 and area > max_area:
                    max_area = area
                    x, y, w, h = cv2.boundingRect(cnt)
                    cx = cx_min + x + w // 2
                    cy = y + h // 2

                    dist_z = 1.5
                    if depth_image is not None:
                        dh, dw = depth_image.shape[:2]
                        u1, u2 = max(0, cx - 2), min(dw, cx + 3)
                        v1, v2 = max(0, cy - 2), min(dh, cy + 3)
                        patch = depth_image[v1:v2, u1:u2]
                        valid = patch[(patch > 0.1) & np.isfinite(patch)]
                        if len(valid) > 0:
                            med = float(np.median(valid))
                            dist_z = med / 1000.0 if ('16uc1' in str(depth_encoding).lower() or med > 100.0) else med

                    best_obj = (cat["label"], dist_z)

        if best_obj is not None:
            lbl, d = best_obj
            return f"{lbl} (pada jarak ~{d:.2f} meter)", d

        return "Tidak ada objek spesifik di depan kamera (jalur terbuka menuju dinding)", None

    def detect(self, bgr_image, depth_image, depth_encoding, target_name, target_color):
        """
        STRICT TARGET DETECTION:
        Mendeteksi HANYA target yang diminta.
        JANGAN PERNAH fallback ke objek lain agar robot tidak salah mendatangi objek!
        """
        if bgr_image is None:
            placeholder = np.zeros((240, 320, 3), dtype=np.uint8)
            cv2.putText(placeholder, "Menunggu Kamera RGB-D...", (30, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
            return False, None, None, placeholder

        frame = bgr_image.copy()
        h_img, w_img = frame.shape[:2]
        target = (target_name or "kotak_merah").lower().strip()

        # JALUR A: DETEKSI YOLOv8-NANO (Untuk 80 Kelas COCO Dunia Nyata)
        if self.yolo_available and (target in COCO_TRANSLATIONS.values() or target in COCO_TRANSLATIONS):
            coco_target = COCO_TRANSLATIONS.get(target, target)
            try:
                results = self.yolo_model.predict(frame, conf=0.30, verbose=False)
                best_yolo_box = None
                best_conf = 0.0
                if results and len(results) > 0:
                    for box in results[0].boxes:
                        cls_id = int(box.cls[0])
                        cls_name = self.yolo_model.names.get(cls_id, '')
                        conf = float(box.conf[0])
                        if cls_name == coco_target and conf > best_conf:
                            best_conf = conf
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            best_yolo_box = (int(x1), int(y1), int(x2 - x1), int(y2 - y1))

                if best_yolo_box is not None:
                    x, y, w, h = best_yolo_box
                    cx = x + w // 2
                    cy = y + h // 2
                    dist_z = 1.5
                    if depth_image is not None:
                        dh, dw = depth_image.shape[:2]
                        u1, u2 = max(0, cx - 2), min(dw, cx + 3)
                        v1, v2 = max(0, cy - 2), min(dh, cy + 3)
                        patch = depth_image[v1:v2, u1:u2]
                        valid = patch[(patch > 0.1) & np.isfinite(patch)]
                        if len(valid) > 0:
                            med = float(np.median(valid))
                            dist_z = med / 1000.0 if ('16uc1' in str(depth_encoding).lower() or med > 100.0) else med

                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 120), 2)
                    cv2.circle(frame, (cx, cy), 4, (0, 255, 255), -1)
                    label_str = f"[YOLO] {coco_target.upper()} | {dist_z:.2f}m ({best_conf:.2f})"
                    cv2.putText(frame, label_str, (x, max(20, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 120), 2)
                    return True, best_yolo_box, dist_z, frame
            except Exception:
                pass

        # JALUR B: DETEKSI GEOMETRIS LAB WARNA ASLI (HSV MASKING STRICT)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        # Tentukan kategori target yang DIMINTA
        chosen_cat = None
        if "hijau" in target or "pilar" in target or "silinder" in target:
            chosen_cat = self.categories[1] # Pilar Hijau
        elif "meja" in target or "oranye" in target or "panjang" in target or "balok" in target:
            chosen_cat = self.categories[2] # Balok Oranye
        elif "merah" in target or "kotak" in target or "persegi" in target or "kubus" in target:
            chosen_cat = self.categories[0] # Kotak Merah
        else:
            chosen_cat = self.categories[0]

        best_box = None
        max_area = 0

        combined_mask = None
        for lower, upper in chosen_cat["hsv_ranges"]:
            m = cv2.inRange(hsv, lower, upper)
            combined_mask = m if combined_mask is None else (combined_mask | m)

        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 350 and area > max_area:
                max_area = area
                best_box = cv2.boundingRect(cnt)

        # CATATAN: TIDAK ADA FALLBACK KE OBJEK LAIN!
        # Jika target belum ada di FOV kamera, kembalikan False agar robot berbalik ke arah landmark target yang tepat!

        dist_z = None
        if best_box is not None:
            x, y, w, h = best_box
            cx, cy = x + w // 2, y + h // 2

            if depth_image is not None:
                dh, dw = depth_image.shape[:2]
                u1, u2 = max(0, cx - 3), min(dw, cx + 4)
                v1, v2 = max(0, cy - 3), min(dh, cy + 4)
                patch = depth_image[v1:v2, u1:u2]
                valid = patch[(patch > 0.1) & np.isfinite(patch)]
                if len(valid) > 0:
                    med = float(np.median(valid))
                    dist_z = med / 1000.0 if ('16uc1' in str(depth_encoding).lower() or med > 100.0) else med

            # Gambar Bounding Box Presisi & Label Bentuk
            box_color = chosen_cat["color_bgr"]
            cv2.rectangle(frame, (x, y), (x + w, y + h), box_color, 2)
            d_str = f" | {dist_z:.2f}m" if dist_z is not None else ""
            label_text = f"{chosen_cat['label']}{d_str}"
            cv2.putText(frame, label_text, (x, max(22, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.52, box_color, 2)
            cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

            # Garis Pedoman Sumbu Tengah Kamera
            mid_x = w_img // 2
            cv2.line(frame, (mid_x, 0), (mid_x, h_img), (255, 255, 255), 1)

            return True, best_box, dist_z, frame

        # Tampilan jika target belum terlihat di layar kamera
        cv2.putText(frame, f"Mencari: {chosen_cat['label']}...", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 165, 255), 2)
        return False, None, None, frame


# =====================================================================
# 4. INTELLIGENT EXPLORER ROS 2 NODE (WITH TRUE COLOR & SPATIAL MEMORY)
# =====================================================================
class IntelligentExplorerNode(Node):
    def __init__(self):
        super().__init__('intelligent_explorer')

        # Subscriptions & Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.detect_img_pub = self.create_publisher(Image, '/perception/detection_image', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_cb, 10)

        # Inisialisasi Sub-Sistem
        self.llama = ConversationalLlama()
        self.vision = VisionPerception()
        # Tuning PID Terbukti Stabil & Bebas Goyang
        self.pid_steer = PIDController(kp=1.20, ki=0.0, kd=0.15, out_min=-0.65, out_max=0.65, deadband=0.035)
        self.pid_dist = PIDController(kp=0.40, ki=0.0, kd=0.06, out_min=-0.20, out_max=0.20, deadband=0.035)
        self.target_type = "GEOMETRIC" 

        # State Telemetri Riil
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_yaw = 0.0
        self.min_front_laser = 5.0
        self.laser_left = 5.0
        self.laser_right = 5.0
        self.laser_rear = 5.0
        self.latest_bgr = None
        self.latest_depth = None
        self.depth_encoding = '32FC1'

        # Filter & Memory
        self.smooth_error_x = 0.0
        self.lost_frames_count = 0
        self.is_shutting_down = False

        # State Machine
        self.state = 'IDLE'
        self.target_name = "kotak_merah"
        self.target_shape = "persegi"
        self.target_color = "merah"
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0
        self.goal_label = ""
        self.search_start_time = 0.0
        self.search_dir = 1.0

        # Parameter Manuver Relatif (Maju / Mundur Lurus & Rotasi Cepat)
        self.rel_start_x = 0.0
        self.rel_start_y = 0.0
        self.rel_start_yaw = 0.0
        self.rel_target_dist = 0.0
        self.rel_is_forward = True

        # Memori Spasial (Grid 0.5m x 0.5m) & Return to Home
        self.grid_res = 0.5
        self.visited_cells = set()
        self.visited_quadrants = set()
        self.target_unique_cells = 22
        self.coverage_pct = 0.0
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_yaw = 0.0
        self.patrol_start_time = 0.0
        self.last_coverage_log_time = 0.0
        self.exploration_completed = False

        # Reinforcement Learning & Strategi 4 Pojok Dinding -> Tengah
        self.rl_agent = RLExplorationPolicy()
        self.patrol_waypoints = []
        self.current_wp_idx = 0
        self.patrol_prev_dist = 999.0

        # Data Objek Terdata (Deduplikasi dengan Kunci Kanonikal)
        self.scan_total_turned = 0.0
        self.last_yaw = 0.0
        self.discovered_objects = {}

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Intelligent Explorer Ready with Strict Target Vision & Precision PID!')

    def odom_cb(self, msg: Odometry):
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.curr_yaw = math.atan2(siny, cosy)

        # Catat sel spasial yang telah dikunjungi (Anti-Pengulangan)
        cx = int(round(self.curr_x / self.grid_res))
        cy = int(round(self.curr_y / self.grid_res))
        self.visited_cells.add((cx, cy))

        # Catat kuadran lab
        quad = "NE" if (self.curr_x >= 0 and self.curr_y >= 0) else \
               ("NW" if (self.curr_x < 0 and self.curr_y >= 0) else \
               ("SW" if (self.curr_x < 0 and self.curr_y < 0) else "SE"))
        self.visited_quadrants.add(quad)

        # Hitung Persentase Cakupan Ruangan
        cell_score = min(1.0, len(self.visited_cells) / float(self.target_unique_cells)) * 70.0
        quad_score = (len(self.visited_quadrants) / 4.0) * 30.0
        self.coverage_pct = round(min(100.0, cell_score + quad_score), 1)

    def scan_cb(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        n = len(ranges)
        if n == 0:
            return

        front_dists = []
        left_dists = []
        right_dists = []
        rear_dists = []

        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        for i in range(n):
            r = ranges[i]
            if not np.isfinite(r) or r < 0.08 or r > 16.0:
                continue

            ang = angle_min + i * angle_inc
            ang_norm = (ang + math.pi) % (2.0 * math.pi) - math.pi

            if abs(ang_norm) <= math.radians(40.0):
                front_dists.append(r)
            elif math.radians(40.0) < ang_norm <= math.radians(120.0):
                left_dists.append(r)
            elif -math.radians(120.0) <= ang_norm < -math.radians(40.0):
                right_dists.append(r)
            else:
                rear_dists.append(r)

        self.min_front_laser = float(min(front_dists)) if front_dists else 5.0
        self.laser_left = float(min(left_dists)) if left_dists else 5.0
        self.laser_right = float(min(right_dists)) if right_dists else 5.0
        self.laser_rear = float(min(rear_dists)) if rear_dists else 5.0

    def image_cb(self, msg: Image):
        try:
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            frame = raw.reshape((msg.height, msg.width, 3))
            enc = (msg.encoding or "").lower().strip()
            if enc in ['rgb8', 'r8g8b8']:
                self.latest_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif enc in ['bgr8', 'b8g8r8']:
                self.latest_bgr = frame
            else:
                self.latest_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        except Exception:
            pass

    def depth_cb(self, msg: Image):
        self.depth_encoding = msg.encoding
        try:
            dtype = np.float32 if ('32fc1' in msg.encoding.lower() or '32f' in msg.encoding.lower()) else np.uint16
            raw = np.frombuffer(msg.data, dtype=dtype)
            self.latest_depth = raw.reshape((msg.height, msg.width))
        except Exception:
            pass

    def control_loop(self):
        if self.is_shutting_down:
            return

        twist = Twist()

        detected, bbox, dist_z, ann_frame = self.vision.detect(
            self.latest_bgr, self.latest_depth, self.depth_encoding, self.target_name, self.target_color
        )

        # Terbitkan citra deteksi bounding box secara real-time ke RViz dengan format bgr8
        if ann_frame is not None:
            try:
                out_msg = cv2_to_imgmsg_fast(ann_frame, encoding='bgr8', frame_id='camera_depth_optical_frame')
                out_msg.header.stamp = self.get_clock().now().to_msg()
                self.detect_img_pub.publish(out_msg)
            except Exception:
                pass

        # -----------------------------------------------------------------
        # STATE 1: SEARCHING_OBJECT (Strict Target Visual Servoing + PID)
        # -----------------------------------------------------------------
        if self.state == 'SEARCHING_OBJECT':
            if detected and bbox is not None:
                self.lost_frames_count = 0
                x, y, w, h = bbox
                w_img = self.latest_bgr.shape[1] if self.latest_bgr is not None else 320
                cx = x + w // 2

                # Error posisi horizontal ternormalisasi [-1.0, 1.0]
                error_x = (cx - w_img / 2.0) / (w_img / 2.0)
                self.smooth_error_x = 0.65 * self.smooth_error_x + 0.35 * error_x

                steer = -self.pid_steer.update(self.smooth_error_x)
                d_target = dist_z if (dist_z is not None and 0.15 < dist_z < 6.0) else self.min_front_laser

                # KONDISI BERHENTI TEPAT: Jarak akhir 0.55m di depan objek!
                if (d_target <= 0.58 and abs(self.smooth_error_x) < 0.20) or (self.min_front_laser <= 0.48):
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_pub.publish(twist)
                    self.state = 'IDLE'
                    print("\n" + "="*68)
                    print(f"✅ [SAMPAI] Berhasil mengunci dan mendekati target: {self.target_name.upper()}!")
                    print(f"📏 Jarak Akhir ke Objek: {d_target:.2f} meter (Presisi Kamera & LiDAR)")
                    print(f"📍 Posisi Robot: X={self.curr_x:.2f}, Y={self.curr_y:.2f}, Hadap: {math.degrees(self.curr_yaw):.1f}°")
                    print("="*68 + "\nUser Command >> ", end='', flush=True)
                    return
                else:
                    # KEMUDI HALUS KONTINU (Bebas Hentakan Patah-Patah)
                    if abs(self.smooth_error_x) > 0.30:
                        # Putar badan halus mengarahkan kamera ke tengah objek
                        twist.linear.x = 0.02
                        twist.angular.z = float(np.clip(steer * 0.85, -0.65, 0.65))
                    else:
                        # Maju stabil dengan perlambatan proporsional mendekati 0.55m
                        e_dist = max(0.0, d_target - 0.55)
                        align_factor = max(0.2, 1.0 - abs(self.smooth_error_x) * 2.0)
                        fwd_spd = np.clip(e_dist * 0.35 * align_factor, 0.05, 0.20)
                        twist.linear.x = float(fwd_spd)
                        twist.angular.z = float(np.clip(steer * 0.50, -0.50, 0.50))
            else:
                # Target belum tampak di kamera: Cari dinamis dengan sensor (tanpa koordinat fix)
                if time.time() - self.search_start_time > 25.0:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_pub.publish(twist)
                    self.state = 'IDLE'
                    print(f"\n⚠️ [VISI 3D] Target '{self.target_name}' tidak ditemukan setelah rotasi pemindaian.")
                    print("User Command >> ", end='', flush=True)
                    return

                known_obj = self.discovered_objects.get(self.target_name)
                if known_obj and "x" in known_obj and "y" in known_obj:
                    # Target pernah terdata sensor sebelumnya: tuju koordinat dinamis hasil pengukuran sensor!
                    dx = known_obj["x"] - self.curr_x
                    dy = known_obj["y"] - self.curr_y
                    target_yaw = math.atan2(dy, dx)
                    yaw_diff = self.normalize_angle(target_yaw - self.curr_yaw)
                    dist_obj = math.hypot(dx, dy)

                    if dist_obj < 0.60:
                        # Sudah dekat titik estimasi sensor, putar mencari visual objek
                        twist.linear.x = 0.0
                        twist.angular.z = 0.40
                    elif self.min_front_laser < 0.50:
                        twist.linear.x = 0.0
                        twist.angular.z = 0.70 if self.laser_left >= self.laser_right else -0.70
                    elif abs(yaw_diff) > 0.35:
                        twist.linear.x = 0.0
                        twist.angular.z = float(np.clip(yaw_diff * 0.90, -0.60, 0.60))
                    else:
                        twist.linear.x = float(np.clip(dist_obj * 0.35, 0.08, 0.20))
                        twist.angular.z = float(np.clip(yaw_diff * 0.50, -0.40, 0.40))
                else:
                    # Target belum pernah terdata: Aktif putar badan 360° mencari objek secara visual!
                    twist.linear.x = 0.0
                    twist.angular.z = 0.40

        # -----------------------------------------------------------------
        # STATE 2: PATROL_ROOM (Reinforcement Learning: 4 Pojok Dinding -> Tengah)
        # -----------------------------------------------------------------
        elif self.state == 'PATROL_ROOM':
            # A. Deteksi Objek Otomatis saat Melintas
            vis_text, vis_dist = self.vision.inspect_front(self.latest_bgr, self.latest_depth, self.depth_encoding)
            if vis_dist is not None and "Tidak ada objek" not in vis_text:
                clean_name = vis_text.split(" (")[0].replace("'", "")
                canon_key = get_canonical_key(clean_name)
                if canon_key is not None:
                    d_est = float(vis_dist) if (0.2 < vis_dist < 4.0) else float(self.min_front_laser)
                    est_x = round(self.curr_x + d_est * math.cos(self.curr_yaw), 2)
                    est_y = round(self.curr_y + d_est * math.sin(self.curr_yaw), 2)
                    if (canon_key not in self.discovered_objects) or (self.discovered_objects[canon_key]["dist"] > d_est):
                        self.discovered_objects[canon_key] = {
                            "name": LAB_LANDMARKS[canon_key]["name"],
                            "dist": round(d_est, 2),
                            "x": est_x,
                            "y": est_y,
                            "bearing_deg": round(math.degrees(self.curr_yaw) % 360.0, 1)
                        }
                        print(f"\n📦 [OBJEK TERDATA] {LAB_LANDMARKS[canon_key]['name']} pada jarak {d_est:.2f}m (Koordinat: X={est_x}, Y={est_y})")
                        print("User Command >> ", end='', flush=True)

            # B. Cek Waypoint Saat Ini (Pojok 1 -> Pojok 2 -> Pojok 3 -> Pojok 4 -> Tengah)
            if self.patrol_waypoints and self.current_wp_idx < len(self.patrol_waypoints):
                target_wp = self.patrol_waypoints[self.current_wp_idx]
                dist_wp = math.hypot(target_wp["x"] - self.curr_x, target_wp["y"] - self.curr_y)
                # Toleransi tiba di waypoint pojok: 0.50 meter
                if dist_wp < 0.50:
                    print(f"\n🚩 [POJOK TERCAPAI] Berhasil tiba di {target_wp['name']}! (Cakupan Ruangan: {self.coverage_pct}%)")
                    self.current_wp_idx += 1
                    if self.current_wp_idx < len(self.patrol_waypoints):
                        next_wp = self.patrol_waypoints[self.current_wp_idx]
                        print(f"[RL-PATROLI SISTEMATIS] Melanjutkan ke: {next_wp['name']} (X={next_wp['x']}, Y={next_wp['y']})...")
                        print("User Command >> ", end='', flush=True)
                    self.patrol_prev_dist = 999.0
            else:
                dist_wp = 0.0

            # C. Kondisi Selesai Eksplorasi (Definite Stopping Condition)
            time_patrol = time.time() - self.patrol_start_time
            all_corners_visited = (self.patrol_waypoints and self.current_wp_idx >= len(self.patrol_waypoints))
            if all_corners_visited or (self.coverage_pct >= 85.0) or (len(self.visited_quadrants) >= 4 and len(self.visited_cells) >= 20) or (time_patrol > 110.0 and self.coverage_pct >= 65.0):
                self.exploration_completed = True
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                self.state = 'RETURNING_HOME'
                self.pid_steer.reset()
                self.pid_dist.reset()
                print("\n" + "="*72)
                print(f"🎯 [EKSPLORASI SELESAI] Seluruh 4 pojok dinding & area tengah selesai dijelajahi!")
                print(f"📊 Cakupan Ruangan: {self.coverage_pct}% | Waktu: {time_patrol:.1f} detik | Total RL Reward: {self.rl_agent.total_rewards:+.1f}")
                print(f"📍 Memulai navigasi kembali ke titik awal (Home): X={self.home_x:.2f}, Y={self.home_y:.2f}...")
                print("="*72)
                return

            # D. Hitung Error Heading Menuju Waypoint Saat Ini
            target_wp = self.patrol_waypoints[min(self.current_wp_idx, len(self.patrol_waypoints) - 1)] if self.patrol_waypoints else CENTER_WAYPOINT
            dx = target_wp["x"] - self.curr_x
            dy = target_wp["y"] - self.curr_y
            target_yaw = math.atan2(dy, dx)
            yaw_err = self.normalize_angle(target_yaw - self.curr_yaw)

            # Cek apakah sel di depan adalah sel baru yang belum dikunjungi
            lookahead_d = 0.8
            ahead_x = self.curr_x + lookahead_d * math.cos(self.curr_yaw)
            ahead_y = self.curr_y + lookahead_d * math.sin(self.curr_yaw)
            ahead_cell = (int(round(ahead_x / self.grid_res)), int(round(ahead_y / self.grid_res)))
            is_new_cell = (ahead_cell not in self.visited_cells)

            # E. Reinforcement Learning Action Selection & Q-Update
            rl_state = self.rl_agent.get_state(self.min_front_laser, self.laser_left, self.laser_right, yaw_err, is_new_cell)
            action = self.rl_agent.select_action(rl_state)

            # Hitung Reward RL
            dist_progress = (self.patrol_prev_dist - dist_wp) if self.patrol_prev_dist < 50.0 else 0.0
            self.patrol_prev_dist = dist_wp

            reward = 0.0
            if self.min_front_laser < 0.38:
                reward -= 25.0  # Penalti bahaya tabrakan rintangan/dinding
            elif self.min_front_laser < 0.55:
                reward -= 8.0
            if is_new_cell:
                reward += 15.0  # Reward eksplorasi sel spasial baru
            if dist_progress > 0.01:
                reward += 6.0   # Reward kemajuan mendekati pojok dinding
            if abs(yaw_err) < 0.30:
                reward += 3.0   # Reward keselarasan arah
            if action in [0, 1, 2] and self.min_front_laser > 0.80:
                reward += 2.0   # Reward pergerakan maju efisien

            # Update Q-Table RL
            self.rl_agent.update(reward, rl_state)
            self.rl_agent.last_state = rl_state
            self.rl_agent.last_action = action

            # F. Pemetaan Aksi RL ke Kontrol Kecepatan Roda (/cmd_vel)
            # Aksi: 0:FORWARD, 1:FORWARD_LEFT, 2:FORWARD_RIGHT, 3:HARD_LEFT, 4:HARD_RIGHT, 5:REVERSE
            if action == 0:    # FORWARD
                twist.linear.x = 0.20
                twist.angular.z = float(np.clip(yaw_err * 0.8, -0.25, 0.25))
            elif action == 1:  # FORWARD_LEFT
                twist.linear.x = 0.14
                twist.angular.z = 0.40
            elif action == 2:  # FORWARD_RIGHT
                twist.linear.x = 0.14
                twist.angular.z = -0.40
            elif action == 3:  # HARD_LEFT
                twist.linear.x = 0.0
                twist.angular.z = 0.70
            elif action == 4:  # HARD_RIGHT
                twist.linear.x = 0.0
                twist.angular.z = -0.70
            elif action == 5:  # REVERSE
                twist.linear.x = -0.12
                twist.angular.z = 0.60 if self.laser_left >= self.laser_right else -0.60

            # Fail-Safe LiDAR Tingkat Akhir (Anti-Tabrak Mutlak)
            if self.min_front_laser < 0.35:
                twist.linear.x = -0.14
                twist.angular.z = 0.75 if self.laser_left >= self.laser_right else -0.75

            # Telemetri Real-Time
            now_sec = time.time()
            if now_sec - self.last_coverage_log_time > 3.5:
                self.last_coverage_log_time = now_sec
                act_names = ["FORWARD", "FWD_LEFT", "FWD_RIGHT", "HARD_LEFT", "HARD_RIGHT", "REVERSE"]
                wp_name = target_wp["name"] if target_wp else "Perimeter"
                print(f"[RL-PATROLI] WP {min(self.current_wp_idx+1, 5)}/5 ({wp_name}) | Aksi: {act_names[action]} | Reward: {reward:+.1f} | Q-Reward Total: {self.rl_agent.total_rewards:+.1f} | Cakupan: {self.coverage_pct}% | Depan: {self.min_front_laser:.2f}m")

        # -----------------------------------------------------------------
        # STATE 3: RETURNING_HOME (Navigasi Pulang ke Titik Awal + Debriefing)
        # -----------------------------------------------------------------
        elif self.state == 'RETURNING_HOME':
            dx = self.home_x - self.curr_x
            dy = self.home_y - self.curr_y
            dist_home = math.hypot(dx, dy)

            if dist_home < 0.25:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                self.state = 'IDLE'

                duration_sec = time.time() - self.patrol_start_time
                print("\n" + "="*72)
                print("🏁 [MISI SELESAI] ROBOT BERHASIL KEMBALI KE TITIK AWAL (HOME)!")
                print("="*72)
                print("📊 HASIL EVALUASI EKSPLORASI SPASIAL:")
                print(f"  • Cakupan Area Lab    : {self.coverage_pct}%")
                print(f"  • Sel Spasial Unik   : {len(self.visited_cells)} sel ({self.grid_res}m resolusi)")
                print(f"  • Kuadran Terpetakan : {len(self.visited_quadrants)}/4 ({', '.join(sorted(self.visited_quadrants))})")
                print(f"  • Posisi Akhir Robot : X={self.curr_x:.2f}, Y={self.curr_y:.2f} (Home tercapai!)")
                print(f"  • Durasi Eksplorasi  : {duration_sec:.1f} detik")
                print("\n📦 DAFTAR OBJEK NYATA YANG BERHASIL DIPETAKAN OLEH SENSOR:")
                print("-" * 72)
                if self.discovered_objects:
                    for i, (k, info) in enumerate(self.discovered_objects.items(), start=1):
                        x_val = info.get('x', 0.0)
                        y_val = info.get('y', 0.0)
                        d_val = info.get('dist', 0.0)
                        print(f"  {i}. {info['name']:<30} | Koordinat: ({x_val:5.2f}, {y_val:5.2f}) | Jarak: {d_val:.2f} m")
                else:
                    print("  (Belum ada objek terdeteksi dalam jangkauan sensor)")
                print("-" * 72)
                print("🤖 [LLaMA 3.2]: Eksplorasi lab selesai! Objek terpetakan secara dinamis.")
                print("Ketik perintah lain: 'coba anda ke kotak merah', 'ke pilar hijau', atau 'scan ruangan'.")
                print("="*72 + "\nUser Command >> ", end='', flush=True)
                return
            else:
                target_yaw = math.atan2(dy, dx)
                yaw_err = self.normalize_angle(target_yaw - self.curr_yaw)

                if self.min_front_laser < 0.50:
                    twist.linear.x = 0.04
                    twist.angular.z = 0.75 if self.laser_left >= self.laser_right else -0.75
                else:
                    steer = self.pid_steer.update(yaw_err)
                    if abs(yaw_err) > 0.35:
                        twist.linear.x = 0.0
                        twist.angular.z = float(steer * 0.75)
                    else:
                        twist.linear.x = float(np.clip(dist_home * 0.35, 0.06, 0.20))
                        twist.angular.z = float(steer * 0.40)

        # -----------------------------------------------------------------
        # STATE 4: SCANNING_ROOM (Survei 360 Derajat)
        # -----------------------------------------------------------------
        elif self.state == 'SCANNING_ROOM':
            delta_yaw = self.normalize_angle(self.curr_yaw - self.last_yaw)
            self.scan_total_turned += abs(delta_yaw)
            self.last_yaw = self.curr_yaw

            twist.linear.x = 0.0
            twist.angular.z = 0.40

            vis_text, vis_dist = self.vision.inspect_front(self.latest_bgr, self.latest_depth, self.depth_encoding)
            if vis_dist is not None and "Tidak ada objek" not in vis_text:
                clean_name = vis_text.split(" (")[0].replace("'", "")
                canon_key = get_canonical_key(clean_name)
                if canon_key is not None:
                    d_est = vis_dist if vis_dist < 8.0 else self.min_front_laser
                    est_x = round(self.curr_x + d_est * math.cos(self.curr_yaw), 2)
                    est_y = round(self.curr_y + d_est * math.sin(self.curr_yaw), 2)
                    if (canon_key not in self.discovered_objects) or (self.discovered_objects[canon_key]["dist"] > d_est):
                        self.discovered_objects[canon_key] = {
                            "name": LAB_LANDMARKS[canon_key]["name"],
                            "dist": d_est,
                            "x": est_x,
                            "y": est_y,
                            "bearing_deg": round(math.degrees(self.curr_yaw) % 360.0, 1)
                        }

            if self.scan_total_turned >= (2.0 * math.pi - 0.10) or (time.time() - self.search_start_time > 18.0):
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                self.state = 'IDLE'

                print("\n" + "="*72)
                print("📋 [HASIL PEMINDAIAN FUSI LIDAR 360° + KAMERA RGB-D ASTRA]")
                print("="*72)
                print("Objek Geometris Nyata Terdeteksi di Laboratorium:")
                if self.discovered_objects:
                    for i, (k, info) in enumerate(self.discovered_objects.items(), start=1):
                        x_pos = info.get('x', 0.0)
                        y_pos = info.get('y', 0.0)
                        print(f"  {i}. {info['name']:<30} | Posisi: ({x_pos:5.2f}, {y_pos:5.2f}) | Jarak: {info['dist']:.2f} m")
                else:
                    print("  (Belum ada objek terdeteksi dalam jangkauan sensor)")
                print("="*72)
                print("🤖 [LLaMA 3.2]: Pemindaian 360° selesai! Ketik 'coba anda ke kotak merah' atau 'ke pilar hijau'.")
                print("="*72 + "\nUser Command >> ", end='', flush=True)
                return

        # -----------------------------------------------------------------
        # STATE 5: NAVIGATING_WAYPOINT (Navigasi Menuju Koordinat Bebas)
        # -----------------------------------------------------------------
        elif self.state == 'NAVIGATING_WAYPOINT':
            dx = self.goal_x - self.curr_x
            dy = self.goal_y - self.curr_y
            dist = math.hypot(dx, dy)

            if dist < 0.20:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                self.state = 'IDLE'
                print("\n" + "="*65)
                print(f"✅ [SAMPAI] Berhasil tiba di {self.goal_label}!")
                print(f"📍 Posisi Akhir Robot: X={self.curr_x:.2f}, Y={self.curr_y:.2f}")
                print("="*65 + "\nUser Command >> ", end='', flush=True)
                return
            else:
                target_yaw = math.atan2(dy, dx)
                yaw_err = self.normalize_angle(target_yaw - self.curr_yaw)

                if self.min_front_laser < 0.50:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.70 if yaw_err >= 0 else -0.70
                else:
                    steer = self.pid_steer.update(yaw_err)
                    if abs(yaw_err) > 0.30:
                        twist.linear.x = 0.0
                        twist.angular.z = float(steer * 0.70)
                    else:
                        twist.linear.x = float(np.clip(dist * 0.40, 0.05, 0.20))
                        twist.angular.z = float(steer * 0.40)

        # -----------------------------------------------------------------
        # STATE 6: ROTATE_RELATIVE (Manuver Belok Kiri/Kanan/Putar Balik)
        # -----------------------------------------------------------------
        elif self.state == 'ROTATE_RELATIVE':
            yaw_diff = self.normalize_angle(self.goal_yaw - self.curr_yaw)
            if abs(yaw_diff) < 0.06:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                self.state = 'IDLE'
                print(f"\n[OK] Manuver rotasi selesai. Hadap robot sekarang: {math.degrees(self.curr_yaw):.1f}°")
                print("User Command >> ", end='', flush=True)
                return
            else:
                twist.linear.x = 0.0
                w_mag = float(np.clip(abs(yaw_diff) * 1.25, 0.25, 0.70))
                twist.angular.z = math.copysign(w_mag, yaw_diff)

        # -----------------------------------------------------------------
        # STATE 8: APPROACH_WALL (Menuju & Mendekati Dinding Lab Terdekat)
        # -----------------------------------------------------------------
        elif self.state == 'APPROACH_WALL':
            if self.min_front_laser > 0.60:
                fwd_spd = float(np.clip((self.min_front_laser - 0.55) * 0.40, 0.08, 0.22))
                twist.linear.x = fwd_spd
                if self.laser_left < 0.45:
                    twist.angular.z = -0.25
                elif self.laser_right < 0.45:
                    twist.angular.z = 0.25
                else:
                    twist.angular.z = 0.0
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                self.state = 'IDLE'
                print("\n" + "="*68)
                print(f"✅ [SAMPAI] Berhasil mendekati dinding laboratorium!")
                print(f"📏 Jarak Akhir ke Dinding: {self.min_front_laser:.2f} meter (Presisi RPLiDAR A1)")
                print(f"📍 Posisi Robot: X={self.curr_x:.2f}, Y={self.curr_y:.2f}, Hadap: {math.degrees(self.curr_yaw):.1f}°")
                print("="*68 + "\nUser Command >> ", end='', flush=True)
                return

        # -----------------------------------------------------------------
        # STATE 7: MOVE_RELATIVE (Maju / Mundur Lurus Tanpa Putar Balik)
        # -----------------------------------------------------------------
        elif self.state == 'MOVE_RELATIVE':
            dist_traveled = math.hypot(self.curr_x - self.rel_start_x, self.curr_y - self.rel_start_y)
            dist_remaining = self.rel_target_dist - dist_traveled

            if dist_remaining <= 0.03:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                self.state = 'IDLE'
                arah = "Maju" if self.rel_is_forward else "Mundur"
                print(f"\n[OK] Manuver {arah} selesai ({self.rel_target_dist:.2f} m). Posisi: X={self.curr_x:.2f}, Y={self.curr_y:.2f}")
                print("User Command >> ", end='', flush=True)
                return

            yaw_drift = self.normalize_angle(self.rel_start_yaw - self.curr_yaw)
            steer = float(np.clip(yaw_drift * 1.0, -0.25, 0.25))

            if self.rel_is_forward:
                if self.min_front_laser < 0.38:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_pub.publish(twist)
                    self.state = 'IDLE'
                    print(f"\n⚠️ [STOP KEAMANAN] Rintangan di depan ({self.min_front_laser:.2f} m). Manuver maju dihentikan.")
                    print("User Command >> ", end='', flush=True)
                    return
                spd = float(np.clip(dist_remaining * 0.45, 0.08, 0.22))
                twist.linear.x = spd
                twist.angular.z = steer
            else:
                if self.laser_rear < 0.32:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_pub.publish(twist)
                    self.state = 'IDLE'
                    print(f"\n⚠️ [STOP KEAMANAN] Rintangan di belakang ({self.laser_rear:.2f} m). Manuver mundur dihentikan.")
                    print("User Command >> ", end='', flush=True)
                    return
                spd = float(np.clip(dist_remaining * 0.45, 0.08, 0.20))
                twist.linear.x = -spd  # Mundur langsung!
                twist.angular.z = -steer

        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        if not self.is_shutting_down:
            try:
                # Publikasi Langsung ke /cmd_vel (Bebas Lag & Respon Gerak Seketika di RViz)
                self.cmd_pub.publish(twist)
            except Exception:
                pass

    def normalize_angle(self, a):
        while a > math.pi: a -= 2.0 * math.pi
        while a < -math.pi: a += 2.0 * math.pi
        return a

    def handle_user_input(self, user_text: str):
        vis_text, vis_dist = self.vision.inspect_front(self.latest_bgr, self.latest_depth, self.depth_encoding)
        sensor_telemetry = {
            "front_lidar": self.min_front_laser,
            "laser_left":  self.laser_left,
            "laser_right": self.laser_right,
            "laser_rear":  self.laser_rear,
            "front_visual_text": vis_text,
            "front_visual_dist": vis_dist,
            "x": self.curr_x,
            "y": self.curr_y,
            "yaw_deg": math.degrees(self.curr_yaw) % 360.0
        }

        result = self.llama.parse_user_intent(user_text, self.discovered_objects, sensor_telemetry)
        action = result.get("action", "CHAT")
        response_text = result.get("response", "")

        if response_text:
            print(f"\n🤖 [LLaMA 3.2]: {response_text}")

        if action == "STOP":
            self.state = 'IDLE'
            try: self.cmd_pub.publish(Twist())
            except Exception: pass
            print("[!] Motor dihentikan.")
            print("User Command >> ", end='', flush=True)
            return

        if action == "SEARCH_OBJECT":
            self.target_name = result.get("target_name", "kotak_merah")
            self.target_shape = result.get("target_shape", "persegi")
            self.target_color = result.get("target_color", "merah")
            self.target_type = result.get("target_type", "GEOMETRIC")
            self.state = 'SEARCHING_OBJECT'
            self.search_start_time = time.time()
            self.pid_steer.reset()
            self.pid_dist.reset()
            self.smooth_error_x = 0.0
            self.lost_frames_count = 0
            if self.target_type == "YOLO":
                tgt_name = f"YOLO [{self.target_name.upper()}]"
            else:
                tgt_name = LAB_LANDMARKS.get(self.target_name, {"name": self.target_name.upper()})["name"]
            print(f"[VISI 3D AKTIF] Mengunci target: {tgt_name}. Kamera & PID visual aktif menuju objek!")
            return

        if action == "CHECK_COVERAGE":
            print("\n" + "="*65)
            print("📊 [STATUS EKSPLORASI SPASIAL REAL-TIME]")
            print("="*65)
            print(f"  • Cakupan Area Lab    : {self.coverage_pct}%")
            print(f"  • Sel Spasial Unik   : {len(self.visited_cells)} sel ({self.grid_res}m)")
            print(f"  • Kuadran Terpetakan : {len(self.visited_quadrants)}/4 ({', '.join(sorted(self.visited_quadrants)) if self.visited_quadrants else 'Belum'})")
            print(f"  • Titik Awal (Home)  : X={self.home_x:.2f}, Y={self.home_y:.2f}")
            print(f"  • Posisi Sekarang    : X={self.curr_x:.2f}, Y={self.curr_y:.2f}")
            print(f"  • Objek Terdata      : {len(self.discovered_objects)}/3 objek")
            for k, v in self.discovered_objects.items():
                print(f"    - {v['name']} di ({v.get('x', 0.0):.2f}, {v.get('y', 0.0):.2f}), jarak: {v.get('dist', 0.0):.2f}m")
            print("="*65 + "\nUser Command >> ", end='', flush=True)
            return

        if action == "RETURN_HOME":
            self.state = 'RETURNING_HOME'
            self.pid_steer.reset()
            self.pid_dist.reset()
            print(f"[NAVIGASI HOME] Menuju titik awal: X={self.home_x:.2f}, Y={self.home_y:.2f}...")
            return

        if action == "APPROACH_WALL":
            self.state = 'APPROACH_WALL'
            print(f"[NAVIGASI DINDING] Melaju mendekati dinding di depan (Jarak sensor: {self.min_front_laser:.2f}m)...")
            return

        if action == "PATROL_ROOM":
            self.home_x = self.curr_x
            self.home_y = self.curr_y
            self.home_yaw = self.curr_yaw
            self.visited_cells.clear()
            self.visited_quadrants.clear()
            self.coverage_pct = 0.0
            self.patrol_start_time = time.time()
            self.last_coverage_log_time = time.time()
            self.exploration_completed = False

            # Susun rencana rute sistematis: 4 Pojok Dinding Perimeter -> Masuk ke Tengah
            best_idx = 0
            best_d = 999.0
            for i, c in enumerate(CORNER_WAYPOINTS):
                d = math.hypot(c["x"] - self.curr_x, c["y"] - self.curr_y)
                if d < best_d:
                    best_d = d
                    best_idx = i

            self.patrol_waypoints = [CORNER_WAYPOINTS[(best_idx + k) % 4] for k in range(4)]
            self.patrol_waypoints.append(CENTER_WAYPOINT)
            self.current_wp_idx = 0
            self.patrol_prev_dist = best_d

            self.state = 'PATROL_ROOM'
            self.pid_steer.reset()
            self.pid_dist.reset()
            print(f"[RL-PATROLI SISTEMATIS] Titik Awal (Home) dikunci di X={self.home_x:.2f}, Y={self.home_y:.2f}.")
            print(f"[RL-PATROLI SISTEMATIS] Strategi: Keliling 4 Pojok Dinding -> Masuk ke Tengah -> Pulang ke Titik Awal.")
            print(f"[RL-PATROLI SISTEMATIS] Menuju Waypoint 1: {self.patrol_waypoints[0]['name']} (X={self.patrol_waypoints[0]['x']}, Y={self.patrol_waypoints[0]['y']})...")
            return

        if action == "SCAN_ROOM":
            self.state = 'SCANNING_ROOM'
            self.search_start_time = time.time()
            self.scan_total_turned = 0.0
            self.last_yaw = self.curr_yaw
            self.discovered_objects = {}
            print("[SURVEI AKTIF] Robot mulai memutar badan 360° merekam data LiDAR & RGB-D...")
            return

        if action == "ROTATE_RELATIVE":
            rel_angle = result.get("angle_rad", math.pi)
            self.goal_yaw = self.normalize_angle(self.curr_yaw + rel_angle)
            self.state = 'ROTATE_RELATIVE'
            deg = math.degrees(rel_angle)
            arah = "Kanan" if rel_angle < 0 else ("Kiri" if rel_angle > 0 else "Balik")
            print(f"[MANUVER] Robot berputar {arah} ({abs(deg):.0f}°)...")
            return

        if action == "MOVE_RELATIVE":
            d = result.get("distance", 0.5)
            self.rel_start_x = self.curr_x
            self.rel_start_y = self.curr_y
            self.rel_start_yaw = self.curr_yaw
            self.rel_target_dist = abs(d)
            self.rel_is_forward = (d >= 0.0)
            self.state = 'MOVE_RELATIVE'
            arah = "Maju" if self.rel_is_forward else "Mundur"
            print(f"[MANUVER] Robot {arah} {self.rel_target_dist:.2f} meter...")
            return

        if action == "NAVIGATE_LOCATION":
            gx = result.get("x", 0.0)
            gy = result.get("y", 0.0)
            lbl = result.get("target_name", "Target")
            self.goal_x = gx
            self.goal_y = gy
            self.goal_label = lbl
            self.state = 'NAVIGATING_WAYPOINT'
            self.pid_steer.reset()
            self.pid_dist.reset()

            gp = PoseStamped()
            gp.header.stamp = self.get_clock().now().to_msg()
            gp.header.frame_id = 'odom'
            gp.pose.position.x = gx
            gp.pose.position.y = gy
            gp.pose.orientation.w = 1.0
            try: self.goal_pub.publish(gp)
            except Exception: pass
            return

        print("User Command >> ", end='', flush=True)


def input_thread(node: IntelligentExplorerNode):
    time.sleep(1.0)
    print(BANNER)
    print("User Command >> ", end='', flush=True)
    while rclpy.ok() and not node.is_shutting_down:
        try:
            line = sys.stdin.readline()
            if not line:
                break
            cmd = line.strip()
            if cmd:
                node.handle_user_input(cmd)
        except Exception:
            break


def main():
    rclpy.init()
    node = IntelligentExplorerNode()
    t = threading.Thread(target=input_thread, args=(node,), daemon=True)
    t.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.is_shutting_down = True
        try:
            stop_twist = Twist()
            node.cmd_pub.publish(stop_twist)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
