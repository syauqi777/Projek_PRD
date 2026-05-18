"""
=============================================================================
  SISTEM DETEKSI HELM — FINAL FIXED VERSION
  Fix: opencv crash, servo tidak jalan, display lambat

  SEBELUM JALANKAN — perbaiki OpenCV dulu di Command Prompt:
    pip uninstall opencv-python opencv-python-headless -y
    pip install opencv-python

  CARA JALANKAN:
    python main_integrasi.py

  Tekan Q di jendela kamera untuk berhenti.
=============================================================================
"""

import serial
import serial.tools.list_ports
import cv2
import csv
import sqlite3
import re
import time
import os
import threading
from datetime import datetime
from ultralytics import YOLO
import easyocr

# ════════════════════════════════════════════════════════
#  KONFIGURASI
# ════════════════════════════════════════════════════════
MODEL_PATH  = "C:/Users/Fakhri Kartiko/Documents/DETEKSI HELM UPGRADE/runs/detect/train2/weights/best.pt"
SERIAL_PORT = "COM3"
BAUD_RATE   = 115200

# Sesuai data.yaml: 0=driver, 1=gapake_helm, 2=pake_helm, 3=plat_nomor
CLASS_TANPA_HELM = 1
CLASS_PAKAI_HELM = 2
CLASS_PLAT       = 3

CONF_THRESHOLD = 0.35
KAMERA_INDEX   = 0

LOG_DIR      = "logs"
CSV_FILE     = "logs/access_log.csv"
DB_FILE      = "logs/access_log.db"
SNAPSHOT_DIR = "logs/snapshots"

# ════════════════════════════════════════════════════════
#  SETUP DATABASE & CSV
# ════════════════════════════════════════════════════════
os.makedirs(LOG_DIR,      exist_ok=True)
os.makedirs(SNAPSHOT_DIR, exist_ok=True)

def init_db():
    con = sqlite3.connect(DB_FILE)
    con.execute("""CREATE TABLE IF NOT EXISTS access_log (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        waktu TEXT, rfid_uid TEXT, plat_nomor TEXT,
        status_helm TEXT, hasil TEXT, snapshot TEXT)""")
    con.commit(); con.close()

def init_csv():
    if not os.path.exists(CSV_FILE):
        with open(CSV_FILE, 'w', newline='', encoding='utf-8') as f:
            csv.writer(f).writerow(
                ["waktu","rfid_uid","plat_nomor","status_helm","hasil","snapshot"])

def simpan_log(rfid, plat, helm_str, hasil_str, snap=""):
    waktu = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    try:
        con = sqlite3.connect(DB_FILE)
        con.execute("INSERT INTO access_log VALUES (NULL,?,?,?,?,?,?)",
                    (waktu,rfid,plat,helm_str,hasil_str,snap))
        con.commit(); con.close()
    except Exception as e:
        print(f"[DB ERROR] {e}")
    try:
        with open(CSV_FILE,'a',newline='',encoding='utf-8') as f:
            csv.writer(f).writerow([waktu,rfid,plat,helm_str,hasil_str,snap])
    except Exception as e:
        print(f"[CSV ERROR] {e}")
    print(f"[LOG] {waktu} | RFID:{rfid} | Plat:{plat or '-'} | {hasil_str}")

# ════════════════════════════════════════════════════════
#  OCR PLAT
# ════════════════════════════════════════════════════════
def baca_plat(frame, box, ocr):
    x1,y1,x2,y2 = map(int, box.xyxy[0])
    pad = 8
    h,w = frame.shape[:2]
    crop = frame[max(0,y1-pad):min(h,y2+pad), max(0,x1-pad):min(w,x2+pad)]
    if crop.size == 0: return ""
    gray   = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    scaled = cv2.resize(gray, None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC)
    _,th   = cv2.threshold(scaled,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    hasil  = ocr.readtext(th, detail=0, paragraph=False)
    return re.sub(r'[^A-Z0-9\s]','', " ".join(hasil).upper()).strip()

# ════════════════════════════════════════════════════════
#  OVERLAY DISPLAY
# ════════════════════════════════════════════════════════
def draw_status_bar(frame, teks, warna):
    """Bar hitam semi-transparan di atas, teks status."""
    h, w = frame.shape[:2]
    bar = frame.copy()
    cv2.rectangle(bar, (0,0), (w,54), (20,20,20), -1)
    cv2.addWeighted(bar, 0.6, frame, 0.4, 0, frame)
    cv2.putText(frame, teks, (14,38),
                cv2.FONT_HERSHEY_SIMPLEX, 0.85, warna, 2, cv2.LINE_AA)

def draw_result_banner(frame, teks, warna):
    """Banner besar di tengah layar saat ada keputusan akses."""
    h, w = frame.shape[:2]
    bar = frame.copy()
    cv2.rectangle(bar, (0, h//2-60), (w, h//2+60), (10,10,10), -1)
    cv2.addWeighted(bar, 0.7, frame, 0.3, 0, frame)
    fs = 1.2; tk = 3
    (tw,th),_ = cv2.getTextSize(teks, cv2.FONT_HERSHEY_SIMPLEX, fs, tk)
    cv2.putText(frame, teks, ((w-tw)//2, h//2+th//2),
                cv2.FONT_HERSHEY_SIMPLEX, fs, warna, tk, cv2.LINE_AA)

# ════════════════════════════════════════════════════════
#  SHARED STATE (thread-safe)
# ════════════════════════════════════════════════════════
class Shared:
    def __init__(self):
        self.lock          = threading.Lock()
        self.frame         = None   # frame BGR terbaru dari webcam
        self.det           = None   # hasil parse YOLO terbaru
        # Trigger dari ESP32
        self.trigger       = False
        self.rfid_uid      = ""
        self.busy          = False  # True = sedang proses, tolak trigger baru
        # Banner hasil
        self.banner_teks   = ""
        self.banner_warna  = (255,255,255)
        self.banner_until  = 0      # timestamp kapan banner berhenti

sh = Shared()

# ════════════════════════════════════════════════════════
#  THREAD 1 — SERIAL LISTENER
#  Baca semua output ESP32, aksi saat ada "RFID:..."
# ════════════════════════════════════════════════════════
def serial_listener(ser):
    buf = ""
    while True:
        try:
            if ser.in_waiting:
                buf += ser.read(ser.in_waiting).decode('utf-8', errors='replace')
                while '\n' in buf:
                    line, buf = buf.split('\n', 1)
                    line = line.strip()
                    if not line: continue
                    print(f"  [ESP32] {line}")
                    if line.upper().startswith("RFID:"):
                        uid = line.split(":",1)[1].strip()
                        with sh.lock:
                            if not sh.busy:
                                sh.trigger  = True
                                sh.rfid_uid = uid
                                sh.busy     = True
                                print(f"\n>>> TRIGGER: RFID={uid}")
            time.sleep(0.02)
        except Exception as e:
            print(f"[SERIAL-ERR] {e}")
            time.sleep(1)

# ════════════════════════════════════════════════════════
#  THREAD 2 — DETECTION PROCESSOR
#  Saat ada trigger: kumpulkan hasil YOLO 3 detik,
#  kirim sinyal ke ESP32, simpan log
# ════════════════════════════════════════════════════════
def detection_processor(ser, ocr):
    while True:
        with sh.lock:
            ada = sh.trigger
            uid = sh.rfid_uid
        if not ada:
            time.sleep(0.05)
            continue

        # ── Banner "sedang menganalisa" ──
        with sh.lock:
            sh.banner_teks  = "MENGANALISA... TETAP DI DEPAN KAMERA"
            sh.banner_warna = (0, 200, 255)
            sh.banner_until = time.time() + 999   # terus sampai hasil keluar

        print("[DETECT] Mengumpulkan frame 3 detik...")
        t0 = time.time()
        helm_ok      = False
        plat_teks    = ""
        frame_snap   = None
        best_conf    = 0.0

        while time.time() - t0 < 3.0:
            with sh.lock:
                det   = sh.det
                frame = sh.frame.copy() if sh.frame is not None else None
            if det and frame is not None:
                if det["pakai_helm"] and det["conf"] > best_conf:
                    best_conf  = det["conf"]
                    helm_ok    = True
                    frame_snap = frame.copy()
                if det["plat_box"] is not None and not plat_teks:
                    plat_teks = baca_plat(frame, det["plat_box"], ocr)
            time.sleep(0.1)

        # ── Tunggu sebentar agar ESP32 sudah di state TUNGGU_PC ──
        time.sleep(0.5)

        # ── Kirim ke ESP32 ──
        pesan = "HELM_OK" if helm_ok else "NO_HELM"
        try:
            ser.write((pesan + "\n").encode())
            ser.flush()
            print(f"[TX → ESP32] {pesan}")
        except Exception as e:
            print(f"[TX-ERR] {e}")

        # ── Set banner hasil ──
        if helm_ok:
            b_teks  = "AKSES DITERIMA - Helm Terdeteksi!"
            b_warna = (0, 255, 80)
            h_str   = "PAKAI_HELM"; r_str = "AKSES_DITERIMA"
        else:
            b_teks  = "AKSES DITOLAK - Tidak Ada Helm!"
            b_warna = (0, 60, 255)
            h_str   = "TANPA_HELM"; r_str = "AKSES_DITOLAK"

        with sh.lock:
            sh.banner_teks  = b_teks
            sh.banner_warna = b_warna
            sh.banner_until = time.time() + 4.0

        # ── Snapshot ──
        snap = ""
        if frame_snap is not None:
            ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
            snap = f"{SNAPSHOT_DIR}/snap_{uid}_{ts}.jpg"
            cv2.imwrite(snap, frame_snap)
            print(f"[SNAP] {snap}")

        simpan_log(uid, plat_teks, h_str, r_str, snap)

        # ── Reset ──
        with sh.lock:
            sh.trigger  = False
            sh.rfid_uid = ""
            sh.busy     = False
        print("[DETECT] Selesai. Siap kendaraan berikutnya.\n")

# ════════════════════════════════════════════════════════
#  MAIN — WEBCAM + YOLO ALWAYS ON
#  cv2.imshow WAJIB di main thread (Windows)
# ════════════════════════════════════════════════════════
def main():
    print("\n" + "="*55)
    print("  SISTEM DETEKSI HELM — REALTIME")
    print("  Tekan Q di jendela kamera untuk keluar")
    print("="*55 + "\n")

    # Cek versi OpenCV punya GUI atau tidak
    build_info = cv2.getBuildInformation()
    if "GTK" not in build_info and "Win32 UI" not in build_info \
            and "Cocoa" not in build_info and "Qt" not in build_info:
        print("="*55)
        print("[!] PERINGATAN: OpenCV tidak punya GUI support!")
        print("    Jalankan dua perintah ini di Command Prompt,")
        print("    lalu jalankan ulang script ini:")
        print()
        print("    pip uninstall opencv-python opencv-python-headless -y")
        print("    pip install opencv-python")
        print("="*55 + "\n")
        print("    Melanjutkan tanpa tampilan video...")
        has_gui = False
    else:
        has_gui = True

    init_db(); init_csv()

    print(f"[MODEL] Memuat YOLO...")
    model = YOLO(MODEL_PATH)
    print("[MODEL] OK\n")

    print("[OCR] Memuat EasyOCR (tunggu 10-30 detik)...")
    ocr = easyocr.Reader(['id','en'], gpu=False)
    print("[OCR] OK\n")

    print("[KAMERA] Membuka webcam...")
    cap = cv2.VideoCapture(KAMERA_INDEX, cv2.CAP_DSHOW)  # CAP_DSHOW = lebih cepat di Windows
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)
    if not cap.isOpened():
        print("[ERROR] Webcam tidak bisa dibuka! Coba ganti KAMERA_INDEX = 1")
        return
    # Warm-up cepat supaya display muncul langsung
    for _ in range(5):
        cap.read()
    print("[KAMERA] OK — Webcam aktif\n")

    print(f"[SERIAL] Koneksi ke {SERIAL_PORT}...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        ser.reset_input_buffer()
        print(f"[SERIAL] OK\n")
    except serial.SerialException as e:
        print(f"[SERIAL ERROR] {e}")
        print("Port yang tersedia:")
        for p in serial.tools.list_ports.comports():
            print(f"  → {p.device}: {p.description}")
        cap.release()
        return

    # Start background threads
    threading.Thread(target=serial_listener,
                     args=(ser,), daemon=True, name="serial").start()
    threading.Thread(target=detection_processor,
                     args=(ser,ocr), daemon=True, name="detect").start()

    print("[SISTEM] Semua siap! Menunggu kendaraan...\n")

    # ── LOOP UTAMA ──
    while True:
        ret, frame = cap.read()
        if not ret:
            print("[ERROR] Frame gagal dibaca!")
            time.sleep(0.1)
            continue

        # YOLO inference
        results = model(frame, conf=CONF_THRESHOLD, verbose=False)
        r       = results[0]

        # Parse bounding boxes
        pakai = False; tanpa = False; plat_box = None; conf = 0.0
        for box in r.boxes:
            cls  = int(box.cls[0])
            conf_val = float(box.conf[0])
            if cls == CLASS_PAKAI_HELM and conf_val > conf:
                pakai = True; conf = conf_val
            if cls == CLASS_TANPA_HELM:
                tanpa = True
            if cls == CLASS_PLAT:
                if plat_box is None or conf_val > float(plat_box.conf[0]):
                    plat_box = box

        # Update shared state
        with sh.lock:
            sh.frame = frame.copy()
            sh.det   = {
                "pakai_helm": pakai and not tanpa,
                "plat_box":   plat_box,
                "conf":       conf
            }
            busy         = sh.busy
            b_teks       = sh.banner_teks
            b_warna      = sh.banner_warna
            b_until      = sh.banner_until

        # ── Tampilkan ──
        if has_gui:
            display = r.plot()

            # Status bar atas
            if busy:
                draw_status_bar(display, "SEDANG MEMPROSES RFID...", (0,200,255))
            elif tanpa:
                draw_status_bar(display, "TIDAK ADA HELM! Harap pakai helm.", (0,60,255))
            elif pakai:
                draw_status_bar(display, f"HELM TERDETEKSI ({conf:.0%})", (0,255,80))
            else:
                draw_status_bar(display, "Menunggu kendaraan... Tap kartu RFID", (180,180,180))

            # Banner hasil (tampil beberapa detik)
            if b_until > time.time():
                draw_result_banner(display, b_teks, b_warna)

            # Petunjuk keluar
            h,w = display.shape[:2]
            cv2.putText(display, "Tekan Q untuk keluar",
                        (w-240, h-14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100,100,100), 1, cv2.LINE_AA)

            try:
                cv2.imshow("Sistem Deteksi Helm", display)
            except Exception as e:
                print(f"[DISPLAY ERR] {e}")
                print("    Jalankan: pip uninstall opencv-python opencv-python-headless -y")
                print("    Lalu    : pip install opencv-python")
                has_gui = False   # matikan display, serial tetap jalan

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\n[SISTEM] Q ditekan. Keluar...")
                break
        else:
            # Tanpa GUI: print status ke terminal saja
            if pakai and not tanpa:
                print(f"\r[STATUS] HELM OK ({conf:.0%}) | Plat: {'ada' if plat_box else '-'}   ", end="")
            elif tanpa:
                print(f"\r[STATUS] TIDAK ADA HELM!                                              ", end="")
            else:
                print(f"\r[STATUS] Menunggu kendaraan...                                        ", end="")
            time.sleep(0.1)

    cap.release()
    cv2.destroyAllWindows()
    ser.close()
    print("\n[SISTEM] Sistem dimatikan.")


if __name__ == "__main__":
    main()