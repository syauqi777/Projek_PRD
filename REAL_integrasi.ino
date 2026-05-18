/*
 =============================================================================
   SISTEM HELM GATE — ESP32 SKETCH (Berdasarkan REAL.ino yang sudah jalan)

   PERUBAHAN dari REAL.ino asli:
     + Ultrasonik sebagai trigger PERTAMA (sebelum RFID, sesuai workflow)
     + Kirim "RFID:<UID>" ke PC saat kartu ditap (untuk logging)
     + Terima "HELM_OK" / "NO_HELM" dari PC (protokol tetap sama)
     + Tambah BUZZER untuk indikator akses ditolak
     + State machine ringan (tanpa blocking while)

   PIN (persis sama dengan REAL.ino kamu):
     SERVO  → GPIO 15   (tidak berubah)
     SS     → GPIO 5    (tidak berubah)
     RST    → GPIO 22   (tidak berubah)
     TRIG   → GPIO 12   (tidak berubah)
     ECHO   → GPIO 14   (tidak berubah)
     BUZZER → GPIO 26   (BARU — tambah kabel ke buzzer)

   LIBRARY yang dibutuhkan (sudah kamu install):
     - MFRC522
     - ESP32Servo
 =============================================================================
*/

#include <SPI.h>
#include <MFRC522.h>
#include <ESP32Servo.h>

// ── PIN — sama persis dengan REAL.ino kamu ──
#define SS_PIN    5
#define RST_PIN   22
#define SERVO_PIN 15
#define TRIG_PIN  12
#define ECHO_PIN  14
#define BUZZ_PIN  26   // ← BARU: sambungkan buzzer ke GPIO 26

// ── Objek hardware ──
MFRC522 rfid(SS_PIN, RST_PIN);
Servo   portalServo;

// ── Konfigurasi sistem ──
const int   JARAK_DETEKSI_CM  = 100;    // jarak ultrasonik deteksi kendaraan (cm)
const int   JARAK_LEWAT_CM    = 100;   // jarak ultrasonik untuk nutup portal (sama REAL.ino)
const int   TIMEOUT_PC_MS     = 15000; // tunggu Python max 15 detik
const int   TIMEOUT_RFID_MS   = 10000; // tunggu RFID tap max 10 detik
const int   DEBOUNCE_MS       = 1500;  // jeda antar baca kartu

// ── State Machine ──
enum State {
  IDLE,             // scan ultrasonik, tunggu kendaraan
  TUNGGU_RFID,      // kendaraan terdeteksi, tunggu tap kartu
  TUNGGU_PC,        // RFID ditap, tunggu respons Python
  BUKA_PORTAL,      // HELM_OK diterima, portal terbuka
  TOLAK_AKSES,      // NO_HELM / timeout, akses ditolak
  COOLDOWN          // jeda 2 detik sebelum kembali IDLE
};

State  state         = IDLE;
unsigned long timerMulai = 0;
String bufferSerial  = "";

// ─────────────────────────────────────────
//  FUNGSI ULTRASONIK (sama persis REAL.ino)
// ─────────────────────────────────────────
long bacaJarak() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long durasi = pulseIn(ECHO_PIN, HIGH, 30000UL); // timeout 30ms
  if (durasi == 0) return 999;
  return durasi * 0.034 / 2;
}

// ─────────────────────────────────────────
//  FUNGSI RFID (sama persis REAL.ino)
// ─────────────────────────────────────────
String bacaUID() {
  if (!rfid.PICC_IsNewCardPresent()) return "";
  if (!rfid.PICC_ReadCardSerial())  return "";

  String uid = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) uid += "0";
    uid += String(rfid.uid.uidByte[i], HEX);
  }
  uid.toUpperCase();

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  return uid;
}

// ─────────────────────────────────────────
//  FUNGSI BACA SERIAL (non-blocking)
// ─────────────────────────────────────────
// Menggantikan Serial.readStringUntil() yang blocking di REAL.ino
bool bacaSerial(String &output) {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      output = bufferSerial;
      output.trim();
      bufferSerial = "";
      return true;
    } else if (c != '\r') {
      bufferSerial += c;
    }
  }
  return false;
}

// ─────────────────────────────────────────
//  FUNGSI BUZZER
// ─────────────────────────────────────────
void buzzerOK() {
  // 2 beep pendek = akses diterima
  for (int i = 0; i < 2; i++) {
    digitalWrite(BUZZ_PIN, HIGH); delay(150);
    digitalWrite(BUZZ_PIN, LOW);  delay(100);
  }
}

void buzzerGagal() {
  // 1 beep panjang = akses ditolak
  digitalWrite(BUZZ_PIN, HIGH); delay(700);
  digitalWrite(BUZZ_PIN, LOW);
}

// ─────────────────────────────────────────
//  TRANSISI STATE
// ─────────────────────────────────────────
void pindahState(State baru) {
  state      = baru;
  timerMulai = millis();
}

// ─────────────────────────────────────────
//  SETUP (sama dengan REAL.ino + pin buzzer)
// ─────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  SPI.begin();
  rfid.PCD_Init();

  portalServo.attach(SERVO_PIN);
  portalServo.write(0);  // portal tertutup

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZ_PIN, OUTPUT);  // BARU
  digitalWrite(BUZZ_PIN, LOW);

  while (Serial.available()) Serial.read();  // bersihkan buffer

  Serial.println("==================================");
  Serial.println("Sistem Parkir Otomatis AI Camera");
  Serial.println("Status: SIAP!");
  Serial.println("Menunggu kendaraan mendekat...");
  Serial.println("==================================");

  pindahState(IDLE);
}

// ─────────────────────────────────────────
//  LOOP UTAMA — STATE MACHINE
// ─────────────────────────────────────────
void loop() {
  unsigned long sekarang = millis();
  String pesanPC = "";
  bool adaPesanPC = bacaSerial(pesanPC);

  // ───────────────────────────────────────
  //  IDLE: scan ultrasonik, tunggu kendaraan
  // ───────────────────────────────────────
  if (state == IDLE) {
    long jarak = bacaJarak();
    if (jarak < JARAK_DETEKSI_CM) {
      Serial.println("\n[1] Kendaraan terdeteksi! Silakan tap kartu...");
      pindahState(TUNGGU_RFID);
    }
  }

  // ───────────────────────────────────────
  //  TUNGGU_RFID: tunggu tap kartu RFID
  // ───────────────────────────────────────
  else if (state == TUNGGU_RFID) {
    if (sekarang - timerMulai > TIMEOUT_RFID_MS) {
      Serial.println("[!] Timeout tap kartu. Sistem reset.");
      pindahState(COOLDOWN);
      return;
    }

    String uid = bacaUID();
    if (uid.length() > 0) {
      Serial.println("[1] Kartu terdeteksi! Memproses...");
      delay(500);
      Serial.println("[1] Pembayaran Rp 5000 Berhasil.");

      // Kirim UID ke Python untuk logging
      // Format: "RFID:<uid>" → Python catat di CSV/DB
      Serial.println("RFID:" + uid);

      Serial.println("[2] Menunggu verifikasi helm dari AI Kamera...");
      pindahState(TUNGGU_PC);
      delay(DEBOUNCE_MS);
    }
  }

  // ───────────────────────────────────────
  //  TUNGGU_PC: tunggu respons Python
  //  Protokol sama persis REAL.ino: HELM_OK / NO_HELM
  // ───────────────────────────────────────
  else if (state == TUNGGU_PC) {
    if (sekarang - timerMulai > TIMEOUT_PC_MS) {
      Serial.println("[!] ERROR: Kamera tidak merespons (Timeout). Portal tetap ditutup.");
      buzzerGagal();
      pindahState(COOLDOWN);
      return;
    }

    if (adaPesanPC) {
      // Tampilkan semua pesan masuk (sama dengan REAL.ino asli)
      Serial.print("[PC] Diterima: ");
      Serial.println(pesanPC);

      if (pesanPC == "HELM_OK") {
        Serial.println("[3] Helm Terdeteksi! Portal Dibuka...");
        buzzerOK();
        portalServo.write(90);   // buka portal (sama REAL.ino)
        pindahState(BUKA_PORTAL);

      } else if (pesanPC == "NO_HELM") {
        Serial.println("[!] AKSES DITOLAK: Pengendara tidak menggunakan helm!");
        Serial.println("    Sistem direset. Portal tetap ditutup.");
        buzzerGagal();
        pindahState(TOLAK_AKSES);
      }
    }
  }

  // ───────────────────────────────────────
  //  BUKA_PORTAL: tunggu kendaraan lewat
  //  (sama persis logika REAL.ino — ultrasonik jarak > 100cm)
  // ───────────────────────────────────────
  else if (state == BUKA_PORTAL) {
    long jarak = bacaJarak();
    if (jarak >= JARAK_LEWAT_CM) {
      delay(1000);               // jeda 1 detik (sama REAL.ino)
      portalServo.write(0);      // tutup portal
      Serial.println("Kendaraan telah lewat. Portal ditutup kembali.");
      pindahState(COOLDOWN);
    }
  }

  // ───────────────────────────────────────
  //  TOLAK_AKSES: tampilkan pesan 2 detik lalu reset
  // ───────────────────────────────────────
  else if (state == TOLAK_AKSES) {
    if (sekarang - timerMulai > 2000) {
      pindahState(COOLDOWN);
    }
  }

  // ───────────────────────────────────────
  //  COOLDOWN: jeda sebelum kembali ke IDLE
  // ───────────────────────────────────────
  else if (state == COOLDOWN) {
    if (sekarang - timerMulai > 2000) {
      Serial.println("\n--- Menunggu kendaraan selanjutnya ---");
      pindahState(IDLE);
    }
  }

  delay(50);  // ~20Hz loop
}
