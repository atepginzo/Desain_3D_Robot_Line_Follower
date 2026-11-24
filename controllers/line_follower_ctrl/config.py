"""Konfigurasi parameter Robot Line Follower Webots.

File ini memusatkan seluruh parameter kendali, kecepatan motor, dan batas
ambang pembacaan sensor agar mudah disetel (tuning) tanpa perlu memodifikasi
logika utama controller.
"""

# ==============================================================================
# PARAMETER SIMULASI WEBOTS
# ==============================================================================
TIME_STEP: int = 16  # Durasi siklus per langkah simulasi (ms)

# ==============================================================================
# PARAMETER KECEPATAN MOTOR (rad/s)
# ==============================================================================
BASE_SPEED: float = 4.5       # Kecepatan linier normal saat bergerak lurus
MAX_SPEED: float = 6.8        # Kecepatan putar maksimum motor
MIN_SPEED: float = -1.5       # Kecepatan minimum saat counter-steering
REVERSE_SPEED: float = -0.5   # Kecepatan mundur satu roda saat tikungan tajam / garis melintang

# ==============================================================================
# PARAMETER KENDALI PD (PROPORTIONAL - DERIVATIVE)
# ==============================================================================
KP: float = 0.015             # Penguatan proporsional untuk koreksi arah
KD: float = 0.011             # Penguatan derivatif untuk meredam osilasi
KI: float = 0.000             # Penguatan integral (opsional / pelindung offset)

# ==============================================================================
# PARAMETER SENSOR INFRARED (IR) TCRT5000
# ==============================================================================
# Nilai pantulan sensor IR pada Webots:
# ~1000 = permukaan putih / reflektif
# ~758  = garis hitam pekat
THRESHOLD_BLACK: float = 880.0  # Batas ambang deteksi garis hitam
ERROR_DEADBAND: float = 3.0     # Ambang batas toleransi perbedaan bacaan sensor

# ==============================================================================
# PENGATURAN TELEMETRI & DEBUGGING
# ==============================================================================
DEBUG_MODE: bool = False       # True untuk mencetak pembacaan sensor dan aksi motor
