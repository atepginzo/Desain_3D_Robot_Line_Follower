"""Line Follower Controller (Kendali PD Teroptimasi untuk Robot 2 Sensor IR).

Menggunakan kontroler Proportional-Derivative (PD) untuk mengarahkan robot
mengikuti lintasan garis hitam dengan transisi kecepatan yang halus serta
kemampuan recovery saat menghadapi tikungan tajam.
"""

from typing import Tuple
from controller import Robot
import config


def clamp(value: float, min_val: float, max_val: float) -> float:
    """Membatasi nilai dalam rentang batas bawah dan batas atas."""
    return max(min_val, min(max_val, value))


class LineFollowerRobot:
    """Kelas pengontrol navigasi robot line follower otonom."""

    def __init__(self) -> None:
        self.robot = Robot()
        self.time_step = config.TIME_STEP

        # Inisialisasi aktuator motor
        self.left_motor = self.robot.getDevice("left_motor")
        self.right_motor = self.robot.getDevice("right_motor")
        self._init_motors()

        # Inisialisasi sensor infra-red (IR)
        self.ir_left = self.robot.getDevice("ir_left")
        self.ir_right = self.robot.getDevice("ir_right")
        self._init_sensors()

        # State tracking kendali
        self.prev_error: float = 0.0
        self.last_turn_dir: int = 0  # -1 = belok kiri, 1 = belok kanan
        self.integral_error: float = 0.0

    def _init_motors(self) -> None:
        """Mengatur mode motor ke mode kecepatan berkelanjutan."""
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

    def _init_sensors(self) -> None:
        """Mengaktifkan sensor IR dengan interval sampling TIME_STEP."""
        self.ir_left.enable(self.time_step)
        self.ir_right.enable(self.time_step)

    def read_sensors(self) -> Tuple[float, float]:
        """Membaca nilai reflektansi dari sensor IR kiri dan kanan."""
        return self.ir_left.getValue(), self.ir_right.getValue()

    def compute_pid(self, val_left: float, val_right: float) -> float:
        """Menghitung nilai koreksi kemudi menggunakan algoritma PD."""
        # Nilai error positif: sensor kanan menyentuh garis gelap -> robot harus belok kanan
        # Nilai error negatif: sensor kiri menyentuh garis gelap -> robot harus belok kiri
        error = val_left - val_right

        if abs(error) < config.ERROR_DEADBAND:
            error = 0.0

        derivative = error - self.prev_error
        self.integral_error = clamp(self.integral_error + error, -200.0, 200.0)

        correction = (
            config.KP * error +
            config.KD * derivative +
            config.KI * self.integral_error
        )

        self.prev_error = error

        # Catat memori arah belokan terakhir untuk antisipasi kehilangan garis
        if error < -30:
            self.last_turn_dir = -1
        elif error > 30:
            self.last_turn_dir = 1

        return correction

    def calculate_motor_speeds(
        self, val_left: float, val_right: float
    ) -> Tuple[float, float]:
        """Menentukan kecepatan motor kiri dan kanan berdasarkan posisi sensor."""
        # Kondisi khusus: kedua sensor mendeteksi garis hitam (garis henti / tikungan tajam)
        if (
            val_left < config.THRESHOLD_BLACK
            and val_right < config.THRESHOLD_BLACK
        ):
            if self.last_turn_dir <= 0:
                return config.REVERSE_SPEED, config.BASE_SPEED
            else:
                return config.BASE_SPEED, config.REVERSE_SPEED

        # Kondisi kendali proporsional normal
        correction = self.compute_pid(val_left, val_right)
        left_speed = clamp(
            config.BASE_SPEED + correction, config.MIN_SPEED, config.MAX_SPEED
        )
        right_speed = clamp(
            config.BASE_SPEED - correction, config.MIN_SPEED, config.MAX_SPEED
        )

        return left_speed, right_speed

    def run(self) -> None:
        """Loop eksekusi utama pengontrol robot."""
        while self.robot.step(self.time_step) != -1:
            val_left, val_right = self.read_sensors()
            left_speed, right_speed = self.calculate_motor_speeds(
                val_left, val_right
            )

            if config.DEBUG_MODE:
                print(
                    f"IR [L:{val_left:.1f}, R:{val_right:.1f}] -> "
                    f"Speed [L:{left_speed:.2f}, R:{right_speed:.2f}]"
                )

            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)


def main() -> None:
    controller = LineFollowerRobot()
    controller.run()


if __name__ == "__main__":
    main()
