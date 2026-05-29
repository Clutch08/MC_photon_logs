from datetime import datetime
from pathlib import Path
import csv
import time
import struct

import pyvisa
from ThorlabsPM100 import ThorlabsPM100

from pyftdi.ftdi import Ftdi
from pyftdi.serialext import serial_for_url


# =========================
# USER SETTINGS
# =========================

STAGE_SERIAL = "27504233"
PM100A_RESOURCE = "USB0::4883::32889::P1001982::0::INSTR"

WAVELENGTH_NM = 650

PMMA_THICKNESS = 5

timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
SAMPLE_NAME = f"{WAVELENGTH_NM}_{PMMA_THICKNESS}mm_pmma_{timestamp}"

START_DEG = 0
STOP_DEG = 20
STEP_DEG = 0.03

SAMPLES_PER_ANGLE = 21
SETTLE_TIME_S = 1.0
READ_DELAY_S = 0.1
NUM_SWEEPS = 1  # add to USER SETTINGS

# Empirically confirmed:
#   4.9999° → 9598  counts  →  9598  / 4.9999 = 1919.638
#   9.9998° → 19196 counts  →  19196 / 9.9998 = 1919.638
COUNTS_PER_DEG = 1919.638393

OUTPUT_FILE = Path.home() / "Desktop" / "Python" / f"{SAMPLE_NAME}_scan.csv"


# =========================
# KDC101 STAGE CLASS
# =========================

class KDC101Stage:
    def __init__(self, serial_number, counts_per_deg):
        Ftdi.add_custom_product(0x0403, 0xfaf0)

        url = f"ftdi://0x0403:0xfaf0:{serial_number}/1"

        self.ser = serial_for_url(
            url,
            baudrate=115200,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=1
        )

        self.counts_per_deg = counts_per_deg
        self.zero_counts = None

        print("KDC101 connected.")

    # --------------------------------------------------
    # Low-level APT protocol (from automata_photon_01.py)
    # --------------------------------------------------

    def read_position_counts(self):
        # MGMSG_MOT_REQ_POSCOUNTER = 0x0411
        self.ser.reset_input_buffer()
        self.ser.write(bytes([0x11, 0x04, 0x01, 0x00, 0x50, 0x01]))
        time.sleep(0.2)

        response = self.ser.read(100)

        if len(response) >= 12:
            return struct.unpack("<i", response[8:12])[0]

        raise RuntimeError(
            f"Could not read position. Raw response: {response.hex(' ')}"
        )

    def relative_move_counts(self, delta_counts):
        # MGMSG_MOT_MOVE_RELATIVE = 0x0448
        data = struct.pack("<Hi", 1, int(delta_counts))
        header = struct.pack("<HHBB", 0x0448, len(data), 0xD0, 0x01)
        self.ser.write(header + data)

    def close(self):
        self.ser.close()
        print("KDC101 closed.")

    # --------------------------------------------------
    # Optical zero coordinate system
    # --------------------------------------------------

    def set_current_position_as_zero(self):
        """
        Read the current encoder position and store it as the optical zero.
        Call this after physically aligning the detector with the laser beam.
        """
        self.zero_counts = self.read_position_counts()
        print(f"Optical zero set at {self.zero_counts} counts.")

    def counts_to_optical_deg(self, counts):
        if self.zero_counts is None:
            raise RuntimeError("Optical zero has not been set.")
        return (counts - self.zero_counts) / self.counts_per_deg

    def optical_deg_to_counts(self, angle_deg):
        if self.zero_counts is None:
            raise RuntimeError("Optical zero has not been set.")
        return int(round(self.zero_counts + angle_deg * self.counts_per_deg))

    # --------------------------------------------------
    # High-level movement
    # --------------------------------------------------

    def move_to_deg(self, target_deg):
        current_counts = self.read_position_counts()
        target_counts = self.optical_deg_to_counts(target_deg)
        delta_counts = target_counts - current_counts

        print(
            f"Moving to optical {target_deg:.4f}° "
            f"({target_counts} counts, delta {delta_counts} counts)"
        )

        self.relative_move_counts(delta_counts)

        # Simple time-based wait. Replace with status polling in a future version.
        move_time = max(1.0, abs(delta_counts) / 2000)
        time.sleep(move_time)

    def home(self):
        """
        Send MGMSG_MOT_MOVE_HOME to the KDC101.
        This triggers the same routine as 'Start Homing' on the cube menu —
        the controller drives to its limit switch and resets its internal zero.
        After homing, call set_current_position_as_zero() to re-sync optical zero.
        """
        # MGMSG_MOT_MOVE_HOME = 0x0443
        self.ser.write(bytes([0x43, 0x04, 0x01, 0x00, 0x50, 0x01]))
        print("Homing command sent. Waiting for stage to reach home position...")
        time.sleep(15)  # conservative wait — homing traverses the full range
        print("Homing complete.")


# =========================
# PM100A CLASS
# =========================

class PM100A:
    def __init__(self, resource_name, wavelength_nm):
        rm = pyvisa.ResourceManager("@py")
        self.inst = rm.open_resource(resource_name)
        self.pm = ThorlabsPM100(inst=self.inst)

        self.pm.sense.correction.wavelength = wavelength_nm

        print("PM100A connected.")
        print(f"Wavelength set to {wavelength_nm} nm.")

    def read_power(self):
        return float(self.pm.read)


# =========================
# MAIN SCAN
# =========================

def main():
    stage = KDC101Stage(STAGE_SERIAL, COUNTS_PER_DEG)
    pm = PM100A(PM100A_RESOURCE, WAVELENGTH_NM)

    # --- Pre-scan: ensure stage starts at true home ---
    # If the stage is already at 0 counts we can trust the position.
    # If not (e.g. script was interrupted mid-scan, or stage was moved manually),
    # home first so the encoder reference is clean before we set optical zero.
    startup_counts = stage.read_position_counts()
    if startup_counts != 0:
        print(f"Stage is at {startup_counts} counts (not home). Homing before scan...")
        stage.home()
    else:
        print("Stage is at home position (0 counts). No pre-scan homing needed.")

    # Now the encoder is at mechanical home — define this as optical zero.
    stage.set_current_position_as_zero()

    n_steps = int(round((STOP_DEG - START_DEG) / STEP_DEG)) + 1
    angles = [START_DEG + i * STEP_DEG for i in range(n_steps)]

    print(f"\nSaving to: {OUTPUT_FILE}")
    print(f"Angles: {START_DEG}° → {STOP_DEG}° in {STEP_DEG}° steps ({len(angles)} points)\n")

    # Build column headers: one row per angle, samples as individual columns
    sample_cols = [f"sample_{i+1}" for i in range(SAMPLES_PER_ANGLE)]
    header = [
        "sweep",
        "timestamp",
        "target_angle_deg",
        "actual_angle_deg",
        "position_counts",
        *sample_cols,
        "mean_W",
        "std_W",
        "mean_uW",
    ]

    with open(OUTPUT_FILE, "w", newline="") as f:
        writer = csv.writer(f)

        # --- Metadata block at top of file ---
        writer.writerow(["# sample_name", SAMPLE_NAME])
        writer.writerow(["# wavelength_nm", WAVELENGTH_NM])
        writer.writerow(["# start_deg", START_DEG])
        writer.writerow(["# stop_deg", STOP_DEG])
        writer.writerow(["# step_deg", STEP_DEG])
        writer.writerow(["# samples_per_angle", SAMPLES_PER_ANGLE])
        writer.writerow(["# settle_time_s", SETTLE_TIME_S])
        writer.writerow(["# read_delay_s", READ_DELAY_S])
        writer.writerow(["# num_sweeps", NUM_SWEEPS])
        writer.writerow(["# scan_started", datetime.now().isoformat()])
        writer.writerow([])  # blank separator
        writer.writerow(header)

        for sweep in range(NUM_SWEEPS):
            print(f"\n--- Sweep {sweep + 1} of {NUM_SWEEPS} ---")
            for angle in angles:
                stage.move_to_deg(angle)
                time.sleep(SETTLE_TIME_S)

                actual_counts = stage.read_position_counts()
                actual_angle = stage.counts_to_optical_deg(actual_counts)

                readings = []
                for i in range(SAMPLES_PER_ANGLE):
                    power = pm.read_power()
                    readings.append(power)
                    time.sleep(READ_DELAY_S)

                mean_W = sum(readings) / len(readings)
                variance = sum((x - mean_W) ** 2 for x in readings) / len(readings)
                std_W = variance ** 0.5
                mean_uW = mean_W * 1e6

                writer.writerow([
                    sweep + 1,
                    datetime.now().isoformat(),
                    f"{angle:.6f}",
                    f"{actual_angle:.6f}",
                    actual_counts,
                    *readings,
                    mean_W,
                    std_W,
                    mean_uW,
                ])

                print(
                    f"Sweep {sweep+1} | {angle:>7.4f}° target | "
                    f"{actual_angle:>7.4f}° actual | "
                    f"Mean = {mean_W:.6e} W  Std = {std_W:.2e} W"
                )

            stage.home()
            stage.set_current_position_as_zero()  # re-zero between sweeps

    stage.close()

    print("\nScan complete.")
    print(f"Saved CSV: {OUTPUT_FILE}")


if __name__ == "__main__":
    main()
