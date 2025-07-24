import time
from pymavlink import mavutil

try:
    from siyi_sdk import SIYISDK
except ImportError:
    print("Errore: impossibile importare siyi_sdk. Assicurati che sia installato.")
    exit(1)

class YawSmoothController:
    def __init__(self, connection_string="/dev/ttyAMA0", baud=57600, cam_ip="192.168.144.25", cam_port=37260):
        self.master = mavutil.mavlink_connection(connection_string, baud=baud)
        self.cam = SIYISDK(server_ip=cam_ip, port=cam_port)
        if not self.cam.connect():
            print("Errore: impossibile connettersi alla telecamera")
            exit(1)
        self.current_pitch = 0.0
        self.previous_yaw = None

    def map_rc_to_yaw(self, rc_value):
        # Clamp RC between 1000 and 2000
        rc_value = max(1000, min(2000, rc_value))
        # Map 1000-2000 → -90 to 90
        yaw = ((rc_value - 1000) / 1000) * 180 - 90
        return round(yaw, 1)

    def run(self):
        print("Controllo fluido YAW da RC channel...")
        while True:
            msg = self.master.recv_match(type='RC_CHANNELS', blocking=True, timeout=1)
            if msg:
                raw_value = msg.chan10_raw  # Cambia se usi altro canale

                if raw_value < 900 or raw_value > 2100:
                    continue

                yaw_deg = self.map_rc_to_yaw(raw_value)

                # Invia solo se cambia significativamente (evita spam)
                if self.previous_yaw is None or abs(yaw_deg - self.previous_yaw) > 0.5:
                    self.cam.setGimbalRotation(yaw_deg, self.current_pitch)
                    print(f"YAW: {yaw_deg:.1f}° (RC={raw_value})")
                    self.previous_yaw = yaw_deg

            time.sleep(0.02)  # molto fluido

if __name__ == "__main__":
    controller = YawSmoothController()
    controller.run()
