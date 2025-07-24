import time
import threading
from pymavlink import mavutil

try:
    from siyi_sdk import SIYISDK
except ImportError:
    print("Errore: impossibile importare siyi_sdk.")
    exit(1)

class QuantizedGimbalController:
    def __init__(self, connection="/dev/ttyAMA0", baud=57600, cam_ip="192.168.144.25", cam_port=37260):
        self.master = mavutil.mavlink_connection(connection, baud=baud)
        self.master.wait_heartbeat()
        print("✅ Heartbeat ricevuto.")

        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
            20,
            1
        )

        self.cam = SIYISDK(server_ip=cam_ip, port=cam_port)
        if not self.cam.connect():
            print("Errore: impossibile connettersi alla telecamera")
            exit(1)

        self.running = True
        self.new_data = False
        self.prev_zoom_level = 0

        # gimbal vars
        self.prev_yaw = None
        self.prev_pitch = None
        self.latest_yaw = 0
        self.latest_pitch = 0
        self.data_lock = threading.Lock()

        self.yaw_step = 5.0
        self.pitch_step = 5.0

        self.yaw_scale = 240 / 1000
        self.yaw_offset = -120 - (1000 * self.yaw_scale)
        self.pitch_scale = 115 / 1000
        self.pitch_offset = -90 - (1000 * self.pitch_scale)

        # Photo button state
        self.prev_photo_button = 0

    def map_rc_fast(self, rc_value, scale, offset):
        return rc_value * scale + offset

    def quantize_angle(self, angle, step):
        return round(angle / step) * step

    def get_zoom_level_from_rc(self, rc_value):
        if rc_value < 1200:
            return 0
        elif rc_value < 1700:
            return 1
        else:
            return 2

    def apply_zoom(self, desired_level):
        if desired_level == self.prev_zoom_level:
            return

        steps = abs(desired_level - self.prev_zoom_level)
        direction = "in" if desired_level > self.prev_zoom_level else "out"

        for _ in range(steps):
            if direction == "in":
                self.cam.requestZoomIn()
            else:
                self.cam.requestZoomOut()
            time.sleep(1)

        self.cam.requestZoomHold()
        print(f"🔍 Zoom aggiornato: {desired_level * 50}%")
        self.prev_zoom_level = desired_level

    def zoom_control_thread(self):
        print("🔍 Thread zoom avviato...")
        while self.running:
            try:
                msg = self.master.recv_match(type='RC_CHANNELS', blocking=True, timeout=0.05)
                if msg:
                    rc_zoom = msg.chan8_raw
                    zoom_level = self.get_zoom_level_from_rc(rc_zoom)
                    self.apply_zoom(zoom_level)
            except:
                time.sleep(0.1)

    def handle_photo_button(self, photo_btn):
        photo_state = 1 if photo_btn > 1500 else 0  # soglia semplice

        # Rileva fronte di salita (0->1)
        if photo_state == 1 and self.prev_photo_button == 0:
            print("📸 Scatto foto")
            try:
                self.cam.requestPhoto()
            except Exception as e:
                print(f"Errore nello scatto foto: {e}")

        self.prev_photo_button = photo_state

    def rc_reader_thread(self):
        print("🎮 Thread RC YAW/PITCH/FOTO avviato...")
        while self.running:
            try:
                msg = self.master.recv_match(type='RC_CHANNELS', blocking=True, timeout=0.05)
                if msg:
                    rc_yaw = msg.chan12_raw
                    rc_pitch = msg.chan10_raw
                    rc_photo_btn = msg.chan9_raw

                    if 900 < rc_yaw < 2100 and 900 < rc_pitch < 2100:
                        rc_yaw = max(1000, min(2000, rc_yaw))
                        rc_pitch = max(1000, min(2000, rc_pitch))

                        yaw = self.quantize_angle(self.map_rc_fast(rc_yaw, self.yaw_scale, self.yaw_offset), self.yaw_step)
                        pitch = self.quantize_angle(self.map_rc_fast(rc_pitch, self.pitch_scale, self.pitch_offset), self.pitch_step)

                        # Gestione bottone foto
                        self.handle_photo_button(rc_photo_btn)

                        with self.data_lock:
                            if yaw != self.latest_yaw or pitch != self.latest_pitch:
                                self.latest_yaw = yaw
                                self.latest_pitch = pitch
                                self.new_data = True
            except:
                time.sleep(0.05)

    def gimbal_control_thread(self):
        print("🎯 Thread gimbal avviato...")
        while self.running:
            with self.data_lock:
                if self.new_data:
                    yaw = self.latest_yaw
                    pitch = self.latest_pitch
                    self.new_data = False
                else:
                    time.sleep(0.005)
                    continue
            try:
                self.cam.setGimbalRotation(yaw, pitch)
                print(f"YAW: {yaw:.0f}° | PITCH: {pitch:.0f}°")
            except Exception as e:
                print(f"Errore controllo gimbal: {e}")
                time.sleep(0.01)

    def run(self):
        print("🚀 Controller attivo: YAW (CH12), PITCH (CH10), ZOOM (CH8), FOTO (CH9)")

        threading.Thread(target=self.rc_reader_thread, daemon=True).start()
        threading.Thread(target=self.gimbal_control_thread, daemon=True).start()
        threading.Thread(target=self.zoom_control_thread, daemon=True).start()

        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n🛑 Arresto in corso...")
            self.running = False
            time.sleep(1)
            print("✅ Tutto fermato.")

if __name__ == "__main__":
    controller = QuantizedGimbalController()
    controller.run()
