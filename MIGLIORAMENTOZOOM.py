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

        # gimbal vars con lock separati per massima performance
        self.target_yaw = 0
        self.target_pitch = 0
        self.current_yaw = 0
        self.current_pitch = 0
        
        self.target_lock = threading.Lock()
        self.current_lock = threading.Lock()
        self.target_updated = threading.Event()

        self.yaw_step = 5.0
        self.pitch_step = 5.0

        self.yaw_scale = 240 / 1000
        self.yaw_offset = -120 - (1000 * self.yaw_scale)
        self.pitch_scale = 115 / 1000
        self.pitch_offset = -90 - (1000 * self.pitch_scale)

        # Zoom flags
        self.zoom_in = False
        self.zoom_out = False

    def map_rc_fast(self, rc_value, scale, offset):
        return rc_value * scale + offset

    def quantize_angle(self, angle, step):
        return round(angle / step) * step

    # 🆕 THREAD 1: Lettura RC Zoom (CH8 e CH11)
    def zoom_rc_reader_thread(self):
        print("🎮 Thread Zoom RC Reader avviato...")
        while self.running:
            try:
                msg = self.master.recv_match(type='RC_CHANNELS', blocking=True, timeout=0.01)
                if msg:
                    self.zoom_in = msg.chan8_raw > 1700
                    self.zoom_out = msg.chan11_raw > 1700
            except:
                time.sleep(0.005)

    # 🆕 THREAD 2: Esecuzione dello Zoom (in / out / hold)
    def zoom_executor_thread(self):
        print("🔍 Thread Zoom Executor avviato...")
        zoom_interval = 0.5
        last_zoom_time = 0
        zooming = False

        while self.running:
            now = time.time()

            if self.zoom_in and now - last_zoom_time >= zoom_interval:
                self.cam.requestZoomIn()
                print("🔍 Zoom IN")
                last_zoom_time = now
                zooming = True

            elif self.zoom_out and now - last_zoom_time >= zoom_interval:
                self.cam.requestZoomOut()
                print("🔎 Zoom OUT")
                last_zoom_time = now
                zooming = True

            elif not self.zoom_in and not self.zoom_out and zooming:
                self.cam.requestZoomHold()
                print("⏸️ Zoom HOLD")
                zooming = False

            time.sleep(0.01)

    def rc_position_writer_thread(self):
        print("📝 Thread RC Position Writer avviato...")
        while self.running:
            try:
                msg = self.master.recv_match(type='RC_CHANNELS', blocking=True, timeout=0.01)
                if msg:
                    rc_yaw = msg.chan12_raw
                    rc_pitch = msg.chan10_raw

                    if 900 < rc_yaw < 2100 and 900 < rc_pitch < 2100:
                        rc_yaw = max(1000, min(2000, rc_yaw))
                        rc_pitch = max(1000, min(2000, rc_pitch))

                        new_yaw = self.quantize_angle(self.map_rc_fast(rc_yaw, self.yaw_scale, self.yaw_offset), self.yaw_step)
                        new_pitch = self.quantize_angle(self.map_rc_fast(rc_pitch, self.pitch_scale, self.pitch_offset), self.pitch_step)

                        with self.target_lock:
                            position_changed = (new_yaw != self.target_yaw or new_pitch != self.target_pitch)
                            if position_changed:
                                self.target_yaw = new_yaw
                                self.target_pitch = new_pitch
                                self.target_updated.set()
            except:
                time.sleep(0.001)

    def gimbal_executor_thread(self):
        print("🎯 Thread Gimbal Executor avviato...")
        
        while self.running:
            if self.target_updated.wait(timeout=0.01):
                self.target_updated.clear()
                
                with self.target_lock:
                    target_yaw = self.target_yaw
                    target_pitch = self.target_pitch
                
                with self.current_lock:
                    if target_yaw == self.current_yaw and target_pitch == self.current_pitch:
                        continue

                try:
                    self.cam.setGimbalRotation(target_yaw, target_pitch)
                    with self.current_lock:
                        self.current_yaw = target_yaw
                        self.current_pitch = target_pitch
                    print(f"🎯 YAW: {target_yaw:.0f}° | PITCH: {target_pitch:.0f}°")
                except Exception as e:
                    print(f"Errore controllo gimbal: {e}")
            else:
                continue

    def run(self):
        print("🚀 Controller attivo: YAW (CH12), PITCH (CH10), ZOOM IN (CH8), ZOOM OUT (CH11)")
        print("⚡ Sistema multithread per massima reattività")

        threading.Thread(target=self.zoom_rc_reader_thread, daemon=True).start()
        threading.Thread(target=self.zoom_executor_thread, daemon=True).start()
        threading.Thread(target=self.rc_position_writer_thread, daemon=True).start()
        threading.Thread(target=self.gimbal_executor_thread, daemon=True).start()

        try:
            while True:
                time.sleep(0.0001)
        except KeyboardInterrupt:
            print("\n🛑 Arresto in corso...")
            self.running = False
            self.target_updated.set()
            time.sleep(1)
            print("✅ Tutto fermato.")

if __name__ == "__main__":
    controller = QuantizedGimbalController()
    controller.run()
