import time
import threading
from pymavlink import mavutil

try:
    from siyi_sdk import SIYISDK
except ImportError:
    print("Errore: impossibile importare siyi_sdk.")
    exit(1)

class GimbalController:
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

        # Gimbal - senza lock
        self.latest_yaw = 0
        self.latest_pitch = 0
        self.current_yaw = None
        self.current_pitch = None

        self.yaw_scale = 240 / 1000
        self.yaw_offset = -120 - (1000 * self.yaw_scale)
        self.pitch_scale = 115 / 1000
        self.pitch_offset = -90 - (1000 * self.pitch_scale)

        # Zoom
        self.zoom_in = False
        self.zoom_out = False

    def map_rc(self, rc_value, scale, offset):
        return rc_value * scale + offset

    # 🧠 RC Lettura (yaw/pitch + zoom)
    def rc_reader_thread(self):
        print("🎮 Thread RC Reader attivo...")
        while self.running:
            try:
                msg = self.master.recv_match(type='RC_CHANNELS', blocking=True, timeout=0.005)
                if msg:
                    rc_yaw = msg.chan12_raw
                    rc_pitch = msg.chan10_raw

                    if 900 < rc_yaw < 2100 and 900 < rc_pitch < 2100:
                        rc_yaw = max(1000, min(2000, rc_yaw))
                        rc_pitch = max(1000, min(2000, rc_pitch))

                        self.latest_yaw = self.map_rc(rc_yaw, self.yaw_scale, self.yaw_offset)
                        self.latest_pitch = self.map_rc(rc_pitch, self.pitch_scale, self.pitch_offset)

                    # Zoom
                    self.zoom_in = msg.chan8_raw > 1700
                    self.zoom_out = msg.chan11_raw > 1700
            except:
                time.sleep(0.001)

    # 🎯 Esecuzione Gimbal
    def gimbal_executor_thread(self):
        print("🎯 Thread Gimbal Executor attivo...")
        while self.running:

            try:
                self.cam.setGimbalRotation(self.latest_yaw, self.latest_pitch)


                print(f"🎯 Gimbal aggiornato → YAW: {self.latest_yaw:.2f}°, PITCH: {self.latest_pitch:.2f}°")
            except Exception as e:
                print(f"⚠️ Errore invio gimbal: {e}")
            time.sleep(0.003)

    # 🔍 Zoom Esecutore a scatti
    def zoom_executor_thread(self):
        print("🔍 Thread Zoom Executor attivo...")
        zoom_interval = 0.1
        last_zoom_time = 0
        prev_in = False
        prev_out = False

        while self.running:
            now = time.time()

            # Interrompi zoom se il tasto è stato appena rilasciato
            if prev_in and not self.zoom_in:
                self.cam.requestZoomHold()
                print("🛑 STOP Zoom IN")
            if prev_out and not self.zoom_out:
                self.cam.requestZoomHold()
                print("🛑 STOP Zoom OUT")

            # Zoom a scatti
            if self.zoom_in and not self.zoom_out and now - last_zoom_time >= zoom_interval:
                self.cam.requestZoomIn()
                print("🔍 Zoom IN")
                last_zoom_time = now

            elif self.zoom_out and not self.zoom_in and now - last_zoom_time >= zoom_interval:
                self.cam.requestZoomOut()
                print("🔎 Zoom OUT")
                last_zoom_time = now

            # Salva stato per confronti futuri
            prev_in = self.zoom_in
            prev_out = self.zoom_out

            time.sleep(0.003)

    def run(self):
        print("🚀 Avvio controller... [YAW: CH12 | PITCH: CH10 | ZOOM: CH8/CH11]")
        threading.Thread(target=self.rc_reader_thread, daemon=True).start()
        threading.Thread(target=self.gimbal_executor_thread, daemon=True).start()
        threading.Thread(target=self.zoom_executor_thread, daemon=True).start()

        try:
            while True:
                time.sleep(0.001)
        except KeyboardInterrupt:
            print("🛑 Interruzione ricevuta.")
            self.running = False
            time.sleep(0.1)
            print("✅ Terminato.")

if __name__ == "__main__":
    controller = GimbalController()
    controller.run()
