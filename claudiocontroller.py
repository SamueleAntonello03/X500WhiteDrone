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

        # gimbal vars con lock separati per massima performance
        self.target_yaw = 0
        self.target_pitch = 0
        self.current_yaw = 0
        self.current_pitch = 0

        self.target_lock = threading.Lock()
        self.current_lock = threading.Lock()
        self.target_updated = threading.Event()

        self.yaw_scale = 240 / 1000
        self.yaw_offset = -120 - (1000 * self.yaw_scale)
        self.pitch_scale = 115 / 1000
        self.pitch_offset = -90 - (1000 * self.pitch_scale)

        # Zoom flags
        self.zoom_in = False
        self.zoom_out = False

    def map_rc_fast(self, rc_value, scale, offset):
        return rc_value * scale + offset

    # THREAD 1: Lettura RC Zoom OTTIMIZZATA (timeout ridotto)
    def zoom_rc_reader_thread(self):
        print("🎮 Thread Zoom RC Reader ULTRA-REATTIVO avviato...")
        while self.running:
            try:
                # TIMEOUT RIDOTTO per reattività
                msg = self.master.recv_match(type='RC_CHANNELS', blocking=True, timeout=0.005)
                if msg:
                    self.zoom_in = msg.chan8_raw > 1700
                    self.zoom_out = msg.chan11_raw > 1700
            except:
                time.sleep(0.001)  # Pausa ridotta

    # THREAD 2: Zoom Executor REATTIVO - rileva subito quando molli
    def zoom_executor_thread(self):
        print("🔍 Thread Zoom Executor REATTIVO avviato...")
        zooming_state = None
        zoom_interval = 0.08  # Intervallo ancora più ridotto
        last_zoom_time = 0
        
        # VARIABILI PER RILEVARE CAMBIO STATO
        prev_zoom_in = False
        prev_zoom_out = False

        while self.running:
            now = time.time()
            
            # PRIORITÀ ASSOLUTA: RILEVA QUANDO MOLLI IL TASTO
            if prev_zoom_in and not self.zoom_in:
                print("🛑 Zoom IN MOLLATO - STOP immediato")
                self.cam.requestZoomHold()
                zooming_state = None
                prev_zoom_in = self.zoom_in  # Aggiorna subito
                continue  # SALTA tutto il resto del loop
                
            if prev_zoom_out and not self.zoom_out:
                print("🛑 Zoom OUT MOLLATO - STOP immediato")
                self.cam.requestZoomHold()
                zooming_state = None
                prev_zoom_out = self.zoom_out  # Aggiorna subito
                continue  # SALTA tutto il resto del loop

            # LOGICA ZOOM NORMALE (solo se non hai appena mollato)
            if self.zoom_in and not self.zoom_out:
                if now - last_zoom_time >= zoom_interval:
                    self.cam.requestZoomIn()
                    print("🔍 Zoom IN")
                    last_zoom_time = now
                zooming_state = "in"

            elif self.zoom_out and not self.zoom_in:
                if now - last_zoom_time >= zoom_interval:
                    self.cam.requestZoomOut()
                    print("🔎 Zoom OUT")
                    last_zoom_time = now
                zooming_state = "out"

            elif not self.zoom_in and not self.zoom_out:
                if zooming_state is not None:
                    self.cam.requestZoomHold()
                    print("⏸️ Zoom HOLD")
                    zooming_state = None

            # SALVA STATO PRECEDENTE per rilevare cambi
            prev_zoom_in = self.zoom_in
            prev_zoom_out = self.zoom_out

            time.sleep(0.003)  # Check ancora più frequente 

    # THREAD 3: RC Position Writer Ultra-Reattivo
    def rc_position_writer_thread(self):
        print("📝 Thread RC Position Writer OTTIMIZZATO avviato...")
        while self.running:
            try:
                msg = self.master.recv_match(type='RC_CHANNELS', blocking=True, timeout=0.001)
                if msg:
                    rc_yaw = msg.chan12_raw
                    rc_pitch = msg.chan10_raw

                    if 900 < rc_yaw < 2100 and 900 < rc_pitch < 2100:
                        rc_yaw = max(1000, min(2000, rc_yaw))
                        rc_pitch = max(1000, min(2000, rc_pitch))

                        new_yaw = self.map_rc_fast(rc_yaw, self.yaw_scale, self.yaw_offset)
                        new_pitch = self.map_rc_fast(rc_pitch, self.pitch_scale, self.pitch_offset)

                        if self.target_lock.acquire(blocking=False):
                            try:
                                if new_yaw != self.target_yaw or new_pitch != self.target_pitch:
                                    self.target_yaw = new_yaw
                                    self.target_pitch = new_pitch
                                    self.target_updated.set()
                            finally:
                                self.target_lock.release()
            except:
                time.sleep(0.0001)

    # THREAD 4: Gimbal Executor con Resilienza
    def gimbal_executor_thread(self):
        print("🎯 Thread Gimbal Executor OTTIMIZZATO avviato...")
        consecutive_errors = 0
        max_errors = 5

        while self.running:
            if self.target_updated.wait(timeout=0.001):
                self.target_updated.clear()

                target_acquired = False
                if self.target_lock.acquire(blocking=False):
                    try:
                        target_yaw = self.target_yaw
                        target_pitch = self.target_pitch
                        target_acquired = True
                    finally:
                        self.target_lock.release()

                if not target_acquired:
                    continue

                needs_update = True
                if self.current_lock.acquire(blocking=False):
                    try:
                        needs_update = (
                            target_yaw != self.current_yaw or
                            target_pitch != self.current_pitch
                        )
                    finally:
                        self.current_lock.release()

                if needs_update:
                    try:
                        self.cam.setGimbalRotation(target_yaw, target_pitch)

                        if self.current_lock.acquire(blocking=False):
                            try:
                                self.current_yaw = target_yaw
                                self.current_pitch = target_pitch
                            finally:
                                self.current_lock.release()

                        print(f"🎯 YAW: {target_yaw:.2f}° | PITCH: {target_pitch:.2f}°")
                        consecutive_errors = 0
                    except Exception as e:
                        consecutive_errors += 1
                        print(f"⚠️ Errore gimbal #{consecutive_errors}: {e}")
                        if consecutive_errors >= max_errors:
                            print("🚨 Troppi errori gimbal, pausa safety 100ms...")
                            time.sleep(0.1)
                            consecutive_errors = 0

    def run(self):
        print("🚀 Controller OTTIMIZZATO attivo: YAW (CH12), PITCH (CH10), ZOOM IN (CH8), ZOOM OUT (CH11)")
        print("🚁 Sistema dual-thread ottimizzato per applicazioni drone")
        print("🎯 Precisione CONTINUA: movimento fluido senza step discreti")

        threading.Thread(target=self.zoom_rc_reader_thread, daemon=True).start()
        threading.Thread(target=self.zoom_executor_thread, daemon=True).start()
        threading.Thread(target=self.rc_position_writer_thread, daemon=True).start()
        threading.Thread(target=self.gimbal_executor_thread, daemon=True).start()

        try:
            while True:
                time.sleep(0.0001)
        except KeyboardInterrupt:
            print("\n🛑 Arresto controller...")
            self.running = False
            self.target_updated.set()
            time.sleep(0.1)
            print("✅ Controller fermato.")

if __name__ == "__main__":
    controller = GimbalController()
    controller.run()
