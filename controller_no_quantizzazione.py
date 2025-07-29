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

        # Quantizzazione rimossa per precisione massima
        # self.yaw_step = 5.0
        # self.pitch_step = 5.0

        self.yaw_scale = 240 / 1000
        self.yaw_offset = -120 - (1000 * self.yaw_scale)
        self.pitch_scale = 115 / 1000
        self.pitch_offset = -90 - (1000 * self.pitch_scale)

        # Zoom flags
        self.zoom_in = False
        self.zoom_out = False

    def map_rc_fast(self, rc_value, scale, offset):
        return rc_value * scale + offset

    # Quantizzazione rimossa per precisione continua
    # def quantize_angle(self, angle, step):
    #     return round(angle / step) * step

    # THREAD 1: Lettura RC Zoom (CH8 e CH11)
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

    # THREAD 2: Esecuzione dello Zoom (in / out / hold)
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

    # 🚁 THREAD 3 OTTIMIZZATO: RC Position Writer Ultra-Reattivo
    def rc_position_writer_thread(self):
        """Thread RC ottimizzato per droni - massima reattività pilota"""
        print("📝 Thread RC Position Writer OTTIMIZZATO avviato...")
        
        while self.running:
            try:
                # 🚀 TIMEOUT RIDOTTO: 1ms per reattività massima
                msg = self.master.recv_match(type='RC_CHANNELS', blocking=True, timeout=0.001)
                if msg:
                    rc_yaw = msg.chan12_raw
                    rc_pitch = msg.chan10_raw

                    if 900 < rc_yaw < 2100 and 900 < rc_pitch < 2100:
                        rc_yaw = max(1000, min(2000, rc_yaw))
                        rc_pitch = max(1000, min(2000, rc_pitch))

                        # 🎯 PRECISIONE CONTINUA: calcolo diretto senza quantizzazione
                        new_yaw = self.map_rc_fast(rc_yaw, self.yaw_scale, self.yaw_offset)
                        new_pitch = self.map_rc_fast(rc_pitch, self.pitch_scale, self.pitch_offset)

                        # 🚀 NON-BLOCKING LOCK: non aspetta mai, priorità al pilota
                        if self.target_lock.acquire(blocking=False):
                            try:
                                position_changed = (new_yaw != self.target_yaw or new_pitch != self.target_pitch)
                                if position_changed:
                                    self.target_yaw = new_yaw
                                    self.target_pitch = new_pitch
                                    self.target_updated.set()
                            finally:
                                self.target_lock.release()
                        # Se il lock è occupato, salta questo update - il pilota non aspetta mai
                        
            except:
                # 🚀 RECOVERY ULTRA-VELOCE: 0.1ms
                time.sleep(0.0001)

    # 🚁 THREAD 4 OTTIMIZZATO: Gimbal Executor con Resilienza
    def gimbal_executor_thread(self):
        """Thread gimbal ottimizzato per droni - resiliente e veloce"""
        print("🎯 Thread Gimbal Executor OTTIMIZZATO avviato...")
        
        consecutive_errors = 0
        max_errors = 5
        
        while self.running:
            # 🚀 TIMEOUT RIDOTTO: 1ms per latenza minima
            if self.target_updated.wait(timeout=0.001):
                self.target_updated.clear()
                
                # 🚀 ACQUISIZIONE ATOMICA NON-BLOCKING
                target_acquired = False
                if self.target_lock.acquire(blocking=False):
                    try:
                        target_yaw = self.target_yaw
                        target_pitch = self.target_pitch
                        target_acquired = True
                    finally:
                        self.target_lock.release()
                
                if not target_acquired:
                    continue  # Skip se non riesce ad acquisire il target
                
                # 🚀 CHECK VELOCE se serve aggiornamento
                needs_update = True
                if self.current_lock.acquire(blocking=False):
                    try:
                        needs_update = (target_yaw != self.current_yaw or target_pitch != self.current_pitch)
                    finally:
                        self.current_lock.release()
                
                if needs_update:
                    try:
                        # 🚀 ESECUZIONE GIMBAL con timeout implicito
                        self.cam.setGimbalRotation(target_yaw, target_pitch)
                        
                        # 🚀 UPDATE VELOCE posizione corrente (non-blocking)
                        if self.current_lock.acquire(blocking=False):
                            try:
                                self.current_yaw = target_yaw
                                self.current_pitch = target_pitch
                            finally:
                                self.current_lock.release()
                        
                        print(f"🎯 YAW: {target_yaw:.2f}° | PITCH: {target_pitch:.2f}°")
                        consecutive_errors = 0  # Reset error counter
                        
                    except Exception as e:
                        consecutive_errors += 1
                        print(f"⚠️ Errore gimbal #{consecutive_errors}: {e}")
                        
                        # 🚁 SAFETY: Se troppi errori consecutivi, pausa per recovery
                        if consecutive_errors >= max_errors:
                            print(f"🚨 Troppi errori gimbal, pausa safety 100ms...")
                            time.sleep(0.1)
                            consecutive_errors = 0
            else:
                # Timeout scaduto - continua il loop
                continue

    def run(self):
        print("🚀 Controller OTTIMIZZATO attivo: YAW (CH12), PITCH (CH10), ZOOM IN (CH8), ZOOM OUT (CH11)")
        print("🚁 Sistema dual-thread ottimizzato per applicazioni drone")
        print("🎯 Precisione CONTINUA: movimento fluido senza step discreti")

        # Avvio thread ottimizzati
        threading.Thread(target=self.zoom_rc_reader_thread, daemon=True).start()
        threading.Thread(target=self.zoom_executor_thread, daemon=True).start()
        threading.Thread(target=self.rc_position_writer_thread, daemon=True).start()
        threading.Thread(target=self.gimbal_executor_thread, daemon=True).start()

        try:
            while True:
                time.sleep(0.0001)  # Thread principale minimale
        except KeyboardInterrupt:
            print("\n🛑 Arresto controller...")
            self.running = False
            self.target_updated.set()  # Sveglia thread gimbal per uscita pulita
            time.sleep(0.1)  # Tempo per cleanup
            print("✅ Controller fermato.")

if __name__ == "__main__":
    controller = QuantizedGimbalController()
    controller.run()
