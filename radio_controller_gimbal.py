import time
from pymavlink import mavutil
import threading
from collections import deque
try:
    from siyi_sdk import SIYISDK
except ImportError:
    print("Errore: impossibile importare siyi_sdk.")
    exit(1)

class QuantizedGimbalController:
    def __init__(self, connection="/dev/ttyAMA0", baud=57600, cam_ip="192.168.144.25", cam_port=37260):
        self.master = mavutil.mavlink_connection(connection, baud=baud)
        
        # Attendi il primo heartbeat
        self.master.wait_heartbeat()
        print("Heartbeat ricevuto.")
        
        # Richiedi lo stream RC_CHANNELS a frequenza più alta
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
            20,  # Aumentato a 20Hz per ridurre latenza
            1    # start = true
        )
        
        # Connessione camera
        self.cam = SIYISDK(server_ip=cam_ip, port=cam_port)
        if not self.cam.connect():
            print("Errore: impossibile connettersi alla telecamera")
            exit(1)
        
        # Variabili per il controllo
        self.prev_yaw = None
        self.prev_pitch = None
        
        # Threading per separare lettura e controllo
        self.latest_yaw = 0
        self.latest_pitch = 0
        self.new_data = False
        self.running = True
        
        # Lock per thread safety
        self.data_lock = threading.Lock()
        
        # Step di quantizzazione in gradi
        self.yaw_step = 5.0    # Step di 5° per YAW
        self.pitch_step = 5.0  # Step di 5° per PITCH
        
        # Pre-calcolo costanti per mapping
        self.yaw_scale = 240 / 1000  # (120 - (-120)) / (2000 - 1000)
        self.yaw_offset = -120 - (1000 * self.yaw_scale)
        
        self.pitch_scale = 115 / 1000  # (25 - (-90)) / (2000 - 1000)
        self.pitch_offset = -90 - (1000 * self.pitch_scale)
        
        print(f"Quantizzazione attiva: YAW step = {self.yaw_step}°, PITCH step = {self.pitch_step}°")
    
    def map_rc_fast(self, rc_value, scale, offset):
        """Mapping ottimizzato con pre-calcolo"""
        return rc_value * scale + offset
    
    def quantize_angle(self, angle, step):
        """Quantizza l'angolo al multiplo di step più vicino"""
        return round(angle / step) * step
    
    def rc_reader_thread(self):
        """Thread dedicato alla lettura dei dati RC"""
        print("Thread RC reader avviato...")
        
        while self.running:
            try:
                # Usa blocking=True con timeout breve per evitare errori seriali
                msg = self.master.recv_match(type='RC_CHANNELS', blocking=True, timeout=0.05)
                
                if msg:
                    rc_yaw = msg.chan12_raw
                    rc_pitch = msg.chan10_raw
                    
                    # Validazione rapida
                    if 900 < rc_yaw < 2100 and 900 < rc_pitch < 2100:
                        # Clamp values
                        rc_yaw = max(1000, min(2000, rc_yaw))
                        rc_pitch = max(1000, min(2000, rc_pitch))
                        
                        # Mapping ottimizzato
                        yaw_raw = self.map_rc_fast(rc_yaw, self.yaw_scale, self.yaw_offset)
                        pitch_raw = self.map_rc_fast(rc_pitch, self.pitch_scale, self.pitch_offset)
                        
                        # Quantizzazione a step
                        yaw = self.quantize_angle(yaw_raw, self.yaw_step)
                        pitch = self.quantize_angle(pitch_raw, self.pitch_step)
                        
                        with self.data_lock:
                            # Aggiorna solo se i valori quantizzati sono cambiati
                            if self.latest_yaw != yaw or self.latest_pitch != pitch:
                                self.latest_yaw = yaw
                                self.latest_pitch = pitch
                                self.new_data = True
                                
            except Exception as e:
                if self.running:  # Solo stampa errore se non stiamo fermando
                    print(f"Errore lettura RC: {e}")
                time.sleep(0.1)  # Pausa più lunga in caso di errore
    
    def gimbal_control_thread(self):
        """Thread dedicato al controllo del gimbal"""
        print("Thread controllo gimbal avviato...")
        
        while self.running:
            with self.data_lock:
                if self.new_data:
                    yaw = self.latest_yaw
                    pitch = self.latest_pitch
                    self.new_data = False
                else:
                    "time.sleep(0.005)"  # 5ms quando non ci sono nuovi dati
                    continue
            
            # Con la quantizzazione, invia sempre il comando quando ci sono nuovi dati
            # perché i valori cambiano solo quando c'è un movimento significativo
            try:
                # Comando al gimbal
                self.cam.setGimbalRotation(yaw, pitch)
                print(f"YAW: {yaw:.0f}° | PITCH: {pitch:.0f}°")
                
                self.prev_yaw = yaw
                self.prev_pitch = pitch
                
            except Exception as e:
                print(f"Errore controllo gimbal: {e}")
                time.sleep(0.01)
    
    def run(self):
        """Avvia il controllo multi-threaded"""
        print("Controllo YAW & PITCH quantizzato (CH12 = YAW, CH10 = PITCH)...")
        
        # Avvia i thread
        rc_thread = threading.Thread(target=self.rc_reader_thread, daemon=True)
        gimbal_thread = threading.Thread(target=self.gimbal_control_thread, daemon=True)
        
        rc_thread.start()
        gimbal_thread.start()
        
        try:
            # Mantieni il programma in esecuzione
            while True:
                time.sleep(0.001)
                
        except KeyboardInterrupt:
            print("\nInterruzione rilevata, fermando i thread...")
            self.running = False
            rc_thread.join(timeout=1)
            gimbal_thread.join(timeout=1)
            print("Controller fermato.")

if __name__ == "__main__":
    controller = QuantizedGimbalController()
    controller.run()
