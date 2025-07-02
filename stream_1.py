#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script da eseguire SUL RASPBERRY PI
Riceve video dalla telecamera SIYI del drone e lo trasmette al PC via RTMP
"""

import cv2
import imutils
from imutils.video import VideoStream
import logging
from time import time, sleep
import subprocess
import threading
import socket
import sys

# Importa le classi dal file stream.py originale
from stream import SIYIRTSP, RTMPSender

class DroneStreamManager:
    def __init__(self, pc_ip="192.168.0.124", debug=True):
        """
        Gestisce lo streaming dalla telecamera del drone al PC
        
        Params:
        - pc_ip: IP del PC che deve ricevere lo stream
        - debug: Abilita messaggi di debug
        """
        self.pc_ip = pc_ip
        self.debug = debug
        
        # Configurazione telecamera SIYI
        self.siyi_rtsp_url = "rtsp://192.168.144.25:8554/main.265"
        self.siyi_camera = None
        
        # Configurazione RTMP verso il PC
        self.rtmp_url = f"rtmp://{pc_ip}:1935/live/drone"
        self.rtmp_sender = None
        
        # Statistiche
        self.frame_count = 0
        self.start_time = time()
        self.last_stats_time = time()
        
        # Flag di controllo
        self.running = False
        
        # Setup logging
        if debug:
            level = logging.DEBUG
        else:
            level = logging.INFO
            
        logging.basicConfig(
            format='[%(levelname)s] %(asctime)s [DroneStream] %(message)s',
            level=level
        )
        self.logger = logging.getLogger(__name__)
        
    def check_network_setup(self):
        """Verifica la configurazione di rete"""
        self.logger.info("🔍 Verifica configurazione di rete...")
        
        # Test connessione alla telecamera
        try:
            result = subprocess.run(['ping', '-c', '2', '192.168.144.25'], 
                                  capture_output=True, text=True, timeout=10)
            if result.returncode == 0:
                self.logger.info("✅ Telecamera SIYI raggiungibile")
            else:
                self.logger.error("❌ Telecamera SIYI NON raggiungibile")
                self.logger.error("🔧 Configura l'IP del Raspberry Pi: sudo ip addr add 192.168.144.10/24 dev eth0")
                return False
        except Exception as e:
            self.logger.error(f"❌ Errore test telecamera: {e}")
            return False
            
        # Test connessione al PC
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5)
            result = sock.connect_ex((self.pc_ip, 22))  # Test porta SSH come esempio
            sock.close()
            
            if result == 0:
                self.logger.info(f"✅ PC {self.pc_ip} raggiungibile")
            else:
                self.logger.warning(f"⚠️  PC {self.pc_ip} potrebbe non essere raggiungibile")
                self.logger.info("   (Normale se non hai SSH attivo)")
                
        except Exception as e:
            self.logger.warning(f"⚠️  Test connessione PC: {e}")
            
        return True
        
    def setup_siyi_camera(self):
        """Inizializza la connessione alla telecamera SIYI"""
        try:
            self.logger.info("📹 Connessione alla telecamera SIYI...")
            self.siyi_camera = SIYIRTSP(
                rtsp_url=self.siyi_rtsp_url,
                cam_name="drone_siyi_a8",
                debug=self.debug
            )
            # Disabilita la finestra sulla Raspberry Pi (headless)
            self.siyi_camera.setShowWindow(False)
            self.logger.info("✅ Telecamera SIYI connessa")
            return True
            
        except Exception as e:
            self.logger.error(f"❌ Errore connessione SIYI: {e}")
            return False
            
    def setup_rtmp_sender(self):
        """Inizializza l'invio RTMP verso il PC"""
        try:
            self.logger.info(f"📡 Setup streaming RTMP verso {self.pc_ip}...")
            self.rtmp_sender = RTMPSender(
                rtmp_url=self.rtmp_url,
                debug=self.debug
            )
            
            # Configura parametri per drone (qualità vs prestazioni)
            self.rtmp_sender.setImageSize(w=1280, h=720)  # Risoluzione
            self.rtmp_sender.setFPS(30)  # Frame rate
            
            self.rtmp_sender.start()
            self.logger.info("✅ Streaming RTMP avviato")
            return True
            
        except Exception as e:
            self.logger.error(f"❌ Errore setup RTMP: {e}")
            return False
            
    def print_stats(self):
        """Stampa statistiche di streaming"""
        current_time = time()
        elapsed = current_time - self.start_time
        fps = self.frame_count / elapsed if elapsed > 0 else 0
        
        self.logger.info(f"📊 Frame: {self.frame_count} | "
                        f"Tempo: {elapsed:.1f}s | "
                        f"FPS: {fps:.1f}")
        
    def start_streaming(self):
        """Avvia lo streaming principale"""
        if not self.check_network_setup():
            return False
            
        if not self.setup_siyi_camera():
            return False
            
        if not self.setup_rtmp_sender():
            return False
            
        self.logger.info("🚀 Streaming dal drone avviato!")
        self.logger.info(f"📺 Apri questo URL sul tuo PC: rtmp://{self.pc_ip}:1935/live/drone")
        self.logger.info("🎮 Premi Ctrl+C per fermare")
        
        self.running = True
        self.start_time = time()
        self.last_stats_time = time()
        
        try:
            while self.running:
                # Ottieni frame dalla telecamera
                frame = self.siyi_camera.getFrame()
                
                if frame is not None:
                    self.frame_count += 1
                    
                    # Invia frame via RTMP al PC
                    self.rtmp_sender.setFrame(frame)
                    
                    # Stampa statistiche ogni 5 secondi
                    if time() - self.last_stats_time > 5.0:
                        self.print_stats()
                        self.last_stats_time = time()
                        
                else:
                    self.logger.warning("⚠️  Frame vuoto dalla telecamera")
                    sleep(0.1)
                    
        except KeyboardInterrupt:
            self.logger.info("🛑 Interruzione da utente")
        except Exception as e:
            self.logger.error(f"❌ Errore durante streaming: {e}")
        finally:
            self.stop_streaming()
            
    def stop_streaming(self):
        """Ferma lo streaming e libera le risorse"""
        self.logger.info("🔄 Chiusura streaming...")
        self.running = False
        
        if self.rtmp_sender:
            try:
                self.rtmp_sender.stop()
            except:
                pass
                
        if self.siyi_camera:
            try:
                self.siyi_camera.close()
            except:
                pass
                
        self.logger.info("✅ Streaming chiuso")

def show_usage():
    """Mostra le istruzioni d'uso"""
    print("""
🚁 DRONE STREAMING SETUP - Raspberry Pi
=====================================

QUESTO SCRIPT VA ESEGUITO SUL RASPBERRY PI

1. 📋 Configurazione di rete richiesta sul Raspberry Pi:
   sudo ip addr add 192.168.144.10/24 dev eth0

2. 🚀 Avvio dello script:
   python3 drone_streaming.py [IP_DEL_TUO_PC]

3. 📺 Sul tuo PC, apri lo stream con:
   - VLC: rtmp://192.168.0.124:1935/live/drone
   - FFplay: ffplay rtmp://192.168.0.124:1935/live/drone
   - OBS: Aggiungi sorgente Media con l'URL RTMP

Esempi:
   python3 drone_streaming.py                    # Usa IP predefinito 192.168.0.124
   python3 drone_streaming.py 192.168.0.100     # Usa IP personalizzato

⚠️  IMPORTANTE: Sul PC deve essere attivo un server RTMP (es. nginx-rtmp)
   oppure usa VLC per vedere direttamente lo stream.
""")

def main():
    if len(sys.argv) > 1 and sys.argv[1] in ['-h', '--help']:
        show_usage()
        return
        
    # IP del PC (personalizzabile da riga di comando)
    pc_ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.0.124"
    
    print(f"🚁 Avvio streaming drone verso PC: {pc_ip}")
    
    # Crea e avvia il gestore streaming
    drone_stream = DroneStreamManager(pc_ip=pc_ip, debug=True)
    drone_stream.start_streaming()

if __name__ == "__main__":
    main()