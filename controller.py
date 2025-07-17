import sys
import os
import signal
import time
import select
import tty
import termios

# Aggiungi il percorso per il SDK
current = os.path.dirname(os.path.realpath(__file__))
parent_directory = os.path.dirname(current)
sys.path.append(parent_directory)

try:
    from siyi_sdk import SIYISDK
except ImportError:
    print("Errore: impossibile importare siyi_sdk. Assicurati che sia installato correttamente.")
    sys.exit(1)

class SIYIController:
    def __init__(self, server_ip="192.168.144.25", port=37260):
        self.cam = SIYISDK(server_ip=server_ip, port=port)
        self.current_yaw = 0.0
        self.current_pitch = 0.0
        self.movement_step = 5.0  # Gradi per ogni movimento
        self.running = True
        
        # Salva le impostazioni del terminale
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        # Connessione alla telecamera
        if not self.cam.connect():
            print("Errore: impossibile connettersi alla telecamera")
            sys.exit(1)
        
        print("Connesso alla telecamera SIYI A8 Mini")
        self.print_instructions()
    
    def print_instructions(self):
        """Stampa le istruzioni di controllo"""
        print("\n=== CONTROLLI - MODALITÀ STEP ===")
        print("W - Movimento SU (step singolo)")
        print("S - Movimento GIÙ (step singolo)")
        print("A - Movimento SINISTRA (step singolo)")
        print("D - Movimento DESTRA (step singolo)")
        print("Q - Zoom IN (step singolo)")
        print("E - Zoom OUT (step singolo)")
        print("R - Reset posizione (0,0)")
        print("+ - Aumenta step di movimento")
        print("- - Diminuisci step di movimento")
        print("X - Esci")
        print(f"\nStep corrente: {self.movement_step}°")
        print("\nPremi un tasto per il movimento...")
    
    def move_up(self):
        """Movimento verso l'alto (pitch +)"""
        new_pitch = min(45, self.current_pitch + self.movement_step)
        if new_pitch != self.current_pitch:
            self.current_pitch = new_pitch
            self.cam.setGimbalRotation(self.current_yaw, self.current_pitch)
            self.print_position()
        else:
            print(f"\rLimite superiore raggiunto (45°)", end="", flush=True)
    
    def move_down(self):
        """Movimento verso il basso (pitch -)"""
        new_pitch = max(-135, self.current_pitch - self.movement_step)
        if new_pitch != self.current_pitch:
            self.current_pitch = new_pitch
            self.cam.setGimbalRotation(self.current_yaw, self.current_pitch)
            self.print_position()
        else:
            print(f"\rLimite inferiore raggiunto (-135°)", end="", flush=True)
    
    def move_left(self):
        """Movimento verso sinistra (yaw -)"""
        new_yaw = max(-160, self.current_yaw - self.movement_step)
        if new_yaw != self.current_yaw:
            self.current_yaw = new_yaw
            self.cam.setGimbalRotation(self.current_yaw, self.current_pitch)
            self.print_position()
        else:
            print(f"\rLimite sinistro raggiunto (-160°)", end="", flush=True)
    
    def move_right(self):
        """Movimento verso destra (yaw +)"""
        new_yaw = min(160, self.current_yaw + self.movement_step)
        if new_yaw != self.current_yaw:
            self.current_yaw = new_yaw
            self.cam.setGimbalRotation(self.current_yaw, self.current_pitch)
            self.print_position()
        else:
            print(f"\rLimite destro raggiunto (160°)", end="", flush=True)
    
    def zoom_in(self):
        """Zoom in singolo"""
        self.cam.requestZoomIn()
        time.sleep(0.1)
        self.cam.requestZoomHold()
        zoom_level = self.cam.getZoomLevel()
        print(f"\rZoom IN - Level: {zoom_level}", end="", flush=True)
    
    def zoom_out(self):
        """Zoom out singolo"""
        self.cam.requestZoomOut()
        time.sleep(0.1)
        self.cam.requestZoomHold()
        zoom_level = self.cam.getZoomLevel()
        print(f"\rZoom OUT - Level: {zoom_level}", end="", flush=True)
    
    def reset_position(self):
        """Reset della posizione a (0,0)"""
        self.current_yaw = 0.0
        self.current_pitch = 0.0
        self.cam.setGimbalRotation(0.0, 0.0)
        print(f"\nPosizione resettata a (0, 0)")
    
    def increase_step(self):
        """Aumenta lo step di movimento"""
        if self.movement_step < 20.0:
            self.movement_step += 1.0
            print(f"\nStep aumentato a {self.movement_step}°")
        else:
            print(f"\nStep massimo raggiunto ({self.movement_step}°)")
    
    def decrease_step(self):
        """Diminuisce lo step di movimento"""
        if self.movement_step > 1.0:
            self.movement_step -= 1.0
            print(f"\nStep diminuito a {self.movement_step}°")
        else:
            print(f"\nStep minimo raggiunto ({self.movement_step}°)")
    
    def print_position(self):
        """Stampa la posizione corrente"""
        print(f"\rPosizione: Yaw={self.current_yaw:.1f}°, Pitch={self.current_pitch:.1f}° | Step={self.movement_step}°", end="", flush=True)
    
    def get_key_blocking(self):
        """Legge un carattere in modalità bloccante"""
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
            return key
        except:
            return None
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
    
    def handle_key_press(self, key):
        """Gestisce la pressione dei tasti con movimento singolo"""
        if key is None:
            return True
            
        key = key.lower()
        
        if key == 'x':
            return False  # Esci
        elif key == 'r':
            self.reset_position()
        elif key == 'w':
            self.move_up()
        elif key == 's':
            self.move_down()
        elif key == 'd':
            self.move_left()
        elif key == 'a':
            self.move_right()
        elif key == 'q':
            self.zoom_in()
        elif key == 'e':
            self.zoom_out()
        elif key == '+' or key == '=':
            self.increase_step()
        elif key == '-' or key == '_':
            self.decrease_step()
        elif key == '\x03':  # Ctrl+C
            return False
        else:
            print(f"\nTasto '{key}' non riconosciuto")
        
        return True
    
    def start_controller(self):
        """Avvia il controller principale"""
        try:
            print("\n--- Controller attivo - MODALITÀ STEP ---")
            self.print_position()
            
            while self.running:
                key = self.get_key_blocking()
                if key and not self.handle_key_press(key):
                    break
                    
        except KeyboardInterrupt:
            print("\n\nInterrotto dall'utente (Ctrl+C)")
        except Exception as e:
            print(f"\nErrore: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Pulisce le risorse"""
        self.running = False
        print("\n\nDisconnessione dalla telecamera...")
        
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        except:
            pass
        
        self.reset_position()
        time.sleep(1)
        self.cam.disconnect()
        print("Disconnesso")

def main():
    try:
        print("=== SIYI A8 Mini Controller - STEP MODE ===")
        print("Inizializzazione...")
        
        controller = SIYIController()
        controller.start_controller()
        
    except Exception as e:
        print(f"Errore: {e}")
    finally:
        os.kill(os.getpid(), signal.SIGKILL)

if __name__ == "__main__":
    main()
