import socket
from siyi_message import *
from time import sleep, time
import logging
from utils import toInt
import threading

class SIYISDK:
    def __init__(self, server_ip="192.168.144.25", port=37260, debug=False):
        """
        Params
        --
        - server_ip [str] IP address of the camera
        - port: [int] UDP port of the camera
        """
        self._debug = debug
        if self._debug:
            d_level = logging.DEBUG
        else:
            d_level = logging.INFO
        LOG_FORMAT = ' [%(levelname)s] %(asctime)s [SIYISDK::%(funcName)s] :\t%(message)s'
        logging.basicConfig(format=LOG_FORMAT, level=d_level)
        self._logger = logging.getLogger(self.__class__.__name__)

        # Message sent to the camera
        self._out_msg = SIYIMESSAGE(debug=self._debug)
        
        # Message received from the camera
        self._in_msg = SIYIMESSAGE(debug=self._debug)        

        self._server_ip = server_ip
        self._port = port
        self._BUFF_SIZE = 1024

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rcv_wait_t = 2
        self._socket.settimeout(self._rcv_wait_t)

        self._connected = False

        # Message containers
        self._fw_msg = FirmwareMsg()
        self._hw_msg = HardwareIDMsg()
        self._autoFocus_msg = AutoFocusMsg()
        self._manualZoom_msg = ManualZoomMsg()
        self._manualFocus_msg = ManualFocusMsg()
        self._gimbalSpeed_msg = GimbalSpeedMsg()
        self._center_msg = CenterMsg()
        self._record_msg = RecordingMsg()
        self._mountDir_msg = MountDirMsg()
        self._motionMode_msg = MotionModeMsg()
        self._funcFeedback_msg = FuncFeedbackInfoMsg()
        self._att_msg = AttitdueMsg()
        self._last_att_seq = -1

        # Thread control
        self._stop = False
        self._recv_thread = threading.Thread(target=self.recvLoop, daemon=True)

        # Ottimizzazioni: frequenze ridotte e thread opzionali
        self._last_fw_seq = 0
        self._conn_loop_rate = 2.0  # Ridotto da 1 a 2 secondi
        self._conn_thread = threading.Thread(target=self.connectionLoop, args=(self._conn_loop_rate,), daemon=True)

        # Thread gimbal info ridotto a 0.5Hz invece di 1Hz
        self._gimbal_info_loop_rate = 2.0  # Ogni 2 secondi
        self._g_info_thread = threading.Thread(target=self.gimbalInfoLoop, args=(self._gimbal_info_loop_rate,), daemon=True)

        # Thread attitude ridotto a 5Hz invece di 10Hz
        self._gimbal_att_loop_rate = 0.2  # Ogni 200ms invece di 100ms
        self._g_att_thread = threading.Thread(target=self.gimbalAttLoop, args=(self._gimbal_att_loop_rate,), daemon=True)

        # Flag per controllo thread opzionali
        self._enable_info_thread = False
        self._enable_att_thread = False

    def resetVars(self):
        """Reset variables to initial values"""
        self._connected = False
        self._fw_msg = FirmwareMsg()
        self._hw_msg = HardwareIDMsg()
        self._autoFocus_msg = AutoFocusMsg()
        self._manualZoom_msg = ManualZoomMsg()
        self._manualFocus_msg = ManualFocusMsg()
        self._gimbalSpeed_msg = GimbalSpeedMsg()
        self._center_msg = CenterMsg()
        self._record_msg = RecordingMsg()
        self._mountDir_msg = MountDirMsg()
        self._motionMode_msg = MotionModeMsg()
        self._funcFeedback_msg = FuncFeedbackInfoMsg()
        self._att_msg = AttitdueMsg()
        return True

    def connect(self, maxWaitTime=3.0, enable_info_thread=False, enable_att_thread=False):
        """
        Connect to camera with optional background threads
        
        Params
        --
        maxWaitTime [float] Maximum time to wait before giving up
        enable_info_thread [bool] Enable gimbal info background thread
        enable_att_thread [bool] Enable gimbal attitude background thread
        """
        self._enable_info_thread = enable_info_thread
        self._enable_att_thread = enable_att_thread
        
        if not self._recv_thread.is_alive():
            self._recv_thread.start()
        if not self._conn_thread.is_alive():
            self._conn_thread.start()
            
        t0 = time()
        while True:
            if self._connected:
                # Avvia thread opzionali solo se richiesti
                if self._enable_info_thread and not self._g_info_thread.is_alive():
                    self._g_info_thread.start()
                if self._enable_att_thread and not self._g_att_thread.is_alive():
                    self._g_att_thread.start()
                return True
            if (time() - t0) > maxWaitTime and not self._connected:
                self.disconnect()
                self._logger.error("Failed to connect to camera")
                return False

    def disconnect(self):
        """Disconnect and stop all threads"""
        self._logger.info("Stopping all threads")
        self._stop = True
        self.resetVars()

    def checkConnection(self):
        """Check connection by requesting firmware version"""
        self.requestFirmwareVersion()
        sleep(0.1)
        if self._fw_msg.seq != self._last_fw_seq and len(self._fw_msg.gimbal_firmware_ver) > 0:
            self._connected = True
            self._last_fw_seq = self._fw_msg.seq
        else:
            self._connected = False

    def connectionLoop(self, t):
        """Background connection checking loop"""
        while not self._stop:
            self.checkConnection()
            sleep(t)
        self._connected = False
        self.resetVars()
        self._logger.warning("Connection checking loop stopped")

    def isConnected(self):
        return self._connected

    def gimbalInfoLoop(self, t):
        """Background gimbal info loop (opzionale)"""
        while not self._stop and self._connected:
            self.requestGimbalInfo()
            sleep(t)
        self._logger.warning("Gimbal info thread stopped")

    def gimbalAttLoop(self, t):
        """Background gimbal attitude loop (opzionale)"""
        while not self._stop and self._connected:
            self.requestGimbalAttitude()
            sleep(t)
        self._logger.warning("Gimbal attitude thread stopped")

    def sendMsg(self, msg):
        """Send message to camera"""
        try:
            b = bytes.fromhex(msg)
            self._socket.sendto(b, (self._server_ip, self._port))
            return True
        except Exception as e:
            self._logger.error("Could not send bytes: %s", e)
            return False

    def recvLoop(self):
        """Main receiving loop"""
        self._logger.debug("Started data receiving thread")
        while not self._stop:
            try:
                self.bufferCallback()
            except Exception as e:
                if not self._stop:
                    self._logger.warning("Error in receive loop: %s", e)
        self._logger.debug("Exiting data receiving thread")

    def bufferCallback(self):
        """Receive and parse messages - versione ottimizzata"""
        try:
            buff, addr = self._socket.recvfrom(self._BUFF_SIZE)
        except socket.timeout:
            return
        except Exception as e:
            self._logger.warning("Socket error: %s", e)
            return

        buff_str = buff.hex()
        if self._debug:
            self._logger.debug("Buffer: %s", buff_str)

        MINIMUM_DATA_LENGTH = 20  # 10 bytes * 2 chars
        HEADER = '5566'
        
        # Parsing ottimizzato
        while len(buff_str) >= MINIMUM_DATA_LENGTH:
            if not buff_str.startswith(HEADER):
                buff_str = buff_str[2:]  # Rimuovi 1 byte (2 chars)
                continue

            # Estrai lunghezza dati
            try:
                low_b = buff_str[6:8]
                high_b = buff_str[8:10]
                data_len = int(high_b + low_b, 16)
                char_len = data_len * 2
            except ValueError:
                buff_str = buff_str[2:]
                continue

            # Verifica se ci sono abbastanza dati
            if len(buff_str) < (MINIMUM_DATA_LENGTH + char_len):
                break
            
            packet = buff_str[:MINIMUM_DATA_LENGTH + char_len]
            buff_str = buff_str[MINIMUM_DATA_LENGTH + char_len:]

            # Decodifica pacchetto
            val = self._in_msg.decodeMsg(packet)
            if val is None:
                continue
            
            data, data_len, cmd_id, seq = val[0], val[1], val[2], val[3]
            
            # Parsing ottimizzato con dict lookup
            parsers = {
                COMMAND.ACQUIRE_FW_VER: self.parseFirmwareMsg,
                COMMAND.ACQUIRE_HW_ID: self.parseHardwareIDMsg,
                COMMAND.ACQUIRE_GIMBAL_INFO: self.parseGimbalInfoMsg,
                COMMAND.ACQUIRE_GIMBAL_ATT: self.parseAttitudeMsg,
                COMMAND.FUNC_FEEDBACK_INFO: self.parseFunctionFeedbackMsg,
                COMMAND.GIMBAL_ROT: self.parseGimbalSpeedMsg,
                COMMAND.AUTO_FOCUS: self.parseAutoFocusMsg,
                COMMAND.MANUAL_FOCUS: self.parseManualFocusMsg,
                COMMAND.MANUAL_ZOOM: self.parseZoomMsg,
                COMMAND.CENTER: self.parseGimbalCenterMsg,
            }
            
            parser = parsers.get(cmd_id)
            if parser:
                parser(data, seq)
            else:
                self._logger.warning("CMD ID %s not recognized", cmd_id)

    # Request functions (mantenute come originali ma con migliore error handling)
    def requestFirmwareVersion(self):
        msg = self._out_msg.firmwareVerMsg()
        return self.sendMsg(msg)

    def requestHardwareID(self):
        msg = self._out_msg.hwIdMsg()
        return self.sendMsg(msg)

    def requestGimbalAttitude(self):
        msg = self._out_msg.gimbalAttMsg()
        return self.sendMsg(msg)

    def requestGimbalInfo(self):
        msg = self._out_msg.gimbalInfoMsg()
        return self.sendMsg(msg)

    def requestGimbalSpeed(self, yaw_speed: int, pitch_speed: int):
        msg = self._out_msg.gimbalSpeedMsg(yaw_speed, pitch_speed)
        return self.sendMsg(msg)

    def requestCenterGimbal(self):
        msg = self._out_msg.centerMsg()
        return self.sendMsg(msg)

    def requestPhoto(self):
        msg = self._out_msg.takePhotoMsg()
        return self.sendMsg(msg)

    def requestRecording(self):
        msg = self._out_msg.recordMsg()
        return self.sendMsg(msg)

    def requestAutoFocus(self):
        msg = self._out_msg.autoFocusMsg()
        return self.sendMsg(msg)

    def requestZoomIn(self):
        msg = self._out_msg.zoomInMsg()
        return self.sendMsg(msg)

    def requestZoomOut(self):
        msg = self._out_msg.zoomOutMsg()
        return self.sendMsg(msg)

    def requestZoomHold(self):
        msg = self._out_msg.stopZoomMsg()
        return self.sendMsg(msg)

    # Parsing functions (mantenute come originali)
    def parseFirmwareMsg(self, msg: str, seq: int):
        try:
            self._fw_msg.gimbal_firmware_ver = msg[8:16]
            self._fw_msg.seq = seq
            if self._debug:
                self._logger.debug("Firmware version: %s", self._fw_msg.gimbal_firmware_ver)
            return True
        except Exception as e:
            self._logger.error("Error parsing firmware msg: %s", e)
            return False

    def parseHardwareIDMsg(self, msg: str, seq: int):
        try:
            self._hw_msg.seq = seq
            self._hw_msg.id = msg
            if self._debug:
                self._logger.debug("Hardware ID: %s", self._hw_msg.id)
            return True
        except Exception as e:
            self._logger.error("Error parsing hardware ID msg: %s", e)
            return False

    def parseAttitudeMsg(self, msg: str, seq: int):
        try:
            self._att_msg.seq = seq
            self._att_msg.yaw = toInt(msg[2:4] + msg[0:2]) / 10.0
            self._att_msg.pitch = toInt(msg[6:8] + msg[4:6]) / 10.0
            self._att_msg.roll = toInt(msg[10:12] + msg[8:10]) / 10.0
            self._att_msg.yaw_speed = toInt(msg[14:16] + msg[12:14]) / 10.0
            self._att_msg.pitch_speed = toInt(msg[18:20] + msg[16:18]) / 10.0
            self._att_msg.roll_speed = toInt(msg[22:24] + msg[20:22]) / 10.0

            if self._debug:
                self._logger.debug("Attitude (yaw, pitch, roll): (%.1f, %.1f, %.1f)", 
                                 self._att_msg.yaw, self._att_msg.pitch, self._att_msg.roll)
            return True
        except Exception as e:
            self._logger.error("Error parsing attitude msg: %s", e)
            return False

    def parseGimbalInfoMsg(self, msg: str, seq: int):
        try:
            self._record_msg.seq = seq
            self._mountDir_msg.seq = seq
            self._motionMode_msg.seq = seq
            
            self._record_msg.state = int(msg[6:8], 16)
            self._motionMode_msg.mode = int(msg[8:10], 16)
            self._mountDir_msg.dir = int(msg[10:12], 16)

            if self._debug:
                self._logger.debug("Recording: %s, Mode: %s, Dir: %s", 
                                 self._record_msg.state, self._motionMode_msg.mode, self._mountDir_msg.dir)
            return True
        except Exception as e:
            self._logger.error("Error parsing gimbal info msg: %s", e)
            return False

    def parseAutoFocusMsg(self, msg: str, seq: int):
        try:
            self._autoFocus_msg.seq = seq
            self._autoFocus_msg.success = bool(int(msg, 16))
            if self._debug:
                self._logger.debug("Auto focus success: %s", self._autoFocus_msg.success)
            return True
        except Exception as e:
            self._logger.error("Error parsing auto focus msg: %s", e)
            return False

    def parseZoomMsg(self, msg: str, seq: int):
        try:
            self._manualZoom_msg.seq = seq
            self._manualZoom_msg.level = int(msg[2:4] + msg[0:2], 16) / 10.0
            if self._debug:
                self._logger.debug("Zoom level: %.1f", self._manualZoom_msg.level)
            return True
        except Exception as e:
            self._logger.error("Error parsing zoom msg: %s", e)
            return False

    def parseManualFocusMsg(self, msg: str, seq: int):
        try:
            self._manualFocus_msg.seq = seq
            self._manualFocus_msg.success = bool(int(msg, 16))
            if self._debug:
                self._logger.debug("Manual focus success: %s", self._manualFocus_msg.success)
            return True
        except Exception as e:
            self._logger.error("Error parsing manual focus msg: %s", e)
            return False

    def parseGimbalSpeedMsg(self, msg: str, seq: int):
        try:
            self._gimbalSpeed_msg.seq = seq
            self._gimbalSpeed_msg.success = bool(int(msg, 16))
            if self._debug:
                self._logger.debug("Gimbal speed success: %s", self._gimbalSpeed_msg.success)
            return True
        except Exception as e:
            self._logger.error("Error parsing gimbal speed msg: %s", e)
            return False

    def parseGimbalCenterMsg(self, msg: str, seq: int):
        try:
            self._center_msg.seq = seq
            self._center_msg.success = bool(int(msg, 16))
            if self._debug:
                self._logger.debug("Gimbal center success: %s", self._center_msg.success)
            return True
        except Exception as e:
            self._logger.error("Error parsing gimbal center msg: %s", e)
            return False

    def parseFunctionFeedbackMsg(self, msg: str, seq: int):
        try:
            self._funcFeedback_msg.seq = seq
            self._funcFeedback_msg.info_type = int(msg, 16)
            if self._debug:
                self._logger.debug("Function feedback code: %s", self._funcFeedback_msg.info_type)
            return True
        except Exception as e:
            self._logger.error("Error parsing function feedback msg: %s", e)
            return False

    # Get functions
    def getAttitude(self):
        return (self._att_msg.yaw, self._att_msg.pitch, self._att_msg.roll)

    def getAttitudeSpeed(self):
        return (self._att_msg.yaw_speed, self._att_msg.pitch_speed, self._att_msg.roll_speed)

    def getFirmwareVersion(self):
        return self._fw_msg.gimbal_firmware_ver

    def getHardwareID(self):
        return self._hw_msg.id

    def getRecordingState(self):
        return self._record_msg.state

    def getMotionMode(self):
        return self._motionMode_msg.mode

    def getMountingDirection(self):
        return self._mountDir_msg.dir

    def getFunctionFeedback(self):
        return self._funcFeedback_msg.info_type

    def getZoomLevel(self):
        return self._manualZoom_msg.level

    # Set functions
    def setGimbalRotation(self, yaw, pitch, err_thresh=1.0, kp=4, max_attempts=50):
        """
        Sets gimbal attitude angles yaw and pitch in degrees - versione ottimizzata
        
        Params
        --
        yaw: [float] desired yaw in degrees
        pitch: [float] desired pitch in degrees  
        err_thresh: [float] acceptable error threshold, in degrees
        kp: [float] proportional gain
        max_attempts: [int] maximum number of attempts
        """
        if not (-90 <= pitch <= 25):
            self._logger.error("Desired pitch %.1f is outside controllable range -90~25", pitch)
            return False
        
        if not (-45 <= yaw <= 45):
            self._logger.error("Desired yaw %.1f is outside controllable range -45~45", yaw)
            return False
        
        self._logger.info("Starting gimbal rotation to yaw=%.1f, pitch=%.1f", yaw, pitch)
        
        attempts = 0
        while attempts < max_attempts:
            self.requestGimbalAttitude()
            
            # Attendi messaggio di attitude aggiornato
            if self._att_msg.seq == self._last_att_seq:
                self._logger.debug("Waiting for new attitude message")
                self.requestGimbalSpeed(0, 0)
                sleep(0.2)  # Ridotto da 0.5 a 0.2
                attempts += 1
                continue
            
            self._last_att_seq = self._att_msg.seq
            
            # Calcola errori
            yaw_err = -yaw + self._att_msg.yaw  # Yaw invertito
            pitch_err = pitch - self._att_msg.pitch
            
            if self._debug:
                self._logger.debug("Errors - yaw: %.2f, pitch: %.2f", yaw_err, pitch_err)
            
            # Controlla se obiettivo raggiunto
            if abs(yaw_err) <= err_thresh and abs(pitch_err) <= err_thresh:
                self.requestGimbalSpeed(0, 0)
                self._logger.info("Goal rotation reached")
                return True
            
            # Calcola velocità con saturazione
            y_speed_sp = max(-100, min(100, int(kp * yaw_err)))
            p_speed_sp = max(-100, min(100, int(kp * pitch_err)))
            
            if self._debug:
                self._logger.debug("Speed setpoints - yaw: %d, pitch: %d", y_speed_sp, p_speed_sp)
            
            self.requestGimbalSpeed(y_speed_sp, p_speed_sp)
            sleep(0.1)  # Frequenza comando ridotta
            attempts += 1
        
        # Timeout raggiunto
        self._logger.warning("Max attempts (%d) reached without reaching goal rotation", max_attempts)
        self.requestGimbalSpeed(0, 0)
        return False


def test():
    """Test function ottimizzato"""
    cam = SIYISDK(debug=False)

    # Connessione senza thread automatici per risparmiare risorse
    if not cam.connect(enable_info_thread=False, enable_att_thread=False):
        exit(1)

    print("Firmware version:", cam.getFirmwareVersion())
    
    # Test movimento gimbal
    cam.requestGimbalSpeed(10, 0)
    sleep(2)
    cam.requestGimbalSpeed(0, 0)
    print("Attitude:", cam.getAttitude())
    
    # Test rotazione ottimizzata
    success = cam.setGimbalRotation(10, 20, err_thresh=1, kp=4)
    print("Rotation success:", success)
    
    cam.disconnect()

if __name__ == "__main__":
    test()
