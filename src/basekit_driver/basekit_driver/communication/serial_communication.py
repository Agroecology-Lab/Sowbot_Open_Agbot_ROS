import serial
import rclpy

class SerialCommunication:
    def __init__(self, node, port="virtual", baudrate=115200):
        self._node = node
        self._logger = node.get_logger()
        
        try:
            self._port = node.get_parameter('port').value
        except:
            self._port = port
            
        self._serial = None
        
        if self._port in ["virtual", "NONE", "virtual_mcu", ""]:
            self._logger.info("🛠️  GHOST MODE: Serial communication bypassed. Compatibility active.")
            return

        try:
            self._serial = serial.Serial(self._port, baudrate, timeout=0.1)
            self._logger.info(f"✅ Connected to {self._port}")
        except Exception as e:
            self._logger.error(f"❌ Failed to open {self._port}: {e}")

    @property
    def serial(self):
        """Allows the driver to check 'if self.serial is not None' without crashing."""
        return self._serial

    # --- Registration Stubs ---
    def register_core_observer(self, observer):
        pass
        
    def register_bms_observer(self, observer):
        pass
        
    def register_gps_observer(self, observer):
        pass

    def send(self, data):
        if self._serial and self._serial.is_open:
            try:
                self._serial.write(data)
            except:
                pass

    def read(self):
        if self._serial and self._serial.is_open:
            return self._serial.readline()
        return b""
