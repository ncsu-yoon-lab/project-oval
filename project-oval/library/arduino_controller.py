import serial
import time
from threading import Thread
import queue

class ArduinoController:
    def __init__(self, port='/dev/ttyACM0', baud_rate=115200):
        self.serial_port = serial.Serial(port, baud_rate, timeout=1)
        self.message_queue = queue.Queue()
        self.running = True
        
        # Start reading thread
        self.read_thread = Thread(target=self._read_serial)
        self.read_thread.daemon = True
        self.read_thread.start()
    
    def _read_serial(self):
        while self.running:
            if self.serial_port.in_waiting:
                try:
                    line = self.serial_port.readline().decode('utf-8').strip()
                    if line:
                        self.message_queue.put(line)
                except Exception as e:
                    print(f"Error reading serial: {e}")
            time.sleep(0.01)
    
    def set_mode(self, mode):
        """Set the operation mode (0 for auto, 1 for manual)"""
        command = f"MODE,{mode}\n"
        self.serial_port.write(command.encode('utf-8'))
    
    def set_throttle(self, throttle):
        """Set the throttle value (1000-2000)"""
        command = f"THROTTLE,{throttle}\n"
        self.serial_port.write(command.encode('utf-8'))
    
    def get_latest_status(self):
        """Get the latest status message from the Arduino"""
        messages = []
        while not self.message_queue.empty():
            messages.append(self.message_queue.get())
        return messages[-1] if messages else None
    
    def close(self):
        """Clean up resources"""
        self.running = False
        if self.read_thread.is_alive():
            self.read_thread.join()
        self.serial_port.close()

# def main():
#     # Example usage
#     try:
#         # Initialize controller
#         controller = ArduinoController()
#         print("Connected to Arduino")
        
#         controller.set_mode(1)
        
#         # Test different throttle values
#         test_values = [1500, 1600, 1700, 1600, 1500]
#         for throttle in test_values:
#             print(f"\nSetting throttle to {throttle}")
#             controller.set_throttle(throttle)
#             time.sleep(2)
#             status = controller.get_latest_status()
#             if status:
#                 print(f"Status: {status}")
        
#     except KeyboardInterrupt:
#         print("\nExiting...")
#     finally:
#         controller.close()

# if __name__ == "__main__":
#     main()