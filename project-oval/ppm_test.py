import Jetson.GPIO as GPIO
import time
from threading import Thread
import numpy as np

class PPMGenerator:
    def __init__(self, pin):
        """
        Initialize PPM generator
        pin: GPIO pin number to output PPM signal
        """
        self.pin = pin
        self.channel_values = np.ones(8) * 1.5  # Fixed 8 channels, initialized to center
        self.running = False
        
        # PPM timing constants (in microseconds)
        self.FRAME_LENGTH = 20000  # 50Hz = 20ms frame length
        self.SYNC_LENGTH = 300     # Sync pulse length
        
        # Pre-calculate fixed timings for unchanged channels
        self.fixed_channel_times = [1500] * 7  # 1.5ms for channels 1-7
        
        # Setup GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT)
        GPIO.output(self.pin, False)
        
    def set_channel(self, value_ms):
        """
        Set channel 0 value in milliseconds (1.0 to 2.0 ms)
        """
        value_ms = max(1.0, min(2.0, value_ms))  # Clamp between 1-2ms
        self.channel_values[0] = value_ms
            
    def generate_ppm(self):
        """
        Generate PPM signal continuously
        """
        while self.running:
            # Generate sync pulse
            GPIO.output(self.pin, True)
            time.sleep(self.SYNC_LENGTH / 1000000.0)
            GPIO.output(self.pin, False)
            
            # Generate channel 0 (variable)
            pulse_time = int(self.channel_values[0] * 1000)
            time.sleep(pulse_time / 1000000.0)
            GPIO.output(self.pin, True)
            time.sleep(self.SYNC_LENGTH / 1000000.0)
            GPIO.output(self.pin, False)
            
            # Generate channels 1-7 (fixed at 1.5ms)
            for _ in range(6):  # Changed from 7 to 6 to get exactly 8 channels total
                time.sleep(1500 / 1000000.0)  # 1.5ms fixed timing
                GPIO.output(self.pin, True)
                time.sleep(self.SYNC_LENGTH / 1000000.0)
                GPIO.output(self.pin, False)
            
            # Calculate and wait for remainder of frame
            total_time = self.SYNC_LENGTH + pulse_time + self.SYNC_LENGTH + \
                        (6 * (1500 + self.SYNC_LENGTH))  # Updated calculation for 6 fixed channels
            remaining = self.FRAME_LENGTH - total_time
            if remaining > 0:
                time.sleep(remaining / 1000000.0)
                
    def start(self):
        """
        Start PPM signal generation in a separate thread
        """
        self.running = True
        self.thread = Thread(target=self.generate_ppm)
        self.thread.daemon = True
        self.thread.start()
        
    def stop(self):
        """
        Stop PPM signal generation
        """
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join()
        GPIO.output(self.pin, False)
        
    def cleanup(self):
        """
        Cleanup GPIO resources
        """
        self.stop()
        GPIO.cleanup()

# Example usage
if __name__ == "__main__":
    try:
        # Initialize PPM generator on pin 18
        ppm = PPMGenerator(pin=18)
        
        # Start generating PPM signal
        ppm.start()
        
        # Example: Sweep channel 0 back and forth
        while True:
            for pos in np.arange(1.0, 2.0, 0.1):
                ppm.set_channel(pos)
                time.sleep(0.1)
            for pos in np.arange(2.0, 1.0, -0.1):
                ppm.set_channel(pos)
                time.sleep(0.1)
                
    except KeyboardInterrupt:
        print("Stopping PPM generation...")
    finally:
        if 'ppm' in locals():
            ppm.cleanup()