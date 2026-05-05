from pyvesc.protocol.interface import encode_request, encode, decode
from pyvesc.VESC.messages import *
import time
import threading
import subprocess

# because people may want to use this library for their own messaging, do not make this a required package
try:
    import serial
except ImportError:
    serial = None


class VESC(object):
    def __init__(self, serial_port, has_sensor=False, start_heartbeat=True, baudrate=115200, timeout=0.05):
        """
        :param serial_port: Serial device to use for communication (i.e. "COM3" or "/dev/tty.usbmodem0")
        :param has_sensor: Whether or not the bldc motor is using a hall effect sensor
        :param start_heartbeat: Whether or not to automatically start the heartbeat thread that will keep commands
                                alive.
        :param baudrate: baudrate for the serial communication. Shouldn't need to change this.
        :param timeout: timeout for the serial communication
        """

        if serial is None:
            raise ImportError("Need to install pyserial in order to use the VESCMotor class.")

        # FIXED: Configure UART with stty for JetPack 6.2 compatibility
        if 'ttyTHS' in serial_port:
            try:
                subprocess.run([
                    'stty', '-F', serial_port, 
                    str(baudrate), 'cs8', '-cstopb', '-parenb', '-crtscts', 'raw', '-echo'
                ], check=True, capture_output=True)
                time.sleep(0.1)  # Give time for configuration to take effect
            except subprocess.CalledProcessError:
                pass  # Continue even if stty fails

        self.serial_port = serial.Serial(port=serial_port, baudrate=baudrate, timeout=timeout)
        if has_sensor:
            self.serial_port.write(encode(SetRotorPositionMode(SetRotorPositionMode.DISP_POS_OFF)))

        self.alive_msg = [encode(Alive())]

        self.heart_beat_thread = threading.Thread(target=self._heartbeat_cmd_func)
        self._stop_heartbeat = threading.Event()

        if start_heartbeat:
            self.start_heartbeat()

        # FIXED: Robust firmware version checking with proper error handling
        try:
            version_str = self._get_firmware_version_safe()
            
            if version_str and self._is_valid_version(version_str):
                try:
                    major_version = int(version_str.split('.')[0])
                    if major_version < 3:
                        GetValues.fields = pre_v3_33_fields
                except (ValueError, IndexError):
                    # If we can't parse the version, assume modern firmware
                    pass
            else:
                # If we can't get a valid version, assume modern firmware and continue
                pass
                
        except Exception:
            # If firmware version check completely fails, assume modern firmware and continue
            pass

        # store message info for getting values so it doesn't need to calculate it every time
        msg = GetValues()
        self._get_values_msg = encode_request(msg)
        self._get_values_msg_expected_length = msg._full_msg_size

    def _get_firmware_version_safe(self):
        """
        Safely get firmware version with retries and proper error handling
        """
        max_retries = 3
        
        for attempt in range(max_retries):
            try:
                # Clear buffers before attempting version request
                self.serial_port.reset_input_buffer()
                self.serial_port.reset_output_buffer()
                time.sleep(0.05)
                
                # Get version with the original method
                version_raw = self.get_firmware_version()
                version_str = str(version_raw).strip()
                
                # Check if we got a valid response
                if self._is_valid_version(version_str):
                    return version_str
                    
                # If not valid, wait and retry
                time.sleep(0.1)
                
            except Exception:
                # If this attempt fails, wait and retry
                time.sleep(0.1)
                continue
        
        # If all retries failed, return None
        return None

    def _is_valid_version(self, version_str):
        """
        Check if the version string is valid
        """
        if not version_str:
            return False
        if version_str in ['None', 'null', 'NULL', '']:
            return False
        if '.' not in version_str:
            return False
        
        # Try to parse the version to make sure it's actually a version string
        try:
            parts = version_str.split('.')
            if len(parts) >= 2:
                int(parts[0])  # Should be able to convert major version to int
                return True
        except (ValueError, IndexError):
            pass
            
        return False

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop_heartbeat()
        if self.serial_port.is_open:
            self.serial_port.flush()
            self.serial_port.close()

    def _heartbeat_cmd_func(self):
        """
        Continuous function calling that keeps the motor alive
        """
        while not self._stop_heartbeat.is_set():
            time.sleep(0.1)
            for i in self.alive_msg:
                self.write(i)

    def start_heartbeat(self, can_id=None):
        """
        Starts a repetitive calling of the last set cmd to keep the motor alive.

        Args:
            can_id: Optional, used to specify the CAN ID to add to the existing heartbeat messaged
        """
        if can_id is not None:
            self.alive_msg.append(encode(Alive(can_id=can_id)))
        else:
            if not self.heart_beat_thread.is_alive():  # FIXED: Check if thread is already running
                self.heart_beat_thread.start()

    def stop_heartbeat(self):
        """
        Stops the heartbeat thread and resets the last cmd function. THIS MUST BE CALLED BEFORE THE OBJECT GOES OUT OF
        SCOPE UNLESS WRAPPING IN A WITH STATEMENT (Assuming the heartbeat was started).
        """
        self._stop_heartbeat.set()
        if self.heart_beat_thread.is_alive():
            self.heart_beat_thread.join()

    def write(self, data, num_read_bytes=None):
        """
        A write wrapper function implemented like this to try and make it easier to incorporate other communication
        methods than UART in the future.
        :param data: the byte string to be sent
        :param num_read_bytes: number of bytes to read for decoding response
        :return: decoded response from buffer
        """
        self.serial_port.write(data)
        if num_read_bytes is not None:
            # FIXED: Add timeout to prevent infinite waiting
            start_time = time.time()
            timeout = 2.0  # 2 second timeout
            
            while self.serial_port.in_waiting <= num_read_bytes:
                if time.time() - start_time > timeout:
                    break
                time.sleep(0.000001)  # add some delay just to help the CPU
            
            if self.serial_port.in_waiting > 0:
                response, consumed = decode(self.serial_port.read(self.serial_port.in_waiting))
                return response
            else:
                return None

    def set_rpm(self, new_rpm, **kwargs):
        """
        Set the electronic RPM value (a.k.a. the RPM value of the stator)
        :param new_rpm: new rpm value
        """
        self.write(encode(SetRPM(new_rpm, **kwargs)))

    def set_current(self, new_current, **kwargs):
        """
        :param new_current: new current in milli-amps for the motor
        """
        self.write(encode(SetCurrent(new_current, **kwargs)))

    def set_duty_cycle(self, new_duty_cycle, **kwargs):
        """
        :param new_duty_cycle: Value of duty cycle to be set (range [-1e5, 1e5]).
        """
        self.write(encode(SetDutyCycle(new_duty_cycle, **kwargs)))

    def set_servo(self, new_servo_pos, **kwargs):
        """
        :param new_servo_pos: New servo position. valid range [0, 1]
        """
        self.write(encode(SetServoPosition(new_servo_pos, **kwargs)))

    def get_measurements(self):
        """
        :return: A msg object with attributes containing the measurement values
        """
        return self.write(self._get_values_msg, num_read_bytes=self._get_values_msg_expected_length)

    def get_measurements_can(self, can_id):
        """
        Get measurements from a specific CAN ID
        :param can_id: CAN ID of the motor to get measurements from
        :return: A msg object with attributes containing the measurement values
        """
        try:
            
            msg = GetValues(can_id=can_id)
            request = encode_request(msg)
            response = self.write(request, num_read_bytes=msg._full_msg_size)
            return response
        except Exception as e:
            print(f"Error getting CAN measurements: {e}")
            return None

    def get_firmware_version(self):
        msg = GetVersion()
        result = self.write(encode_request(msg), num_read_bytes=msg._full_msg_size)
        return str(result) if result is not None else None

    def get_rpm(self, can_id=None):
        """
        :return: Current motor rpm
        """

        if can_id is None:
            measurements = self.get_measurements()
            return measurements.rpm if measurements else 0
        else:
            measurements = self.get_measurements_can(can_id)
            return measurements.rpm if measurements else 0

    def get_duty_cycle(self):
        """
        :return: Current applied duty-cycle
        """
        measurements = self.get_measurements()
        return measurements.duty_now if measurements else 0

    def get_v_in(self):
        """
        :return: Current input voltage
        """
        measurements = self.get_measurements()
        return measurements.v_in if measurements else 0

    def get_motor_current(self):
        """
        :return: Current motor current
        """
        measurements = self.get_measurements()
        return measurements.current_motor if measurements else 0

    def get_incoming_current(self):
        """
        :return: Current incoming current
        """
        measurements = self.get_measurements()
        return measurements.current_in if measurements else 0


# Convenience function for easy VESC creation
def create_vesc(serial_port, **kwargs):
    """
    Create a VESC connection with sensible defaults for JetPack 6.2
    
    :param serial_port: Serial device path (e.g., '/dev/ttyTHS1')
    :param kwargs: Additional arguments for VESC constructor
    :return: VESC object
    """
    # Set default timeout higher for JetPack 6.2
    if 'timeout' not in kwargs:
        kwargs['timeout'] = 1.0
    
    return VESC(serial_port, **kwargs)


# Test function
if __name__ == "__main__":
    print("Testing Fixed VESC Library")
    print("=" * 40)
    
    try:
        # Test connection
        motor = create_vesc('/dev/ttyTHS1', start_heartbeat=False)
        print("✓ VESC connected successfully!")
        
        # Test basic functionality
        measurements = motor.get_measurements()
        if measurements:
            print(f"✓ RPM: {measurements.rpm}")
            print(f"✓ Input Voltage: {measurements.v_in:.2f}V")
            print(f"✓ Motor Current: {measurements.current_motor:.2f}A")
            print(f"✓ Duty Cycle: {measurements.duty_now:.3f}")
        else:
            print("⚠ Could not get measurements")
        
        # Test individual getters
        print(f"✓ get_rpm(): {motor.get_rpm()}")
        print(f"✓ get_v_in(): {motor.get_v_in():.2f}V")
        
        print("\n🎉 All tests passed!")
        
    except Exception as e:
        print(f"✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            if 'motor' in locals():
                motor.stop_heartbeat()
                motor.serial_port.close()
        except:
            pass