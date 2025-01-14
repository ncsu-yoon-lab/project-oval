
import Jetson.GPIO as GPIO
import keyboard
import time

# PWM PIN - Change this to match your setup
PWM_PIN = 18  # Example pin, adjust according to your wiring

# PWM Parameters
FREQUENCY = 50  # 50Hz for standard servo/ESC
MIN_DUTY = 5    # 1ms pulse (5% duty cycle at 50Hz)
MAX_DUTY = 10   # 2ms pulse (10% duty cycle at 50Hz)
CENTER_DUTY = 7.5  # 1.5ms pulse (7.5% duty cycle at 50Hz)
DEADBAND = 15   # 15% deadband

# Initialize GPIO
def setup():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(PWM_PIN, GPIO.OUT)
    pwm = GPIO.PWM(PWM_PIN, FREQUENCY)
    pwm.start(CENTER_DUTY)
    return pwm

# Calculate deadband range
def get_deadband_range():
    range_size = MAX_DUTY - MIN_DUTY
    deadband_size = range_size * (DEADBAND / 100)
    deadband_min = CENTER_DUTY - (deadband_size / 2)
    deadband_max = CENTER_DUTY + (deadband_size / 2)
    return deadband_min, deadband_max

def main():
    pwm = setup()
    current_duty = CENTER_DUTY
    deadband_min, deadband_max = get_deadband_range()
    step = 0.1  # Adjustment step size
    
    print("PWM Control Started")
    print("Use Up/Down arrow keys to control")
    print("Press 'q' to quit")
    
    try:
        while True:
            if keyboard.is_pressed('up'):
                current_duty = min(MAX_DUTY, current_duty + step)
                if deadband_min <= current_duty <= deadband_max:
                    current_duty = deadband_max
                pwm.ChangeDutyCycle(current_duty)
                print(f"Duty Cycle: {current_duty:.1f}%")
                
            elif keyboard.is_pressed('down'):
                current_duty = max(MIN_DUTY, current_duty - step)
                if deadband_min <= current_duty <= deadband_max:
                    current_duty = deadband_min
                pwm.ChangeDutyCycle(current_duty)
                print(f"Duty Cycle: {current_duty:.1f}%")
                
            elif keyboard.is_pressed('space'):
                current_duty = CENTER_DUTY
                pwm.ChangeDutyCycle(current_duty)
                print(f"Centered: {current_duty:.1f}%")
                
            elif keyboard.is_pressed('q'):
                break
                
            time.sleep(0.01)  # Small delay to prevent CPU overload
            
    except KeyboardInterrupt:
        pass
    
    finally:
        pwm.stop()
        GPIO.cleanup()
        print("\nPWM Control Stopped")

if __name__ == "__main__":
    main()