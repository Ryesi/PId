from machine import Pin, PWM, time_pulse_us
import time

# Define pins
trigger_pin = Pin(5, Pin.OUT)
echo_pin = Pin(18, Pin.IN)
servo_pin = Pin(13, Pin.OUT)

# Initialize PWM for servo
pwm = PWM(servo_pin)
pwm.freq(50)

# Define the target distance (center of the beam)
TARGET_DISTANCE = 20.0  # Example target in cm
Kp = 0.26 # Proportional gain (tune this value)
Ki = 0.005  # Integral gain (tune this value)

# Initialize integral term
integral_error = 0.0

def map_value(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def servo_angle(angle):
    pwm.duty(map_value(angle, 0, 180, 20, 120))

def get_distance():
    # Send a 10us pulse to trigger the ultrasonic module
    trigger_pin.off()
    time.sleep_us(2)
    trigger_pin.on()
    time.sleep_us(10)
    trigger_pin.off()

    # Measure the duration of the pulse in microseconds
    duration = time_pulse_us(echo_pin, 1)

    # Calculate the distance in centimeters
    distance = (duration / 2) / 29.1
    return distance

def pi_controller(target, current, dt):
    global integral_error
    
    # Calculate the error
    error = target - current
    
    # Accumulate the integral error
    integral_error += error * dt
    
    # Apply the PI control
    control_signal = Kp * error + Ki * integral_error
    
    # Map the control signal to the servo angle
    servo_position = map_value(control_signal, target-34, 34-target, 135, 35)
    
    # Ensure servo angle stays within limits
    servo_position = max(0, min(180, servo_position))
    
    transfer = target+ error 
    print("transfer:",transfer )
    
    return servo_position

last_time = time.ticks_ms()

while True:
    # Get the current distance from the ultrasonic sensor
    current_distance = get_distance()
    
    # Calculate time difference in seconds
    current_time = time.ticks_ms()
    dt = time.ticks_diff(current_time, last_time) / 1000.0
    last_time = current_time
    
    # Calculate the required servo position using the PI-controller
    angle = pi_controller(TARGET_DISTANCE, current_distance, dt)
    
    # Set the servo to the calculated angle
    servo_angle(angle)
    
    # Print the distance and the servo angle for monitoring
    #print("Distance: {:.2f} cm, Servo Angle: {:.2f}".format(current_distance, angle))
    
    # Small delay to allow the system to stabilize
    time.sleep(0.1)
