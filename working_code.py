from machine import Pin, PWM, time_pulse_us, ADC
import time

# Define pins
trigger_pin1 = Pin(5, Pin.OUT)
echo_pin1 = Pin(18, Pin.IN)

trigger_pin2 = Pin(12, Pin.OUT)
echo_pin2 = Pin(14, Pin.IN)

servo_pin = Pin(13, Pin.OUT)

# Initialize PWM for servo
pwm = PWM(servo_pin)
pwm.freq(50)

# PID constants
#TARGET_DISTANCE = 20.0  # Example target in cm
# Kp = 0.3  # Proportional gain (tune this value)
# Ki = 0.1  # Integral gain (tune this value)
# Kd = 0.05  # Derivative gain (tune this value)

Kp_read = ADC(Pin(32))
Kp_read.atten(ADC.ATTN_11DB)       #Full range: 3.3v

Ki_read = ADC(Pin(34))
Ki_read.atten(ADC.ATTN_11DB)

Kd_read = ADC(Pin(35))
Kd_read.atten(ADC.ATTN_11DB)

# Initialize integral and derivative terms
integral_error = 0.0
last_error = 0.0

def map_value(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def map_pot(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def servo_angle(angle):
    pwm.duty(map_value(angle, 0, 180, 20, 120))

def get_distance(trigger_pin,echo_pin):
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

def pid_controller(target, current, dt):
    global integral_error, last_error
    
    print(Kp,Ki,Kd)
    print(current_distance,TARGET_DISTANCE)
    # Calculate the error
    error = target - current
    
    # Accumulate the integral error
    integral_error += error * dt
    
    # Calculate the derivative of the error
    derivative_error = (error - last_error) / dt if dt > 0 else 0
    
    # Apply the PID control
    control_signal = Kp * error + Ki * integral_error + Kd * derivative_error
    
    # Update last_error for the next cycle
    last_error = error
    
    # Map the control signal to the servo angle
    servo_position = map_value(control_signal, -15, 15, 135, 35)
    
    # Ensure servo angle stays within limits
    servo_position = max(0, min(180, servo_position))
    
    return servo_position

last_time = time.ticks_ms()

while True:
    
    current_distance = round(get_distance(trigger_pin1,echo_pin1),0)
    
    TARGET_DISTANCE=20
    # Calculate time difference in seconds
    current_time = time.ticks_ms()
    dt = time.ticks_diff(current_time, last_time) / 1000.0
    last_time = current_time
    
    Kp_value = Kp_read.read()
    Kp= map_pot(Kp_value, 0, 4095, 0, 0.8)
    Kp = round(Kp, 1)

    Ki_value = Ki_read.read()
    Ki= map_pot(Ki_value, 0, 4095, 0, 0.5)
    Ki = round(Ki, 1)

    Kd_value = Kd_read.read()
    Kd= map_pot(Kd_value, 0, 4095, 0, 0.08)
    Kd = round(Kd, 2)
    
    # Calculate the required servo position using the PID controller
    angle = pid_controller(TARGET_DISTANCE, current_distance, dt)
    
    # Set the servo to the calculated angle
    servo_angle(angle)
    
    # Print the distance and the servo angle for monitoring
    #print("Distance: {:.2f} cm, Servo Angle: {:.2f}".format(current_distance, angle))
    
    # Small delay to allow the system to stabilize
    time.sleep(0.1)


