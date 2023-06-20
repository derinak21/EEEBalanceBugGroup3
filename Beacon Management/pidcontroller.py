from machine import Pin, ADC, PWM
import time
import network

#ssid = 'Rujena'
#password = 'getyourownfuckingwifi'

#wlan = network.WLAN(network.STA_IF)
#wlan.active(True)
#wlan.connect(ssid, password)

# Wait for connect or fail
#max_wait = 10

va = 0.0
vb = 0.0
vret = 0.0
measured_current = 0.0
setpoint = 0.1

Kp = 1000
Ki = 900
Kd = 800

pwm = PWM(Pin(0))
pwm.freq(100000)
pwm_en = Pin(1, Pin.OUT)
pwm_out = 0

#desired_interval = 100

class PIDController:
    def _init_(self, setpoint, Kp, Ki, Kd):
        self.setpoint = setpoint
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_error = 0.0
        self.integral = 0.0

    def compute(self, current):
        #calculation of error
        error = self.setpoint - current
        #calculation of P
        P = self.Kp * error
        
        #calculation of I
        self.integral += error
        I = self.Ki * self.integral
        
        #calculation of D
        derivative = error - self.last_error      
        D = self.Kd * derivative
        
        #PID is combined
        output = P + I + D
        
        #current error is saved for the next loop
        self.last_error = error
        
        return output

def saturate(duty):
    if duty > 62500:
        duty = 62500
    if duty < 100:
        duty = 100
    return duty
    
#while max_wait > 0:
 #   if wlan.status() < 0 or wlan.status() >= 3:
  #      break
   # max_wait -= 1
    #print('waiting for connection...')
    #time.sleep(1)

# Handle connection error
#if wlan.status() != 3:
 #   raise RuntimeError('network connection failed')
#else:
 #   print('connected')
  #  status = wlan.ifconfig()
   # print( 'ip = ' + status[0] )

pid = PIDController(setpoint, Kp, Ki, Kd)
    
#start_time = time.ticks_ms()
    
while True:
    pwm_en.value(1)

    vret_pin = ADC(Pin(26))
    vret_16 = vret_pin.read_u16()
    vret = vret_16 * (3.33 / 12979)
    
    measured_current = vret / 1.02
    
    output = pid.compute(measured_current)
    
    pwm_out = saturate(output)
    print("output", output)
    print("pwm out" ,pwm_out)
    print("current", measured_current)
    
    pwm.duty_u16(int(pwm_out))
    
    #elapsed_time = time.ticks_diff(time.ticks_ms(), start_time)
    #if elapsed_time >= desired_interval:
     #   start_time = time.ticks_ms()