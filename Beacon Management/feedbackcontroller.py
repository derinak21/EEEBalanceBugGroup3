from machine import Pin, ADC, PWM
import time

va = 0.0
vb = 0.0
vret = 0.0
measured_current = 0.0
setpoint = 0.2

pwm = PWM(Pin(0))
pwm.freq(100000)
pwm_en = Pin(1, Pin.OUT)
pwm_out = 0
pwm_ref = 10000


def saturate(duty):
    if duty > 62500:
        duty = 62500
    if duty < 100:
        duty = 100
    return duty
    
pwm_out = saturate(pwm_ref)
pwm.duty_u16(pwm_out)
    
while True:
    pwm_en.value(1)
    
    
    vret_pin = ADC(Pin(26))
    vret_16 = vret_pin.read_u16()
    vret = vret_16 * (3.33 / 12979)
    
    #setpoint is set to 0.2A
    measured_current = vret / 1.02 #find output current
    
    #calculate and round the differenc between
    #setpoint and measured value by 2dp
    error = setpoint - measured_current 
    error_rounded = round(error,2)
    
    #if else statements to alter the duty
    #cycle depending on the error value
    if error_rounded > 0:
        pwm_out += 100
    elif error_rounded < 0:
        pwm_out -= 100
    
    #saturates duty cycle between 0 and 62500
    pwm_ref = saturate(pwm_out)
    #apply changed duty cycle to the pwm signal
    pwm.duty_u16(pwm_ref)
   
    print("measured_current", measured_current)
    print("setpoint", setpoint)
    print("error: ", error)
    print("error rounded:", error_rounded)
    print("measured_current", measured_current)
    print("pwm_out", pwm_out)
    print("pwm_ref", pwm_ref)
    print(" ")