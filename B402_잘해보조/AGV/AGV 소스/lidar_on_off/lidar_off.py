import subprocess
import os, time

if os.name == "posix":
    import RPi.GPIO as GPIO

def radar_close(run_launch):
    def radar_low():
        GPIO.setmode(GPIO.BCM)
        time.sleep(0.1)
        GPIO.setup(20, GPIO.OUT)
        GPIO.output(20, GPIO.LOW)

    radar_low()
    time.sleep(0.05)

    close_command = "ps -ef | grep -E " + run_launch + " | grep -v 'grep' | awk '{print $2}' | xargs kill -2"
    subprocess.run(close_command, shell=True)



radar_close('myagv_active.launch')