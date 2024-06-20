import subprocess
import os, time

if os.name == "posix":
    import RPi.GPIO as GPIO

def radar_open():
    def radar_high():
        GPIO.setmode(GPIO.BCM)
        time.sleep(0.1)
        GPIO.setup(20, GPIO.OUT)
        GPIO.output(20, GPIO.HIGH)

    radar_high()
    time.sleep(0.05)
    launch_command = "roslaunch myagv_odometry myagv_active.launch"  # 使用ros 打开
    subprocess.run(['gnome-terminal', '-e', f"bash -c '{launch_command}; exec $SHELL'"])

radar_open()