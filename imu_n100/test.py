import os
import signal
import subprocess
import getch
import time

while True:
	char = getch.getch()
	if char == 's':
		while True:
			cmd = ["ros2","launch","imu_n100","imu.launch.py"]
			proc = subprocess.Popen(cmd)
			char = getch.getch()
			if char == 'a':
				#time.sleep(1)  # maybe needed to wait the process to do something useful
				proc.terminate()
				continue
			else:
				proc.terminate()
				break
		
	else:
		
		break

