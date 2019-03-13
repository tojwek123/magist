import time
import numpy as np
import cv2
import pyrealsense as pyrs

with pyrs.Service() as serv:
	with serv.Device() as dev:
	
		dev.apply_ivcam_preset(0)
		last = time.time()
	
		while True:
			dev.wait_for_frames()
			c = dev.color
			c = cv2.cvtColor(c, cv2.COLOR_RGB2BGR)
			d = dev.depth * dev.depth_scale
			#d = cv2.applyColorMap(d.astype(np.uint8), cv2.COLORMAP_RAINBOW)
		
			#cd = np.concatenate((c, d), axis=1)
			#cv2.putText(cd, str(fps_smooth)[:4], (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0))
			
			cv2.imshow('d', d)
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break
