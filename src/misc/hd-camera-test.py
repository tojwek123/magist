import cv2
import numpy as np
import os
import v4l2capture
import select
import time



class Camera():
	
	RGB_HD          = 0
	RGB_REALSENSE   = 1
	DEPTH_REALSENSE = 2
	GRAY_BOTTOM     = 3

	_Config = {
		RGB_HD          : { 'dev': '/dev/video13', 'shape': (1080, 1920, 3) },
		RGB_REALSENSE   : { 'dev': None          , 'shape': (0, 0, 3)       },
		DEPTH_REALSENSE : { 'dev': None          , 'shape': (0, 0, 3)       }, 
		GRAY_BOTTOM     : { 'dev': None          , 'shape': (0, 0, 3)       }
	}

	def __init__(self, camera_type):
		self._camera_type = camera_type
		self._video_device = v4l2capture.Video_device(Camera._Config[camera_type]['dev'])
		
		shape = Camera._Config[camera_type]['shape']
		actual_width, actual_height = self._video_device.set_format(shape[1], shape[0])
		self._shape = (actual_height, actual_width, shape[2])		

		self._video_device.create_buffers(1)
		self._video_device.queue_all_buffers()
		self._video_device.start()
		
		#Dummy read first frame as it sometimes fails
		try:
			self.read()
		except BlockingIOError:
			pass

	def read(self):
		select.select((self._video_device,), (), ())
		raw = self._video_device.read_and_queue()
		frame = np.frombuffer(raw, dtype=np.uint8)
		frame = np.reshape(frame, self._shape)
		
		return frame

def main():

	rgb_hd = Camera(Camera.RGB_HD)

	while True:
		frame = rgb_hd.read()
		frame = cv2.resize(frame, (640, 480))
		frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
		cv2.imshow('', frame)
		key = cv2.waitKey(1)
		
		if key & 0xFF == ord('q'):
			break	

	#video = v4l2capture.Video_device('/dev/video13')
	#size_x, size_y = video.set_format(1920, 1080)
	#print('Video size {}x{}'.format(size_x, size_y))
	
	#video.create_buffers(2)
	#video.queue_all_buffers()

	#print('Starting capture...')
	#video.start()

#	while True:
#		select.select((video,), (), ())
#		
#		try:
#			image_data = video.read_and_queue()
#		except BlockingIOError:
#			print('dd')
#			continue
#		frame = np.frombuffer(image_data, dtype=np.uint8)
#		frame = np.reshape(frame, (size_y, size_x, 3))
#		frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
#		frame = cv2.resize(frame, (800, 600))		

		#frame = cv2.imdecode(np.frombuffer(image_data, dtype=np.uint8), cv2.IMREAD_COLOR)
		
#		cv2.imshow('', frame)
#		key = cv2.waitKey(1)
#
#		if key & 0xFF == ord('q'):
#			break	
#

if __name__ == '__main__':
	main()
