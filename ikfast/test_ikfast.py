import pyikfast
import numpy as np

# testing a random location
# print(pyikfast.inverse([0.2239256672224522, 0.0, 0.3999220958863359], 
# 	[-5.1032511549919946e-12, 0.0, 1.0, 0.0, -1.0, 0.0, 1.0, 0.0, 5.1032511549919946e-12]))

# print(pyikfast.forward([0, -0.7853981634, 0, 0, 0, -0.7853981634, 0, 0]))

frontBackAngle = [0.0,-45.0,0.0,0.0,0.0,-45.0,0.0]
angles = list(np.radians(frontBackAngle))

print("start (joints): ", angles)

translate, rotate  = pyikfast.forward(angles)

print("FK (translate, rotate): ", translate, rotate, "\n")

joints = pyikfast.inverse(translate, rotate)

for result in joints:
	print("IK (joints): ", result)

