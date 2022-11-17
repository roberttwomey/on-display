from math import sqrt, atan2, sin, cos

def global_plane_finder(p1, p2, p3):
	V12 = [p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]]
	V13 = [p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]]
	VZ = mycross(V12, V13)
	Tmat = pose2Tmat(p1)
	VZofp1 = [Tmat[2], Tmat[6], Tmat[10]]
	value = mydot(VZ, VZofp1)
	if (value < -0.00001):
		VZ = mycross(V13, V12)
	VZ = myunitised(VZ)
	VY = myunitised(V12)
	VX = mycross(VY,VZ)
	output = [VX[0], VY[0], VZ[0], p1[0], VX[1], VY[1], VZ[1], p1[1], VX[2], VY[2], VZ[2], p1[2], 0, 0, 0, 1]
	pose = Tmat2pose(output)
	return pose

def mycross(a, b):
	output = [0, 0, 0]
	output[0] = a[1]*b[2] - a[2]*b[1]
	output[1] = a[2]*b[0] - a[0]*b[2]
	output[2] = a[0]*b[1] - a[1]*b[0]
	return output

def Tmat2pose(Tmat):
	output = p[0, 0, 0, 0, 0, 0]
	output[0] = Tmat[3]
	output[1] = Tmat[7]
	output[2] = Tmat[11]
	sy = mynorm([Tmat[0], Tmat[4], 0])
	if (sy > 0.00001):
		x = atan2(Tmat[9], Tmat[10])
		y = atan2(-Tmat[8], sy)
		z = atan2(Tmat[4], Tmat[0])
	else:
		x = atan2(-Tmat[6], Tmat[5])
		y = atan2(-Tmat[8], sy)
		z = 0
	rotvec = rpy2rotvec([x, y, z])
	output[3] = rotvec[0]
	output[4] = rotvec[1]
	output[5] = rotvec[2]
	return output

def pose2Tmat(pose):
	if (pose[3]) == 0 and (pose[4] == 0) and (pose[5] == (0)):
		Rmat = [1, 0, 0, 0, 1, 0, 0, 0, 1]
	else:
		x = pose[3]
		y = pose[4]
		z = pose[5]
		ang = mynorm([x, y, z])
		x = x/ang
		y = y/ang
		z = z/ang
		s = sin(ang)
		c = cos(ang)
		t = 1 - c
		Rmat = [c+(txx), (txy)-(sz), (txz)+(sy), (txy)+(sz), c+(tyy), (tyz)-(sx), (txz)-(sy), (tyz)+(sx), c+(tzz)]
	Tmat = [Rmat[0], Rmat[1], Rmat[2], pose[0], Rmat[3], Rmat[4], Rmat[5], pose[1], Rmat[6], Rmat[7], Rmat[8], pose[2]]
	return Tmat

def mynorm(a):
	base = sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2])
	return base

def myunitised(a):
	output = [0, 0, 0]
	base = mynorm(a)
	output[0] = a[0] / base
	output[1] = a[1] / base
	output[2] = a[2] / base
	return output

def mydot(a, b):
	output = a[0]*b[0] + a[1]*b[1] + a[2]*b[2]
	return output


topleft = [222.7, -256.0, 110.1, 180, 0, 0]
botleft = [557.2, -256.0, 110.1, 180.0, 0.0, 0.0]
topright = [240.7, 250.0, 110.1, 180.0, 0.0, 0.0]

global_plane_finder(topleft, topright, botleft)	