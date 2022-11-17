from math import acos, sqrt, radians, sin


def norm(a):
	base = sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2])
	return base

def normed(a):
	output = [0, 0, 0]
	base = norm(a)
	output[0] = a[0] / base
	output[1] = a[1] / base
	output[2] = a[2] / base
	return output


def cross_product(u, v):
	u1 = u[0]
	u2 = u[1]
	u3 = u[2]
	v1 = v[0]
	v2 = v[1]
	v3 = v[2]

	s = [ (u2*v3-u3*v2), (u3*v1-u1*v3), (u1*v2-u2*v1) ]

	return s

def rotmat2rotvec(rotmat):
		# array to matrix
	r11 = rotmat[0]
	r21 = rotmat[1]
	r31 = rotmat[2]
	r12 = rotmat[3]
	r22 = rotmat[4]
	r32 = rotmat[5]
	r13 = rotmat[6]
	r23 = rotmat[7]
	r33 = rotmat[8]

	# rotation matrix to rotation vector
	theta = acos((r11+r22+r33-1)/2)
	sth = sin(theta)

	if ((theta > radians(179.99)) or (theta < radians(-179.99))):
		theta = radians(180)
		if (r21 < 0):
			if (r31 < 0):
				ux = sqrt((r11+1)/2)
				uy = -sqrt((r22+1)/2)
				uz = -sqrt((r33+1)/2)
			else:
				ux = sqrt((r11+1)/2)
				uy = -sqrt((r22+1)/2)
				uz = sqrt((r33+1)/2)
		else:
			if (r31 < 0):
				ux = sqrt((r11+1)/2)
				uy = sqrt((r22+1)/2)
				uz = -sqrt((r33+1)/2)
			else:
				ux = sqrt((r11+1)/2)
				uy = sqrt((r22+1)/2)
				uz = sqrt((r33+1)/2)
	else:
		ux = (r32-r23)/(2*sth)
		uy = (r13-r31)/(2*sth)
		uz = (r21-r12)/(2*sth)

	rotvec = [(theta*ux),(theta*uy),(theta*uz)]

	return rotvec



def get_feature_plane(p1, p2, p3):

	# Step 1. Get the direction vectors
	d12 = [ p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2] ]
	d13 = [ p3[0]-p1[0], p3[1]-p1[1], p3[2]-p1[2] ]

	# Step 2. Get the direction vector of Z axis by cross product of d12 and d13
	dz = cross_product(d12, d13)

	# Step 3. Get the X and Z unit direction vectors by normalizing d12 and dz
	temp = norm(d12)
	ux = [ d12[0]/temp, d12[1]/temp, d12[2]/temp ]
	temp = norm(dz)
	uz = [ dz[0]/temp, dz[1]/temp, dz[2]/temp ]

	# Step 4. Get Y unit direction vector by cross product of uz and ux
	uy = cross_product(uz, ux)

	# Step 5. Get the rotation matrix from the unit direction vectors
	rotmat = [ ux[0], ux[1], ux[2], uy[0], uy[1], uy[2], uz[0], uz[1], uz[2] ]

	# Step 6. Get the rotation vector from the rotation matrix
	rotvec = rotmat2rotvec(rotmat)

	# Step 7. Get the feature plane with the origin at p1 and the frame achieved at step 6
	feature_plane = [ p1[0], p1[1], p1[2], rotvec[0], rotvec[1], rotvec[2] ]

	return feature_plane


topleft = [222.7, -256.0, 110.1, 180, 0, 0]
botleft = [557.2, -256.0, 110.1, 180.0, 0.0, 0.0]
topright = [240.7, 250.0, 110.1, 180.0, 0.0, 0.0]

plane = get_feature_plane(topleft, topright, botleft)	
print(plane)

