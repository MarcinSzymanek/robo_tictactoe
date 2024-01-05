def invkin(xyz):
	"""
	Python implementation of the the inverse kinematics for the crustcrawler
	Input: xyz position
	Output: Angels for each joint: q1,q2,q3,q4
	
	You might adjust parameters (d1,a1,a2,d4).
	The robot model shown in rviz can be adjusted accordingly by editing au_crustcrawler_ax12.urdf
	"""

	d1 = 16.7 # cm (height of 2nd joint)
	a1 = 5.5 # (distance along "y-axis" to 2nd joint)
	a2 = 17.3 # (distance between 2nd and 3rd joints)
	d4 = 15.0 # (distance from 3rd joint to gripper center - all inclusive, ie. also 4th joint)

	# Insert code here!!!
	# Calculate oc
	oc = xyz
	xc = oc[0]
	yc = oc[1]-a1
	zc = oc[2]-d1
	direction = 0
	
	
	# Calculate q1
	if (xc > 0) & (yc > 0):
		q1 = math.atan2(yc, xc)-math.pi/2
	elif (xc > 0) & (yc < 0):
		q1 = -(math.pi/2-math.atan2(yc, xc))
	elif (xc < 0) & (yc > 0):
		q1 = math.atan2(yc, xc)-math.pi/2
	elif (xc < 0) & (yc < 0):
		q1 = -math.atan2(yc, xc)
	elif (xc == 0) & (yc > 0):
		q1 = 0
	elif (xc == 0) & (yc < 0):
		q1 = math.pi
	elif (xc > 0) & (yc == 0):
		q1 = -math.pi/2
	elif (xc < 0) & (yc == 0):
		q1 = math.pi/2
	elif (xc == 0) & (yc == 0):
		q1 = 0

	if (q1 > (5*math.pi/6)):
		q1 = q1-math.pi 
		direction = 1
	elif (q1 < -(5*math.pi/6)):
		q1 = q1+math.pi
		direction = 1
	else:
		None

	# Calculate q3 
	cosXY = (((xc**2+yc**2)+zc**2)-a2**2-d4**2)/(-2*a2*d4)
	q3 = -(math.pi-math.atan2(math.sqrt(abs(1-cosXY**2)),cosXY))

	if (q3 > 2.05):
		q3 = 2.05
	elif (q3 < -2.05):
		q3 = -2.05
	else:
		None

	if direction == 1:
		q3 = -q3
	else: 
		None

	# Calculate q2 
	q2 = math.atan2(zc,math.sqrt(xc**2+yc**2))-math.atan2((d4*math.sin(q3)),(a2+d4*math.cos(q3)))-math.pi/2

	if (q2 > 2.05):
		q2 = 2.05
	elif (q2 < -2.05):
		q2 = -2.05
	else:
		None

	if direction == 1:
		q2 = -q2
	else: 
		None

	
	#r2 = (xc-a1*math.cos(q1))**2+(yc-a1*math.sin(q1))**2
	#s = (zc-d1)
	#d = (r2+s**2-a2**2-d4**2)/(2*a2*d4)
	#q3 = math.pi - math.atan2(-math.sqrt(1-d**2), d)
	#q2 = math.atan2(s,math.sqrt(r2))-math.atan2(d4*math.sin(q3), a2+d4*math.cos(q3))
	
	# Calculate q4
	q4 = 0.0

	return q1,q2,q3,q4