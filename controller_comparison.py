from car import Car
from draw import Draw
from controllers import MPC
import cv2
from utils import *
import numpy as np


W, H = 700, 700
draw = Draw(W, H, window_name = "Canvas")

car_mpc = Car(150, 150)

horizon = 5
controller_mpc = MPC(horizon = horizon)

#
# Newly Added
#

# Construct Referecen Trajectory
initial_x = 200
final_x = 600
increment = 2

state_dimension = 3    # x, y and theta
data_size = int((final_x - initial_x)/increment)


way_points = np.zeros((state_dimension, data_size), int)


# 3D point
for i in range(0, data_size):
	x = initial_x + increment * i
	y = H/2 + 200*np.sin(2*np.pi*0.25*(x + 200)/100)
	# way_points.append([x, int(y)])
	way_points[0, i] = x
	way_points[1, i] = int(y)




'''
way_points = []
for x in range(initial_x, final_x, increment):
	
	y = H/2 + 200*np.sin(2*np.pi*0.25*(x + 200)/100)
	way_points.append([x, int(y)])

'''




lw = 0
rw = 0
current_idx_mpc = 0
current_idx_pid = 0
linear_v = 0
angular_v = 0

mpc_car_points = []
pid_car_points = []

while True:
	# print(current_idx_mpc)
	draw.clear()



	# show reference trajectory
	if data_size>0:
		draw.draw_reference(way_points, color = (255, 0, 0), thickness = 1)

	if len(mpc_car_points)>0:
		draw.draw_path(mpc_car_points, color = (0, 255, 0), thickness = 1, dotted = True)


	draw.draw(car_mpc.get_points(), color = (0, 255, 0), thickness = 1)
	# draw.draw(car_pid.get_points(), color = (0, 0, 255), thickness = 1)

	draw.add_text("MPC Controller", color = (0, 255, 0), fontScale = 0.5, thickness = 1, org = (100, 75))
	draw.add_text("Trajectory", color = (255, 0, 0), fontScale = 0.5, thickness = 1, org = (100, 100))
	

	k = draw.show()

	

	# print(car_mpc.x_dot[2, 0])

	# MPC Car
	x, _ = car_mpc.get_state()

	if len(way_points[1])>0 and current_idx_mpc != len(way_points[1]):
		mpc_car_points.append([int(x[0, 0]), int(x[1, 0])])
		goal_pt = way_points[: , current_idx_mpc]
		# print(way_points[:, current_idx_mpc:current_idx_mpc+horizon])

		# points = [[200, 349], [202, 356], [204, 362], [206, 368], [208, 375]]
		linear_v, angular_v = controller_mpc.optimize(car = car_mpc, reference_point_array = way_points[:, current_idx_mpc:current_idx_mpc+horizon])


		dist = get_distance(x[0, 0], x[1, 0], goal_pt[0], goal_pt[1])
		# dist = get_distance(x[0, 0], x[1, 0], goal_pt[0, 0], goal_pt[1, 0])
		if dist<10:
			current_idx_mpc+= 1
	else:
		
		linear_v = 0
		angular_v = 0
	car_mpc.set_robot_velocity(linear_v, angular_v)
	car_mpc.update(0.5)
