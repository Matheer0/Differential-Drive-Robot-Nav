import numpy as np
from utils import *
from copy import deepcopy
from scipy.optimize import minimize


class MPC:
	def __init__(self, horizon):
		self.horizon = horizon
		self.R = np.diag([0.01, 0.01])                 # input cost matrix
		self.Rd = np.diag([0.01, 1.0])                 # input difference cost matrix
		self.Q = np.diag([1.0, 1.0])                   # state cost matrix
		self.Qf = self.Q							   # state final matrix

	def cost(self, u_k, car, path):
		path = np.array(path)
		controller_car = deepcopy(car)
		u_k = u_k.reshape(self.horizon, 2).T
		z_k = np.zeros((2, self.horizon+1))

		desired_state = path.T

		cost = 0.0

		for i in range(self.horizon):
			controller_car.set_robot_velocity(u_k[0,i], u_k[1,i])
			controller_car.update(0.5)
			x, _ = controller_car.get_state()
			z_k[:,i] = [x[0, 0], x[1, 0]]
			cost += np.sum(self.R@(u_k[:,i]**2))
			cost += np.sum(self.Q@((desired_state[:,i]-z_k[:,i])**2))
			if i < (self.horizon-1):     
				cost += np.sum(self.Rd@((u_k[:,i+1] - u_k[:,i])**2))

		return cost

	def optimize(self, car, reference_point_array):
		bnd = [(0, 5),(np.deg2rad(-60), np.deg2rad(60))]*self.horizon

		# make points to tuple
		reference_point_list = []
		for i in range(self.horizon):
			reference_point_list.append([reference_point_array[0, i], reference_point_array[1, i]])

		result = minimize(self.cost, args=(car, reference_point_list), x0 = np.zeros((2*self.horizon)), method='SLSQP', bounds = bnd)
		return result.x[0],  result.x[1]
