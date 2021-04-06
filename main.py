import numpy as np
import matplotlib.pyplot as plt
from casadi import *
from casadi.tools import *
import pdb
import sys
sys.path.append('../../')
import do_mpc
from do_mpc.tools.timer import Timer

import matplotlib as mpl

import pickle
import time


from template_model import template_model
from template_mpc import template_mpc
from template_simulator import template_simulator
from do_mpc.data import save_results, load_results

""" User settings: """
show_animation = True
store_results = False


model = template_model()
mpc = template_mpc(model)
simulator = template_simulator(model)
estimator = do_mpc.estimator.StateFeedback(model)

x0 = np.array([0.0, 0.0, 0.0]).reshape(-1,1)
mpc.x0 = x0
simulator.x0 = x0

mpc.set_initial_guess()

mpl.rcParams['font.size'] = 18
mpl.rcParams['lines.linewidth'] = 3
mpl.rcParams['axes.grid'] = True

mpc_graphics = do_mpc.graphics.Graphics(mpc.data)
sim_graphics = do_mpc.graphics.Graphics(simulator.data)


pos_x = []
pos_y = []
car_v = []
car_delta = []

for k in range(50):
    u0 = mpc.make_step(x0)
    car_delta.append(float(u0[1]))
    y_next = simulator.make_step(u0)
    x0 = estimator.make_step(y_next)
    pos_x.append(float(x0[0]))
    pos_y.append(float(x0[1]))


#print(pos_y)
#print(pos_x)
#print(pos_x)

plt.plot(pos_x, pos_y)
plt.show()


