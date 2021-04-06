import numpy as np
from casadi import *
from casadi.tools import *
import pdb
import sys
sys.path.append('../../')
import do_mpc


def template_simulator(model):

    simulator = do_mpc.simulator.Simulator(model)

    params_simulator = {
        't_step': 0.1
    }

    simulator.set_param(**params_simulator)

    simulator.setup()



    return simulator