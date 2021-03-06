import matplotlib.pyplot as plt
import numpy as np

from casadi import *
from casadi.tools import *
from scipy.optimize import minimize



def objective(x): # Objective function tries to simply minimize constants a,b,c in ax+by+c >= 0
    return x[0] + x[1]


def constraint_ego_car(x, a, b): #constraint that car coordinates (x_c, y_c) >= ax + by +c >= 0
    return b - (x[0] * a + x[1])

def constraint_opp_car(x, a, b): #constraint that car coordinates (x_c, y_c) >= ax + by +c >= 0
    return (x[0] * a + x[1]) - b



def find_constraint(ego_x, ego_y, opp_x, opp_y):



    x_car = ego_x
    y_car = ego_y


# initial guesses
    n = 2
    x0 = np.zeros(n)
    x0[0] = 1.0
    x0[1] = 5.0

    arguments = (x_car, y_car)
    arguments2 = (opp_x, opp_y)
    #arguments2 = (opp_x1, opp_x2, opp_y1, opp_y2)


# optimize
    b = (-15, 15)
    bnds = (b, b)
    con0 = {'type': 'ineq', 'args': arguments, 'fun': constraint_ego_car}
    con1 = {'type': 'ineq',  'args': arguments2, 'fun': constraint_opp_car}

    cons = ([con0, con1]) 
    solution = minimize(objective,x0,method='SLSQP',\
                    bounds=bnds,constraints=cons)
    x = solution.x

    return x 


