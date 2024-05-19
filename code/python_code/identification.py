import matplotlib
matplotlib.use("TkAgg", force=True)
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from scipy.optimize import minimize
from scipy.interpolate import interp1d
import pandas as pd
import control as ct








def fotd_system_identification(filename='prbs_open_exp.csv'):
    try:
        data = pd.read_csv(filename)
    except:
        filename = 'http://apmonitor.com/pdc/uploads/Main/tclab_data2.txt'
        data = pd.read_csv(filename)


    # defining the vector of initial conditions
    x0 = np.zeros(4)
    x0[0] = 1  # Kp
    x0[1] = 60.0  # taup
    x0[2] = 5.0  # thetap
    x0[3] = 0.2  # thetap

    # reading the variables from experiment
    t = data['t'].values - data['t'].values[0]
    U = data['u'].values
    Y = data['y'].values

    y0 = Y[0]

    # detrending the input and the output by substracting the operation point
    Ud = U - U[0]
    Yd = Y - Y[0]

    t = t[1:-1]
    t = t - t[0]
    Ud = Ud[1: -1]
    Yd = Yd[1: -1]

    # The environment temperature perturbation input translated to operation point
    Tenv = 25 - Y[0];


    # new initial conditions after detrending
    y0d = Yd[0]
    u0d = Ud[0]


     # specify number of steps
    ns = len(t)

    # create linear interpolation of the u data versus time

    Uf = interp1d(t, Ud)
    print(Uf(0))


    def fopdt(Ys, t, Uf, kp, taup, thetap, kd):
        #  T      = states
        #  t      = time
        #  Qf1    = input linear function (for time shift)
        #  Qf2    = input linear function (for time shift)
        #  Kp     = model gain
        #  Kd     = disturbance gain
        #  taup   = model time constant
        #  thetap = model time constant
        #  time-shift Q
        try:
            if (t - thetap) <= 0:
                Um = Uf(0.0)
            else:
                Um = Uf(t - thetap)

        except:
            Um = u0d
        # calculate derivative
        dYdt = (-Ys[0]  + kp *Um  +  kd * Tenv) / taup

        return dYdt


    def sim_model(x):
        # input arguments
        kp, taup, thetap, kd = x
        # storage for model values
        Yp = np.ones(ns) * y0d

        # loop through time steps
        for i in range(0, ns - 1):
            ts = [t[i], t[i + 1]]
            Ystep  = odeint(fopdt, Yp[i], ts, args=(Uf, kp, taup, thetap,  kd))
            Yp[i + 1] = Ystep[-1, 0]
        return Yp

    def objective(x):
        # simulate model
        Yp = sim_model(x)
        # return objective
        return sum(np.abs(Yp - Yd))

    bnds = ((0.5, 1.5), (40.0, 120.0), (0.0, 10), (-10.0, 10.0))
    print('Initial SSE Objective: ' + str(objective(x0)))
    print("starting optimization")
    solution = minimize(objective, x0=x0, bounds=bnds,  method='SLSQP')
    x_end = solution.x
    kp, taup, thetap, kd = x_end
    print('Final SSE Objective: ' + str(objective(x_end)))
    print('Kp: ' + str(kp))
    print('taup: ' + str(taup))
    print('thetap: ' + str(thetap))
    print('Test: ' + str(kd))
    Y1p = sim_model(x_end)
    plt.figure(1, figsize=(15, 7))
    plt.subplot(2, 1, 1)
    plt.plot(t, Yd, 'r', linewidth=1, label='Measured temperature')
    plt.plot(t, Y1p, 'b--', linewidth=1, label='Predicted Temperature')
    plt.subplot(2, 1, 2)
    plt.plot(t, U, 'b--', linewidth=1, label=r'Heater 1 ($Q_1$)')
    plt.show()
    return x_end

if __name__ == "__main__":
    x = fotd_system_identification(filename='prbs_open_exp.csv')
    print(x)

