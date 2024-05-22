import matplotlib
matplotlib.use("TkAgg", force=True)
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from scipy.optimize import minimize
from scipy.interpolate import interp1d
from scipy.stats import linregress
import pandas as pd
import control as ct
from thermal_iot import ThermalSystemIoT, PATH, long2hex, float2hex, hex2long, set_pid, hex2float
import json
from math import ceil
from queue import Queue
from pathlib import Path
import csv
# def fotd_system_identification(filename='prbs_open_exp.csv'):
#     try:
#         data = pd.read_csv(filename)
#     except:
#         filename = 'http://apmonitor.com/pdc/uploads/Main/tclab_data2.txt'
#         data = pd.read_csv(filename)
#
#
#     # defining the vector of initial conditions
#     x0 = np.zeros(4)
#     x0[0] = 1  # Kp
#     x0[1] = 60.0  # taup
#     x0[2] = 5.0  # thetap
#     x0[3] = 0.2  # thetap
#
#     # reading the variables from experiment
#     t = data['t'].values - data['t'].values[0]
#     U = data['u'].values
#     Y = data['y'].values
#
#     y0 = Y[0]
#
#     # detrending the input and the output by substracting the operation point
#     Ud = U - U[0]
#     Yd = Y - Y[0]
#
#     t = t[1:-1]
#     t = t - t[0]
#     Ud = Ud[1: -1]
#     Yd = Yd[1: -1]
#
#     # The environment temperature perturbation input translated to operation point
#     Tenv = 25 - Y[0];
#
#
#     # new initial conditions after detrending
#     y0d = Yd[0]
#     u0d = Ud[0]
#
#
#      # specify number of steps
#     ns = len(t)
#
#     # create linear interpolation of the u data versus time
#
#     Uf = interp1d(t, Ud)
#     print(Uf(0))
#
#
#     def fopdt(Ys, t, Uf, kp, taup, thetap, kd):
#         #  T      = states
#         #  t      = time
#         #  Qf1    = input linear function (for time shift)
#         #  Qf2    = input linear function (for time shift)
#         #  Kp     = model gain
#         #  Kd     = disturbance gain
#         #  taup   = model time constant
#         #  thetap = model time constant
#         #  time-shift Q
#         try:
#             if (t - thetap) <= 0:
#                 Um = Uf(0.0)
#             else:
#                 Um = Uf(t - thetap)
#
#         except:
#             Um = u0d
#         # calculate derivative
#         dYdt = (-Ys[0]  + kp *Um  +  kd * Tenv) / taup
#
#         return dYdt
#
#
#     def sim_model(x):
#         # input arguments
#         kp, taup, thetap, kd = x
#         # storage for model values
#         Yp = np.ones(ns) * y0d
#
#         # loop through time steps
#         for i in range(0, ns - 1):
#             ts = [t[i], t[i + 1]]
#             Ystep  = odeint(fopdt, Yp[i], ts, args=(Uf, kp, taup, thetap,  kd))
#             Yp[i + 1] = Ystep[-1, 0]
#         return Yp
#
#     def objective(x):
#         # simulate model
#         Yp = sim_model(x)
#         # return objective
#         return sum(np.abs(Yp - Yd))
#
#     bnds = ((0.5, 1.5), (40.0, 120.0), (0.0, 10), (-10.0, 10.0))
#     print('Initial SSE Objective: ' + str(objective(x0)))
#     print("starting optimization")
#     solution = minimize(objective, x0=x0, bounds=bnds,  method='SLSQP')
#     x_end = solution.x
#     kp, taup, thetap, kd = x_end
#     print('Final SSE Objective: ' + str(objective(x_end)))
#     print('Kp: ' + str(kp))
#     print('taup: ' + str(taup))
#     print('thetap: ' + str(thetap))
#     print('Test: ' + str(kd))
#     Y1p = sim_model(x_end)
#     plt.figure(1, figsize=(15, 7))
#     plt.subplot(2, 1, 1)
#     plt.plot(t, Yd, 'r', linewidth=1, label='Measured temperature')
#     plt.plot(t, Y1p, 'b--', linewidth=1, label='Predicted Temperature')
#     plt.subplot(2, 1, 2)
#     plt.plot(t, U, 'b--', linewidth=1, label=r'Heater 1 ($Q_1$)')
#     plt.show()
#     return x_end



def read_csv_file3(filepath=PATH + 'DCmotor_prbs_open_exp.csv'):
    with open(filepath , newline='') as file:
        reader = csv.reader(file)
        # Iterate over each row in the CSV file
        num_line = 0
        t = []
        u = []
        y = []
        for row in reader:
            if num_line != 0:
               t.append(float(row[0]))
               u.append(float(row[1]))
               y.append(float(row[2]))
            num_line += 1
        return t, u, y




def step_closed_staticgain(system,r0=0,r1=40,t0=0,t1=40):
    def step_message(system, userdata, message):
        q.put(message)


    low_val = r0
    high_val = r1
    low_time = t0
    high_time = t1

    topic_pub = system.codes["USER_SYS_STEP_CLOSED"]
    topic_sub = system.codes["SYS_USER_SIGNALS_CLOSED"]
    sampling_time = system.codes["THERMAL_SAMPLING_TIME"]
    points_high = round(high_time / sampling_time)
    points_low = round(low_time / sampling_time)
    points_low_hex = long2hex(points_low)
    points_high_hex = long2hex(points_high)
    low_val_hex = float2hex(low_val)
    high_val_hex = float2hex(high_val)
    message = json.dumps({"low_val": low_val_hex,
                          "high_val": high_val_hex,
                          "points_low": points_low_hex,
                          "points_high": points_high_hex,
                          })

    system.client.on_message = step_message
    system.connect()
    system.subscribe(topic_sub)
    system.publish(topic_pub, message)
    q = Queue()
    n = 0
    y = []
    u = []
    t = []
    ax = plt.gca()
    line_y, = ax.plot(t, y, linestyle = 'solid', color="#0044AA60", linewidth=1)
    line_u, = ax.plot(t, u, linestyle='solid', color="#ff000060", linewidth=1)
    ax.legend([line_y, line_u], [r'Temperature $(~^o C)$', 'Power input ($\%$ of 2.475W)'], fontsize=16 , loc = "lower right")
    points = points_high + points_low
    n = 0
    sync = False
    while n <= points:
        try:
            message = q.get(True, 20 * sampling_time)
        except:
            raise TimeoutError("The connection has been lost. Please try again")

        decoded_message = str(message.payload.decode("utf-8"))
        msg_dict = json.loads(decoded_message)
        n_hex = str(msg_dict["np"])
        n = hex2long(n_hex)
        if n==0:
            sync = True
        if sync:
            t_curr = n * sampling_time
            t.append(t_curr)
            y_curr = hex2float(msg_dict["y"])
            y.append(y_curr)
            u_curr = hex2float(msg_dict["u"])
            u.append(u_curr)
            line_y.set_data(t, y)
            line_u.set_data(t, u)
            plt.draw()
            plt.pause(sampling_time)
    line_y.set_data([], [])
    line_u.set_data([], [])
    system.disconnect()
    return u, y




def get_static_model(system, step = 5):

    # This is the configuration for the figure displayed while acquiring data
    yee = []
    uee = []
    fig, ax = plt.subplots(figsize=(16, 9))
    fig.set_facecolor('#b7c8be')
    ax.set_title('Static gain response experiment for UNThermalSystem')
    ax.set_xlabel(r'Percent of Power ($\%$ of 2.475W) / Seconds for the current experiment')
    ax.set_ylabel(r'Steady state temperature (C)')
    ax.set_facecolor('#f4eed7')
    ax.set_xticks([0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100])
    ax.set_yticks([0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100])
    ax.grid(color='#1a1a1a40', linestyle='--', linewidth=0.25)
    line_exp, = ax.plot(uee, yee, color="#00aa00", linewidth=1, marker=r"$\circ$",markeredgewidth=0.1)
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    exp = []
    y_test = np.arange(30,100, step)
    Path(PATH).mkdir(exist_ok=True)

    for yi in y_test:
        u, y = step_closed_staticgain(system, r0=0, r1=yi, t0=0, t1=60)
        yf = np.mean(y[-15:])
        uf = np.mean(u[-15:])
        exp.append([uf, yf])
        yee.append(yf)
        uee.append(uf)
        line_exp.set_data(uee, yee)
        plt.draw()
        plt.pause(0.1)
        np.savetxt(PATH + "Thermal_static_gain_response.csv", exp, delimiter=",", fmt="%0.8f", comments="",
                   header='u,t')

    res = linregress(uee, yee, alternative='greater')
    m = res.slope
    b = res.intercept
    ymod = m * np.array(uee) + b
    line_mod, = ax.plot(uee, ymod, color="#0088aaff", linewidth=1.5)
    stringmodel = r"Model:  $T_{ee}= m\,u_{ee} + b=$" + f"{m:0.2f}" + r"$\,u_{ee}+$" + f"{b:0.2f}"
    ax.legend([line_exp, line_mod], ['Data', stringmodel], fontsize=16)
    np.savetxt(PATH + "Thermal_static_gain_model.csv", [m,b], delimiter=",", fmt="%0.8f", comments="",
               header='m,b')
    plt.show()
    system.disconnect()
    return



def pbrs_open(system, op_point=50, peak_amp=5, stab_time=60, uee_time=15, divider=35, filepath = "prbs_open_exp.csv"):
    def pbrs_message(system, userdata, message):
        q.put(message)

    topic_pub = system.codes["USER_SYS_PRBS_OPEN"]
    topic_sub = system.codes["SYS_USER_SIGNALS_OPEN"]
    sampling_time = system.codes["THERMAL_SAMPLING_TIME"]
    peak_amp_hex = float2hex(peak_amp)
    op_point_hex = float2hex(op_point)
    stab_points = ceil(stab_time / sampling_time)
    uee_points = ceil(uee_time / sampling_time)
    stab_points_hex = long2hex(stab_points)
    uee_points_hex = long2hex(uee_points)
    divider_hex = long2hex(divider)
    points = divider * 63 + stab_points + uee_points
    message = json.dumps({"peak_amp": peak_amp_hex,
                          "op_point": op_point_hex,
                          "stab_points": stab_points_hex,
                          "uee_points": uee_points_hex,
                          "divider": divider_hex
                          })
    system.client.on_message = pbrs_message
    system.connect()
    system.subscribe(topic_sub)
    system.publish(topic_pub, message)
    q = Queue()


    y = []
    u = []
    t = []
    yt = []
    ut = []
    tt = []
    exp = []
    m = 1.2341015052212259
    b = 24.750915901094388
    uf_est = (op_point - b) / m
    percent = 0.2
    ymax = op_point + m*peak_amp + 2
    ymin = op_point - m*peak_amp - 2
    umax = uf_est + (1 + percent) * peak_amp
    umin =  np.min([0, uf_est - (1 + percent) * peak_amp - 5.0])

    fig, (yax, uax) = plt.subplots(nrows=2, ncols=1, width_ratios=[1], height_ratios=[2, 1], figsize=(16, 9))
    fig.set_facecolor('#b7c4c8f0')
    yax.set_title(f'PRBS identification with {points:d} samples and a duration of {points * sampling_time: 0.2f} seconds')
    yax.set_ylabel(r'Temperature ($~^oC$)')
    yax.grid(True);
    yax.grid(color='#1a1a1a40', linestyle='--', linewidth=0.25)
    yax.set_facecolor('#f4eed7')
    yax.set_xlim(0, sampling_time * points)
    yax.set_ylim(ymin, ymax)
    yax.set_xlabel('Time (s)')
    uax.set_xlabel('Time (s)')
    uax.set_ylabel(r'Power Input ($\%$)')
    uax.grid(True);
    uax.set_facecolor('#d7f4ee')
    uax.grid(color='#1a1a1a40', linestyle='--', linewidth=0.25)
    uax.set_xlim(0, sampling_time * points)
    uax.set_ylim(umin, umax)

    line_y, = yax.plot(t, y, color="#ff6680")
    line_u, = uax.plot(t, u, color="#00d4aa")
    line_yt, = yax.plot(t, yt, color="#d40055")
    line_ut, = uax.plot(t, ut, color="#338000")
    box = dict(boxstyle='round,pad=0.5', facecolor='white', edgecolor='white', alpha=0.75)
    txt1 = yax.text(25, ymin + 1, 'Current Temperature:', fontsize=15, color="#ff6680",ha='left', va='bottom', bbox=box)
    txt2 = uax.text(25, 5, f'Current Input:', fontsize=15, color="#00d4aa",ha='left', va='bottom', bbox=box)
    n = -1
    sync = False
    while n < points:
        try:
            message = q.get(True, 20 * sampling_time)
        except:
            raise TimeoutError("The connection has been lost. Please try again")

        decoded_message = str(message.payload.decode("utf-8"))
        msg_dict = json.loads(decoded_message)
        n_hex = str(msg_dict["np"])
        n = hex2long(n_hex)
        if n == 0:
            sync = True

        if sync == True:
            if n <= stab_points + uee_points:
                t_curr = n*sampling_time
                t.append(t_curr)
                y_curr = hex2float(msg_dict["y"])
                y.append(y_curr)
                u_curr = hex2float(msg_dict["u"])
                u.append(u_curr)
                line_y.set_data(t, y)
                line_u.set_data(t, u)
                txt1.set_text( f'Current Temperature: {y_curr: 0.2f}$~^oC$')
                txt2.set_text( f'Current Input: {u_curr:0.2f}% ({0.02475*u_curr:0.2f} W)')
                if n == stab_points + uee_points:
                    tt.append(t_curr)
                    yt.append(y_curr)
                    ut.append(u_curr)
                    exp.append([0, u_curr, y_curr])
            else:
                tt_curr = n * sampling_time
                if n == stab_points + uee_points + 1:
                    t0 = t_curr
                tt.append(tt_curr)
                yt_curr = hex2float(msg_dict["y"])
                yt.append(yt_curr)
                ut_curr = hex2float(msg_dict["u"])
                ut.append(ut_curr)
                line_yt.set_data(tt, yt)
                line_ut.set_data(tt, ut)
                line_y.set_data(t, y)
                line_u.set_data(t, u)
                txt1.set_color("#d40055")
                txt2.set_color("#338000")
                txt1.set_text( f'Current Temperature: {yt_curr: 0.2f}$~^oC$')
                txt2.set_text( f'Current Input: {ut_curr:0.2f}% ({0.02475*ut_curr:0.2f} W)')
                exp.append([tt_curr-t0, ut_curr, yt_curr])
                np.savetxt(filepath, exp, delimiter=",",
                           fmt="%0.8f", comments="", header='t,u,y')
            plt.draw()
            plt.pause(0.1)
    # PATH1 = r'/home/leonardo/sharefolder/ProyectoSabatico/Reporte/figures/'
    # plt.savefig(PATH1 + "Thermal_pbrs.svg", format="svg", bbox_inches="tight")
    system.disconnect()
    plt.show()
    return t, u, y



def get_models_prbs(system, yop = 50, usefile = True):

    norm = np.linalg.norm

    def simulate_fomodel(x):
        # this function simulates the model
        alpha, tau = x
        s = ct.TransferFunction.s
        G = alpha / (tau*s + 1)
        tsim, ysim = ct.forced_response(G, t, um)
        return G, ysim

    def simulate_somodel(x):
        # this function simulates the model
        alpha, tau1, tau2 = x
        s = ct.TransferFunction.s
        G = alpha / (( tau1*s + 1)*(tau2*s + 1))
        tsim, ysim = ct.forced_response(G, t, um)
        return G, ysim

    def objective_fo(x):
        # simulate model
        G, ysim = simulate_fomodel(x)
        # return objective
        return norm(ysim - ym)

    def objective_so(x):
        # simulate model
        G, ysim = simulate_somodel(x)
        # return objective
        return norm(ysim - ym)


    if  (yop >= 100):
         raise ValueError(f"The maximum temperature for this system is 100 degrees celsius")

    if usefile:
        t,u,y = read_csv_file3(filepath = PATH + 'Thermal_prbs_open_exp.csv')
    else:
        t, u, y =  pbrs_open(system, op_point=50, peak_amp=4, stab_time=60, uee_time=10, divider=35, filepath = PATH + "Thermal_prbs_open_exp.csv")


    m = 1.2341015052212259
    Tenv = y[0] - m * u[0]
    Tenv_op = Tenv - yop
    ymean = np.mean(y)
    um = np.array(u) - u[0]
    ym = np.array(y) - ymean
    """Now we obtain the initial parameters for the model
                            alpha
              G(s) =  ----------------------
                      (tau1*s +1)(tau2*s+1)
    """
    alpha_0 = 1.23
    tau1_0 = 60
    tau2_0 = 10
    x02 = [alpha_0, tau1_0, tau2_0]
    x01 = [alpha_0, tau1_0]
    #
    # # These are the bounds for alpha, tau1 and tau2
    #
    bounds2 = [(0.8, 1.5), (1 , 150), (1, 150)]
    bounds1 = bounds2[0:2]
    #
    # Now we run the optimization algorithm
    print(f'Starting optimization for first order model...\n\t Initial cost function: {objective_fo(x01):.2f}' )
    solution = minimize(objective_fo, x0=x01, bounds= bounds1)
    xmin = solution.x
    alpha, tau = xmin
    fmin = objective_fo(xmin)
    print(f'\t Final cost function: {fmin:.2f}' )
    print(f'alpha={alpha:.2f} \t tau1={tau:.3f}')
    #
    # We compare the experimental data with the simulation model
    G1, ysim1 = simulate_fomodel(xmin)
    r1 = 100*(1 - norm(ym - ysim1) / norm(ym))
    #
    print(f'\n\nStarting optimization for second order model...\n\t Initial cost function: {objective_so(x02):.2f}' )
    solution = minimize(objective_so, x0=x02, bounds= bounds2)
    xmin = solution.x
    alpha2, tau1, tau2 = xmin
    fmin = objective_so(xmin)
    print(f'\t Final cost function: {fmin:.2f}' )
    print(f'alpha={alpha2:.2f} \t tau1={tau1:.3f} \t tau2={tau2:.3f}')

    # We compare the experimental data with the simulation model
    G2, ysim2 = simulate_somodel(xmin)
    r2 = 100*(1 - norm(ym - ysim2) / norm(y - np.mean(y)))

    # we calculate the step response from the model
    # now we compare the model with the experimental data
    fig, (ay, au) = plt.subplots(nrows=2, ncols=1, width_ratios=[1], height_ratios=[5, 1], figsize=(16, 9))
    fig.set_facecolor('#b7c4c8f0')

    # settings for the upper axes, depicting the model and speed data
    #ay.set_title('Data and estimated second order model for UNDCMotor')
    ay.set_ylabel('Speed (Degrees/s)')
    ay.grid(True)
    ay.grid(color='#1a1a1a40', linestyle='--', linewidth=0.125)
    ay.set_facecolor('#f4eed7')
    xlimits = [60, t[-1]]
    ay.set_xlim(xlimits[0], xlimits[1])
    ay.set_ylabel('Speed (Degrees/s)')

    # settings for the lower, depicting the input
    au.set_xlim(xlimits[0], xlimits[1])
    #au.grid(True);
    au.set_facecolor('#d7f4ee')
    au.grid(color='#1a1a1a40', linestyle='--', linewidth=0.125)

    au.set_xlabel('Time (seconds)')
    au.set_ylabel('Voltage (V)')

    line_exp, = ay.plot(t, ym + ymean, color="#0088aaAF", linewidth=1.5, linestyle=(0, (1, 1)))
    line_model1, = ay.plot(t, ysim1 + ymean, color="#00AA44", linewidth=1.5, )
    line_model2, = ay.plot(t, ysim2 + ymean, color="#ff0066", linewidth=1.5, )


    #ay.plot(timestep + tau, ya + 0.63212 * delta_y, color="#ff0066", linewidth=1.5, marker=".", markersize=13)
    line_u, = au.plot(t, u, color="#00aa00")
    modelstr1 = r"Model $G_1(s) = \frac{%0.3f }{%0.3f\,s+1}$ ($FIT = %0.1f$" % (alpha, tau, r1) + "%)"
    modelstr2 = r"Model $G_2(s) = \frac{%0.3f }{(%0.3f\,s+1)(%0.3f\,s+1)}$ ($FIT = %0.1f$"%(alpha2, tau1, tau2, r2) + "%)"
    ay.set_title("Comparison of first and second order models estimated with a PRBS signal at the point $y_{OP}=%0.1f^o/s$."%yop)
    ay.legend([line_exp, line_model1, line_model2], ['Data', modelstr1, modelstr2],
              fontsize=12, loc = 'lower left',framealpha=0.95)
    au.legend([line_u], ['PRBS Input'], fontsize=14)
    PATH1 = r'/home/leonardo/sharefolder/ProyectoSabatico/Reporte/figures/'
    plt.savefig(PATH1 + "DCmotor_pbrs.svg", format="svg", bbox_inches="tight")
    plt.show()
    Path(PATH).mkdir(exist_ok=True)
    fo_model = [[alpha, tau]]
    so_model = [[alpha2, tau1, tau2]]
    np.savetxt(PATH + "DCmotor_fo_model_pbrs.csv", fo_model, delimiter=",",
               fmt="%0.8f", comments="", header='alpha, tau')

    np.savetxt(PATH + "DCmotor_so_model_pbrs.csv", so_model, delimiter=",",
               fmt="%0.8f", comments="", header='alpha2, tau1, tau2')

    system.disconnect()
    return #G1, G2

def step_open(system, op_point=50, amplitude=5, high_time=200, stab_time=150, uee_time=20, filepath = "step_open_exp.csv"):
    def step_message(system, userdata, message):
        q.put(message)

    topic_pub = system.codes["USER_SYS_STEP_OPEN"]
    topic_sub = system.codes["SYS_USER_SIGNALS_OPEN"]
    sampling_time = system.codes["THERMAL_SAMPLING_TIME"]
    amp_hex = float2hex(amplitude)
    points_high = math.ceil(high_time / sampling_time)
    points_high_hex = long2hex(points_high)
    op_point_hex = float2hex(op_point)
    stab_points = math.ceil(stab_time / sampling_time)
    uee_points = math.ceil(uee_time / sampling_time)
    stab_points_hex = long2hex(stab_points)
    uee_points_hex = long2hex(uee_points)
    message = json.dumps({"amplitude": amp_hex,
                          "op_point": op_point_hex,
                          "stab_points": stab_points_hex,
                          "uee_points": uee_points_hex,
                          "points_high": points_high_hex,
                          })
    system.client.on_message = step_message
    system.connect()
    system.subscribe(topic_sub)
    system.publish(topic_pub, message)
    q = Queue()
    np = None
    y = []
    u = []
    t = []
    yt = []
    ut = []
    tt = []
    exp =[]
    fig, ax = plt.subplots()
    line_y, = ax.plot(t, y, color="#ffcc00")
    line_u, = ax.plot(t, u, color="#00d4aa")
    line_yt, = ax.plot(t, yt, color="#d40055")
    line_ut, = ax.plot(t, ut, color="#338000")

    m = 1.23
    b = -24.75
    uf_est = m * op_point + b
    points = stab_points + uee_points + points_high
    ymax = (uf_est + amplitude - a2) / a1
    ax.set_xlim(0, sampling_time * points)
    ax.set_ylim(uf_est - 5, ymax + 5)
    plt.grid()
    npc = 1
    while npc <= points:
        try:
            message = q.get(True, 20 * sampling_time)
        except:
            raise TimeoutError("The connection has been lost. Please try again")

        decoded_message = str(message.payload.decode("utf-8"))
        msg_dict = json.loads(decoded_message)
        np_hex = str(msg_dict["np"])
        np = hex2long(np_hex)
        print(points, npc, np, hex2float(msg_dict["u"]), hex2float(msg_dict["y"]))
        if np == npc:
            if npc <= stab_points + uee_points:
                t_curr = (np - 1) * sampling_time
                t.append(t_curr)
                y_curr = hex2float(msg_dict["y"])
                y.append(y_curr)
                u_curr = hex2float(msg_dict["u"])
                u.append(u_curr)
                line_y.set_data(t, y)
                line_u.set_data(t, u)

            else:
                tt_curr = (np - 1) * sampling_time
                if npc <= stab_points + uee_points + 1:
                    t0 = tt_curr
                tt.append(tt_curr)
                yt_curr = hex2float(msg_dict["y"])
                yt.append(yt_curr)
                ut_curr = hex2float(msg_dict["u"])
                ut.append(ut_curr)
                exp.append([tt_curr - t0, ut_curr, yt_curr])
                line_yt.set_data(tt, yt)
                line_ut.set_data(tt, ut)
                line_y.set_data(t, y)
                line_u.set_data(t, u)
            plt.draw()
            plt.pause(0.1)
            npc += 1
    npy.savetxt(filepath, exp, delimiter=",",
                fmt="%0.8f", comments="", header='t,u,y')
    system.disconnect()
    plt.show()
    return t, u, y




if __name__ == "__main__":
    plant = ThermalSystemIoT()
    get_models_prbs(plant, yop=50, usefile=True)
    #get_static_model(plant, step=2.5)
   # pbrs_open(plant, op_point=50, peak_amp=4, stab_time=60, uee_time=10, divider=35, filepath=PATH + "thermal_prbs_open_exp.csv")
   # x = fotd_system_identification(filename='prbs_open_exp.csv')
   # print(x)

