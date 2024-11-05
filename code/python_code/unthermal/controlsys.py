# Required libraries

import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import control as ct
import struct
from queue import Queue
from math import ceil
import json
import time

from scipy.signal import cont2discrete
from .thermalsys import ThermalSystemIoT, PATH_DATA, PATH_DEFAULT, FONT_SIZE

def float2hex(value):
    val_binary = struct.pack('>f', value)
    return val_binary.hex()


def long2hex(value):
    val_binary = struct.pack('>L', value)
    return val_binary.hex()


def hex2float(str):
    value = struct.unpack('!f', bytes.fromhex(str))[0]
    return value


def hex2long(str):
    value = struct.unpack('!L', bytes.fromhex(str))[0]
    return value


def signal2hex(signal):
    hstring = ""
    for point in signal:
        hstring += float2hex(point)
    return hstring


def matrix2hex(matrix):
    hstring = ""
    for row in matrix:
        for element in row:
            hstring += float2hex(element)
    return hstring


def time2hex(time_points):
    hstring = ""
    for t in time_points:
        hstring += long2hex(t)
    return hstring


def display_immediately(fig):
    canvas = fig.canvas
    display(canvas)
    canvas._handle_message(canvas, {'type': 'send_image_mode'}, [])
    canvas._handle_message(canvas, {'type': 'refresh'}, [])
    canvas._handle_message(canvas, {'type': 'initialized'}, [])
    canvas._handle_message(canvas, {'type': 'draw'}, [])


def set_reference(system, ref_value=50):
    ref_hex = float2hex(ref_value)
    topic_pub = system.codes["USER_SYS_SET_REF"]
    message = json.dumps({"reference": ref_hex})
    system.connect()
    system.publish(topic_pub, message)
    system.disconnect()
    rcode = True
    print("Reference changed")
    return rcode


def set_pid(system, kp=1, ki=0.4, kd=0, N=5, beta=0):
    topic_pub = system.codes["USER_SYS_SET_PID"]
    kp_hex = float2hex(kp)
    ki_hex = float2hex(ki)
    kd_hex = float2hex(kd)
    N_hex = float2hex(N)
    beta_hex = float2hex(beta)
    message = json.dumps({"kp": kp_hex,
                          "ki": ki_hex,
                          "kd": kd_hex,
                          "N": N_hex,
                          "beta": beta_hex})
    system.connect()
    system.publish(topic_pub, message)
    system.disconnect()
    print("PID parameters changed")
    return


def step_closed(system, r0=0 , r1=100, t0=0 ,  t1=1):
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
    points = points_low + points_high
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
    y = []
    r = []
    u = []
    t = []


    # Setting the graphics configuration for visualizing the experiment

    with plt.ioff():
        fig, (ay, au) = plt.subplots(nrows=2, ncols=1, width_ratios=[1], height_ratios=[4, 1], figsize=(10, 6))
    display_immediately(fig)

    # display config
    fig.set_facecolor('#ffffff') #'#b7c4c8f0')

    # settings for the upper axes, depicting the model and speed data
    ay.set_title(f'Closed loop step response experiment with an initial value of'
                 f'  $r_0=${r0:0.2f} and a  final value of $r_0=${r1:0.2f}', fontsize=FONT_SIZE)
    ay.set_ylabel(r'Temperature $o^C$')
    ay.set_xlabel(r'Time (s)')
    ay.grid(True);
    ay.grid(color='#806600ff', linestyle='--', linewidth=0.25)
    ay.set_facecolor('#f4eed7ff')
    ay.set_xlim(0, t0 + t1  - sampling_time)

    #Setting the limits of figure
    py = 0.7
    delta_r = abs(r1 - r0)
    ylimits = [r0 , r1]
    ylimits = [np.min(ylimits)- py * delta_r , np.max(ylimits) + py * delta_r]
    ay.set_ylim(ylimits[0], ylimits[1])

    au.set_facecolor('#d7f4e3ff')
    ay.set_ylabel(r'Power Input (%)')
    au.set_xlabel(r'Time (s)')
    au.set_ylim(0, 100)
    au.set_xlim(0, t0 + t1 - sampling_time )
    au.grid(color='#008066ff', linestyle='--', linewidth=0.25)


    line_r, = ay.plot(t, r, drawstyle='steps-post', color="#008066ff", linewidth=1.25)
    line_y, = ay.plot(t, y, color="#ff0066ff")
    line_u, = au.plot(t, u, color="#0066ffff")



    exp = []
    n = -1
    sync = False


    while n < points:
        try:
            message = q.get(True, 20)
        except:
            system.disconnect()
            raise TimeoutError("The connection has been lost. Please try again")

        decoded_message = str(message.payload.decode("utf-8"))
        msg_dict = json.loads(decoded_message)
        n_hex = str(msg_dict["np"])
        n = hex2long(n_hex)
        r_curr = hex2float(msg_dict["r"])
        if n == 0:
            sync = True           

        if (sync == True) & ((r_curr == r0)|(r_curr == r1)):            
            t_curr = n * sampling_time
            t.append(t_curr)
            y_curr = hex2float(msg_dict["y"])
            y.append(y_curr)
            r.append(r_curr)
            u_curr = hex2float(msg_dict["u"])
            u.append(u_curr)
            exp.append([t_curr, r_curr, y_curr, u_curr])
            ay.legend([line_r, line_y], [f'$r(t):$ {r_curr:0.2f}$~^oC$', f'$y(t):$ {y_curr: 0.3f}$~^oC$'], fontsize= FONT_SIZE, loc="upper left")
            au.legend([line_u], [f'$u(t):$ {u_curr: 0.1f} (% of 2.475 W)'], fontsize=  FONT_SIZE)
            line_r.set_data(t, r)
            line_y.set_data(t, y)
            line_u.set_data(t, u)
            fig.canvas.draw()
            time.sleep(0.1)
    Path(PATH_DATA).mkdir(exist_ok=True)
    np.savetxt(PATH_DEFAULT + "Thermal_step_closed_exp.csv",  exp, delimiter=",", fmt="%0.8f", comments="", header='t,r,y,u')
    np.savetxt(PATH_DATA + "Thermal_step_closed_exp.csv",  exp, delimiter=",", fmt="%0.8f", comments="", header='t,r,y,u')
    system.disconnect()
    return t, r, y, u


def stairs_closed(system, stairs=[40, 50, 60], duration=50):
    def stairs_message(system, userdata, message):
        q.put(message)


    # accessing fields of system IoT system
    sampling_time = system.codes["THERMAL_SAMPLING_TIME"]
    topic_pub = system.codes["USER_SYS_STAIRS_CLOSED"]
    topic_sub = system.codes["SYS_USER_SIGNALS_CLOSED"]
    points = len(stairs)
    points_hex = long2hex(points)
    signal_hex = signal2hex(stairs)
    duration_points = ceil(duration / sampling_time)
    duration_points_hex = long2hex(duration_points)
    message = json.dumps({"signal": signal_hex,
                          "duration": duration_points_hex,
                          "points": points_hex})
    system.client.on_message = stairs_message
    system.connect()
    system.subscribe(topic_sub)
    system.publish(topic_pub, message)

    total_points = duration_points * points - 1

    q = Queue()
    y = []
    r = []
    u = []
    t = []

    # Setting the graphics configuration for visualizing the experiment

    with plt.ioff():
        fig, (ay, au) = plt.subplots(nrows=2, ncols=1, width_ratios=[1], height_ratios=[4, 1], figsize=(10, 6))
    display_immediately(fig)

    # display configuration
    fig.set_facecolor('#ffffff')

    # settings for the upper axes, depicting the model and speed data
    ay.set_title(
        f'Profile response experiment with a duration of {total_points*sampling_time:0.2f} seconds and {points:d} stairs')
    ay.set_ylabel('')
    ay.grid(True);
    ay.grid(color='#806600ff', linestyle='--', linewidth=0.25)
    ay.set_facecolor('#f4eed7ff')
    ay.set_xlim(0, total_points*sampling_time)

    # Setting the limits of figure
    ylimits = [np.min(stairs) - 5, np.max(stairs) + 5]
    ay.set_ylim(ylimits[0], ylimits[1])

    au.set_facecolor('#d7f4e3ff')
    au.set_ylim(0, 100)
    au.set_xlim(0, total_points*sampling_time)
    au.grid(color='#008066ff', linestyle='--', linewidth=0.25)

    line_r, = ay.plot(t, r, drawstyle='steps-post', color="#008066ff", linewidth=1.25)
    line_y, = ay.plot(t, y, color="#ff0066ff")
    line_u, = au.plot(t, u, color="#0066ffff")

    exp = []
    n = -1
    sync = False

    while n < total_points:
        try:
            message = q.get(True, 20)
        except:
            system.disconnect()
            raise TimeoutError("The connection has been lost. Please try again")

        decoded_message = str(message.payload.decode("utf-8"))
        msg_dict = json.loads(decoded_message)
        n_hex = str(msg_dict["np"])
        n = hex2long(n_hex)

        if n == 0:
            sync = True
        if sync == True:
            t_curr = n * sampling_time
            t.append(t_curr)
            y_curr = hex2float(msg_dict["y"])
            y.append(y_curr)
            r_curr = hex2float(msg_dict["r"])
            r.append(r_curr)
            u_curr = hex2float(msg_dict["u"])
            u.append(u_curr)
            exp.append([t_curr, r_curr, y_curr, u_curr])
            ay.legend([line_r, line_y], [f'$r(t):$ {r_curr:0.2f}', f'$y(t):$ {y_curr: 0.3f}$~^oC$'], fontsize=FONT_SIZE,
                      loc="upper left")
            au.legend([line_u], [f'$u(t):$ {u_curr: 0.1f} (% of 2.475 W)'], fontsize=FONT_SIZE)
            line_r.set_data(t, r)
            line_y.set_data(t, y)
            line_u.set_data(t, u)
            fig.canvas.draw()
            time.sleep(0.1)

    np.savetxt(PATH_DEFAULT + "Thermal_stairs_closed_exp.csv", exp, delimiter=",", fmt="%0.8f", comments="", header='t,r,y,u')
    np.savetxt(PATH_DATA + "Thermal_stairs_closed_exp.csv", exp, delimiter=",", fmt="%0.8f", comments="", header='t,r,y,u')
    system.disconnect()

    return t, r, y, u



def set_controller(system, controller):


    topic_pub = system.codes["USER_SYS_SET_GENCON"]
    sampling_time = system.codes["THERMAL_SAMPLING_TIME"]
    struct = len(controller.den[0])
    type_control = struct - 1


    if struct == 1:
        con = ct.tf(ct.tf(controller.num[0][0], controller.den[0][0]))
        N1, D1 = ct.tfdata(con)
        N1 = N1[0][0]
        D1 = D1[0][0]
        N1 = N1 / D1[0]
        D1 = D1 / D1[0]

        if len(N1) == len(D1):
            d1 = N1[0]
            N1 = N1 - d1* D1
            N1 = N1[1:]
        else:
            d1 = 0

        DB = np.array([-D1[1:]])
        DB = DB.T
        size = len(D1)-2
        In_1 = np.eye(size)
        ZR = np.zeros((1,size))
        Acon = np.block([[In_1], [ZR]])
        Acon = np.block([DB, Acon])
        Bcon = np.array([N1]).T
        Ccon = np.append([1], ZR)
        Dcon = np.array([d1])



    elif struct == 2:
        con1 = ct.tf(ct.tf(controller.num[0][0], controller.den[0][0]))
        con2 = ct.tf(ct.tf(controller.num[0][1], controller.den[0][1]))
        N1, D1 = ct.tfdata(con1)
        N2, D2 = ct.tfdata(con2)

        N1 = N1[0][0]
        D1 = D1[0][0]
        N1 = N1 / D1[0]
        D1 = D1 / D1[0]
        N2 = N2[0][0]
        D2 = D2[0][0]
        N2 = N2 / D2[0]
        D2 = D2/ D2[0]


        if len(N1) == len(D1):
            d1 = N1[0]
            N1 = N1 - d1* D1
            N1 = N1[1:]
        else:
            d1 = 0

        if len(N2) == len(D2):
            d2 = N2[0]
            N2 = N2 - d2* D2
            N2 = N2[1:]
        else:
            d2 = 0

        DB = np.array([-D1[1:]])
        DB = DB.T
        size = len(D1)-2
        In_1 = np.eye(size)
        ZR = np.zeros((1,size))
        Acon = np.block([[In_1], [ZR]])
        Acon = np.block([DB, Acon])
        B1 = np.array([N1]).T
        B2 = np.array([N2]).T
        Bcon = np.block([B1,B2])
        Ccon = np.append([1], ZR)
        Dcon = np.block([d1, d2])


    Ad, Bd, Cd, Dd, dt = cont2discrete((Acon, Bcon, Ccon, Dcon), sampling_time, method='bilinear')
    Cve = ct.ss(Ad, Bd, Cd, Dd)
    Ai, Bi, Ci, Di = Cve.A, Cve.B[:, 0], Cve.C, Cve.D[0][0]
    int_system = ct.ss(Ai, Bi, Ci, Di)
    int_system, T = ct.canonical_form(int_system)
    Cve = ct.similarity_transform(Cve, T)
    A = Cve.A
    B = Cve.B
    Cc = Cve.C
    Dc = Cve.D
    order = np.size(A, 0)
    In = np.diag([10000 for i in range(order)])
    L, S, E = ct.dlqr(np.transpose(A), np.transpose(Cc), In, 1)
    L = np.transpose(L)
    Ac = A - L * Cc
    Bc = B - L * Dc
    if struct == 1:
        Bc = np.block([Bc, -Bc])
        Dc = np.array([[Dc[0][0], -Dc[0][0]]])
    A_hex = matrix2hex(Ac)
    B_hex = matrix2hex(Bc)
    C_hex = matrix2hex(Cc)
    D_hex = matrix2hex(Dc)
    L_hex = matrix2hex(L)
    order_hex = long2hex(order)
    type_control_hex = long2hex(type_control)
    message = json.dumps({"order": order_hex,
                          "A": A_hex,
                          "B": B_hex,
                          "C": C_hex,
                          "D": D_hex,
                          "L": L_hex,
                          "typeControl": type_control_hex,
                          })

    system.connect()
    system.publish(topic_pub, message)
    system.disconnect()
    print("Controller updated")
    return


def profile_closed(system, timevalues = [0, 50, 100 , 150], refvalues = [40, 50, 60, 70]):
    def profile_message(system, userdata, message):
        # This is the callback for receiving messages from the plant
        q.put(message)

    # reading the configuration parameters from the code's field in the plant

    topic_pub = system.codes["USER_SYS_PROFILE_CLOSED"]
    topic_sub = system.codes["SYS_USER_SIGNALS_CLOSED"]
    sampling_time = system.codes["THERMAL_SAMPLING_TIME"]


    # setting the parameters of the step response for sending to ESP32

    int_timevalues = [round(p/sampling_time) for p in timevalues]
    if int_timevalues[0] != 0:
        int_timevalues.insert(0, int_timevalues[0]-1)
        int_timevalues.insert(0,0)
        refvalues.insert(0,0)
        refvalues.insert(0,0)


    int_timevalues_hex = time2hex(int_timevalues)
    refvalues_hex = signal2hex(refvalues)
    points = len(int_timevalues)
    points_hex = long2hex(points)

    # user's command for obtaining the profile
    # al values are transmitted in hexadecimal
    min_val = np.min(refvalues)
    max_val = np.max(refvalues)
    min_val_hex = float2hex(min_val)
    max_val_hex = float2hex(max_val)

    message = json.dumps({"timevalues":  int_timevalues_hex,
                          "refvalues":   refvalues_hex,
                          "points":      points_hex,
                          "min_val":     min_val_hex,
                          "max_val":     max_val_hex,
                          })

    # setting the callback fro receiving data from the ESP32 for obtaining the profile response
    system.client.on_message = profile_message

    # connecting the system
    system.connect()

    # subscribing to topic published by ESP32
    system.subscribe(topic_sub)

    # command sent to ESP32 for obtaining the profile response
    system.publish(topic_pub, message)

    # setting the total of points and the total of frames
    total_points = int_timevalues[-1]

    q = Queue()
    y = []
    r = []
    u = []
    t = []


    # Setting the graphics configuration for visualizing the experiment

    with plt.ioff():
        fig, (ay, au) = plt.subplots(nrows=2, ncols=1, width_ratios=[1], height_ratios=[4, 1], figsize=(10, 6))
    display_immediately(fig)

    # display config
    fig.set_facecolor('#ffffff') #'#b7c4c8f0')

    # settings for the upper axes, depicting the model and speed data
    ay.set_title(f'Profile response experiment with a duration of {timevalues[-1]:0.2f} seconds and {len(timevalues):d} edges')
    ay.set_ylabel('')
    ay.grid(True);
    ay.grid(color='#806600ff', linestyle='--', linewidth=0.25)
    ay.set_facecolor('#f4eed7ff')
    ay.set_xlim(0, timevalues[-1])

    #Setting the limits of figure
    ylimits = [np.min(refvalues) - 5, np.max(refvalues) + 5]
    ay.set_ylim(ylimits[0], ylimits[1])

    au.set_facecolor('#d7f4e3ff')
    au.set_ylim(0, 100)
    au.set_xlim(0, timevalues[-1])
    au.grid(color='#008066ff', linestyle='--', linewidth=0.25)


    line_r, = ay.plot(t, r, color="#008066ff", linewidth=1.25)
    line_y, = ay.plot(t, y, color="#ff0066ff")
    line_u, = au.plot(t, u, color="#0066ffff")



    box = dict(boxstyle='round,pad=0.5', facecolor='white', edgecolor='white', alpha=0.75)
    # y_txt = ay.text( t0 + t1- 3, ylimits[0] + 1 , 'Temperature:\n ', fontsize=15, color="#ff6680",ha='right', va='bottom', bbox=box)
    # u_txt = au.text(t0 + t1 - 3 , 15, f'Input:', fontsize=15, color="#0066ffB0",ha='right', va='bottom', bbox=box)


    exp = []
    n = -1
    sync = False

    while n < total_points:
        try:
            message = q.get(True, 20)
        except:
            system.disconnect()
            raise TimeoutError("The connection has been lost. Please try again")

        decoded_message = str(message.payload.decode("utf-8"))
        msg_dict = json.loads(decoded_message)
        n_hex = str(msg_dict["np"])
        n = hex2long(n_hex)

        if n == 0:
            sync = True
        if sync == True:
            t_curr = n * sampling_time
            t.append(t_curr)
            y_curr = hex2float(msg_dict["y"])
            y.append(y_curr)
            r_curr = hex2float(msg_dict["r"])
            r.append(r_curr)
            u_curr = hex2float(msg_dict["u"])
            u.append(u_curr)
            exp.append([t_curr, r_curr, y_curr, u_curr])
            ay.legend([line_r, line_y], [f'$r(t):$ {r_curr:0.2f}', f'$y(t):$ {y_curr: 0.3f}$~^oC$'], fontsize=FONT_SIZE, loc="upper left")
            au.legend([line_u], [f'$u(t):$ {u_curr: 0.1f} (% of 2.475 W)'], fontsize=FONT_SIZE)
            line_r.set_data(t, r)
            line_y.set_data(t, y)
            line_u.set_data(t, u)
            fig.canvas.draw()
            time.sleep(0.1)




    np.savetxt(PATH_DEFAULT + "Thermal_profile_closed_exp.csv", exp, delimiter=",", fmt="%0.8f",
               comments="", header='t,r,y,u')
    np.savetxt(PATH_DATA + "Thermal_profile_closed_exp.csv", exp, delimiter=",", fmt="%0.8f",
               comments="", header='t,r,y,u')

    system.disconnect()
    return t, r, y, u










if __name__ == "__main__":
    plant = ThermalSystemIoT()
    set_pid(plant, kp=16.796, ki=2, kd=16.441, N=27.38, beta=0.5)
    t,  r, y, u = step_closed(plant, 40, 60, 30, 30)
    #signal = [30, 40, 50, 60, 70, 80]
    # set_pid(plant, kp = 16.796, ki = 5, kd = 16.441, N = 27.38, beta = 1)
    #step_open(plant, op_point=45, amplitude=5, high_time=200, stab_time=150, uee_time=20)
    #pbrs_open(plant, op_point=45, peak_amp=5, stab_time=150, uee_time=20, divider=35)
    #stair_closed(plant, signal, 10)


