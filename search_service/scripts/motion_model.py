import math
import numpy as np
import scipy.interpolate

# motion parameter
L = 1.0  # wheel base
ds = 0.30  # course distanse
# v = 10.0 / 3.6  # velocity [m/s]
v = 3.0  # velocity [m/s]


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


def pi_2_pi(angle):
    return (angle + math.pi) % (2*math.pi) - math.pi


def update(state, v, delta, dt,L):

    state.v = v
    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    # state.yaw = state.yaw + state.v / L * math.tan(delta) * dt
    state.yaw = state.yaw + delta* dt
    state.yaw = pi_2_pi(state.yaw)

    return state


def generate_trajectory(cur_states, s, km, kf, k0):

    if s<0:
        s=abs(s)
    # if s>500:
        # print("s: ", s)
        # print("s is too large")
        # input()
    n = s / ds
    time = s / v  # [s]
    # tk = np.array([0.0, time / 2.0, time])
    tk = np.arange(0.0, time+0.02,  time/ 2.0)
    km_tmp = np.squeeze(np.asarray(km))
    kf_tmp = np.squeeze(np.asarray(kf))
    kk = np.array([k0, km_tmp, kf_tmp])
    t = np.arange(0.0, time, time / n)
    # fkp = scipy.interpolate.interp1d(tk, kk, kind="quadratic")
    # print("tk", tk)
    # print("kk", kk)
    # print("isnan(tk)", np.isnan(tk))
    # print("isnan(kk)",np.isnan(kk))
    fkp = scipy.interpolate.interp1d(tk, kk, kind="quadratic")
    # print("fkp", fkp)
    # print("t", t)
    kp = [fkp(ti) for ti in t]
    dt = float(time / n)

    #  plt.plot(t, kp)
    #  plt.show()

    state = State(x=cur_states[0], y=cur_states[1],yaw=cur_states[2], v = cur_states[3])
    x, y, yaw = [state.x], [state.y], [state.yaw]

    for ikp in kp:
        state = update(state, v, ikp, dt, L)
        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)

    return x, y, yaw


def generate_last_state(s, km, kf, k0):

    if s<0:
        s= abs(s)

    n = s / ds
    time = s / v  # [s]
    # tk = np.array([0.0, time / 2.0, time])
    # kk = np.array([k0, km, kf])
    tk = np.arange(0.0, time+0.01,  time/ 2.0)
    km_tmp = np.squeeze(np.asarray(km))
    kf_tmp = np.squeeze(np.asarray(kf))
    kk = np.array([k0, km_tmp, kf_tmp])

    t = np.arange(0.0, time, time / n)
    # print("t", t)
    # print("kk", kk)
    state = State()
    if tk[-1]!=0.0 and tk[-1]<6.0:
        # print("tk[-1]", tk[-1])
        fkp = scipy.interpolate.interp1d(tk, kk, kind="quadratic")
        kp = [fkp(ti) for ti in t]
        dt = time / n
    # else:
        # t = np.arange(0.0, 5.0, time / n)


    #  plt.plot(t, kp)
    #  plt.show()
        state = State()

        [update(state, v, ikp, dt, L) for ikp in kp]
    
    # print("state x, y, yaw:", state.x, ", ", state.y, ", ", state.yaw)

    return state.x, state.y, state.yaw
