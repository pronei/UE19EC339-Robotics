#!/usr/bin/env python3

import numpy as np

class Integral(object):
    def __init__(self, IC, Ts, gain):
        self.IC = np.array(IC) if isinstance(IC, list) else IC
        self.x = self.IC
        self.Ts = Ts
        self.K = gain
        self.n = 0
        self.y = 0

    def getState(self):
        return self.x

    def __call__(self):
        self.n += 1
        return self.y
    
    def reset(self, newIC=False):
        self.n = 0
        self.y = 0
        if not newIC:
            self.x = self.IC
        else:
            self.x = np.array(newIC)
    
    def update(self):
        pass


class ForwardEuler(Integral):
    def __init__(self, IC, Ts, gain=1):
        super(ForwardEuler, self).__init__(IC, Ts, gain)
        self.y = self.IC

    def update(self, u):
        self.x = self.x + self.K * self.Ts * u
        self.y = self.x
    
    def __call__(self, u):
        output = self.y
        self.update(u)
        self.n += 1
        return output


class BackwardEuler(Integral):
    def __init__(self, IC, Ts, gain=1):
        super(BackwardEuler, self).__init__(IC, Ts, gain)

    def update(self, u):
        self.y = self.x + self.K * self.Ts * u
        self.x = self.y
    
    def __call__(self, u):
        self.update(u)
        self.n += 1
        return self.y


class Trapezoidal(Integral):
    def __init__(self, IC, Ts, gain=1):
        super(Trapezoidal, self).__init__(IC, Ts, gain)
    
    def update(self, u):
        self.y = self.x + self.K * self.Ts/2 * u
        self.x = self.y + self.K * self.Ts/2 * u

    def __call__(self, u):
        self.update(u)
        self.n += 1
        return self.y


if __name__ == "__main__":
    # Unit testing
    import matplotlib.pyplot as plt

    # f(t) = 2*pi*sin(2*pi*t)
    # f'(t) = -cos(2*pi*t)
    t = np.linspace(0, 1, 101)
    f = 2*np.pi * np.sin(2*np.pi*t)
    f_drv = -np.cos(2*np.pi*t)

    i1 = ForwardEuler(-1, 0.01)
    i2 = BackwardEuler(-1, 0.01)
    i3 = Trapezoidal(-1, 0.01)
    i1_out = np.zeros(101)
    i2_out = np.copy(i1_out)
    i3_out = np.copy(i1_out)

    for i in range(t.size):
        i1_out[i] = i1(f[i])
        i2_out[i] = i2(f[i])
        i3_out[i] = i3(f[i])
    
    fig, (ax1, ax2, ax3) = plt.subplots(3)
    fig.suptitle("Integral tests")
    ax1.plot(t, f_drv, label="f't)")
    ax1.plot(t, i1_out, label="forward euler")
    ax2.plot(t, f_drv, label="f'(t)")
    ax2.plot(t, i2_out, label="backward euler")
    ax3.plot(t, f_drv, label="f'(t)")
    ax3.plot(t, i3_out, label="trapezoidal")
    ax1.legend()
    ax2.legend()
    ax3.legend()
    plt.show()
