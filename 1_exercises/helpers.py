import matplotlib.pyplot as plt
import numpy as np

def draw_planar_robot(q:np.ndarray, a:np.ndarray, area:np.ndarray):
    n = len(q)

    o = (0, 0)
    qinc = q.cumsum()

    for i, val in enumerate(q):
        # calculate frame i origo
        oi = (a[i]*np.cos(qinc[i]) + o[0], a[i]*np.sin(qinc[i]) + o[1])

        x1, y1 = [o[0], oi[0]], [o[1], oi[1]]
        plt.plot(x1, y1, marker = 'o')
        # plt.plot(oi[0], oi[1])

        o = oi

    plt.show()