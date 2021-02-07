import numpy as np
import matplotlib.pyplot as plt
from cmath import polar, rect
def curve(r, n):
    diff_angle = 2*np.pi/n
    init_phasor = rect(r,0)
    roots = [ (r, i*diff_angle) for i in range(n) ]
    edge = 2*r*np.sin(diff_angle/2)
    ds = 0.1/edge
    s = np.arange(0,n,ds)
    x, y = [], []
    for i_s in s:
        i = np.floor(i_s)
        tau = i_s - i
        point = r*rect(1,diff_angle*i)*((1-tau) + tau*rect(1,diff_angle))
        x.append(point.real)
        y.append(point.imag)    
    root_coors = [rect(*root).real for root in roots ],[rect(*root).imag for root in roots ]
    #points = []
    #np.polar()
    return root_coors, (x,y)
def plot(x,y, points=True):

    
    plt.plot(x,y, "xb" if points else "-r", label="input")
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.legend()

if __name__ == "__main__":
    root, curv = curve(50,10)
    plt.subplots(1)
    plot(*root, points=True)
    plot(*curv, points=False)    
    plt.show()