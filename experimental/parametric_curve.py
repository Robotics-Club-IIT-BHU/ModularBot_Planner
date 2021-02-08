import numpy as np
import matplotlib.pyplot as plt
from cmath import polar, rect
class ParamPoly2D:
    def __init__(self,r ,n):
        '''
        '''
        self.r = r
        self.n = n
        self.root_coor, self.curve = self.generate_curve()

    def generate_curve(self):
        '''
        '''
        self.diff_angle = 2 * np.pi/ self.n
        self.init_phasor = rect(self.r,0)
        roots = [ (self.r, i*self.diff_angle) for i in range(self.n) ]
        self.edge = 2*self.r*np.sin(self.diff_angle/2)
        ds = 0.05/self.edge
        self.s = np.arange(0,self.n,ds)
        x, y = [], []
        for i_s in self.s:
            i = np.floor(i_s)
            tau = i_s - i
            point = self.r*rect(1,self.diff_angle*i)*((1-tau) + tau*rect(1,self.diff_angle))
            x.append(point.real)
            y.append(point.imag)    
        root_coors = [rect(*root).real for root in roots ],[rect(*root).imag for root in roots ]
    
        return root_coors, (x,y)
    
    def plot(self,x,y,points=True):
        '''
        '''
        plt.plot(x,y, "xb" if points else "-r", label="input")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()

    def __call__(self, r, n):
        '''
        '''
        self.__init__(r,n)
        return self

    def sample(self, point):
        '''
        '''
        best_point = min([root for root in zip(*self.root_coor)], key=lambda x: (x[0]-point[0])**2+(x[1]-point[1])**2 )
        plt.plot(*best_point,"xg")
        return best_point

if __name__ == "__main__":
    polygon = ParamPoly2D(50,9)
    plt.subplots(1)
    polygon.plot(*polygon.root_coor, points=True)
    polygon.plot(*polygon.curve, points=False)
    #polygon.sample((1,2))
    #polygon = polygon(5,3)
    #polygon.plot(*polygon.root_coor, points=True)
    #polygon.plot(*polygon.curve, points=False)
    plt.show()
