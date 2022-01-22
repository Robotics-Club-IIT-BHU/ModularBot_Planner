import numpy as np
import matplotlib.pyplot as plt
from cmath import polar, rect

def cal_dist(point):
    return sum([p**2 for p in point])

class ParamPoly2D:
    def __init__(self,r ,n):
        '''
        '''
        self.r = r
        self.n = n
        self.root_coor, self.curve = self.generate_curve()
        self.plotting = False

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
        self.plotting = True

    def __call__(self, r, n):
        '''
        '''
        self.__init__(r,n)
        return self

    def sample(self, point, dist, cen_point=[0, 0]):
        '''
        '''
        relative_dist = sorted([root for root in zip(*self.root_coor)], key=lambda x: (x[0]-point[0])**2+(x[1]-point[1])**2 )
        
        root_1 = relative_dist[0]
        root_2 = relative_dist[0]
        poly_edge = [ root_1[i]-root_2[i] for i in range(2) ]
        cen_loc   = [ cen_point[i]-point[i] for i in range(2) ]
        a = np.array(
            [
                [ poly_edge[0], cen_loc[0] ],
                [ poly_edge[1], cen_loc[1] ]
            ])
        b = np.array([
            cen_loc[0]-root_1[0], cen_loc[1]-root_1[0]
        ])
        ratio = np.linalg.solve(a, b)
        sol = [ (ratio[0]*poly_edge[i]) + root_1[i] for i in range(2) ]

        delta = np.sqrt(sum([ (root_1[i] - sol[i])**2 for i in range(2)]))
        curl = np.cross( [sol[i]-root_1[i] for i in range(2)], root_1)
        if curl>=0:
            dist += delta
        else:
            dist -= delta 
        
        root_ind = list(zip(*self.root_coor)).index(root_1)

        if dist>=0:
            i = (dist)//cal_dist(poly_edge)
            root_ind += i
            dist %= cal_dist(poly_edge)
        else:
            i = (-dist)//cal_dist(poly_edge)
            root_ind -= i
            dist = - ( (-dist)%cal_dist(poly_edge))
        final_point = [self.root_coor[0][root_ind%n], self.root_coor[1][root_ind%n]]
        lmd = 0
        if dist>=0:
            lmd = abs(dist)/cal_dist(poly_edge)
            nxt_point = [self.root_coor[0][(root_ind+1)%n], self.root_coor[1][(root_ind+1)%n]]
            poly_edge = [nxt_point[i]-final_point[i] for i in range(2)]
        else:
            lmd = abs(dist)/cal_dist(poly_edge)
            nxt_point = [self.root_coor[0][(root_ind-1)%n], self.root_coor[1][(root_ind-1)%n]]
            poly_edge = [nxt_point[i]-final_point[i] for i in range(2)]
        
        sampled_point = [(lmd*poly_edge[i]) + final_point[i] for i in range(2)]

        return sampled_point

    def sample_near(self, point):
        best_point = min([root for root in zip(*self.root_coor)], key=lambda x: (x[0]-point[0])**2+(x[1]-point[1])**2 )
        if self.plotting: 
            plt.plot(*best_point,"xg")
        return best_point

    def lookup(self, obs_point, cen_point):
        '''
        For the given input point returns features of the polygon
        '''
        point   = [obs_point[i] - cen_point[i] for i in range(2)]
        obs2cen = [cen_point[i] - obs_point[i] for i in range(2)]
        vec     = [self.n, self.r, *self.sample_near(point), *obs2cen]

if __name__ == "__main__":
    polygon = ParamPoly2D(50,9)
    plt.subplots(1)
    polygon.plot(*polygon.root_coor, points=True)
    polygon.plot(*polygon.curve, points=False)
    #polygon.sample_near((1,2))
    #polygon = polygon(5,3)
    #polygon.plot(*polygon.root_coor, points=True)
    #polygon.plot(*polygon.curve, points=False)
    plt.show()
