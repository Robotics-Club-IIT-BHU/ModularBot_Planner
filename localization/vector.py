import numpy as np

class Vector:
    def __init__(self,x,y,z=None):
        if z is None:
            self = Vect2d(x,y)
        else:
            self = Vect3d(x,y,z)

    def __repr__(self):
        return "<Vector "+str(self.x)+","+str(self.y)+" = "+str(self.dst())+">"

class Vect2d(Vector):
    def __init__(self,x,y=None):
        if isinstance(x, Vect3d) or isinstance(x, Vect2d):
            self.x = x.x
            self.y = x.y
            self.z = None
    def dst(self,resp=None):
        temp = self - (resp or Vect2d(0,0))
        return (temp.x)**2 + (temp.y)**2

    def my_unit(self):
        return self / np.sqrt(self.dst())

    def __truediv__(self,val):
        return Vect2d(self.x / val,self.y / val)

    def __sub__(self, other):
        return Vect2d(self.x-other.x,self.y-other.y, None if other.z is None else -other.z )

    def __mul__(self,other): ## sin of the angle between them
        temp_self = self.my_unit()
        temp_other = other.my_unit()
        return temp_self.x*temp_other.y - temp_self.y*temp_other.x

    def __and__(self,other): ## cos of the angle between them
        temp_self = self.my_unit()
        temp_other = other.my_unit()
        return temp_self.x*temp_other.x + temp_self.y*temp_other.y

    def get_values(self):
        return (self.x, self.y)

    def __repr__(self):
        return "<Vect2d "+str(self.x)+","+str(self.y)+" = "+str(self.dst())+">"



class Vect3d(Vector):
    def __init__(self,x,y=None,z=None):
        if isinstance(x,Vect2d) or isinstance(x, Vect3d):
            self.x = x.x
            self.y = x.y
            self.z = x.y if isinstance(x, Vect3d) else 0
        else:
            self.x = x or 0
            self.y = y or 0
            self.z = z or 0
    def dst(self,resp=None):
        temp = self - (resp or Vect3d(0,0,0))
        return (temp.x)**2 + (temp.y)**2 + (temp.z)**2

    def my_unit(self):
        return self / np.sqrt(self.dst())

    def __truediv__(self,val):
        return Vect3d(self.x / val,self.y / val, self.z / val)

    def __sub__(self, other):
        return Vect3d(self.x-other.x,self.y-other.y, (self.z or 0)-(other.z or 0))

    def __mul__(self,other): ## sin of the angle between them
        temp_self = self.my_unit()
        temp_other = other.my_unit()
        return Vect3d(
                      temp_self.y*temp_other.z - temp_other.y*temp_self.z,
                      temp_other.x*temp_self.z - temp_self.x*temp_other.z,
                      temp_self.x*temp_other.y - temp_self.y*temp_other.x,
                      )

    def __and__(self,other): ## cos of the angle between them
        temp_self = self.my_unit()
        temp_other = other.my_unit()
        return (temp_self.x*temp_other.x, temp_self.y*temp_other.y, temp_self.z*temp_other.z)

    def get_values(self):
        return (self.x, self.y, self.z)

    def __repr__(self):
        return "<Vect3d "+str(self.x)+","+str(self.y)+','+str(self.z)+" = "+str(self.dst())+">"
