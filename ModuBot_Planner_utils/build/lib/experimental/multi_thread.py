from multiprocessing import Pool
from time import time, sleep
def work(flag,fl):
    dp = 0
    for i in range(1000000):
        dp+=i
    return 0

t_init = time()
a = []
for i in range(100):
    a.append(work(i,i))

print(len(a))
print(time()-t_init)

t_init = time()
def wrapper(arg):
    return work(*arg)
f = lambda x : work(*x)
with Pool(5) as p:
    print(type(p.map(f, [ (1,2),(2,4),(7,4)] )))

print(time()-t_init)