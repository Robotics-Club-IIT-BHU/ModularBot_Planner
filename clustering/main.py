import argparse
from cluster import sim

######################################################################
# Options
# --------
parser = argparse.ArgumentParser(description='creating stuctural represenation of spwaned bots')
parser.add_argument('--num_bots',default='8', type=int,help='number of bots to spawn: e.g. 15  26  89')
parser.add_argument('--num_clusters',default='2', type=int,help='minimum number of clusters to form: e.g. 3 5 7')
parser.add_argument('--debug',default=False, type=bool,help='add debug lines and text bw cluster members, helps in visualisation')
parser.add_argument('--seed',default='0', type=int,help='intialize random number generator for debugging')
opt = parser.parse_args()
print(f"--> spawning {opt.num_bots} bots")
print(f"--> creating {opt.num_clusters} clusters")
print(f"--> setting seed to {opt.seed}")
if opt.debug is True:
	print(f"--> Debugging is on")

######################################################################
#starting sim and ending
# --------
# help(sim)
sim(num_bots=opt.num_bots, seed=opt.seed, num_clusters=opt.num_clusters, debug=opt.debug)
