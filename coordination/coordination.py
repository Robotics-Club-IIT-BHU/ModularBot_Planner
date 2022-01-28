from .models import Pi_One, Pi_Two, CondPiOneCritic, CondPiTwoCritic
import torch as th
import numpy as np
from modbot_planner.utils.replay_buffer import HeirReplayBuffer

class Coordination:
    def __init__(self, k, n, pi_loc=None, top_k=5, max_edge=10, max_r=2, training=False):
        
        self.k        = k
        self.n        = n
        self.top_k    = top_k
        self.max_edge = max_edge
        self.max_r    = max_r
        self.training = training
        pi_one_ll     = np.array([0.0, 0.0, 0.0, 0.0])
        pi_one_ul     = np.array([self.max_edge, self.max_r, 2*self.max_r, 2*np.pi])
        self.pi_one   = Pi_One(input_shape = 5 + 2*self.k, output_shape = 2+2)                            # output size is 2 for parametric polygon and 2 for centroid of polygon
        self.pi_two   = Pi_Two(input_shape = 6, output_shape = 1, vec_size = 4)
        self.pi1_V    = CondPiOneCritic(input_shape = 5 + 2*k, cond_shape = 2+2)
        self.pi2_V    = CondPiTwoCritic(input_shape = 6, option_shape = 4, cond_shape = 1)
        self.opt_vec  = None

        if pi_loc is not None:
            self.pi_one.load_state_dict(th.load(pi_loc + "/pi_one.th"))  # loading the results
            self.pi_two.load_state_dict(th.load(pi_loc + "/pi_two.th"))  # loading the results
            self.pi1_V.load_state_dict(th.load(pi_loc  + "/pi1_V.th"))  # loading the results
            self.pi2_V.load_state_dict(th.load(pi_loc  + "/pi2_V.th"))  # loading the results
            self.pi_loc = pi_loc
            self.pi_one.eval()
            self.pi_two.eval()
        else:
            self.pi_loc = "results/models"

        if self.training:
            self.pi_1_buffer = HeirReplayBuffer(128, 32, 5 + 2*self.k, 4, self.k)
            self.pi_2_buffer = HeirReplayBuffer(128, 32, 6+4, 1, self.n)
        
    def save(self):
        th.save(self.pi_one.state_dict(), self.pi_loc+ "/pi_one.th")                  
        th.save(self.pi_two.state_dict(), self.pi_loc+ "/pi_two.th")
        th.save(self.pi1_V.state_dict(), self.pi_loc + "/pi1_V.th")                  
        th.save(self.pi2_V.state_dict(), self.pi_loc + "/pi2_V.th")
    
    def forward(self, state_info, poly=None):

        action = {}
        replan_cluster = state_info["replan_cluster"] or 0
        if self.opt_vec is None or self.cluster_criteria(state_info) or replan_cluster==1:
            self.forward_cluster(state_info)
            replan_cluster = -1 if replan_cluster==1 else 1
        
        for cluster in state_info["clusters"]:
            cluter_action = self.forward_lower(cluster, poly)
            action.update(cluster_action)
        
        return action, replan_cluster  

    def forward_cluster(self, state_info):

        for cluster in state_info["clusters"]:

            cen_x,cen_y = cluster["center_coor"]
            cen_n = cluster["number"]
            mean_x, mean_y = cluster["coords"].mean(axis=0)
            
            ## Fetching top k frontier coordinates
            coords = cluster["coords"]
            dist_coor = np.pow( coords - np.array([cen_x, cen_y]).reshape(1,2) , 2).sum(axis=-1)
            indx = np.argmax(dist_coor)
            sel_coord = coords[indx[:self.top_k], :]
            
            cluster_state = [cen_n, cen_x, cen_y, mean_x, mean_y]
            for i in range(self.k):
                cluster_state.append(sel_coord[i,0])
                cluster_state.append(sel_coord[i,1])

            cluster_state = np.array(cluster_state)
            opt_vec, log_prob, mean = self.pi_one.sample(cluster_state)
            self.opt_vec = opt_vec

            # if self.training:
            #     self.pi_one_buffer.push("opt_vec", opt_vec)
            #     self.pi_one_buffer.push("cluster_state", cluster_state)
            #     #self.pi_one_buffer.push("", )

    def forward_lower(self, cluster_info, poly=None):

        action = {}
        for agent in cluster_info["agents"]:
            cen_x, cen_y = agent["coord"]
            position = [cen_x, cen_y]
            poly = poly(self.opt_vec[:2])

            poly_cen = cmath.rect(*self.opt_vec[-2:])
            poly_cen = poly_cen.real, poly_cen.imag
            
            agent_state = poly.lookup(position, poly_cen)
            
            act_i = self.pi_two.sample(agent_state, self.opt_vec)
            sampled_point = poly.sample(position, act_i, poly_cen)
            action[agent["id"]] = sampled_point
        
        return action
    
    def cluster_criteria(self, state_info):
        err = 0.0

        for cluster in state_info["clusters"]:
            cen_x,cen_y = cluster["center_coor"]
            cen_n = cluster["number"]
            mean_x, mean_y = cluster["coords"].mean(axis=0)

            dist = (mean_x-cen_x)**2 + (mean_y-cen_y)**2
            coords = cluster["coords"]
            dist_coor = np.pow( coords - np.array([cen_x, cen_y]).reshape(1,2) , 2).sum(axis=-1)
            
            mu, sig = dist_coor.mean(), dist_coor.std()
            if dist <= mu-sig or dist>=mu+sig:              # If the distribution of agents and centroid of the cluster doesnt lie in the same 2D circle they are missaligned
                err += cen_n
            
        if err>=0.1*state_info['number']:                   # If more than 10% of the agents are misaligned
            return 1
        else:
            return 0