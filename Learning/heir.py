import torch
from torch import nn
from torch.distributions import Categorical, Bernoulli

from math import exp
import numpy as np

def to_tensor(obs):
    obs = np.asarray(obs)
    obs = torch.from_numpy(obs).float()
    return obs

class OptionCriticConv(nn.Module):
    def __init__(self, in_features, num_actions, num_options, temperature=1.0,
                eps_start=1.0, eps_min=0.1,)