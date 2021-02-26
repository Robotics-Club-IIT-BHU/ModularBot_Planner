import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.distributions import Categorical

def normalized_columns_initializer(weights, std=1.0):
    out = torch.randn(weights.size())
    out *= std / torch.sqrt(out.pow(2).sum(1, keepdim=True))
    return out

def weights_init(m):
    classname = m.__class__.__name__
    if classname.find('Conv') != -1:
        weight_shape = list(m.weight.data.size())
        fan_in = np.prod(weight_shape[1:4])
        fan_out = np.prod(weight_shape[2:4]) * weight_shape[0]
        w_bound = np.sqrt(6. / (fan_in + fan_out))

class Actor(nn.Module):

    def __init__(self,num_inputs, action_space):
        super(Actor, self).__init__()
        self.conv1 = nn.Conv2D(num_inputs, 32, 3, stride=2, padding=1)
        self.conv2 = nn.Conv2D(32, 32, 3, stride=2, padding=1)
        self.conv3 = nn.Conv2D(32, 32, 3, stride=2, padding=1)
        self.conv4 = nn.Conv2D(32, 32, 3, stride=2, padding=1)
        
        self.lstm = nn.LSTMCell(32*3*3, 256)

        num_outputs = action_space.n
        self.critic_linear = nn.Linear(256,1)
        self.actor_linear = nn.Linear(256, num_outputs)

        self.apply(weights_init)
        self.actor_linear.weight.data = normalized_c