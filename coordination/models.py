import torch as th
from torch.nn import Module
from torch.nn import functional as F
from torch.distributions import Normal

class Pi_One(Module):
    def __init__(self, input_shape, output_shape, lower_limit=None, upper_limit=None):
        super(Pi_One, self).__init__()
        
        self.fc1 = torch.nn.Linear(input_shape, 32)
        self.fc2 = torch.nn.Linear(32, 48)
        self.fc3 = torch.nn.Linear(48, 24)
        self.fc_mu = torch.nn.Linear(24, output_shape)
        self.fc_logvar = torch.nn.Linear(24, output_shape)

        if upper_limit is None or lower_limit is None:
            self.action_scale = torch.tensor(1.)
            self.action_bias = torch.tensor(0.)
        else:
            self.action_scale = torch.FloatTensor(
                (upper_limit - lower_limit)/2.
            )
            self.action_bias = torch.FloatTensor(
                (upper_limit + lower_limit)/2.
            )
        
    def forward(self, x):
        inter = self.fc1(x)
        inter = F.relu(inter)
        inter = self.fc2(inter)
        inter = F.relu(inter)
        inter = self.fc3(inter)
        inter = F.relu(inter)

        mu_inter = self.fc_mu(inter)
        logvar_inter = self.fc_logvar(inter)

        return mu_inter, logvar_inter      

    def sample(self, cluster_state):
        mean, log_var = self.forward(cluster_state)
        std = (0.5*log_var).exp()
        normal = Normal(mean, std)

        x_t = normal.rsample()
        y_t = torch.tanh(x_t)

        action = y_t*self.action_scale + self.action_bias

        log_prob = normal.log_prob(x_t)
        
        log_prob = log_prob - torch.log(self.action_scale*(1 - y_t.pow(2)) + epsilon)
        log_prob = log_prob.sum(1, keepdim=True)

        mean = torch.tanh(mean)*self.action_scale + self.action_bias

        return action, log_prob, mean

    def to(self, device):
        self.action_scale = self.action_scale.to(device)
        self.action_bias = self.action_bias.to(device)
        return super(Pi_One, self).to(device)


class Pi_Two(Module):
    def __init__(self, input_shape, output_shape, vec_size, action_space=None):
        super(Pi_One, self).__init__()
        
        self.fc_state1 = torch.nn.Linear(input_shape, 32)
        self.fc_vec1 = torch.nn.Linear(vec_size, 10)
        self.fc_vec2 = torch.nn.Linear(10, 8)
        self.fc_state2 = torch.nn.Linear(32, 42)
        self.fc3 = torch.nn.Linear(42 + 8, 24)
        self.fc_mu = torch.nn.Linear(24, output_shape)
        self.fc_logvar = torch.nn.Linear(24, output_shape)

        if action_space is None:
            self.action_scale = torch.tensor(1.)
            self.action_bias = torch.tensor(0.)
        else:
            self.action_scale = torch.FloatTensor(
                (action_space.high[0, ...] - action_space.low[0, ...])/2.
            )
            self.action_bias = torch.FloatTensor(
                (action_space.high[0, ...] + action_space.low[0, ...])/2.
            )

    def forward(self, state, vec):
        cond = self.fc_vec1(vec)
        cond = th.tanh(cond)
        cond = self.fc_vec2(cond)
        cond = th.tanh(cond)
        inter = self.fc_state1(state)
        inter = F.relu(inter)
        inter = self.fc_state2(state)
        inter = F.relu(inter)
        inter = th.cat((inter, cond), -1)
        inter = self.fc3(inter)
        inter = F.relu(inter)
        inter = self.fc4(inter)
        inter = F.relu(inter)

        mu_inter = self.fc_mu(inter)
        logvar_inter = self.fc_logvar(inter)

        return mu_inter, logvar_inter      

    def sample(self, state, option_vec):
        mean, log_var = self.forward(state, option_vec)
        std = (0.5*log_var).exp()
        normal = Normal(mean, std)

        x_t = normal.rsample()
        y_t = torch.tanh(x_t)

        action = y_t*self.action_scale + self.action_bias

        log_prob = normal.log_prob(x_t)
        
        log_prob = log_prob - torch.log(self.action_scale*(1 - y_t.pow(2)) + epsilon)
        log_prob = log_prob.sum(1, keepdim=True)

        mean = torch.tanh(mean)*self.action_scale + self.action_bias

        return action, log_prob, mean

    def to(self, device):
        self.action_scale = self.action_scale.to(device)
        self.action_bias = self.action_bias.to(device)
        return super(Pi_Two, self).to(device)


class CondPiOneCritic(Module):
    def __init__(self, input_shape, cond_shape):

        super(CondPiOneCritic, self).__init__()
        self.fc1 = torch.nn.Linear(input_shape, 32)
        self.fc2 = torch.nn.Linear(cond_shape, 16)
        self.fc3 = torch.nn.Linear(32 + 16, 36)
        self.fc4 = torch.nn.Linear(36, 8)
        self.fc_mu = torch.nn.Linear(8, 1)
        self.fc_logvar = torch.nn.Linear(8, 1)

        self.optim = Adam(self.parameters(), lr=0.005)

        self.obs_idx = list(range(0,input_shape))
        self.cond_idx = list(range(input_shape, input_shape+cond_shape))

    def forward(self, obs, cond):
        
        inter_obs = self.fc1(obs)
        inter_cond = self.fc2(cond)
        inter = th.cat((inter_obs, inter_cond), -1)
        inter = th.tanh(inter)
        inter = self.fc3(inter)
        inter = th.tanh(inter)
        inter = self.fc4(inter)
        inter = th.tanh(inter)
        
        mu_inter = self.fc_mu(inter)
        logvar_inter = self.fc_logvar(inter)

        return mu_inter, logvar_inter

    def compute(self, state, cond):
        mean, log_var = self.forward(state, cond)
        std = (0.5*log_var).exp()
        normal = Normal(mean, std)

        v_t = normal.rsample()

        return v_t

    def train(self, state_t_1, state_t, heir_return):
        obs_t_1, cond_t_1 = state_t_1[:, self.obs_idx], state_t_1[:, self.cond_idx]
        obs_t, cond_t = state_t[:,self.obs_idx], state_t[:, self.cond_idx]

        for epoch in range(5):
            self.optim.zero_grad()
            v_t_1 = self.compute(obs_t_1, cond_t_1)
            v_t   = self.compute(obs_t, cond_t)

            td_loss = v_t_1 + heir_return - v_t
            td_loss.backward()
            self.optim.step()

class CondPiTwoCritic(Module):
    def __init__(self, input_shape, option_shape, cond_shape):

        super(CondPiTwoCritic, self).__init__()
        self.fc1 = torch.nn.Linear(input_shape, 16)
        self.fc2 = torch.nn.Linear(option_shape, 8)
        self.fc3 = torch.nn.Linear(cond_shape, 4)
        self.fc4 = torch.nn.Linear(16 + 8 + 4, 18)
        self.fc5 = torch.nn.Linear(18, 6)
        self.fc_mu = torch.nn.Linear(6, 1)
        self.fc_logvar = torch.nn.Linear(6, 1)

        self.optim = Adam(self.parameters(), lr=0.0001)

        self.obs_idx = list(range(0, input_shape))
        self.opt_idx = list(range(input_shape, input_shape+option_shape))
        self.cond_idx = list(range(input_shape+option_shape, input_shape+option_shape+cond_shape))

    def forward(self, obs, opt, cond):
        
        inter_obs = self.fc1(obs)
        inter_cond = self.fc2(cond)
        inter = th.cat((inter_obs, inter_cond), -1)
        inter = th.tanh(inter)
        inter = self.fc3(inter)
        inter = th.tanh(inter)
        inter = self.fc4(inter)
        inter = th.tanh(inter)
        
        mu_inter = self.fc_mu(inter)
        logvar_inter = self.fc_logvar(inter)

        return mu_inter, logvar_inter

    def compute(self, state, opt, cond):
        mean, log_var = self.forward(state, cond)
        std = (0.5*log_var).exp()
        normal = Normal(mean, std)

        v_t = normal.rsample()

        return v_t   

    def train(self, state_t_1, state_t, heir_return):
        obs_t_1, opt_t_1, cond_t_1 = state_t_1[:, self.obs_idx], state_t_1[:, self.opt_idx], state_t_1[:, self.cond_idx]
        obs_t, opt_t, cond_t = state_t[:,self.obs_idx], state_t[:, self.opt_idx], state_t[:, self.cond_idx]

        for epoch in range(5):
            self.optim.zero_grad()
            v_t_1 = self.compute(obs_t_1, opt_t_1, cond_t_1)
            v_t   = self.compute(obs_t, opt_t, cond_t)

            td_loss = v_t_1 + heir_return - v_t
            td_loss.backward()
            self.optim.step()