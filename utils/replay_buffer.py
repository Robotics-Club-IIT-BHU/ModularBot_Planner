###################################################################################################################
## Reference : https://github.com/AboudyKreidieh/h-baselines/blob/master/hbaselines/multiagent/replay_buffer.py  ##
###################################################################################################################

import numpy as np

class HeirReplayBuffer(object):
    """
    Experience replay buffer for Heirarchical Learning.

    This replay buffer supports centralized training by including a full-states
    term for training centralized critics. In addition, information from all
    agents are stored under the same replay buffer.
    """

    def __init__(self,
                 buffer_size,
                 batch_size,
                 obs_dim,
                 ac_dim,
                 n_agents,
                 all_obs_dim,
                 gamma=0.99):
        """Instantiate a buffer.

        Parameters
        ----------
        buffer_size : int
            Max number of transitions to store in the buffer. When the buffer
            overflows the old memories are dropped.
        batch_size : int
            number of elements that are to be returned as a batch
        obs_dim : int
            number of elements in the observation
        ac_dim : int
            number of elements in the action
        n_agents : int
            the expected number of agents in the environment. Only relevant if
            using shared policies with MADDPG or goal-conditioned hierarchies.
        all_obs_dim : int
            number of elements in the full state observations
        """
        self._maxsize = buffer_size
        self._size = 0
        self._next_idx = 0
        self._batch_size = batch_size

        self.obs_t = [
            np.zeros((buffer_size, obs_dim), dtype=np.float32)
            for _ in range(n_agents)]
        self.action = [
            np.zeros((buffer_size, ac_dim), dtype=np.float32)
            for _ in range(n_agents)]
        self.reward = np.zeros(
            buffer_size, dtype=np.float32)
        self.obs_tp1 = [
            np.zeros((buffer_size, obs_dim), dtype=np.float32)
            for _ in range(n_agents)]
        self.done = np.zeros(
            buffer_size, dtype=np.float32)
        self.all_obs_t = np.zeros(
            (buffer_size, all_obs_dim), dtype=np.float32)
        self.all_obs_tp1 = np.zeros(
            (buffer_size, all_obs_dim), dtype=np.float32)
        self.gamma = gamma

    def __len__(self):
        """Return the number of elements stored."""
        return self._size

    @property
    def buffer_size(self):
        """Return the (float) max capacity of the buffer."""
        return self._maxsize

    def can_sample(self):
        """Check if n_samples samples can be sampled from the buffer.

        Returns
        -------
        bool
            True if enough sample exist, False otherwise
        """
        return len(self) >= self._batch_size

    def is_full(self):
        """Check whether the replay buffer is full or not.

        Returns
        -------
        bool
            True if it is full, False otherwise
        """
        return len(self) == self.buffer_size

    def add(self,
            obs_t,
            action,
            reward,
            obs_tp1,
            done,
            all_obs_t,
            all_obs_tp1):
        """Add a new transition to the buffer.

        Parameters
        ----------
        obs_t : list of array_like
            the last observation of each agent
        action : list of array_like
            the action of each agent
        reward : float
            the shared reward of the transition
        obs_tp1 : list of array_like
            the current observation of each agent
        done : float
            the shared episode done mask
        all_obs_t : array_like
            the last full state observation
        all_obs_tp1 : array_like
            the current full state observation
        """
        for i in range(len(obs_t)):
            self.obs_t[i][self._next_idx, :] = obs_t[i]
            self.action[i][self._next_idx, :] = action[i]
            self.reward[self._next_idx] = reward
            self.obs_tp1[i][self._next_idx, :] = obs_tp1[i]
            self.done[self._next_idx] = done
        self.all_obs_t[self._next_idx, :] = all_obs_t
        self.all_obs_tp1[self._next_idx, :] = all_obs_tp1
        self.tau_t = 1
        # Increment the next index and size terms
        self._next_idx = (self._next_idx + 1) % self._maxsize
        self._size = min(self._size + 1, self._maxsize)

    def add_tau_reward(self, reward):
        self.reward[self._next_idx] += (self._gamma**self.tau_t)*reward
        self.tau_t += 1
 
    def _encode_sample(self, idxes):
        """Convert the indices to appropriate samples."""
        return [obs_t[idxes, :] for obs_t in self.obs_t], \
            [action[idxes, :] for action in self.action], \
            self.reward[idxes], \
            [obs_tp1[idxes, :] for obs_tp1 in self.obs_tp1], \
            self.done[idxes], \
            self.all_obs_t[idxes, :], \
            self.all_obs_tp1[idxes, :]

    def sample(self, **_kwargs):
        """Sample a batch of experiences.

        Returns
        -------
        list of array_like
            (batch_size, obs_dim) batch of observations for each agent
        array_like
            (batch_size, ac_dim) batch of actions executed given obs_batch for
            each agent
        array_like
            (batch_size,) vector of  rewards received as results of executing
            act_batch
        list of array_like
            (batch_size, obs_dim) batch of next step observations seen after
            executing act_batch for each agent
        list of bool
            done_mask[i] = 1 if executing act_batch[i] resulted in the end of
            an episode and 0 otherwise.
        array_like
            (batch_size, all_obs_dim) batch of full-state observations
        array_like
            (batch_size, all_obs_dim) batch of next step full-state
            observations
        """
        indices = np.random.randint(0, self._size, size=self._batch_size)
        return self._encode_sample(indices)

