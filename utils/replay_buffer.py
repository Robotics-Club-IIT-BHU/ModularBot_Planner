from collections import deque

class ReplayBuffer:

    def __init__(self, max_size):
        super().__init__([], maxlen=int(max_size))

    def sample(self, batch_size):
        pass

class ParallelBuffer(ReplayBuffer):
