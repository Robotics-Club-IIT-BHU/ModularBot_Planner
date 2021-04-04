import ray
import os
from ray.rllib import agents
import shutil

CHECKPOINT_ROOT = 'tmp/ppo/taxi'
shutil.rmtree(CHECKPOINT_ROOT, ignore_errors=True, onerror=None)
ray_results = os.getenv('HOME') + '/ray_results/'
shutil.rmtree(ray_results, ignore_errors=True, onerror=None)

SELECT_ENV = 'Taxi-v3'
config = agents.ppo.DEFAULT_CONFIG.copy()
config['framework'] = 'torch'
config['num_workers'] = 8
print(config['framework'], config['num_workers'])

ray.init()

configuration = {'gamma': 0.9,
          'lr': 1e-2,
          'num_workers': 4,
          'train_batch_size':1000,
          'model': {
                'fcnet_hiddens':[128,128]
         },
          'framework':'torch'
         }
print('Initialization worked')
trainer = agents.ppo.PPOTrainer(env='CartPole-v0', config=config)
N_ITER = 30
s = '{:3d} reward {:6.2f}/{:6.2f}/{:6.2f} len {:6.2f} saved {}'

for i in range(N_ITER):
    result = trainer.train()
    file_name = trainer.save(CHECKPOINT_ROOT)
    print(s.format(i+1, result['episode_reward_min'], result['episode_reward_mean'], result['episode_reward_max'], result['episode_len_mean'],file_name))
