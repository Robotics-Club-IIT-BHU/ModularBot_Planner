from absl import app
from absl import flags
import gym
import gym_iOTA
import pybullet as p
from modbot_planner.planning import planning
from modbot_planner.clustering import cluster
from modbot_planner.coordination import Coordination, ParamPoly2D

FLAGS = flags.FLAGS

flag.DEFINE_string('checkpoint', None, 'Location of saved weights')

def main():
    if FLAGS.checkpoint is None:
        raise Exception('Input checkpoint location')
    
    env = gym.make('iOTA-v0')
    obs = env.reset()
    done = False
    poly = ParamPoly2D(2,10)
    swarm = Coordination(env.k, env.n, pi_loc=FLAGS.checkpoint)
    sum_reward = 0.0
    gamma = 0.99
    timestep = 0
    while not done:
        state_info = gym_iOTA.utils.obs_for_modbot_planner(obs)
        parents, groups, dictionaries = cluster(obs, 
                                                [iota.base_id for iota in env.iotas],
                                                env.no_of_clusters, 
                                                debug=True,
                                                pClient=env.pClient
                                                )
        state_info = gym_iOTA.utils.split_cluster(state_info, dictionaries)
        action_dict = {}
        act_dict = swarm.forward(state_info, poly)
        action_dict.update(act_dict)
        target_point = gym_iOTA.utils.to_target(action_dict)
        robot_position = [ list(p.getBasePositionAndOrientation(iota.id, env.pClient)[0]) for iota in env.iotas ]
        obstacles = robot_position + env.fetch_obstacles()
        paths = planning(obs, target_point, (0,0,0), obstacles, 5)
        smooth_path = []
        progress = []
        ds = 0.05
        for path in paths:
            print(path)
            sp = Spline2D(*path)
            s = np.arange(0, sp.s[-1], ds)
            rx, ry = [], []
            for i_s in s:
                ix, iy = sp.calc_position(i_s)
                rx.append(ix)
                ry.append(iy)
            smooth_path.append(list(zip(rx,ry)))
            progress.append(0)
        i=0
        while i<100: 
            print(i)
            action = np.ones((env.no_of_modules, 3))
            for j in range(env.no_of_modules):
                if progress[j]!=-1:
                    if progress[j]==(len(smooth_path[j])-1):
                        progress[j] = -1
                    else: 
                        progress[j] +=1
                action[j,:] = [*smooth_path[j][progress[j]], 0.01]
            dock = np.zeros(
                    (env.no_of_modules,
                    env.no_of_modules))             
            obs, reward, done, info = env.step(action, dock)
            ## Try pooling the control
            if i%20==0:
                for j in range(env.no_of_modules):
                    setpoints[j,:] = [*poly.sample_near(obs[j,:2]),0.01] 
                cluster(obs, [iota.base_id for iota in env.iotas], env.no_of_clusters, debug=True, pClient=env.pClient )
            # time.sleep(0.1)
            i+=1
        obs, rew, done, _ = env.step(action)
        sum_reward += (gamma**timestep)*rew
        timestep += 1 
    env.reset()
    print("Total Returns is", sum_reward, "in", timestep, "timesteps")

if __name__ == "__main__":
    app.run(main)

