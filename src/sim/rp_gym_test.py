import gymnasium as gym
import sim
from gymnasium.envs.registration import register

register(
    id="RiktigPatrick-v0",
    entry_point="sim.envs.rp_env:GymRP",
    max_episode_steps=300,
)

env = gym.make("RiktigPatrick-v0", render_mode="rgb_array")
env.reset()


import matplotlib.pyplot as plt

pixels = env.render()
plt.imshow(pixels)
plt.show()

from riktigpatric.patrick import StepAction

act = StepAction()
act.left_wheel = 0
act.right_wheel = 0
act.head_pitch = 0
act.head_turn = 0
env.step(act)


pixels = env.render()
plt.imshow(pixels)
plt.show()
