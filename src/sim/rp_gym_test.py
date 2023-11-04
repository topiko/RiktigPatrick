import gymnasium as gym
import matplotlib.pyplot as plt
from gymnasium.envs.registration import register
from riktigpatric.patrick import StepAction

import sim

register(
    id="RiktigPatrick-v0",
    entry_point="sim.envs.rp_env:GymRP",
    max_episode_steps=300,
)

env = gym.make("RiktigPatrick-v0", render_mode="rgb_array")
env.reset()


pixels = env.render()
plt.imshow(pixels)
plt.show()


act = StepAction()
act.left_wheel = 0
act.right_wheel = 0
act.head_pitch = 0
act.head_turn = 0
env.step(act)


pixels = env.render()
plt.imshow(pixels)
plt.show()
