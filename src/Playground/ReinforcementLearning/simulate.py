from ballbouncer import BouncingBall
from agent import Agent
import time
import numpy as np

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import matplotlib.animation as animation

dt = .005
NKICK=5000
MINSPEED = .1
MAXSPEED = 1
nhist = 100
ninput = 5
DTinput = 50
sp = (nhist - DTinput)//ninput

ball = BouncingBall(dt, g=1, fric=.1, vinit=MINSPEED)
agent = Agent(inputlen=ninput)

# Create figure for plotting
fig, ax = plt.subplots(figsize=(4,4))
ax.set_aspect('equal')
ax.set_xlim([-.1,1.1])
ax.set_ylim([-.1,1.1])

# Create a Rectangle patch
rect = patches.Rectangle((0, 0), 1, 1, linewidth=.5, edgecolor='black', facecolor='none')

# Add the patch to the Axes
ax.add_patch(rect)

xs, ys = np.zeros(nhist), np.zeros(nhist)
predx, predy = np.zeros(ninput), np.zeros(ninput)

history = ax.plot(xs, ys, lw=1)[0]
preddat = ax.plot(predx, predy, 'o', lw=2, markersize=2)[0]
bouncer = ax.plot(xs[0], ys[0], 'o', color="C0", markersize=10)[0]
MLpred  = ax.plot(0, 1, '*', color="red", markersize=10)[0]
scoretext = ax.text(.5, 1.02, '', fontsize=7, horizontalalignment='center')

plt.axis('off')

def animate(i, xs, ys):

    if i%NKICK==0:
        ball.set_vel(*np.random.rand(2)*(MAXSPEED-MINSPEED) + np.ones(2)*MINSPEED)
        agent.resethistory()



    ball.update()
    x, y = ball.get_pos()

    xs[1:] = xs[:-1]
    xs[0] = x
    ys[1:] = ys[:-1]
    ys[0] = y


    if (i//NKICK)*NKICK + nhist < i:
        # Predictions are based on these:
        xpred = xs[DTinput::sp]
        ypred = ys[DTinput::sp]
        agent.setinput(xpred, ypred, x,y)

        agent.updatemodel()
        Xpred = agent.predict()

        MLpred.set_data(*Xpred)
        scoretext.set_text(r'R^2 Score {:.4f}'.format(agent.scoremodel()))
        preddat.set_data(xpred, ypred)

    bouncer.set_data(x, y)
    history.set_data(xs, ys)

    return history, bouncer, preddat, MLpred, scoretext,

# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=1, blit=True)
plt.show()

