import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

data = []
with open('serial_log.txt', 'r') as f:
    line = 'dummy'
    while line:
        line = f.readline()
        if line.startswith('Updating servo'):
            vals = [line.split(':')[-1]]
            for i in range(7):
                vals.append(f.readline())

            vals = [v.replace('\n', '').strip() for v in vals]


            data.append(vals)


df = pd.DataFrame(data, columns=['servo',
                                 'since_prev[ms]',
                                 'speed_int',
                                 'speed_scale',
                                 'max_speed',
                                 'speed',
                                 'pos',
                                 'pos_update'])


df.servo = df.servo.astype(int)
mask = df.servo==0
print(df)
#df.loc[mask, 'since_prev[ms]'].astype(float).plot.hist()
df.loc[mask, 'pos_update'].astype(float).plot.hist()
plt.show()
