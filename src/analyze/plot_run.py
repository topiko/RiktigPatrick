import matplotlib.pyplot as plt
import pandas as pd
import numpy as np


df = pd.read_hdf('data/rp.hdf5')




plt.hist(np.diff(df.rptime.values*1000), bins=np.linspace(0,100,101))
plt.xlabel('rptime diff [ms]')
plt.show()
