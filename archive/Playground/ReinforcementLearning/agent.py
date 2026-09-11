import numpy as np
from sklearn.neural_network import MLPRegressor


class Agent():

    def __init__(self, inputlen=10):

        self.input2d = np.zeros((inputlen, 2))
        self.model = MLPRegressor(hidden_layer_sizes=(inputlen*2, inputlen*2, inputlen, ),
                                  activation='logistic',
                                  learning_rate_init=0.001,
                                  warm_start=True)
        self.X = np.zeros((10000, 2*len(self.input2d)))
        self.y = np.zeros((10000, 2))
        self.resethistory()


    def setinput(self, Xpred, Ypred, xpos, ypos):

        self.input2d[:, 0] = Xpred
        self.input2d[:, 1] = Ypred
        self.X[self.i, :] = self.input2d.ravel()
        self.y[self.i, :] = xpos, ypos
        self.i += 1

    def predict(self):

        X = self.input2d.ravel()

        return self.model.predict(X.reshape(1,-1))

    def updatemodel(self):
        """
        """
        X = self.X[:max(self.i,2)]
        y = self.y[:max(self.i,2)]

        self.model = self.model.partial_fit(X, y)

    def scoremodel(self):

        if self.i < 2: return -1

        X = self.X[:self.i]
        y = self.y[:self.i]

        return self.model.score(X, y)

    def resethistory(self):

        #self.X = np.zeros((10000, 2*len(self.input2d)))
        #self.y = np.zeros((10000, 2))
        self.i = 0



