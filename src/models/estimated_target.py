import numpy as np

class EstimatedTarget3D:
    def __init__(self, state):
        self.state = np.array(state, dtype=float)

    @property
    def pos(self):
        return self.state[0:3]

    @property
    def vel(self):
        return self.state[3:6]