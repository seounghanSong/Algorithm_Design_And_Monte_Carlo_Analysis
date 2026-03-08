import numpy as np

class MeasuredTarget3D:
    def __init__(self, position, velocity):
        self._pos = np.array(position, dtype=float)
        self._vel = np.array(velocity, dtype=float)

    @property
    def pos(self):
        return self._pos

    @property
    def vel(self):
        return self._vel