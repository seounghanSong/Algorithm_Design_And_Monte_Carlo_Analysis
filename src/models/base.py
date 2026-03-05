from abc import ABC, abstractmethod

class DynamicModel(ABC):
    @abstractmethod
    def step(self, dt):
        pass