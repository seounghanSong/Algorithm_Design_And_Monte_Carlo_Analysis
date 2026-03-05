from abc import ABC, abstractmethod

class GuidanceLaw(ABC):
    @abstractmethod
    def compute_command(self, missile, target):
        pass