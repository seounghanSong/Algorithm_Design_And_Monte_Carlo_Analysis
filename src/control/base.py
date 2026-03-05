from abc import ABC, abstractmethod

class Controller(ABC):
    @abstractmethod
    # 입력된 가속도 명령을 그대로 미사일에 전달(지연X)
    def update(self, missile, acc_cmd):
        pass