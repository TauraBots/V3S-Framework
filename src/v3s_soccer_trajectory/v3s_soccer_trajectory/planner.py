from abc import ABC, abstractmethod

class TrajectoryPlanner(ABC):
    @abstractmethod
    def compute_trajectory(self, start, goal):
        """
        Computa e retorna uma trajetória do ponto 'start' ao ponto 'goal'.
        O retorno pode ser uma lista de waypoints (cada waypoint pode ser, por exemplo, uma tupla (x, y)).
        """
        pass
