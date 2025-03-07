from .planner import TrajectoryPlanner

class StraightLinePlanner(TrajectoryPlanner):
    def compute_trajectory(self, start, goal):
        """
        Retorna uma trajetória em linha reta: uma lista contendo apenas o ponto final.
        Em uma implementação mais completa, você poderia calcular pontos intermediários.
        """
        return [goal]
