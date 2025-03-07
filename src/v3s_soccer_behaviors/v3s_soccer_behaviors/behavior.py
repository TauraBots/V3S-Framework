from abc import ABC, abstractmethod

class Behavior(ABC):
    def __init__(self, planner_mode="straight"):
        """
        Inicializa o comportamento com o modo do planejador.
        
        :param planner_mode: Modo do planejador a ser utilizado (ex.: "straight","astar"...).
        """
        self.planner_mode = planner_mode.lower().strip()

    @abstractmethod
    def execute(self, field_data, robot_index):
        """
        Processa os dados de visão (field_data) e retorna os comandos calculados
        para o robô identificado por robot_index.
        Deve retornar uma tupla: (left_speed, right_speed).
        """
        pass

    def set_planner_mode(self, mode: str):
        """
        Atualiza o modo do planejador.
        
        :param mode: Novo modo a ser utilizado (ex.: "straight", "astar"...).
        """
        self.planner_mode = mode.lower().strip()
