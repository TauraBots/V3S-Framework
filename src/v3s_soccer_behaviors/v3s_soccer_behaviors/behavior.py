from abc import ABC, abstractmethod

class Behavior(ABC):
    @abstractmethod
    def execute(self, field_data, robot_index):
        """
        Processa os dados de visão (field_data) e retorna os comandos calculados
        para o robô identificado por robot_index.
        Deve retornar uma tuple: (left_speed, right_speed).
        """
        pass
