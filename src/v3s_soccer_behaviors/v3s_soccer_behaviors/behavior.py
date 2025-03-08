from abc import ABC, abstractmethod

class Behavior(ABC):
    @abstractmethod
    def execute(self, field_data, robot_index):
        """
        DEPRECATED: Mantido para compatibilidade.
        Processa os dados de visão (field_data) e retorna os comandos calculados
        para o robô identificado por robot_index.
        Deve retornar uma tuple: (left_speed, right_speed).
        """
        pass
        
    @abstractmethod
    def get_target_pose(self, field_data, robot_index):
        """
        Determina a posição alvo para o robô com base na estratégia deste comportamento.
        
        Args:
            field_data: Dados do campo incluindo posições dos robôs e da bola
            robot_index: Índice do robô para o qual calcular o alvo
            
        Returns:
            Tupla (x, y, theta) com as coordenadas e orientação alvo
        """
        pass