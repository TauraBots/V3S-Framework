# plot_process.py
import time
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from multiprocessing import Queue

# Constantes do campo (em metros)
FIELD_WIDTH = 1.3
FIELD_LENGTH = 1.5
GOAL_WIDTH = 0.4
GOAL_DEPTH = 0.1
GOAL_AREA_WIDTH = 0.7
GOAL_AREA_DEPTH = 0.15
ROBOT_SIZE = 0.075

def draw_field(ax):
    """Desenha o campo fixo."""
    ax.clear()
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Campo de Robô Soccer")
    half_length = FIELD_LENGTH / 2.0
    half_width = FIELD_WIDTH / 2.0
    ax.set_xlim(-half_length - 0.3, half_length + 0.3)
    ax.set_ylim(-half_width - 0.3, half_width + 0.3)
    # Retângulo do campo
    field_rect = patches.Rectangle((-half_length, -half_width),
                                   FIELD_LENGTH, FIELD_WIDTH,
                                   edgecolor='black', facecolor='none', lw=2)
    ax.add_patch(field_rect)
    # Linha central
    ax.plot([0, 0], [-half_width, half_width], 'k--', lw=1)
    # Círculo central
    center_circle = patches.Circle((0,0), 0.1, edgecolor='black', facecolor='none', lw=1)
    ax.add_patch(center_circle)
    # Gol direito
    goal_right = patches.Rectangle((half_length, -GOAL_WIDTH/2),
                                   GOAL_DEPTH, GOAL_WIDTH,
                                   edgecolor='red', facecolor='none', lw=2)
    ax.add_patch(goal_right)
    goal_area_right = patches.Rectangle((half_length - GOAL_AREA_DEPTH, -GOAL_AREA_WIDTH/2),
                                        GOAL_AREA_DEPTH, GOAL_AREA_WIDTH,
                                        edgecolor='red', facecolor='none', lw=1)
    ax.add_patch(goal_area_right)
    # Gol esquerdo
    goal_left = patches.Rectangle((-half_length - GOAL_DEPTH, -GOAL_WIDTH/2),
                                  GOAL_DEPTH, GOAL_WIDTH,
                                  edgecolor='blue', facecolor='none', lw=2)
    ax.add_patch(goal_left)
    goal_area_left = patches.Rectangle((-half_length, -GOAL_AREA_WIDTH/2),
                                       GOAL_AREA_DEPTH, GOAL_AREA_WIDTH,
                                       edgecolor='blue', facecolor='none', lw=1)
    ax.add_patch(goal_area_left)

def plot_process(plot_queue: Queue):
    """
    Processo dedicado para plotagem.
    Abre a janela de plot e, em loop, atualiza os dados recebidos pela queue.
    """
    plt.ion()  # Modo interativo
    fig, ax = plt.subplots()
    draw_field(ax)
    plt.show(block=True)  # A janela ficará aberta no processo de plotagem

    while True:
        try:
            # Tenta obter dados da queue com timeout para não travar o loop
            data = plot_queue.get(timeout=0.05)
        except Exception:
            data = None

        draw_field(ax)
        if data:
            waypoints, field_data, target_pose, robot_index = data
            # Plota os waypoints
            if waypoints:
                x_vals = [wp.pose.position.x for wp in waypoints]
                y_vals = [wp.pose.position.y for wp in waypoints]
                ax.plot(x_vals, y_vals, 'bo-', label="Trajetória")
            # Plota o alvo
            if target_pose:
                tx = target_pose.pose.position.x
                ty = target_pose.pose.position.y
                ax.plot(tx, ty, 'rx', markersize=10, label="Alvo")
            # Plota o robô (assumindo que field_data.robots_blue exista)
            if field_data is not None and hasattr(field_data, 'robots_blue') and len(field_data.robots_blue) > robot_index:
                robot = field_data.robots_blue[robot_index]
                ax.plot(robot.x, robot.y, 'go', markersize=8, label="Robô")
                # Desenha um círculo representando o tamanho do robô
                robot_circle = patches.Circle((robot.x, robot.y), ROBOT_SIZE/2.0,
                                              edgecolor='green', facecolor='none', lw=1)
                ax.add_patch(robot_circle)

        ax.legend()
        ax.relim()
        ax.autoscale_view()
        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        time.sleep(0.05)
