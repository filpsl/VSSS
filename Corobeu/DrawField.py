import matplotlib.pyplot as plt
import matplotlib.patches as patches
import pandas as pd

from datetime import datetime
import os

# Field dimensions in centimeters
field_length = 150
field_width = 130

def draw_field(ax):
    """
    Desenha o campo de futebol no gráfico fornecido.
    
    Parâmetros:
        ax (matplotlib.axes.Axes): O objeto Axes onde o campo será desenhado.
    """
    # Draw the outer boundary
    ax.plot([-field_length / 2, -field_length / 2], [-field_width / 2, field_width / 2], color="black")
    ax.plot([-field_length / 2, field_length / 2], [field_width / 2, field_width / 2], color="black")
    ax.plot([field_length / 2, field_length / 2], [field_width / 2, -field_width / 2], color="black")
    ax.plot([field_length / 2, -field_length / 2], [-field_width / 2, -field_width / 2], color="black")

    # Draw the center circle
    center_circle = plt.Circle((0, 0), 20, color="black", fill=False)
    ax.add_patch(center_circle)

    # Draw the center line
    ax.plot([0, 0], [-field_width / 2, field_width / 2], color="black")

    # Draw the penalty areas
    penalty_area_left = patches.Rectangle((-field_length / 2, -35), 15, 70, linewidth=1, edgecolor='black', facecolor='none')
    ax.add_patch(penalty_area_left)
    penalty_area_right = patches.Rectangle((field_length / 2 - 15, -35), 15, 70, linewidth=1, edgecolor='black', facecolor='none')
    ax.add_patch(penalty_area_right)

    # Draw semi-circles in front of the goals touching the goal area
    left_semi_circle = patches.Arc((-field_length / 2 + 4.5, 0), 30, 30, angle=0, theta1=315, theta2=45, color='black')
    right_semi_circle = patches.Arc((field_length / 2 - 4.5, 0), 30, 30, angle=0, theta1=135, theta2=225, color='black')
    ax.add_patch(left_semi_circle)
    ax.add_patch(right_semi_circle)
    
    # Draw the goals
    goal_left = patches.Rectangle((-field_length / 2 - 10, -20), 10, 40, linewidth=1, edgecolor='black', facecolor='none')
    ax.add_patch(goal_left)
    goal_right = patches.Rectangle((field_length / 2, -20), 10, 40, linewidth=1, edgecolor='black', facecolor='none')
    ax.add_patch(goal_right)

    # Draw corner arcs
    corner_arc_angles = [0, 270, 180, 90]
    corner_coords = [
        (-field_length / 2, -field_width / 2),
        (-field_length / 2, field_width / 2),
        (field_length / 2, field_width / 2),
        (field_length / 2, -field_width / 2)
    ]
    for angle, coord in zip(corner_arc_angles, corner_coords):
        corner_arc = patches.Arc(coord, 14, 14, angle=angle, theta1=0, theta2=90, color='black')
        ax.add_patch(corner_arc)

    # Mark the center spot
    center_spot = plt.Circle((0, 0), 1, color="black")
    ax.add_patch(center_spot)

    # Mark the free-kick ball positions
    free_kick_positions = [
        (-field_length / 2 + 37.5, -field_width / 2 + 25),
        (-field_length / 2 + 37.5, field_width / 2 - 25),
        (field_length / 2 - 37.5, -field_width / 2 + 25),
        (field_length / 2 - 37.5, field_width / 2 - 25),
        (-field_length / 2 + 37.5, 0),
        (field_length / 2 - 37.5, 0)
    ]
    for pos in free_kick_positions:
        free_kick_spot = plt.Circle(pos, 1, color="black")
        ax.add_patch(free_kick_spot)

    # Mark the free-kick robots positions
    robots_kick_positions = [
        (-field_length / 2 + 17.5, -field_width / 2 + 25),
        (-field_length / 2 + 17.5, field_width / 2 - 25),
        (field_length / 2 - 17.5, -field_width / 2 + 25),
        (field_length / 2 - 17.5, field_width / 2 - 25),
        (-field_length / 2 + 57.5, -field_width / 2 + 25),
        (-field_length / 2 + 57.5, field_width / 2 - 25),
        (field_length / 2 - 57.5, -field_width / 2 + 25),
        (field_length / 2 - 57.5, field_width / 2 - 25),
    ]
    for pos in robots_kick_positions:
        robots_kick_spot = plt.Circle(pos, 1, color="green")
        ax.add_patch(robots_kick_spot)

    # Add text labels for positions
    ax.text(-field_length / 2 + 37.5, -field_width / 2 + 25, 'BL', color='red', fontsize=12, ha='center', va='center')
    ax.text(-field_length / 2 + 37.5, field_width / 2 - 25, 'BL', color='red', fontsize=12, ha='center', va='center')
    ax.text(field_length / 2 - 37.5, -field_width / 2 + 25, 'BL', color='red', fontsize=12, ha='center', va='center')
    ax.text(field_length / 2 - 37.5, field_width / 2 - 25, 'BL', color='red', fontsize=12, ha='center', va='center')

    # Set the aspect of the plot to be equal
    ax.set_aspect('equal')
    ax.set_xlim(-field_length / 2 - 10, field_length / 2 + 10)
    ax.set_ylim(-field_width / 2 - 10, field_width / 2 + 10)
    ax.axis('on')  # Turn on the axis

# Function to plot the robot path
def plot_robot_path(desired_path_x, desired_path_y, executed_path_x, executed_path_y, filename):
    """
    Plota o caminho desejado e o caminho executado pelo robô no campo.
    
    Parâmetros:
        desired_path_x (list): Lista de coordenadas X do caminho desejado.
        desired_path_y (list): Lista de coordenadas Y do caminho desejado.
        executed_path_x (list): Lista de coordenadas X do caminho executado.
        executed_path_y (list): Lista de coordenadas Y do caminho executado.
    """
    fig, ax = plt.subplots(figsize=(10, 8))

    # Draw the soccer field
    draw_field(ax)

    # Convert path positions from meters to centimeters
    def convert_path(path_x, path_y):
        return [(x * 100, y * 100) for x, y in zip(path_x, path_y)]

    desired_path = convert_path(desired_path_x, desired_path_y)
    executed_path = convert_path(executed_path_x, executed_path_y)

    # Plot the desired path
    if desired_path:
        xs, ys = zip(*desired_path)
        ax.plot(xs, ys, color='red', marker='o', markersize=5, linestyle='-', linewidth=2, label='Trajetória Desejada')

    # Plot the executed path
    if executed_path:
        xs, ys = zip(*executed_path)
        ax.plot(xs, ys, color='blue', linestyle='-', linewidth=2, label='Trajetória Implementada')

    ax.legend()
    plt.title("Trajetórias Implementadas no Campo")
    name = filename + '_Trajectory.pdf'
    # plt.savefig(name)

    timestamp = datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
    results_dir = "Resultados Atualizados - Filipe"
    response_file = os.path.join(results_dir, f"Trajetória_{timestamp}.png")
    plt.savefig(response_file)

    plt.show()

if __name__ == '__main__':
    #Plotting the Path Points
    data = pd.read_csv('bancoPosicoes.csv')
    # Extract x and y coordinates
    x = data.iloc[:, 1] * 100  # Convert to cm
    y = data.iloc[:, 2] * 100  # Convert to cm
    fig, ax = plt.subplots(figsize=(10, 8))
    # Draw the soccer field
    draw_field(ax)
    # Plot the points using scatter
    ax.scatter(x, y, color='green', marker='o')
    plt.show()
    # Example usage
    # pathX = [0, 0.1, 0.2, 0.3, 0.4]
    # pathY = [0, 0.1, 0.2, 0.3, 0.4]
    # crb01_xOut = [0, 0.08, 0.18, 0.28, 0.38]
    # crb01_yOut = [0, 0.09, 0.19, 0.29, 0.39]
    # plot_robot_path(pathX, pathY, crb01_xOut, crb01_yOut)