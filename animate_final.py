import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches

# --- Parameters ---
dt = 0.1
steps = 300
alpha = 0.1
k_att = 1.0
k_rep = 100.0
rho_0 = 3.0

# --- Environment Setup ---
goal = np.array([10.0, 10.0])
obstacles = [
    np.array([4.0, 5.0]),
    np.array([6.0, 6.0]),
    np.array([8.0, 4.0])
]

# --- Robot Initial Pose [x, y, theta] ---
pose = np.array([0.0, 0.0, 0.0])
trajectory = [pose[:2].copy()]

# --- Setup Plot ---
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-2, 15)
ax.set_ylim(-2, 12)
ax.set_title("Differential Drive Robot - PFM with Box Obstacles")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.grid(True)
ax.set_aspect('equal')

# --- Plot Static Elements ---
goal_plot = ax.plot(goal[0], goal[1], 'g*', markersize=15, label='Goal')[0]
obs_plots = [ax.plot(obs[0], obs[1], 'rx', markersize=10)[0] for obs in obstacles]

# --- Add Obstacle Boxes ---
box_size = 1.0
obs_boxes = []
for obs in obstacles:
    lower_left = obs - box_size / 2
    box = patches.Rectangle(lower_left, box_size, box_size, color='red', alpha=0.3, edgecolor='black')
    ax.add_patch(box)
    obs_boxes.append(box)

# --- Dynamic Plot Elements ---
path_plot, = ax.plot([], [], 'b-', linewidth=2, label='Path')
robot_plot = ax.plot([], [], 'ko', markersize=6, label='Robot')[0]
heading_plot, = ax.plot([], [], 'k-', linewidth=1)  # robot's direction

ax.legend()

# --- Animation Update Function ---
def update(frame):
    global pose, trajectory

    x, y, theta = pose

    # --- Attractive force ---
    F_att = -k_att * (np.array([x, y]) - goal)

    # --- Repulsive force from obstacles ---
    F_rep = np.zeros(2)
    for obs in obstacles:
        diff = np.array([x, y]) - obs
        rho = np.linalg.norm(diff)
        if 0 < rho < rho_0:
            grad = diff / rho
            mag = k_rep * (1 / rho - 1 / rho_0) / (rho**2)
            F_rep += mag * grad

    # --- Total force ---
    F_total = F_att + F_rep

    # --- Escape local minima with small perturbation ---
    if np.linalg.norm(F_total) < 1e-3:
        F_total += np.random.uniform(-0.1, 0.1, size=2)

    # --- Goal proximity check ---
    if np.linalg.norm(np.array([x, y]) - goal) < 0.5:
        v = 0
        w = 0
        print("Goal reached!")
    else:
        # Heading control
        desired_theta = np.arctan2(F_total[1], F_total[0])
        delta_theta = np.arctan2(np.sin(desired_theta - theta), np.cos(desired_theta - theta))
        w = 2.0 * delta_theta
        v = alpha * np.linalg.norm(F_total)

    # --- Update pose ---
    pose[0] += v * np.cos(theta) * dt
    pose[1] += v * np.sin(theta) * dt
    pose[2] += w * dt
    trajectory.append(pose[:2].copy())

    # --- Update plots ---
    path = np.array(trajectory)
    path_plot.set_data(path[:, 0], path[:, 1])
    robot_plot.set_data([pose[0]], [pose[1]])
    hx = pose[0] + 0.5 * np.cos(pose[2])
    hy = pose[1] + 0.5 * np.sin(pose[2])
    heading_plot.set_data([pose[0], hx], [pose[1], hy])

    return path_plot, robot_plot, heading_plot

# --- Run Animation ---
ani = animation.FuncAnimation(fig, update, frames=steps, interval=50, blit=True)
plt.show()