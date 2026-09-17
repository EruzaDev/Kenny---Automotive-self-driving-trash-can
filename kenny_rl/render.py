"""Optional visualization; matplotlib is never imported by headless workers."""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle


def render(env):
    if env.figure is None:
        env.figure, env.axes = plt.subplots(figsize=(7, 7))
    ax = env.axes
    ax.clear()
    colors = {"wall": "#657786", "bag": "#bc7b40", "overhang": "#a970d6",
              "chair_seat": "#39a7a0", "chair_leg": "#195652", "chair_back": "#26877f"}
    for box, kind in zip(env.world.boxes, env.world.kinds):
        ax.add_patch(Rectangle(box[:2], box[3]-box[0], box[4]-box[1],
                               facecolor=colors[kind], alpha=.75))
    for x, y, xx, yy in env.world.cliffs:
        ax.add_patch(Rectangle((x, y), xx-x, yy-y, facecolor="#191919", hatch="xx", edgecolor="red"))
    ax.scatter(*env.world.markers.T, s=10, marker="s", color="#c4b500", label="marker landmarks")
    if len(env.route):
        ax.plot(*env.route.T, color="#1485d0", linewidth=1, label="estimated route")
    for x, y, vx, vy, radius in env.world.people:
        ax.add_patch(Circle((x, y), radius, color="#ef7d32"))
        ax.arrow(x, y, vx*.3, vy*.3, width=.02, color="#ef7d32")
    ax.add_patch(Circle(env.pose[:2], env.robot.radius, facecolor="#3478d5", alpha=.7))
    ax.arrow(*env.pose[:2], .5*np.cos(env.pose[2]), .5*np.sin(env.pose[2]), width=.025, color="#153958")
    ax.plot(*env.estimate[:2], "r+", label="estimated position")
    ax.plot(*env.world.goal, "g*", markersize=16, label="goal")
    ax.set(xlim=(0, env.world.size), ylim=(0, env.world.size), aspect="equal", xlabel="x (m)", ylabel="y (m)",
           title=f"KENNY | {env.config.stage} / {env.config.split} | step {env.steps} | stops {env.interventions}")
    ax.legend(loc="upper right", fontsize=7)
    env.figure.canvas.draw()
    if env.render_mode == "human":
        plt.pause(.001)
        return None
    return np.asarray(env.figure.canvas.buffer_rgba())[:, :, :3].copy()
