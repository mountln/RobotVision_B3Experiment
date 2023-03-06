import time
import numpy as np

PATTERNS = {"oval", "eight"}


def get_path_generator(pattern: str, T, center=(0, 0), size=90, point_num=50):
    """
    Returns the path generator corresponding to ``pattern`` and ``T``.

    Path generator is a function that take ``current_time`` as argument, and
    returns the next position to move to.

    :param pattern: The pattern of glass ball movement
    :param T: The period of glass ball movement
    :param center: The center coordinate of the path
    :param size: The size of the path
    :param point_num: The number of points in a cycle
    """
    if pattern not in PATTERNS:
        raise ValueError("The pattern", pattern, "is undefined.")

    theta = np.linspace(0, np.pi * 2, point_num + 1)

    if pattern == "oval":
        x_axis = np.cos(theta) * size * 1.2 + center[0]
        y_axis = np.sin(theta) * size * 0.4 + center[1]
    elif pattern == "eight":
        x_axis = np.cos(theta) * size + center[0]
        y_axis = np.sin(theta) * np.cos(theta) * size * 0.8 + center[1]
    x_axis = x_axis.astype(int)
    y_axis = y_axis.astype(int)

    def path_generator():
        index = int(time.monotonic() % T * (point_num / T))
        return x_axis[index], y_axis[index]

    return path_generator
