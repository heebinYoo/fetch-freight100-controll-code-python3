import math
import sys

import numpy as np

R = 0.4 #0.264  # m, from fetch diameter

orig_debug = []
far_debug = []


def g(x, phi):
    return math.tan(phi) * (x - R * math.cos(phi)) + R * math.sin(phi)


# return collisionable diatance when move 1m forward
def calculate_collision_distance(theta):
    distance = 0
    phi = math.pi / 2 - theta

    if -math.pi / 2 >= theta or math.pi / 2 <= theta:
        return -1

    elif -sys.float_info.epsilon <= theta <= sys.float_info.epsilon:
        orig_debug.append(np.array([0, R]));
        far_debug.append(np.array([0, 1 + R]))
        return 1

    elif 0 < phi < math.pi / 2 and 1 / R >= math.tan(phi):
        a = np.array([R * math.cos(phi), R * math.sin(phi)])
        b = np.array([R, g(R, phi)])
        distance = np.linalg.norm(a - b)

        orig_debug.append(a);
        far_debug.append(b)

    elif math.pi / 2 < phi < math.pi and -1 / R <= math.tan(phi):
        a = np.array([R * math.cos(phi), R * math.sin(phi)])
        b = np.array([-R, g(-R, phi)])
        distance = np.linalg.norm(a - b)

        orig_debug.append(a);
        far_debug.append(b)

    else:  # touch to circle
        a = math.tan(phi) ** 2 + 1
        b = -2 * math.tan(phi)
        c = 1 - R ** 2

        if 0 < phi < math.pi / 2:
            x = (-b + math.sqrt(b ** 2 - 4 * a * c)) / (2 * a)
        else:
            x = (-b - math.sqrt(b ** 2 - 4 * a * c)) / (2 * a)
        y = math.sqrt(R ** 2 - x ** 2) + 1

        p = np.array([R * math.cos(phi), R * math.sin(phi)])
        q = np.array([x, y])
        distance = np.linalg.norm(p - q)

        orig_debug.append(p);
        far_debug.append(q)

    return distance
