import numpy as np
import scipy as sp
from matplotlib import pyplot as plt
from scipy.signal import convolve2d  # Uncomment if you want to use something else for finding the configuration space

def generate_cspace(map_path):
    m = np.load(map_path)
    kernal = [[1 for _ in range(50)] for _ in range(50)]

    blurry = sp.signal.convolve2d(m, kernal, mode="same")
    c_space = []
    # Part 2.2: Compute an approximation of the “configuration space”
    for row in blurry:
        c_space.append([1 if col != 0 else 0 for col in row])

    plt.imshow(c_space)
    plt.show()
    np.save("../cspace.npy", np.array(c_space))

if __name__ == "__main__":
    generate_cspace("../map.npy")