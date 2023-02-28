import matplotlib.pyplot as plt
import numpy as np
import progressbar


def plot(obstacles):
    plt.figure()
    plt.scatter(obstacles[:, 1], obstacles[:, 0], s=1)
    plt.show()


if __name__ == '__main__':
    namenum = 4

    img = np.load("mid/" + str(namenum) + ".npy")
    plot(img)