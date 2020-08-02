import math
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import src.AllConstants as Const


def atan3(y_change, x_change):
    """
    To calculate line slop in form of 0 to 2 pi.
    :param y_change: change in y
    :param x_change: change in x
    :return: the slope in form of 0 to 2pi
    """
    atan = math.atan2(y_change, x_change)
    if atan > 0:
        return atan
    else:
        return 2 * math.pi + atan


def dist(x, y):
    """
    To calculate the distance between to coordinates.
    :param x: first point
    :param y: second point
    :return: distance between points.
    """
    return math.sqrt((y[0] - x[0]) ** 2 + (y[1] - x[1]) ** 2)


def jpg_to_bin_image(floor_img_adr=Const.FLOOR_IMG_ADR):
    """
    Generates standard floor black and white image from the input map jpeg or png image.
    Read the image address from AllConstants file and write to FloorPro.png. FloorPro.png the floor map based on input
    image, binary (true black and white) and centered.

    :param floor_img_adr: address to floor image default is the FLOOR_IMG_ADR parameter in AllConstants script.
    :return: binary (true black and white) and centered
    """
    # read image
    im_bw = Image.open(floor_img_adr).convert(mode='L', dither=None)

    ls = list(im_bw.getdata())
    img_s = im_bw.size[::-1]
    img_b = np.reshape(ls, img_s)
    img_b_v1 = (np.round(img_b / 200))
    img_b_v2 = img_b_v1 * -1 + 1

    plt.imshow(img_b_v2, cmap='gray', interpolation='nearest')
    y = set(img_b_v2.flat)

    img_b_v2 = img_b_v2.transpose()
    img_b_v2 = img_b_v2[:, ::-1]
    plt.imshow(img_b_v2[:, ::-1].transpose(), cmap='hot', interpolation='nearest')

    print('Image', floor_img_adr, 'is imported as', img_s)
    print('Image is converted to int array of', len(img_b_v2), '*', len(img_b_v2[0]), 'with values consisted of', y)
    plt.imsave('FloorPro.png', img_b_v1, cmap='gray')
    return img_b_v2


def obstacle_scale(grid, rr=Const.ROBOT_RADIUS):
    """
    Generates the obstacle margin within the input grid. Every grid cell that is occupied will be extended as  much
    as robot radius in all directions.
    :param grid: binary matrix (grid) of input map
    :param rr: robot radius
    :return: binary matrix (grid) of input map with obstacle margins
    """
    a = np.copy(grid)
    for i in range(rr):
        grid = grid + 0.001 * np.roll(a, i + 1, 0)  # right
        grid = grid + 0.001 * np.roll(a, -(i + 1), 0)  # left
        grid = grid + 0.001 * np.roll(a, i + 1, 1)  # up
        grid = grid + 0.001 * np.roll(a, -(i + 1), 1)  # down
        grid = grid + 0.001 * np.roll(np.roll(a, i + 1, 0), i + 1, 1)
        grid = grid + 0.001 * np.roll(np.roll(a, i + 1, 0), -(i + 1), 1)
        grid = grid + 0.001 * np.roll(np.roll(a, -(i + 1), 0), i + 1, 1)
        grid = grid + 0.001 * np.roll(np.roll(a, -(i + 1), 0), -(i + 1), 1)
    grid = np.ceil(grid * 1.01) / 2
    return grid


def grid_square(grid):
    """
    Add columns and rows to grid to make the size equal to powers of 2.

    :param grid: binary matrix (grid) of input map
    :return: binary matrix (grid) of input map with edge size equal to powers of 2.
    """
    t = (math.log(len(grid), 2))
    t2 = 2 ** (int(t) + 1) - len(grid)
    sq_grid = np.vstack((grid, np.ones((t2, len(grid[0])))))
    t3 = abs(len(sq_grid) - len(sq_grid[0]))
    sq_grid = np.hstack((sq_grid, np.ones((len(sq_grid), t3))))
    return sq_grid


def calc_path_length(path):
    """
    Calculates the length of a path. Path is a list of coordinates.
    :param path: a list of coordinates
    :return: length of the path
    """
    total = 0
    for i in range(len(path) - 1):
        total = total + dist(path[i], path[i + 1])
    return total
