#!/usr/bin/env python

import yaml
from matplotlib import pyplot as plt
from matplotlib import style
import numpy as np
from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument('c', help='contour file')
args = parser.parse_args()

style.use('seaborn-paper')

# search for last '/' that separates filename from path
i_slash = args.c.rfind('/')
i_dot = args.c.rfind('.')

BASEDIR = args.c[:i_slash + 1]
EXTENSION = args.c[i_dot:] # e.g. '.yaml'
filename = args.c[i_slash+1:i_dot]

def showContourTemplate(filename, ax):
    global BASEDIR, EXTENSION
    global fig
    with open(BASEDIR + str(filename) + EXTENSION) as f:
        plt.gca().clear()

        f.readline()  # skip the OpenCV header
        data = yaml.load(f)

        for contour_n in data["contour_trees"]:
            contour = contour_n["contour"]
            contour_arr = np.asarray(contour)
            contour_arr = np.reshape(contour_arr, (-1, 2))
            y_shift = -contour_n["distance"]
            contour_arr[:, 1] += y_shift

            plt.scatter(contour_arr[:, 1], contour_arr[:, 0])
            # add first point to close polygon
            contour_arr = np.concatenate([contour_arr, contour_arr[:-1, :]])
            plt.plot(contour_arr[:, 1], contour_arr[:, 0])

            for c in contour_n["children"]:
                contour = c["contour"]
                contour_arr = np.asarray(contour)
                contour_arr = np.reshape(contour_arr, (-1, 2))
                contour_arr = contour_arr - np.asarray(c["relative_position"])
                contour_arr[:, 0] += c["relative_position"][0]
                contour_arr[:, 1] += y_shift + c["relative_position"][1]

                plt.scatter(contour_arr[:, 1], contour_arr[:, 0])
                # add first point to close polygon
                contour_arr = np.concatenate([contour_arr, contour_arr[:-1, :]])
                plt.plot(contour_arr[:, 1], contour_arr[:, 0])

        ax.invert_xaxis()

        plt.show(block=False)

        ax.set_aspect('equal', adjustable='box')
        ax.set_title('Contours '+ str(filename) + EXTENSION)
        ax.set_xlabel('y')
        ax.set_ylabel('x')



def onclick(event):
    global filename, ax

    current_filename = int(filename)

    try:
        if(event.key == 'right'):
            current_filename += 1
        if(event.key == 'left'):
            current_filename -= 1

        if(event.key == 't'):
            import rospkg
            r = rospkg.RosPack()
            path = r.get_path('perception')

            import shutil
            shutil.copyfile(BASEDIR + str(filename) + EXTENSION,
                            path + '/data/contour_templates/teach_in_' + str(filename) + EXTENSION)

            print("Template copied to {}".format(path + '/data/contour_templates/teach_in_' + str(filename) + EXTENSION))

        showContourTemplate(current_filename, ax)
        filename = current_filename

    except(IOError):
        print("File {} not available.".format(current_filename))

    except(TypeError):
        filename = current_filename

fig, ax = plt.subplots()


try:
    current_filename = int(filename)
    filename = current_filename
    cid = fig.canvas.mpl_connect('key_press_event', onclick)
except ValueError:
    pass

showContourTemplate(filename, ax)


plt.show()
