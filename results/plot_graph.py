import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle

# plt.style.use('fivethirtyeight')

def read_file(arg1):
    with open('/home/israel/catkin_ws/src/tp_2_pmr/results/voronoi.txt', 'r') as arquivo:
        linhas = arquivo.readlines()
        eixo_x = []
        eixo_y = []
        for item in linhas:
            try:
                eixo_x.append(float(item.split('\t')[0]))
                eixo_y.append(float(item.split('\t')[1]))
            except:
                pass

    plt.cla()

    # rectangle = plt.Rectangle((-25,-25), 49, 50, fc='white',ec="black")
    # plt.gca().add_patch(rectangle)
    plt.plot(eixo_x, eixo_y, color='green')
    plt.axis('equal')

    # plt.grid()
    # Show the major grid lines with dark grey lines
    # plt.grid(b=True, which='major', color='#88888888', linestyle='-')
    # Show the minor grid lines with very faint and almost transparent grey lines
    plt.minorticks_on()
    # plt.grid(b=True, which='minor', color='#999999', linestyle='-', alpha=0.2)

    plt.xlim([-30, 30])
    plt.ylim([-25, 25])
    plt.tight_layout()


if __name__ == '__main__':    

    # ani = FuncAnimation(plt.gcf(), read_file, interval=10)
    read_file(1)

    plt.show()
