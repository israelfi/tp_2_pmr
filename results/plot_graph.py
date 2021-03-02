import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
from matplotlib.animation import FuncAnimation

plt.style.use('fivethirtyeight')

def read_file(arg1):
    with open('voronoi.txt', 'r') as arquivo:
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
    plt.plot(eixo_x, eixo_y, color='green')
    plt.tight_layout()


if __name__ == '__main__':    

    ani = FuncAnimation(plt.gcf(), read_file, interval=10)

    plt.show()
