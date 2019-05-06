##
# Pure Pursuit Visualizer Tool
#
# To view live:
# https://stackoverflow.com/questions/43397162/show-matplotlib-plots-in-ubuntu-windows-subsystem-for-linux
##
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
import numpy as np
import sys, os
import csv
import math

x = []
y = []
t = []
cx = []
cy = []
ct = []
tx = []
ty = []

def update_line(num, data, line):
    line.set_data(data[..., :num])
    return line,

def main(fname):
	# line, = ax.plot(x, x - 5, 'r-', linewidth=2)
	with open(fname) as csv_file:
		csv_reader = csv.reader(csv_file, delimiter=',')
		line_count = 0
		length = 0.1
		i = 0
		j = 0
		foundfirstrow = 0
		for row in csv_reader:
			try:
				var = row[0]
			except IndexError:
				i = i + 1
				j = i
				continue
			if row[0] != 'sx' and not foundfirstrow:
				i = i + 1
				j = i
				continue
			elif row[0] == 'sx':
				i = i + 1
				j = i
				foundfirstrow = 1
				continue
			if 'WARNING' in row[0]:
				i = i + 1
				j = i
				continue
			x.append(float(row[0]))
			y.append(float(row[1]))
			t.append(float(row[2]))
			cx.append(float(row[3]))
			cy.append(float(row[4]))
			ct.append(float(row[5]))
			tx.append(float(row[6]))
			ty.append(float(row[7]))
			plt.cla()
			plt.plot(cx, cy, ".r", label="course")
			plt.plot(x, y, "-b", label="trajectory")
			terminus_x = x[i - j] + length * math.sin(t[i - j])
			terminus_y = y[i - j] + length * math.cos(t[i - j])
			plt.plot([x[i - j], terminus_x],[y[i - j],terminus_y])
			plt.plot(tx[i - j], ty[i - j], "xg", label="target")
			plt.plot([x[i - j], tx[i - j]], [y[i - j], ty[i - j]], "-y")
			plt.axis("equal")
			plt.grid(True)
			plt.pause(0.001)
			i = i + 1

	plt.show()

	name = os.path.splitext(fname)[0]
	plt.cla()
	plt.plot(cx, cy, ".r", label="course")
	plt.plot(x, y, "-b", label="trajectory")
	plt.axis("equal")
	plt.grid(True)
	plt.xlabel("Time[s]")
	plt.ylabel("Speed[km/h]")
	plt.savefig(name + '.png')

if __name__ == "__main__":
   main(sys.argv[1])
