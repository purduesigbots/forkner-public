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

def update_line(num, data, line):
    line.set_data(data[..., :num])
    return line,

def main(fname):
	with open(fname) as csv_file:
		csv_reader = csv.reader(csv_file, delimiter=',')
		line_count = 0
		length = 20
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
			if row[0] == 'O' and row[1] == ' sX':
				i = i + 1
				j = i
				foundfirstrow = 1
				continue
			elif row[0] != 'O':
				i = i + 1
				j = i
				continue
			if float(row[2]) == '0' or float(row[2]) > 4000:
				i = i + 1
				j = i
				continue
			x.append(float(row[1]))
			y.append(float(row[2]))
			t.append(float(row[3]))
			cx.append(float(row[4]))
			cy.append(float(row[5]))
			ct.append(float(row[6]))
			plt.cla()
			plt.plot(cx, cy, "-r", label="course")
			ct[i - j] = math.pi * ct[i - j] / 180
			terminus_cx = cx[i - j] + length * math.sin(ct[i - j])
			terminus_cy = cy[i - j] + length * math.cos(ct[i - j])
			plt.plot([cx[i - j], terminus_cx],[cy[i - j],terminus_cy], "-g")
			plt.plot(x, y, "-b", label="trajectory")
			t[i - j] = math.pi * t[i - j] / 180
			terminus_x = x[i - j] + length * math.sin(t[i - j])
			terminus_y = y[i - j] + length * math.cos(t[i - j])
			plt.plot([x[i - j], terminus_x],[y[i - j],terminus_y], "-y")
			plt.axis("equal")
			plt.grid(True)
			plt.pause(0.0001)
			i = i + 1

	plt.show()

	name = os.path.splitext(fname)[0]
	plt.cla()
	plt.plot(cx, cy, ".r", label="course")
	plt.plot(x, y, "-b", label="trajectory")
	plt.axis("equal")
	plt.grid(True)
	plt.savefig(name + '.png')

if __name__ == "__main__":
   main(sys.argv[1])
