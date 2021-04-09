import cv2
import configparser
import math
import sys
import time
import matplotlib.pyplot as plt
import numpy as np

with open("F1333.TXT") as file:
    liness = file.readlines()
liness.pop()
result = [([float(x) for x in line.split(",")]) for line in liness]

xpoints = []
ypoints = []

time = 0

for i in range(len(result)):
    xpoints.append(time)
    ypoints.append(result[i][4])
    time = time + 0.5

plt.plot(xpoints, ypoints, "-b")

plt.xlabel('Time (seconds)')
plt.ylabel('Altitude (meters)')

plt.show()