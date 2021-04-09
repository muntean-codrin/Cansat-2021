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

for i in range(len(result)):
    xpoints.append(result[i][4])
    ypoints.append(result[i][6])

plt.plot(xpoints, ypoints, "-r")

plt.xlabel('Altitude (meters)')
plt.ylabel('Temperature (Celsius)')

plt.show()