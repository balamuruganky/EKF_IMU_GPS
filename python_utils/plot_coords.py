#!/usr/bin/python

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter

df = pd.read_csv("./test_data/gnss.csv")
print df.head()
print df.count()

df_gnss, df_filter = df.iloc[:,[1,2,3,4,5]], df.iloc[:,[6,7,8,9,10]]
bbox = ((df_gnss.longitude.min(),df_gnss.longitude.max(),df_gnss.latitude.min(),df_gnss.latitude.max()))
print bbox

data = (df_gnss, df_filter)
colors = ("blue", "green")
groups = ("GNSS Raw + Heading", "EKF Filtered GNSS + Heading")

# Create plot
fig, ax = plt.subplots()
ax.set_xlim(bbox[0],bbox[1])
ax.set_ylim(bbox[2],bbox[3])
ax.xaxis.set_major_formatter(FormatStrFormatter('%2.7f'))
ax.yaxis.set_major_formatter(FormatStrFormatter('%2.7f'))

for i, (data, color, group) in enumerate(zip(data, colors, groups)):
	if i == 0:
		x,y,yaw,pitch,roll = data.longitude, data.latitude, data.yaw, data.pitch, data.roll
	else:
		x,y,yaw,pitch,roll = data.filtered_longitude, data.filtered_latitude, data.filtered_yaw, data.filtered_pitch, data.filtered_roll
	_len = (len(x) - 1)
	ax.plot(x[0],y[0],"yo",markersize=15)
	ax.plot(x[_len],y[_len],"ro",markersize=15)
	ax.scatter(x, y, alpha=0.8, c=color, edgecolors='none', s=30, label=group)
	new_y = np.sin(yaw)
	new_x = np.cos(yaw)
	ax.quiver(x, y, new_x, new_y, width=0.001)


plt.title('GNSS Raw (Vs) EKF Filtered GNSS')
plt.legend(loc=2)
plt.show()
