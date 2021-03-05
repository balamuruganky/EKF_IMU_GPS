#!/usr/bin/python

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter

df = pd.read_csv("./test_data/gnss.csv")
print df.head()
print df.count()

df_gnss, df_filter = df.iloc[:,[1,2]], df.iloc[:,[3,4]]
bbox = ((df_gnss.longitude.min(),df_gnss.longitude.max(),df_gnss.latitude.min(),df_gnss.latitude.max()))
print bbox

data = (df_gnss, df_filter)
colors = ("blue", "green")
groups = ("GNSS Raw", "EKF Filtered GNSS")

# Create plot
fig, ax = plt.subplots()
ax.set_xlim(bbox[0],bbox[1])
ax.set_ylim(bbox[2],bbox[3])
ax.xaxis.set_major_formatter(FormatStrFormatter('%2.7f'))
ax.yaxis.set_major_formatter(FormatStrFormatter('%2.7f'))

for i, (data, color, group) in enumerate(zip(data, colors, groups)):
	if i == 0:
		x,y = data.longitude, data.latitude
	else:
		x,y = data.filtered_longitude, data.filtered_latitude
	ax.scatter(x, y, alpha=0.8, c=color, edgecolors='none', s=30, label=group)

plt.title('GNSS Raw (Vs) EKF Filtered GNSS')
plt.legend(loc=2)
plt.show()
