#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May 19 15:54:19 2025

@author: benjamin
"""
%matplotlib qt
import numpy as np
import matplotlib.pyplot as plt
import time
import os


plt.ion()


x_vals, y_vals, z_vals = [], [], []
MAX_POINTS = 100  

# Crear la figura
fig, ax = plt.subplots()
line_x, = ax.plot([], [], 'r-', label='X')
line_y, = ax.plot([], [], 'g-', label='Y')
line_z, = ax.plot([], [], 'b-', label='Z')

# Configurar ejes
ax.set_xlabel("Timestep")
ax.set_ylabel("Magnetic field [μT]")
ax.set_title("Real-time Magnetic Field (Sensor 0)")
ax.grid(True)
ax.legend()

file_path = "campo_global.txt"

while True:
    try:
        if os.path.exists(file_path) and os.path.getsize(file_path) > 0:
            data = np.loadtxt(file_path)

            if data.size == 0:
                continue

            if data.ndim == 1:
                row = data
            else:
                row = data[0]

            x_vals.append(row[0])
            y_vals.append(row[1])
            z_vals.append(row[2])

            if len(x_vals) > MAX_POINTS:
                x_vals = x_vals[-MAX_POINTS:]
                y_vals = y_vals[-MAX_POINTS:]
                z_vals = z_vals[-MAX_POINTS:]

 
            t = np.arange(len(x_vals))

            line_x.set_data(t, x_vals)
            line_y.set_data(t, y_vals)
            line_z.set_data(t, z_vals)

            # ax.set_xlim(0, MAX_POINTS)
            ax.set_xlim(max(0, len(t) - MAX_POINTS), len(t))

            y_all = x_vals + y_vals + z_vals
            ax.set_ylim(min(y_all) - 1000000, max(y_all) + 1000000)


            fig.canvas.draw()
            fig.canvas.flush_events()

        time.sleep(0.1)

    except KeyboardInterrupt:
        print("Interrumpido por el usuario.")
        break
    except Exception as e:
        print("Error:", e)
        time.sleep(0.5)
