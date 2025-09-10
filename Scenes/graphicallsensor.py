import numpy as np
import matplotlib.pyplot as plt
import time, os
import Geometries.Constants as Const

plt.ion()

MAX_POINTS = 100  
file_path = "campo_global.txt"
num_sensores = Const.NSensors

figs, axes, lines, data_buffers = [], [], [], []

# Crear una figura por sensor
for i in range(num_sensores):
    fig, ax = plt.subplots(figsize=(6,4))  
    line_x, = ax.plot([], [], 'r-', label='X', linewidth=1.5)
    line_y, = ax.plot([], [], 'g-', label='Y', linewidth=1.5)
    line_z, = ax.plot([], [],  label='Z', linewidth=1.5)

    ax.set_ylabel("B [μT]")
    ax.set_xlabel("Timestep")
    ax.set_title(f"Sensor {i+1}")
    ax.legend()
    ax.grid(True)
    ax.ticklabel_format(style='sci', axis='y', scilimits=(0,0))

    figs.append(fig)
    axes.append(ax)
    lines.append((line_x, line_y, line_z))
    data_buffers.append(([], [], []))  

while True:
    try:
        if os.path.exists(file_path) and os.path.getsize(file_path) > 0:
            # Leer solo últimas N líneas
            data = np.genfromtxt(file_path, dtype=float)

            if data.ndim == 1:  
                data = data[np.newaxis, :]  


            for i in range(min(num_sensores, data.shape[0])):
                Bx, By, Bz = data[i]
                x_vals, y_vals, z_vals = data_buffers[i]

                x_vals.append(Bx)
                y_vals.append(By)
                z_vals.append(Bz)

                if len(x_vals) > MAX_POINTS:
                    x_vals[:] = x_vals[-MAX_POINTS:]
                    y_vals[:] = y_vals[-MAX_POINTS:]
                    z_vals[:] = z_vals[-MAX_POINTS:]

                t = np.arange(len(x_vals))

                lines[i][0].set_data(t, x_vals)
                lines[i][1].set_data(t, y_vals)
                lines[i][2].set_data(t, z_vals)

                axes[i].set_xlim(max(0, len(t) - MAX_POINTS), len(t))

                y_all = np.concatenate([x_vals, y_vals, z_vals])
                y_min, y_max = np.min(y_all), np.max(y_all)
                margin = 0.1 * (y_max - y_min + 1e-9)
                axes[i].set_ylim(y_min - margin, y_max + margin)

                figs[i].canvas.draw()
                figs[i].canvas.flush_events()

        time.sleep(0.01)  

    except KeyboardInterrupt:
        print("Interrumpido por el usuario.")
        break
    except Exception as e:
        print("Error:", e)
        time.sleep(0.5)
