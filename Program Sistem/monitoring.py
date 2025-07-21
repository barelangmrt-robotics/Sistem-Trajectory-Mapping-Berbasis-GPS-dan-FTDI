import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os
import math

initial_position = None

# Ukuran grid arena (25m x 25m)
grid_size = 25

# Titik bola merah
red_balls = [(-3.5, 9), (-4.8, 12),(-3, 14.6), (-9, 20.5), (-11 ,20.5), (-13,20.5), (-15,20.5), (-22,17),(-23.5, 13.5),(-23.5, 9.5)]
# Titik bola hijau
green_balls = [(-2, 9), (-3.3, 12), (-1.4, 14.6), (-9,22),(-11,22),(-13,22),(-15, 22),(-20.5,17),(-22.2,13.5),(-22.2,9.5)]
# Kotak start dan finish
red_square = (-3, 2)
blue_square = (-18, 3)
green_square = (-21, 6)

# Setup plot
fig, ax = plt.subplots()
ax.set_xlim(-grid_size, 0)
ax.set_ylim(0, grid_size)
ax.set_aspect('equal')
ax.grid(True)

# Tambahkan elemen tetap
for x, y in red_balls:
    ax.add_patch(plt.Circle((x, y), 0.3, color='red'))
for x, y in green_balls:
    ax.add_patch(plt.Circle((x, y), 0.3, color='green'))
ax.add_patch(plt.Rectangle((red_square[0]-0.5, red_square[1]-0.5), 1, 1, color='red'))
ax.add_patch(plt.Rectangle((blue_square[0]-0.5, blue_square[1]-0.5), 1, 1, color='blue'))
ax.add_patch(plt.Rectangle((green_square[0]-0.5, green_square[1]-0.5), 1, 1, color='green'))

# Titik robot (bergerak)
robot_dot, = ax.plot([], [], 'bo', markersize=10)
# Jejak lintasan
path_x, path_y = [], []
path_line, = ax.plot([], [], 'b--', linewidth=1)
# Arah panah
arrow = ax.arrow(0, 0, 0, 0, head_width=0.3, head_length=0.5, fc='blue', ec='blue')

# Fungsi update animasi
def update(frame):
    global arrow, initial_position
    if not os.path.exists("position.txt"):
        return robot_dot, path_line, arrow

    try:
        with open("position.txt", "r") as f:
            line = f.readline().strip()
            if line:
                x_str, y_str = line.split()
                x = float(x_str)
                y = float(y_str)

                # Inisialisasi posisi awal (saat pertama kali baca data)
                if initial_position is None:
                    initial_position = (x, y)

                # Hitung offset relatif posisi sekarang terhadap posisi awal
                dx = x - initial_position[0]
                dy = y - initial_position[1]

                # Jika posisi awal adalah (0,0), pindahkan ke koordinat kotak merah
                # Posisi robot di plot = kotak merah + offset
                robot_x = red_square[0] + dx
                robot_y = red_square[1] + dy

                # Update posisi robot
                robot_dot.set_data(robot_x, robot_y)

                # Simpan jejak
                path_x.append(robot_x)
                path_y.append(robot_y)
                path_line.set_data(path_x, path_y)

                # Update arah jika memungkinkan
                if len(path_x) >= 2:
                    vx = path_x[-1] - path_x[-2]
                    vy = path_y[-1] - path_y[-2]
                    mag = math.hypot(vx, vy)
                    if mag > 0.01:
                        vx /= mag
                        vy /= mag
                        arrow.remove()
                        arrow = ax.arrow(robot_x, robot_y, vx*1.0, vy*1.0, head_width=0.4, head_length=0.6, fc='blue', ec='blue')

    except Exception as e:
        print("Error reading file:", e)

    return robot_dot, path_line, arrow


# Judul dan label
plt.title("Simulasi Arena KKI ASV (Grid 25x25m) - Jejak & Arah Robot")
plt.xlabel("Meter (X)")
plt.ylabel("Meter (Y)")

# Jalankan animasi
ani = animation.FuncAnimation(fig, update, interval=500)
plt.show()
