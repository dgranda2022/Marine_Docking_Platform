import socket
import json
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

# --- CONFIGURATION ---
UDP_IP = "0.0.0.0"  # Listen to ALL incoming traffic
UDP_PORT = 5005
BUFFER_SIZE = 100   # How many data points to show on screen

# --- SETUP NETWORK ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)  # Don't freeze if no data comes in

# --- SETUP DATA STORAGE ---
times = deque(maxlen=BUFFER_SIZE)
roll_data = deque(maxlen=BUFFER_SIZE)
motor_data = deque(maxlen=BUFFER_SIZE)

# --- SETUP PLOT ---
fig, ax = plt.subplots()
line_roll, = ax.plot([], [], 'r-', label='IMU Roll (deg)')
line_motor, = ax.plot([], [], 'b--', label='Motor Target (deg)')

ax.set_ylim(-60, 60)
ax.set_xlim(0, BUFFER_SIZE)
ax.set_title("Real-Time Stabilizer Dashboard")
ax.set_ylabel("Angle (Degrees)")
ax.legend(loc='upper right')
ax.grid(True)

def update(frame):
    # 1. Read all waiting packets (drain the queue to get latest data)
    try:
        while True:
            data, addr = sock.recvfrom(1024)
            message = json.loads(data.decode())
            
            # Store data
            times.append(len(times)) # Simple counter
            roll_data.append(message['roll'])
            motor_data.append(message['motor'])
            
            # Update limits if robot goes wild
            if abs(message['roll']) > 45:
                ax.set_facecolor('#ffdddd') # Flash red if unsafe
            else:
                ax.set_facecolor('white')

    except BlockingIOError:
        pass  # No new data, just draw what we have

    # 2. Update Graph Lines
    line_roll.set_data(range(len(roll_data)), roll_data)
    line_motor.set_data(range(len(motor_data)), motor_data)
    
    return line_roll, line_motor

print(f"Listening for Jetson data on Port {UDP_PORT}...")
ani = FuncAnimation(fig, update, interval=20, blit=True) # 50 FPS
plt.show()
