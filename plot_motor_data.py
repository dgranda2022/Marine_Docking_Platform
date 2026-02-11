import matplotlib.pyplot as plt
import csv
import sys
import glob
import os

def main():
    # 1. Find the latest CSV file
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        list_of_files = glob.glob('motor_log_*.csv')
        if not list_of_files:
            print("No log files found! Run motor_driver.py first.")
            return
        filename = max(list_of_files, key=os.path.getctime)

    print(f"Plotting data from: {filename}")

    # 2. Read Data
    times = []
    cmds = []
    positions = []
    torques = []
    speeds = []
    voltages = []

    try:
        with open(filename, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                times.append(float(row['Time']))
                cmds.append(float(row['Command_Torque']))
                positions.append(float(row['Position']))
                torques.append(float(row['Actual_Torque']))
                speeds.append(float(row['Speed']))
                voltages.append(float(row['Voltage']))
    except Exception as e:
        print(f"Error reading CSV: {e}")
        return

    # 3. Create Subplots
    fig, axs = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
    fig.suptitle(f"Motor Performance Report\nSource: {filename}", fontsize=16)

    # Plot 1: Position
    axs[0].plot(times, positions, 'b-', linewidth=2)
    axs[0].set_ylabel("Position (Deg)")
    axs[0].set_title("Position Response")
    axs[0].grid(True)

    # Plot 2: Torque (Command vs Actual)
    axs[1].plot(times, cmds, 'r--', label='Command (A)', alpha=0.7)
    axs[1].plot(times, torques, 'g-', label='Actual (A)', linewidth=1.5)
    axs[1].set_ylabel("Torque (Amps)")
    axs[1].set_title("Torque Tracking")
    axs[1].legend(loc='upper right')
    axs[1].grid(True)

    # Plot 3: Speed
    axs[2].plot(times, speeds, 'm-', linewidth=1.5)
    axs[2].set_ylabel("Speed (Raw/RPM)")
    axs[2].set_title("Motor Speed")
    axs[2].grid(True)

    # Plot 4: Voltage
    axs[3].plot(times, voltages, 'k-', linewidth=1.5)
    axs[3].set_ylabel("Voltage (V)")
    axs[3].set_title("Bus Voltage")
    axs[3].set_xlabel("Time (seconds)")
    axs[3].grid(True)

    # Save and Show
    output_img = filename.replace('.csv', '.png')
    plt.tight_layout()
    plt.savefig(output_img)
    print(f"Graph saved to: {output_img}")
    plt.show()

if __name__ == "__main__":
    main()
