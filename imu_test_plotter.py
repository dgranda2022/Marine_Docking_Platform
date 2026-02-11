import matplotlib.pyplot as plt
import csv
import sys
import glob
import os

def main():
    # Find the most recent CSV file if none provided
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        list_of_files = glob.glob('imu_log_*.csv')
        if not list_of_files:
            print("No log files found!")
            return
        filename = max(list_of_files, key=os.path.getctime)

    print(f"Plotting data from: {filename}")

    times, rolls, pitches, yaws = [], [], [], []

    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            times.append(float(row['Time']))
            rolls.append(float(row['Roll']))
            pitches.append(float(row['Pitch']))
            yaws.append(float(row['Yaw']))

    # Create the Plot
    plt.figure(figsize=(12, 6))
    
    plt.plot(times, rolls, label='Roll', color='red', linewidth=2)
    plt.plot(times, pitches, label='Pitch', color='blue', linewidth=1.5, alpha=0.7)
    # Yaw is often less critical for balance, but included for completeness
    plt.plot(times, yaws, label='Yaw', color='green', linestyle='--', alpha=0.5)

    # Add "Proof" lines
    plt.axhline(45, color='gray', linestyle='--', linewidth=1, label='Target +/- 45°')
    plt.axhline(-45, color='gray', linestyle='--', linewidth=1)
    plt.axhline(0, color='black', linewidth=1)

    plt.title(f"IMU Sensor Validation\nSource: {filename}")
    plt.xlabel("Time (seconds)")
    plt.ylabel("Angle (Degrees)")
    plt.legend()
    plt.grid(True, which='both', linestyle='--', alpha=0.6)
    
    output_file = filename.replace('.csv', '.png')
    plt.savefig(output_file)
    print(f"Graph saved to: {output_file}")

if __name__ == "__main__":
    main()
