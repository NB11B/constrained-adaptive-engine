import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import os

def visualize_flight(csv_path):
    df = pd.read_csv(csv_path)
    flight_id = os.path.basename(csv_path).replace('.csv', '')
    
    # 1. 3D Trajectory Plot
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot trajectory
    ax.plot(df['pos_x'], df['pos_y'], df['pos_z'], label='Drone Path', color='blue', alpha=0.7)
    
    # Plot target/pad location (taking the last known target)
    target_x = df['target_x'].iloc[-1]
    target_y = df['target_y'].iloc[-1]
    target_z = df['target_z'].iloc[-1]
    ax.scatter([target_x], [target_y], [target_z], color='red', s=100, label='Landing Pad', marker='X')
    
    # Highlight start point
    ax.scatter([df['pos_x'].iloc[0]], [df['pos_y'].iloc[0]], [df['pos_z'].iloc[0]], color='green', s=50, label='Start')
    
    # Highlight end point
    ax.scatter([df['pos_x'].iloc[-1]], [df['pos_y'].iloc[-1]], [df['pos_z'].iloc[-1]], color='orange', s=50, label='End/Collision')
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(f'Flight Trajectory: {flight_id}')
    ax.legend()
    
    plt.savefig(f'trajectory_{flight_id}.png')
    plt.close()
    
    # 2. XY Top-Down View with Focus on Landing
    plt.figure(figsize=(10, 10))
    plt.plot(df['pos_x'], df['pos_y'], label='Drone Path', color='blue', alpha=0.5)
    plt.scatter([target_x], [target_y], color='red', s=200, label='Landing Pad', marker='X')
    
    # Draw a circle for the 4.25m landing gate
    circle = plt.Circle((target_x, target_y), 4.25, color='green', fill=False, linestyle='--', label='Landing Gate (4.25m)')
    plt.gca().add_patch(circle)
    
    # Draw a circle for the terminal assist gate (0.75m for Warehouse)
    term_gate = 0.75 if df['terrain_id'].iloc[0] == 5 else 1.45
    circle2 = plt.Circle((target_x, target_y), term_gate, color='orange', fill=False, linestyle=':', label=f'Terminal Gate ({term_gate}m)')
    plt.gca().add_patch(circle2)
    
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title(f'Top-Down View: {flight_id}')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    
    # Zoom in on the landing area
    plt.xlim(target_x - 10, target_x + 10)
    plt.ylim(target_y - 10, target_y + 10)
    
    plt.savefig(f'topdown_{flight_id}.png')
    plt.close()
    
    # 3. Altitude vs Distance Plot
    plt.figure(figsize=(12, 6))
    plt.plot(df['dist_xy'], df['pos_z'], label='Altitude (Z)', color='purple')
    plt.axhline(y=target_z, color='red', linestyle='--', label='Pad Altitude')
    plt.xlabel('Distance to Pad XY (m)')
    plt.ylabel('Altitude Z (m)')
    plt.title(f'Altitude vs Distance: {flight_id}')
    plt.gca().invert_xaxis() # Move toward 0 distance
    plt.legend()
    plt.grid(True)
    
    plt.savefig(f'altitude_{flight_id}.png')
    plt.close()

    # 4. Pad-Frame Centering Plot
    plt.figure(figsize=(8, 8))
    plt.plot(df['pad_frame_dx'], df['pad_frame_dy'], label='Drone Position in Pad Frame', color='blue', alpha=0.7)
    plt.scatter([0], [0], color='red', s=200, label='Pad Center', marker='X')
    plt.xlabel('Pad Frame X (m)')
    plt.ylabel('Pad Frame Y (m)')
    plt.title(f'Pad-Frame Centering: {flight_id}')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.xlim(-1.0, 1.0)
    plt.ylim(-1.0, 1.0)
    plt.savefig(f'pad_frame_centering_{flight_id}.png')
    plt.close()
    
    print(f"Visualizations saved for {flight_id}")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        visualize_flight(sys.argv[1])
    else:
        print("Usage: python3 visualize_flight.py <path_to_csv>")
