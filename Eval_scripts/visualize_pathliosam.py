import yaml
import matplotlib.pyplot as plt

def read_yaml_file(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def extract_translation_data(data):
    timestamps = []
    x_vals = []
    y_vals = []
    z_vals = []

    for entry in data:
        # Extract the timestamp in a consistent format (e.g., seconds and nanoseconds combined)
        timestamp = entry['header']['stamp']
        timestamps.append(timestamp)

        # Extract the position values
        position = entry['pose']['position']
        x_vals.append( position['x'])
        y_vals.append( position['y'])
        z_vals.append( position['z'])

    return timestamps, x_vals, y_vals, z_vals

def plot_translation(timestamps, x_vals, y_vals, z_vals):
    plt.figure(figsize=(12, 8))

    plt.subplot(3, 1, 1)
    plt.plot(timestamps, x_vals, label='x', color='r')
    plt.xlabel('Time (s)')
    plt.ylabel('Translation X')
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 1, 2)
    plt.plot(timestamps, y_vals, label='y', color='g')
    plt.xlabel('Time (s)')
    plt.ylabel('Translation Y')
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 1, 3)
    plt.plot(timestamps, z_vals, label='z', color='b')
    plt.xlabel('Time (s)')
    plt.ylabel('Translation Z')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

def plot_3d_translation(x_vals, y_vals, z_vals):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x_vals, y_vals, z_vals, label='Translation (x, y, z)', color='b')
    ax.set_xlabel('Translation X')
    ax.set_ylabel('Translation Y')
    ax.set_zlabel('Translation Z')
    ax.legend()

    # Set the same scale for all axes
    max_range = max(max(x_vals) - min(x_vals), max(y_vals) - min(y_vals), max(z_vals) - min(z_vals))
    mid_x = (max(x_vals) + min(x_vals)) * 0.5
    mid_y = (max(y_vals) + min(y_vals)) * 0.5
    mid_z = (max(z_vals) + min(z_vals)) * 0.5

    ax.set_xlim(mid_x - max_range * 0.5, mid_x + max_range * 0.5)
    ax.set_ylim(mid_y - max_range * 0.5, mid_y + max_range * 0.5)
    ax.set_zlim(mid_z - max_range * 0.5, mid_z + max_range * 0.5)

    plt.show()

def main():
    yaml_file = 'lio_sam_traj.yaml'
    data = read_yaml_file(yaml_file)
    timestamps, x_vals, y_vals, z_vals = extract_translation_data(data)
    #plot_translation(timestamps, x_vals, y_vals, z_vals)
    plot_3d_translation(x_vals, y_vals, z_vals)

if __name__ == '__main__':
    main()

