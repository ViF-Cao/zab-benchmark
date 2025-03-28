# AI generated content included
import yaml
import numpy as np

def load_yaml(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def extract_positions(path_data):
    positions = np.array([[pose['pose']['position']['x'],
                           pose['pose']['position']['y'],
                           pose['pose']['position']['z']] for pose in path_data])
    return positions

def calculate_trajectory_length(positions):
    """
    Calculate the total length of the trajectory based on the 3D positions.

    Parameters:
    positions (numpy.ndarray): Array of positions, shape (N, 3)

    Returns:
    float: Total length of the trajectory
    """
    # Calculate the Euclidean distance between consecutive points
    distances = np.linalg.norm(positions[1:] - positions[:-1], axis=1)
    # Sum the distances to get the total length
    total_length = np.sum(distances)
    return total_length

def main(yaml_file):
    path_data = load_yaml(yaml_file)
    positions = extract_positions(path_data)
    total_length = calculate_trajectory_length(positions)
    print(f"Total trajectory length: {total_length} meters")

if __name__ == "__main__":
    yaml_file = 'zabbox_32_groundtruth.yaml'  # Replace with your actual YAML file path
    main(yaml_file)
