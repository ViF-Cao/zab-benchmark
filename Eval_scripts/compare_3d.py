# AI generated content included
import yaml
import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import plotly.graph_objs as go
import plotly.offline as pyo

def load_path_from_yaml(yaml_file):
    with open(yaml_file, 'r') as file:
        paths = yaml.safe_load(file)
    return paths

def extract_positions_and_timestamps(path_data):
    timestamps = np.array([pose['header']['stamp'] for pose in path_data])
    positions = np.array([[pose['pose']['position']['x'], 
                           pose['pose']['position']['y'], 
                           pose['pose']['position']['z']] for pose in path_data])
    return timestamps, positions

def extract_positions_and_timestamps_odom(path_data):
    timestamps = np.array([pose['header']['stamp'] for pose in path_data])
    positions = np.array([[pose['position']['x'], 
                           pose['position']['y'], 
                           pose['position']['z']] for pose in path_data])
    return timestamps, positions

def align_starting_timestamps(gt_timestamps, slam_timestamps):
    start_time = max(gt_timestamps[0], slam_timestamps[0])
    aligned_gt_timestamps = gt_timestamps - gt_timestamps[0] + start_time
    aligned_slam_timestamps = slam_timestamps - slam_timestamps[0] + start_time
    return aligned_gt_timestamps, aligned_slam_timestamps

def interpolate_positions(gt_timestamps, gt_positions, target_timestamps):
    interp_func = interp1d(gt_timestamps, gt_positions, axis=0, fill_value='extrapolate')
    interpolated_positions = interp_func(target_timestamps)
    return interpolated_positions

def visualize_paths(gt_positions, slam_positions, links):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    ax.plot(gt_positions[:, 0], gt_positions[:, 1], gt_positions[:, 2], label='Ground Truth Path', color='blue')
    ax.plot(slam_positions[:, 0], slam_positions[:, 1], slam_positions[:, 2], label='SLAM Path', color='red')
    
    for i, j in links:
        ax.plot([gt_positions[i, 0], slam_positions[j, 0]], 
                [gt_positions[i, 1], slam_positions[j, 1]], 
                [gt_positions[i, 2], slam_positions[j, 2]], color='green', linestyle='dotted')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.show()

def visualize_paths_2d(gt_positions, slam_positions, links):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='2d')
    
    ax.plot(gt_positions[:, 0], gt_positions[:, 1], label='Ground Truth Path', color='blue')
    ax.plot(slam_positions[:, 0], slam_positions[:, 1], label='SLAM Path', color='red')
    
    for i, j in links:
        ax.plot([gt_positions[i, 0], slam_positions[j, 0]], 
                [gt_positions[i, 1], slam_positions[j, 1]], 
                color='green', linestyle='dotted')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.legend()
    plt.show()

def visualize_paths_interactive(gt_positions, slam_positions, links):
    trace1 = go.Scatter3d(x=gt_positions[:, 0], y=gt_positions[:, 1], z=gt_positions[:, 2],
                          mode='lines', name='Ground Truth Path', line=dict(color='blue'))
    
    trace2 = go.Scatter3d(x=slam_positions[:, 0], y=slam_positions[:, 1], z=slam_positions[:, 2],
                          mode='lines', name='SLAM Path', line=dict(color='red'))
    
    traces = [trace1, trace2]
    
    for i, j in links:
        trace = go.Scatter3d(x=[gt_positions[i, 0], slam_positions[j, 0]], 
                             y=[gt_positions[i, 1], slam_positions[j, 1]], 
                             z=[gt_positions[i, 2], slam_positions[j, 2]], 
                             mode='lines', line=dict(color='green', dash='dot'), showlegend=False)
        traces.append(trace)
    
    layout = go.Layout(scene=dict(xaxis=dict(title='X'),
                                  yaxis=dict(title='Y'),
                                  zaxis=dict(title='Z')),
                       title='Path Comparison')
    
    fig = go.Figure(data=traces, layout=layout)
    pyo.plot(fig, filename='legoloam-zabbox_refine.html')

def calculate_errors(gt_positions, slam_positions):
    errors = np.linalg.norm(gt_positions - slam_positions, axis=1)
    print(errors)
    max_error = np.max(errors)
    min_error = np.min(errors)
    rmse = np.sqrt(np.mean(errors**2))
    return max_error, min_error, rmse

def calculate_2d_errors(gt_positions, slam_positions):
    """
    Calculate the max error, min error, and RMSE between the ground truth and SLAM 2D positions.

    Parameters:
    gt_positions (numpy.ndarray): Ground truth positions, shape (N, 3)
    slam_positions (numpy.ndarray): SLAM-generated positions, shape (N, 3)

    Returns:
    max_error (float): Maximum error between corresponding points in 2D
    min_error (float): Minimum error between corresponding points in 2D
    rmse (float): Root Mean Squared Error between corresponding points in 2D
    accuracy_rate (float): Percentage of points with error < 2 meters
    """
    # Extract the x and y coordinates for 2D error calculation
    gt_positions_2d = gt_positions[:, :2]
    slam_positions_2d = slam_positions[:, :2]
    
    # Compute the Euclidean distance between corresponding points in 2D
    errors = np.linalg.norm(gt_positions_2d - slam_positions_2d, axis=1)
    print(errors)  # Print errors for debugging/inspection

    # Compute the maximum error
    max_error = np.max(errors)
    
    # Compute the minimum error
    min_error = np.min(errors)
    
    # Compute the Root Mean Squared Error (RMSE)
    rmse = np.sqrt(np.mean(errors**2))
    
    # Compute accuracy rate: proportion of errors < 2 meters
    accuracy_rate = np.sum(errors < 2) / len(errors)

    return  max_error, min_error, rmse, accuracy_rate #

def main(gt_yaml_file, slam_yaml_file):
    gt_data = load_path_from_yaml(gt_yaml_file)
    slam_data = load_path_from_yaml(slam_yaml_file)

    gt_timestamps, gt_positions = extract_positions_and_timestamps(gt_data)
    slam_timestamps, slam_positions = extract_positions_and_timestamps(slam_data)

    # aligned_gt_timestamps, aligned_slam_timestamps = align_starting_timestamps(gt_timestamps, slam_timestamps)

    interpolated_gt_positions = interpolate_positions(gt_timestamps, gt_positions, slam_timestamps)

    links = list(zip(range(len(slam_positions)), range(len(slam_positions))))

    # visualize_paths(interpolated_gt_positions, slam_positions, links)
    # visualize_paths_interactive(interpolated_gt_positions, slam_positions, links)

    #max_error, min_error, rmse = calculate_errors(interpolated_gt_positions, slam_positions)
    max_error2, min_error2, rmse2, ar = calculate_2d_errors(interpolated_gt_positions, slam_positions) # 
    
    #print(f"Max Error: {max_error}")
    #print(f"Min Error: {min_error}")
    #print(f"RMSE: {rmse}")

    print(f"Max 2D error: {max_error2}")
    print(f"Min 2D error: {min_error2}")
    print(f"RMSE 2D: {rmse2}")
    print(f"Accuracy rate: {ar}")

if __name__ == "__main__":
    gt_yaml_file = 'zab0_refine_groundtruth.yaml'  # Replace with your ground truth YAML file
    slam_yaml_file = 'fastlio2_zab0_3.yaml'  # Replace with your SLAM generated path YAML file

    main(gt_yaml_file, slam_yaml_file)
