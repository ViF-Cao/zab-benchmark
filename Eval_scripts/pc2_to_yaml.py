import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
import yaml
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

def pointcloud_to_dict(pointcloud):
    """
    Convert a PointCloud2 message to a dictionary with timestamps and x, y, z coordinates.
    """
    points_list = []
    for point in read_points(pointcloud, field_names=("x", "y", "z"), skip_nans=True):
        point_dict = {
            'x': float(point[2]),
            'y': float(point[0]),
            'z': float(point[1])
        }
        points_list.append(point_dict)
    
    return {
        'header': {
            'stamp': pointcloud.header.stamp.sec + pointcloud.header.stamp.nanosec * 1e-9,
            'frame_id': pointcloud.header.frame_id,
        },
        'position': points_list[-1]
    }

def convert_bag_to_yaml(bag_file, yaml_file, topic_name):
    """
    Convert a ROS 2 bag file to a YAML file for PointCloud2 data.
    """
    storage_options = StorageOptions(uri=bag_file, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    pointcloud_data = []
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == topic_name:
            msg_type = get_message('sensor_msgs/msg/PointCloud2')
            msg = deserialize_message(data, msg_type)
            pointcloud_data.append(pointcloud_to_dict(msg))

    with open(yaml_file, 'w') as outfile:
        yaml.dump(pointcloud_data, outfile, default_flow_style=False)

if __name__ == "__main__":
    rclpy.init()
    bag_file = 'LeGO_zabbox_refine'  # Replace with your input ROS 2 bag file
    yaml_file = 'lego_zabbox_refine_traj.yaml'  # Replace with your desired output YAML file
    topic_name = '/key_pose_origin'  # Replace with your topic name

    convert_bag_to_yaml(bag_file, yaml_file, topic_name)
    rclpy.shutdown()
