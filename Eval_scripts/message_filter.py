import logging
import rclpy
from rclpy.node import Node
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
import yaml
from tf2_msgs.msg import TFMessage

class BagFilterToYaml(Node):

    def __init__(self, input_bag, output_yaml, topic, frame_id, child_frame_id):
        super().__init__('bag_filter_to_yaml')
        self.input_bag = input_bag
        self.output_yaml = output_yaml
        self.topic = topic
        self.frame_id = frame_id
        self.child_frame_id = child_frame_id

    def filter_bag(self):
        # Setup storage options for reading
        storage_options = StorageOptions(uri=self.input_bag, storage_id='sqlite3')
        converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

        reader = SequentialReader()
        reader.open(storage_options, converter_options)

        filtered_messages = []

        while reader.has_next():
            try:
                topic, data, t = reader.read_next()
                if topic == self.topic:
                    msg = deserialize_message(data, TFMessage)
                    for transform in msg.transforms:
                        if transform.header.frame_id == self.frame_id and transform.child_frame_id == self.child_frame_id:
                            filtered_messages.append(self.transform_to_dict(transform, t))
            except Exception as e:
                logging.error(f"Error deserializing message: {e}")

        with open(self.output_yaml, 'w') as yaml_file:
            yaml.dump(filtered_messages, yaml_file, default_flow_style=False)
 


    
    def transform_to_dict(self, transform, timestamp):
        # Convert the transform to a dictionary for YAML serialization
        transform_dict = {
            # 'timestamp': timestamp.nanoseconds,  # Adjust if you need a different format
            'header': {
                'frame_id': transform.header.frame_id,
                'stamp':  transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9,
            },
            'child_frame_id': transform.child_frame_id,
            'pose': {
                'position': {
                    'x': transform.transform.translation.z, #z
                    'y': transform.transform.translation.x, #x
                    'z': transform.transform.translation.y, #y
                },
                'orientation': {
                    'x': transform.transform.rotation.z, #z
                    'y': transform.transform.rotation.x, #x
                    'z': transform.transform.rotation.y, #y 
                    'w': transform.transform.rotation.w,
                }
            }
        }
        return transform_dict

def main(args=None):
    rclpy.init(args=args)


    input_bag = '/media/caoadm/Extreme SSD/results/lego_32'
    output_yaml = 'lego_32.yaml'
    topic = '/tf'
    frame_id = '/camera_init'
    child_frame_id = '/camera'

    #input_bag = '/media/caoadm/Extreme SSD/ros_bags/zabbox_faster'
    #output_yaml = 'zabbox_faster_groundtruth.yaml'
    #topic = '/vehicle_blue/sim_ground_truth_pose'
    #frame_id = 'zab'
    #child_frame_id = 'vehicle_blue'

    bag_filter = BagFilterToYaml(input_bag, output_yaml, topic, frame_id, child_frame_id)
    bag_filter.filter_bag()

    

    rclpy.shutdown()

if __name__ == '__main__':
    main()
