from interfaces.srv import MapToPose
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header
import os
from rclpy.node import Node
import rclpy
import yaml

class MapToPoseService(Node):
    def __init__(self):
        super().__init__('map_to_pose_service')
        self.srv = self.create_service(MapToPose, 'map_to_pose', self.map_to_pose_callback) 

        self.declare_parameter('waypoints', 'hospital')
        self.waypoints_name = self.get_parameter('waypoints').get_parameter_value().string_value
        # Initialize saved coordinates
        self.named_coordinates = self.load_coordinates()


    def load_coordinates(self):
        try:
            yaml_file = os.path.join(
                              get_package_share_directory('services'),
                              'service_materials',
                              self.waypoints_name
                              )
            self.get_logger().info(f"Opening waypoint file {yaml_file}")
            with open(yaml_file, 'r') as file:
                data = yaml.safe_load(file)
            return data['waypoints']
        except Exception as e:
            self.get_logger().error(f"{e}")
            return {}



    def map_to_pose_callback(self, request, response):
        
        # list names that are request their saved Poses
        coordinate_names = request.coordinate_names
        self.get_logger().info(f"Incoming request for {coordinate_names}")


        # find Poses
        poses_found = []
        for name in coordinate_names:

            # Name for pose not does not exist
            if name not in self.named_coordinates.keys():
                self.get_logger().error(f"Location {name} not found!")
                continue
            
            
            pose_stamped = PoseStamped()

            pose_stamped.header = Header() 
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = "map"


            # Get saved pose.
            name_pose_orientation = self.named_coordinates[name]


            # Create new pose with values of 'pose' and 'orientation' that maps to 'name'
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = \
                float(name_pose_orientation['pose'][0]), float(name_pose_orientation['pose'][1]), float(name_pose_orientation['pose'][2]) 
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = \
                    float(name_pose_orientation['orientation'][0]), float(name_pose_orientation['orientation'][1]), float(name_pose_orientation['orientation'][2]), float(name_pose_orientation['orientation'][3])  


            pose_stamped.pose = pose
            poses_found.append(pose_stamped)

        # Set response to [] list of Poses
        response.pose_coordinates = poses_found
        return response


    

def main(args=None):
    rclpy.init(args=args)
    map_to_pose_service = MapToPoseService()
    rclpy.spin(map_to_pose_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()