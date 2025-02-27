import rclpy
from rclpy.node import Node
from interfaces.srv import MapToPose
from courier_navigation.courier_navigator import TurtleBot4Navigator
import sys

class MapToPoseClientAsync(Node):
    def __init__(self):
        super().__init__('map_to_client_async')
        self.cli = self.create_client(MapToPose, 'map_to_pose')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not avaiilable')

        self.req = MapToPose.Request()
        
    def send_request(self, coordinate_names):
        self.req.coordinate_names = coordinate_names
        return self.cli.call_async(self.req)


   

def main():

    rclpy.init()

    if len(sys.argv) < 2:
        print("Must provide coordinate(s) mapping names")
        sys.exit(1)

    requested_args = sys.argv[1:]


    client = MapToPoseClientAsync()
    future = client.send_request(coordinate_names=requested_args)

    rclpy.spin_until_future_complete(client, future)

    response = future.result()
    client.get_logger().info(f"Finished processing - response is {response.pose_coordinates}")
    
    
    robot =  TurtleBot4Navigator()
    robot.waitUntilNav2Active()

    try:
        robot.startFollowWaypoints(response.pose_coordinates)
    except Exception as e:
        print("Error navigating to directed poses")


    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

