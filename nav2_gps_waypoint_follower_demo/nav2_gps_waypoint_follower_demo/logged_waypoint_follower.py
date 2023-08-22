import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
import yaml
from ament_index_python.packages import get_package_share_directory
import os

from nav2_gps_waypoint_follower_demo.utils.gps_utils import latLonYaw2Geopose


class YamlWaypointParser:
    """
    Parse a set of gps waypoints from a yaml file
    """
    def __init__(self, waypoints_file_url: str) -> None:
        with open(waypoints_file_url, 'r') as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)

    def get_wps(self):
        """
        Get an array of geographic_msgs/msg/GeoPose objects from the yaml file
        """
        gepose_wps = []
        for wp in self.wps_dict["waypoints"]:
            latitude, longitude, yaw = wp["latitude"], wp["longitude"], wp["yaw"]
            gepose_wps.append(latLonYaw2Geopose(latitude, longitude, yaw))
        return gepose_wps
    
class GpsWpCommander(Node):
    """
    Node to use nav2 gps waypoint follower to follow a set of waypoints logged in a yaml file
    """
    def __init__(self):
        super().__init__(node_name="gps_wp_commander")
        self.navigator = BasicNavigator("basic_navigator")
        
        wps_file_dir = os.path.join(get_package_share_directory("nav2_gps_waypoint_follower_demo"), "config", "demo_waypoints.yaml")
        self.wp_parser = YamlWaypointParser(wps_file_dir)

        self.wpf_tmr = self.create_timer(1.0, self.start_wpf)

    def start_wpf(self):
        """
        Timer callback to start the waypoint following
        """
        self.navigator.waitUntilNav2Active(localizer='robot_localization')
        self.wpf_tmr.cancel()
        wps = self.wp_parser.get_wps()
        self.navigator.followGpsWaypoints(wps)
        if(self.navigator.isTaskComplete()):
            self.get_logger().info("wps completed successfully")

def main():
    rclpy.init()
    gps_wpf = GpsWpCommander()
    rclpy.spin(gps_wpf)

if __name__ == "__main__":
    main()

        


