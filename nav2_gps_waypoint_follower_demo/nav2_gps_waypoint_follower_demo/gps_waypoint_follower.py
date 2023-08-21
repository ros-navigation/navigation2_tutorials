import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geographic_msgs.msg import GeoPose
from geometry_msgs.msg import Quaternion
import yaml
from ament_index_python.packages import get_package_share_directory
import math
import os

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q 

def latLonYaw2Geopose(latitude: float, longitude: float, yaw: float) -> GeoPose:
    geopose = GeoPose()
    geopose.position.latitude = latitude
    geopose.position.longitude = longitude
    geopose.orientation = quaternion_from_euler(0.0, 0.0, yaw)
    return geopose

class YamlWaypointParser:
    def __init__(self, waypoints_file_url: str) -> None:
        with open(waypoints_file_url, 'r') as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)

    def get_wps(self): 
        gepose_wps = []
        for wp in self.wps_dict["waypoints"]:
            latitude, longitude, yaw = wp["latitude"], wp["longitude"], wp["yaw"]
            gepose_wps.append(latLonYaw2Geopose(latitude, longitude, yaw))
        return gepose_wps
    
class GpsWpCommander(Node):
    def __init__(self):
        super().__init__(node_name="gps_wp_commander")
        self.navigator = BasicNavigator("basic_navigator")
        
        wps_file_dir = os.path.join(get_package_share_directory("nav2_gps_waypoint_follower_demo"), "config", "demo_waypoints.yaml")
        self.wp_parser = YamlWaypointParser(wps_file_dir)

        self.wpf_tmr = self.create_timer(1.0, self.start_wpf)

    def start_wpf(self):
        # self.navigator.waitUntilNav2Active(localizer='rl')
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

        


