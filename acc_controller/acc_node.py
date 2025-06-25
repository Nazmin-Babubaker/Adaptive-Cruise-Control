import rclpy
from rclpy.node import Node
from inertial_msgs.msg import Pose as ImuPose
from radar_msgs.msg import RadarTrackList
from vehiclecontrol.msg import Control


LANE_WIDTH = 2
LANE_CENTER_Y = 0.0
LANE_MARGIN = 1  
cruise_acc = 0.25


class ACCNode(Node):
    def __init__(self):
        super().__init__('acc_node')
        self.create_subscription(ImuPose, '/InertialData', self.imu_callback, 10)
        self.create_subscription(RadarTrackList, '/radar_processed', self.radar_callback, 10)
        self.publisher = self.create_publisher(Control, '/vehicle_control', 10)
        self.prev_lead_speed = None 
        self.prev_time = None        
        self.ego_velocity = 0.0
        self.ego_position = 0.0
        self.lead_distance = None
        self.lead_speed = None

        self.timer_period = 0.01
        self.timer = self.create_timer(self.timer_period, self.control_loop)

    def imu_callback(self, msg):
        self.ego_velocity = msg.velocity.x
        self.ego_position = msg.position.x
        self.get_logger().info(f'EGO VELOCITY: {self.ego_velocity:.2f}')


    def radar_callback(self, msg):
        
        in_lane_objs = [o for o in msg.objects if abs(o.y_distance - LANE_CENTER_Y) <= LANE_MARGIN]

        if not in_lane_objs:
            self.lead_distance = None
            self.lead_speed = None
            return

        closest_obj = min(in_lane_objs, key=lambda o: abs(o.x_distance))
        self.lead_distance = closest_obj.x_distance
        self.lead_speed = closest_obj.vx
        self.get_logger().info(f'GAP: {self.lead_distance:.2f}, REL VELOCITY: {self.lead_speed:.2f}')
    

    


    def control_loop(self):
       
        if self.lead_distance is None or self.lead_speed is None:
            
             if self.ego_velocity < 7.5:
                 self.publish_control(cruise_acc)
             else:
                 self.publish_control(0.0)
             return
      


       
        time_gap = 1.05
        desired_gap = max(3.0, self.ego_velocity * time_gap + 5)
        gap_error = self.lead_distance - desired_gap
        rel_velocity = self.lead_speed  

        
        current_time = self.get_clock().now().nanoseconds / 1e9  

        if self.prev_lead_speed is not None and self.prev_time is not None:
            dt = current_time - self.prev_time
            if dt > 0:
                lead_accel = (self.lead_speed - self.prev_lead_speed) / dt
            else:
                lead_accel = 0.0
        else:
            lead_accel = 0.0

        
        self.prev_lead_speed = self.lead_speed
        self.prev_time = current_time

        Kp = 0.12 if self.lead_distance >= 3.0 else 0.16
        Kd = 0.06 if self.lead_distance >= 3.0 else 0.095

        Kf = 0.3  

        desired_acc = Kp * gap_error + Kd * rel_velocity - Kf * lead_accel


        self.publish_control(desired_acc)

    def publish_control(self, acc):
        acc = float(max(min(acc, 2.0), -3.0))

        ctrl_msg = Control()
        if acc >= 0:
            ctrl_msg.throttle = min(acc / 3, 1.0)
            ctrl_msg.brake = 0.0
        else:
            ctrl_msg.throttle = 0.0
            ctrl_msg.brake = min(-acc / 3, 1.0)

        ctrl_msg.steering = 0.0
        ctrl_msg.longswitch = 1
        ctrl_msg.latswitch = 0

        self.publisher.publish(ctrl_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ACCNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
