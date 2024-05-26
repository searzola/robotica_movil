#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import Empty


class Angular_PController( Node ):

  def __init__( self, kp, ki = 0, kd = 0 ):
    super().__init__( 'angular_p_controller' )
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.setpoint = None
    self.state = None
    self.proportional_action = 0

    self.actuation_pub = self.create_publisher( Float64, 'angular_control_effort', 1 )
    self.dist_set_point_sub = self.create_subscription( Float64, 'angular_setpoint', self.setpoint_cb, 1 )
    self.dist_state_sub = self.create_subscription( Float64, 'angular_state', self.state_cb, 1 )

  def setpoint_cb( self, msg ):
    self.get_logger().info( '[APCTRL] new setpoint received: %.2f' % (msg.data) )
    self.reset()
    self.setpoint = msg.data

  def state_cb( self, msg ):
    if self.setpoint == None:
      return
    self.state = msg.data
    error = self.setpoint - self.state

    # Proportional
    p_actuation = self.kp*error

    # Integrative (Implement me!)
    i_actuation = self.ki*0

    # Derivative (Implement me!)
    d_actuation = self.kd*0

    # Actuation
    actuation = p_actuation + i_actuation + d_actuation

    # Message sending
    msg = Float64()
    msg.data = actuation
    self.actuation_pub.publish( msg )

  def reset( self ):
    self.setpoint = None
    self.state = None


def main():
  rclpy.init()
  ang_p_ctrl = Angular_PController( 0.5 )
  rclpy.spin( ang_p_ctrl )

if __name__ == '__main__':
  main()







