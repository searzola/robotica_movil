#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import Empty


class Linear_PIDController( Node ):

  def __init__( self, kd = 0 ):
    super().__init__( 'linear_pid_controller' )
    self.declare_parameter( 'kp', 0.5)
    self.declare_parameter( 'ki', 0.0 )
    self.kp = self.get_parameter( 'kp' ).value
    self.ki = self.get_parameter( 'ki' ).value
    self.get_logger().info( 'Kp = %.3f' % (self.kp) )
    self.get_logger().info( 'Ki = %.3f' % (self.ki) )
    self.kd = kd
    self.setpoint = None
    self.state = None
    self.proportional_action = 0
    self.cumulative_error = 0.0

    self.actuation_pub = self.create_publisher( Float64, 'control_effort', 1 )
    self.dist_set_point_sub = self.create_subscription( Float64, 'setpoint', self.setpoint_cb, 1 )
    self.dist_state_sub = self.create_subscription( Float64, 'state', self.state_cb, 1 )

  def setpoint_cb( self, msg ):
    self.get_logger().info( '[PICTRL] new setpoint received: %.2f' % (msg.data) )
    self.reset()
    self.setpoint = msg.data

  def state_cb( self, msg ):
    if self.setpoint == None:
      return
    self.state = msg.data
    error = self.setpoint - self.state
    self.cumulative_error += error

    # Proportional
    p_actuation = self.kp*error

    # Integrative
    i_actuation = self.ki*self.cumulative_error

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
    self.cumulative_error = 0.0


def main():
  rclpy.init()
  lin_p_ctrl = Linear_PIDController()
  rclpy.spin( lin_p_ctrl )

if __name__ == '__main__':
  main()







