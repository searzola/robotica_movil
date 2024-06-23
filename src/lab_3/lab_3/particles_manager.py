#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseArray, Vector3
from tf_transformations import quaternion_from_euler

import time
import numpy as np
from random import gauss, uniform


class Particle( object ):

  def __init__( self, x, y, ang, sigma = 0.1 ):
    self.x, self.y, self.ang = x, y, ang
    self.last_x, self.last_y, self.last_ang = x, y, ang
    self.sigma = sigma

  def move( self, delta_x, delta_y, delta_ang ):
    self.x += delta_x +  gauss (0, self.sigma )
    self.y += delta_y + gauss( 0, self.sigma )
    self.ang += delta_ang + gauss( 0, self.sigma )

  def pos( self ):
    return [self.x, self.y, self.ang]

  def last_pos( self ):
    return [self.last_x, self.last_y, self.last_ang]


class ParticlesManager( Node ):
  
  def __init__( self, num_particles ):
    super().__init__( 'particles_manager' )
    self.num_particles = num_particles
    self.sigma = 0.01
    self.particles = []
    self.pub_particles = self.create_publisher(PoseArray, 'particles', 10 )
    self.particle_move_sub = self.create_subscription(Vector3, '/move_particles', self.update_particles, 10)
    # For testing only:
    #self.create_timer( 1.0, self.rotate_particles )

  def create_particles( self, range_x, range_y ):
    for i in range( 0, self.num_particles ):
      x = uniform( range_x[0], range_x[1] )
      y = uniform( range_y[0], range_y[1] )
      ang = uniform( -np.pi, np.pi )
      new_particle = Particle( x, y, ang, sigma = self.sigma )
      self.particles.append( new_particle )
    self.publish_particles()

  def update_particles( self, move_vector):
    delta_x = move_vector.x
    delta_y = move_vector.y
    delta_ang = move_vector.z
    for particle in self.particles:
      particle.move( delta_x, delta_y, delta_ang )
    self.publish_particles()

  def publish_particles(self):
    pose_array_msg = PoseArray()
    pose_array_msg.header = Header()
    pose_array_msg.header.frame_id = "map"

    for part in self.particles:
      part_pose = Pose()
      part_pose.position.x, part_pose.position.y = part.x, part.y
      quat = quaternion_from_euler(0,0, part.ang)

      part_pose.orientation.x = quat[0]
      part_pose.orientation.y = quat[1]
      part_pose.orientation.z = quat[2]
      part_pose.orientation.w = quat[3]

      pose_array_msg.poses.append(part_pose)

    self.pub_particles.publish(pose_array_msg)

  # For testing only:
  #def rotate_particles( self ):
  #  self.update_particles( 0, 0, (30*np.pi/180) )


def main():
  rclpy.init()

  map_width_pix = 270 # [pix]
  map_height_pix = 270 # [pix]
  map_resolution = 0.01 # [m/pix]

  map_width_m = map_width_pix * map_resolution
  map_height_m = map_height_pix * map_resolution

  particle_manager = ParticlesManager( num_particles = 100 )
  particle_manager.create_particles( [0, map_width_m], [0, map_height_m] )

  rclpy.spin( particle_manager )
  particle_manager.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()



