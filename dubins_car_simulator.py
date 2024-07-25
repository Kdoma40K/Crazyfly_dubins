import math

from geometry_msgs.msg import Pose

import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# Constants for the simulation
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 600
CAR_WIDTH = 20
CAR_HEIGHT = 10
BACKGROUND_COLOR = (255, 255, 255)
CAR_COLOR = (0, 0, 255)
FPS = 30


class DubinsCarSimulator(Node):
    def __init__(self):
        super().__init__('dubins_car_simulator')
        self.subscription = self.create_subscription(
            Float32,
            'turn_angle',
            self.turn_angle_callback,
            10)
        self.publisher_ = self.create_publisher(Pose, 'next_setpoint', 10)
        self.subscription  # prevent unused variable warning

        self.x = WINDOW_WIDTH / 2.0
        self.y = WINDOW_HEIGHT / 2.0
        self.z = 0.4
        self.yaw = 0.0
        self.velocity = 2.0  # pixels per frame
        self.dt = 1.0 / FPS  # s

        # Initialize Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption('Dubins Car Simulator')
        self.clock = pygame.time.Clock()

    def turn_angle_callback(self, msg):
        turn_angle = msg.data
        self.update_position(turn_angle)

    def update_position(self, turn_angle):
        self.x += self.velocity * math.cos(self.yaw) * self.dt
        self.y += self.velocity * math.sin(self.yaw) * self.dt
        self.yaw += turn_angle * self.dt

        pose_msg = Pose()
        pose_msg.position.x = self.x
        pose_msg.position.y = self.y
        pose_msg.position.z = self.z
        # Assuming yaw is the rotation around z-axis
        pose_msg.orientation.z = math.sin(self.yaw / 2.0)
        pose_msg.orientation.w = math.cos(self.yaw / 2.0)

        self.publisher_.publish(pose_msg)
        self.get_logger().info(f'Publishing: x={self.x}, y={self.y}, z={self.z}, yaw={self.yaw}')

    def draw_car(self):
        # Clear the screen
        self.screen.fill(BACKGROUND_COLOR)
        # Calculate car position and rotation
        car_center = (int(self.x), int(self.y))
        car_surface = pygame.Surface((CAR_WIDTH, CAR_HEIGHT))
        car_surface.fill(CAR_COLOR)
        rotated_car = pygame.transform.rotate(car_surface, -math.degrees(self.yaw))
        rotated_rect = rotated_car.get_rect(center=car_center)
        # Draw the car
        self.screen.blit(rotated_car, rotated_rect.topleft)
        pygame.display.flip()

    def main_loop(self):
        try:
            while rclpy.ok():
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        rclpy.shutdown()

                self.draw_car()
                rclpy.spin_once(self, timeout_sec=0.1)
                self.clock.tick(FPS)

        except KeyboardInterrupt:
            pass
        finally:
            pygame.quit()


def main(args=None):
    rclpy.init(args=args)
    node = DubinsCarSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
