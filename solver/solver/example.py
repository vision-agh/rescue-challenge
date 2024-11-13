from rclpy.node import Node
import rclpy


class ExampleClass(Node):

  def __init__(self):
      super().__init__('example_node')

      self.timer = self.create_timer(1.0/30.0, self.main)


  def main(self):
      pass
      # Your code here


def main(args=None):
    rclpy.init(args=args)
    example_node = ExampleClass()

    rclpy.spin(example_node)
    example_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()