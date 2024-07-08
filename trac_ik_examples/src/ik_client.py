import rclpy
from rclpy.node import Node
from trac_ik_examples.srv import IkService

class IKClient(Node):
    def __init__(self):
        super().__init__('ik_client')
        self.client = self.create_client(IkService, 'ik_service')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('IK service not available, waiting...')

    def move_robot(self, pos_x, pos_y, pos_z, orient_x, orient_y, orient_z, orient_w):
        request = IkService.Request()
        request.pos_x = pos_x
        request.pos_y = pos_y
        request.pos_z = pos_z
        request.orient_x = orient_x
        request.orient_y = orient_y
        request.orient_z = orient_z
        request.orient_w = orient_w

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result().joint_angles
        else:
            self.get_logger().error('Service call failed')
            return None

def main(args=None):
    rclpy.init(args=args)
    ik_client = IKClient()
    
    # Example usage
    joint_angles = ik_client.move_robot(
        -0.07226132922801609, -0.03117821036125143, 0.37203747270478615,
        -0.48683405994888185, -0.49617100675010856, -0.395045938017476, 0.6006210427467262
    )
    
    if joint_angles:
        print('Received joint angles:', joint_angles)
    
    ik_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
