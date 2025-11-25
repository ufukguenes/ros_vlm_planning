from sensor_msgs.msg import CompressedImage, CameraInfo
import rclpy
from rclpy.node import Node

class DummyImgPublisher(Node):
    def __init__(self):
        super().__init__('dummy_img_publisher')
        
        self.publisher_ = self.create_publisher(CompressedImage, 'dummy_img/compressed', 1)
        self.info_publisher_ = self.create_publisher(CameraInfo, 'camera_info', 1)


        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.dummy_img = CompressedImage()
        self.dummy_img.format = 'jpeg'
        
        img_path = '/home/ufuk/Documents/Programming/ros_vlm_planning/dummy_img.jpg'
        

        with open(img_path, 'rb') as f:
            self.dummy_img.data = f.read()
            self.get_logger().info('Image data loaded successfully.')

        self.dummy_img.header.frame_id = 'world'


        self.camera_info = CameraInfo()
        self.camera_info.header.frame_id = 'world'
        self.camera_info.width = 4000
        self.camera_info.height = 3000
        self.camera_info.distortion_model = 'plumb_bob'
        self.camera_info.k = [0.0] * 5
        self.camera_info.d = [0.0] * 5
        self.camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.camera_info.p = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]


        self.get_logger().info('Dummy Image Publisher Node Initialized.')


    def timer_callback(self):
        self.dummy_img.header.stamp = self.get_clock().now().to_msg()
        self.camera_info.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.dummy_img)
        self.info_publisher_.publish(self.camera_info)

def main(args=None):
    rclpy.init(args=args)

    dummy_img_publisher = DummyImgPublisher()

    rclpy.spin(dummy_img_publisher) 

    dummy_img_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()