import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import time
import threading

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        self.bridge = CvBridge()
        
        # Используем стандартный QoS для совместимости
        self.camera_pub = self.create_publisher(Image, '/junky/camera/image', 10)
        self.status_pub = self.create_publisher(String, '/junky/camera_status', 10)
        
        self.cap = None
        self.is_running = True
        self.frame_count = 0
        self.start_time = time.time()
        
        # Статистика
        self.fps = 0
        self.last_stats_time = time.time()
        
        self.initialize_camera()
        
        # Основной цикл в отдельном потоке
        self.capture_thread = threading.Thread(target=self.capture_loop)
        self.capture_thread.daemon = True
        self.capture_thread.start()
        
        # Таймер для статуса
        self.status_timer = self.create_timer(2.0, self.status_callback)
        
        self.get_logger().info('Camera node started')

    def initialize_camera(self):
        try:
            self.cap = cv2.VideoCapture(0)
            
            if not self.cap.isOpened():
                self.get_logger().error('Cannot open camera')
                return
                
            # Настройка параметров камеры
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            # Проверяем реальные значения
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
            
            self.get_logger().info(
                f'Camera initialized: {actual_width}x{actual_height} at {actual_fps:.1f} FPS'
            )
            
        except Exception as e:
            self.get_logger().error(f'Camera initialization error: {str(e)}')

    def capture_loop(self):
        while self.is_running and rclpy.ok():
            try:
                if self.cap is None or not self.cap.isOpened():
                    time.sleep(0.1)
                    continue
                
                # Чтение кадра
                ret, frame = self.cap.read()
                
                if not ret:
                    self.get_logger().warn('Failed to read frame from camera')
                    time.sleep(0.1)
                    continue
                
                # Публикация кадра
                self.publish_frame(frame)
                
                self.frame_count += 1
                
                # Расчет FPS
                current_time = time.time()
                if current_time - self.last_stats_time >= 1.0:
                    self.fps = self.frame_count / (current_time - self.last_stats_time)
                    self.frame_count = 0
                    self.last_stats_time = current_time
                    
            except Exception as e:
                self.get_logger().error(f'Error in camera loop: {str(e)}')
                time.sleep(0.1)

    def publish_frame(self, frame):
        try:
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "camera_frame"
            self.camera_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f'Error converting image: {str(e)}')

    def status_callback(self):
        status_msg = String()
        if self.cap and self.cap.isOpened():
            width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            status_msg.data = (
                f"Активна | Разрешение: {width}x{height} | "
                f"FPS: {self.fps:.1f}"
            )
        else:
            status_msg.data = "Неактивна"
        
        self.status_pub.publish(status_msg)

    def destroy_node(self):
        self.is_running = False
        if hasattr(self, 'capture_thread') and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=1.0)
        
        if self.cap:
            self.cap.release()
        
        super().destroy_node()

def main():
    rclpy.init()
    node = CameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()