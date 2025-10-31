import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import time
import threading
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class AdvancedCameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        self.bridge = CvBridge()
        
        # QoS настройки для видео (лучше подходят для потокового видео)
        video_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.camera_pub = self.create_publisher(Image, '/junky/camera/image', video_qos)
        self.status_pub = self.create_publisher(String, '/junky/camera_status', 10)
        
        self.cap = None
        self.is_running = True
        self.frame_count = 0
        self.start_time = time.time()
        
        # Статистика
        self.fps = 0
        self.last_stats_time = time.time()
        self.processing_time = 0
        
        # Параметры
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480) 
        self.declare_parameter('target_fps', 30)
        self.declare_parameter('show_fps', False)
        
        self.initialize_camera()
        
        # Основной цикл в отдельном потоке для максимальной производительности
        self.capture_thread = threading.Thread(target=self.capture_loop)
        self.capture_thread.daemon = True
        self.capture_thread.start()
        
        self.get_logger().info('Advanced camera node started')

    def initialize_camera(self):
        try:
            camera_id = self.get_parameter('camera_id').value
            width = self.get_parameter('width').value
            height = self.get_parameter('height').value
            target_fps = self.get_parameter('target_fps').value
            
            self.cap = cv2.VideoCapture(camera_id)
            
            if not self.cap.isOpened():
                self.get_logger().error(f'Failed to open camera {camera_id}')
                # Попытка найти другую камеру
                for i in range(3):
                    self.cap = cv2.VideoCapture(i)
                    if self.cap.isOpened():
                        self.get_logger().info(f'Found camera at index {i}')
                        break
            
            if not self.cap.isOpened():
                self.get_logger().error('No cameras available')
                return
            
            # Оптимальные настройки для минимальной задержки
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_FPS, target_fps)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Минимальный буфер
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            
            # Проверяем реальные значения
            actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
            
            self.get_logger().info(
                f'Camera: {actual_width}x{actual_height} @ {actual_fps:.1f} FPS'
            )
            
        except Exception as e:
            self.get_logger().error(f'Camera init error: {str(e)}')

    def capture_loop(self):
        last_frame_time = time.time()
        target_frame_time = 1.0 / self.get_parameter('target_fps').value
        
        while self.is_running and rclpy.ok():
            try:
                # Регулировка FPS
                current_time = time.time()
                elapsed = current_time - last_frame_time
                
                if elapsed < target_frame_time:
                    time.sleep(max(0, target_frame_time - elapsed - 0.001))
                    continue
                
                if not self.cap or not self.cap.isOpened():
                    time.sleep(0.1)
                    continue
                
                # Захват кадра
                ret, frame = self.cap.read()
                
                if not ret:
                    self.get_logger().warn('Frame capture failed')
                    time.sleep(0.01)
                    continue
                
                # Обработка и публикация
                self.process_and_publish_frame(frame)
                
                # Статистика
                self.frame_count += 1
                last_frame_time = current_time
                
                # Обновление FPS каждую секунду
                if current_time - self.last_stats_time >= 1.0:
                    self.fps = self.frame_count / (current_time - self.last_stats_time)
                    self.frame_count = 0
                    self.last_stats_time = current_time
                    
            except Exception as e:
                self.get_logger().error(f'Capture loop error: {str(e)}')
                time.sleep(0.1)

    def process_and_publish_frame(self, frame):
        start_time = time.time()
        
        try:
            # Добавление FPS на кадр (опционально)
            if self.get_parameter('show_fps').value:
                cv2.putText(frame, f'FPS: {self.fps:.1f}', (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Конвертация и публикация
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "camera"
            
            self.camera_pub.publish(ros_image)
            
        except Exception as e:
            self.get_logger().error(f'Frame processing error: {str(e)}')
        
        self.processing_time = time.time() - start_time

    def status_callback(self):
        status_msg = String()
        if self.cap and self.cap.isOpened():
            status_msg.data = (
                f"Активна | FPS: {self.fps:.1f} | "
                f"Обработка: {self.processing_time*1000:.1f}ms | "
                f"Режим: Непрерывный цикл"
            )
        else:
            status_msg.data = "Неактивна"
        
        self.status_pub.publish(status_msg)

    def destroy_node(self):
        self.is_running = False
        if hasattr(self, 'capture_thread') and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=2.0)
        
        if self.cap:
            self.cap.release()
            cv2.destroyAllWindows()
        
        self.get_logger().info('Camera node shutdown complete')
        super().destroy_node()

def main():
    rclpy.init()
    node = AdvancedCameraNode()
    
    # Таймер для статуса
    status_timer = node.create_timer(2.0, node.status_callback)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
