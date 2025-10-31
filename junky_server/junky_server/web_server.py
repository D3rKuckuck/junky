import rclpy
from rclpy.node import Node
from flask import Flask, render_template, Response, jsonify, request
import threading
import os
import json
import time
import cv2
import base64
from io import BytesIO
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Получаем путь к пакету
from ament_index_python.packages import get_package_share_directory

app = Flask(__name__)

# Глобальные переменные для управления доступом
current_controller = None  # 'pc', 'phone', или None
controller_lock = threading.Lock()

# Глобальные переменные для видео
latest_frame = None
frame_lock = threading.Lock()
bridge = CvBridge()

class WebServerNode(Node):
    def __init__(self):
        super().__init__('web_server')
        
        # Получаем путь к share директории пакета
        package_share_directory = get_package_share_directory('junky_server')
        self.templates_dir = os.path.join(package_share_directory, 'templates')
        
        # Устанавливаем путь к шаблонам для Flask
        app.template_folder = self.templates_dir
        
        self.get_logger().info(f'Templates directory: {self.templates_dir}')
        
        # Публикаторы для управления
        self.key_pub = self.create_publisher(String, '/junky/keyboard_control', 10)
        self.speed_pub = self.create_publisher(Int32, '/junky/speed_level', 10)
        self.joy_pub = self.create_publisher(Twist, '/junky/joystick_control', 10)
        self.mode_pub = self.create_publisher(Int32, '/junky/control_mode', 10)
        
        # Подписчики для статуса
        self.camera_status_sub = self.create_subscription(
            String, '/junky/camera_status', self.camera_status_callback, 10)
        
        # Подписчик для камеры (стандартный QoS)
        self.camera_sub = self.create_subscription(
            Image, '/junky/camera/image', self.camera_callback, 10)
        
        self.camera_status = "Неактивна"
        self.get_logger().info('Web server node started')

    def camera_status_callback(self, msg):
        self.camera_status = msg.data

    def camera_callback(self, msg):
        global latest_frame
        try:
            # Конвертируем ROS Image в OpenCV
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            with frame_lock:
                latest_frame = cv_image
        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {str(e)}')

    def publish_key(self, key, state):
        msg = String()
        msg.data = f"{key}:{state}"
        self.key_pub.publish(msg)

    def publish_speed(self, speed_level):
        msg = Int32()
        msg.data = speed_level
        self.speed_pub.publish(msg)

    def publish_joystick(self, left_y, right_x):
        msg = Twist()
        msg.linear.x = float(left_y) / 100.0
        msg.angular.z = float(right_x) / 100.0
        self.joy_pub.publish(msg)

    def publish_control_mode(self, mode):
        msg = Int32()
        msg.data = mode
        self.mode_pub.publish(msg)

# Глобальный экземпляр узла
web_node = None

def generate_frames():
    while True:
        with frame_lock:
            if latest_frame is not None:
                try:
                    # Конвертируем кадр в JPEG
                    ret, buffer = cv2.imencode('.jpg', latest_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    if ret:
                        frame = buffer.tobytes()
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                except Exception as e:
                    print(f"Error encoding frame: {e}")
        time.sleep(0.033)  # ~30 FPS

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    try:
        return render_template('index.html')
    except Exception as e:
        return f"Error loading template: {str(e)}", 500

@app.route('/control/pc')
def control_pc():
    user_agent = request.headers.get('User-Agent', '').lower()
    if 'mobile' in user_agent or 'android' in user_agent or 'iphone' in user_agent:
        return "Доступ запрещен: эта страница предназначена только для ПК", 403
    
    with controller_lock:
        global current_controller
        if current_controller is not None and current_controller != 'pc':
            return "Робот уже управляется с другого устройства", 403
        current_controller = 'pc'
        if web_node:
            web_node.publish_control_mode(0)
    
    try:
        return render_template('control_pc.html')
    except Exception as e:
        return f"Error loading template: {str(e)}", 500

@app.route('/control/phone')
def control_phone():
    user_agent = request.headers.get('User-Agent', '').lower()
    if 'mobile' not in user_agent and 'android' not in user_agent and 'iphone' not in user_agent:
        return "Доступ запрещен: эта страница предназначена только для телефонов", 403
    
    with controller_lock:
        global current_controller
        if current_controller is not None and current_controller != 'phone':
            return "Робот уже управляется с другого устройства", 403
        current_controller = 'phone'
        if web_node:
            web_node.publish_control_mode(1)
    
    try:
        return render_template('control_phone.html')
    except Exception as e:
        return f"Error loading template: {str(e)}", 500

@app.route('/sensors')
def sensors():
    try:
        return render_template('sensors.html')
    except Exception as e:
        return f"Error loading template: {str(e)}", 500

@app.route('/api/keypress', methods=['POST'])
def handle_keypress():
    data = request.json
    key = data.get('key')
    state = data.get('state')
    
    if key and state is not None and web_node:
        web_node.publish_key(key, state)
        return jsonify({'status': 'success'})
    
    return jsonify({'status': 'error'})

@app.route('/api/speed', methods=['POST'])
def handle_speed():
    data = request.json
    speed_level = data.get('speed_level')
    
    if speed_level is not None and web_node:
        web_node.publish_speed(speed_level)
        return jsonify({'status': 'success'})
    
    return jsonify({'status': 'error'})

@app.route('/api/joystick', methods=['POST'])
def handle_joystick():
    data = request.json
    left_y = data.get('left_y')
    right_x = data.get('right_x')
    
    if left_y is not None and right_x is not None and web_node:
        web_node.publish_joystick(left_y, right_x)
        return jsonify({'status': 'success'})
    
    return jsonify({'status': 'error'})

@app.route('/api/release_control', methods=['POST'])
def release_control():
    with controller_lock:
        global current_controller
        current_controller = None
        if web_node:
            web_node.publish_control_mode(-1)
    
    return jsonify({'status': 'success'})

@app.route('/api/camera_status')
def get_camera_status():
    if web_node:
        return jsonify({'status': web_node.camera_status})
    return jsonify({'status': 'Web node not available'})

# Статические файлы
@app.route('/static/css/style.css')
def static_css():
    css = """
    .control-grid {
        display: grid;
        grid-template-columns: repeat(3, 1fr);
        gap: 5px;
        max-width: 150px;
        margin: 0 auto;
    }
    
    .control-grid button:nth-child(1) { grid-column: 2; }
    .control-grid button:nth-child(2) { grid-column: 1; grid-row: 2; }
    .control-grid button:nth-child(3) { grid-column: 2; grid-row: 2; }
    .control-grid button:nth-child(4) { grid-column: 3; grid-row: 2; }
    
    .control-btn.active {
        background-color: #007bff;
        color: white;
    }
    
    .speed-controls {
        display: grid;
        grid-template-columns: repeat(3, 1fr);
        gap: 3px;
    }
    
    .speed-btn.active {
        background-color: #28a745;
        color: white;
        border-color: #28a745;
    }
    
    .current-state {
        background-color: #f8f9fa;
        padding: 10px;
        border-radius: 5px;
        border: 1px solid #dee2e6;
    }
    
    .joystick-container {
        position: relative;
        width: 100px;
        height: 100px;
        margin: 0 auto;
    }
    
    .joystick-outer {
        width: 100%;
        height: 100%;
        border: 2px solid #007bff;
        border-radius: 50%;
        position: relative;
        touch-action: none;
    }
    
    .joystick-inner {
        width: 40px;
        height: 40px;
        background-color: #007bff;
        border-radius: 50%;
        position: absolute;
        top: 50%;
        left: 50%;
        transform: translate(-50%, -50%);
        transition: all 0.1s;
    }
    """
    return Response(css, mimetype='text/css')

@app.route('/static/js/script.js')
def static_js():
    js = """
    // Basic JavaScript functionality
    console.log('Junky Robot JS loaded');
    """
    return Response(js, mimetype='application/javascript')

def main():
    global web_node
    
    rclpy.init()
    web_node = WebServerNode()
    
    # Запуск Flask в отдельном потоке
    flask_thread = threading.Thread(
        target=lambda: app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    )
    flask_thread.daemon = True
    flask_thread.start()
    
    try:
        rclpy.spin(web_node)
    except KeyboardInterrupt:
        pass
    finally:
        web_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()