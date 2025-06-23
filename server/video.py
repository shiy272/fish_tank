from flask import Flask, Response, request
import threading
import queue
import time

app = Flask(__name__)

# 使用线程安全的队列
frame_queue = queue.Queue(maxsize=10)
clients_lock = threading.Lock()
active_clients = set()
clients_last_active = {}

@app.route('/video_feed', methods=['POST'])
def video_feed():
    try:
        # 获取完整的帧数据
        frame_data = request.data
        
        # 将帧放入队列
        try:
            frame_queue.put_nowait(frame_data)
            print(f"Received frame: {len(frame_data)} bytes")
        except queue.Full:
            print("Frame queue full, dropping frame")
        
        return '', 204
    except Exception as e:
        print(f"Error processing frame: {str(e)}")
        return 'Server Error', 500

def generate_frames(client_id):
    """生成视频流，直接转发接收到的完整MJPEG帧"""
    print(f"New client connected: {client_id}")
    last_frame_time = time.time()
    
    with clients_lock:
        active_clients.add(client_id)
        clients_last_active[client_id] = time.time()
    
    try:
        while True:
            try:
                # 从队列获取帧
                frame_data = frame_queue.get(timeout=1.0)
                
                # 直接转发完整帧
                yield frame_data
                frame_queue.task_done()
                
                # 更新最后活动时间
                clients_last_active[client_id] = time.time()
                last_frame_time = time.time()
                
            except queue.Empty:
                # 每5秒发送keep-alive
                current_time = time.time()
                if current_time - last_frame_time > 5:
                    # 发送边界标记保持连接
                    keep_alive = b"--frame\r\n"
                    yield keep_alive
                    clients_last_active[client_id] = current_time
                    last_frame_time = current_time
                    print(f"Keep-alive sent to client {client_id}")
                
    except GeneratorExit:
        print(f"Client disconnected: {client_id}")
    finally:
        with clients_lock:
            if client_id in active_clients:
                active_clients.remove(client_id)
            if client_id in clients_last_active:
                del clients_last_active[client_id]
        print(f"Cleaned up client: {client_id}")

@app.route('/')
def index():
    return '''<html>
<head>
<title>ESP32 Camera Stream</title>
<script>
    // JavaScript自动重连代码
    // ... [如上所述] ...
</script>
<style>
    body { text-align: center; background: #121212; color: white; }
    img { max-width: 90%; margin: 20px auto; border: 2px solid #444; }
</style>
</head>
<body>
<h1>ESP32 Camera Live Stream</h1>
<img id="video-stream" />
</body>
</html>'''

@app.route('/stream')
def stream():
    """视频流端点"""
    client_id = threading.get_ident()
    return Response(
        generate_frames(client_id),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

def client_monitor():
    """监控空闲客户端"""
    while True:
        time.sleep(30)
        now = time.time()
        with clients_lock:
            inactive_clients = [
                cid for cid, last_active in clients_last_active.items()
                if now - last_active > 60  # 60秒无活动
            ]
            
            for client_id in inactive_clients:
                if client_id in active_clients:
                    active_clients.remove(client_id)
                if client_id in clients_last_active:
                    del clients_last_active[client_id]
                print(f"Removed inactive client: {client_id}")

def queue_monitor():
    """监控队列状态"""
    while True:
        time.sleep(10)
        print(f"Queue status: size={frame_queue.qsize()}, clients={len(active_clients)}")

if __name__ == '__main__':
    # 启动监控线程
    threading.Thread(target=client_monitor, daemon=True).start()
    threading.Thread(target=queue_monitor, daemon=True).start()
    
    app.run(host='0.0.0.0', port=5002, threaded=True)
