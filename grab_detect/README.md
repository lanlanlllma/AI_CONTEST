# Grab Detect

ROS 2 抓取检测服务包，通过 socket 与外部抓取检测服务器通信。

## 功能特性

- **话题监听模式**：监听触发话题，自动处理最新的 color/depth/mask 图像
- **服务调用模式**：提供 ROS 2 服务接口，可直接传入图像进行检测
- **Socket 通信**：通过 TCP socket 与抓取检测服务器通信
- **灵活配置**：支持 YAML 配置文件自定义话题、服务器地址等参数

## 消息和服务定义

### GraspResult.msg
```
float64 score                     # 抓取质量得分
float64 width                     # 抓取宽度
float64 height                    # 抓取高度
float64 depth                     # 抓取深度
float64[3] translation            # 平移向量 [x, y, z]
float64[9] rotation_matrix        # 旋转矩阵（行优先存储）
```

### GraspDetect.srv
```
# 请求
sensor_msgs/Image color_image     # RGB彩色图像
sensor_msgs/Image depth_image     # 深度图像
sensor_msgs/Image mask_image      # 可选的掩码图像
---
# 响应
bool success                      # 检测是否成功
int32 num_grasps                  # 检测到的抓取数量
string message                    # 返回消息或错误描述
GraspResult[] grasps              # 抓取结果数组
```

### GraspDetectTrigger.srv
```
# 请求（空，使用最新订阅的图像）
---
# 响应
bool success                      # 检测是否成功
int32 num_grasps                  # 检测到的抓取数量
string message                    # 返回消息或错误描述
GraspResult[] grasps              # 抓取结果数组
```

### GraspDetectTriggerMask.srv
```
# 请求（仅传入 mask，color 和 depth 使用最新订阅的图像）
sensor_msgs/Image mask_image      # 掩码图像
---
# 响应
bool success                      # 检测是否成功
int32 num_grasps                  # 检测到的抓取数量
string message                    # 返回消息或错误描述
GraspResult[] grasps              # 抓取结果数组
```

## 编译

```bash
cd /home/ma/roboarm/aicontest
colcon build --packages-select grab_detect
source install/setup.zsh
```

## 配置

编辑 `config/grasp_detect.yaml`：

```yaml
grasp_detect_node:
  ros__parameters:
    # Socket服务器配置
    server_host: "localhost"      # 服务器地址
    server_port: 5555             # 服务器端口
    socket_timeout: 30.0          # 超时时间（秒）
    
    # 话题配置
    trigger_topic: "/grasp_detect/trigger"  # 触发话题
    color_image_topic: "/color/image_raw"   # 彩色图像话题
    depth_image_topic: "/depth/image_raw"   # 深度图像话题
    mask_image_topic: ""                    # 掩码话题（可选）
    
    # 服务配置
    service_name: "grasp_detect"   # 服务名称
```

## 使用方法

### 1. 启动节点

使用默认配置：
```bash
ros2 launch grab_detect grasp_detect.launch.py
```

使用自定义配置：
```bash
ros2 launch grab_detect grasp_detect.launch.py config:=/path/to/your/config.yaml
```

### 2. 话题触发模式

节点会订阅配置中指定的 color、depth、mask 图像话题。发布一个触发消息即可自动处理：

```bash
ros2 topic pub --once /grasp_detect/trigger std_msgs/msg/Empty
```

节点会使用最新接收到的图像进行检测，结果会打印在终端。

### 3. 服务调用模式

#### 3.1 直接传图像（GraspDetect）

使用命令行（需要准备图像数据）：
```bash
ros2 service call /grasp_detect grab_detect/srv/GraspDetect \
  "{color_image: {...}, depth_image: {...}, mask_image: {...}}"
```

#### 3.2 触发式调用（GraspDetectTrigger）

使用最新订阅的图像进行检测：
```bash
ros2 service call /grasp_detect_trigger grab_detect/srv/GraspDetectTrigger
```

#### 3.3 触发式调用 + 自定义 Mask（GraspDetectTriggerMask）

使用最新订阅的 color/depth 图像 + 自定义 mask：
```bash
ros2 service call /grasp_detect_trigger_mask grab_detect/srv/GraspDetectTriggerMask \
  "{mask_image: {...}}"
```

Python 客户端示例：
```python
import rclpy
from rclpy.node import Node
from grab_detect.srv import GraspDetectTriggerMask
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class GraspTriggerMaskClient(Node):
    def __init__(self):
        super().__init__('grasp_trigger_mask_client')
        self.client = self.create_client(GraspDetectTriggerMask, 'grasp_detect_trigger_mask')
        self.bridge = CvBridge()
        
    def call_service(self, mask_image_array):
        request = GraspDetectTriggerMask.Request()
        
        # 转换 mask 图像（假设是 numpy 数组）
        request.mask_image = self.bridge.cv2_to_imgmsg(mask_image_array, "mono8")
        
        # 等待服务可用
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待服务...')
        
        # 调用服务
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        if response.success:
            print(f"检测成功！找到 {response.num_grasps} 个抓取点")
            for i, grasp in enumerate(response.grasps[:5]):
                print(f"抓取 {i+1}: score={grasp.score:.4f}, "
                      f"pos={grasp.translation}")
        else:
            print(f"检测失败: {response.message}")
        
        return response

# 使用
rclpy.init()
client = GraspTriggerMaskClient()
# 创建一个示例 mask (例如从检测结果生成)
mask = np.zeros((720, 1280), dtype=np.uint8)
mask[100:200, 100:200] = 255  # 感兴趣区域
result = client.call_service(mask)
```

#### 3.4 直接传图像的 Python 示例（GraspDetect）
```python
import rclpy
from rclpy.node import Node
from grab_detect.srv import GraspDetectTrigger

class GraspTriggerClient(Node):
    def __init__(self):
        super().__init__('grasp_trigger_client')
        self.client = self.create_client(GraspDetectTrigger, 'grasp_detect_trigger')
        
    def call_service(self):
        request = GraspDetectTrigger.Request()
        
        # 等待服务可用
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待服务...')
        
        # 调用服务
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        if response.success:
            print(f"检测成功！找到 {response.num_grasps} 个抓取点")
            for i, grasp in enumerate(response.grasps[:5]):
                print(f"抓取 {i+1}: score={grasp.score:.4f}, "
                      f"pos={grasp.translation}")
        else:
            print(f"检测失败: {response.message}")
        
        return response

# 使用
rclpy.init()
client = GraspTriggerClient()
result = client.call_service()
```

### 4. 服务对比

| 服务类型 | 请求参数 | 返回值 | 使用场景 |
|---------|---------|---------|----------|
| 话题触发 (`/grasp_detect/trigger`) | 无 | 无（仅打印） | 简单测试 |
| GraspDetectTrigger | 无 | 完整抓取结果 | 使用最新图像，需要结果 |
| GraspDetectTriggerMask | 仅 mask 图像 | 完整抓取结果 | 自定义感兴趣区域 |
| GraspDetect | 3个图像（color/depth/mask） | 完整抓取结果 | 传入特定图像 |

#### 3.4 直接传图像的 Python 示例（GraspDetect）

使用命令行（需要准备图像数据）：
```bash
ros2 service call /grasp_detect grab_detect/srv/GraspDetect \
  "{color_image: {...}, depth_image: {...}, mask_image: {...}}"
```

使用 Python 客户端示例：
```python
import rclpy
from rclpy.node import Node
from grab_detect.srv import GraspDetect
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class GraspClient(Node):
    def __init__(self):
        super().__init__('grasp_client')
        self.client = self.create_client(GraspDetect, 'grasp_detect')
        self.bridge = CvBridge()
        
    def call_service(self, color_path, depth_path):
        request = GraspDetect.Request()
        
        # 读取图像
        color_img = cv2.imread(color_path)
        depth_img = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
        
        # 转换为ROS消息
        request.color_image = self.bridge.cv2_to_imgmsg(color_img, "bgr8")
        request.depth_image = self.bridge.cv2_to_imgmsg(depth_img, "16UC1")
        
        # 调用服务
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        if response.success:
            print(f"检测成功！找到 {response.num_grasps} 个抓取点")
            for i, grasp in enumerate(response.grasps[:5]):
                print(f"抓取 {i+1}: score={grasp.score:.4f}, "
                      f"pos={grasp.translation}")
        else:
            print(f"检测失败: {response.message}")

# 使用
rclpy.init()
client = GraspClient()
client.call_service('/path/to/color.png', '/path/to/depth.png')
```

## Socket 协议

节点与服务器的通信协议：

**发送格式**：
1. color_size (4 bytes, uint32, 网络字节序)
2. color_data (PNG 编码)
3. depth_size (4 bytes, uint32, 网络字节序)
4. depth_data (PNG 编码)
5. mask_size (4 bytes, uint32, 网络字节序)
6. mask_data (PNG 编码，无 mask 则 size=0)

**接收格式**：
1. result_size (4 bytes, uint32, 网络字节序)
2. result_data (JSON 格式)

**JSON 格式**：
```json
{
  "success": true,
  "num_grasps": 10,
  "message": "Detection completed",
  "grasps": [
    {
      "score": 0.95,
      "width": 0.08,
      "height": 0.02,
      "depth": 0.05,
      "translation": [0.1, 0.2, 0.3],
      "rotation_matrix": [[1,0,0], [0,1,0], [0,0,1]]
    }
  ]
}
```

## 测试

使用提供的 demo 脚本测试 socket 服务器：
```bash
python3 socket_client_demo.py \
  --host localhost \
  --port 5555 \
  --color_path /path/to/color.png \
  --depth_path /path/to/depth.png
```

## 依赖

- ROS 2 Humble
- OpenCV
- cv_bridge
- jsoncpp
- sensor_msgs
- std_msgs

## 注意事项

1. 确保 socket 服务器已启动并监听指定端口
2. 图像话题的 frame_id 和时间戳会被保留
3. 深度图像支持 16UC1 和 32FC1 编码
4. 超时时间可根据服务器处理速度调整
5. mask_image 可选，留空则不发送
