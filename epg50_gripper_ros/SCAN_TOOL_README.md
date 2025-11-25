# EPG50 Slave ID 扫描工具使用指南

## 快速开始

### 1. 编译工具
```bash
cd /home/ma/roboarm/aicontest
colcon build --packages-select epg50_gripper_ros
source install/setup.zsh
```

### 2. 检查串口设备
```bash
# 查看可用的串口设备
ls -l /dev/ttyACM* /dev/ttyUSB*

# 查看最近的USB设备连接日志
dmesg | tail -n 30 | grep -Ei 'tty|usb'
```

### 3. 运行扫描工具

#### 基础用法（扫描全部 ID）
```bash
# 默认扫描 /dev/ttyACM0，范围 1-255
ros2 run epg50_gripper_ros scan_slave_id
```

#### 指定串口
```bash
# 如果设备在 /dev/ttyUSB0
ros2 run epg50_gripper_ros scan_slave_id -p /dev/ttyUSB0
```

#### 限制扫描范围（加快速度）
```bash
# 只扫描常见范围 1-32
ros2 run epg50_gripper_ros scan_slave_id -s 1 -e 32

# 只扫描 0x08-0x10 (十六进制)
ros2 run epg50_gripper_ros scan_slave_id -s 0x08 -e 0x10
```

#### 查看帮助
```bash
ros2 run epg50_gripper_ros scan_slave_id --help
```

## 权限问题处理

### 临时方案（仅当前会话）
```bash
sudo chmod 666 /dev/ttyACM0
```

### 永久方案（推荐）
```bash
# 将用户加入 dialout 组
sudo usermod -a -G dialout $USER

# 退出并重新登录，或者：
newgrp dialout

# 验证是否生效
groups | grep dialout
```

## 使用扫描结果

假设扫描发现 Slave ID 为 `0x10` (十进制 16)：

```bash
# 方法1: 使用 launch 文件启动（推荐）
ros2 launch epg50_gripper_ros launch.py port:=/dev/ttyACM0 default_slave_id:=16

# 方法2: 直接运行节点
ros2 run epg50_gripper_ros epg50_gripper_node --ros-args \
  -p port:=/dev/ttyACM0 \
  -p default_slave_id:=16
```

## 故障排查

### 1. 未找到任何设备
- **检查连接**: 确保夹爪已通电并连接到 USB
- **检查设备路径**: `ls /dev/tty*` 确认设备名称
- **查看内核日志**: `dmesg | tail -n 50 | grep tty`

### 2. 权限被拒绝
```bash
# 检查当前权限
ls -l /dev/ttyACM0

# 应该显示类似: crw-rw---- 1 root dialout ...
# 确保你在 dialout 组
groups
```

### 3. 设备已被占用
```bash
# 查找占用进程
fuser /dev/ttyACM0
# 或
lsof | grep ttyACM0

# 停止占用进程
sudo kill -9 <PID>
```

### 4. 波特率不匹配
该工具固定使用 **115200 8N1**，这是 EPG50 的标准配置。如果你的设备使用不同波特率，需要修改源码。

## 常见 Slave ID

EPG50 夹爪常见的出厂 Slave ID：
- `0x09` (十进制 9) - 较老的固件
- `0x10` (十进制 16) - 较新的固件
- 自定义 ID - 通过 `rename_gripper` 服务修改过

## 扫描示例输出

```
═══════════════════════════════════════════════
   EPG50 Gripper Slave ID 扫描工具
═══════════════════════════════════════════════
配置:
  串口: /dev/ttyACM0
  波特率: 115200 8N1
  扫描范围: 0x1 - 0xff

开始扫描 Slave ID 范围: 0x1 - 0xff
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

扫描进度: 0x1 (1/255) ........
✓ 发现设备! Slave ID = 0x10 (十进制: 16)
...............................
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

扫描完成!
✓ 找到 1 个设备:
  - Slave ID: 0x10 (十进制: 16)

使用方法:
  ros2 launch epg50_gripper_ros launch.py default_slave_id:=16
```

## 技术细节

- **协议**: Modbus RTU
- **功能码**: 0x03 (读保持寄存器)
- **测试寄存器**: 0x07D0-0x07D3 (EPG50 状态寄存器)
- **超时**: 200ms/每个ID
- **延迟**: 20ms/每次查询（避免总线拥塞）

## 高级选项

### 快速扫描（仅扫描可能的 ID）
```bash
# 扫描 8-20 范围（最常见）
ros2 run epg50_gripper_ros scan_slave_id -s 8 -e 20
```

### 调试模式
如需更详细的输出，可修改源码中的调试选项，或使用 `strace` 追踪：
```bash
strace -e openat,read,write ros2 run epg50_gripper_ros scan_slave_id
```

## 相关链接

- EPG50 夹爪文档: [查看产品手册]
- Modbus RTU 协议: [Modbus 官网](https://modbus.org/)
- ROS 2 串口编程: [ROS 2 文档](https://docs.ros.org/)
