# Detector 模块

## 功能描述
Detector模块负责相机目标检测功能，采用TensorRT加速的LW-DETR和YOLOv8模型进行双层推理：
1. 第一层：寻找车辆 LW-DETR 
2. 第二层：寻找车辆对应装甲板 YOLOV8 

在寻找车辆时框出了远处的ROI区域 
对每块ROI区域进行多上下文推理 **num_ROI最大为9**

## 输入话题
- `/image_for_radar` - 相机图像话题

## 发布话题
- `/detector/detections` - 检测结果
>- 自定义消息类型具体见`lidar_interfaces`
>- 若没有检测到装甲板 则该车辆的`is_empty`为true，armor.id=-1, armor.score=0, armor.bbox=[0,0,0,0]
- `/detector/img_car` - 车辆图像
- `/detector/img_detect` - 结果图像

## 依赖
- TensorRT
- OpenCV
- CUDA
- ROS2

## 文件结构
- `detector_node.hpp/cpp` - ROS2节点主体
- `yolov8.hpp/cpp` - YOLOv8模型加载和推理实现 
- `lw_detr.hpp/cpp` - LW-DETR模型加载和推理实现

## 使用方法
1. 确保已安装TensorRT和相关依赖-例：`export LIBRARY_PATH=/home/phoenix/TensorRT-8.6.1.6/lib::$LIBRARY_PATH`
2. 确保模型文件路径正确
3. 启动节点即可进行目标检测

## 性能指标
- 推理速度：在 `56hz发布节点、ROI区域数量为6、四个可视化窗口、四个终端` 的情况下 
>- 车辆检测时间在 21 ms 
>- 单个装甲板检测时间在 0.7 ms 可忽略
>- 总耗时在 47 ms
>- `/detector/detect`节点在 19 hz 
- 检测准确率：
>- 对只露出一个头的英雄依然识别不稳定
>- 装甲板识别较差

## 硬件消耗
在60帧视频下 开三个终端和一个可视化
- GPU：4080 40% 1.7GB

## 未来计划
- 提高检测准确率 尤其是装甲板的处理
- 完善车辆及装甲板分类
- 重构代码 提高可维护性
- yolov8的多上下文推理


## infer输出 多batch_size
由于无法导出多batch_size的模型，所以实际没有实现。
```
// 原始图像（2张）
std::vector<cv::Mat> images = {img0, img1};

// 推理时间
double inference_time = 15.3;

// 检测框（按图像分别组织）
std::vector<std::vector<float>> boxes = {
    {100.0, 200.0, 150.0, 250.0,   // 图像0，目标1
     300.0, 400.0, 350.0, 450.0},  // 图像0，目标2

    {50.0,  60.0,  100.0, 120.0}   // 图像1，目标1
};

// 置信度（按图像分别组织）
std::vector<std::vector<float>> scores = {
    {0.95, 0.88},   // 图像0两个目标
    {0.90}          // 图像1一个目标
};

// 类别ID（按图像分别组织）
std::vector<std::vector<int>> class_ids = {
    {1, 3},         // 图像0的两个类别
    {0}             // 图像1的一个类别
};

// 保留项或 embedding（为空或某些模型扩展用）没用到
std::vector<std::vector<float>> embeddings = {
    {}, {} 
};
```

## 运行
```
ros2 run detector detector_node_exe
```

## 参数
| 参数名 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| `model_path_car` | string | /home/phoenix/Desktop/lwdetr_medium_coco_0729/inference_model.engine | 车辆模型路径 |
| `plugin_path_car` | string | /home/phoenix/roboarm/FairinoDualArm/src/detector/asserts/layernorm_plugin.so | 车辆模型插件路径 |
| `model_path_armor` | string | /home/phoenix/tensorrtx/yolov8/build/yolov8s.engine | 装甲板模型路径 |
| `plugin_path_armor` | string | /home/phoenix/tensorrtx/yolov8/build/layernorm_plugin.so | 装甲板模型插件路径 |
| `confidence_threshold` | float | 0.65 | 置信度阈值 |
| `nms_threshold` | float | 0.4 | NMS阈值 |
| `image_input_topic`| string | `/image_for_radar` | 输入图像话题 |
| `image_car_topic` | string | `/detector/img_car` | 车辆结果图像话题 |
| `image_detector_topic` | string | `/detector/img_detect` | 检测结果图像话题 |
| `detections_topic` | string | `/detector/detections` | 检测结果话题 |
| `show_pic_detector` | bool | true | 是否发布检测结果图像 |
| `show_pic_car` | bool | true | 是否发布车辆结果图像 |
| `show_pic_armor` | bool | true | 是否发布装甲板结果图像 |
| `rosbag_flag` | bool | true | 是否使用rosbag |
| `ROI_point_x` | int | 0 | ROI左上角x值 |
| `ROI_point_y` | int | 1150 | ROI左上角y值 |
| `num_ROI` | int | 6 | ROI数量 |
| `ROI_overlap` | int | 150 | ROI重叠的像素值 |
| `ROI_open` | bool | true | ROI是否打开 |

## 动态调参
可通过命令行调参
- 参数列表
```
ros2 param list /detector_node
```
- 获得当前参数值
```
ros2 param get /detector_node <param_name>
```
- 设置参数值
```
ros2 param set /detector_node <param_name> <param_value>
```


## 注意
yolov8推理的enqueue已废弃（deprecated） 未来可能会移除
若使用 enqueueV2 或 enqueueV3 则需重新导出模型
