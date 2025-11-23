cmake_minimum_required(VERSION 3.8)
project(detector_wjr)

# 设置 C++ 标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-O2)

# 查找必要的包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)
find_package(rosidl_default_generators REQUIRED)



# 设置 TensorRT 相关路径
set(TENSORRT_DIR /home/ma/Downloads/TensorRT-8.6.1.6)
set(TENSORRT_PLUGIN_LIB "" CACHE FILEPATH "Path to a custom TensorRT plugin library (optional)")

if(NOT TENSORRT_PLUGIN_LIB AND DEFINED ENV{TENSORRT_PLUGIN_LIB})
  set(TENSORRT_PLUGIN_LIB $ENV{TENSORRT_PLUGIN_LIB})
endif()

# 添加消息定义 - 将消息生成目标改为${PROJECT_NAME}，与代码中使用的命名空间一致
rosidl_generate_interfaces(${PROJECT_NAME}
  "/home/ma/roboarm/aicontest/src/detector:msg/Bbox2d.msg"
  "/home/ma/roboarm/aicontest/src/detector:msg/Bbox2dArray.msg"
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

# 设置源文件
set(DETECTOR_SOURCES 
  src/detector_node.cpp
  src/yolov8.cpp
  src/lw_detr.cpp
)

# 生成接口
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# 创建组件库 - 使用独立的库名称
add_library(detector_wjr_lib SHARED ${DETECTOR_SOURCES})

# 包含目录
target_include_directories(detector_wjr_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CUDA_INCLUDE_DIRS}
  ${TENSORRT_DIR}/include
)

# 设置依赖项
ament_target_dependencies(detector_wjr_lib
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  OpenCV
  rosidl_default_runtime
)

# 链接外部库和消息接口
set(detector_link_libraries
  ${OpenCV_LIBS}
  ${CUDA_LIBRARIES}
  nvinfer
  nvinfer_plugin
  nvonnxparser
  ${CUDA_CUDART_LIBRARY}
  dl
  "${cpp_typesupport_target}"
)

if(TENSORRT_PLUGIN_LIB)
  if(EXISTS "${TENSORRT_PLUGIN_LIB}")
    list(APPEND detector_link_libraries "${TENSORRT_PLUGIN_LIB}")
  else()
    message(WARNING "Requested TensorRT plugin library '${TENSORRT_PLUGIN_LIB}' was not found; continuing without it")
  endif()
endif()

target_link_libraries(detector_wjr_lib ${detector_link_libraries})

# 注册组件
rclcpp_components_register_node(detector_wjr_lib
  PLUGIN "detector_wjr::detector_node_wjr"
  EXECUTABLE detector_node_exe
)

# 重命名库以保持向后兼容
set_target_properties(detector_wjr_lib
  PROPERTIES OUTPUT_NAME detector
)

# 安装规则
install(TARGETS detector_wjr_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# 导出依赖项
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  OpenCV
  rosidl_default_runtime
)

ament_export_include_directories(include)
ament_export_libraries(detector_wjr_lib)

ament_package()