#include "detector/lw_detr.hpp"

#include <algorithm>
#include <cstring>

// 定义Logger
static class Logger : public nvinfer1::ILogger {
  void log(Severity severity, const char *msg) noexcept override {
    // 只显示非INFO级别的消息
    if (severity != Severity::kINFO) {
      std::cout << msg << std::endl;
    }
  }
} gLogger;

detector::LwDetr::LwDetr() {
  // 分配模型输入内存
  host_inputs = new float[batch_size * 3 * input_h * input_w];

  // 分配模型输出内存 LW-DETR输出包括检测框和类别标签两部分
  contexts_ = std::vector<InferContext>(NUM_THREAD);
  for (int i = 0; i < NUM_THREAD; i++) {
    contexts_[i].host_dets =
        new float[batch_size * num_boxes *
                  4]; // 检测框: [batch, num_boxes, 4]，每框包含[x1,y1,x2,y2]
    contexts_[i].host_labels =
        new float[batch_size * num_boxes *
                  num_classes]; // 标签: [batch, num_boxes, num_classes]
  }
}

detector::LwDetr::~LwDetr() {

  delete[] host_inputs;

  for (int i = 0; i < NUM_THREAD; i++) {
    delete[] contexts_[i].host_dets;
    delete[] contexts_[i].host_labels;

    if (contexts_[i].cuda_input) {
      cudaFree(contexts_[i].cuda_input);
    }
    if (contexts_[i].cuda_dets) {
      cudaFree(contexts_[i].cuda_dets);
    }
    if (contexts_[i].cuda_labels) {
      cudaFree(contexts_[i].cuda_labels);
    }

    if (contexts_[i].stream) {
      cudaStreamDestroy(contexts_[i].stream);
    }

    delete contexts_[i].context;
  }

  delete engine;
  delete runtime;
}

int detector::LwDetr::init(std::string engine_path, std::string plugin_path) {
  // 加载插件库
#ifndef PLUGIN_REGIST
#define PLUGIN_REGIST
  if (!plugin_path.empty()) {
    void *handle = dlopen(plugin_path.c_str(), RTLD_LAZY);
    if (!handle) {
      std::cerr << "错误: 加载插件库失败: " << dlerror() << std::endl;
      return -1;
    }
  }
#endif

  return init(engine_path);
}
int detector::LwDetr::init(std::string engine_path) {
  // 初始化NvInfer插件
  initLibNvInferPlugins(&gLogger, "");

  // 加载引擎文件
  std::ifstream engine_file(engine_path, std::ios::binary);
  if (!engine_file) {
    std::cerr << "错误: 无法打开引擎文件: " << engine_path << std::endl;
    return -1;
  }

  // 加载和反序列化引擎
  runtime = nvinfer1::createInferRuntime(gLogger);
  if (!runtime) {
    std::cerr << "错误: 创建TensorRT运行时时失败" << std::endl;
    return -1;
  }

  std::vector<char> engine_data((std::istreambuf_iterator<char>(engine_file)),
                                std::istreambuf_iterator<char>());
  engine = runtime->deserializeCudaEngine(engine_data.data(), engine_data.size());
  if (!engine) {
    std::cerr << "错误: 反序列化TensorRT引擎失败" << std::endl;
    return -1;
  }

  if (!resolveTensorNames()) {
    return -1;
  }

  auto checkCuda = [](cudaError_t status, const char *action) {
    if (status != cudaSuccess) {
      std::cerr << "CUDA错误(" << action << "): " << cudaGetErrorString(status) << std::endl;
      return false;
    }
    return true;
  };

  for (int i = 0; i < NUM_THREAD; i++) {
    contexts_[i].context = engine->createExecutionContext();
    if (!contexts_[i].context) {
      std::cerr << "错误: 创建执行上下文失败" << std::endl;
      return -1;
    }

    if (!checkCuda(cudaMalloc(reinterpret_cast<void **>(&contexts_[i].cuda_input),
                              batch_size * 3 * input_h * input_w * sizeof(float)),
                   "cudaMalloc(input)")) {
      return -1;
    }
    if (!checkCuda(cudaMalloc(reinterpret_cast<void **>(&contexts_[i].cuda_dets),
                              batch_size * num_boxes * 4 * sizeof(float)),
                   "cudaMalloc(dets)")) {
      return -1;
    }
    if (!checkCuda(cudaMalloc(reinterpret_cast<void **>(&contexts_[i].cuda_labels),
                              batch_size * num_boxes * num_classes * sizeof(float)),
                   "cudaMalloc(labels)")) {
      return -1;
    }

    if (!checkCuda(cudaStreamCreate(&contexts_[i].stream), "cudaStreamCreate")) {
      return -1;
    }

    bool address_ok = contexts_[i].context->setTensorAddress(input_tensor_name_.c_str(), contexts_[i].cuda_input) &&
                      contexts_[i].context->setTensorAddress(boxes_tensor_name_.c_str(), contexts_[i].cuda_dets) &&
                      contexts_[i].context->setTensorAddress(labels_tensor_name_.c_str(), contexts_[i].cuda_labels);
    if (!address_ok) {
      std::cerr << "错误: 绑定TensorRT张量地址失败" << std::endl;
      return -1;
    }
  }

  return 0;
}

bool detector::LwDetr::resolveTensorNames() {
  if (!engine) {
    std::cerr << "错误: 引擎尚未初始化，无法解析张量信息" << std::endl;
    return false;
  }

  const int32_t tensor_count = engine->getNbIOTensors();
  std::vector<std::string> input_names;
  std::vector<std::string> output_names;
  input_names.reserve(1);
  output_names.reserve(2);

  for (int32_t i = 0; i < tensor_count; ++i) {
    const char *tensor_name = engine->getIOTensorName(i);
    if (tensor_name == nullptr) {
      continue;
    }
    auto mode = engine->getTensorIOMode(tensor_name);
    if (mode == nvinfer1::TensorIOMode::kINPUT) {
      input_names.emplace_back(tensor_name);
    } else if (mode == nvinfer1::TensorIOMode::kOUTPUT) {
      output_names.emplace_back(tensor_name);
    }
  }

  if (input_names.size() != 1 || output_names.size() < 2) {
    std::cerr << "错误: 期望1个输入和2个输出张量，实际输入数量=" << input_names.size()
              << " 输出数量=" << output_names.size() << std::endl;
    return false;
  }

  input_tensor_name_ = input_names.front();

  for (const auto &name : output_names) {
    auto dims = engine->getTensorShape(name.c_str());
    int32_t last_dim = (dims.nbDims > 0) ? dims.d[dims.nbDims - 1] : -1;
    if (last_dim == 4 && boxes_tensor_name_.empty()) {
      boxes_tensor_name_ = name;
    } else if (last_dim == num_classes && labels_tensor_name_.empty()) {
      labels_tensor_name_ = name;
    }
  }

  if (boxes_tensor_name_.empty()) {
    boxes_tensor_name_ = output_names[0];
  }
  if (labels_tensor_name_.empty()) {
    labels_tensor_name_ = (output_names.size() > 1) ? output_names[1] : output_names[0];
  }

  return true;
}

std::tuple<std::vector<cv::Mat>, double, std::vector<std::vector<float>>,
           std::vector<std::vector<float>>, std::vector<std::vector<int>>,
           std::vector<std::vector<float>>>
detector::LwDetr::infer(std::vector<cv::Mat> &raw_image_generator,
                        int thread_id) {
  // 准备批处理输入
  std::vector<cv::Mat> batch_image_raw = {};
  std::vector<int> batch_origin_h = {};
  std::vector<int> batch_origin_w = {};
  std::vector<float> batch_input_image(batch_size * 3 * input_h * input_w);

  // 处理每张输入图像
  for (size_t i = 0;
       i < raw_image_generator.size() && i < batch_size && i < NUM_THREAD;
       ++i) {
    cv::Mat image_raw = raw_image_generator[i];
    int origin_h = image_raw.rows;
    int origin_w = image_raw.cols;

    // 图像预处理
    cv::Mat input_image = preprocess_image(image_raw);

    // 保存原始图像和尺寸信息
    batch_image_raw.push_back(image_raw);
    batch_origin_h.push_back(origin_h);
    batch_origin_w.push_back(origin_w);

    // 复制处理后的图像数据到批处理缓冲区
    std::memcpy(batch_input_image.data() + i * 3 * input_h * input_w,
                input_image.data, 3 * input_h * input_w * sizeof(float));
  }

  // 将输入数据传输到GPU
  cudaMemcpyAsync(contexts_[thread_id].cuda_input, batch_input_image.data(),
                  batch_size * 3 * input_h * input_w * sizeof(float),
                  cudaMemcpyHostToDevice, contexts_[thread_id].stream);

  // 设置输入张量的维度
  nvinfer1::Dims4 inputDims{batch_size, 3, input_h, input_w};
  if (!contexts_[thread_id].context->setInputShape(input_tensor_name_.c_str(), inputDims)) {
    std::cerr << "错误: 设置输入维度失败" << std::endl;
    return std::make_tuple(
        batch_image_raw, 0.0, std::vector<std::vector<float>>{},
        std::vector<std::vector<float>>{}, std::vector<std::vector<int>>{},
        std::vector<std::vector<float>>{});
  }

  // 检查维度是否有效
  if (!contexts_[thread_id].context->allInputDimensionsSpecified()) {
    std::cerr << "错误: 未能正确指定所有输入维度" << std::endl;
    return std::make_tuple(
        batch_image_raw, 0.0, std::vector<std::vector<float>>{},
        std::vector<std::vector<float>>{}, std::vector<std::vector<int>>{},
        std::vector<std::vector<float>>{});
  }

  // 执行推理
  auto start = std::chrono::high_resolution_clock::now();
  bool status = contexts_[thread_id].context->enqueueV3(contexts_[thread_id].stream);
  if (!status) {
    std::cerr << "错误: 推理执行失败" << std::endl;
  }

  // 将结果从GPU复制回主机
  cudaMemcpyAsync(contexts_[thread_id].host_dets,
                  contexts_[thread_id].cuda_dets,
                  batch_size * num_boxes * 4 * sizeof(float),
                  cudaMemcpyDeviceToHost, contexts_[thread_id].stream);

  cudaMemcpyAsync(contexts_[thread_id].host_labels,
                  contexts_[thread_id].cuda_labels,
                  batch_size * num_boxes * num_classes * sizeof(float),
                  cudaMemcpyDeviceToHost, contexts_[thread_id].stream);

  // 同步等待推理和数据传输完成
  cudaStreamSynchronize(contexts_[thread_id].stream);
  auto err = cudaGetLastError();
  if (err != cudaSuccess) {
    std::cerr << "CUDA 错误: " << cudaGetErrorString(err) << std::endl;
  }
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> duration = end - start;

  // 准备结果容器
  std::vector<std::vector<float>> result_boxes = {};
  std::vector<std::vector<float>> result_scores = {};
  std::vector<std::vector<int>> result_classID = {};
  std::vector<std::vector<float>> det = {};

  // 对每个批次图像进行后处理
  for (int i = 0; i < batch_size && i < raw_image_generator.size(); ++i) {
    float *dets_offset = contexts_[thread_id].host_dets + i * num_boxes * 4;
    float *labels_offset =
        contexts_[thread_id].host_labels + i * num_boxes * num_classes;

    // 后处理获取检测结果
    auto post_result =
        post_process(dets_offset, labels_offset, batch_origin_w[i],
                     batch_origin_h[i], num_boxes, num_classes);

    // 存储检测结果
    result_boxes.push_back(std::get<0>(post_result));
    result_scores.push_back(std::get<1>(post_result));
    result_classID.push_back(std::get<2>(post_result));
  }

  return std::make_tuple(batch_image_raw, duration.count(), result_boxes,
                         result_scores, result_classID, det);
}

cv::Mat detector::LwDetr::preprocess_image(const cv::Mat &img) {
  cv::Mat resized, float_img, rgb_img;
  cv::resize(img, resized, cv::Size(input_w, input_h));
  cv::cvtColor(resized, rgb_img, cv::COLOR_BGR2RGB); // BGR -> RGB

  rgb_img.convertTo(float_img, CV_32FC3, 1.0 / 255.0); // 转 float 并归一化

  // 拆分通道
  std::vector<cv::Mat> channels(3);
  cv::split(float_img, channels);

  // 归一化
  /*(mean/std)构建支持批处理的单一引擎模型。通过将batch_size设置为10，可以一次性处理10张图像。这种方案具有以下优势：

  显著减少内存占用
  最大化GPU利用率
  避免多线程调度开销
  2. 多CUDA流并行
    如果必须使用多线程方案，可以为每个线程创建独立的CUDA流：
        每个流拥有独立的命令队列
        减少流间的同步等待
        需要合理控制并发流数量以避免过度竞争
  3. 多配置文件引擎
  */
  // 基于coco预训练的mean和std
  const float mean[3] = {0.485, 0.456, 0.406};
  const float std[3] = {0.229, 0.224, 0.225};

  for (int i = 0; i < 3; ++i) {
    channels[i] = (channels[i] - mean[i]) / std[i];
  }

  // 合并成连续 CHW 格式
  std::vector<float> chw_data(3 * input_h * input_w);
  for (int i = 0; i < 3; ++i) {
    std::memcpy(chw_data.data() + i * input_h * input_w, channels[i].data,
                input_h * input_w * sizeof(float));
  }

  return cv::Mat(1, 3 * input_h * input_w, CV_32FC1, chw_data.data()).clone();
}

std::tuple<std::vector<float>, std::vector<float>, std::vector<int>>
detector::LwDetr::post_process(float *dets, float *labels, int origin_w,
                               int origin_h, int num_boxes, int num_classes) {
  std::vector<float> boxes;
  std::vector<float> scores;
  std::vector<int> classID;

  // 处理每个检测框
  for (int i = 0; i < num_boxes; i++) {
    // 获取检测框坐标 (DETR通常输出归一化的坐标[0,1])
    float x_center = dets[i * 4];
    float y_center = dets[i * 4 + 1];
    float width = dets[i * 4 + 2];
    float height = dets[i * 4 + 3];

    // 如果是归一化坐标，转换为实际像素坐标
    if (x_center <= 1 && y_center <= 1 && width <= 1 && height <= 1) {
      x_center *= input_w;
      y_center *= input_h;
      width *= input_w;
      height *= input_h;
    }

    // 转换为左上角和右下角坐标 (XYXY格式)
    float x1 = x_center - width / 2.0;
    float y1 = y_center - height / 2.0;
    float x2 = x_center + width / 2.0;
    float y2 = y_center + height / 2.0;

    // 将坐标映射回原始图像尺寸 - 直接采用等比例缩放
    // 预处理中使用 cv::resize 进行等比例缩放，这里采用相同方式进行逆变换
    float scale_w = (float)input_w / (float)origin_w;
    float scale_h = (float)input_h / (float)origin_h;

    // 计算实际缩放后的尺寸（与预处理一致）
    int scaled_w = origin_w * scale_w;
    int scaled_h = origin_h * scale_h;

    // 将坐标从模型输入尺寸映射回原始图像尺寸
    x1 = x1 / scale_w;
    y1 = y1 / scale_h;
    x2 = x2 / scale_w;
    y2 = y2 / scale_h;

    // 限制坐标在图像边界内
    x1 = std::max(0.0f, std::min(x1, (float)origin_w));
    y1 = std::max(0.0f, std::min(y1, (float)origin_h));
    x2 = std::max(0.0f, std::min(x2, (float)origin_w));
    y2 = std::max(0.0f, std::min(y2, (float)origin_h));

    // 找出最高置信度的类别
    int best_class = -1;
    float best_score = 0;
    for (int j = 0; j < num_classes; j++) {
      float score = sigmoid(labels[i * num_classes + j]);
      if (score > best_score && score > THRESHOLD) {
        best_score = score;
        best_class = j;
      }
    }

    // 如果有有效预测，添加到结果中
    if (best_class >= 0) {
      boxes.push_back(x1);
      boxes.push_back(y1);
      boxes.push_back(x2);
      boxes.push_back(y2);
      scores.push_back(best_score);
      classID.push_back(best_class);
    }
  }

  // 执行非极大值抑制
  // return non_max_suppression(boxes, scores, classID, NMS_THRESHOLD);
  return std::make_tuple(boxes, scores, classID);
}