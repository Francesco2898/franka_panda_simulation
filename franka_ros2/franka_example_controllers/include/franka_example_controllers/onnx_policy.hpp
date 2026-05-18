#pragma once

#include <array>
#include <string>
#include <vector>
#include <stdexcept>
#include <sstream>
#include <onnxruntime_cxx_api.h>

namespace franka_example_controllers {

class OnnxPolicy {
public:
  static constexpr int OBS_DIM = 28;
  static constexpr int ACT_DIM = 6;

  explicit OnnxPolicy(const std::string& onnx_path,
                      int intra_op_threads = 1,
                      int inter_op_threads = 1)
  : env_(ORT_LOGGING_LEVEL_WARNING, "onnx_policy"),
    session_(nullptr),
    allocator_() 
  {
    Ort::SessionOptions so;
    so.SetIntraOpNumThreads(intra_op_threads);
    so.SetInterOpNumThreads(inter_op_threads);
    so.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    // CPU Execution Provider is default; no need to add explicitly for ORT CPU build.
    // If you later use GPU build, you can add CUDA provider here.

    session_ = Ort::Session(env_, onnx_path.c_str(), so);

    // Expect exactly 1 input + 1 output for your exported policy
    if (session_.GetInputCount() != 1 || session_.GetOutputCount() != 1) {
      std::ostringstream oss;
      oss << "ONNX model must have 1 input and 1 output. "
          << "Got inputs=" << session_.GetInputCount()
          << ", outputs=" << session_.GetOutputCount();
      throw std::runtime_error(oss.str());
    }

    // Cache input / output names
    {
      auto n = session_.GetInputNameAllocated(0, allocator_);
      input_name_ = std::string(n.get());
    }
    {
      auto n = session_.GetOutputNameAllocated(0, allocator_);
      output_name_ = std::string(n.get());
    }

    // Pre-set shapes: batch=1
    input_shape_  = {1, OBS_DIM};
    output_shape_ = {1, ACT_DIM};

    // Pre-allocate buffers to avoid per-step allocations
    input_buf_.resize(OBS_DIM, 0.0f);
  }

  const std::string& input_name() const { return input_name_; }
  const std::string& output_name() const { return output_name_; }

  // Inference with batch=1
  std::array<float, ACT_DIM> infer(const std::array<float, OBS_DIM>& obs_raw) {
    // Copy obs into contiguous buffer
    for (int i = 0; i < OBS_DIM; ++i) input_buf_[i] = obs_raw[i];

    Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(
      OrtArenaAllocator, OrtMemTypeDefault
    );

    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
      mem_info,
      input_buf_.data(),
      input_buf_.size(),
      input_shape_.data(),
      input_shape_.size()
    );

    const char* in_names[]  = { input_name_.c_str() };
    const char* out_names[] = { output_name_.c_str() };

    auto outputs = session_.Run(
      Ort::RunOptions{nullptr},
      in_names, &input_tensor, 1,
      out_names, 1
    );

    if (outputs.size() != 1) {
      throw std::runtime_error("ONNXRuntime Run() returned unexpected number of outputs.");
    }

    // Output is shape [1, 6]
    float* out = outputs[0].GetTensorMutableData<float>();

    std::array<float, ACT_DIM> act{};
    for (int i = 0; i < ACT_DIM; ++i) act[i] = out[i];
    return act;
  }

private:
  Ort::Env env_;
  Ort::Session session_;
  Ort::AllocatorWithDefaultOptions allocator_;

  std::string input_name_;
  std::string output_name_;

  std::vector<int64_t> input_shape_;
  std::vector<int64_t> output_shape_;

  std::vector<float> input_buf_;
};

}  // namespace franka_example_controllers