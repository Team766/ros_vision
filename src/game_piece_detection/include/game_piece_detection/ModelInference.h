#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

// Logger for TensorRT info/warning/errors
class Logger : public nvinfer1::ILogger {
  void log(Severity severity, const char* msg) noexcept override {
    if (severity != Severity::kINFO) {
      std::cout << msg << std::endl;
    }
  }
} gLogger;

// Helper function to check CUDA errors
#define CHECK_CUDA(call)                                     \
  do {                                                       \
    cudaError_t status = call;                               \
    if (status != cudaSuccess) {                             \
      std::cerr << "CUDA error at line " << __LINE__ << ": " \
                << cudaGetErrorString(status) << std::endl;  \
      return false;                                          \
    }                                                        \
  } while (0)

class ModelInference {
 public:
  ModelInference(const std::string& enginePath)
      : m_engine_(nullptr), m_context_(nullptr) {
    initializeEngine(enginePath);
  }

  ~ModelInference() {
    for (void* buf : m_device_buffers_) {
      cudaFree(buf);
    }
    if (m_stream_) cudaStreamDestroy(m_stream_);
  }

  bool initializeEngine(const std::string& enginePath) {
    // Read engine file
    std::ifstream file(enginePath, std::ios::binary);
    if (!file.good()) {
      std::cerr << "Error opening engine file: " << enginePath << std::endl;
      return false;
    }

    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<char> engineData(size);
    file.read(engineData.data(), size);
    file.close();

    // Create runtime and engine
    m_runtime_ = std::unique_ptr<nvinfer1::IRuntime>(
        nvinfer1::createInferRuntime(gLogger));
    if (!m_runtime_) {
      std::cerr << "Error creating TensorRT runtime" << std::endl;
      return false;
    }

    m_engine_ = std::unique_ptr<nvinfer1::ICudaEngine>(
        m_runtime_->deserializeCudaEngine(engineData.data(), size));
    if (!m_engine_) {
      std::cerr << "Error deserializing CUDA engine" << std::endl;
      return false;
    }

    m_context_ = std::unique_ptr<nvinfer1::IExecutionContext>(
        m_engine_->createExecutionContext());
    if (!m_context_) {
      std::cerr << "Error creating execution context" << std::endl;
      return false;
    }

    // Create CUDA stream
    CHECK_CUDA(cudaStreamCreate(&m_stream_));

    // Allocate device buffers
    for (int i = 0; i < m_engine_->getNbIOTensors(); i++) {
      const char* tensor_name = m_engine_->getIOTensorName(i);
      nvinfer1::Dims dims = m_engine_->getTensorShape(tensor_name);
      size_t size = 1;
      for (int j = 0; j < dims.nbDims; j++) {
        size *= dims.d[j];
      }
      size *= sizeof(float);  // Assuming float32 data type

      void* deviceBuffer;
      CHECK_CUDA(cudaMalloc(&deviceBuffer, size));
      m_device_buffers_.push_back(deviceBuffer);

      if (m_engine_->getTensorIOMode(tensor_name) ==
          nvinfer1::TensorIOMode::kINPUT) {
        m_input_size_ = size;
        m_input_dims_ = dims;
      } else {
        m_output_size_ = size;
        m_output_dims_ = dims;
      }
    }

    return true;
  }

  bool infer(const float* input, float* output) {
    // Copy input to device
    CHECK_CUDA(cudaMemcpyAsync(m_device_buffers_[0], input, m_input_size_,
                               cudaMemcpyHostToDevice, m_stream_));

    // Execute inference
    for (int i = 0; i < m_engine_->getNbIOTensors(); i++) {
      const char* tensor_name = m_engine_->getIOTensorName(i);
      m_context_->setTensorAddress(tensor_name, m_device_buffers_[i]);
    }

    if (!m_context_->enqueueV3(m_stream_)) {
      std::cerr << "Error running inference: enqueueV3 failed!" << std::endl;
      return false;
    }

    // Copy output back to host
    CHECK_CUDA(cudaMemcpyAsync(output, m_device_buffers_[1], m_output_size_,
                               cudaMemcpyDeviceToHost, m_stream_));

    // Synchronize stream
    CHECK_CUDA(cudaStreamSynchronize(m_stream_));

    return true;
  }

  nvinfer1::Dims getInputDims() const { return m_input_dims_; }
  nvinfer1::Dims getOutputDims() const { return m_output_dims_; }
  size_t getInputSize() const { return m_input_size_; }
  size_t getOutputSize() const { return m_output_size_; }

 private:
  std::unique_ptr<nvinfer1::IRuntime> m_runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> m_engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> m_context_;
  cudaStream_t m_stream_;
  std::vector<void*> m_device_buffers_;
  size_t m_input_size_;
  size_t m_output_size_;
  nvinfer1::Dims m_input_dims_;
  nvinfer1::Dims m_output_dims_;
};
