#ifndef TFLITEEXECUTER_H
#define TFLITEEXECUTER_H

#include <cstddef>  // for size_t
#include <cstdint>
#include <string>
#include <vector>

#include <tensorflow/lite/c/c_api.h>
#include <tensorflow/lite/c/common.h>
#include <tensorflow/lite/delegates/xnnpack/xnnpack_delegate.h>

namespace htwk {

class TFLiteExecuter {
public:
    TFLiteExecuter() = default;
    ~TFLiteExecuter();
    
    TFLiteExecuter(TFLiteExecuter& h) = delete;
    TFLiteExecuter(TFLiteExecuter&& h) = delete;
    TFLiteExecuter& operator=(const TFLiteExecuter&) = delete;
    TFLiteExecuter& operator=(TFLiteExecuter&&) = delete;
    
    void loadModelFromFile(const std::string& file, std::vector<int> inputDims, int numThreads = 1, TfLiteType input_type = kTfLiteFloat32, TfLiteType output_type = kTfLiteFloat32);
    
    void loadModelFromArray(const void* modelData, size_t length, std::vector<int> inputDims, int numThreads = 1);
    
    float* getInputTensor();
    const float *getOutputTensor();

    template <typename T> T * getInputTensorTyped() {
      return (T*)(getInputTensor());
    }

    template <typename T> const T * getOutputTensorTyped() {
      return (const T*)(getOutputTensor());
    }
    
    size_t getElementsInputTensor();
    size_t getElementsOutputTensor();
    
    void execute();
    float* getResult();

    template <typename T> T * getResult() {
      return (T*)(getResult());
    }
    
    static std::string getTFliteModelPath();
    
private:
    TfLiteInterpreter* interpreter = nullptr;
    TfLiteDelegate* delegate = nullptr;
};

}

#endif // TFLITEEXECUTER_H
