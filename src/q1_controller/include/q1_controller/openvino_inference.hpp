
// 文件: openvino_inference.hpp
#ifndef OPENVINO_INFERENCE_HPP
#define OPENVINO_INFERENCE_HPP

#include "inference_base.hpp"
#include <openvino/openvino.hpp>
#include "motor_manager.hpp" // 新增：Motor管理类

/**
 * @brief OpenVINO 后端的推理类，继承自 InferenceBase。
 *
 * 实现 OpenVINO 特定的模型加载和推理逻辑。
 * 假设使用 CPU 设备；可扩展到 GPU 等。
 */
class OpenVINOInference : public InferenceBase
{
  public:
    OpenVINOInference(const YAML::Node &config, std::shared_ptr<NetworkIOBase> network_io,std::shared_ptr<MotorManager> motor_manager);

  protected:
    void LoadModel(const std::string &model_path) override;
    std::map<std::string, Eigen::MatrixXf> Infer(const std::vector<Eigen::MatrixXf> &inputs) override;

  private:
    ov::Core core_;
    std::shared_ptr<ov::Model> model_;
    ov::CompiledModel compiled_model_;
    ov::InferRequest infer_request_;

    // 辅助函数：Eigen 到 OpenVINO Tensor 转换
    ov::Tensor EigenToOVTensor(const Eigen::MatrixXf &matrix, const ov::Shape &shape);
    Eigen::MatrixXf OVTensorToEigen(const ov::Tensor &tensor);
};

#endif // OPENVINO_INFERENCE_HPP