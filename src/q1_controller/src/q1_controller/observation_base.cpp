// 文件: observation_base.cpp
#include "../../include/q1_controller/observation_base.hpp"

ObservationComponent::ObservationComponent(int dim, const std::string &name, float scale,
                                           std::shared_ptr<DataStore> data_store)
    : dim_(dim), name_(name), scale_(scale), data_store_(data_store)
{
    std::cout << name_ << ": dim:" << dim_ << " scale:" << scale_ << std::endl;
}