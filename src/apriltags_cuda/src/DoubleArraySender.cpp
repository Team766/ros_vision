#include "apriltags_cuda/DoubleArraySender.h"

/*
Right now, the table is /SmartDashboard so we can visualize the values sent from
the Orin. Later, this should be changed to a new table, such as /Orin or
/Vision, to avoid populating the SmartDashboard with unnecessary values
*/

DoubleArraySender::DoubleArraySender(const std::string& key,
                                     const std::string& table_address,
                                     const std::string& table_name) {
  inst_ = nt::NetworkTableInstance::GetDefault();
  inst_.SetServer(table_address.c_str());
  inst_.StartClient4(table_address.c_str());
  auto table = inst_.GetTable(table_name);
  nt::DoubleArrayTopic topic = table->GetDoubleArrayTopic(key);
  publisher_ = topic.Publish();
}
void DoubleArraySender::sendValue(std::vector<double> value) {
  publisher_.Set(value);
  inst_.Flush();
}
void DoubleArraySender::setDefaultValue(std::vector<double> value) {
  publisher_.SetDefault(value);
  inst_.Flush();
}
