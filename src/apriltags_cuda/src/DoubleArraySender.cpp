#include "apriltags_cuda/DoubleArraySender.h"

/*
Right now, the table is /SmartDashboard so we can visualize the values sent from
the Orin. Later, this should be changed to a new table, such as /Orin or
/Vision, to avoid populating the SmartDashboard with unnecessary values
*/

DoubleArraySender::DoubleArraySender(const std::string& key,
                                     std::shared_ptr<nt::NetworkTable> table)
    : table_(table) {
  nt::DoubleArrayTopic topic = table_->GetDoubleArrayTopic(key);
  publisher_ = topic.Publish();
}

void DoubleArraySender::sendValue(std::vector<double> value) {
  publisher_.Set(value);
}

void DoubleArraySender::setDefaultValue(std::vector<double> value) {
  publisher_.SetDefault(value);
  inst_.Flush();
}
