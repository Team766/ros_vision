#include "apriltags_cuda/AprilTagDataSender.h"

/*
Right now, the table is /SmartDashboard so we can visualize the values sent from
the Orin. Later, this should be changed to a new table, such as /Orin or
/Vision, to avoid populating the SmartDashboard with unnecessary values
*/

AprilTagDataSender::AprilTagDataSender(const std::string& key,
                                       const std::string& table_address,
                                       const std::string& table_name) {
  inst_ = nt::NetworkTableInstance::GetDefault();
  inst_.SetServer(table_address.c_str());
  inst_.StartClient4(table_address.c_str());
  auto table = inst_.GetTable(table_name);

  // Double Array Topic
  nt::DoubleArrayTopic da_topic = table->GetDoubleArrayTopic(key);
  publisher_ = da_topic.Publish();

  // Protobuf Topic
  nt::ProtobufTopic<com::team766::vision::ApriltagListProto> pb_topic =
      table_->GetProtobufTopic<com::team766::vision::ApriltagListProto>(key + "_protobuf");
  protobuf_publisher_ = pb_topic.Publish();
}

void AprilTagDataSender::sendValue(const std::vector<double>& value) {
  publisher_.Set(value);
  inst_.Flush();
}

void AprilTagDataSender::sendProtobuf(const com::team766::vision::ApriltagListProto& value) {
  protobuf_publisher_.Set(value);
}

void AprilTagDataSender::setDefaultValue(const std::vector<double>& value) {
  publisher_.SetDefault(value);
  inst_.Flush();
}

