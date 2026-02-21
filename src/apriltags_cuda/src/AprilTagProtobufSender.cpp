#include "apriltags_cuda/AprilTagProtobufSender.h"

#include "networktables/NetworkTable.h"

AprilTagProtobufSender::AprilTagProtobufSender(const std::string& key,
                                               std::shared_ptr<nt::NetworkTable> table)
    : table_(table) {
  nt::ProtobufTopic<com::team766::vision::ApriltagListProto> topic =
      table_->GetProtobufTopic<com::team766::vision::ApriltagListProto>(key + "_protobuf");
  publisher_ = topic.Publish();
}

void AprilTagProtobufSender::sendProtobuf(const com::team766::vision::ApriltagListProto& value) {
  publisher_.Set(value);
}
