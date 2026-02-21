#ifndef APRILTAG_DATA_SENDER_H
#define APRILTAG_DATA_SENDER_H

#include <string>
#include <vector>
#include <memory>

#include "networktables/DoubleArrayTopic.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/ProtobufTopic.h"
#include "apriltags_cuda/apriltag_proto_traits.h"

class AprilTagDataSender {
 private:
  nt::NetworkTableInstance inst_;
  nt::DoubleArrayPublisher publisher_;
  nt::ProtobufPublisher<com::team766::vision::ApriltagListProto> protobuf_publisher_;

 public:
  AprilTagDataSender(const std::string& key,
                     const std::string& table_address,
                     const std::string& table_name);

  void sendValue(const std::vector<double>& value);
  void sendProtobuf(const com::team766::vision::ApriltagListProto& value);
  void setDefaultValue(const std::vector<double>& value);
};

#endif // APRILTAG_DATA_SENDER_H
