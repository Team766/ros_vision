#ifndef APRILTAG_PROTOBUF_SENDER_H
#define APRILTAG_PROTOBUF_SENDER_H

#include <string>

#include <memory>
#include "apriltags_cuda/apriltag_proto_traits.h"
#include "networktables/NetworkTable.h"
#include "networktables/ProtobufTopic.h"

class AprilTagProtobufSender {
 public:
  AprilTagProtobufSender(const std::string& key,
                         std::shared_ptr<nt::NetworkTable> table);

  void sendProtobuf(const com::team766::vision::ApriltagListProto& value);

 private:
  std::shared_ptr<nt::NetworkTable> table_;
  nt::ProtobufPublisher<com::team766::vision::ApriltagListProto> publisher_;
};

#endif // APRILTAG_PROTOBUF_SENDER_H
