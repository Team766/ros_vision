#ifndef DOUBLEARRAYSENDER_H
#define DOUBLEARRAYSENDER_H

#include <string>
#include <vector>

#include "apriltag.pb.h"

#include "networktables/DoubleArrayTopic.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/ProtobufTopic.h"

class DoubleArraySender {
 private:
  nt::NetworkTableInstance inst_;
  nt::DoubleArrayPublisher publisher_;
  nt::ProtobufPublisher<com::team766::vision::ApriltagListProto> protobuf_publisher_;

  std::string table_name_;
  std::string table_address_;

 public:
  // Constructor declaration
  DoubleArraySender(const std::string& key, const std::string& table_address,
                    const std::string& table_name);

  // Method declarations
  void sendValue(std::vector<double> value);
  void setDefaultValue(std::vector<double> value);
  void sendProtobuf(const com::team766::vision::ApriltagListProto& value);

 private:
};

#endif