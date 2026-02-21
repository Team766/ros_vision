#ifndef APRILTAG_PROTO_TRAITS_H
#define APRILTAG_PROTO_TRAITS_H

#include <string_view>
#include <span>
#include <cstdint>

#include "apriltag.pb.h"
#include "wpi/protobuf/Protobuf.h"

namespace wpi {

template <>
struct Protobuf<com::team766::vision::ApriltagProto> {
  using Message = google::protobuf::Message;
  static std::string_view GetTypeName() {
    return "com.team766.vision.ApriltagProto";
  }
  static google::protobuf::Descriptor const* GetDescriptor() {
    return com::team766::vision::ApriltagProto::descriptor();
  }
  static Message* New(google::protobuf::Arena* arena) {
    return google::protobuf::Arena::CreateMessage<com::team766::vision::ApriltagProto>(arena);
  }
  static void Pack(Message* msg,
                   const com::team766::vision::ApriltagProto& value) {
    if (auto* proto = static_cast<com::team766::vision::ApriltagProto*>(msg)) {
      *proto = value;
    }
  }
  static com::team766::vision::ApriltagProto Unpack(const Message& msg) {
    if (auto const* proto =
            static_cast<com::team766::vision::ApriltagProto const*>(&msg)) {
      return *proto;
    }
    return {};
  }
};

template <>
struct Protobuf<com::team766::vision::ApriltagListProto> {
  using Message = google::protobuf::Message;
  static std::string_view GetTypeName() {
    return "com.team766.vision.ApriltagListProto";
  }
  static google::protobuf::Descriptor const* GetDescriptor() {
    return com::team766::vision::ApriltagListProto::descriptor();
  }
  static Message* New(google::protobuf::Arena* arena) {
    return google::protobuf::Arena::CreateMessage<com::team766::vision::ApriltagListProto>(arena);
  }
  static void Pack(Message* msg,
                   const com::team766::vision::ApriltagListProto& value) {
    if (auto* proto = static_cast<com::team766::vision::ApriltagListProto*>(msg)) {
      *proto = value;
    }
  }
  static com::team766::vision::ApriltagListProto Unpack(const Message& msg) {
    if (auto const* proto =
            static_cast<com::team766::vision::ApriltagListProto const*>(&msg)) {
      return *proto;
    }
    return {};
  }
};

}  // namespace wpi

#endif // APRILTAG_PROTO_TRAITS_H
