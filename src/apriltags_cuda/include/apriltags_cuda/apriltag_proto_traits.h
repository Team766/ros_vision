#ifndef APRILTAG_PROTO_TRAITS_H
#define APRILTAG_PROTO_TRAITS_H

#include <string_view>
#include <span>
#include <cstdint>

#include "apriltag.pb.h"
#include "wpi/Protobuf.h"

namespace wpi {

template <>
struct Protobuf<com::team766::vision::ApriltagProto> {
  static std::string_view GetTypeName() {
    return "com.team766.vision.ApriltagProto";
  }
  static google::protobuf::Descriptor const* GetDescriptor() {
    return com::team766::vision::ApriltagProto::descriptor();
  }
  static com::team766::vision::ApriltagProto New(google::protobuf::Arena* arena) {
    if (arena) {
      return *google::protobuf::Arena::CreateMessage<com::team766::vision::ApriltagProto>(arena);
    }
    return com::team766::vision::ApriltagProto{};
  }
  static void Pack(std::span<uint8_t> out,
                   const com::team766::vision::ApriltagProto& msg) {
    msg.SerializeToArray(out.data(), out.size());
  }
  static bool Unpack(com::team766::vision::ApriltagProto* out,
                     std::span<const uint8_t> in) {
    return out->ParseFromArray(in.data(), in.size());
  }
};

template <>
struct Protobuf<com::team766::vision::ApriltagListProto> {
  static std::string_view GetTypeName() {
    return "com.team766.vision.ApriltagListProto";
  }
  static google::protobuf::Descriptor const* GetDescriptor() {
    return com::team766::vision::ApriltagListProto::descriptor();
  }
  static com::team766::vision::ApriltagListProto New(google::protobuf::Arena* arena) {
    if (arena) {
      return *google::protobuf::Arena::CreateMessage<com::team766::vision::ApriltagListProto>(arena);
    }
    return com::team766::vision::ApriltagListProto{};
  }
  static void Pack(std::span<uint8_t> out,
                   const com::team766::vision::ApriltagListProto& msg) {
    msg.SerializeToArray(out.data(), out.size());
  }
  static bool Unpack(com::team766::vision::ApriltagListProto* out,
                     std::span<const uint8_t> in) {
    return out->ParseFromArray(in.data(), in.size());
  }
};

}  // namespace wpi

#endif // APRILTAG_PROTO_TRAITS_H
