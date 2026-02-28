#include <gtest/gtest.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/ProtobufTopic.h>
#include "apriltags_cuda/AprilTagDataSender.h"
#include "apriltags_cuda/apriltag_proto_traits.h"

/**
 * @brief Test fixture for AprilTagDataSender tests.
 * Uses an isolated NetworkTableInstance to avoid interference.
 */
class AprilTagSenderTest : public ::testing::Test {
protected:
  void SetUp() override {
    inst_ = nt::NetworkTableInstance::Create();
    // Start a local server/client for the isolated instance
    inst_.StartClient4("test_client");
    
    sender_ = std::make_unique<AprilTagDataSender>(
        "test_key", "127.0.0.1", "test_table", &inst_);
  }

  void TearDown() override {
    sender_.reset();
    inst_.StopClient4();
  }

  nt::NetworkTableInstance inst_;
  std::unique_ptr<AprilTagDataSender> sender_;
};

/**
 * @brief Test that DoubleArray values are sent correctly.
 */
TEST_F(AprilTagSenderTest, SendDoubleArray) {
  std::vector<double> test_data = {1.0, 2.0, 3.0, 4.2};
  sender_->sendValue(test_data);
  
  // Wait a small amount for NT synchronization in memory
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  
  auto table = inst_.GetTable("test_table");
  auto entry = table->GetEntry("test_key");
  auto result = entry.GetDoubleArray({});
  
  ASSERT_EQ(result.size(), test_data.size());
  for (size_t i = 0; i < test_data.size(); ++i) {
    EXPECT_DOUBLE_EQ(result[i], test_data[i]);
  }
}

/**
 * @brief Test the Protobuf Traits Pack/Unpack logic directly.
 * This verifies our static_cast and assignment logic in apriltag_proto_traits.h
 */
TEST(AprilTagProtoTraitsTest, PackUnpackConsistency) {
  com::team766::vision::ApriltagProto original;
  original.set_id(42);
  original.set_pose_error(0.01);
  original.set_distance(5.5);
  
  // Pack using the trait
  // Note: Local tests of the trait can use the traits directly
  google::protobuf::Arena arena;
  google::protobuf::Message* msg = wpi::Protobuf<com::team766::vision::ApriltagProto>::New(&arena);
  
  wpi::Protobuf<com::team766::vision::ApriltagProto>::Pack(msg, original);
  
  // Unpack using the trait
  com::team766::vision::ApriltagProto unpacked = 
      wpi::Protobuf<com::team766::vision::ApriltagProto>::Unpack(*msg);
      
  EXPECT_EQ(unpacked.id(), original.id());
  EXPECT_DOUBLE_EQ(unpacked.pose_error(), original.pose_error());
  EXPECT_DOUBLE_EQ(unpacked.distance(), original.distance());
}

/**
 * @brief Test that Protobuf messages are sent correctly via NetworkTables.
 */
TEST_F(AprilTagSenderTest, SendProtobuf) {
  com::team766::vision::ApriltagListProto list;
  auto* tag = list.add_tags();
  tag->set_id(7);
  tag->set_distance(1.23);
  
  sender_->sendProtobuf(list);
  
  // Wait for NT sync
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  
  auto table = inst_.GetTable("test_table");
  auto subscriber = table->GetProtobufTopic<com::team766::vision::ApriltagListProto>("test_key_protobuf")
                        .Subscribe(com::team766::vision::ApriltagListProto{});
  
  auto result = subscriber.Get();
  
  ASSERT_EQ(result.tags_size(), 1);
  EXPECT_EQ(result.tags(0).id(), 7);
  EXPECT_DOUBLE_EQ(result.tags(0).distance(), 1.23);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
