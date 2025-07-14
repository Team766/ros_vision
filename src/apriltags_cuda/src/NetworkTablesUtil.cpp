#include "apriltags_cuda/NetworkTablesUtil.h"

#include "apriltags_cuda/NetworkTablesConfig.h"

NetworkTablesUtil::NetworkTablesUtil() {
  //   inst_ = nt::NetworkTableInstance::GetDefault();
  //   inst_.SetServer(TABLE_ADDRESS);
  //   inst_.StartClient4(TABLE_ADDRESS);
}

double NetworkTablesUtil::getTime() { return wpi::GetSystemTime(); }
