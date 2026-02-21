#ifndef DOUBLEARRAYSENDER_H
#define DOUBLEARRAYSENDER_H

#include <string>
#include <vector>

#include <memory>
#include "networktables/NetworkTable.h"
#include "networktables/DoubleArrayTopic.h"

class DoubleArraySender {
 private:
  std::shared_ptr<nt::NetworkTable> table_;
  nt::DoubleArrayPublisher publisher_;

 public:
  // Constructor declaration
  DoubleArraySender(const std::string& key,
                    std::shared_ptr<nt::NetworkTable> table);

  // Method declarations
  void sendValue(std::vector<double> value);
  void setDefaultValue(std::vector<double> value);
};

#endif
