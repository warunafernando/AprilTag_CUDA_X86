#ifndef INTEGERVALUESENDER_H
#define INTEGERVALUESENDER_H

#include <string>

#include "networktables/IntegerTopic.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

class IntegerValueSender {
private:
  nt::NetworkTableInstance inst_;
  nt::IntegerPublisher publisher_;

public:
  // Constructor declaration
  IntegerValueSender(std::string key);

  // Method declarations
  void sendValue(int value);
  void setDefaultValue(int value);
};

#endif