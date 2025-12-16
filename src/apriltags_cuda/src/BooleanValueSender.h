#ifndef BOOLEANVALUESENDER_H
#define BOOLEANVALUESENDER_H

#include <string>

#include "networktables/BooleanTopic.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

class BooleanValueSender {
private:
  nt::NetworkTableInstance inst_;
  nt::BooleanPublisher publisher_;

public:
  // Constructor declaration
  BooleanValueSender(std::string key);

  // Method declarations
  void sendValue(bool value);
  void setDefaultValue(bool value);
};

#endif