#ifndef DOUBLEARRAYSUBSCRIBER_H
#define DOUBLEARRAYSUBSCRIBER_H

#include <string>
#include <vector>
#include <functional>

#include "networktables/DoubleArrayTopic.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

class DoubleArraySubscriber {
private:
  nt::NetworkTableInstance inst_;
  nt::DoubleArraySubscriber subscriber_;

public:
  // Constructor declaration
  DoubleArraySubscriber(std::string key, std::string server_address = "localhost");

  // Method declarations
  std::vector<double> getValue() const;
  void setValueCallback(std::function<void(std::vector<double>)> callback);
};

#endif

