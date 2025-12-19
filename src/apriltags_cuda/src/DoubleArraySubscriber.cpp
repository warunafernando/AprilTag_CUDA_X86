#include "DoubleArraySubscriber.h"

#include "NetworkTablesConfig.h"

DoubleArraySubscriber::DoubleArraySubscriber(std::string key, std::string server_address) {
  inst_ = nt::NetworkTableInstance::GetDefault();
  if (server_address == "") {
    server_address = TABLE_ADDRESS;
  }
  inst_.SetServer(server_address.c_str());
  inst_.StartClient4("camera_gui");
  auto table = inst_.GetTable(TABLE_NAME);
  nt::DoubleArrayTopic topic = table->GetDoubleArrayTopic(key);
  subscriber_ = topic.Subscribe({});
}

std::vector<double> DoubleArraySubscriber::getValue() const {
  return subscriber_.Get();
}

void DoubleArraySubscriber::setValueCallback(std::function<void(std::vector<double>)> callback) {
  subscriber_.SetCallback([callback](const nt::ValueEvent& event) {
    if (event.value && event.value->IsDoubleArray()) {
      callback(event.value->GetDoubleArray());
    }
  });
}

