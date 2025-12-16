#include <stdio.h>

#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;
int main(void) {
  std::ifstream f("/home/nvidia/code/OrinVisionSystem/cameracalibration/"
                  "calibrationmatrix.json");
  json data = json::parse(f);
  std::cout << "Matrix: " << std::endl;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      std::cout << data["matrix"][i][j] << " ";
    }
    std::cout << std::endl;
  }

  std::cout << "Distortion Coeffs:" << std::endl;
  for (int i = 0; i < 5; ++i) {
    std::cout << data["disto"][0][i] << ", ";
  }
  std::cout << std::endl;
  return 0;
}
