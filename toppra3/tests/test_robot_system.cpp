#include "toppra/robot_model.hpp"

int main() {
  RobotSystem robot_model("calibrated_arm.xml");
  std::cout << robot_model.getJointName(0) << std::endl;
  return 0;
}
