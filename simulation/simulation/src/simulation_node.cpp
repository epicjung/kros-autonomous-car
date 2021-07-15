
#include "simulation/simulation.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simulation_node");
  Simulation node;
  node.run();
  return 0;
}
