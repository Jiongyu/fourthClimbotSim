#include "./new_modular_robots_simulation.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "control_robot_by_conf_file");
    std::unique_ptr<NewModularRobotSimulation>demo =   \
        std::make_unique<NewModularRobotSimulation>();

    demo->controlRobotByConfigureFile("climbot6d_path_list.txt");
    return 0;
}
