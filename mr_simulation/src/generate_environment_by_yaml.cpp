#include "generate_environment.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "generate_environment_by_yaml");
    std::unique_ptr<EnvironmentGenerator> demo  \
        = std::make_unique<EnvironmentGenerator>();

    demo->generateEnvironmentByYaml();
    
    return 0;
}
