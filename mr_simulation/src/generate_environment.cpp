#include "./generate_environment.h"
#include <tf/tf.h> // tf::createQuaternionMsgFromRollPitchYaw
#include <map>

EnvironmentGenerator::EnvironmentGenerator(/* args */)
{
    item_number_ = 0;

    nh_ = std::make_shared<ros::NodeHandle>();

    std::string temp_generate_environment_topic_;
    ROS_ASSERT(nh_->getParam("world_axis", world_axis_));
    ROS_ASSERT(nh_->getParam("generate_environment_topic",   \
                    temp_generate_environment_topic_));

    marker_pub_ = std::make_unique<ros::Publisher>(std::move(    \
        nh_->advertise<visualization_msgs::MarkerArray>  \
        (temp_generate_environment_topic_, 1)));

    mark_ = std::make_unique<visualization_msgs::Marker>();
    mark_array_ = std::make_unique<visualization_msgs::MarkerArray>();

    mark_->header.frame_id = world_axis_;
    mark_->lifetime = ros::Duration();

    env_pole_ = std::make_unique<std::vector<std::string>>();
    env_wall_ = std::make_unique<std::vector<std::string>>();
    env_point_ = std::make_unique<std::vector<std::string>>();
}

void EnvironmentGenerator::addPole(const std::vector<double>& position,    \
                const std::vector<double>& orientation, \
                const std::vector<double>& scale)
{
    // grey
    std::vector<double>color{0.34,0.34,0.34,1};
    setMarker(position, orientation, scale, color,   \
                MARK_TYPE::CYLINDER, "pole");
}

void EnvironmentGenerator::addWall(const std::vector<double>& position,    \
                const std::vector<double>& orientation, \
                const std::vector<double>& scale)
{
    // white
    std::vector<double>color{1,1,1,0.5};
    setMarker(position, orientation, scale, color,   \
                MARK_TYPE::CUBE, "wall") ;
}

void EnvironmentGenerator::addPoint(const std::vector<double>& position,    \
                const std::vector<double>& orientation, \
                const std::vector<double>& scale)
{
    // green
    std::vector<double>color{0,1,0,1};
    setMarker(position, orientation, scale, color,   \
                MARK_TYPE::SPHERE, "point");
}

void EnvironmentGenerator::addLineStrip(const std::vector<double>& position)
{
    setTrajMarker(position, MARK_TYPE::POINT);
    setTrajMarker(position, MARK_TYPE::LINE_STRIP);
    generateEnvironment_();
    mark_array_->markers.clear();
}

void EnvironmentGenerator::clearRobotStepLineStrip()
{
    mark_->points.clear();
}


void EnvironmentGenerator::generateEnvironmentByYaml()
{

    std::vector<double> temp_position(3,0);
    std::vector<double> temp_orientation(3,0);
    std::vector<double> temp_scale(3,0); 

    std::string temp_env_pole_str = "environment_poles";
    bool env_pole_flag_ = nh_->getParam(    \
        temp_env_pole_str + "/name", *(env_pole_.get()));

    if(env_pole_flag_){
        for(size_t i = 0; i < env_pole_->size(); ++i)
        {
            ROS_WARN_STREAM("Generate " << i + 1 << " Pole.");
            processYamlFile( std::move(temp_env_pole_str + "/" +    \
                                env_pole_->at(i)), &temp_position,   \
                                &temp_orientation, &temp_scale);

            addPole(temp_position, temp_orientation, temp_scale);
        }
    }

    std::string temp_env_wall_str = "environment_walls";
    bool env_wall_flag_ =  nh_->getParam(    \
        temp_env_wall_str + "/name", *(env_wall_.get()));

    if(env_wall_flag_){
        for(size_t i = 0; i < env_wall_->size(); ++i)
        {
            ROS_WARN_STREAM("Generate " << i + 1 << " Wall.");
            processYamlFile( std::move(temp_env_wall_str + "/" +    \
                                env_wall_->at(i)), &temp_position,   \
                                &temp_orientation, &temp_scale);

            addWall(temp_position, temp_orientation, temp_scale);
        }
    }

    std::string temp_env_point_str = "environment_points";
    bool env_point_flag_ =  nh_->getParam(  \
        temp_env_point_str + "/name", *(env_point_.get()));

    if(env_point_flag_){
        for(size_t i = 0; i < env_point_->size(); ++i)
        {
            ROS_WARN_STREAM("Generate " << i + 1 << " Point.");
            processYamlFile( std::move(temp_env_point_str + "/" +   \
                                env_point_->at(i)), &temp_position,   \
                                &temp_orientation, &temp_scale);

            addPoint(temp_position, temp_orientation, temp_scale);
        }
    }

    generateEnvironment();
}

void EnvironmentGenerator::processYamlFile(const std::string& name,  \
                        std::vector<double>* position,  \
                        std::vector<double>* orientation,  \
                        std::vector<double>* scale  )
{

        std::map<std::string, double> temp_;
        ROS_ASSERT(nh_->getParam(name, temp_));

        position->at(0) = temp_["pos_x"];
        position->at(1) = temp_["pos_y"];
        position->at(2) = temp_["pos_z"];

        orientation->at(0) = temp_["ori_r"];
        orientation->at(1) = temp_["ori_p"];
        orientation->at(2) = temp_["ori_ry"];

        scale->at(0) = temp_["scale_x"];
        scale->at(1) = temp_["scale_y"];
        scale->at(2) = temp_["scale_z"];
}


void EnvironmentGenerator::generateEnvironment()
{   

    int hz = 5;
    ros::Rate rate(hz);
    int index = 0;
    // 发布5秒 （大于缓冲区大小）
    int timeout = 5 * hz;
    while(index < timeout){
        generateEnvironment_();
        rate.sleep();
        index ++;
    }
}

void EnvironmentGenerator::generateEnvironment_()
{
    marker_pub_->publish(*(mark_array_.get()));
    ros::spinOnce();
}

void EnvironmentGenerator::setMarker(   \
                        const std::vector<double>& position,    \
                        const std::vector<double>& orientation, \
                        const std::vector<double>& scale,   \
                        const std::vector<double>& color,   \
                        const enum MARK_TYPE& type, \
                        const std::string& name_space)
{
    // 设置添加时间
    mark_->header.stamp = ros::Time::now();
    // 设置命名空间
    mark_->ns = name_space;
    // 设置id
    mark_->id = item_number_ ++;
    // 设置类型
    switch (type)
    {
        case MARK_TYPE::CUBE:
            mark_->type = visualization_msgs::Marker::CUBE;
            break;
        
        case MARK_TYPE::CYLINDER:
            mark_->type = visualization_msgs::Marker::CYLINDER;
            break;

        case MARK_TYPE::SPHERE:
            mark_->type = visualization_msgs::Marker::SPHERE;
            break;

        case MARK_TYPE::POINT:
            mark_->type = visualization_msgs::Marker::POINTS;
            break;
        case MARK_TYPE::LINE_STRIP:
            mark_->type = visualization_msgs::Marker::LINE_STRIP;
            break;

        default:
            ROS_WARN("default marker type set(CUBE).");
            mark_->type = visualization_msgs::Marker::CUBE;
            break;
    }
    // 设置添加动作
    mark_->action = visualization_msgs::Marker::ADD;
    //设置位置
    mark_->pose.position.x = position[0];
    mark_->pose.position.y = position[1];
    mark_->pose.position.z = position[2];
    // 设置姿态
    // rpy --> xyzw
    mark_->pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(  \
                std::move(orientation[0] * DEG_TO_RAD_),    \
                std::move(orientation[1] * DEG_TO_RAD_),    \
                std::move(orientation[2] * DEG_TO_RAD_));
    // 设置物体大小
    mark_->scale.x = scale[0];
    mark_->scale.y = scale[1];
    mark_->scale.z = scale[2];
    // 设置颜色
    mark_->color.r = color[0];
    mark_->color.g = color[1];
    mark_->color.b = color[2];
    mark_->color.a = color[3];

    mark_array_->markers.push_back(*(mark_.get()));
}


void EnvironmentGenerator::setTrajMarker(const std::vector<double>& position, \
                    const enum MARK_TYPE& type)
{
    // 设置添加时间
    mark_->header.stamp = ros::Time::now();

    // 设置类型
    switch (type)
    {
        case MARK_TYPE::POINT:
            mark_->type = visualization_msgs::Marker::POINTS;
            break;
        case MARK_TYPE::LINE_STRIP:
            mark_->type = visualization_msgs::Marker::LINE_STRIP;
            break;

        default:
            ROS_WARN("default marker type set(POINTS).");
            mark_->type = visualization_msgs::Marker::POINTS;
            break;
    }
    // 设置添加动作
    mark_->action = visualization_msgs::Marker::ADD;
    //设置位置
    geometry_msgs::Point tmp_point;
    tmp_point.x = position[0];
    tmp_point.y = position[1];
    tmp_point.z = position[2];
    mark_->points.push_back(std::move(tmp_point));

    // 设置姿态
    // rpy --> xyzw
    mark_->pose.orientation.w = 1;

    // 设置物体大小
    if(type == MARK_TYPE::POINT){
        // 设置命名空间
        mark_->ns = "traj_point";
        mark_->scale.x = 0.02;
        mark_->scale.y = 0.02;
        mark_->scale.z = 0.02;
        // red
        mark_->color.r = 1;
        mark_->color.g = 0;
        mark_->color.b = 0;
        mark_->color.a = 1;
        mark_->id = item_number_ ++ ;
    }else if(type == MARK_TYPE::LINE_STRIP){
        mark_->ns = "traj_line";
        mark_->scale.x = 0.01;
        mark_->scale.y = 0.01;
        mark_->scale.z = 0.01;
        // blue
        mark_->color.r = 0;
        mark_->color.g = 0;
        mark_->color.b = 1;
        mark_->color.a = 1;
        mark_->id = item_number_ ++ ;
    }
    mark_array_->markers.push_back(*(mark_.get()));
}
