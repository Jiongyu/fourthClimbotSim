#include "./new_modular_robots_simulation.h"
#include <ros/package.h>  // ros::package::getPath 
#include <fstream> // std::ifstream

namespace{

/**
 * @brief 替换字符串目标字符
 * 
 * @param str 字符串
 * @param src 被替换字符
 * @param dec 替换字符
 */
void replaceStr(std::string *str, const char& src, const char& dec)
{
    if(str->empty()) return;
    size_t temp = 0; 
    while (temp != std::string::npos)
    {
        temp = str->find(src, temp);
        if(temp == std::string::npos){
            break;
        }
        str->at(temp) = dec;
    }
}

/**
 * @brief 根据特定字符，分割获取字符串右侧
 * 
 * @param str 字符串
 * @param src 特定字符
 */
void splitStrGetRight(std::string *str, const char& src)
{
    if(str->empty()) return;
    size_t temp = 0;
    temp = str->find(src);
    if(temp != std::string::npos){
        *str = (*str).substr(temp + 1, str->size() - 1); 
    }
}

/**
 * @brief 分割空格字符， 提取字符串为 vector
 * 
 * @param result vector
 * @param str 字符串
 */
void splitStrGetResult(std::vector<double>& result, const std::string& str)
{
    if(str.empty()) return;
    int index = 0;
    int i = 0;
    while(index < str.size())
    {
        if(str[index] == ' '){
            index ++;
            continue;
        }

        int left = index;
        
        while(str[index] != ' ')
        {
            index ++;
        }   

        int right = index;

        if(str[index] == ' '){
            result.push_back( std::stod( str.substr(left, right)) );
            i++;
        }
        index ++;
    }
}

/**
 * @brief  解析路径文件，获取路径数据
 * 
 * @param path_data 
 * @return int 0:success  -1:error
 */
int readConfigureFile(std::vector<std::vector<double>> & path_data,     \
    const std::string &name)
{

    // const std::string path_file_ =   \
            // ros::package::getPath("mr_simulation") + "/config/path_list.txt";
    const std::string path_file_ =   \
            ros::package::getPath("mr_simulation") + "/config/" + name;
            
    std::ifstream infile;
    infile.open(path_file_, std::ios::in);

    if(! infile.is_open()){
        ROS_ERROR("Can not open \"path_list.txt\"! ");
        return -1;
    }

    std::string temp_str;
    try{
        while (getline(infile, temp_str))
        {
            if(! temp_str.empty()){
                
                std::vector<double> temp_path_data;

                if(temp_str.at(0) == 'P'){
                    splitStrGetRight(&temp_str, '=');
                    replaceStr(&temp_str, ',',' ');
                    replaceStr(&temp_str, ';',' ');
                    splitStrGetResult(temp_path_data, temp_str);
                    path_data.push_back(std::move(temp_path_data));
                    // std::cout << temp_path_data.size() << std::endl;

                }else if(temp_str == "HALT"){
                    // size == 0
                    temp_path_data.clear();
                    path_data.push_back(temp_path_data);
                }
            }
        }
    }catch(std::exception e){

        ROS_ERROR_STREAM("path data parse error: " << e.what());
        return -1;
    }
    return 0;
}

} // end namespace 


sensor_msgs::JointState NewModularRobotSimulation::joint_state_msg_ =  \
    sensor_msgs::JointState();

std::atomic_bool NewModularRobotSimulation::if_get_joint_state_msg_(false);

std::unique_ptr<std::vector<int>> NewModularRobotSimulation::joint_pos_direction_ = \
    std::make_unique<std::vector<int>>();

std::unique_ptr<std::vector<int>> NewModularRobotSimulation::joint_neg_direction_ = \
    std::make_unique<std::vector<int>>();

NewModularRobotSimulation::NewModularRobotSimulation(/* args */)
{
    tf_broadcaster_ = std::make_unique<tf::TransformBroadcaster>();
    tf_listener_ = std::make_unique<tf::TransformListener>();

    // publisher init
    std::string temp_topic_;
    int temp_topic_size_;
    ROS_ASSERT(nh_.getParam("joint_commands_topic",temp_topic_));
    ROS_ASSERT(nh_.getParam("joint_commands_topic_queue_size",  \
        temp_topic_size_));
    pub_joint_command_ = std::make_unique<ros::Publisher>(std::move(    \
        nh_.advertise<sensor_msgs::JointState>(temp_topic_, temp_topic_size_)));

    // subscriber init
    ROS_ASSERT(nh_.getParam("joint_state_topic",temp_topic_));
    ROS_ASSERT(nh_.getParam("joint_state_topic_queue_size",  \
        temp_topic_size_));
    sub_joint_state_ = std::make_unique<ros::Subscriber>(std::move( \
        nh_.subscribe<sensor_msgs::JointState>(temp_topic_,     \
        temp_topic_size_,subRobotStateCallback)));

    joint_name_ = std::make_shared<std::vector<std::string>>();
    ROS_ASSERT(nh_.getParam("joint_names" ,*(joint_name_.get())));
    ROS_ASSERT(joint_name_->size() != 0);

    joint_command_msg_ = std::make_unique<sensor_msgs::JointState>();
    joint_command_msg_->name = *(joint_name_.get());
    joint_command_msg_->position.resize(joint_name_->size());

    ROS_ASSERT(nh_.getParam("max_joint_velocity",max_joint_velocity_));
    max_joint_velocity_ = max_joint_velocity_ * DEG_TO_RAD_;
    ROS_ASSERT(nh_.getParam("world_axis", world_axis_));
    ROS_ASSERT(nh_.getParam("g0_base", g0_base_));
    ROS_ASSERT(nh_.getParam("g6_base", g6_base_));
    ROS_ASSERT(nh_.getParam("positive_tcp", positive_tcp_));
    ROS_ASSERT(nh_.getParam("negetive_tcp", negetive_tcp_));
    ROS_ASSERT(nh_.getParam("joint_pos_direction", *(joint_pos_direction_.get())));
    ROS_ASSERT(nh_.getParam("joint_neg_direction", *(joint_neg_direction_.get())));

    ROS_ASSERT(nh_.getParam("if_display_robot_tcp_trajectory",  \
        if_generate_display_tcp_traj_));

    if(if_generate_display_tcp_traj_){
        environment_generator_ = std::make_unique<EnvironmentGenerator>();
    }

    traj_rate_hz_ = 30;
    
    // 初始化夹持器状态
    which_base_ = GRIPPER_ID::G0;
    g0_state_ = GRIPPER_ACTION::CLOSE;
    g6_state_ = GRIPPER_ACTION::CLOSE;

    // -2 夹持器数量
    joint_number_ = joint_name_->size() / 2 - 2;

    getBasePos();

}

int NewModularRobotSimulation::sentJointCommands(  \
    std::vector<std::vector<double> >& path_list)
{
    for(size_t i = 0; i < path_list.size(); ++i)
    {
        ROS_ASSERT(path_list[i].size() == joint_number_);
        // 修改机器人旋转方向
        for(size_t j = 0; j < path_list[i].size(); ++ j)
        {
            if(GRIPPER_ID::G0 == which_base_){
                path_list[i][j] = path_list[i][j] * joint_pos_direction_->at(j);
            }else{
                path_list[i][j] = path_list[i][j] * joint_neg_direction_->at(j);
            }
        }
    }

    // 根据夹持器基座， 切换关节数据
    changeJointPathByBase(path_list);

    return sentJointCommands_(std::move(getPathPlanResult(path_list)));
}

int NewModularRobotSimulation::gripperControl(const enum  GRIPPER_ID& id,    \
        const enum  GRIPPER_ACTION& action)
{
    
    // 使能机器人运动
    int ret = enableGripperPlan(id, action);

    // 更新夹持器基座
    // changeBaseGripper();

    return ret;                              
}

int NewModularRobotSimulation::controlRobotByConfigureFile( \
    const std::string &name)
{
    
    std::vector<std::vector<double>>path_data;
    readConfigureFile(path_data, name);
    // std::cout << path_data.size() << std::endl;
    int i = 0;
    int left = i;
    int right = i;
    int step = 1;
    std::vector<std::vector<double>>temp_path_data;
    for(; i < path_data.size(); ++i){
        if(path_data[i].size() ==  joint_number_)
        {
            temp_path_data.push_back(std::move(path_data[i]));
            right ++;
        }else if(path_data[i].size() == 0)
        {
            ROS_WARN_STREAM("The " << step << " step!");
            climbotStepMove(temp_path_data);
            temp_path_data.clear();
            left = i + 1;
            right = left;
            step ++;
        }
    }
}

void NewModularRobotSimulation::changeJointPathByBase(  \
    std::vector<std::vector<double> >& path_list)
{
    if(which_base_ == GRIPPER_ID::G6){
        // std::cout << "G6" << std::endl;
        for(size_t i = 0; i < path_list.size(); ++i)
        {
            for(size_t j = 0; j < path_list[i].size() / 2; ++ j)
            {
                double temp = path_list[i][j];
                path_list[i][j] = path_list[i][path_list[i].size() - j - 1];
                path_list[i][path_list[i].size() - j - 1] = temp;
    
            }
            // 五自由度三关节 交换
            if(path_list[i].size() % 2 != 0){
                path_list[i][path_list[i].size() / 2] =     \
                    - path_list[i][path_list[i].size() / 2];
            }
        }
    }
}

int NewModularRobotSimulation::changeRobotJoint()
{
    while(getRobotCurrentPosition()){};
    if(which_base_ == GRIPPER_ID::G0){
        // 改变以g6为基座的机器人位置
        // - 1 虚拟夹持器
        int j = joint_state_msg_.position.size() / 2 - 2;
        for(size_t i = joint_command_msg_->name.size() / 2;     \
            i < joint_command_msg_->name.size(); ++i)
        {
            // g0 g7
            if( i ==  joint_command_msg_->name.size() -1 ||  \
                    i ==  joint_command_msg_->name.size() / 2){
                joint_command_msg_->position[i] = joint_state_msg_.position[j--];

            }else{
                joint_command_msg_->position[i] = - joint_state_msg_.position[j--];
            }
        }

    }else{
        // 改变以g0为基座的机器人位置
        // - 1 虚拟夹持器
        int j = joint_state_msg_.position.size() - 2;
        for(size_t i = 0; i < joint_command_msg_->name.size() / 2; i++)
        {
            // g0 g7
            if(0 == i || i == joint_command_msg_->name.size() / 2 - 1){
                joint_command_msg_->position[i] = joint_state_msg_.position[j--];

            }else{
                joint_command_msg_->position[i] = - joint_state_msg_.position[j--];
            }
        }
    }

    pub_joint_command_->publish(*(joint_command_msg_.get()));
    ros::spinOnce();
    return 0;
}

std::vector<sensor_msgs::JointState>   \
    NewModularRobotSimulation::getPathPlanResult(  \
    const std::vector<std::vector<double>>& path_list) const
{

    std::vector<sensor_msgs::JointState> traj_list;
    traj_list.clear();
    
    for(size_t i = 0; i < path_list.size() - 1; ++i)
    {
        // 1. 获取机器人关节位移,计算最大绝对位移
        // std::cout << "\n1. 获取机器人关节位移,计算最大绝对位移" << std::endl;
        double max_displacement = 0;
        std::vector<double>joint_move_distance(path_list[i].size(), 0);
        for(size_t j = 0; j < path_list[i].size(); ++j)
        {
            joint_move_distance[j] = (path_list[i+1][j] - path_list[i][j])  \
                                        * DEG_TO_RAD_;
  
            // std::cout << joint_move_distance[j] << " ";
            if(fabs(joint_move_distance[j]) > max_displacement)
            {
                max_displacement = fabs(joint_move_distance[j]);
                // std::cout << "\n" << max_displacement << std::endl;
            }
        }
        // std::cout << "\n" << max_displacement << std::endl;

        // 2. 结合最大速度，计算运行时间, 各关节运行速度
        // std::cout << "\n2. 结合最大速度，计算运行时间, 各关节运行速度" << std::endl;
        double operation_time = max_displacement / max_joint_velocity_;
        // std::cout << operation_time << std::endl;

        std::vector<double>joint_move_velocity(joint_move_distance.size(), 0);
        for(size_t j = 0; j < path_list[i].size(); ++j)
        {
            joint_move_velocity[j] = joint_move_distance[j] / operation_time;
            // std::cout << joint_move_velocity[j] << " ";
        }

        // 3. 计算各插补增量值
        // std::cout << "\n3. 计算各插补增量值" << std::endl;
        double traj_operation_time = operation_time * traj_rate_hz_;
        // std::cout << "\n" << traj_operation_time << std::endl;
        std::vector<double>joint_derta(joint_move_distance.size(), 0);
        for(int j = 0; j < joint_derta.size(); ++j)
        {
            joint_derta[j] = joint_move_distance[j] / traj_operation_time;
            // std::cout << joint_derta[j] << " ";
        }
        
        // 4. 插值细化路径
        // std::cout << "\n4. 插值细化路径" << std::endl;
        // 判断机器人基座选择轨迹关节起点
        int temp_start_position;
        if(GRIPPER_ID::G0 == which_base_){
            temp_start_position = 1;
        }else{
            // 3  =  2个正向urdf夹持器 + 1个逆向urdf夹持器
            temp_start_position = 3 + joint_number_;
        }
        int index = 1;
        while (index <= traj_operation_time)
        {
            for(size_t j = 0; j < joint_derta.size(); ++ j)
            {
                // 判断哪个机器人
                    if(0 != joint_derta[j]){
                        // j+1: 跳过夹持器
                        joint_command_msg_->position[j + temp_start_position] =\
                            path_list[i][j] * DEG_TO_RAD_ + \
                            index * joint_derta[j]; 
                    }else{
                        joint_command_msg_->position[j + temp_start_position] =\
                            path_list[i][j] * DEG_TO_RAD_;
                    }
                // std::cout << joint_command_msg_->position[j + 1] << " ";
            }
            // std::cout << "\n";
            traj_list.push_back(*joint_command_msg_);
            index ++;
        }
    }

    return traj_list;
}

int NewModularRobotSimulation::getTransformOneToOne_(const std::string &link1,   \
     const std::string &link2, tf::StampedTransform *transform)
{

    bool flag = true;
    int index = 0;
    while(flag){
        try{
            tf_listener_->waitForTransform(link1, link2, ros::Time(0),  \
                                            ros::Duration(1.0));
            tf_listener_->lookupTransform(link1, link2,ros::Time(0),    \
                                            *transform);
            flag = false;
        }
        catch (tf::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            index ++;
            if(index > 10){
                ROS_ERROR("Can not listen the Transform.");
                return -1;
            }
        }
    }
    return 0;
}

int NewModularRobotSimulation::setTransformOneToOne_(  \
    const tf::StampedTransform &transform)
{
    // 经验设置时间 2s
    ros::Rate  rate(5);
    int index = 0;
    while (nh_.ok() && index < 10)
    {
        tf_broadcaster_->sendTransform(transform);
        ros::spinOnce();    
        rate.sleep();
        index ++;
    }
    if(! nh_.ok()) return -1;
    
    return 0;
}

int NewModularRobotSimulation::getRobotCurrentPosition()
{
    int count = 10;
    ros::Rate rate(count);
    // 超时3秒
    int timeout = 3 * count;
    int index = 0;
    if_get_joint_state_msg_ = false;
    while(!if_get_joint_state_msg_ && index < timeout){
        ros::spinOnce();
        rate.sleep();
        index ++;
    }
    if_get_joint_state_msg_ = false;

    if(index >= timeout)
    {
        ROS_WARN("getRobotCurrentPosition timeout.");
        return -1;
    }

    return 0;
}

int NewModularRobotSimulation::sentJointCommands_( \
    const std::vector<sensor_msgs::JointState> &cmd)
{
    if(cmd.empty())
    {
        ROS_WARN("path list is empty.");
        return -1;
    }

    int length = cmd.size();
    int i = 0;
    ros::Rate hz(traj_rate_hz_);
    while (i < length)
    {
        pub_joint_command_->publish(cmd[i]);
        hz.sleep();
        i++;

        if(if_generate_display_tcp_traj_){
            generate_display_tcp_trajectory();
        }
    }
    return 0;
}

void NewModularRobotSimulation::subRobotStateCallback( \
    const sensor_msgs::JointState::ConstPtr& msg)
{   
    joint_state_msg_ = *msg;
    // for(int i = 2; i <=6; i++){
    //     std::cout << joint_state_msg_.position[i] << " ";
    // }
    if_get_joint_state_msg_ = true;
}

void NewModularRobotSimulation::changeBaseGripper()
{
    if(g0_state_ == GRIPPER_STATE::CLOSE &&     \
        g6_state_ == GRIPPER_STATE::OPEN)
    {
        which_base_ = GRIPPER_ID::G0;
        // std::cout << "1.G0----------"<< "\n";
    }
    else if(g0_state_ == GRIPPER_STATE::OPEN &&    \
             g6_state_ == GRIPPER_STATE::CLOSE)
    {
        which_base_ = GRIPPER_ID::G6;
        // std::cout << "1.G6----------"<< "\n";
    }
}

int NewModularRobotSimulation::enableGripperPlan(const enum  GRIPPER_ID& id,    \
        const enum  GRIPPER_ACTION& action)
{

    // 2次获取机器人最新状态
    while(getRobotCurrentPosition()){};
    while(getRobotCurrentPosition()){};

    for(size_t i = 0; i < joint_command_msg_->position.size() / 2; ++i)
    {
        joint_command_msg_->position[i] = joint_state_msg_.position[i+1];
        // std::cout << joint_command_msg_->position[i] << " ";
    }

    int j = 1;
    for(size_t i = joint_command_msg_->position.size() / 2; \
        i < joint_command_msg_->position.size(); ++i)
    {
        joint_command_msg_->position[i] = joint_state_msg_.position[    \
            joint_state_msg_.position.size() / 2 + j];
        j++;
    }

    // std::cout << std::endl;
    // 获取机器人夹持打开关闭极限距离
    double gripper_distance;
    if(GRIPPER_ACTION::OPEN == action){
        ROS_ASSERT(nh_.getParam("gripper_open_distance", gripper_distance));
    }else{
        ROS_ASSERT(nh_.getParam("gripper_close_distance", gripper_distance));
    }

    // 夹持器运行时间 2s
    int operation_time = 2 * traj_rate_hz_;
    ros::Rate rate(traj_rate_hz_);

    double temp_pos;
    // 获取机器人夹持器位移距离
    if(GRIPPER_ID::G0 == id){
        if(GRIPPER_ID::G0 == which_base_){
            gripper_distance = gripper_distance -   \
                *(joint_state_msg_.position.begin());
            temp_pos = *(joint_state_msg_.position.begin());
        }else{
            gripper_distance = gripper_distance -   \
                joint_state_msg_.position.at(   \
                    joint_state_msg_.position.size() - 2);
            temp_pos = joint_state_msg_.position.at(    \
                joint_state_msg_.position.size() - 2);
        }
        
    }else{
        // G6 joint
        if(GRIPPER_ID::G0 == which_base_){
            gripper_distance = gripper_distance - joint_state_msg_. \
                position.at(joint_state_msg_.position.size() / 2 - 1);
            temp_pos = joint_state_msg_. \
                position.at(joint_state_msg_.position.size() / 2 - 1);
        }else{
            gripper_distance = gripper_distance - joint_state_msg_. \
                position.at(joint_state_msg_.position.size() / 2 + 1);
            temp_pos = joint_state_msg_. \
                position.at(joint_state_msg_.position.size() / 2 + 1);
        }
        
    }
    // 夹持器位移增量
    double derta = fabs(gripper_distance / operation_time);

    // 延时循环发布
    int index = 0;
    while(index < operation_time )
    {
        if(GRIPPER_ID::G0 == id){
            if(GRIPPER_ID::G0 == which_base_){
                *(joint_command_msg_->position.begin()) = temp_pos;
            }else{
                (joint_command_msg_->position.at(   \
                    joint_command_msg_->position.size() - 1)) = temp_pos;
            }
        }else{
            // G6 joint
            if(GRIPPER_ID::G0 == which_base_){
                joint_command_msg_->position.at(joint_number_ + 1) = temp_pos;
            }else{
                joint_command_msg_->position.at(joint_number_ + 2) = temp_pos;
            }
        }

        pub_joint_command_->publish(*joint_command_msg_);
        ros::spinOnce();
        if(GRIPPER_ACTION::OPEN == action){
            temp_pos += derta;
        }else{
            temp_pos -= derta;
        }
        index ++;
        rate.sleep();
    }

    // 更新夹持器状态
    if(GRIPPER_ID::G0 == id){
        g0_state_ = action;
    }else{
        g6_state_ = action;
    }
    return 0;
}


void NewModularRobotSimulation::climbotStepMove(   \
    std::vector<std::vector<double>>& data)
{

    hiddenRobot();

    GRIPPER_ID temp;

    if(GRIPPER_ID::G0 == which_base_ ){
        temp = GRIPPER_ID::G6;
    }else{
        temp = GRIPPER_ID::G0;
    }

    // gripper open
    gripperControl( temp, GRIPPER_ACTION::OPEN);

    sentJointCommands(data);

    // gripper close
    gripperControl( temp, GRIPPER_ACTION::CLOSE);

    dispalyRobot();

    // 切换基座
    which_base_ = (GRIPPER_ID::G0 == which_base_) ? GRIPPER_ID::G6 : \
                    GRIPPER_ID::G0;
    
}

int NewModularRobotSimulation::getBasePos()
{
    if(which_base_ == GRIPPER_ID::G0){
        return getTransformOneToOne_(world_axis_, positive_tcp_,     \
                &transform_base_to_world_);
    }else{
        return getTransformOneToOne_(world_axis_, negetive_tcp_,     \
                &transform_base_to_world_);
    }
}


int NewModularRobotSimulation::hiddenRobot()
{   
    tf::StampedTransform temp;
    if(which_base_ == GRIPPER_ID::G0){
        temp.child_frame_id_ = g6_base_;
    }else{
        temp.child_frame_id_ = g0_base_;
    }
    temp.frame_id_ = world_axis_;
    // 远离可视范围 999 (经验值)
    temp.setOrigin(tf::Vector3(999,0,0));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    temp.setRotation(q);
    temp.stamp_ = ros::Time(0);
    return setTransformOneToOne_(temp);

}

int NewModularRobotSimulation::dispalyRobot()
{
    if(changeRobotJoint()) return -1;
    sleep(1);   // 等待修改关节位置，经验值
    if(getBasePos()) return -1;

    tf::StampedTransform temp;
    if(which_base_ == GRIPPER_ID::G0){
        temp.child_frame_id_ = g6_base_;
    }else{
        temp.child_frame_id_ = g0_base_;
    }
    temp.frame_id_ = world_axis_;
    temp.setOrigin(transform_base_to_world_.getOrigin());
    temp.setRotation(transform_base_to_world_.getRotation());
    temp.stamp_ = ros::Time(0);
    return setTransformOneToOne_(temp);

}

void NewModularRobotSimulation::generate_display_tcp_trajectory()
{
    if(if_generate_display_tcp_traj_)
    {
        // 计算tcp位置
        tf::StampedTransform temp_trans;

        if(GRIPPER_ID::G0 == which_base_){
            getTransformOneToOne_(world_axis_, positive_tcp_, &temp_trans);
        }else{
            getTransformOneToOne_(world_axis_, negetive_tcp_, &temp_trans);
        }

        // 生成环境
        tf::Vector3 posv = temp_trans.getOrigin();
        std::vector<double> position{posv.x(), posv.y(), posv.z()};
        environment_generator_->addLineStrip(position);
    }
}
