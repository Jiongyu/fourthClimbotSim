#ifndef NEW_MODULAR_ROBOT_SIMULATION_H_
#define NEW_MODULAR_ROBOT_SIMULATION_H_

#define DEG_TO_RAD_ (0.0174531)

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>

#include <math.h>

#include <memory>
#include <atomic>

# include "./generate_environment.h"

class NewModularRobotSimulation
{

public:
    // 夹持器id
    enum class GRIPPER_ID : int{
        G0 = 0,
        G6,
    };
    // 夹持器动作
    enum class GRIPPER_ACTION : int{
        OPEN = 0,
        CLOSE,
    };

private:
    // 夹持器状态
    typedef enum GRIPPER_ACTION GRIPPER_STATE;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       

public:
    NewModularRobotSimulation(/* args */);
    ~NewModularRobotSimulation(){};

    /**
     * @brief 发送机器人一个步态路径数据
     * 
     * @param path_list 路径数据
     * @return int 0:success, -1:error
     */
    int sentJointCommands(std::vector<std::vector<double> >& path_list);

    /**
     * @brief 夹持器公职
     * 
     * @param id : 夹持器id
     * @param action 夹持器动作gripperControl
     * @return int 0:success, -1:error
     */
    int gripperControl(const enum  GRIPPER_ID& id,  \
        const enum  GRIPPER_ACTION& action);

    /**
     * @brief 读取 path_list.txt , 控制机器人
     * 
     * @return int 0:success, -1:error
     */
    int controlRobotByConfigureFile(const std::string &name);

private:
    /**
     * @brief 基座为G6时， 翻转各关节数据
     * 
     * @param path_list 
     */
    void changeJointPathByBase(std::vector<std::vector<double> >& path_list);
    
    /**
     * @brief 根据夹持器基座，隐藏另外一个机器人
     * 
     * @return int 
     */
    int hiddenRobot();

    /**
     * @brief 步态切换时，显示另外一个机器人 
     * 
     * @return int 
     */
    int dispalyRobot();

    /**
     * @brief 获取基座端位姿
     * 
     * @return int 
     */
    int getBasePos();

    /**
     * @brief 切换夹持器时，翻转关节数据
     * 
     * @return int 0:success, -1:error
     */
    int changeRobotJoint();

    /**
     * @brief 获取当前机器人关节数据
     * 
     * @return int 0:success, -1:error
     */
    int getRobotCurrentPosition();

    /**
     * @brief 获取路径细插补结果
     * 
     * @param path_list  粗糙路径
     * @return std::vector<sensor_msgs::JointState>  路径细插补结果
     */
    std::vector<sensor_msgs::JointState> getPathPlanResult( \
        const std::vector<std::vector<double>>& path_list) const;

    /**
     * @brief 夹持器运动细插补
     * 
     */
    int enableGripperPlan(const enum  GRIPPER_ID& id,    \
        const enum  GRIPPER_ACTION& action);

    /**
     * @brief 订阅机器人状态回调函数
     * 
     */
    static void subRobotStateCallback(  \
        const sensor_msgs::JointState::ConstPtr& msg);

    /**
     * @brief 切换机器人基座
     * 
     */
    void changeBaseGripper();

    /**
     * @brief 机器人步态运动
     * 
     * @param data 机器人步态数据
     */
    void climbotStepMove(std::vector<std::vector<double>>& data);

private:
    /**
     * @brief 获取 link2 相对于 link1 坐标变换
     * 
     * @param link1 
     * @param link2 
     * @param transform 
     * @return int 0:success, -1:error
     */
    int getTransformOneToOne_(const std::string &link1, \
        const std::string &link2, tf::StampedTransform *transform);

    /**
     * @brief 设置坐标系变换
     * 
     * @param transform 
     * @return int 0:success, -1:error
     */
    int setTransformOneToOne_(const tf::StampedTransform &transform);
    
    /**
     * @brief 模拟底层伺服控制机器人
     * 
     * @param cmd 路径点数据
     * @return int 0:success, -1:error
     */
    int sentJointCommands_(const std::vector<sensor_msgs::JointState> &cmd);

    /**
     * @brief 生成可视化机器人tcp端轨迹
     * 
     */
    void generate_display_tcp_trajectory();

private:

    // 关节数量
    int joint_number_; 

    // ros 句柄
    ros::NodeHandle nh_;

    // ros坐标变化发布器，更改 (g0_tcp or g6_tcp) --> world
    std::unique_ptr<tf::TransformBroadcaster> tf_broadcaster_;
    
    // ros坐标变化监听器 获取 (g0_tcp or g6_tcp) --> world
    std::unique_ptr<tf::TransformListener> tf_listener_;
    
    // 机器人关节位置命令
    std::unique_ptr<sensor_msgs::JointState> joint_command_msg_;

    // 机器人正逆urdf  关节名称
    std::shared_ptr<std::vector<std::string>> joint_name_;
    // 机器人关节状态命令
    static sensor_msgs::JointState joint_state_msg_;
    static std::atomic_bool if_get_joint_state_msg_; 

    // 机器人关节控制命令发布器
    std::unique_ptr<ros::Publisher> pub_joint_command_;

    // 获取机器人关节状态订阅器
    std::unique_ptr<ros::Subscriber> sub_joint_state_;

    // base --> world 变换矩阵
    tf::StampedTransform transform_base_to_world_;

    // 轨迹生成单位每秒次数(hz)
    int traj_rate_hz_;

    // 判断机器人当前哪一个夹持器为基座
    enum GRIPPER_ID which_base_;
    GRIPPER_STATE g0_state_;
    GRIPPER_STATE g6_state_;

    // 最大关节速度
    double max_joint_velocity_;

    // 世界坐标系
    std::string world_axis_;

    // g0 base
    std::string g0_base_;

    // g6 base
    std::string g6_base_;

    // 正向机器人tcp
    std::string positive_tcp_;

    // 逆向机器人tcp
    std::string negetive_tcp_;

    // 机器人关节方向
    static std::unique_ptr<std::vector<int>> joint_pos_direction_;
    static std::unique_ptr<std::vector<int>> joint_neg_direction_;

    // 是否自动生成机器人tcp轨迹可视化
    bool if_generate_display_tcp_traj_;

    // 生成机器人末端tcp轨迹
    std::unique_ptr<EnvironmentGenerator> environment_generator_;

};

#endif // MODULAR_ROBOT_SIMULATION_H_ end
