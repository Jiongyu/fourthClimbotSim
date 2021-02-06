#ifndef GENERATE_ENVIRONMENT_H_
#define GENERATE_ENVIRONMENT_H_

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <memory>

#define DEG_TO_RAD_ (0.0174531)

class EnvironmentGenerator
{

    // mark 类型
    enum class MARK_TYPE : int{
        CUBE = 0,
        SPHERE,
        CYLINDER,
        POINT,
        LINE_STRIP,
    };

public:
    EnvironmentGenerator(/* args */);
    ~EnvironmentGenerator(){};
    
    /**
     * @brief 向环境中添加杆件
     * 
     * @param position 物体端点位置（xyz）单位:m
     * @param orientation 物体姿态（rpy）单位:deg
     * @param scale 物体尺寸 （xyz）单位:m 
     */
    void addPole(const std::vector<double>& position,    \
                  const std::vector<double>& orientation, \
                  const std::vector<double>& scale);

    /**
     * @brief 向环境中添加壁面
     * 
      * @param position 物体端点位置（xyz）单位:m
     * @param orientation 物体姿态（rpy）单位:deg
     * @param scale 物体尺寸 （xyz）单位:m
     */     
    void addWall(const std::vector<double>& position,    \
                  const std::vector<double>& orientation, \
                  const std::vector<double>& scale);

    /**
     * @brief 向环境中添加点
     * 
     * @param position 物体端点位置（xyz）单位:m
     * @param orientation 物体姿态（rpy）单位:deg
     * @param scale 物体尺寸 （xyz）单位:m
     */
    void addPoint(const std::vector<double>& position,    \
                    const std::vector<double>& orientation, \
                    const std::vector<double>& scale);

    /**
     * @brief 向环境中添加线段
     * 
     * @param position 物体端点位置（xyz）单位:m
     */
    void addLineStrip(const std::vector<double>& position);

    /**
     * @brief 清楚机器人步态过渡生成轨迹的直线
     * 
     */
    void clearRobotStepLineStrip();

    /**
     * @brief 生成环境
     * 
     */
    void generateEnvironment();
    
    void generateEnvironmentByYaml();

private:
    /**
     * @brief 设置环境物体
     * 
     * @param position 物体端点位置（xyz）单位:m
     * @param orientation 物体姿态（rpy）单位:deg
     * @param scale 物体尺寸 （xyz）单位:m
     * @param color 物体颜色 (rgba)
     * @param type  物体类型(MARK_TYPE)
     * @param name_space 物体命名空间(/pole, /wall, /point)
     */
    void setMarker(const std::vector<double>& position, \
                    const std::vector<double>& orientation, \
                    const std::vector<double>& scale, \
                    const std::vector<double>& color, \
                    const enum MARK_TYPE& type, \
                    const std::string& name_space);

    void setTrajMarker(const std::vector<double>& position,    \
                    const enum MARK_TYPE& type);


    void processYamlFile(const std::string& name,  \
                        std::vector<double>* position,  \
                        std::vector<double>* orientation,  \
                        std::vector<double>* scale);

    /**
     * @brief 生成环境
     * 
     */
    void generateEnvironment_();

private:
    
    // 环境物体数量
    unsigned int item_number_;

    // 世界参考坐标系
    std::string world_axis_;

    // ros 句柄
    std::shared_ptr<ros::NodeHandle> nh_;

    // 环境物体发布器
    std::unique_ptr<ros::Publisher> marker_pub_;

    // 环境物体
    std::unique_ptr<visualization_msgs::Marker> mark_;
    // 环境物体集合
    std::unique_ptr<visualization_msgs::MarkerArray> mark_array_;

    std::unique_ptr<std::vector<std::string>> env_pole_;
    std::unique_ptr<std::vector<std::string>> env_wall_;
    std::unique_ptr<std::vector<std::string>> env_point_;

 
};  

#endif // end GENERATE_ENVIRONMENT_H_

