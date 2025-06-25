#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <geometry_msgs/PoseStamped.h>
#include <control/BoundingBox.h>
#include <control/BoundingBoxes.h>
#include <tf2/utils.h>

// 全局变量声明
ros::Subscriber state_sub, poscmd_sub, bounding_boxes_sub, pose_sub;
ros::Publisher setlocal_pub;
ros::ServiceClient arming_client, set_mode_client;
double roll, pitch, yaw;

mavros_msgs::State current_state;
quadrotor_msgs::PositionCommand postion_cmd;
geometry_msgs::PoseStamped iris_pose;
bool receivedCmd = false;
ros::Time last_cmd_time;

// 目标检测相关变量
bool target_detected = false;
double target_center_x = 0.0;
double target_center_y = 0.0;
const double IMAGE_CENTER_X = 640.0;
const double IMAGE_CENTER_Y = 360.0;
const double CENTER_THRESHOLD = 50.0;  // 像素阈值
const double Kp = 0.002; 
double target_yaw = 0;

// 无人机状态枚举
enum State {
    INIT,           // 初始化
    EXPLORE,        // 探索模式
    HOVER,          // 悬停模式
    TARGET_ADJUST,  // 目标调整
    LAND            // 着陆模式
};

State exec_state = INIT;

// ====================== 回调函数 ======================
void MAVROSStateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void PositionCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg)
{
    receivedCmd = true;
    postion_cmd = *msg;
    last_cmd_time = ros::Time::now();
}

void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    iris_pose = *msg;
}

// 获取当前偏航角
double getCurrentYaw()
{
    tf2::Quaternion q(
        iris_pose.pose.orientation.x,
        iris_pose.pose.orientation.y,
        iris_pose.pose.orientation.z,
        iris_pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

void BoundingBoxesCallback(const control::BoundingBoxes::ConstPtr& msg)
{
    if (msg->bounding_boxes.empty() || abs(postion_cmd.position.x) < 3 || abs(postion_cmd.position.y) < 3)
    {
        target_detected = false;
        return;
    }

    // 使用第一个检测到的目标
    const control::BoundingBox& bbox = msg->bounding_boxes[0];

    // 计算目标中心坐标
    target_center_x = (bbox.xmin + bbox.xmax) / 2.0;
    target_center_y = (bbox.ymin + bbox.ymax) / 2.0;
    target_yaw = getCurrentYaw();
    target_detected = true;
    
    // 如果当前在探索或悬停模式，切换到目标调整模式
    if (exec_state == EXPLORE || exec_state == HOVER)
    {
        exec_state = TARGET_ADJUST;
        ROS_INFO("Target detected! Switching to TARGET_ADJUST mode.");
    }
}

// ====================== 辅助函数 ======================
// 计算目标偏移量并转换为无人机移动指令（带偏航补偿）
void calculateTargetAdjustment(double& move_x, double& move_y)
{
    // 计算目标中心与图像中心的偏移量
    double dx = target_center_x - IMAGE_CENTER_X;  // 图像x方向偏移（正数表示目标在右侧）
    double dy = target_center_y - IMAGE_CENTER_Y;  // 图像y方向偏移（正数表示目标在下方）
    
    // 相机坐标系到机体坐标系转换（假设相机固定向下）
    // 图像dx -> 无人机应向右移动（X轴正方向）
    // 图像dy -> 无人机应向前移动（Y轴负方向）
    
    // 计算机体坐标系的期望移动量
    double body_move_x = -Kp * dy;
    double body_move_y = -Kp * dx;
    
    // 坐标系变换：从机体坐标系到世界坐标系
    // 使用旋转矩阵补偿偏航角
    move_x = body_move_x * cos(yaw) - body_move_y * sin(yaw);
    move_y = body_move_x * sin(yaw) + body_move_y * cos(yaw);
    
}

// ====================== 主函数 ======================
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "control");
    ros::NodeHandle nh("~");
    
    // 从参数服务器获取参数
    double hover_height = 2.0; // 悬停高度
    nh.param("hover_height", hover_height, 2.0);

    // --------------------- 初始化订阅者 ---------------------
    state_sub = nh.subscribe<mavros_msgs::State>("/iris_0/mavros/state", 1, MAVROSStateCallback);
    poscmd_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 1, PositionCmdCallback);
    bounding_boxes_sub = nh.subscribe<control::BoundingBoxes>("/yolov11/BoundingBoxes", 10, BoundingBoxesCallback);
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/iris_0/mavros/local_position/pose", 10, PoseCallback);

    // --------------------- 初始化发布者 ---------------------
    setlocal_pub = nh.advertise<mavros_msgs::PositionTarget>("/iris_0/mavros/setpoint_raw/local", 10);

    // --------------------- 初始化服务客户端 ---------------------
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/iris_0/mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/iris_0/mavros/set_mode");
    
    // 100Hz控制频率
    ros::Rate rate(100);

    // 等待PX4连接
    ROS_INFO("Waiting for PX4 connection...");
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("PX4 connected!");

    // 初始化位置目标
    mavros_msgs::PositionTarget setTarget;
    setTarget.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    setTarget.type_mask = mavros_msgs::PositionTarget::IGNORE_VX + 
                          mavros_msgs::PositionTarget::IGNORE_VY + 
                          mavros_msgs::PositionTarget::IGNORE_VZ + 
                          mavros_msgs::PositionTarget::IGNORE_AFX + 
                          mavros_msgs::PositionTarget::IGNORE_AFY + 
                          mavros_msgs::PositionTarget::IGNORE_AFZ +
                          mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    setTarget.position.x = 0;
    setTarget.position.y = 0;
    setTarget.position.z = hover_height;
    setTarget.yaw = 0;

    // 发布初始目标
    for(int i = 0; ros::ok() && i < 100; i++)
    {
        setlocal_pub.publish(setTarget);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::PositionTarget last_setTarget = setTarget;
    bool isLanding = false;

    // 准备服务和状态变量
    mavros_msgs::SetMode offboard_set_mode;
    offboard_set_mode.request.custom_mode = "OFFBOARD";
    
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    
    ros::Time last_request = ros::Time::now();
    ros::Time last_state_info_time = ros::Time::now();
    ros::Time last_detection_time;

    ROS_INFO("Starting control loop...");
    
    // ====================== 主控制循环 ======================
    while(ros::ok())
    {
        // --------------------- 模式切换处理 ---------------------
        if(current_state.mode != "OFFBOARD" && 
           (ros::Time::now() - last_request > ros::Duration(0.5)) &&
           !isLanding)
        {
            if(set_mode_client.call(offboard_set_mode) && offboard_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
                // 重新发布目标保持连接
                setlocal_pub.publish(last_setTarget);
            }
            last_request = ros::Time::now();
        } 
        // --------------------- 解锁处理 ---------------------
        else if(!current_state.armed && 
                (ros::Time::now() - last_request > ros::Duration(0.5)) &&
                !isLanding)
        {
            if(arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }

        // --------------------- 状态机处理 ---------------------
        switch(exec_state)
        {
            case INIT:
                // 初始状态，切换到探索模式
                exec_state = EXPLORE;
                ROS_INFO("Switching to EXPLORE mode.");
                break;
                
            case EXPLORE:
                // 探索模式：使用规划模块的命令
                if (receivedCmd && (ros::Time::now() - postion_cmd.header.stamp) < ros::Duration(0.1))
                {
                    setTarget.position.x = postion_cmd.position.x;
                    setTarget.position.y = postion_cmd.position.y;
                    setTarget.position.z = postion_cmd.position.z;
                    setTarget.yaw = postion_cmd.yaw;

                    setlocal_pub.publish(setTarget);
                    last_setTarget = setTarget;
                    last_detection_time = ros::Time::now();
                }
                else
                {
                    // 没有新命令时保持位置
                    setlocal_pub.publish(last_setTarget);
                    if ((ros::Time::now() - last_cmd_time) > ros::Duration(1.0))
                    {
                        exec_state = HOVER;
                        ROS_INFO("No command received for 1 second. Switching to HOVER mode.");
                    }
                }
                break;
                
            case HOVER:
                // 悬停模式：保持最后位置
                setlocal_pub.publish(last_setTarget);
                if ((ros::Time::now() - last_cmd_time) < ros::Duration(0.1))
                {
                    exec_state = EXPLORE;
                    ROS_INFO("Received new planning command. Switching to EXPLORE mode.");
                }
                break;
                
            case TARGET_ADJUST:
                // 目标调整模式
                last_detection_time = ros::Time::now();
                
                // 计算无人机需要的移动量
                double move_x, move_y;

                calculateTargetAdjustment(move_x, move_y);
                
                // 设置新目标位置
                setTarget.position.x = iris_pose.pose.position.x + move_x;
                setTarget.position.y = iris_pose.pose.position.y + move_y;
                setTarget.position.z = hover_height;
                setTarget.yaw = target_yaw;
                
                setlocal_pub.publish(setTarget);
                
                // 检查目标是否居中
                {
                    double dx = target_center_x - IMAGE_CENTER_X;
                    double dy = target_center_y - IMAGE_CENTER_Y;
                    ROS_INFO("dx: %f, dy: %f\n", dx, dy);
                    if (std::abs(dx) < CENTER_THRESHOLD && std::abs(dy) < CENTER_THRESHOLD)
                    {
                        ROS_INFO("Target centered! Switching to LAND mode.");
                        isLanding = true;
                        exec_state = LAND;
                    }
                    else
                    {
                        exec_state = TARGET_ADJUST;
                    }
                }
                break;
                
            case LAND:
                // 着陆模式
                mavros_msgs::SetMode land_set_mode;
                land_set_mode.request.custom_mode = "AUTO.LAND";
                if(set_mode_client.call(land_set_mode))
                {
                    if(land_set_mode.response.mode_sent)
                    {
                        ROS_INFO("Landing mode activated");
                    }
                    else
                    {
                        ROS_WARN("Landing mode not accepted by PX4");
                    }
                }
                else
                {
                    ROS_WARN("Failed to call land service");
                }
                break;
        }

        // --------------------- 状态信息输出 ---------------------
        if ((ros::Time::now() - last_state_info_time) > ros::Duration(1.0))
        {
            const char* state_names[] = {"INIT", "EXPLORE", "HOVER", "TARGET_ADJUST", "LAND"};
            ROS_INFO_STREAM("Current state: " << state_names[exec_state]
                         << " | Height: " << iris_pose.pose.position.z << "m"
                         << " | Mode: " << current_state.mode);
            
            if (target_detected)
            {
                ROS_INFO_STREAM("Target position: (" << target_center_x << ", " << target_center_y << ")");
            }
            
            last_state_info_time = ros::Time::now();
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}