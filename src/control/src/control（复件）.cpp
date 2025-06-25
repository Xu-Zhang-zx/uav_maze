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

mavros_msgs::State current_state;
quadrotor_msgs::PositionCommand postion_cmd;
geometry_msgs::PoseStamped iris_pose;
bool receivedCmd = false;
ros::Time last_cmd_time;

// 目标检测相关变量
int continuous_count = 0;  // 连续检测计数器
const int DETECTION_THRESHOLD = 20;  // 连续检测阈值
const double IMAGE_WIDTH = 1280.0;
const double IMAGE_HEIGHT = 720.0;
const double IMAGE_CENTER_X = IMAGE_WIDTH / 2.0;
const double IMAGE_CENTER_Y = IMAGE_HEIGHT / 2.0;
const double CENTER_THRESHOLD = 30.0;  // 像素阈值
double target_center_x = 0.0;
double target_center_y = 0.0;

// 状态标志
bool isLanding = false;
bool isImgServer = false;
std::string exec_state = "INIT";

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

void BoundingBoxesCallback(const control::BoundingBoxes::ConstPtr& msg)
{
    if (msg->bounding_boxes.empty() || abs(postion_cmd.position.x) < 3 || abs(postion_cmd.position.y) < 3)
    {
        continuous_count = 0;
        return;
    }

    // 使用第一个检测到的目标
    const control::BoundingBox& bbox = msg->bounding_boxes[0];

    if(bbox.Class == "H")
    {
        // 计算目标中心坐标
        target_center_x = (bbox.xmin + bbox.xmax) / 2.0;
        target_center_y = (bbox.ymin + bbox.ymax) / 2.0;
        continuous_count++;
    } else {
        continuous_count = 0;
    }
}

// 获取当前偏航角
double getCurrentYaw()
{
    tf2::Quaternion q(
        iris_pose.pose.orientation.x,
        iris_pose.pose.orientation.y,
        iris_pose.pose.orientation.z,
        iris_pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    return yaw;
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
    ros::Time last_request = ros::Time::now();
    ros::Time last_state_info_time = ros::Time::now();

    ROS_INFO("Starting control loop...");
    exec_state = "EXPLORING";
    
    // ====================== 主控制循环 ======================
    while(ros::ok())
    {
        // --------------------- 模式切换处理 ---------------------
        if(current_state.mode != "OFFBOARD" && 
           (ros::Time::now() - last_request > ros::Duration(0.5)) &&
           !isLanding && !isImgServer)
        {
            mavros_msgs::SetMode offboard_set_mode;
            offboard_set_mode.request.custom_mode = "OFFBOARD";
            if(set_mode_client.call(offboard_set_mode) && 
               offboard_set_mode.response.mode_sent)
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
                !isLanding && !isImgServer)
        {
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = true;
            if(arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }

        // --------------------- 正常飞行控制 ---------------------
        if (!isImgServer && !isLanding)
        {
            if (receivedCmd && (ros::Time::now() - postion_cmd.header.stamp) < ros::Duration(0.05))
            {
                setTarget.position.x = postion_cmd.position.x;
                setTarget.position.y = postion_cmd.position.y;
                setTarget.position.z = postion_cmd.position.z;
                setTarget.yaw = postion_cmd.yaw;

                setlocal_pub.publish(setTarget);
                last_setTarget = setTarget;
                exec_state = "EXPLORING";
            }
            else
            {
                setlocal_pub.publish(last_setTarget);
                exec_state = "HOVER";
            }
        }

        // 连续检测到目标200次进入调整模式
        if (!isImgServer && !isLanding && continuous_count > DETECTION_THRESHOLD)
        {
            isImgServer = true;
            exec_state = "Img_server";
            ROS_INFO("Continuous target detected! Switching to adjustment mode.");
        }

        // 目标调整模式
        if (isImgServer && !isLanding)
        {
            // 计算图像误差
            double x_img_error = target_center_x - IMAGE_CENTER_X;
            double y_img_error = target_center_y - IMAGE_CENTER_Y;
            
            // 获取当前偏航角
            double current_yaw = getCurrentYaw();
            
            // 计算机体坐标系下的期望移动量
            double desire_x_body = -0.002 * y_img_error;
            double desire_y_body = -0.002 * x_img_error;
            
            // 转换为世界坐标系
            double desire_x_local = desire_x_body * cos(current_yaw) - desire_y_body * sin(current_yaw);
            double desire_y_local = desire_x_body * sin(current_yaw) + desire_y_body * cos(current_yaw);
            
            // 设置目标位置
            setTarget.position.x = iris_pose.pose.position.x + desire_x_local;
            setTarget.position.y = iris_pose.pose.position.y + desire_y_local;
            setTarget.position.z = postion_cmd.position.z;
            setTarget.yaw = current_yaw;
            
            setlocal_pub.publish(setTarget);
            last_setTarget = setTarget;
            
            ROS_INFO_THROTTLE(0.5, "Adjusting: dx=%.1f, dy=%.1f", x_img_error, y_img_error);
            
            // 检查目标是否居中
            if (std::abs(x_img_error) < CENTER_THRESHOLD && std::abs(y_img_error) < CENTER_THRESHOLD)
            {
                ROS_INFO("Target centered! Switching to LAND mode.");
                isLanding = true;
                exec_state = "LANDING";
            }
        }

        // 着陆模式
        if (isLanding)
        {
            mavros_msgs::SetMode land_set_mode;
            land_set_mode.request.custom_mode = "AUTO.LAND";
            if(set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent)
            {
                ROS_INFO_THROTTLE(1, "Landing mode activated");
            }
        }

        // --------------------- 状态信息输出 ---------------------
        if ((ros::Time::now() - last_state_info_time) > ros::Duration(1.0))
        {
            ROS_INFO_STREAM("Current state: " << exec_state
                         << " | Height: " << iris_pose.pose.position.z << "m"
                         << " | Mode: " << current_state.mode
                         << " | Continuous detections: " << continuous_count);
            
            last_state_info_time = ros::Time::now();
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}