/*****************************************************************************************
 * 自定义控制器跟踪egoplanner轨迹
 * 本代码采用的mavros的速度控制进行跟踪
 * 编译成功后直接运行就行，遥控器先position模式起飞，然后rviz打点，再切offborad模式即可
 ******************************************************************************************/
#include <XmlRpc.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <cmath>
#include <vector>

#include "quadrotor_msgs/PositionCommand.h"
#define VELOCITY2D_CONTROL \
  0b101111000111  // 设置好对应的掩码，从右往左依次对应PX/PY/PZ/VX/VY/VZ/AX/AY/AZ/FORCE/YAW/YAW-RATE
// 设置掩码时注意要用的就加上去，用的就不加，这里是用二进制表示，我需要用到VX/VY/VZ/YAW，所以这四个我给0，其他都是1.

struct waypoint {
  double x;
  double y;
  double z;
  double yaw;
  int flag;
};

class Ctrl {
 public:
  Ctrl();
  void state_cb(const mavros_msgs::State::ConstPtr& msg);
  void position_cb(const nav_msgs::Odometry::ConstPtr& msg);
  void target_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void twist_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg);
  void control(const ros::TimerEvent&);
  void Pub_ego_cmd();
  void Pub_px4_cmd(double cmd_x, double cmd_y, double cmd_z, double cmd_yaw);
  void Waypoint_get();
  void sendLandingCommand();
  void performSpiralMovement(const waypoint& target);
  void executeEgoPoints();
  void executeYawRotation(const waypoint& target);
  bool isAtTarget(const waypoint& target);
  void publishEgoWaypoint(const waypoint& target);

  ros::NodeHandle nh;
  visualization_msgs::Marker trackpoint;
  quadrotor_msgs::PositionCommand ego;
  tf::StampedTransform ts;  // 用来发布无人机当前位置的坐标系坐标轴
  tf::TransformBroadcaster tfBroadcasterPointer;  // 广播坐标轴
  unsigned short velocity_mask = VELOCITY2D_CONTROL;
  mavros_msgs::PositionTarget current_goal;
  mavros_msgs::RCIn rc;
  nav_msgs::Odometry position_msg;
  geometry_msgs::PoseStamped target_pos;
  mavros_msgs::State current_state;
  float position_x, position_y, position_z, now_x, now_y, now_yaw, current_yaw,
      targetpos_x, targetpos_y;
  float ego_pos_x, ego_pos_y, ego_pos_z, ego_vel_x, ego_vel_y, ego_vel_z,
      ego_a_x, ego_a_y, ego_a_z, ego_yaw,
      ego_yaw_rate;  // EGO planner information has position velocity
                     // acceleration yaw yaw_dot
  bool receive, get_now_pos, landing_succelly_flag;
  ;  // 触发轨迹的条件判断
  int mission_flag;
  ros::Subscriber state_sub, twist_sub, target_sub, position_sub;
  ros::Publisher local_pos_pub, pubMarker, mission_waypoint_pub;
  ros::ServiceClient arming_client, set_mode_client;
  ros::ServiceClient landing_client_;
  ros::Timer timer;
  ros::Time last_request;
  // 标志flag 表示ego进入房间内完成
  std_msgs::Int32 ego_into_end_flag;
  mavros_msgs::SetMode offb_set_mode;
  mavros_msgs::CommandBool arm_cmd;
  mavros_msgs::PositionTarget init_pos;
  geometry_msgs::PoseStamped mission_point;
  std::vector<waypoint> ego_target_points;
  XmlRpc::XmlRpcValue point_list;
  int ego_points_index;
  int pub_times;
  ros::Time start_time_;
};
Ctrl::Ctrl() {
  timer = nh.createTimer(ros::Duration(0.01), &Ctrl::control, this);
  state_sub = nh.subscribe("/iris_0/mavros/state", 10, &Ctrl::state_cb, this);
  position_sub = nh.subscribe("/iris_0/mavros/local_position/odom", 10,
                              &Ctrl::position_cb, this);
  target_sub =
      nh.subscribe("/move_base_simple/goal", 10, &Ctrl::target_cb, this);
  mission_waypoint_pub =
      nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  twist_sub = nh.subscribe("/position_cmd", 10, &Ctrl::twist_cb, this);
  local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>(
      "/iris_0/mavros/setpoint_raw/local", 1);
  pubMarker = nh.advertise<visualization_msgs::Marker>("/track_drone_point", 5);

  arming_client =
      nh.serviceClient<mavros_msgs::CommandBool>("/iris_0/mavros/cmd/arming");
  set_mode_client =
      nh.serviceClient<mavros_msgs::SetMode>("/iris_0/mavros/set_mode");
  landing_client_ =
      nh.serviceClient<mavros_msgs::CommandTOL>("/iris_0/mavros/cmd/land");
  get_now_pos = false;
  receive = true;
  mission_flag = 0;
  offb_set_mode.request.custom_mode = "OFFBOARD";
  arm_cmd.request.value = true;
  landing_succelly_flag = false;
  // offboard 起飞预先发布点
  init_pos.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  init_pos.header.stamp = ros::Time::now();
  init_pos.type_mask = 0b101111111000;
  init_pos.position.x = 0;
  init_pos.position.y = 0;
  init_pos.position.z = 0;
  init_pos.yaw = 0;
  // ego目标点序号
  ego_points_index = 0;
  pub_times = 0;
}
void Ctrl::state_cb(const mavros_msgs::State::ConstPtr& msg) {
  current_state = *msg;
}

void Ctrl::sendLandingCommand() {
  mavros_msgs::CommandTOL landing_cmd;
  landing_cmd.request.min_pitch = 0;
  landing_cmd.request.yaw = 0;
  landing_cmd.request.latitude = 0;
  landing_cmd.request.longitude = 0;
  landing_cmd.request.altitude = 0;
  if (landing_client_.call(landing_cmd)) {
    if (landing_cmd.response.success) {
      landing_succelly_flag = true;
      ROS_INFO("Landing command sent successfully");
    } else {
      ROS_ERROR("Failed to send landing command");
    }
  } else {
    ROS_ERROR("Failed to call landing service");
  }
  ros::Duration(1.0).sleep();
}

void Ctrl::target_cb(
    const geometry_msgs::PoseStamped::ConstPtr& msg)  // 读取rviz的航点
{
  // ROS_INFO("WAYPOINT GET");
  if (mission_flag == 4) {
    ROS_INFO("waypoint rviz get");
    receive = true;
    // target_pos = *msg;
    // targetpos_x = target_pos.pose.position.x;
    // targetpos_y = target_pos.pose.position.y;

    mission_flag = 5;
    // for (int i = 0; i < 100; i++) ROS_INFO("mf:%d", mission_flag);
  }
}

void Ctrl::Waypoint_get() {
  // ego目标点坐标姿态
  // ros::Rate rate(20.0);
  // while (yaw_init_flag == 0) {
  //   ROS_INFO("-----");
  //   ros::spinOnce();
  //   rate.sleep();
  // }
  if (nh.getParam("waypoints", point_list)) {
    for (int i = 0; i < point_list.size(); ++i) {
      waypoint point;
      point.x = static_cast<double>(point_list[i]["x"]);
      point.y = static_cast<double>(point_list[i]["y"]);
      point.z = static_cast<double>(point_list[i]["z"]);
      point.yaw = static_cast<double>(point_list[i]["yaw"]);
      point.flag = static_cast<int>(point_list[i]["flag"]);
      ego_target_points.push_back(point);
    }
  }
}

// read vehicle odometry
void Ctrl::position_cb(const nav_msgs::Odometry::ConstPtr& msg) {
  position_msg = *msg;
  tf2::Quaternion quat;
  tf2::convert(
      msg->pose.pose.orientation,
      quat);  // 把mavros/local_position/pose里的四元数转给tf2::Quaternion quat
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  ts.stamp_ = msg->header.stamp;
  ts.frame_id_ = "world";
  ts.child_frame_id_ = "drone_frame";
  ts.setRotation(tf::Quaternion(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
  ts.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y,
                           msg->pose.pose.position.z));
  tfBroadcasterPointer.sendTransform(ts);
  if (!get_now_pos) {
    now_x = position_msg.pose.pose.position.x;
    now_y = position_msg.pose.pose.position.y;
    tf2::Quaternion quat;
    tf2::convert(msg->pose.pose.orientation, quat);
    now_yaw = yaw;
    get_now_pos = true;
  }
  position_x = position_msg.pose.pose.position.x;
  position_y = position_msg.pose.pose.position.y;
  position_z = position_msg.pose.pose.position.z;
  current_yaw = yaw;
}

void Ctrl::Pub_ego_cmd() {
  current_goal.coordinate_frame = mavros_msgs::PositionTarget::
      FRAME_LOCAL_NED;  // 选择local系，一定要local系
  current_goal.header.stamp = ros::Time::now();
  current_goal.type_mask =
      velocity_mask;  // 这个就算对应的掩码设置，可以看mavros_msgs::PositionTarget消息格式
  current_goal.velocity.x = 0.8 * ego_vel_x + (ego_pos_x - position_x) * 1;
  current_goal.velocity.y = 0.8 * ego_vel_y + (ego_pos_y - position_y) * 1;
  current_goal.velocity.z = (ego_pos_z - position_z) * 1;
  current_goal.yaw = ego_yaw;
  ROS_INFO(
      "已触发控制器，当前EGO规划速度：vel_x,vel_z = %.2f,%.2f\n",
      sqrt(pow(current_goal.velocity.x, 2) + pow(current_goal.velocity.y, 2)),
      current_goal.velocity.z);
  // ROS_INFO("1111111-------------");

  local_pos_pub.publish(current_goal);
}

void Ctrl::Pub_px4_cmd(double cmd_x, double cmd_y, double cmd_z,
                       double cmd_yaw) {
  current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  current_goal.header.stamp = ros::Time::now();
  current_goal.type_mask = 0b101111111000;
  current_goal.position.x = cmd_x;
  current_goal.position.y = cmd_y;
  current_goal.position.z = cmd_z;
  current_goal.yaw = cmd_yaw;
  // ROS_INFO("Pub px4 cmd\n");

  local_pos_pub.publish(current_goal);
}

void Ctrl::executeEgoPoints() {
  if (ego_points_index < ego_target_points.size()) {
    const auto& current_target = ego_target_points[ego_points_index];

    // 根据目标点的标志进行处理
    switch (current_target.flag) {
      case 0:  // 普通目标点
        if (pub_times < 5) {
          publishEgoWaypoint(current_target);
          pub_times++;
        }

        else {
          // 检查是否到达目标点
          if (isAtTarget(current_target)) {
            ego_points_index++;
            pub_times = 0;  // 重置发布标志
          }
          Pub_ego_cmd();
        }
        break;

      case 1:  // 旋转目标点
        executeYawRotation(current_target);
        break;

      case 2:  // 循环旋转
        performSpiralMovement(current_target);
        break;

      default:
        ROS_WARN("Unknown target flag: %d", current_target.flag);
        break;
    }
  } else {
    mission_flag = 6;  // 完成所有目标点
    start_time_ = ros::Time::now();
  }
}

void Ctrl::publishEgoWaypoint(const waypoint& target) {
  mission_point.header.stamp = ros::Time::now();
  mission_point.header.frame_id = "world";
  mission_point.pose.position.x = target.x;
  mission_point.pose.position.y = target.y;
  mission_point.pose.position.z = target.z;

  mission_waypoint_pub.publish(mission_point);
}

bool Ctrl::isAtTarget(const waypoint& target) {
  return (abs(target.x - position_x) < 0.2f &&
          abs(target.y - position_y) < 0.2f);
}

void Ctrl::executeYawRotation(const waypoint& target) {
  static bool yawCalculated = false;
  static double target_yaw = 0.0;

  if (!yawCalculated) {
    target_yaw = target.yaw + current_yaw;
    // 归一化目标yaw
    if (target_yaw > M_PI) target_yaw -= 2 * M_PI;
    if (target_yaw < -M_PI) target_yaw += 2 * M_PI;
    yawCalculated = true;  // 设定标志，避免重复计算
  }

  // 旋转逻辑
  if (abs(target_yaw - current_yaw) > 0.1) {
    Pub_px4_cmd(target.x, target.y, target.z, target_yaw);
    ROS_INFO("Yaw Command:target/current: %f,%f", target_yaw, current_yaw);
  } else {
    // 达到目标 yaw，重置标志
    yawCalculated = false;
    ego_points_index++;  // 移动到下一个目标点
  }
}

void Ctrl::performSpiralMovement(const waypoint& target) {
  static bool yawCalculated = false;
  static double first_yaw = 0.0;
  static double duration = 360;      // 旋转持续时间
  static double rotation_rate = 20;  // 旋转速度
  static double zixuan_yaw = 0.f;

  if (!yawCalculated) {
    first_yaw = current_yaw;
    start_time_ = ros::Time::now();
    yawCalculated = true;
  }

  if ((ros::Time::now() - start_time_).toSec() * rotation_rate <= duration) {
    double elapsed_time = (ros::Time::now() - start_time_).toSec();
    zixuan_yaw = elapsed_time * rotation_rate * M_PI / 180.0;

    Pub_px4_cmd(target.x, target.y, target.z, first_yaw + zixuan_yaw);
  } else {
    yawCalculated = false;
    ego_points_index++;  // 完成当前目标点
  }

  // ROS_INFO("time:%f", (ros::Time::now() - start_time_).toSec());
  ROS_INFO("target:%f", zixuan_yaw + first_yaw);
  // ROS_INFO("current yaw:%f", current_yaw);
}

// 读取ego里的位置速度加速度yaw和yaw-dot信息，其实只需要ego的位置速度和yaw就可以了
void Ctrl::twist_cb(
    const quadrotor_msgs::PositionCommand::ConstPtr& msg)  // ego的回调函数
{
  ego = *msg;
  ego_pos_x = ego.position.x;
  ego_pos_y = ego.position.y;
  ego_pos_z = ego.position.z;
  ego_vel_x = ego.velocity.x;
  ego_vel_y = ego.velocity.y;
  ego_vel_z = ego.velocity.z;
  ego_yaw = ego.yaw;
  ego_yaw_rate = ego.yaw_dot;
}

void Ctrl::control(const ros::TimerEvent&) {
  // ros::Rate rate(20.0);
  static bool offboard_mode_set = false;
  static bool armed = false;
  static double T = 20;  // 自旋速度
  static double zixuan_yaw;
  static double now_yaw;

  // ROS_INFO("current yaw:%f", current_yaw);
  //  在进入Offboard模式之前，必须已经启动了LocalPosPub_数据流，否则模式切换将被拒绝。
  //  这里的100可以被设置为任意数

  if (!offboard_mode_set) {
    for (int i = 100; ros::ok() && i > 0; --i) {
      local_pos_pub.publish(init_pos);
      ros::spinOnce();
      // rate.sleep();
    }
  }

  if (current_state.mode != "OFFBOARD" &&
      (ros::Time::now() - last_request > ros::Duration(0.1)) &&
      !offboard_mode_set) {
    // 客户端set_mode_client向服务端offb_set_mode发起请求call，然后服务端回应response将模式返回，这就打开了offboard模式
    if (set_mode_client.call(offb_set_mode) &&
        offb_set_mode.response.mode_sent) {
      // 切换到 OFFBOARD 模式
      ROS_INFO("OFFBOARD MODE");
      offboard_mode_set = true;
    } else
      ROS_INFO("OFFBOARD fail");
    last_request = ros::Time::now();
  }

  // 判断当前状态是否解锁，如果没有解锁，则进入if语句内部
  // 这里是5秒钟进行一次判断，避免飞控被大量的请求阻塞
  else if (!current_state.armed &&
           (ros::Time::now() - last_request > ros::Duration(0.1)) && !armed) {
    if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
      // 获取目标点
      Waypoint_get();
      ROS_INFO("lift off!");
      armed = true;
    } else
      ROS_INFO("ARM XXX");
    last_request = ros::Time::now();
  }

  if (armed) {
    ROS_INFO("ego point index:%d", ego_points_index + 1);
    switch (mission_flag) {
      case 0:
        // 不经过ego起飞
        Pub_px4_cmd(0.f, 0.f, 1.0f, current_yaw);
        ROS_INFO("TAKEOFF\n");

        if (abs(current_goal.position.z - position_z) < 0.1f) {
          ROS_INFO("Takeoff");
          mission_flag = 1;
        }

        break;

      case 1:
        // ego目标点序列
        executeEgoPoints();
        break;

      case 4:
        // 如果没有在rviz上打点，则保持当前系统
        Pub_px4_cmd(position_x, position_y, position_z, current_yaw);
        ROS_INFO("REMAIN POSITION\n");

        if ((ros::Time::now() - start_time_).toSec() > 3) mission_flag = 6;
        break;

      case 5:
        // 探索完后允许rviz给点追踪
        ROS_INFO("RVIZ POINT");
        Pub_ego_cmd();
        break;

      case 6:
        sendLandingCommand();
        if (landing_succelly_flag) mission_flag = 7;
        ROS_INFO("REMAIN POSITION----landing\n");
        break;

      case 7:
        // 探索完后允许rviz给点追踪
        ROS_INFO("land success");
        break;

      default:
        ROS_INFO("Unknown");
        break;
    }
  }
  ros::spinOnce();
  // rate.sleep();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "cxr_egoctrl_v1");
  setlocale(LC_ALL, "");

  ros::NodeHandle n;

  Ctrl ctrl;
  ros::spin();
  return 0;
}