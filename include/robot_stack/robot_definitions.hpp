//****************************************************************************************
//
// Author : Omey Mohan Manyar (manyar@usc.edu), University of Southern California
//
//****************************************************************************************



#ifndef ROBOT_DEFINITIONS_HPP
#define  ROBOT_DEFINITIONS_HPP

/***********************************************/
//STANDARD HEADERS
/************************************************/
#include <iostream>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <map>
#include <vector>
#include <string>
#include <functional>
#include <bits/stdc++.h>
#include <utility>
#include <float.h>
/*******************************************/
//ROS HEADERS
/********************************************/

//Standard ROS Headers
#include <ros/package.h>
#include <ros/console.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include <urdf/model.h>

//ROS MSGS
#include "robot_stack/RobotJointAngles.h"

//ROS Services
#include "robot_stack/GetLinkTransformation.h"
#include "robot_stack/GetEndEffectorPose.h"
#include "robot_stack/SolveRobotIK.h"
#include "robot_stack/GetManipulatorJacobian.h"
#include "robot_stack/GetManipulabilityIndex.h"
#include "robot_stack/PlanCartesianPath.h"

//ROS Move-it Headers for ROS Melodic
#include <pluginlib/class_loader.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/move_group_interface/move_group_interface.h> // needed for movegroupinterface https://ros-industrial.github.io/industrial_training/_source/session4/Motion-Planning-CPP.html
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <geometric_shapes/shape_operations.h>
//#include <moveit_visual_tools/moveit_visual_tools.h>



/****************************************************/
////////External Headers/////////////////////////
/***************************************************/
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <igl/readSTL.h>


class robot_definitions
{



public:
    //ROS Definitions
    
    ros::NodeHandle robot_def;              //ROS Node Handle
    
    //ROS Publishers and Subscribers
    ros::Subscriber joint_angles_sub;       //Subscriber Object for getting the joint angles
    ros::Publisher planning_scene_diff_publisher;


    //ROS Services
    ros::ServiceServer  link_transformations;   //Service server object to calculate the transformation a particular link 
    ros::ServiceServer  end_effector_pose;      //Service server object to return the transformation of end link
    ros::ServiceServer  robot_ik_solver;        //Service server object to return the IK and in turn the joint angles
    ros::ServiceServer  manipulator_jacobian;   //Service server object to return the Manipulator Jacobian
    ros::ServiceServer  manipulability_index;   //Service server object to return the Manipulability Index
    ros::ServiceServer  cartesian_path_planner; //Service server object to plan the robot path

    //Move-it Robot Model and Robot State Definitions
    std::string robot_description;
    //robot_model_loader::RobotModelLoader robot_model_loader_new{"/iiwa/robot_description"};
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_ptr;//{new robot_model_loader::RobotModelLoader("/iiwa/robot_description")};
    
    moveit::core::RobotModelPtr kinematic_model;// = robot_model_loader_new.getModel();
    moveit::core::RobotStatePtr kinematic_state;//{new moveit::core::RobotState{kinematic_model}};
    urdf::ModelInterfaceSharedPtr Robot_Model;

    std::vector<moveit_msgs::CollisionObject> Collision_Objects; //The Collision object message that adds a collision object in the world using the add collision object method
    moveit_msgs::Constraints path_constraints, goal_constraints;     //Defines end effector path and goal constraints
    geometry_msgs::PoseStamped ee_goal_pose;      //Defines the end effector goal and path pose
    geometry_msgs::QuaternionStamped ee_path_pose;
    
    
    
    //Move Group Parameters
    //moveit::planning_interface::MoveGroupInterface iiwa_move_group;//{"manipulator"};
    moveit::planning_interface::MoveGroupInterfacePtr iiwa_move_group_ptr;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface::Plan iiwa_cartesian_plan;
    moveit_msgs::OrientationConstraint goal_pose_constraint;
    
    const moveit::core::JointModelGroup* joint_model_group;
    std::vector<std::string> joint_names;
    std::vector<std::string> link_names;


    bool plan_custom_path_method(const std::string& planner_id, Eigen::Matrix4d Start_Point, Eigen::Matrix4d End_Point);
    
    bool plan_cartesian_path_method(Eigen::Matrix4d Start_Point, Eigen::Matrix4d End_Point, int point_id, bool transition_pose_flag = true, bool tcp_frame_flag = true);
    
    moveit_msgs::RobotTrajectory cartesian_trajectory;
    moveit_msgs::RobotTrajectory custom_trajectory;
    Eigen::MatrixXd Robot_Cartesian_Trajectory;
    Eigen::MatrixXd Robot_Custom_Trajectory;

    Eigen::MatrixXd moveit2Eigen_Robot_Trajectory(moveit_msgs::RobotTrajectory moveit_trajectory);
    geometry_msgs::Pose Eigen_2_GeomPose(Eigen::Matrix4d Eigen_Pose);
    Eigen::Matrix4d GeomPose_2_Eigen(geometry_msgs::Pose geom_pose);
    void define_cartesian_constraints(Eigen::Quaterniond pose_constraint);
    int collision_obj_count = 0;                    //Keeps a count of number of objects added
    std::vector<geometry_msgs::Pose> waypoints;
    
    
    void savePath_Joint_Angles(moveit_msgs::RobotTrajectory& computed_path, int id);
    void savePath_Waypoints(moveit_msgs::RobotTrajectory& computed_path, int id);
    //Planning Scene Methods
    //void initialize_planning_scene();               //This method intializes the planning scene
    //void define_constraints();                      //This method initializes the path and goal constraints
    
    
    //Robot Attributes
    std::vector<double> joint_values;
    std::string robot_name;

    //This Method defines the Robot Transformations and EE Transformation in World Frame i.e. wrt the mold
    void define_transformations(Eigen::Matrix4d Current_Robot_Transformation, Eigen::Matrix4d Current_EE_Transformation);
   
    
    Eigen::Matrix4d Robot_Transformation = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d EE_Transformation = Eigen::Matrix4d::Identity();

    //Methods
    void setJointAngles(robot_stack::RobotJointAngles joint_angles);
    void setJointAnglesMethod(std::vector<double> joint_angles);
    void loadSTLFile(const char* filename, shape_msgs::Mesh& Obj_Mesh);
    std::vector<double> getJointAngles(Eigen::Isometry3d end_effector_state);
    Eigen::Isometry3d getFK(int link_number, std::vector<double> joint_angles);
    Eigen::MatrixXd getJacobianMethod(Eigen::Isometry3d EE_Pose);
    Eigen::MatrixXd getJacobianMethod();
    
//public:
    //Constructor
    robot_definitions(std::string robot_description, ros::NodeHandle& nh);
    //~robot_definitions();
    void add_collision_object(std::string object_name, const char* filepath, Eigen::Matrix4d Object_Pose = Eigen::Matrix4d::Identity());   //Adds a Collision Object to the Environment
    
    Eigen::Matrix4d ee_2_flange(Eigen::Matrix4d robot_pose);        //This is a work around method to define points via flange frame, make sure to add the end effector as an attached collision object
    Eigen::Matrix4d flange_2_ee(Eigen::Matrix4d robot_pose);
    Eigen::Matrix4d ee_2_flange(geometry_msgs::Pose robot_pose);
    Eigen::Matrix4d flange_2_ee(geometry_msgs::Pose robot_pose);

    bool getLinkFK(robot_stack::GetLinkTransformation::Request &req,
                   robot_stack::GetLinkTransformation::Response &res);          //Callback Methods to get the FK of a certain link
    bool EndEffectorPose(robot_stack::GetEndEffectorPose::Request &req,
                        robot_stack::GetEndEffectorPose::Response &res);        //Service call back Method to get the End Effector Pose
    bool solveiiwaIK(robot_stack::SolveRobotIK::Request &req,
                     robot_stack::SolveRobotIK::Response &res);                 //Service call back Method to solve FK and get Joint Angles 
    bool getRobotJacobian(robot_stack::GetManipulatorJacobian::Request &req,
                     robot_stack::GetManipulatorJacobian::Response &res);       //Service call back Method to get the Manipulator Jacobian

    bool getManipulabilityIndex(robot_stack::GetManipulabilityIndex::Request &req,
                                robot_stack::GetManipulabilityIndex::Response &res);       //Service call back Method to get the Manipulability Index of Manipualtor
    
    bool planCartesianPath(robot_stack::PlanCartesianPath::Request &req,
                   robot_stack::PlanCartesianPath::Response &res);          //Callback Methods to get the FK of a certain link

    inline ros::NodeHandle getNodeHandle(){return robot_def;}
    inline std::string getRobotName(){return robot_name;}
    double ComputeManipulabilityIndex(std::vector<double> pose);                 
};


#endif