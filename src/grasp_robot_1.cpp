#include "robot_stack/robot_definitions.hpp"








int main(int argc, char** argv)
{
  
    ros::init(argc, argv,"robot_definitions_green");
    ros::NodeHandle robot1_nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ROS_INFO("Defining Robot 1");
    robot_definitions Robot1("/iiwa7_Green", robot1_nh);
    
    //These two values will be read in form of a ROS message
    //Eigen::Quaterniond robot1_quat(0.7089,0.7036,-0.0343, 0.035);
    //Eigen::Vector3d robot1_linear(0.419741,-0.17507,0.240281);

    //Changing this to reflect new position
    //Eigen::Vector3d robot1_linear(0.419741,-0.0768,0.240281);
    //Eigen::Vector3d robot1_linear(0.419741,-0.0768,0.2876);
    //Eigen::Vector3d robot1_linear(0.44,-0.0768,0.145);

    Eigen::Vector3d robot1_linear(0.43,-0.053,0.139);                  //<-----------New Transformation after registering (09/19)
    //Eigen::Vector3d robot1_linear(0.45,-0.053,0.139);
    Eigen::Quaterniond robot1_quat(0.706, 0.708,0.0078,0.0121);


    Eigen::Matrix4d Robot1_Transformation = Eigen::Matrix4d::Identity();
    Robot1_Transformation.block<3,3>(0,0) = robot1_quat.normalized().toRotationMatrix();
    Robot1_Transformation.block<3,1>(0,3) = robot1_linear.transpose();    
    std::cout<<"Robot1 Transformations = \n"<<Robot1_Transformation<<std::endl;

    
    //These two values will be read in form of a ROS message
    Eigen::Quaterniond rb1_ee_quat(1,0,0,0);
    //Eigen::Vector3d rb1_ee_linear(0, 0, 0.165);

    //Changing this to reflect new roller grippers
    //Eigen::Vector3d rb1_ee_linear(0, 0, 0.25);
    //Eigen::Vector3d rb1_ee_linear(0, 0, 0.2);
    //Eigen::Vector3d rb1_ee_linear(0, 0, 0.22);
    Eigen::Vector3d rb1_ee_linear(0, 0, 0.2);
    //Eigen::Vector3d rb1_ee_linear(0, 0, 0);

    Eigen::Matrix4d RB1_EE_Transformation = Eigen::Matrix4d::Identity();
    RB1_EE_Transformation.block<3,3>(0,0) = rb1_ee_quat.normalized().toRotationMatrix();
    RB1_EE_Transformation.block<3,1>(0,3) = rb1_ee_linear.transpose();
    std::cout<<"Robot1 EE Transformations = \n"<<RB1_EE_Transformation<<std::endl;

    Robot1.define_transformations(Robot1_Transformation, RB1_EE_Transformation);


    const char* mold_filename = "package://gen_utilities/meshes/molds/BoeingMold_test4.dae";
    std::string object_name = "mold";
    Robot1.add_collision_object(object_name, mold_filename, Robot1_Transformation);

    // Eigen::Matrix4d start_point = Eigen::Matrix4d::Identity();
    // Eigen::Vector3d start_position(0.385, -0.293699, 0.934134);
    // start_point.block<3,1>(0,3) = start_position.transpose();
    // start_point.block<3,3>(0,0) = Robot1_Transformation.block<3,3>(0,0);

    // Eigen::Matrix4d end_point = Eigen::Matrix4d::Identity();
    // Eigen::Vector3d end_position(0.388, -0.425, 0.82);
    // end_point.block<3,1>(0,3) = end_position.transpose();
    // end_point.block<3,3>(0,0) = Robot1_Transformation.block<3,3>(0,0);
    // std::cout<<"Added Object Planning"<<std::endl;  
    // Robot1.plan_cartesian_path_method(start_point, end_point, 1, true);
    
    // start_position = Eigen::Vector3d(0.388, -0.425, 0.82);
    // start_point.block<3,1>(0,3) = start_position.transpose();
    // start_point.block<3,3>(0,0) = Robot1_Transformation.block<3,3>(0,0);

    // end_position = Eigen::Vector3d(0.37, -0.5, 0.79);
    // end_point.block<3,1>(0,3) = end_position.transpose();
    // end_point.block<3,3>(0,0) = Robot1_Transformation.block<3,3>(0,0);
    
    // Robot1.plan_cartesian_path_method(start_point, end_point, 2, true);
    
    ros::waitForShutdown();
    //   while(ros::ok()){
    //      ros::spinOnce();
    //   }

}