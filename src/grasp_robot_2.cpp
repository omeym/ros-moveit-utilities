#include "robot_stack/robot_definitions.hpp"













int main(int argc, char** argv)
{
  
    ros::init(argc, argv,"robot_definitions_blue");
    ros::NodeHandle robot2_nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    
    robot_definitions Robot2("/iiwa7_Blue", robot2_nh);
    

    //These two values will be read in form of a ROS message
    //Eigen::Quaterniond robot2_quat(0.0182,-0.0114, 0.7088, 0.7051);
    //Eigen::Vector3d robot2_linear(1.4476,0.0768,0.2876);
    //Eigen::Vector3d robot2_linear(1.4476,0.0768,0.145);

    Eigen::Vector3d robot2_linear(1.4692,0.0651,0.139);                 //<-----------New Transformation after registering (09/19)
    //Eigen::Vector3d robot2_linear(1.48,0.0651,0.139);
    Eigen::Quaterniond robot2_quat(0.00778, -0.0037,0.7065,0.7077);      //<-----------New Transformation after registering (09/19)

    Eigen::Matrix4d Robot2_Transformation = Eigen::Matrix4d::Identity();
    Robot2_Transformation.block<3,3>(0,0) = robot2_quat.normalized().toRotationMatrix();
    Robot2_Transformation.block<3,1>(0,3) = robot2_linear.transpose();
    std::cout<<"Robot2 Transformations = \n"<<Robot2_Transformation<<std::endl;

    
    //These two values will be read in form of a ROS message
    Eigen::Quaterniond rb2_ee_quat(1,0,0,0);
    //Eigen::Vector3d rb2_ee_linear(0,0,0.165);

    //Eigen::Vector3d rb2_ee_linear(0, 0, 0.25);
    //Eigen::Vector3d rb2_ee_linear(0, 0, 0.2);
    //Eigen::Vector3d rb2_ee_linear(0, 0, 0.22);
    Eigen::Vector3d rb2_ee_linear(0, 0, 0.2);
    //Eigen::Vector3d rb2_ee_linear(0, 0, 0);
    

    Eigen::Matrix4d RB2_EE_Transformation = Eigen::Matrix4d::Identity();
    RB2_EE_Transformation.block<3,3>(0,0) = rb2_ee_quat.normalized().toRotationMatrix();
    RB2_EE_Transformation.block<3,1>(0,3) = rb2_ee_linear.transpose();
    std::cout<<"Robot2 EE Transformations = \n"<<RB2_EE_Transformation<<std::endl;
    
    
    
    Robot2.define_transformations(Robot2_Transformation, RB2_EE_Transformation);    

    const char* mold_filename = "package://gen_utilities/meshes/molds/BoeingMold_test4.dae";
    std::string object_name = "mold";
    Robot2.add_collision_object(object_name, mold_filename, Robot2_Transformation);

    ros::waitForShutdown();

    //ros::spin();
    //   while(ros::ok()){
    //      ros::spinOnce();
    //   }

}