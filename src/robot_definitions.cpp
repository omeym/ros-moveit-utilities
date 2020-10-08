//****************************************************************************************
//
// Author : Omey Mohan Manyar (manyar@usc.edu), University of Southern California
//
//****************************************************************************************



#include "robot_stack/robot_definitions.hpp"


robot_definitions::robot_definitions(std::string robot_name, ros::NodeHandle& nh)
{   this->robot_description = robot_name + "/robot_description";
    this->robot_def = nh;
    robot_model_loader_ptr = std::make_shared<robot_model_loader::RobotModelLoader>(robot_description);
    kinematic_model = robot_model_loader_ptr->getModel();
    kinematic_state = std::make_shared<moveit::core::RobotState>(kinematic_model);
    Robot_Model = robot_model_loader_ptr->getURDF();
    std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface::Options Opt{PLANNING_GROUP, robot_description, ros::NodeHandle(robot_name)};
    
    iiwa_move_group_ptr = std::make_shared<moveit::planning_interface::MoveGroupInterface>(Opt);
    joint_model_group = iiwa_move_group_ptr->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    joint_names = joint_model_group->getVariableNames();
    link_names = joint_model_group->getLinkModelNames();
    
    kinematic_state->setToDefaultValues();
    kinematic_state->enforceBounds();
    //kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    
    this->robot_name = robot_name;    
    
    //Service and Publisher/Subscriber Definitions
    joint_angles_sub = robot_def.subscribe("/robot_stack"+this->robot_name+"/joint_angles", 1000, &robot_definitions::setJointAngles, this);
    link_transformations = robot_def.advertiseService("robot_stack"+this->robot_name+"/link_transformation", &robot_definitions::getLinkFK, this);
    end_effector_pose = robot_def.advertiseService("/robot_stack"+this->robot_name+"/end_effector_pose",&robot_definitions::EndEffectorPose, this);
    robot_ik_solver =   robot_def.advertiseService("/robot_stack"+this->robot_name+"/robot_ik", &robot_definitions::solveiiwaIK, this);
    manipulator_jacobian = robot_def.advertiseService("/robot_stack"+this->robot_name+"/jacobian", &robot_definitions::getRobotJacobian, this);
    manipulability_index = robot_def.advertiseService("/robot_stack"+this->robot_name+"/manipulability_index", &robot_definitions::getManipulabilityIndex, this);
    cartesian_path_planner = robot_def.advertiseService("/robot_stack"+this->robot_name+"/plan_cartesian_path", &robot_definitions::planCartesianPath, this); 
}


void robot_definitions::define_transformations(Eigen::Matrix4d Current_Robot_Transformation, Eigen::Matrix4d Current_EE_Transformation){
    this->Robot_Transformation = Current_Robot_Transformation;
    this->EE_Transformation = Current_EE_Transformation;
    
    Eigen::Matrix4d Changed_Orientation = Eigen::Matrix4d::Identity();
    Changed_Orientation.block<3,3>(0,0) = Current_Robot_Transformation.block<3,3>(0,0);
    EE_Transformation = Changed_Orientation * EE_Transformation;        //Converting TCP to Robot Frame in the World Frame
    EE_Transformation.block<3,3>(0,0) = Eigen::Matrix3d::Identity();    //Converting TCP Orientation back to robot frame
    std::cout<<"Updated EE Transformations = "<<EE_Transformation<<std::endl;
}


Eigen::Matrix4d robot_definitions::ee_2_flange(Eigen::Matrix4d robot_pose){

    
    Eigen::Matrix4d updated_pose = EE_Transformation.inverse()*robot_pose;
    return updated_pose;    
  
}

Eigen::Matrix4d robot_definitions::flange_2_ee(Eigen::Matrix4d robot_pose){
    
    Eigen::Matrix4d updated_pose = EE_Transformation * robot_pose;
    return updated_pose;
}


Eigen::Matrix4d robot_definitions::ee_2_flange(geometry_msgs::Pose robot_pose){

    Eigen::Matrix4d robot_pose_eig = GeomPose_2_Eigen(robot_pose);
    Eigen::Matrix4d updated_pose = EE_Transformation.inverse()*robot_pose_eig;
    return updated_pose;    
  
}

Eigen::Matrix4d robot_definitions::flange_2_ee(geometry_msgs::Pose robot_pose){
    Eigen::Matrix4d robot_pose_eig = GeomPose_2_Eigen(robot_pose);
    Eigen::Matrix4d updated_pose = EE_Transformation * robot_pose_eig;
    return updated_pose;
}



void robot_definitions::setJointAngles(robot_stack::RobotJointAngles joint_angles){
    
    joint_values.clear();
    //Defining Joint angles
    joint_values.emplace_back(joint_angles.a1);
    joint_values.emplace_back(joint_angles.a2);
    joint_values.emplace_back(joint_angles.a3);
    joint_values.emplace_back(joint_angles.a4);
    joint_values.emplace_back(joint_angles.a5);
    joint_values.emplace_back(joint_angles.a6);
    joint_values.emplace_back(joint_angles.a7);

    //Update the kinematic state
    if(joint_values.size()!=7){
        joint_values.assign(7,0);           //Default position of {0,0,0,0,0,0,0} joint angles
    }
    kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
    //Enforcing Joint Limits
    if(!kinematic_state->satisfiesBounds()){
        kinematic_state->enforceBounds();
    }
    std::cout << "Returning from Joint Angles Callback" << std::endl;

}

//Usign Custom STL function because shape messages was giving an error reading the stl file
void robot_definitions::loadSTLFile(const char* filename, shape_msgs::Mesh& Obj_Mesh)
	{
		Eigen::MatrixXd v, f, n;
		std::string _filename = filename;
		igl::readSTL(_filename, v, f, n);
		int vc = v.cols();
		int fc = f.cols();
		long int vr = v.rows();
		long int fr = f.rows();
		if ((vc != 3)||(fc != 3)) 
		{
			std::cerr<<"this STL file has wrong format!"<<std::endl;
			return;
		}else
		{
			int tri[3];
            Obj_Mesh.vertices.resize(vr);
			for(long int i = 0; i < vr; ++i)
			{   
				Obj_Mesh.vertices[i].x = v(i, 0);
                Obj_Mesh.vertices[i].y = v(i, 1);
                Obj_Mesh.vertices[i].z = v(i, 2);
			}
            Obj_Mesh.triangles.resize(fr);
			for(long int j = 0; j < fr; ++j)
			{
				tri[0] = f(j, 0);
				tri[1] = f(j, 1);
				tri[2] = f(j, 2);
				Obj_Mesh.triangles[j].vertex_indices[0] = tri[0];
                Obj_Mesh.triangles[j].vertex_indices[1] = tri[1];
                Obj_Mesh.triangles[j].vertex_indices[2] = tri[2];
			}
		}
	}



void robot_definitions::define_cartesian_constraints(Eigen::Quaterniond pose_constraint){
    //Defining Path Constraints
    goal_pose_constraint.link_name = "iiwa_link_6";
    goal_pose_constraint.header.frame_id = "iiwa_link_0";
    goal_pose_constraint.orientation.w = pose_constraint.w();
    goal_pose_constraint.orientation.x = pose_constraint.x();
    goal_pose_constraint.orientation.y = pose_constraint.y();
    goal_pose_constraint.orientation.z = pose_constraint.z();
    goal_pose_constraint.absolute_x_axis_tolerance = 1.57;
    goal_pose_constraint.absolute_y_axis_tolerance = 0.1;
    goal_pose_constraint.absolute_z_axis_tolerance = 0.1;
    goal_pose_constraint.weight = 1.0;


    path_constraints.orientation_constraints.push_back(goal_pose_constraint);
    iiwa_move_group_ptr->setPathConstraints(path_constraints);

    iiwa_move_group_ptr->setGoalOrientationTolerance(0.1);
    iiwa_move_group_ptr->setGoalPositionTolerance(0.01);
    path_constraints.orientation_constraints.clear();



}

geometry_msgs::Pose robot_definitions::Eigen_2_GeomPose(Eigen::Matrix4d Eigen_Pose){
    
    geometry_msgs::Pose requested_pose;
    Eigen::Quaterniond start_orientation(Eigen_Pose.block<3,3>(0,0));
    requested_pose.orientation.w = start_orientation.w();
    requested_pose.orientation.x = start_orientation.x();
    requested_pose.orientation.y = start_orientation.y();
    requested_pose.orientation.z = start_orientation.z();

    requested_pose.position.x = Eigen_Pose(0,3);
    requested_pose.position.y = Eigen_Pose(1,3);
    requested_pose.position.z = Eigen_Pose(2,3);

    return requested_pose;
}

Eigen::Matrix4d robot_definitions::GeomPose_2_Eigen(geometry_msgs::Pose geom_pose){
    Eigen::Matrix4d result_pose = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond point_orientation;
    
    result_pose(0,3) = geom_pose.position.x;
    result_pose(1,3) = geom_pose.position.y;
    result_pose(2,3) = geom_pose.position.z;

    point_orientation.w() = geom_pose.orientation.w;
    point_orientation.x() = geom_pose.orientation.x;
    point_orientation.y() = geom_pose.orientation.y;
    point_orientation.z() = geom_pose.orientation.z;

    result_pose.block<3,3>(0,0) = point_orientation.normalized().toRotationMatrix();

    return result_pose;



}

void robot_definitions::savePath_Joint_Angles(moveit_msgs::RobotTrajectory& computed_path, int id){

    std::string rb_indicator = this->robot_name;
    rb_indicator.erase(rb_indicator.begin() + 0);      //Removing the "/" from robot name
    std::string gp_file_path = ros::package::getPath("robot_stack") + "/data/"+rb_indicator+"_joint_angles.csv";
	std::fstream grasping_points;
	grasping_points.open(gp_file_path, std::ios::out | std::ios::app);



    std::cout<<"Size of WayPoints ="<<computed_path.joint_trajectory.points.size()<<std::endl;
    for(int i=0; i<computed_path.joint_trajectory.points.size(); i++){
        
        grasping_points<<id<<","<<computed_path.joint_trajectory.points[i].positions[0]<<","<<computed_path.joint_trajectory.points[i].positions[1]<<","<<computed_path.joint_trajectory.points[i].positions[2]<<","<<computed_path.joint_trajectory.points[i].positions[3]<<","<<computed_path.joint_trajectory.points[i].positions[4]<<","<<computed_path.joint_trajectory.points[i].positions[5]<<","<<computed_path.joint_trajectory.points[i].positions[6]<<"\n";
		
    }
    ROS_INFO("Saved the Current Joint Angles");
	

}

void robot_definitions::savePath_Waypoints(moveit_msgs::RobotTrajectory& computed_path, int id){

    std::string rb_indicator = this->robot_name;
    rb_indicator.erase(rb_indicator.begin() + 0);      //Removing the "/" from robot name
    std::string gp_file_path = ros::package::getPath("robot_stack") + "/data/"+rb_indicator+"_waypoints.csv";
	std::fstream grasping_points;
	grasping_points.open(gp_file_path, std::ios::out | std::ios::app);

    std::vector<double> current_joint_values;
    Eigen::Matrix4d robot_pose_eigen;
    Eigen::Isometry3d robot_transformation;
    Eigen::Vector3d Euler_Angles;
    Eigen::Matrix3d temp_rot_mat;
    robot_state::RobotStatePtr temp_start_state;

    std::cout<<"Size of WayPoints ="<<computed_path.joint_trajectory.points.size()<<std::endl;
    for(int i=0; i<computed_path.joint_trajectory.points.size(); i++){
        current_joint_values.clear();

        current_joint_values.emplace_back(computed_path.joint_trajectory.points[i].positions[0]);
        current_joint_values.emplace_back(computed_path.joint_trajectory.points[i].positions[1]);
        current_joint_values.emplace_back(computed_path.joint_trajectory.points[i].positions[2]);
        current_joint_values.emplace_back(computed_path.joint_trajectory.points[i].positions[3]);
        current_joint_values.emplace_back(computed_path.joint_trajectory.points[i].positions[4]);
        current_joint_values.emplace_back(computed_path.joint_trajectory.points[i].positions[5]);
        current_joint_values.emplace_back(computed_path.joint_trajectory.points[i].positions[6]);
        
        temp_start_state = iiwa_move_group_ptr->getCurrentState();
        temp_start_state->setJointGroupPositions(joint_model_group, current_joint_values); 
        robot_transformation = temp_start_state->getGlobalLinkTransform("iiwa_link_6");
        temp_rot_mat = robot_transformation.matrix().block<3,3>(0,0);
        Euler_Angles = temp_rot_mat.eulerAngles(0,1,2);
        robot_pose_eigen = robot_transformation.matrix();
        robot_pose_eigen = flange_2_ee(robot_pose_eigen);

        //Storing X,Y,Z,A,B,C (ZYX Euler Angles)
        grasping_points<<id<<","<<robot_pose_eigen(0,3)<<","<<robot_pose_eigen(1,3)<<","<<robot_pose_eigen(2,3)<<","<<Euler_Angles(2)<<","<<Euler_Angles(1)<<","<<Euler_Angles(0)<<"\n";
    }
	
    ROS_INFO("Saved the Current Waypoints");

}




bool robot_definitions::plan_cartesian_path_method(Eigen::Matrix4d Start_Point, Eigen::Matrix4d End_Point, int point_id, bool transition_pose_flag, bool tcp_frame_flag){

    geometry_msgs::Pose start_pose;
    Eigen::Quaterniond start_orientation(Start_Point.block<3,3>(0,0));
    waypoints.clear();

    //Checking if the Point is in Robot TCP Frame or in Flange Frame
    if(tcp_frame_flag){
        Start_Point = ee_2_flange(Start_Point);
        End_Point = ee_2_flange(End_Point);
    }

  
    
    std::vector<double> current_joint_angles;
    //Setting the Start Point of the Robot
    Eigen::Isometry3d start_state_transform;
    start_state_transform.matrix().block<3,3>(0,0) = Start_Point.block<3,3>(0,0);
    start_state_transform.matrix().block<3,1>(0,3) = Start_Point.block<3,1>(0,3);
    

    robot_state::RobotState start_state(*iiwa_move_group_ptr->getCurrentState());
    current_joint_angles = iiwa_move_group_ptr->getCurrentJointValues();
    std::cout<<"The current joint values from move group  = "<<std::endl;

    for(auto itr:current_joint_angles){
        std::cout<<itr<<std::endl;
    }

    double timeout = 1.0;
    start_state.setFromIK(joint_model_group, start_state_transform, timeout);   
    iiwa_move_group_ptr->setJointValueTarget(start_state_transform);    //Setting the move group to starting position
    current_joint_angles = {0.799,0.9255,0.909667,1.37,0.116,-0.8,-0.86};
    iiwa_move_group_ptr->getCurrentState()->setJointGroupPositions(joint_model_group, current_joint_angles);
  

    iiwa_move_group_ptr->setPlanningTime(10);

    start_pose = this->Eigen_2_GeomPose(Start_Point);




    //when we receive the first trajectory, take the robot to start point first
    if(point_id == 1){
        
        iiwa_move_group_ptr->setPoseTarget(start_pose);
        //Using RRT to 
        bool success = (iiwa_move_group_ptr->plan(iiwa_cartesian_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO("Visualizing plan for current to start point (pose goal) %s", success ? "" : "FAILED");
        iiwa_move_group_ptr->execute(iiwa_cartesian_plan.trajectory_);
        current_joint_angles.clear();
        current_joint_angles = iiwa_move_group_ptr->getCurrentJointValues();


        std::cout<<"The current joint values after executing the first point trajectpry= "<<std::endl;

        for(auto itr:current_joint_angles){
            std::cout<<itr<<std::endl;
        }
    }

    //Adding the start point in the waypoints container
    waypoints.push_back(start_pose);

    //Defining the transition point
    if(transition_pose_flag){

        geometry_msgs::Pose transition_pose;
        Eigen::Quaterniond transition_orientation(Start_Point.block<3,3>(0,0));
        transition_pose.orientation.w = transition_orientation.w();
        transition_pose.orientation.x = transition_orientation.x();
        transition_pose.orientation.y = transition_orientation.y();
        transition_pose.orientation.z = transition_orientation.z();

        transition_pose.position.x = End_Point(0,3);            //Making sure the robot is aligned in x position on the grasping location
        transition_pose.position.y = Start_Point(1,3);
        transition_pose.position.z = Start_Point(2,3);
        waypoints.push_back(transition_pose);
    }

    //Defining Path Constraints
    this->define_cartesian_constraints(start_orientation);
    
    //Planning from start to transition point
    const double jump_threshold = 0.0;
    const double eef_step = 0.001;

    double fraction = iiwa_move_group_ptr->computeCartesianPath(waypoints, eef_step, jump_threshold, cartesian_trajectory);

    std::cout<<"Size of WayPoints ="<<cartesian_trajectory.joint_trajectory.points.size()<<std::endl;
    ROS_INFO("Visualizing plan from Start to Transition (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
    
    //Saving in this manner so that the trajectory is split in two parts for grippers to function
    if(fraction>0.1){
        //This if condition is required to ensure that Region 1 path has ID's 1 & 2 for 2 splines
        savePath_Joint_Angles(cartesian_trajectory, ((2*point_id)-1));
        savePath_Waypoints(cartesian_trajectory, ((2*point_id)-1));
        
        iiwa_move_group_ptr->execute(cartesian_trajectory);
        current_joint_angles.clear();
        current_joint_angles = iiwa_move_group_ptr->getCurrentJointValues();


        std::cout<<"The current joint values after executing the trajectpry= "<<std::endl;

        for(auto itr:current_joint_angles){
            std::cout<<itr<<std::endl;
        }

    }

    waypoints.clear();
    geometry_msgs::Pose target_pose;
    target_pose = this->Eigen_2_GeomPose(End_Point);
    waypoints.push_back(target_pose);


    fraction = iiwa_move_group_ptr->computeCartesianPath(waypoints, eef_step, jump_threshold, cartesian_trajectory);

    std::cout<<"Size of WayPoints ="<<cartesian_trajectory.joint_trajectory.points.size()<<std::endl;
    ROS_INFO("Visualizing plan from Transition to End Point (Cartesian path) (%.2f%% acheived)", fraction * 100.0);


    //Saving in this manner so that the trajectory is split in two parts for grippers to function
    if(fraction>0.1){
        //This if condition is required to ensure that Region 1 path has ID's 1 & 2 for 2 splines
    
        savePath_Joint_Angles(cartesian_trajectory, (2*point_id));
        savePath_Waypoints(cartesian_trajectory, (2*point_id));
        
        
        iiwa_move_group_ptr->execute(cartesian_trajectory);
        current_joint_angles.clear();
        current_joint_angles = iiwa_move_group_ptr->getCurrentJointValues();


        std::cout<<"The current joint values after executing the trajectpry= "<<std::endl;

        for(auto itr:current_joint_angles){
            std::cout<<itr<<std::endl;
        }
        iiwa_move_group_ptr->clearPathConstraints();

        return true;

    }

    else{
        return false;
    }
    

}

bool robot_definitions::planCartesianPath(robot_stack::PlanCartesianPath::Request &req, robot_stack::PlanCartesianPath::Response &res){
    
    Eigen::Matrix4d Start_Point = GeomPose_2_Eigen(req.Start_Point);
    Eigen::Matrix4d End_Point = GeomPose_2_Eigen(req.End_Point);

    std::cout<<"The Start Point is = "<<Start_Point<<std::endl;
    std::cout<<"The End Point is = "<<End_Point<<std::endl;

    if(this->plan_cartesian_path_method(Start_Point, End_Point, req.point_id.data)){
        res.cartesian_trajectory = this->cartesian_trajectory;
    }

    else{
        res.cartesian_trajectory = this->cartesian_trajectory;
        ROS_INFO("Plan Cartesian Path Service Failed");
    }

    
}


///////////////////////////Under Construction//////////////////////////////////////
Eigen::MatrixXd robot_definitions::moveit2Eigen_Robot_Trajectory(moveit_msgs::RobotTrajectory moveit_trajectory){
    Eigen::MatrixXd robot_trajectory;
    robot_trajectory.resize(1,7);



}


bool robot_definitions::plan_custom_path_method(const std::string& planner_id, Eigen::Matrix4d Start_Point, Eigen::Matrix4d End_Point){
    
    
    Eigen::Isometry3d Start_Pose;
    Start_Pose.matrix().block<3,3>(0,0) = Start_Point.block<3,3>(0,0);
    Start_Pose.matrix().block<3,1>(0,3) = Start_Point.block<3,1>(0,3);

    std::vector<double> joint_angles = this->getJointAngles(Start_Pose);

    iiwa_move_group_ptr->setJointValueTarget(joint_angles);     //Setting the move group to starting position

    robot_state::RobotState start_state(*iiwa_move_group_ptr->getCurrentState());
    geometry_msgs::Pose start_pose;
    start_pose = this->Eigen_2_GeomPose(Start_Point);

    start_state.setFromIK(joint_model_group, start_pose);
    iiwa_move_group_ptr->setStartState(start_state);

    geometry_msgs::Pose target_pose;
    target_pose = this->Eigen_2_GeomPose(End_Point);
    iiwa_move_group_ptr->setPoseTarget(target_pose);

    iiwa_move_group_ptr->setPlannerId(planner_id);    
    iiwa_move_group_ptr->setPlanningTime(10);        //In Seconds
    

    bool success = (iiwa_move_group_ptr->plan(iiwa_cartesian_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    if(success){
        custom_trajectory = iiwa_cartesian_plan.trajectory_;

    }


    return success;


}

void robot_definitions::add_collision_object(std::string object_name, const char* filepath, Eigen::Matrix4d Object_Pose){
    moveit_msgs::CollisionObject new_object;
    shapes::Mesh* obj_mesh = shapes::createMeshFromResource(filepath);
    shape_msgs::Mesh obj_shape_mesh;
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(obj_mesh, mesh_msg);
    obj_shape_mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
    new_object.id = object_name;
    new_object.header.frame_id = iiwa_move_group_ptr->getPlanningFrame();
    new_object.meshes.resize(collision_obj_count+1);
    new_object.meshes[collision_obj_count] = obj_shape_mesh;
    //Define the object pose too
    Eigen::Quaterniond object_orientation(Object_Pose.block<3,3>(0,0));

    new_object.mesh_poses.resize(collision_obj_count+1);
    new_object.mesh_poses[collision_obj_count].orientation.w = object_orientation.w();
    new_object.mesh_poses[collision_obj_count].orientation.x = object_orientation.x();
    new_object.mesh_poses[collision_obj_count].orientation.y = object_orientation.y();
    new_object.mesh_poses[collision_obj_count].orientation.z = object_orientation.z();

    new_object.mesh_poses[collision_obj_count].position.x = Object_Pose(3,0);
    new_object.mesh_poses[collision_obj_count].position.y = Object_Pose(3,1);
    new_object.mesh_poses[collision_obj_count].position.z = Object_Pose(3,2);

    new_object.operation = new_object.ADD;
    Collision_Objects.push_back(new_object);
    planning_scene_interface.addCollisionObjects(Collision_Objects);


    //planning_scene_msg.world.collision_objects.push_back(Collision_Objects);
    //planning_scene_msg.is_diff = true;
    //planning_scene_diff_publisher.publish(planning_scene_msg);
    collision_obj_count++;
    //this->initialize_planning_scene();

}



bool robot_definitions::getLinkFK(robot_stack::GetLinkTransformation::Request &req, robot_stack::GetLinkTransformation::Response &res)
{
    
    const Eigen::Isometry3d& link_state = kinematic_state->getGlobalLinkTransform(link_names[req.link-1]);
    std::cout << "FK Requested for Link No. = "<<req.link << std::endl;
    //Generating the response
    Eigen::Vector3d linear_mat = link_state.matrix().block<3,1>(0,3);
    Eigen::Quaterniond Link_quaternion(link_state.matrix().block<3,3>(0,0));
    
    //Translation part assignment
    res.Link_Transformation.translation.x = linear_mat(0);
    res.Link_Transformation.translation.y = linear_mat(1);
    res.Link_Transformation.translation.z = linear_mat(2);
    //Rotation Part Assignment
    res.Link_Transformation.rotation.x = Link_quaternion.x();
    res.Link_Transformation.rotation.y = Link_quaternion.y();
    res.Link_Transformation.rotation.z = Link_quaternion.z();
    res.Link_Transformation.rotation.w = Link_quaternion.w();

    return true;
}

bool robot_definitions::solveiiwaIK(robot_stack::SolveRobotIK::Request &req,
                                    robot_stack::SolveRobotIK::Response &res)
{
    
    double timeout = 2;
    Eigen::Isometry3d EE_Transformation = Eigen::Isometry3d::Identity();
    
    //Generating the rotation and linear part from the requested data
    Eigen::Vector3d linear_mat(req.end_effector_pose.translation.x, req.end_effector_pose.translation.y, req.end_effector_pose.translation.z);
    Eigen::Quaterniond Link_quaternion(req.end_effector_pose.rotation.w, req.end_effector_pose.rotation.x, req.end_effector_pose.rotation.y, req.end_effector_pose.rotation.z);
    
    //Creating an affine transformation pose of end effector
    EE_Transformation.matrix().block<3,3>(0,0) = Link_quaternion.normalized().toRotationMatrix();
    EE_Transformation.matrix().block<3,1>(0,3) = linear_mat.transpose();
    std::cout << "In the IK Routine" << std::endl;
    //Move-it IK Routine
    bool found_ik = kinematic_state->setFromIK(joint_model_group, EE_Transformation, timeout);
    std::vector<double> temp_joint_angles;

   
    if (found_ik)
    {
        kinematic_state->copyJointGroupPositions(joint_model_group, temp_joint_angles);
        
        ROS_INFO("IK Found Returning the Joint Values for Robot %s",this->robot_name.c_str());

        res.Joint_Angles.a1 = temp_joint_angles[0];
        res.Joint_Angles.a2 = temp_joint_angles[1];
        res.Joint_Angles.a3 = temp_joint_angles[2];
        res.Joint_Angles.a4 = temp_joint_angles[3];
        res.Joint_Angles.a5 = temp_joint_angles[4];
        res.Joint_Angles.a6 = temp_joint_angles[5];
        res.Joint_Angles.a7 = temp_joint_angles[6];

        return true;
    }
    else
    {
        ROS_INFO("Did not find IK solution for robot %s", this->robot_name.c_str());
        return false;
    }
}

bool robot_definitions::EndEffectorPose(robot_stack::GetEndEffectorPose::Request &req,
                                       robot_stack::GetEndEffectorPose::Response &res)
{   
    //Checks whether Joint angles are being sent or not
    if(req.service_flag){
        joint_values.clear();
        joint_values.emplace_back(req.Joint_Angles_N.a1);
        joint_values.emplace_back(req.Joint_Angles_N.a2);
        joint_values.emplace_back(req.Joint_Angles_N.a3);
        joint_values.emplace_back(req.Joint_Angles_N.a4);
        joint_values.emplace_back(req.Joint_Angles_N.a5);
        joint_values.emplace_back(req.Joint_Angles_N.a6);
        joint_values.emplace_back(req.Joint_Angles_N.a7);
        this->setJointAnglesMethod(joint_values);
    }
    std::cout << "EndEffector Pose Service Requested, End Effector = "<< link_names[6]<< std::endl;
    const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform(link_names[6]);

    //Generating the response
    Eigen::Vector3d linear_mat = end_effector_state.translation();
    Eigen::Quaterniond Link_quaternion(end_effector_state.rotation());
 
    //Translation part assignment
    res.End_Effector_Transformation.translation.x = linear_mat(0);
    res.End_Effector_Transformation.translation.y = linear_mat(1);
    res.End_Effector_Transformation.translation.z = linear_mat(2);
    //Rotation Part Assignment
    res.End_Effector_Transformation.rotation.x = Link_quaternion.x();
    res.End_Effector_Transformation.rotation.y = Link_quaternion.y();
    res.End_Effector_Transformation.rotation.z = Link_quaternion.z();
    res.End_Effector_Transformation.rotation.w = Link_quaternion.w();
    std::cout <<"Returned from EndEffectorPose Service"<< std::endl;
    return true;
}


bool robot_definitions::getRobotJacobian(robot_stack::GetManipulatorJacobian::Request &req,
                                    robot_stack::GetManipulatorJacobian::Response &res)
{
    if(req.service_flag){
        joint_values.clear();
        joint_values.emplace_back(req.Joint_Angles.a1);
        joint_values.emplace_back(req.Joint_Angles.a2);
        joint_values.emplace_back(req.Joint_Angles.a3);
        joint_values.emplace_back(req.Joint_Angles.a4);
        joint_values.emplace_back(req.Joint_Angles.a5);
        joint_values.emplace_back(req.Joint_Angles.a6);
        joint_values.emplace_back(req.Joint_Angles.a7);
        this->setJointAnglesMethod(joint_values);
        ROS_INFO_STREAM("Angles Updated in Jacobian Service \n");
    }
    ROS_INFO_STREAM("Jacobian Requested \n");
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::MatrixXd jacobian;
    kinematic_state->getJacobian(joint_model_group,
                             kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                             reference_point_position, jacobian);
    ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");
    
    res.Manipulator_Jacobian.data.resize(jacobian.rows()*jacobian.cols());
    for(int i=0; i<jacobian.rows(); i++){
        for(int j=0; j<jacobian.cols();j++){
            res.Manipulator_Jacobian.data[jacobian.rows()*i + j] = jacobian(i,j);
        }   
    }

    return true;
}

double robot_definitions::ComputeManipulabilityIndex(std::vector<double> pose){

    Eigen::Vector3d EE_Translation(pose[0],pose[1],pose[2]);
    Eigen::Quaterniond EE_Quaternion(pose[3], pose[4], pose[5], pose[6]);
    Eigen::Isometry3d EE_Transformation = Eigen::Isometry3d::Identity();

    //Creating an affine transformation pose of end effector
    EE_Transformation.rotate(EE_Quaternion);
    EE_Transformation.translate(EE_Translation);
    Eigen::MatrixXd jacobian = getJacobianMethod(EE_Transformation);

    Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(jacobian);
    std::cout << "In Manipulability Index Service" << std::endl;
    
    //Eigen::JacobiSVD<Eigen::MatrixXd> jacobian_svd(jacobian, Eigen::ComputeFullV | Eigen::ComputeFullU);        //Using JacobiSVD method to find the SCD of jacobian
    Eigen::BDCSVD<Eigen::MatrixXd> jacobian_svd(jacobian);                      //Using BDCSVD method to find the SCD of jacobian
    std::cout << "Rank of the Jacobian = "<< jacobian_svd.rank()<< std::endl;
    
    Eigen::VectorXd singular_values;        //Vector containing the singular values of jacobian matrix
    double product_temp = 1;

    double manipulability_index;
    if(jacobian_svd.rank()<6){
        manipulability_index = 0;
        return manipulability_index;
    }

    else{
        singular_values = jacobian_svd.singularValues();
        
        for(int i =0; i<singular_values.rows(); i++){
            product_temp*=singular_values(i)*singular_values(i);
        }
        product_temp = 1/product_temp; 
        manipulability_index = sqrt(product_temp);
        return manipulability_index;
    }

}

bool robot_definitions::getManipulabilityIndex(robot_stack::GetManipulabilityIndex::Request &req,
                                robot_stack::GetManipulabilityIndex::Response &res)
{
    Eigen::Vector3d EE_Translation(req.end_effector_pose.translation.x,req.end_effector_pose.translation.y,req.end_effector_pose.translation.z);
    Eigen::Quaterniond EE_Quaternion(req.end_effector_pose.rotation.w, req.end_effector_pose.rotation.x, req.end_effector_pose.rotation.y,req.end_effector_pose.rotation.z);
    Eigen::Isometry3d EE_Transformation = Eigen::Isometry3d::Identity();

    //Creating an affine transformation pose of end effector
    EE_Transformation.matrix().block<3,3>(0,0) = EE_Quaternion.normalized().toRotationMatrix();
    EE_Transformation.matrix().block<3,1>(0,3) = EE_Translation.transpose();
    Eigen::MatrixXd jacobian = getJacobianMethod(EE_Transformation);

    Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(jacobian);
    std::cout << "In Manipulability Index Service" << std::endl;
    
    //Eigen::JacobiSVD<Eigen::MatrixXd> jacobian_svd(jacobian, Eigen::ComputeFullV | Eigen::ComputeFullU);        //Using JacobiSVD method to find the SCD of jacobian
    Eigen::BDCSVD<Eigen::MatrixXd> jacobian_svd(jacobian);                      //Using BDCSVD method to find the SCD of jacobian
    std::cout << "Rank of the Jacobian = "<< jacobian_svd.rank()<< std::endl;
    
    Eigen::VectorXd singular_values;        //Vector containing the singular values of jacobian matrix
    double product_temp = 1;

    if(jacobian_svd.rank()<6){
        res.manipulability_index = 0;
        return true;
    }

    else{
        singular_values = jacobian_svd.singularValues();
        
        for(int i =0; i<singular_values.rows(); i++){
            product_temp*=singular_values(i)*singular_values(i);
        }
        product_temp = 1/product_temp; 
        res.manipulability_index = sqrt(product_temp);
        return true;
    }


}                                

void robot_definitions::setJointAnglesMethod(std::vector<double> joint_angles){
    joint_values.clear();
    if(joint_angles.size()==7){
        this->joint_values = joint_angles;
    }
        
    else{
        ROS_INFO_STREAM("The Joint Angles Vector is not of Appropriate Size");
        return;
    }
        

    kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
    //Enforcing Joint Limits
    if(!kinematic_state->satisfiesBounds()){
        ROS_INFO_STREAM("Current State is invalid");
        kinematic_state->enforceBounds();
    }

    else{
        ROS_INFO_STREAM("Current State is valid");
    }

}    

Eigen::Isometry3d robot_definitions::getFK(int link_number, std::vector<double> joint_angles){

    if(joint_angles.size()==7){
        this->setJointAnglesMethod(joint_angles);
    }
    const Eigen::Isometry3d& link_state = kinematic_state->getGlobalLinkTransform(link_names[link_number-1]);

    return link_state;

}

Eigen::MatrixXd robot_definitions::getJacobianMethod(){

    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::MatrixXd jacobian;
    bool jacobian_success_flag = kinematic_state->getJacobian(joint_model_group,
                             kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                             reference_point_position, jacobian);
    
    if(jacobian_success_flag){
        ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");
        return jacobian;
    }

    else{
        Eigen::MatrixXd failed_jacobian = Eigen::MatrixXd::Zero(6,7);
        ROS_INFO_STREAM("Failed to Calculate Jacobian, Point is not Feasible");
        return failed_jacobian;
    }
    

}

Eigen::MatrixXd robot_definitions::getJacobianMethod(Eigen::Isometry3d EE_Pose){
    std::vector<double> joint_angles = getJointAngles(EE_Pose);
    kinematic_state->setJointGroupPositions(joint_model_group, joint_angles);
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::MatrixXd jacobian;
    bool jacobian_success_flag = kinematic_state->getJacobian(joint_model_group,
                             kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                             reference_point_position, jacobian);
    
    if(jacobian_success_flag){
        ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");
        return jacobian;
    }

    else{
        Eigen::MatrixXd failed_jacobian = Eigen::MatrixXd::Zero(6,7);
        ROS_INFO_STREAM("Failed to Calculate Jacobian, Point is not Feasible");
        return failed_jacobian;
    }

}

std::vector<double> robot_definitions::getJointAngles(Eigen::Isometry3d end_effector_state){

    std::size_t attempts = 20;
    double timeout = 1;
    std::cout<<"The Pose of End Effector in getJoint Angles is = \n"<<end_effector_state.matrix()<<std::endl;
    //Move-it IK Routine
    bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);
    std::vector<double> temp_joint_angles;

   
    if (found_ik)
    {
        kinematic_state->copyJointGroupPositions(joint_model_group, temp_joint_angles);
        for(size_t i =0; i<temp_joint_angles.size(); i++){
            std::cout<<"The Value of Joint No. "<<i+1<<" = "<<temp_joint_angles[i]<<std::endl;
        }
        kinematic_state->setJointGroupPositions(joint_model_group, temp_joint_angles);
        
        ROS_INFO("IK Found Returning the Joint Values for Robot %s",this->robot_name.c_str());
        return temp_joint_angles;

    }

    else{
        ROS_INFO("Failed to find IK for Robot %s, returning joint angles as 0's",this->robot_name.c_str());
        temp_joint_angles.assign(7,0);
        return temp_joint_angles;
    }

    
    
}

