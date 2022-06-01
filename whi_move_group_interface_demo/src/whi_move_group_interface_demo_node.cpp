/******************************************************************
node of MoveIt move group interface demo

Features:
- move group interface demo
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2022-05-26: Initial version
2022-xx-xx: xxx
******************************************************************/
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/src/Geometry/Transform.h>

int main(int argc, char** argv)
{
    /// node version and copyright announcement
    std::cout << "\nWHI MoveIt move group interface demo VERSION 00.03" << std::endl;
    std::cout << "Copyright © 2022-2023 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

    ros::init(argc, argv, "move_group_interface_demo");
    ros::NodeHandle nodeHandle;

    // ROS spinning must be running for the MoveGroupInterface to get information
    // about the robot's state. One way to do this is to start an AsyncSpinner beforehand
    ros::AsyncSpinner spinner(1);
    spinner.start();

    /// get config params
    std::string paramPlanningGroup;
    nodeHandle.param("/move_group_interface_demo/planning_group", paramPlanningGroup, std::string("whi_arm"));
    std::string paramVisualLink;
    nodeHandle.param("/move_group_interface_demo/visual_link", paramVisualLink, std::string("whi_link0"));
    double paramTitleHeight = 0.0;
    nodeHandle.param("/move_group_interface_demo/title_height", paramTitleHeight, 0.7);
    // plan01
    std::vector<double> paramGoal01;
    nodeHandle.getParam("/move_group_interface_demo/plan01/goal_pose", paramGoal01);
    // plan02
    std::vector<int> paramJointIndex;
    nodeHandle.getParam("/move_group_interface_demo/plan02/joint_index", paramJointIndex);
    std::vector<double> paramJointDelta;
    nodeHandle.getParam("/move_group_interface_demo/plan02/joint_goal", paramJointDelta);
    double paramVelScalingFactor;
    nodeHandle.getParam("/move_group_interface_demo/plan02/velocity_scaling_factor", paramVelScalingFactor);
    double paramAccScalingFactor;
    nodeHandle.getParam("/move_group_interface_demo/plan02/acc_scaling_factor", paramAccScalingFactor);
    // plan03
    std::vector<double> paramStartPose03;
    nodeHandle.getParam("/move_group_interface_demo/plan03/start_pose", paramStartPose03);
    std::vector<double> paramGoal03;
    nodeHandle.getParam("/move_group_interface_demo/plan03/goal_pose", paramGoal03);
    std::string paramConstraintFrame;
    nodeHandle.param("/move_group_interface_demo/plan03/constraint_frame", paramConstraintFrame, std::string("whi_link0"));
    std::string paramConstraintLink;
    nodeHandle.param("/move_group_interface_demo/plan03/constraint_link", paramConstraintLink, std::string("whi_link6"));
    std::vector<double> paramAbsTolerance;
    nodeHandle.getParam("/move_group_interface_demo/plan03/absolute_tolerance", paramAbsTolerance);
    double paramWeight = 0.0;
    nodeHandle.param("/move_group_interface_demo/plan03/weight", paramWeight, 1.0);
    double paramPlanningTime = 0.0;
    nodeHandle.param("/move_group_interface_demo/plan03/planning_time", paramPlanningTime, 10.0);
    // plan04
    double paramIkTimeout = 0.0;
    nodeHandle.param("/move_group_interface_demo/plan04/ik_timeout", paramIkTimeout, 0.5);
    std::vector<double> paramStartPose04;
    nodeHandle.getParam("/move_group_interface_demo/plan04/start_pose", paramStartPose04);
    std::vector<std::string> paramPoseIndex01;
    nodeHandle.getParam("/move_group_interface_demo/plan04/pose_index_01", paramPoseIndex01);
    std::vector<double> paramPoseDelta01;
    nodeHandle.getParam("/move_group_interface_demo/plan04/pose_delta_01", paramPoseDelta01);
    std::vector<std::string> paramPoseIndex02;
    nodeHandle.getParam("/move_group_interface_demo/plan04/pose_index_02", paramPoseIndex02);
    std::vector<double> paramPoseDelta02;
    nodeHandle.getParam("/move_group_interface_demo/plan04/pose_delta_02", paramPoseDelta02);
    std::vector<std::string> paramPoseIndex03;
    nodeHandle.getParam("/move_group_interface_demo/plan04/pose_index_03", paramPoseIndex03);
    std::vector<double> paramPoseDelta03;
    nodeHandle.getParam("/move_group_interface_demo/plan04/pose_delta_03", paramPoseDelta03);
    double paramJumpThreshold = 0.0;
    nodeHandle.param("/move_group_interface_demo/plan04/jump_threshold", paramJumpThreshold, 0.0);
    double paramEndEffectorStep = 0.0;
    nodeHandle.param("/move_group_interface_demo/plan04/end_effector_step", paramEndEffectorStep, 0.01);
    // plan05
    std::vector<double> paramGoal05;
    nodeHandle.getParam("/move_group_interface_demo/plan05/goal_pose", paramGoal05);
    std::vector<double> paramBoxSize;
    nodeHandle.getParam("/move_group_interface_demo/plan05/block_box_size", paramBoxSize);
    std::vector<double> paramBoxPose;
    nodeHandle.getParam("/move_group_interface_demo/plan05/block_box_pose", paramBoxPose);
    double paramCylinderRadius = 0.0;
    nodeHandle.param("/move_group_interface_demo/plan05/grab_cylinder_radius", paramCylinderRadius, 0.02);
    double paramCylinderHeight = 0.0;
    nodeHandle.param("/move_group_interface_demo/plan05/grab_cylinder_height", paramCylinderHeight, 0.1);
    std::vector<std::string> paramCylinderPoseIndex;
    nodeHandle.getParam("/move_group_interface_demo/plan05/grap_cylinder_pose_index", paramCylinderPoseIndex);
    std::vector<double> paramCylinderPose;
    nodeHandle.getParam("/move_group_interface_demo/plan05/grap_cylinder_pose", paramCylinderPose);
    std::string paramAttachLink;
    nodeHandle.param("/move_group_interface_demo/plan05/attach_link", paramAttachLink, std::string(""));
    
    /// setup
    //
    // MoveIt operates on sets of joints called "planning groups" and stores them in an object called the JointModelGroup
    // throughout MoveIt the terms "planning group" and "joint model group" are used interchangeably
    // using just the name of the planning group you would like to control and plan for
    moveit::planning_interface::MoveGroupInterface moveGroupInterface(paramPlanningGroup);

    // use the planning_interface::PlanningSceneInterface class to add and remove collision objects in "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planningSceneInterface;

    // raw pointers are frequently used to refer to the planning group for improved performance
    const moveit::core::JointModelGroup* jointModelGroup =
        moveGroupInterface.getCurrentState()->getJointModelGroup(paramPlanningGroup);

    // visualization
    //
    // package MoveItVisualTools provides many capabilities for visualizing objects, robots,
    // and trajectories in rviz as well as debugging tools such as step-by-step introspection of a script
    // set the base link for visual tools
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visualTools(paramVisualLink);
    visualTools.deleteAllMarkers();

    // remote control is an introspection tool that allows users to step through a high level script
    // via buttons and keyboard shortcuts in RViz
    visualTools.loadRemoteControl();

    // rviz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Isometry3d textPose = Eigen::Isometry3d::Identity();
    textPose.translation().z() = paramTitleHeight;
    visualTools.publishText(textPose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

    // batch publishing is used to reduce the number of messages being sent to rviz for large visualizations
    visualTools.trigger();

    // print the name of the reference frame for this robot
    ROS_INFO_NAMED("demo", "planning frame: %s", moveGroupInterface.getPlanningFrame().c_str());
    // print the name of the end-effector link for this group.
    ROS_INFO_NAMED("demo", "end effector link: %s", moveGroupInterface.getEndEffectorLink().c_str());

    // get a list of all the groups in the robot:
    ROS_INFO_NAMED("demo", "available planning groups:");
    std::copy(moveGroupInterface.getJointModelGroupNames().begin(),
        moveGroupInterface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    /// start the demo
    // 
    visualTools.prompt("press 'next' to start the demo with plan 01");

    /// Plan01: planning to a pose goal
    //
    // plan a motion for this group to a desired pose for the end-effector
    auto initRpy = moveGroupInterface.getCurrentRPY();
    std::cout << "current rpy " << initRpy[0] << ", " << initRpy[1] << ", " << initRpy[2] << std::endl;
    auto initPoseStamped = moveGroupInterface.getCurrentPose();
    tf2::Quaternion convertQuaternion;
    convertQuaternion.setRPY(initRpy[0], initRpy[1], initRpy[2]);
    std::cout << "init w " << initPoseStamped.pose.orientation.w << " convert w " << convertQuaternion.normalized().getW() << std::endl;
    geometry_msgs::Pose goal01;
    //goal01.orientation.x = convertQuaternion.getX();
    //goal01.orientation.y = convertQuaternion.getY();
    //goal01.orientation.z = convertQuaternion.getZ();
    //goal01.orientation.w = convertQuaternion.getW();
    int dof = jointModelGroup->getVariableNames().size();
    if (dof > 6)
    {
        // DOF 7
        goal01.orientation.w = 1.0;
    }
    else
    {
        // DOF 6
        goal01.orientation = initPoseStamped.pose.orientation;
    }
    goal01.position.x = paramGoal01[0];
    goal01.position.y = paramGoal01[1];
    goal01.position.z = paramGoal01[2];
    moveGroupInterface.setPoseTarget(goal01);

    // call the planner to compute the plan and visualize it
    // note it is just planning, not asking move_group_interface to actually move the robot
    moveit::planning_interface::MoveGroupInterface::Plan movePlan;

    bool success = (moveGroupInterface.plan(movePlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("demo", "visualizing plan 01 (single pose goal) %s", success ? "" : "FAILED");

    /// visualizing plans
    //
    // the plan can be visualized as a line with markers in rviz
    ROS_INFO_NAMED("demo", "visualizing plan 01 as trajectory line");
    visualTools.publishAxisLabeled(goal01, "pose01");
    visualTools.publishText(textPose, "single pose goal", rvt::WHITE, rvt::XLARGE);
    visualTools.publishTrajectoryLine(movePlan.trajectory_, jointModelGroup);
    visualTools.trigger();
    visualTools.prompt("press 'next' to continue with plan 02");

    // finally, to execute the trajectory stored in movePlan, the following method can be called:
    // note that this can lead to problems if the robot moved in the meanwhile
    //moveGroupInterface.execute(movePlan);

    // if you do not want to inspect the planned trajectory,
    // the following is a more robust combination of the two-step plan+execute pattern shown above
    // and should be preferred. note that the pose goal we had set earlier is still active,
    // so the robot will try to move to that goal
    //moveGroupInterface.move();

    /// Plan02: planning to a joint-space goal
    //
    // set a joint space goal and move towards it. this will replace the pose target we set above
    // get the current set of joint values for the group
    std::vector<double> jointGroupPositions;
    // first refresh the current state and then get the joint position from current state
    // RobotState is the object that contains all the current position of joint/velocity/acceleration data
    moveit::core::RobotStatePtr currentState = moveGroupInterface.getCurrentState();
    currentState->copyJointGroupPositions(jointModelGroup, jointGroupPositions);

    // now, let's modify one of the joints, plan to the new joint space goal and visualize the plan
    for (std::size_t i = 0; i < paramJointIndex.size(); ++i)
    {
        jointGroupPositions[paramJointIndex[i]] = paramJointDelta[i];
    }
    moveGroupInterface.setJointValueTarget(jointGroupPositions);

    // lower the allowed maximum velocity and acceleration to 5% of their maximum
    // the default values are 10% (0.1) can be set in the joint_limits.yaml file of robot's moveit_config
    // or set explicit factors in code if the faster movement is required
    moveGroupInterface.setMaxVelocityScalingFactor(paramVelScalingFactor);
    moveGroupInterface.setMaxAccelerationScalingFactor(paramAccScalingFactor);

    success = (moveGroupInterface.plan(movePlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("demo", "visualizing plan 02 (joint space goal) %s", success ? "" : "FAILED");

    // visualize the plan in rviz
    visualTools.deleteAllMarkers();
    visualTools.publishText(textPose, "joint space goal", rvt::WHITE, rvt::XLARGE);
    visualTools.publishTrajectoryLine(movePlan.trajectory_, jointModelGroup);
    visualTools.trigger();
    visualTools.prompt("press 'next' to continue with plan 03");

    /// Plan03: planning with path constraints
    //
    // path constraints can easily be specified for a link on the robot
    // specify a path constraint and a pose goal for our group
    // first define the path constraint
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = paramConstraintLink;
    ocm.header.frame_id = paramConstraintFrame;
    if (dof > 6)
    {
        // DOF 7
        ocm.orientation.w = 1.0;
    }
    else
    {
        // DOF 6
        ocm.orientation = initPoseStamped.pose.orientation;
    }
    ocm.absolute_x_axis_tolerance = paramAbsTolerance[0];
    ocm.absolute_y_axis_tolerance = paramAbsTolerance[1];
    ocm.absolute_z_axis_tolerance = paramAbsTolerance[2];
    ocm.weight = paramWeight;

    // then set it as the path constraint for the group
    moveit_msgs::Constraints demoConstraints;
    demoConstraints.orientation_constraints.push_back(ocm);
    moveGroupInterface.setPathConstraints(demoConstraints);

    /// enforce planning in joint space
    //
    // depending on the planning problem MoveIt chooses between `joint space` and `Cartesian space` for problem representation
    // setting the group parameter `enforce_joint_model_state_space:true` in the ompl_planning.yaml file
    // enforces the use of `joint space` for all plans
    //
    // by default planning requests with orientation path constraints
    // are sampled in `Cartesian space` so that invoking IK serves as a generative sampler
    //
    // by enforcing `joint space` the planning process will use rejection sampling to find valid requests
    // please note that this might increase planning time considerably
    //
    // reuse the old goal that we had and plan to it
    // note that this will only work if the current state already satisfies the path constraints
    // so we need to set the start state to a new pose
    moveit::core::RobotState startState03(*moveGroupInterface.getCurrentState());
    geometry_msgs::Pose startPose03;
    if (paramStartPose03.empty())
    {
        startPose03.orientation = initPoseStamped.pose.orientation;
        startPose03.position.x = initPoseStamped.pose.position.x;
        startPose03.position.y = initPoseStamped.pose.position.y;
        startPose03.position.z = initPoseStamped.pose.position.z;
    }
    else
    {
        startPose03.orientation.w = 1.0;
        startPose03.position.x = paramStartPose03[0];
        startPose03.position.y = paramStartPose03[1];
        startPose03.position.z = paramStartPose03[2];
    }
    startState03.setFromIK(jointModelGroup, startPose03);
    moveGroupInterface.setStartState(startState03);

    // plan to a new pose target from the new start that we have just created
    geometry_msgs::Pose goal03;
    if (dof > 6)
    {
        // DOF 7
        goal03.orientation.w = 1.0;
    }
    else
    {
        // DOF 6
        goal03.orientation = initPoseStamped.pose.orientation;
    }
    goal03.position.x = paramGoal03[0];
    goal03.position.y = paramGoal03[1];
    goal03.position.z = paramGoal03[2];
    moveGroupInterface.setPoseTarget(goal03);

    // planning with constraints can be slow because every sample must call an inverse kinematics solver
    // lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed
    moveGroupInterface.setPlanningTime(paramPlanningTime);

    success = (moveGroupInterface.plan(movePlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("demo", "visualizing plan 03 (with constraints) %s", success ? "" : "FAILED");

    // visualize the plan in rviz
    visualTools.deleteAllMarkers();
    visualTools.publishAxisLabeled(startPose03, "start");
    visualTools.publishAxisLabeled(goal03, "goal");
    visualTools.publishText(textPose, "constrained goal", rvt::WHITE, rvt::XLARGE);
    visualTools.publishTrajectoryLine(movePlan.trajectory_, jointModelGroup);
    visualTools.trigger();
    visualTools.prompt("press 'next' to continue with plan 04");

    // when done with the path constraint be sure to clear it
    moveGroupInterface.clearPathConstraints();

    /// Cartesian paths
    //
    // set the state to the one which satisfies the constraints of Cartesian
    geometry_msgs::Pose startPose04;
    if (dof > 6)
    {
        // DOF 7
        startPose04.orientation.w = 1.0;
    }
    else
    {
        // DOF 6
        startPose04.orientation = initPoseStamped.pose.orientation;
    }
    startPose04.position.x = paramStartPose04[0];
    startPose04.position.y = paramStartPose04[1];
    startPose04.position.z = paramStartPose04[2];
    moveit::core::RobotState startState05(*moveGroupInterface.getCurrentState());
    if (startState05.setFromIK(jointModelGroup, startPose04, paramIkTimeout))
    {
        moveGroupInterface.setStartState(startState05);
        ROS_INFO_NAMED("demo", "successfully set state for Cartesian path");

        // get the joint position from states   
        ROS_INFO_NAMED("demo", "state of start");
        const std::vector<std::string>& jointNames = jointModelGroup->getVariableNames();
        std::vector<double> jointValues;
        startState05.copyJointGroupPositions(jointModelGroup, jointValues);
        for (std::size_t i = 0; i < jointNames.size(); ++i)
        {
            ROS_INFO("joint %s: %f", jointNames[i].c_str(), jointValues[i]);
        }
        ROS_INFO_NAMED("demo", "state of current");
        moveGroupInterface.getCurrentState()->copyJointGroupPositions(jointModelGroup, jointValues);
        for (std::size_t i = 0; i < jointNames.size(); ++i)
        {
            ROS_INFO("joint %s: %f", jointNames[i].c_str(), jointValues[i]);
        }

        // forward kinematics
        const Eigen::Isometry3d& endEffectorStartState = startState05.getGlobalLinkTransform(moveGroupInterface.getEndEffectorLink());
        // print end-effector pose. remember that this is in the model frame
        ROS_INFO_STREAM("Translation of start state: \n" << endEffectorStartState.translation() << "\n");
        ROS_INFO_STREAM("Rotation of start state: \n" << endEffectorStartState.rotation() << "\n");
        const Eigen::Isometry3d& endEffectorCurrentState = 
            moveGroupInterface.getCurrentState()->getGlobalLinkTransform(moveGroupInterface.getEndEffectorLink());
        ROS_INFO_STREAM("Translation of current state: \n" << endEffectorCurrentState.translation() << "\n");
        ROS_INFO_STREAM("Rotation of current state: \n" << endEffectorCurrentState.rotation() << "\n");

        // get the Jacobian from states
        Eigen::Vector3d referencePointPosition(0.0, 0.0, 0.0);
        Eigen::MatrixXd jacobian;
        startState05.getJacobian(jointModelGroup,
            startState05.getLinkModel(jointModelGroup->getLinkModelNames().back()),
            referencePointPosition, jacobian);
        ROS_INFO_STREAM("Jacobian of start state: \n" << jacobian << "\n");
        moveGroupInterface.getCurrentState()->getJacobian(jointModelGroup,
            moveGroupInterface.getCurrentState()->getLinkModel(jointModelGroup->getLinkModelNames().back()),
            referencePointPosition, jacobian);
        ROS_INFO_STREAM("Jacobian of current state: \n" << jacobian << "\n");
    }
    else
    {
        ROS_WARN_NAMED("demo", "failed to get the solution of IK");
    }
    
    // plan a Cartesian path directly by specifying a list of waypoints for the end-effector to go through
    // note that we are starting from the new start state above
    // the initial pose (start state) does not need to be added to the waypoint list but adding it can help with visualizations
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(startPose04);

    geometry_msgs::Pose wayPose = startPose04;
    // up and back
    for (std::size_t i = 0; i < paramPoseIndex01.size(); ++i)
    {
        if (paramPoseIndex01[i] == "x")
        {
            wayPose.position.x += paramPoseDelta01[i];
        }
        else if (paramPoseIndex01[i] == "y")
        {
            wayPose.position.y += paramPoseDelta01[i];
        }
        else
        {
            wayPose.position.z += paramPoseDelta01[i];
        }
    }
    waypoints.push_back(wayPose);
    // left
    for (std::size_t i = 0; i < paramPoseIndex02.size(); ++i)
    {
        if (paramPoseIndex02[i] == "x")
        {
            wayPose.position.x += paramPoseDelta02[i];
        }
        else if (paramPoseIndex02[i] == "y")
        {
            wayPose.position.y += paramPoseDelta02[i];
        }
        else
        {
            wayPose.position.z += paramPoseDelta02[i];
        }
    }
    waypoints.push_back(wayPose);
    // down and right
    for (std::size_t i = 0; i < paramPoseIndex03.size(); ++i)
    {
        if (paramPoseIndex03[i] == "x")
        {
            wayPose.position.x += paramPoseDelta03[i];
        }
        else if (paramPoseIndex03[i] == "y")
        {
            wayPose.position.y += paramPoseDelta03[i];
        }
        else
        {
            wayPose.position.z += paramPoseDelta03[i];
        }
    }
    waypoints.push_back(wayPose);

    // specify 0.01 as the max step in Cartesian translation to let path to be interpolated at a resolution of 1 cm
    // specify the jump threshold as 0.0, effectively disabling it
    // WARNING: disabling the jump threshold while operating real hardware can cause
    // large unpredictable motions of redundant joints and could be a safety issue
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = moveGroupInterface.computeCartesianPath(waypoints, paramEndEffectorStep, paramJumpThreshold, trajectory);
    ROS_INFO_NAMED("demo", "visualizing plan 04 (cartesian path) (%.2f%% achieved)", fraction * 100.0);

    // Cartesian motions should often be slow, e.g. when approaching objects
    // the speed of Cartesian plans cannot currently be set through the maxVelocityScalingFactor,
    // but requires you to time the trajectory manually, as describedon: https://groups.google.com/forum/#!topic/moveit-users/MOoFxy2exT4
    // the trajectory can be executed like this
    //moveGroupInterface.execute(trajectory);

    // visualize the plan in rviz
    visualTools.deleteAllMarkers();
    visualTools.publishText(textPose, "Cartesian path", rvt::WHITE, rvt::XLARGE);
    visualTools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
    {
        visualTools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    }
    visualTools.trigger();
    visualTools.prompt("press 'next' to continue with plan 05");

    /// adding objects to the environment
    //
    // first plan to another simple goal with no objects in the way.
    moveGroupInterface.setStartState(*moveGroupInterface.getCurrentState());
    geometry_msgs::Pose goal05;
    if (dof > 6)
    {
        // DOF 7
        goal05.orientation.x = 1.0;
    }
    else
    {
        // DOF 6
        goal05.orientation = initPoseStamped.pose.orientation;
    }
    goal05.position.x = paramGoal05[0];
    goal05.position.y = paramGoal05[1];
    goal05.position.z = paramGoal05[2];
    moveGroupInterface.setPoseTarget(goal05);

    success = (moveGroupInterface.plan(movePlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("demo", "visualizing plan 05 (with no obstacles) %s", success ? "" : "FAILED");

    visualTools.deleteAllMarkers();
    visualTools.publishText(textPose, "clear goal", rvt::WHITE, rvt::XLARGE);
    visualTools.publishTrajectoryLine(movePlan.trajectory_, jointModelGroup);
    visualTools.trigger();
    visualTools.prompt("press 'next' to checkout the next step of plan 05");

    // define a box to add to the world
    shape_msgs::SolidPrimitive boxPrimitive;
    boxPrimitive.type = boxPrimitive.BOX;
    boxPrimitive.dimensions.resize(3);
    boxPrimitive.dimensions[boxPrimitive.BOX_X] = paramBoxSize[0];
    boxPrimitive.dimensions[boxPrimitive.BOX_Y] = paramBoxSize[1];
    boxPrimitive.dimensions[boxPrimitive.BOX_Z] = paramBoxSize[2];

    // define a pose for the box (specified relative to frame_id)
    geometry_msgs::Pose boxPose;
    boxPose.orientation.w = 1.0;
    boxPose.position.x = paramBoxPose[0];
    boxPose.position.y = paramBoxPose[1];
    boxPose.position.z = paramBoxPose[2];

    // define a collision object ROS message for the robot to avoid
    moveit_msgs::CollisionObject collisionObject;
    // frame_id decides the coord frame the object belongs to
    collisionObject.header.frame_id = moveGroupInterface.getPlanningFrame();
    // the id of the object is used to identify it
    collisionObject.id = "collision box";
    collisionObject.primitives.push_back(boxPrimitive);
    collisionObject.primitive_poses.push_back(boxPose);
    collisionObject.operation = collisionObject.ADD;

    std::vector<moveit_msgs::CollisionObject> collisionObjectsList;
    collisionObjectsList.push_back(collisionObject);

    // add the collision object into the world (using a vector)
    ROS_INFO_NAMED("demo", "add an object into the world");
    planningSceneInterface.addCollisionObjects(collisionObjectsList);

    // show text in rviz of status and wait for MoveGroup to receive and process the collision object message
    visualTools.publishText(textPose, "add object", rvt::WHITE, rvt::XLARGE);
    visualTools.trigger();
    visualTools.prompt("press 'next' to view the added collision object");

    // then replan trajectory which will avoid the obstacle
    success = (moveGroupInterface.plan(movePlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("demo", "visualizing plan 05 (pose goal move around cuboid) %s", success ? "" : "FAILED");
    visualTools.publishText(textPose, "obstacle goal", rvt::WHITE, rvt::XLARGE);
    visualTools.publishTrajectoryLine(movePlan.trajectory_, jointModelGroup);
    visualTools.trigger();
    visualTools.prompt("press 'next' to checkout the next step of plan 06");

    /// attaching objects to the robot
    //
    // attach objects to the robot, so that it moves with the robot geometry
    // this simulates picking up the object for the purpose of manipulating it
    // the motion planning should avoid collisions between the two objects as well
    shape_msgs::SolidPrimitive cylinderPrimitive;
    cylinderPrimitive.type = cylinderPrimitive.CYLINDER;
    cylinderPrimitive.dimensions.resize(2);
    cylinderPrimitive.dimensions[cylinderPrimitive.CYLINDER_HEIGHT] = paramCylinderHeight;
    cylinderPrimitive.dimensions[cylinderPrimitive.CYLINDER_RADIUS] = paramCylinderRadius;

    // define the frame/pose for this cylinder so that it appears in the gripper
    geometry_msgs::Pose grabPose;
    if (dof > 6)
    {
        // DOF 7
        grabPose.orientation.w = 1.0;
    }
    else
    {
        // DOF 6
        grabPose.orientation = initPoseStamped.pose.orientation;
    }
    for (std::size_t i = 0; i < paramCylinderPoseIndex.size(); ++i)
    {
        if (paramCylinderPoseIndex[i] == "x")
        {
            grabPose.position.x = paramCylinderPose[i];
        }
        else if (paramCylinderPoseIndex[i] == "y")
        {
            grabPose.position.y = paramCylinderPose[i];
        }
        else
        {
            grabPose.position.z = paramCylinderPose[i];
        }
    }

    // add the object to the world (without using a vector)
    moveit_msgs::CollisionObject object2Attach;
    // frame_id decides the coord frame the object belongs to
    object2Attach.header.frame_id = moveGroupInterface.getEndEffectorLink();
    object2Attach.id = "object cylinder";
    object2Attach.primitives.push_back(cylinderPrimitive);
    object2Attach.primitive_poses.push_back(grabPose);
    object2Attach.operation = object2Attach.ADD;
    planningSceneInterface.applyCollisionObject(object2Attach);

    // attach the object to the robot. it uses the frame_id to determine which robot link it is attached to
    // you could also use applyAttachedCollisionObject to attach an object to the robot directly
    ROS_INFO_NAMED("demo", "attach the object to the robot");
    if (!moveGroupInterface.attachObject(object2Attach.id, paramAttachLink))
    {
        ROS_WARN_NAMED("demo", "failed to attach object to link %s", paramAttachLink.c_str());
    }

    visualTools.publishText(textPose, "object attached to robot", rvt::WHITE, rvt::XLARGE);
    visualTools.trigger();

    // wait for MoveGroup to receive and process the attached collision object message
    visualTools.prompt("press 'next' in when the new object is attached to the robot");

    // replan, but now with the object in hand
    moveGroupInterface.setStartStateToCurrentState();
    success = (moveGroupInterface.plan(movePlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("demo", "visualizing plan 05 (move around cuboid with cylinder) %s", success ? "" : "FAILED");
    visualTools.publishTrajectoryLine(movePlan.trajectory_, jointModelGroup);
    visualTools.trigger();
    visualTools.prompt("press 'next' to detach object from arm");

    /// detaching and removing objects
    //
    // detach the cylinder from the robot's gripper
    ROS_INFO_NAMED("demo", "detach the object from the robot");
    moveGroupInterface.detachObject(object2Attach.id);

    // show text in rviz of status
    visualTools.deleteAllMarkers();
    visualTools.publishText(textPose, "object detached from robot", rvt::WHITE, rvt::XLARGE);
    visualTools.trigger();

    // wait for MoveGroup to receive and process the attached collision object message
    visualTools.prompt("press 'next' when the object is detached from the arm");

    // remove the objects from the world
    ROS_INFO_NAMED("demo", "remove the objects from the world");
    std::vector<std::string> objectIds;
    objectIds.push_back(collisionObject.id);
    objectIds.push_back(object2Attach.id);
    planningSceneInterface.removeCollisionObjects(objectIds);

    // show text in rviz of status
    visualTools.publishText(textPose, "objects removed", rvt::WHITE, rvt::XLARGE);
    visualTools.trigger();

    // wait for MoveGroup to receive and process the attached collision object message
    visualTools.prompt("press 'next' to exit the demo");

    ros::shutdown();
    return 0;
}
