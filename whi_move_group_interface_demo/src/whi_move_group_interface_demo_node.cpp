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
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

const double TAU = 2.0 * M_PI;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_group_interface_demo");
    ros::NodeHandle nodeHandle;

    // ROS spinning must be running for the MoveGroupInterface to get information
    // about the robot's state. One way to do this is to start an AsyncSpinner beforehand
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // setup
    //
    // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
    // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
    // are used interchangeably
    static const std::string PLANNING_GROUP = "whi_arm";

    // planning_interface:`MoveGroupInterface class can be easily setup
    // using just the name of the planning group you would like to control and plan for
    moveit::planning_interface::MoveGroupInterface moveGroupInterface(PLANNING_GROUP);

    // use the planning_interface::PlanningSceneInterface class
    // to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // raw pointers are frequently used to refer to the planning group for improved performance
    const moveit::core::JointModelGroup* planningGroup =
        moveGroupInterface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // visualization
    //
    // package MoveItVisualTools provides many capabilities for visualizing objects, robots,
    // and trajectories in rviz as well as debugging tools such as step-by-step introspection of a script
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visualTools("whi_link0");
    visualTools.deleteAllMarkers();

    // remote control is an introspection tool that allows users to step through a high level script
    // via buttons and keyboard shortcuts in RViz
    visualTools.loadRemoteControl();

    // rviz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Isometry3d textPose = Eigen::Isometry3d::Identity();
    textPose.translation().z() = 1.0;
    visualTools.publishText(textPose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

    // batch publishing is used to reduce the number of messages being sent to rviz for large visualizations
    visualTools.trigger();

    // getting Basic Information
    //
    // print the name of the reference frame for this robot
    ROS_INFO_NAMED("demo", "planning frame: %s", moveGroupInterface.getPlanningFrame().c_str());

    // print the name of the end-effector link for this group.
    ROS_INFO_NAMED("demo", "end effector link: %s", moveGroupInterface.getEndEffectorLink().c_str());

    // get a list of all the groups in the robot:
    ROS_INFO_NAMED("demo", "available planning groups:");
    std::copy(moveGroupInterface.getJointModelGroupNames().begin(),
        moveGroupInterface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    // start the demo
    // 
    visualTools.prompt("press 'next' to start the demo with plan 01");

    // .. _move_group_interface-planning-to-pose-goal:
    //
    // planning to a Pose goal
    //
    // plan a motion for this group to a desired pose for the end-effector
    geometry_msgs::Pose targetPose01;
    targetPose01.orientation.w = 1.0;
    targetPose01.position.x = 0.08;
    targetPose01.position.y = -0.18;
    targetPose01.position.z = 0.18;
    moveGroupInterface.setPoseTarget(targetPose01);

    // call the planner to compute the plan and visualize it
    // note it is just planning, not asking move_group_interface to actually move the robot
    moveit::planning_interface::MoveGroupInterface::Plan movePlan;

    bool success = (moveGroupInterface.plan(movePlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("demo", "visualizing plan 01 (single pose goal) %s", success ? "" : "FAILED");

    // visualizing plans
    //
    // the plan can be visualized as a line with markers in rviz
    ROS_INFO_NAMED("demo", "visualizing plan 01 as trajectory line");
    visualTools.publishAxisLabeled(targetPose01, "pose01");
    visualTools.publishText(textPose, "single pose goal", rvt::WHITE, rvt::XLARGE);
    visualTools.publishTrajectoryLine(movePlan.trajectory_, planningGroup);
    visualTools.trigger();
    visualTools.prompt("press 'next' to continue with plan 02");

    // finally, to execute the trajectory stored in movePlan, the following method can be called:
    // note that this can lead to problems if the robot moved in the meanwhile
    // moveGroupInterface.execute(movePlan);

    // moving to a pose goal
    //
    // if you do not want to inspect the planned trajectory,
    // the following is a more robust combination of the two-step plan+execute pattern shown above
    // and should be preferred. note that the pose goal we had set earlier is still active,
    // so the robot will try to move to that goal
    // moveGroupInterface.move();

    // planning to a joint-space goal
    //
    // set a joint space goal and move towards it. this will replace the pose target we set above
    //
    // to start, we'll create an pointer that references the current robot's state
    // RobotState is the object that contains all the current position/velocity/acceleration data
    moveit::core::RobotStatePtr currentState = moveGroupInterface.getCurrentState();
    //
    // then get the current set of joint values for the group.
    std::vector<double> jointGroupPositions;
    currentState->copyJointGroupPositions(planningGroup, jointGroupPositions);

    // now, let's modify one of the joints, plan to the new joint space goal and visualize the plan
    jointGroupPositions[0] = -TAU / 6.0;  // -1/6 turn in radians
    moveGroupInterface.setJointValueTarget(jointGroupPositions);

    // lower the allowed maximum velocity and acceleration to 5% of their maximum
    // the default values are 10% (0.1) can be set in the joint_limits.yaml file of robot's moveit_config
    // or set explicit factors in code if the faster movement is required
    moveGroupInterface.setMaxVelocityScalingFactor(0.05);
    moveGroupInterface.setMaxAccelerationScalingFactor(0.05);

    success = (moveGroupInterface.plan(movePlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("demo", "visualizing plan 02 (joint space goal) %s", success ? "" : "FAILED");

    // visualize the plan in rviz
    visualTools.deleteAllMarkers();
    visualTools.publishText(textPose, "joint space goal", rvt::WHITE, rvt::XLARGE);
    visualTools.publishTrajectoryLine(movePlan.trajectory_, planningGroup);
    visualTools.trigger();
    visualTools.prompt("press 'next' to continue with plan 03");

    // planning with path constraints
    //
    // path constraints can easily be specified for a link on the robot
    // specify a path constraint and a pose goal for our group
    // first define the path constraint
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "whi_link6";
    ocm.header.frame_id = "whi_link0";
    ocm.orientation.w = 1.0;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;

    // now, set it as the path constraint for the group
    moveit_msgs::Constraints demoConstraints;
    demoConstraints.orientation_constraints.push_back(ocm);
    moveGroupInterface.setPathConstraints(demoConstraints);

    // enforce planning in joint space
    //
    // depending on the planning problem MoveIt chooses between `joint space` and `cartesian space` for problem representation
    // setting the group parameter `enforce_joint_model_state_space:true` in the ompl_planning.yaml file
    // enforces the use of `joint space` for all plans
    //
    // by default planning requests with orientation path constraints
    // are sampled in `cartesian space` so that invoking IK serves as a generative sampler
    //
    // by enforcing `joint space` the planning process will use rejection sampling to find valid requests
    // please note that this might increase planning time considerably
    //
    // we will reuse the old goal that we had and plan to it
    // note that this will only work if the current state already satisfies the path constraints
    // so we need to set the start state to a new pose
    moveit::core::RobotState startState(*moveGroupInterface.getCurrentState());
    geometry_msgs::Pose startPose;
    startPose.orientation.w = -1.0;
    startPose.position.x = 0.2;
    startPose.position.y = 0.05;
    startPose.position.z = 0.6;
    startState.setFromIK(planningGroup, startPose);
    moveGroupInterface.setStartState(startState);

    // now we will plan to the earlier pose target from the new start state that we have just created
    moveGroupInterface.setPoseTarget(targetPose01);

    // planning with constraints can be slow because every sample must call an inverse kinematics solver
    // lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed
    moveGroupInterface.setPlanningTime(10.0);

    success = (moveGroupInterface.plan(movePlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("demo", "visualizing plan 03 (constraints) %s", success ? "" : "FAILED");

    // visualize the plan in rviz
    visualTools.deleteAllMarkers();
    visualTools.publishAxisLabeled(startPose, "start");
    visualTools.publishAxisLabeled(targetPose01, "goal");
    visualTools.publishText(textPose, "constrained goal", rvt::WHITE, rvt::XLARGE);
    visualTools.publishTrajectoryLine(movePlan.trajectory_, planningGroup);
    visualTools.trigger();
    visualTools.prompt("press 'next' to continue with plan 04");

    // when done with the path constraint be sure to clear it
    moveGroupInterface.clearPathConstraints();

    // cartesian paths
    //
    // plan a cartesian path directly by specifying a list of waypoints for the end-effector to go through
    // note that we are starting from the new start state above
    // the initial pose (start state) does not need to be added to the waypoint list but adding it can help with visualizations
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(startPose);

    geometry_msgs::Pose targetPose02 = startPose;
    // down
    targetPose02.position.z -= 0.15;
    waypoints.push_back(targetPose02);
    // right
    targetPose02.position.y -= 0.15;
    waypoints.push_back(targetPose02);
    // up and left
    targetPose02.position.z += 0.15;
    targetPose02.position.y += 0.15;
    targetPose02.position.x -= 0.15;
    waypoints.push_back(targetPose02);

    // we want the cartesian path to be interpolated at a resolution of 1 cm
    // which is why we will specify 0.01 as the max step in cartesian translation
    // specify the jump threshold as 0.0, effectively disabling it
    // WARNING: disabling the jump threshold while operating real hardware can cause
    // large unpredictable motions of redundant joints and could be a safety issue
    moveit_msgs::RobotTrajectory trajectory;
    const double JUMP_THRESHOLD = 0.0;
    const double EE_SETP = 0.01;
    double fraction = moveGroupInterface.computeCartesianPath(waypoints, EE_SETP, JUMP_THRESHOLD, trajectory);
    ROS_INFO_NAMED("demo", "visualizing plan 04 (cartesian path) (%.2f%% achieved)", fraction * 100.0);

    // visualize the plan in rviz
    visualTools.deleteAllMarkers();
    visualTools.publishText(textPose, "cartesian path", rvt::WHITE, rvt::XLARGE);
    visualTools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
    {
        visualTools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    }
    visualTools.trigger();
    visualTools.prompt("press 'next' to continue with plan 05");

    // cartesian motions should often be slow, e.g. when approaching objects
    // the speed of cartesian plans cannot currently be set through the maxVelocityScalingFactor,
    // but requires you to time the trajectory manually, as describedon: https://groups.google.com/forum/#!topic/moveit-users/MOoFxy2exT4
    //
    // you can execute a trajectory like this
    moveGroupInterface.execute(trajectory);

    // adding objects to the environment
    //
    // first plan to another simple goal with no objects in the way.
    moveGroupInterface.setStartState(*moveGroupInterface.getCurrentState());
    geometry_msgs::Pose anotherPose;
    anotherPose.orientation.x = 1.0;
    anotherPose.position.x = 0.7;
    anotherPose.position.y = 0.0;
    anotherPose.position.z = 0.59;
    moveGroupInterface.setPoseTarget(anotherPose);

    success = (moveGroupInterface.plan(movePlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("demo", "visualizing plan 05 (with no obstacles) %s", success ? "" : "FAILED");

    visualTools.deleteAllMarkers();
    visualTools.publishText(textPose, "clear goal", rvt::WHITE, rvt::XLARGE);
    visualTools.publishTrajectoryLine(movePlan.trajectory_, planningGroup);
    visualTools.trigger();
    visualTools.prompt("next step");

    // now define a collision object ROS message for the robot to avoid
    moveit_msgs::CollisionObject collisionObject;
    collisionObject.header.frame_id = moveGroupInterface.getPlanningFrame();

    // the id of the object is used to identify it.
    collisionObject.id = "collision box";

    // define a box to add to the world.
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.1;
    primitive.dimensions[primitive.BOX_Y] = 1.5;
    primitive.dimensions[primitive.BOX_Z] = 0.5;

    // define a pose for the box (specified relative to frame_id)
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.5;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.25;

    collisionObject.primitives.push_back(primitive);
    collisionObject.primitive_poses.push_back(box_pose);
    collisionObject.operation = collisionObject.ADD;

    std::vector<moveit_msgs::CollisionObject> collisionObjectsList;
    collisionObjectsList.push_back(collisionObject);

    // now add the collision object into the world
    // (using a vector that could contain additional objects)
    ROS_INFO_NAMED("demo", "add an object into the world");
    planning_scene_interface.addCollisionObjects(collisionObjectsList);

    // show text in rviz of status and wait for MoveGroup to receive and process the collision object message
    visualTools.publishText(textPose, "add object", rvt::WHITE, rvt::XLARGE);
    visualTools.trigger();
    visualTools.prompt("press 'next' in the RvizVisualToolsGui window to once the collision object appears in rviz");

    // now when we plan a trajectory it will avoid the obstacle
    success = (moveGroupInterface.plan(movePlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("demo", "visualizing plan 06 (pose goal move around cuboid) %s", success ? "" : "FAILED");
    visualTools.publishText(textPose, "obstacle goal", rvt::WHITE, rvt::XLARGE);
    visualTools.publishTrajectoryLine(movePlan.trajectory_, planningGroup);
    visualTools.trigger();
    visualTools.prompt("press 'next' in the RvizVisualToolsGui window once the plan is complete");

    // attaching objects to the robot
    //
    // you can attach objects to the robot, so that it moves with the robot geometry
    // this simulates picking up the object for the purpose of manipulating it
    // the motion planning should avoid collisions between the two objects as well
    moveit_msgs::CollisionObject object2Attach;
    object2Attach.id = "object cylinder";

    shape_msgs::SolidPrimitive cylinder_primitive;
    cylinder_primitive.type = primitive.CYLINDER;
    cylinder_primitive.dimensions.resize(2);
    cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.20;
    cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.04;

    // define the frame/pose for this cylinder so that it appears in the gripper
    object2Attach.header.frame_id = moveGroupInterface.getEndEffectorLink();
    geometry_msgs::Pose grab_pose;
    grab_pose.orientation.w = 1.0;
    grab_pose.position.z = 0.2;

    // first, we add the object to the world (without using a vector)
    object2Attach.primitives.push_back(cylinder_primitive);
    object2Attach.primitive_poses.push_back(grab_pose);
    object2Attach.operation = object2Attach.ADD;
    planning_scene_interface.applyCollisionObject(object2Attach);

    // then, we attach the object to the robot. tt uses the frame_id to determine which robot link it is attached to
    // you could also use applyAttachedCollisionObject to attach an object to the robot directly
    ROS_INFO_NAMED("demo", "attach the object to the robot");
    moveGroupInterface.attachObject(object2Attach.id, "whi_link7");

    visualTools.publishText(textPose, "object attached to robot", rvt::WHITE, rvt::XLARGE);
    visualTools.trigger();

    // wait for MoveGroup to receive and process the attached collision object message
    visualTools.prompt("press 'next' in the RvizVisualToolsGui window once the new object is attached to the robot");

    // replan, but now with the object in hand
    moveGroupInterface.setStartStateToCurrentState();
    success = (moveGroupInterface.plan(movePlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("demo", "visualizing plan 7 (move around cuboid with cylinder) %s", success ? "" : "FAILED");
    visualTools.publishTrajectoryLine(movePlan.trajectory_, planningGroup);
    visualTools.trigger();
    visualTools.prompt("press 'next' in the RvizVisualToolsGui window once the plan is complete");

    // detaching and removing objects
    //
    // detach the cylinder from the robot's gripper
    ROS_INFO_NAMED("demo", "detach the object from the robot");
    moveGroupInterface.detachObject(object2Attach.id);

    // show text in rviz of status
    visualTools.deleteAllMarkers();
    visualTools.publishText(textPose, "object detached from robot", rvt::WHITE, rvt::XLARGE);
    visualTools.trigger();

    // wait for MoveGroup to receive and process the attached collision object message
    visualTools.prompt("press 'next' in the RvizVisualToolsGui window once the new object is detached from the robot");

    // remove the objects from the world
    ROS_INFO_NAMED("demo", "remove the objects from the world");
    std::vector<std::string> objectIds;
    objectIds.push_back(collisionObject.id);
    objectIds.push_back(object2Attach.id);
    planning_scene_interface.removeCollisionObjects(objectIds);

    // show text in rviz of status
    visualTools.publishText(textPose, "objects removed", rvt::WHITE, rvt::XLARGE);
    visualTools.trigger();

    // wait for MoveGroup to receive and process the attached collision object message
    visualTools.prompt("press 'next' in the RvizVisualToolsGui window to once the collision object disappears");

    ros::shutdown();
    return 0;
}
