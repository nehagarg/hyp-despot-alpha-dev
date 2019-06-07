This version is compatible with ubuntu 16. Ros kinetic and vrep functionality is not guaranteed to work. Vrep functionality has been moved to python which invokes vrep on ubuntu 14. This code interacts with a server running vrep on ubuntu 14.
Fo ubuntu 14 version see branch grasping_without_vision of autonomous_grasping project

The transition and observation models are learned using tensorflow. For tenorflow c++ api without bazel and with cmake, install tensorflow from source.
Then follow instructions at : 
https://github.com/PatWie/tensorflow-cmake

Many folders containing data from simulator, object labels, object classification files, python files need to be linked from autonomous grasping project or data stored on unicorn:
python_scripts : autonomousGrasping/python_scripts
g3db_meshes: /data/neha/g3db_meshes/obj_files on unicorn0
g3db_object_labels: autonomousGrasping/grasping_ros_mico/g3db_object_labels
g3db_object_labels_for_classification: autonomousGrasping/grasping_ros_mico/g3db_object_labels_for_classification
config_files: autonomousGrasping/grasping_ros_mico/config_files
commands: autonomousGrasping/grasping_ros_mico/commands
pure_shape_labels: autonomousGrasping/grasping_ros_mico/pure_shape_labels
launch: autonomousGrasping/grasping_ros_mico/launch
scripts: autonomousGrasping/grasping_ros_mico/scripts
data : From unicorn0 in /data/neha/WORK_FOLDER/neha_github/autonomousGrasping/grasping_ros_mico/data
data_low_friction_table_exp_wider_object_workspace_ver7: From unicorn 0 in /data/neha/WORK_FOLDER/neha_github/autonomousGrasping/grasping_ros_mico/data_low_friction_table_exp_wider_object_workspace_ver7

Works with vrep version 3_3_2
version 3_3_2 modified using patch from here:
https://github.com/marcinkaszynski/vrep-ros-plugins/commit/3e3c1c22703a14e55cf32aff4603c23f82b2a5ab
for getting depth image for dexnet
For details see this link : http://marcinkaszynski.com/2016/09/10/vrep-ros-rgbdslam-simulated-kinect.html
dexnet_ws also needs to installed as code uses perception module from it
dexnet_ws has been modified to not impot cnn in perception module and some classes in gqcnn module as it conflicts with tensorflow c++ api


to run vrep in headless mode via ssh
xvfb-run --auto-servernum --server-num=1 -s "-screen 0 640x480x24" ./vrep.sh -h ../../WORK_FOLDER/vrep_scenes/micoWithSensorsMutliObjectTrialWithDespotIKVer1.ttt
Remove the port in remoteapiconnections.txt for running multiple vrep instances
if not using remote api no need to specify new port while runnig vrep, otherwise specify a different remote api port while started a new vrep instance

vrep scenes:
for amazon grasp trial :
micoWithSensorsAmazonGraspTrialWithDespotIKVer3.ttt

for multi object experiments :
micoWithSensorsMutliObjectTrialWithDespotIKVer1.ttt ormicoWithSensorsMutliObjectTrialWithDespotIKVer1<Size>Cylinder.ttt (Cylindrical object too heavy. Data collected does not mimic real robot behaviour)
micoWithSensorsMutliObjectTrialWithDespotIKCuboidVer2.ttt(Cuboid object too heavy. Data collected does not mimic real robot behaviour)
micoWithSensorsMutliObjectTrialWithDespotIKYCBObjectsVer3.ttt

micoWithSensorsMutliObjectTrialWithDespotIKVer4.ttt : (Object closer to real arm and gripper behaviour close to real arm behaviour around object)
table friction material : floor material in vrepsc
objecct height : 10cm
object weight : 0.3027 kg

micoWithSensorsMutliObjectTrialWithDespotIKVer5.ttt : (Difference from Ver4 : Gripper closing behaviour closer to real gripper closing behaviour. Gripper palm can detect collisions. Robot position shifted as previous position resulted in instability during pick at far x locations.
To change the scene from Ver4 to Ver 5 : change the force of MicoHand_finger12_motor1 to 4. Change the maximum angles on MicoHand_joint1_finger1/3 to 90, Divide the velocity in Cup script for j1 by 4.0 and Decimate the MicoHand shape by 90% so that it becomes a smple shape from multishape. Set Mico positon to 0.07,0.01

micoWithSensorsMutliObjectTrialWithDespotIKVer6.ttt : Different from Ver5: Mico target angle set to 90 degrees instead of some random values
Changed initial object y from 1485 to 1562 as it is the mid point of min and max range
Increased gripper z by 0.005 as gripper was colliding with table and was not able to have proper movements
Robot position shifted as gripper unstable when y = 0.08, pick height adjusted to 0.09 because of this
Made mico palm invisible to kinect for easy detection of object movement
(RobotInterface::version6 and RobotInterface::version7 get data from this interface. Version 7 check object movement using vision)

micoWithSensorsMutliObjectTrialWithDespotIKVer7.ttt : Difference from Ver 6: Adjustments for adding vision feedback: Gripper visiible to kinect sensor
RobotInterface::version8 gets data from this interface.


For data collection:
Run vrep
Then:
1. Generate joint data for various gripper locations using GatherJointData function
2. Generate sas'or data for all actions without object (Not needed for open table as default is fine)
2.1 Use the above data to generate sensor observation for open gripper (For open table can get just one value as gripper values do not change because f enountering shelf )
3. Generate gripper sensor observations with gripper closed without object
4. Generate gripper sensor observations with gripper closed with object
5. Calculate touch threshold for defining touch using values in 2. 3. 4.
6. Generate sas'or data with object for all action till close actions using GatherData() function. It will use the touch threshold calculated in ste 5 for stop untl touch actions.
7.  Generate sas'or data with object for open action using GatherData() function. It is required as we simply cannot use reverse state of close action for open action. Object may fall after opening gripperafter closing gripper.


For despot
-v 3 -t <no of seconds> -n < no of scenarios to sample> -l CAP --belief=<belief type> --number = <number type>

belief type :
SINGLE_PARTICLE : Belief contains only true state
GAUSSIAN : Belief contains 50 particles in gaussian distribution
GAUSSIAN_WITH_STATE_IN : Belief contains 50 particles in gaussian distribution and true state

number type:
-1 for generating state through gaussian distribution
0-81 for generating state therough belonging to 9x9 grid for object locations on table


Experiment details
Look in folder low_friction_table/multiObjectType
belief_cylinder_7_8_9_reward100_penalty10 vrep scene ver4, gaussian belief contain experiments woth cylinders
belief_cylinder_7_8_9_reward100_penalty100 vrep scene ver4, gaussian belief , contains experiments with cylinder and cylindrical g3db objects
belief_g3db_1_84_reward100_penalty10 vrep scene ver5, gaussian belief, no experiments
belief_uniform_cylinder_7_8_9_reward100_penalty10 vrep scene ver5, uiform belief ,contains experiments with cylinder and cylindrical g3db objects
belief_uniform_cylinder_7_8_9_reward100_penalty100 vrep scene ver4, uniform belief, done for checking combined learning versionn 4. contains only data experiments with cylinders
belief_uniform_cylinder_8_9_1001_reward100_penalty10 vrep scene ver5, uniform belief, experiment with 8, 9 and G3DB1 in belief
belief_uniform_cylinder_8_9_reward100_penalty10 vrep scene ver5, uniform belief, experiment with cylinder 8 and 9 in belief
belief_uniform_g3db_1_84_reward100_penalty10 vrep scene ver5, uniform belief, experiment with G3DB1 and G3DB84 in belief
belief_uniform_g3db_single_reward100_penalty10 vrep scene ver5, uniform belief, experiment with single g3db objects in belief

For real arm:
First launch mico arm using kinoa ros package:
roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=m1n6s200 use_urdf:=true
Then launch touch sensor in studio code
For command see numatac_can_driver in studio code

Then launch touch client in studio code:
python detect_touch_on_each_finger.py

Then launch mico client in grasping_ros_mico:
python mico_action_feedback_node.py
Note kinova_motion_execution_with_touch.py has been copied from studio code. The studio code version is old. The latest version is in grasping_ros_mico code

Then launch the policy from grasping_ros_mico code

To launce kinect2
Install vision_ws kinect2 branch from AdaCompNus
launch iai_kinect2 using command roslaunch kinect2_bridge kinect2_bridge.launch
To get the frame of aruco_ros marker:
From kinect2 aruco_ros branch launch
roslaunch grasping_ros_mico mico_double_calib_kinect2.launch
Please check that the marker size and id in the launch file is correct.
Then compute the static transform between robot and the marker, if not already done. After that launch static transform file by making appropriate changes
roslaunch grasping_ros_mico static_transform_publisher.launch
To check that static transform is correct, launch kinova ros moveit using command
roslaunch m1n6s200_moveit_config m1n6s200_demo.launch
And make sure that robot arm and point clod overlap completley. m1n6s200_moveit_config package is on nehagarg github or can be copied from mico machine

All this is done using script in commands/commands.txt or commands/commands_rls.txt
real_command.sh contains commands for running real robot experiments

#G3DB objects
Version 7 includes objects which are visible above gripper level. The test objects are same s verion 6 test objects. Only one extra object has been added randomly because one test object was not visible and had to be removed.
