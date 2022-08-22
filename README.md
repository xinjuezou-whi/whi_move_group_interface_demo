# whi_move_group_interface_demo
tutorial source: http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/move_group_interface/move_group_interface_tutorial.html

add yaml configure file to adapte both Panda arm and my own arm

## running the demo
### panda arm
open two shells. in the first shell start RViz and wait for everything to finish loading:
```
roslaunch panda_moveit_config demo.launch 
```
in the second shell, run the launch file:
```
roslaunch whi_move_group_interface_demo move_group_interface_demo.launch arm:=panda
```

### whi arm
open two shells. in the first shell start RViz and wait for everything to finish loading:
```
roslaunch whi_moveit_config demo.launch 
```
in the second shell, run the launch file:
```
roslaunch whi_move_group_interface_demo move_group_interface_demo.launch
```
