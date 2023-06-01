# USI_Robotics_Image_Collection

This is a repository for USI Robotics course project in 2023. The project utilizes the MightyThymio to collect images around the target Robomaster S1. This image collection can be then used as a trianing dataset for pose detection using only Thymio's camera. 

### Running Steps

0. Run
```
sudo nano /etc/hosts
```
and add line
```
127.0.2.1	robotics23_2

```

1. Put the package under workspace and build it.
2. Open a new terminal window and source the workspace by running 
```
source ~/dev_ws/install/setup.bash
```
3. open CoppeliaSim
```
~/apps/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu20_04/coppeliaSim.sh
```
4. Open the cnn_robomaster.ttt scene
5. Open two new terminal windows, source the workspace and run
```
ros2 launch thymioid main.launch device:="tcp:host=localhost;port=33333" simulation:=True name:=thymio0
```
and
```
ros2 launch robomaster_ros  main.launch name:=RM0002 model:=s1 serial_number:=RM0002 tof_0:=true gimbal:=True gimbal_mode:=2
```
6. Open a new window and source the workspace and run
```
ros2 launch cnnrobomaster cnn.launch.xml thymio_name:=thymio0
```
