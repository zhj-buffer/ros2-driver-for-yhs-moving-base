# ros2-driver-for-yhs-moving-base
This is the ros2 version driver work for YHS moving base build and test with ROS2 Foxy and Galactic

CAN to USB is required and the default interface working with this moving base.

Before the test, pls refer to the documents for USB setup on Ubuntu.

# step 1:
mkdir dev_ws
cd dev_ws
mkdir src && cd src
git clone https://github.com/zhj-buffer/ros2-driver-for-yhs-moving-base.git

# step 2:
sudo cp ros2_drive_package_can_ctrol/include/ros2_drive_package_can_ctrol/libcontrolcan.so  /usr/lib/

# step 3:
colcon build

# step 4:

ros2 run ros2_drive_package_can_ctrol ros2_drive_package_can_ctrol

1638245770.232171 [7] ros2_drive: using network interface wlan0 (udp/192.168.1.14) selected arbitrarily from: wlan0, l4tbr0, docker0                                 
[INFO] [1638245770.256129958] [ros2_drive_package_can_ctrol]: Node inited:'                                                                                          
[INFO] [1638245770.256428435] [ros2_drive_package_can_ctrol]: Open Can device                                                                                        
'                                                                                                                                                                    
[INFO] [1638245770.295696221] [ros2_drive_package_can_ctrol]: >>open can deivce error!                                                                               
[INFO] [1638245783.314598460] [ros2_drive_package_can_ctrol]: CAN Datasend >                                                                                         
                                                                             ID: 18c4d1d0                                                                            
                                                                                                                                                                     
[INFO] [1638245783.413864027] [ros2_drive_package_can_ctrol]: CAN Datasend >                                                                                         
                                                                             ID: 18c4d1d0                                                                            
                                                                                                                                                                     
[INFO] [1638245783.514260748] [ros2_drive_package_can_ctrol]: CAN Datasend >                                                                                         
                                                                             ID: 18c4d1d0                                                                            
                                                                                                                                                                     
[INFO] [1638245783.614420082] [ros2_drive_package_can_ctrol]: CAN Datasend >                                                                                         
                                                                             ID: 18c4d1d0                                                                            
                                                                                                                                                                     
[INFO] [1638245783.713839896] [ros2_drive_package_can_ctrol]: CAN Datasend >                                                                                         
                                                                             ID: 18c4d1d0

# step 5:

xavier@xavier-desktop:~/ros2_ws$ ros2 topic echo /ctrl_cmd                                                                                                           
1638243922.519584 [7]       ros2: using network interface wlan0 (udp/192.168.1.14) selected arbitrarily from: wlan0, l4tbr0, docker0                                 
                                                                                                                                                                     
ctrl_cmd_gear: 0                                                                                                                                                     
ctrl_cmd_linear: 1.0                                                                                                                                                 
ctrl_cmd_angular: 2.0                                                                                                                                                
---                                                                                                                                                                  
ctrl_cmd_gear: 0                                                                                                                                                     
ctrl_cmd_linear: 1.0                                                                                                                                                 
ctrl_cmd_angular: 2.0                                                                                                                                                
---                                                                                                                                                                  
ctrl_cmd_gear: 0                                                                                                                                                     
ctrl_cmd_linear: 1.0                                                                                                                                                 
ctrl_cmd_angular: 2.0                                                                                                                                                
---                                                                                                                                                                  
ctrl_cmd_gear: 0                                                                                                                                                     
ctrl_cmd_linear: 1.0                                                                                                                                                 
ctrl_cmd_angular: 2.0

# step 6：

xavier@xavier-desktop:~/ros2_ws$ ros2 topic pub -r 10 /ctrl_cmd ros2_drive_package_msg/msg/CtrlCmd  "{ctrl_cmd_gear: 0, ctrl_cmd_linear: 1.0, ctrl_cmd_angular: 2.0}"
                                                                                                                                                                     
1638244346.337436 [7]       ros2: using network interface wlan0 (udp/192.168.1.14) selected arbitrarily from: wlan0, l4tbr0, docker0                                 
publisher: beginning loop                                                                                                                                            
publishing #1: ros2_drive_package_msg.msg.CtrlCmd(ctrl_cmd_gear=0, ctrl_cmd_linear=1.0, ctrl_cmd_angular=2.0)                                                        
                                                                                                                                                                     
publishing #2: ros2_drive_package_msg.msg.CtrlCmd(ctrl_cmd_gear=0, ctrl_cmd_linear=1.0, ctrl_cmd_angular=2.0)                                                        
                                                                                                                                                                     
publishing #3: ros2_drive_package_msg.msg.CtrlCmd(ctrl_cmd_gear=0, ctrl_cmd_linear=1.0, ctrl_cmd_angular=2.0)


