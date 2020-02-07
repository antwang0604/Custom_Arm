# mbzirc_move_it config Package   
    
## Launch   
To launch the robot in gazebo and moveit    
`roslaunch mbzirc_move_it_config spawn_custom_robot.launch`   
    
While the Launch file is running, serveral different files to run in parallel:      
        
### Moving the Robot into different poses   
`roslaunch mbzirc_move_it_config move_robot.launch`      
**Arguments**   
* pose: packaging, vertical, add_obstacle   

### Launch Service Server to attach bricks to robot   
First Launch the server   
`roslaunch mbzirc_move_it_config start_attach_object_server.launch`   
Then call the server and pass in argument: 
* attach: true or false, true to apply a brick of custom size into moveit, false to remove    
* dimensions: length, width, and height in meters   
