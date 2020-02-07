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
First Launch the server for service /attach_object by typing in terminal
`roslaunch mbzirc_move_it_config start_attach_object_server.launch`   
Then call the server and pass in argument by typing:        
`rosservice call /attach_object` double tab completion for request:     
Two different Requests in service:      
* attach: true or false, true to apply a brick of custom size into moveit, false to remove    
* dimensions: length, width, and height in meters   
