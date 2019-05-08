function sysCall_init()
    -- This child script contains computing path and publishing it into the path_planning.py for waypoint navigation.
    

    -- Declaring drone handle
    drone_handle = sim.getObjectHandle('Start')
    --Declaring the obstacle handle
    collection_handles= sim.getCollectionHandle('Obstacles')
    --Declaring a landing andle so that the drone lands properly
	landing=sim.getObjectHandle('landing_dummy')
 ---------------------------------------------------------------------------------------- 
  --Number of food trees in the arena for position
    no_of_food_handles = 3
    --getting the objecthandle for food trees for position and storing it in an array food
    food = {}
    for i=1,no_of_food_handles do
    	
   		table.insert(food,sim.getObjectHandle('Position_hoop_'..tostring(i)))
 	end
------------------------------------------------------------------------------------   
    --Number of food trees in the arena for orientation
    no_of_orientation_handles = 3
    --getting the objecthandle for food trees for orientation and storing it in an array Orientation
    Orientation = {} 
    for i=1,no_of_orientation_handles do
   		table.insert(Orientation,sim.getObjectHandle('loop'..tostring(i)))
  	end
------------------------------------------------------------------------------------
--Number of non-food trees in the arena for position    
    no_of_non_food_handles = 2
 --getting the objecthandle for non-food tree for position and storing it in an array non_food
    non_food = {}
    for i=1,no_of_non_food_handles do
   		table.insert(non_food,sim.getObjectHandle('non_food_'..tostring(i)))
   	end
------------------------------------------------------------------------------------

    no_of_goal_handles = 8
    goals = {}

    for i=1,no_of_goal_handles do
    	if i==1 then
    	    table.insert(goals,sim.getObjectHandle('initial'))
    	else
   			table.insert(goals,sim.getObjectHandle('goal_'..tostring(i)))
   		end
    	
    end
------------------------------------------------------------------------------------
--Number of goals in the scene 2 for each food tree one at the back-center and other at the front-center    
    no_of_back_front_handles = 6
    goal_set = {}
    --getting the objecthandle for goals(dummies) for position of those dummies and storing it in an array goal_set
	for i=1,no_of_back_front_handles do
   		table.insert(goal_set,sim.getObjectHandle('g_'..tostring(i)))
   	end
------------------------------------------------------------------------------------    
	back_front_position={{},{},{},{},{},{}}
	
------------------------------------------------------------------------------------    
    --getting object handle for back_hoop and front_hoop such that dummies are programitically placed near them while emulating position hoop


    --Declaring a orientation handle for the hoop



        --now setting the postion of goal_3 based on goal_2 by approximately adding some values


    --Declaring the camera handle 
    camera=sim.getObjectHandle('DefaultCamera')
    --array of start handle and goal handle in this order path plan happens index is selcted by count published by /status_flag topic from python node
    goal_handle = {goals[3],goals[2],goals[3],goals[4],goals[5],goals[4],goals[3],goals[2],goals[3],goals[8],goals[7],goals[6],goals[3],goals[2],goals[3],landing}
    --start_handle={goals[1],goals[6],goals[7],goals[8],goals[2],goals[3],goals[6],goals[7],goals[5],goals[4],goals[7],goals[6],goals[1]}

    
    flag=0
    
    --flag_check is use to emulate only once
    flag_check=0

   

   
   
    


    ------------path planning initialization------------------
    --creating a task t1
    t1=simOMPL.createTask('t1')
    --setting the upper and lower bound so that the path is computed within the specified bounds
    --bounds and path are computed once the emulation is completed
    ss={simOMPL.createStateSpace('6d',simOMPL.StateSpaceType.pose3d,drone_handle,{-2,-2,0},{2,2,2.5},1)}
    --setting the statespace
    simOMPL.setStateSpace(t1,ss)
    --choosing RRTConnect algorithm for path planning
    simOMPL.setAlgorithm(t1,simOMPL.Algorithm.RRTConnect)
    --setting the collision pairs so that the drone doesent collide with the obstacles 
    simOMPL.setCollisionPairs(t1,{sim.getObjectHandle('boundary'),collection_handles})
    -----------------------------------------------------------------------------------------------------------------------------------


    --------------------Declaring publisher and subscriber nodes here ---------------------
    --Publishing /vrep/waypoints which would be subscribed by python file to obtain the path computed 
    path_pub1=simROS.advertise('/vrep/waypoints', 'geometry_msgs/PoseArray')
    --Subscribing to /whycon/poses to get whycon values(for position) 
    whycon_sub = simROS.subscribe('/whycon/poses', 'geometry_msgs/PoseArray', 'whycon_callback')
   --subscribing to status_flag which would be published by python node and would be used for emulation
    sub=simROS.subscribe('/status_flag', 'std_msgs/Int16', 'demo')
    --Subscribing to /aruco_marker_publisher/markers to get aruco values(for orientation)
    aruco_sub = simROS.subscribe('/aruco_marker_publisher/markers', 'aruco_msgs/MarkerArray', 'aruco_callback')
    --Subscribing to /input_key which is published by python script which would help us to emulate the trees
    key_input = simROS.subscribe('/input_key', 'std_msgs/Int16', 'key_callback')

    
 --------------------------------------------------------------------------------------------------------------------------------------------

    --scaling factor for x,y axis whereas for z it is 3.3-(value_of_z*3.3)/35 which is computed below
    scale_factor = {9.54,9.54} 
    
    

end
function whycon_callback(msg)
     --  The position of the real-world whycon marker were obtained to set the position of the tree, dummies and drone.
    --  Using Flags to set the position of respective handles
    if flag==8 then
        --Emulating the drone here it is dummy
        sim.setObjectPosition(drone_handle,-1,{(-msg.poses[1].position.x)*0.35/3.34,(msg.poses[1].position.y)*0.35/3.34,3.3-(((msg.poses[1].position.z)*3.3)/35)})
        
    end
    if flag==14 then--initial
        --setting the initial dummy as per initial drone position with z axis value as been set manually in this case it is 25 in whycon coordinate
        sim.setObjectPosition(goals[1],-1,{(-msg.poses[1].position.x)*0.35/3.34,(msg.poses[1].position.y)*0.35/3.34,3.3-(((28)*3.3)/35)})
        sim.setObjectPosition(landing,-1,{(-msg.poses[1].position.x)*0.35/3.34,(msg.poses[1].position.y)*0.35/3.34,3.3-(((34)*3.3)/35)})
        flag=0
    end
--food trees
    if flag==11 then
        --Emulating the position of the food tree
        sim.setObjectPosition(food[3],-1,{(-msg.poses[1].position.x)*0.35/3.34,(msg.poses[1].position.y)*0.35/3.34,3.3-(((msg.poses[1].position.z)*3.3)/35)})
        flag=0
    end
    if flag==13 then
        --Emulating the position of the food tree
        sim.setObjectPosition(food[2],-1,{(-msg.poses[1].position.x)*0.35/3.34,(msg.poses[1].position.y)*0.35/3.34,3.3-(((msg.poses[1].position.z)*3.3)/35)})
        flag=0
    end
    if flag==12 then
        --Emulating the position of the food tree
        sim.setObjectPosition(food[1],-1,{(-msg.poses[1].position.x)*0.35/3.34,(msg.poses[1].position.y)*0.35/3.34,3.3-(((msg.poses[1].position.z)*3.3)/35)})
        flag=0
    end
    --non food tess
    if flag==1 then
        --Emulating the position of non food tree
        sim.setObjectPosition(non_food[1],-1,{(-msg.poses[1].position.x)*0.35/3.34,(msg.poses[1].position.y)*0.35/3.34,3.3-(((msg.poses[1].position.z)*3.3)/35)})
        flag=0
    end
    if flag==2 then
        --Emulating the position of the non food tree
        sim.setObjectPosition(non_food[2],-1,{(-msg.poses[1].position.x)*0.35/3.34,(msg.poses[1].position.y)*0.35/3.34,3.3-(((msg.poses[1].position.z)*3.3)/35)})
        --Emulating the position of non food tree dummy (This is used only if it is required).Not mandatory to use everywhere
        sim.setObjectPosition(goals[8],-1,{(-msg.poses[1].position.x)*0.35/3.34,(msg.poses[1].position.y)*0.35/3.34,3.3-(((msg.poses[1].position.z-5)*3.3)/35)})
        flag=0
    end
    
    


    
    

end


function aruco_callback(msg)
    -- the orientation(quaternion) of the ArUco marker was obtained and  the orientation of the hoop was set
    
    
    if flag==11 then--flag is used to set orientation of sal tree
        
        --Setting the orientation of the tree
        sim.setObjectQuaternion(Orientation[3],camera,{(msg.markers[1].pose.pose.orientation.x),(msg.markers[1].pose.pose.orientation.y),msg.markers[1].pose.pose.orientation.z,msg.markers[1].pose.pose.orientation.w})
        flag =0
        --getting the position of back and front hoop after position hoop orientation is set
        back_front_position[1]=sim.getObjectPosition(goal_set[1],-1)
        back_front_position[2]=sim.getObjectPosition(goal_set[2],-1)
        --now setting the position of goals as per the position obtained above
        sim.setObjectPosition(goals[2],-1,{back_front_position[1][1],back_front_position[1][2],back_front_position[1][3]})
        sim.setObjectPosition(goals[3],-1,{back_front_position[2][1],back_front_position[2][2],back_front_position[2][3]})
        --now setting the postion of goal_3 based on goal_2 by approximately adding some values
        

    end
    if flag==13 then
    	--Setting the orientation of the tree
        sim.setObjectQuaternion(Orientation[2],camera,{(msg.markers[1].pose.pose.orientation.x),(msg.markers[1].pose.pose.orientation.y),msg.markers[1].pose.pose.orientation.z,msg.markers[1].pose.pose.orientation.w})
        flag =0
        --getting the position of back and front hoop after position hoop orientation is set
        back_front_position[3]=sim.getObjectPosition(goal_set[3],-1)
        back_front_position[4]=sim.getObjectPosition(goal_set[4],-1)
        --now setting the position of the goals as per the position obtained above
        sim.setObjectPosition(goals[4],-1,{back_front_position[3][1],back_front_position[3][2],back_front_position[3][3]})
        sim.setObjectPosition(goals[5],-1,{back_front_position[4][1],back_front_position[4][2],back_front_position[4][3]})
    
    end
    if flag==12 then
    	--Setting the orientation of the tree
        sim.setObjectQuaternion(Orientation[1],camera,{(msg.markers[1].pose.pose.orientation.x),(msg.markers[1].pose.pose.orientation.y),msg.markers[1].pose.pose.orientation.z,msg.markers[1].pose.pose.orientation.w})
        flag =0
        --getting the position of back and front hoop after position hoop orientation is set
        back_front_position[5]=sim.getObjectPosition(goal_set[5],-1)
        back_front_position[6]=sim.getObjectPosition(goal_set[6],-1)
        --now setting the position of goal_1 and goal_2 as per the position obtained above
        sim.setObjectPosition(goals[6],-1,{back_front_position[5][1],back_front_position[5][2],back_front_position[5][3]})
        sim.setObjectPosition(goals[7],-1,{back_front_position[6][1],back_front_position[6][2],back_front_position[6][3]})
    end

    

    
end
function key_callback(msg)
    -- Read key input to set or unset position and orientation of food and non-food trees
    --print(msg)
    flag=msg.data
 



end

function demo(msg)
    temp = msg.data
    --Storing the values from python node into temp
    flag_check = 1

    if temp==170 then
        --when temp value is 8 then emulationFlag is set to 8 which inturn emulates the drone
        flag =8

       -- since in lua array starts from 1 
        temp =1
        --flag_check is used to emulate only once once this flag is set high only then the path is been computed
        flag_check = 0
    --********FOR REPOSITION IF THE DRONE MISBEAVES AFTER COLLECTING FOOD FROM 1ST TREE************
    elseif temp==180 then
        --when temp value is 8 then emulationFlag is set to 8 which inturn emulates the drone
        flag =8

        -- since in lua array starts from 1 
        temp =1
        --flag_check is used to emulate only once once this flag is set high only then the path is been computed
        flag_check = 0
        goal_handle={goals[6],goals[7],goals[3],goals[2],goals[3],goals[4],goals[5],goals[4],goals[3],goals[2],goals[3],goals[1],landing}
        --********FOR REPOSITION IF THE DRONE MISBEAVES AFTER COLLECTING 1 FOOD FROM 2ND TREE************
    elseif temp==190 then
        --when temp value is 8 then emulationFlag is set to 8 which inturn emulates the drone
        flag =8

        -- since in lua array starts from 1
        temp =1
        --flag_check is used to emulate only once once this flag is set high only then the path is been computed
        flag_check = 0
        goal_handle={goals[4],goals[5],goals[4],goals[3],goals[2],goals[3],goals[1],landing}
    else
         --flag_check is used to emulate only once once this flag is set high only then the path is been computed
        flag_check = 1

    end
    --g would be used as index of  goal_handleand start_handle 
    g=temp
    
    --temp value act as a index to access start and goal handle it is kept 1 initially because lua indexing start from 1 
    
end


function getpose(handle,ref_handle)
    --to get the position of  handle obtained with respect to ref_handle
    position = sim.getObjectPosition(handle,ref_handle)
    --to get the orientation of  handle obtained with respect to ref_handle
    orientation = sim.getObjectQuaternion(handle,ref_handle)
    --to create a datatype like pose (position(x,y,z)+orientation(x,y,z,w))
    pose = {position[1],position[2],position[3],orientation[1],orientation[2],orientation[3],orientation[4]}
    return pose
end
-- This function can be used to visualize the path that is been computed. This function takes path points as the argument...
function visualizePath( path )
    if not _lineContainer then
        _lineContainer=sim.addDrawingObject(sim.drawing_lines,1,0,-1,99999,{0.2,0.2,1})
    end
    sim.addDrawingObjectItem(_lineContainer,nil)
    if path then
        local pc=#path/7
        for i=1,pc-1,1 do
            lineDat={path[(i-1)*7+1],path[(i-1)*7+2],path[(i-1)*7+3],path[i*7+1],path[i*7+2],path[i*7+3]}
            sim.addDrawingObjectItem(_lineContainer,lineDat)
        end
    end
end


-- This function is used to send the Path computed in the real_world to whycon_world after transformation
-- Message is to be sent in geometry_msgs/PoseArray as WhyCon node subscribes to that message
function packdata(path)

    local sender = {header = {}, poses = {}}
    
    sender['header']={seq=123, stamp=simROS.getTime(), frame_id="drone"}
    sender['poses'] = {}

    for i=1,((no_of_path_points_required-1)*7)+1,7 do
        a = {x = 0, y = 0, w = 0, z = 0}
        b = {x = 0, y = 0, z = 0}
        pose = {position = b, orientation = a, }

        ------------------- x, y and z value after converting real_world to whycon_world using the computed scale_factor--------------------------------

        pose.position.x = (path[i])*scale_factor[1]
        pose.position.y = (path[i+1])*(scale_factor[2])
        pose.position.z = (3.3-(path[i+2]))/0.094 
        sender.poses[math.floor(i/7) + 1] = pose



        --------------------------------------------------------------------------------------------------------------------
    end
    
    
    return sender

end


--- This function is used to compute and publish the path to path_planninglpy
function compute_and_send_path(task)
    local r
    local path
    --giving the task created t1,max time to compute the path,no of path points required to compute to simOMPL.compute as parameters
    --If the distance between the drone handle and the goal handle is in x and y direction is less than 0.01 in vrep frame then the number of points is reduced to 31(experimented).
    --This is our uniquiness in theme implementation
    if math.abs(sim.getObjectPosition(drone_handle,-1)[1]-sim.getObjectPosition(goal_handle[g],-1)[1])<0.01 or math.abs(sim.getObjectPosition(drone_handle,-1)[2]-sim.getObjectPosition(goal_handle[g],-1)[2])<0.01 then
    --The number for path points were determined experimentally for faster theme implementation and proper follow of path wthout any jerk
    no_of_path_points_required=31
    else
    --According to our experiment when the no of path point was higher than 41 it resulted in jerk
    --Due to closer waypoints whereas when we decreased the waypoints below 41 it resulted in non-appropriate motion
	--if the distance of drone handle and goal handle is greater than 0.01 then we are taking 41 points (experimentally for this value it came good).   
    no_of_path_points_required=41	
    end

   
    r,path=simOMPL.compute(t1,10,-1,no_of_path_points_required)
    
    print(r, #path)

    
    

    if(r == true) then
        --path created to be vizulized to check whether the path computed is correct
        visualizePath(path)

        message = packdata(path)  
        simROS.publish(path_pub1,message)
        -- Provide slots for debug to cross check the message or path points you recieve after computing
    end
    return r
end


function sysCall_actuation()
    
    ---- Added the code to set start and goal state after getting present poses of the drone and the new goal when python node request's a path
    ---------------------------------------------------------------------------------------------------------------
    if flag_check == 1 then
        -- Getting startpose
        start_pose = getpose(drone_handle,-1)
        -- Getting the goalpose
        goal_pose = getpose(goal_handle[g],-1)
        -- Setting start state

        simOMPL.setStartState(t1,start_pose)
        
        -- Setting goal state but the orientation is set same as that of startpose
        simOMPL.setGoalState(t1,{goal_pose[1],goal_pose[2],goal_pose[3],start_pose[4],start_pose[5],start_pose[6],start_pose[7]})
        -- Computing path and publishing path points
        status = compute_and_send_path(t1)

        if(status == true) then -- path computed
            --setting flag_check to 0 since we need the path to be computed only once
            flag_check =0
        end
    end 

end

---*******************************************THANKS A LOT HOPING TO REACH THE FINALS AT IIT-B**************************************************


