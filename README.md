# Home service robot

A practice on Ros path planning and navigation


For Home Service function:

- Using actionlib::SimpleActionClient to send instruct(goal pose) to the robot to move.
- Subscribe topic(odom) to check the the location of robot.
- However, the goal pose is not the same as the pose from odom topic - always different. As a compromise, I prefer to trust odom's data.
- Register a doneCallback function for action client so that we know when the action finish.
- Regard the robot has reached the goal(pick up zone or drop off zone), if either action client's doneCallback occur or the pose from odom topic close to(buffer) goal's pose.
- Node pick_object maintain the state to control the whole process. 
- The key point is to synchronize the action between add_marker and pick_object. I have tried using a topic , but finally choose a service as I do need a response.

So, the complete procedure as below. I indicate all the actions by different states which can be found in the programme:
    
### State BEGIN = 0
0. On node pick_objects: init register subscriber to topic /odom, init a action client for moving the base and a service client to communicate with node add_markers 
1. On the node add_markers, init a service server to communicate with pick_objects node, and  init a publisher for simulating pickup/dropoff action by show/hide virtual objects in RViz.
2. add_markers set a default place for the virtual object.
2. pick_object set a pickup goal for robot.
3. state => ON_THE_WAY_TO_PICKUP_ZONE
### ON_THE_WAY_TO_PICKUP_ZONE
1. pick_object continuously check the pose postion from topic /odom
2. If robot arrives the pick up zone(either from topic odom callback or from action client doneCallback), send a request to add_markers to hide the object
3. Set state => START_PICKING_UP
### START_PICKING_UP
1. when add_markers receives the request from pick_objects, it start to hide the object, wait for 5 seconds(like a real picking action), then send the response back to pick_objects node.
2. when pick_object receive the response (hide action done), state => FINISH_PICKING_UP
### FINISH_PICKING_UP
1. pick_objects set drop off goal for robot
2. state =>ON_THE_WAY_TO_DROPOFF_ZONE
### ON_THE_WAY_TO_DROPOFF_ZONE
1. pick_object continuously checks the pose postion from topic /odom, like ON_THE_WAY_TO_PICKUP_ZONE.1.
2. Like ON_THE_WAY_TO_PICKUP_ZONE.2  when robot arrives the drop off zone,  send another request to add_markers to show the object at the drop off place
3. state => START_DROPPING_OFF
### START_DROPPING_OFF
1. add_markers show the object, wait for 5 seconds, and send back the response
2. pick_objects receives the response and set state => FINISH_DROPPING_OFF.
### FINISH_DROPPING_OFF & END
7. print "job done" and state => END
