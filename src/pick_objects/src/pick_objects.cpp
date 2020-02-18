#include <sstream> // for ostringstream
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "nav_msgs/Odometry.h"
#include <functional> // std::bind
#include <boost/assign/list_of.hpp>
#include "std_msgs/String.h"
#include "add_markers/AddMarkers.h" // many thanks for this: https://answers.ros.org/question/242427/how-to-use-a-service-defined-in-another-package/


// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction> MoveBaseClient;

void debug_print(const nav_msgs::Odometry::ConstPtr &msg) {
    ROS_INFO("Seq: [%d]", msg->header.seq);
    ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x, msg->pose.pose.position.y,
             msg->pose.pose.position.z);
    ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x,
             msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x, msg->twist.twist.angular.z);
}

class Listener {
public:
    Listener(MoveBaseClient *ac) : ac_(ac), state_(0), goal_x_(-2), goal_y_(0), goal_z_(0), debug(false) {};

    void stop() {
        this->ac_->cancelAllGoals();
    }

    void move(float x, float y, float w, char *start_print, char *end_print) {
        //////////////  send the goal  /////////////////
        move_base_msgs::MoveBaseGoal goal;

        // set up the frame parameters
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        goal.target_pose.pose.orientation.w = w;
        ROS_INFO(start_print);
        this->ac_->sendGoal(goal);

        // todo: don't wait for the result??!!
//        this->ac_->waitForResult();
//
//        // Check if the robot reached its goal
//        if (this->ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
//            ROS_INFO(end_print);
//
//        } else
//            ROS_INFO("The base failed to move forward 1 meter for some reason");
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
        char buff[50];
        memset(buff, 0, sizeof(buff));
        sprintf(buff, "state: %d", this->state_);
        ROS_INFO(buff);

        switch (state_) {
            case 0: // init
                memset(buff, 0, sizeof(buff));
                sprintf(buff, "heading to pick up => x: %f, y: %f \n", this->goal_x_, this->goal_y_);
                move(this->goal_x_, this->goal_y_, -1.0, buff, "pick up zone arrived");
                this->state_++;
                break;
            case 1: // on the way to pick up
                debug_print(msg);
                if (fabs(msg->pose.pose.position.x - this->goal_x_) < this->distance_error &&
                    fabs(msg->pose.pose.position.y - this->goal_y_) < this->distance_error) {
                    this->stop();
                    this->goal_x_ = -2.5;
                    this->goal_y_ = 1.0;

                    memset(buff, 0, sizeof(buff));
                    sprintf(buff, "heading to drop off => x: %f, y: %f \n", this->goal_x_, this->goal_y_);
                    move(this->goal_x_, this->goal_y_, 1.0, buff, "drop off arrived");
                    state_++;
                }
                break;
            case 2: // on the way to drop off
                // todo: need to wait for display?? do this later
                // on the way to drop off
                // do nothing
                debug_print(msg);
                if (fabs(msg->pose.pose.position.x - this->goal_x_) < this->distance_error &&
                    fabs(msg->pose.pose.position.y - this->goal_y_) < this->distance_error) {
                    this->stop();
                    state_++;
                }
                break;
        }

        return;

//        if (fabs(msg->pose.pose.position.x - this->goal_x_) < this->distance_error &&
//            fabs(msg->pose.pose.position.y - this->goal_y_) < this->distance_error) {
//            this->goal_x_ = -2.5;
//            this->goal_y_ = 1.0;
//
//            memset(buff, 0, sizeof(buff));
//            sprintf(buff, "heading to drop off => x: %f, y: %f \n", this->goal_x_, this->goal_y_);
//            move(this->goal_x_, this->goal_y_, 1.0, buff, "drop off arrived");
//            state_++;
//        } else {
//            // todo: sorry, I can't handle this... feel frustrated.
//            //  there are always some difference between the postion of action goal and position from odom
//            /* question
//            In pick_objects, suppose we set the goal (x, y, z) as (1, 1, 0) for example.
//            Then after the goal has been sent and the result is successful, should we expect that the robot should be at the place (1, 1, 0)?
//            However, if we subscribe from /odom and we can find that the robot's position is not (1, 1, 0).
//            How can I deal with this?
//             I tried to send the new goal to adjust robot's position, or set a large buffer(if robot is nearby I regard it arrive).
//             But these may cause add_marker does not synchronize with pick_objects( eg, object disappear but robot hasn't arrived yet)
//             Is this the right way to synchronize the action between add_mark and pick_objects based on odometry data?
//
//             Another question is that when my robot moves, it always move around and around, like dancing, unless the map has been discovered before. How can I solve this?
//             */
//
//////            // although movebase finished the action, robot may still not reach the goal place.
//////            // I personally trust odom more. So, need to send the adjust goal to make sure the robot get the destination.
//////            // So, the new goal = goal - (odom - goal) = goal * 2 - odom
////            this->goal_x_  = this->origin_goal_x_ - msg->pose.pose.position.x + this->origin_goal_x_;
////            this->goal_y_  = this->origin_goal_y_ - msg->pose.pose.position.y + this->origin_goal_y_;
//////
//////            ROS_INFO("goal-> x: [%f], y: [%f]",
//////                     this->goal_x_, this->goal_y_);
//////
////            char buff[50];
////            sprintf(buff, "adjusting new goal => x: %f, y: %f \n", this->goal_x_, this->goal_y_);
////            move(this->goal_x_, this->goal_y_, 1.0, buff, "adjustment done");
//        }

    }

private:
    MoveBaseClient *ac_;
    float goal_x_;
    float goal_y_;
    float goal_z_;

    float origin_goal_x_ = -2.0;
    float origin_goal_y_ = 0.0;
    int state_;
    bool debug;
    float distance_error = 0.7;
};


int main(int argc, char **argv) {
    // Initialize the simple_navigation_goals node
    ros::init(argc, argv, "pick_objects");
    ros::NodeHandle n;

    // 1. pick_object set pickup goal for robot
    // 2. pick_object subscribe topic odom and check the postion
    // 3. if robot arrives pick up zone, send a request to add_markers to hide the object
    // 4. after pick_object receive the response (hide action done), set drop off goal for robot
    // 5. wait and check position again
    // 6. if robot arrives drop off zone, send another request to add_markers to display the object
    // 7. receive the response and print "done"

        // The key point is how to communicate with add_marker
        // However, I prefer service/client way
        // http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29
        ros::ServiceClient client = n.serviceClient<add_markers::AddMarkers>("add_markers");
        add_markers::AddMarkers srv;
        srv.request.str_request = "hide";
        if (client.call(srv)) {
            std::cout << srv.response.str_response << std::endl;
            ROS_INFO("Response: %s",  srv.response.str_response);
        } else {
            ROS_ERROR("Failed to call service add_markers");
            return 1;
        }

    return 0;
//    // using publisher/subscriber
//    // http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
//    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
//
//    int count{1};
//    ros::Rate loop_rate(10);
//
//    while (ros::ok()) {
//        std_msgs::String msg;
//        std::stringstream ss;
//        ss << "hello world " << count;
//        msg.data = ss.str();
//        chatter_pub.publish(msg);
//        ROS_INFO("%s", msg.data.c_str());
//
//        ros::spinOnce();
//
//        loop_rate.sleep();
//        ++count;
//    }

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait 5 sec for move_base action server to come up
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }


    // todo: but still need to check the odom data, whether the robot is in the pick up zone or not
    // otherwise, add_marker and pick_object can not synchronize the action.
    // method 2: use class to add additional parameters for callback
    Listener listener(&ac);
    ros::Subscriber sub = n.subscribe("odom", 1000, &Listener::odomCallback, &listener);

    ros::spin();

    return 0;
}
