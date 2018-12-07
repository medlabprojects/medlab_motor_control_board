#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "medlab_motor_control_board/CaptureToken.h"
#include "medlab_motor_control_board/ReleaseToken.h"

ros::Publisher token_available_topic;

ros::ServiceServer capture_token_service;
ros::ServiceServer release_token_service;

bool token_captured;
int token_value;


/**
 * This gets called when the token is requested.
 *
 */
bool captureTokenCallback(medlab_motor_control_board::CaptureToken::Request& request, medlab_motor_control_board::CaptureToken::Response& response) {

    if (token_captured) { // if the token is captured then send back a 'false'
        response.token = 0;
        response.success = false;
        response.report = "token unavailable";
    }
    else { // if the token is available then increment the token value and return 'true'
        token_captured = true;
        token_value += 1;
    
        response.token = token_value;
        response.success = true;
        response.report = "";
        
        std_msgs::Bool token_msg;
        token_msg.data = false;
        token_available_topic.publish(token_msg);
    }
           
    return true;
}


/**
 * This gets called when the token is released.  It compares the sent token value with the value that we have on hand.
 * If they are the same, then the token can be released.
 *
 */
bool releaseTokenCallback(medlab_motor_control_board::ReleaseToken::Request& request, medlab_motor_control_board::ReleaseToken::Response& response) {

    if (token_captured && (request.token == token_value)) {
        token_captured = false;
        response.success = true;
        response.report = "";
        
        std_msgs::Bool token_msg;
        token_msg.data = true;
        token_available_topic.publish(token_msg);
    }
    else {
        response.success = false;
        response.report = "not returned";
    }
       
    return true;
}




int main( int argc, char** argv ) {
    ros::init(argc, argv, "handler");
    ros::NodeHandle n;

    token_captured = false;
    token_value = 0;



    token_available_topic = n.advertise<std_msgs::Bool>("token_available", 1, true);


    std_msgs::Bool token_msg;
    token_msg.data = false;
    token_available_topic.publish(token_msg);

    capture_token_service = n.advertiseService("capture_token", captureTokenCallback);
    release_token_service = n.advertiseService("release_token", releaseTokenCallback);




    for (int aa=0; aa<6; ++aa) {
        std::map<std::string,float> gain_map;     
        n.getParam("motor0/gains", gain_map);

        std::cout << "****************" << std::endl;
        std::cout << gain_map["D"] << std::endl;
        
        
    }



    
    
    
    

    while (ros::ok()) {
    
    
        ros::spinOnce();

    }
}
