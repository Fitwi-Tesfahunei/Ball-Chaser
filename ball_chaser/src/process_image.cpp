#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot - DONE
    ROS_INFO_STREAM("Robot moving towards the ball.");
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    int img_height = img.height;
    int img_step = img.step;
    int index;
    int L;
    bool left = false;
    bool center = false;
    bool right = false;
    bool ball_in_sight = false;

    // TODO: Loop through each pixel in the image and check if there's a bright white one - DONE
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    
    for (int k = 0; k < img_height; k++){
	for (int l = 0; l < img_step; l+=3){
	    int i = l+k*img_step;

	if (img.data[i] == white_pixel && img.data[i+1] == white_pixel && img.data[i+2] == white_pixel){
	    ball_in_sight = true;
	    L = l;
	    index = i;
	    break;
	}
	
	}
	
	if (ball_in_sight == true){
	    break;        
	}
    }

    if (L < img_step/3){
	left = true;
    }

    if (L >= img_step/3 && L <= 2*img_step/3){
	center = true;
    }

    if (L > 2*img_step/3){
	right = true;
    }
    //Move right
    if (ball_in_sight == true && right == true){
	ROS_INFO_STREAM("");
	ROS_INFO_STREAM("Ball in sight.");
	ROS_INFO_STREAM("Move right.");
	ROS_INFO_STREAM("L value: " + std::to_string(L));
	ROS_INFO_STREAM("Indexes: " + std::to_string(index) + ", " + std::to_string(index+1) + ", "  + std::to_string(index+2));
	ROS_INFO_STREAM("RGB Value: " + std::to_string(img.data[index]));
	ROS_INFO_STREAM("Region: " + std::to_string(img_step/2 + 100) + " to " + std::to_string(img_step) + ": Right Region");
	drive_robot(0.0,-0.5);
    }
    //Move straight
    if (ball_in_sight == true && center == true){
	ROS_INFO_STREAM("");
	ROS_INFO_STREAM("Ball in sight.");
	ROS_INFO_STREAM("Move straight.");
	ROS_INFO_STREAM("L value: " + std::to_string(L));
	ROS_INFO_STREAM("Indexes: " + std::to_string(index) + ", " + std::to_string(index+1) + ", "  + std::to_string(index+2));
	ROS_INFO_STREAM("RGB Value: " + std::to_string(img.data[index]));
	ROS_INFO_STREAM("Region: " + std::to_string(img_step/2 - 100) + " to " + std::to_string(img_step/2 + 100) + ": Center Region");
	drive_robot(0.5,0.0);
    }
    //Move left
    if (ball_in_sight == true && left == true){
	ROS_INFO_STREAM("");
	ROS_INFO_STREAM("Ball in sight.");
	ROS_INFO_STREAM("Move left.");
	ROS_INFO_STREAM("L value: " + std::to_string(L));
	ROS_INFO_STREAM("Indexes: " + std::to_string(index) + ", " + std::to_string(index+1) + ", "  + std::to_string(index+2));
	ROS_INFO_STREAM("RGB Value: " + std::to_string(img.data[index]));
	ROS_INFO_STREAM("Region: " + std::to_string(0) + " to " + std::to_string(img_step/2 - 100) + ": Left Region");
	drive_robot(0.0,0.5);
    }
    //Do not move
    if (ball_in_sight == false){
	drive_robot(0.0,0.0);
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 1, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
