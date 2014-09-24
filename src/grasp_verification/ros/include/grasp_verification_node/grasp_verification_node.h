#ifndef GRASPVERIFICATIONNODE_H_
#define GRASPVERIFICATIONNODE_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>


#include <grasp_verification/grasp_verification.h>

using namespace std;
using namespace cv;

class GraspVerificationNode
{

    public:
        GraspVerificationNode(ros::NodeHandle &nh);
        virtual ~GraspVerificationNode();
        void eventCallback(const std_msgs::String &event_command);
        void imageCallback(const sensor_msgs::ImageConstPtr &image_message);
        void states();
        void initState();
        void idleState();
        void runState();
        void processGraspVerification();

    private:
        enum States {
            INIT,
            IDLE,
            RUNNING
        };

    private:
        ros::NodeHandle node_handler_;
        ros::Subscriber event_sub_;
        ros::Publisher event_pub_;
        ros::Publisher grasp_verification_pub_;
        image_transport::Subscriber image_sub_;
        image_transport::ImageTransport image_transporter_;
        image_transport::Publisher image_pub_;
        sensor_msgs::ImageConstPtr image_message_;
        bool start_grasp_verification_;
        bool image_sub_status_; 
        int graps_verification_status_ ;
        std_msgs::String status_msg_;
        std_msgs::Bool result_;
        Mat image_;
        Mat debug_image_;
        States run_state_;
        GraspVerification verificator;
};

#endif /* OPTICALFLOWNODE_H_ */
