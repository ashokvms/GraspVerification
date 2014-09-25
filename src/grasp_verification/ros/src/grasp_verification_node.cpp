#include <grasp_verification_node/grasp_verification_node.h>

GraspVerificationNode::GraspVerificationNode(ros::NodeHandle &nh) : node_handler_(nh), image_transporter_(nh)
{
    event_pub_ = node_handler_.advertise<std_msgs::String>("event_out", 1);
    event_sub_ = node_handler_.subscribe("event_in", 1, &GraspVerificationNode::eventCallback, this);
    grasp_verification_pub_ = node_handler_.advertise<std_msgs::Bool>("grasp_verification_out", 1);
    image_sub_ = image_transporter_.subscribe("input_image", 1, &GraspVerificationNode::imageCallback, this);
    start_grasp_verification_ = false;
    run_state_ = IDLE;
    image_sub_status_ = false;
}

GraspVerificationNode::~GraspVerificationNode()
{
    event_pub_.shutdown();
    event_sub_.shutdown();
    grasp_verification_pub_.shutdown();
    image_pub_.shutdown();
    image_sub_.shutdown();
}

void GraspVerificationNode::eventCallback(const std_msgs::String &event_command)
{
    if (event_command.data == "e_start") {
        start_grasp_verification_ = true;
        ROS_INFO("GRASP VERIFICATION ENABLED");
    } else if (event_command.data == "e_stop") {
        start_grasp_verification_ = false;
        ROS_INFO("GRASP VERIFICATION DISABLED");
    }
}

void GraspVerificationNode::imageCallback(const sensor_msgs::ImageConstPtr &img_msg)
{
    image_message_ = img_msg;
    image_sub_status_ = true;
}

void GraspVerificationNode::states()
{
    switch (run_state_) {
        case INIT:
            initState();
            break;
        case IDLE:
            idleState();
            break;
        case RUNNING:
            runState();
            break;
        default:
            initState();
    }
}

void GraspVerificationNode::initState()
{
    if (image_sub_status_) {
        run_state_ = IDLE;
        image_sub_status_ = false;
    }
}

void GraspVerificationNode::idleState()
{
    if (start_grasp_verification_) {
        run_state_ = RUNNING;
    } else {
        run_state_ = INIT;
    }
}

void GraspVerificationNode::runState()
{
    processGraspVerification();
    run_state_ = INIT;
}

void GraspVerificationNode::processGraspVerification()
{
    cv_bridge::CvImagePtr cv_img_ptr;

    try {
        cv_img_ptr = cv_bridge::toCvCopy(image_message_, "rgb8");
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'rgb8'.", image_message_->encoding.c_str());
    }

    image_ = cv_img_ptr->image;

    graps_verification_status_ = verificator.verify_grasp(image_);

    result_.data = graps_verification_status_;

    std::stringstream ss;
    
    
    if (graps_verification_status_ == 1) {
        ss << "There is an object between the gripper";
        status_msg_.data = ss.str();
    }  
    
    else {
        ss << "There is no object between the gripper";
        status_msg_.data = ss.str();
    }

    event_pub_.publish(status_msg_);
    //publish to boolean
    grasp_verification_pub_.publish(result_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "grasp_verification");
    ros::NodeHandle nh("~");
    ROS_INFO("Grasp Verification Node Initialised");
    GraspVerificationNode vf(nh);

    while (ros::ok()) {
        vf.states();
        ros::Duration(0.33).sleep();
        ros::spinOnce();
    }

    return 0;
}
