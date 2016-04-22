// STD
#include <string>
#include <vector>

// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>

//*****************************************************************************************************************
// Global Variables and Methods

bool VERBOSE = false;

int rate_ = 10; // in Hz

bool fetchParameters(const ros::NodeHandle & nodeHandle, std::vector<std::string> & markerFrames, std::string & cameraFrame);
bool fetchMarkerFrameNames(std::string &frames, std::vector<std::string> & frameNames);
void doWork(const tf::TransformListener &listener, const std::vector<std::string> &markerFrameNames, const std::string &cameraFrame);

//*****************************************************************************************************************
//*****************************************************************************************************************

int main(int argc, char ** argv)
{

    ros::init(argc, argv, "navigation_node");

    ROS_INFO("Start marker bundle navigation node");

    ros::NodeHandle nh, pn("~");


    std::vector<std::string> markerFrameNames;
    std::string cameraFrame;
    if (!fetchParameters(pn, markerFrameNames, cameraFrame))
    {
        ros::shutdown();
        return 1;
    }

    // Enter a loop
    tf::TransformListener listener;
    ros::Rate rate(rate_);
    while (ros::ok())
    {
        doWork(listener, markerFrameNames, cameraFrame);
        rate.sleep();
    }

    return 0;

}

//*****************************************************************************************************************
//*****************************************************************************************************************

bool fetchParameters(const ros::NodeHandle &nodeHandle, std::vector<std::string> & markerFrames, std::string & cameraFrame)
{
    nodeHandle.param<bool>("verbose", VERBOSE, false);

    if (VERBOSE)
    {
    }

    if (!nodeHandle.hasParam("marker_frames"))
    {
        ROS_ERROR_STREAM("For " << ros::this_node::getName() << ", parameter 'marker_frames' is missing.");
        return false;
    }

    if (!nodeHandle.hasParam("camera_frame"))
    {
        ROS_ERROR_STREAM("For " << ros::this_node::getName() << ", parameter 'camera_frame' is missing.");
        return false;
    }

    std::string mframes;
    nodeHandle.getParam("marker_frames", mframes);
    if (!fetchMarkerFrameNames(mframes, markerFrames))
    {
        ROS_ERROR_STREAM("Failed to fetch 'marker_frames'. E.g. 'marker_frames=m1;m2;m3'");
        return false;
    }

    nodeHandle.getParam("camera_frame", cameraFrame);

    return true;
}

//*****************************************************************************************************************

bool fetchMarkerFrameNames(std::string & frames, std::vector<std::string> & frameNames)
{
    char * pch = strtok(const_cast<char*>(frames.c_str()), ";");
    while (pch != 0)
    {
        frameNames.push_back(std::string(pch));
        if (VERBOSE) ROS_INFO_STREAM("Frame name : " << frameNames[frameNames.size()-1] << ".");
        pch = strtok(0, ";");
    }
    return !frameNames.empty();
}

//*****************************************************************************************************************

void doWork(const tf::TransformListener & listener, const std::vector<std::string> & markerFrameNames, const std::string & cameraFrame)
{
    std::vector<tf::StampedTransform> transforms(markerFrameNames.size());
    std::vector<bool> foundTransforms(markerFrameNames.size(), false);
    ros::Time now = ros::Time::now();
    int count = 0;
    for (int i=0; i<markerFrameNames.size(); i++)
    {
        try
        {
            listener.lookupTransform(cameraFrame, markerFrameNames[i], now, transforms[i]);
            foundTransforms[i] = true;
            count++;
        }
        catch (const tf::TransformException & ex)
        {
            continue;
        }
    }
    if (count == 0)
    {
        // No transformations is found
        if (VERBOSE) ROS_INFO_STREAM("VERBOSE : No transformations 'marker -> camera' found");
        return;
    }




}

//*****************************************************************************************************************




