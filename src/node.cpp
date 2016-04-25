#include "ntnu_cyborg_coordinator/ReleaseControl.h"
#include "ntnu_cyborg_coordinator/RequestControl.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

#include <string>

namespace ntnu_cyborg_coordinator
{
    class Coordinator
    {
        public:
            Coordinator(ros::NodeHandle& node)
                : node_(node)
            {
                requestControlService_ = node.advertiseService(
                    "/ntnu_cyborg_coordinator/requestControl", &Coordinator::requestControl, this);
                releaseControlService_ = node.advertiseService(
                    "/ntnu_cyborg_coordinator/releaseControl", &Coordinator::releaseControl, this);

                std::string rosAriaCmdVelTopic;
                node.param<std::string>("ros_aria_cmd_vel_topic", rosAriaCmdVelTopic, "RosAria/cmd_vel");

                int rosAriaCmdVelQueueSize;
                node.param("ros_aria_cmd_vel_queue_size", rosAriaCmdVelQueueSize, 10);

                rosAriaCmdVelPublisher_ = node.advertise<geometry_msgs::Twist>(
                    rosAriaCmdVelTopic, rosAriaCmdVelQueueSize);
            }

            void
            cmdVel(const geometry_msgs::Twist::ConstPtr& message)
            {
                rosAriaCmdVelPublisher_.publish(message);
            }

            bool
            requestControl(ntnu_cyborg_coordinator::RequestControl::Request&  request,
                           ntnu_cyborg_coordinator::RequestControl::Response& response)
            {
                if (currentControllerId_.empty())
                {
                    ROS_INFO_NAMED("ntnu_cyborg_coordinator",
                        "requestControl: %s has taken control!", request.id.c_str());

                    const std::string topicBaseName =
                        std::string("/ntnu_cyborg_coordinator/") + request.id;

                    rosAriaCmdVelSubscriber_ = node_.subscribe(
                        topicBaseName + "/RosAria/cmd_vel", 1, &Coordinator::cmdVel, this);

                    currentControllerId_     = request.id;
                    response.controlReceived = true;
                }
                else if (request.id == currentControllerId_)
                {
                    ROS_INFO_NAMED("ntnu_cyborg_coordinator",
                        "requestControl: %s already has control!", request.id.c_str());
                    response.controlReceived = true;
                }
                else
                {
                    ROS_INFO_NAMED("ntnu_cyborg_coordinator",
                        "requestControl: %s has been denied control!", request.id.c_str());
                    response.controlReceived = false;
                }

                return true;
            }

            bool releaseControl(ntnu_cyborg_coordinator::ReleaseControl::Request&  request,
                                ntnu_cyborg_coordinator::ReleaseControl::Response& response)
            {
                if (request.id == currentControllerId_)
                {
                    ROS_INFO_NAMED("ntnu_cyborg_coordinator",
                        "releaseControl: %s gave up control!", request.id.c_str());

                    currentControllerId_.clear();
                    rosAriaCmdVelSubscriber_.shutdown();
                }
                else
                {
                    ROS_INFO_NAMED("ntnu_cyborg_coordinator",
                        "releaseControl: %s tried to give up control, but did not have it!",
                        request.id.c_str());
                }

                return true;
            }

        private:
            ros::NodeHandle&   node_;
            std::string        currentControllerId_;
            ros::Publisher     rosAriaCmdVelPublisher_;
            ros::Subscriber    rosAriaCmdVelSubscriber_;
            ros::ServiceServer requestControlService_;
            ros::ServiceServer releaseControlService_;
    };
}

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "ntnu_cyborg_coordinator");

    ros::NodeHandle node;
    ntnu_cyborg_coordinator::Coordinator coordinator(node);

    ROS_INFO_NAMED("ntnu_cyborg_coordinator", "ntnu_cyborg_coordinator ready to serve!");

    ros::spin();
}

