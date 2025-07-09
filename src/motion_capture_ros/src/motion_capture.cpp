#include "geometry_msgs/PoseStamped.h"
#include "ros/time.h"
#include "tf/transform_datatypes.h"
#include <ros/ros.h>
#include <libmotioncapture/motioncapture.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <map>


class MotionCaptureNode
{
    public:
        MotionCaptureNode(ros::NodeHandle &nh, ros::NodeHandle &private_nh):
            nh(nh),
            private_nh(private_nh),
            tf_broadcaster()
        {
            ROS_INFO("Motion Capture Node initialized.");

            private_nh.getParam("quad_name", quadName);
            
            // Load parameters
            private_nh.getParam("type", motionCaptureType);
            private_nh.getParam("hostname", motionCaptureHostname);
            motionCaptureCfg["hostname"] = motionCaptureHostname;

            mocap = std::unique_ptr<libmotioncapture::MotionCapture>(
               libmotioncapture::MotionCapture::connect(motionCaptureType,
                                                        motionCaptureCfg));
            if (!mocap)
            {
                ROS_ERROR("Failed to connect to motion capture system of type: %s", motionCaptureType.c_str());
                ros::shutdown();
            }
            else
            {
                ROS_INFO("Connected to motion capture system of type: %s at %s", motionCaptureType.c_str(), motionCaptureHostname.c_str());
            }

            // Create publishers
            droneMotionCapturePub = nh.advertise<geometry_msgs::PoseStamped>("mocap/" +quadName+"/pose", 1);
            ballMotionCapturePub = nh.advertise<geometry_msgs::PoseStamped>("mocap/ball/pose", 1);

        }

        ~MotionCaptureNode()
        {
            ROS_INFO("Motion Capture Node shutting down.");
        }

        void run()
        {
            mocap->waitForNextFrame();
            auto time = ros::Time::now();
            
            bool hasDrone = false;
            for (const auto &iter : mocap->rigidBodies()){
                auto rigidBody = iter.second;
                if (rigidBody.name() != quadName){
                    continue; // Skip if not the quadrotor
                }

                // Get rigid body transform
                tf::Transform drone_transform;
                drone_transform.setOrigin(tf::Vector3(rigidBody.position().x(), rigidBody.position().y(), rigidBody.position().z()));
                drone_transform.setRotation(tf::Quaternion(rigidBody.rotation().x(),
                rigidBody.rotation().y(),
                rigidBody.rotation().z(),
                rigidBody.rotation().w())); 
                
                tf::StampedTransform stampedTransform(drone_transform, time, "world", rigidBody.name());

                // Publish drone pose
                geometry_msgs::PoseStamped dronePoseMsg;
                dronePoseMsg.header.stamp = time;
                dronePoseMsg.header.frame_id = "world";
                dronePoseMsg.pose.position.x = drone_transform.getOrigin().x();
                dronePoseMsg.pose.position.y = drone_transform.getOrigin().y();
                dronePoseMsg.pose.position.z = drone_transform.getOrigin().z();
                dronePoseMsg.pose.orientation.w = drone_transform.getRotation().w();
                dronePoseMsg.pose.orientation.x = drone_transform.getRotation().x();
                dronePoseMsg.pose.orientation.y = drone_transform.getRotation().y();
                dronePoseMsg.pose.orientation.z = drone_transform.getRotation().z();

                droneMotionCapturePub.publish(dronePoseMsg);
                tf_broadcaster.sendTransform(stampedTransform);
                hasDrone = true;
                break; // Assuming only one quadrotor is tracked

            }
            if (!hasDrone) {
                // If no drone detected, log a warning. Also, don't try to find a ball since unmatched drone markers might be present.
                ROS_WARN("No drone detected in motion capture data.");
                return;
            }

            auto pointcloud = mocap->pointCloud();
            int num_points = pointcloud.rows();
            if (num_points == 0) {
                ROS_WARN("No unlabeled markers detected.");
                return;
            }
            if (num_points > 1) {
                ROS_WARN("Multiple unlabeled markers detected. Ball pose is ambiguous. Not publishing ball pose.");
                return;
            }
            tf::Transform ballTransform;
            ballTransform.setOrigin(tf::Vector3(pointcloud(0, 0), pointcloud(0, 1), pointcloud(0, 2)));
            ballTransform.setRotation(tf::Quaternion(0, 0, 0, 1)); // Assuming ball is a point, no rotation

            tf::StampedTransform ballStampedTransform(ballTransform, time, "world", "ball");
            geometry_msgs::PoseStamped ballPoseMsg;
            ballPoseMsg.header.stamp = time;
            ballPoseMsg.header.frame_id = "world";
            ballPoseMsg.pose.position.x = ballTransform.getOrigin().x();
            ballPoseMsg.pose.position.y = ballTransform.getOrigin().y();
            ballPoseMsg.pose.position.z = ballTransform.getOrigin().z();
            ballPoseMsg.pose.orientation.w = ballTransform.getRotation().w();
            ballPoseMsg.pose.orientation.x = ballTransform.getRotation().x();
            ballPoseMsg.pose.orientation.y = ballTransform.getRotation().y();
            ballPoseMsg.pose.orientation.z = ballTransform.getRotation().z();

            ballMotionCapturePub.publish(ballPoseMsg);
            tf_broadcaster.sendTransform(ballStampedTransform);
        }

    private:
        ros::NodeHandle& nh;
        ros::NodeHandle& private_nh;

        std::map<std::string, std::string> motionCaptureCfg;
        std::string motionCaptureHostname;
        std::string motionCaptureType;

        ros::Publisher droneMotionCapturePub;
        ros::Publisher ballMotionCapturePub;
        tf::TransformBroadcaster tf_broadcaster;

        std::unique_ptr<libmotioncapture::MotionCapture> mocap;

        std::string quadName;
};





int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_capture_node");
  ros::NodeHandle nh, private_nh("~");

    MotionCaptureNode motionCaptureNode(nh, private_nh);
    while (ros::ok())
    {
        motionCaptureNode.run();
        ros::spinOnce();
    
    }
    
  ros::spin();
  return 0;
}