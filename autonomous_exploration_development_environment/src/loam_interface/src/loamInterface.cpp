#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;

string stateEstimationTopic = "/integrated_to_init";
string registeredScanTopic = "/velodyne_cloud_registered";
string outputStateEstimationTopic = "/state_estimation";
string outputRegisteredScanTopic = "/registered_scan";
string keyPoseTopic = "/key_pose_to_map";
string alignmentReadyTopic = "/dcl_slam/alignment_ready";
string worldFrame = "map";
string childFrame = "sensor";
bool flipStateEstimation = true;
bool flipRegisteredScan = true;
bool sendTF = true;
bool reverseTF = false;
bool publishKeyPose = false;
bool estimateTwist = false;
double keyPoseDistance = 1.0;
double keyPoseAngle = 0.2;
double readinessStableDuration = 2.0;
double inputTimeout = 1.0;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());

nav_msgs::Odometry odomData;
tf::StampedTransform odomTrans;
ros::Publisher *pubOdometryPointer = NULL;
tf::TransformBroadcaster *tfBroadcasterPointer = NULL;
ros::Publisher *pubLaserCloudPointer = NULL;
ros::Publisher *pubKeyPosePointer = NULL;
ros::Publisher *pubAlignmentReadyPointer = NULL;

bool odometryReceived = false;
bool cloudReceived = false;
bool alignmentReady = false;
bool previousOdomValid = false;
bool previousKeyPoseValid = false;
int keyPoseID = 0;
ros::WallTime firstCompleteInputTime;
ros::WallTime lastOdometryWallTime;
ros::WallTime lastCloudWallTime;
nav_msgs::Odometry previousOdom;
geometry_msgs::Pose previousKeyPose;

double normalizeAngle(double angle)
{
  while (angle > PI) angle -= 2.0 * PI;
  while (angle < -PI) angle += 2.0 * PI;
  return angle;
}

double poseYaw(const geometry_msgs::Pose& pose)
{
  double roll, pitch, yaw;
  tf::Matrix3x3(tf::Quaternion(pose.orientation.x, pose.orientation.y,
                              pose.orientation.z, pose.orientation.w)).getRPY(roll, pitch, yaw);
  return yaw;
}

void publishKeyPoseIfNeeded(const nav_msgs::Odometry& odometry)
{
  if (!publishKeyPose || pubKeyPosePointer == NULL) return;

  bool shouldPublish = !previousKeyPoseValid;
  if (previousKeyPoseValid) {
    const double dx = odometry.pose.pose.position.x - previousKeyPose.position.x;
    const double dy = odometry.pose.pose.position.y - previousKeyPose.position.y;
    const double dz = odometry.pose.pose.position.z - previousKeyPose.position.z;
    const double distance = sqrt(dx * dx + dy * dy + dz * dz);
    const double yawDelta = fabs(normalizeAngle(poseYaw(odometry.pose.pose) - poseYaw(previousKeyPose)));
    shouldPublish = distance >= keyPoseDistance || yawDelta >= keyPoseAngle;
  }

  if (!shouldPublish) return;

  nav_msgs::Odometry keyPose = odometry;
  keyPose.pose.covariance.assign(0.0);
  // TARE uses covariance[0] as the key-pose sequence number.
  keyPose.pose.covariance[0] = static_cast<double>(keyPoseID++);
  pubKeyPosePointer->publish(keyPose);
  previousKeyPose = odometry.pose.pose;
  previousKeyPoseValid = true;
}

void estimateTwistFromPose(nav_msgs::Odometry& odometry)
{
  if (!estimateTwist || !previousOdomValid) return;

  const double existingLinear = fabs(odometry.twist.twist.linear.x) +
                                fabs(odometry.twist.twist.linear.y) +
                                fabs(odometry.twist.twist.linear.z);
  const double existingAngular = fabs(odometry.twist.twist.angular.x) +
                                 fabs(odometry.twist.twist.angular.y) +
                                 fabs(odometry.twist.twist.angular.z);
  if (existingLinear > 1e-6 || existingAngular > 1e-6) return;

  const double dt = (odometry.header.stamp - previousOdom.header.stamp).toSec();
  if (dt <= 1e-3 || dt > 1.0) return;

  const double yaw = poseYaw(odometry.pose.pose);
  const double velocityX = (odometry.pose.pose.position.x - previousOdom.pose.pose.position.x) / dt;
  const double velocityY = (odometry.pose.pose.position.y - previousOdom.pose.pose.position.y) / dt;
  odometry.twist.twist.linear.x = cos(yaw) * velocityX + sin(yaw) * velocityY;
  odometry.twist.twist.linear.y = -sin(yaw) * velocityX + cos(yaw) * velocityY;
  odometry.twist.twist.linear.z =
      (odometry.pose.pose.position.z - previousOdom.pose.pose.position.z) / dt;
  odometry.twist.twist.angular.z =
      normalizeAngle(yaw - poseYaw(previousOdom.pose.pose)) / dt;
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  odomData = *odom;

  if (flipStateEstimation) {
    tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

    pitch = -pitch;
    yaw = -yaw;

    geoQuat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

    odomData.pose.pose.orientation = geoQuat;
    odomData.pose.pose.position.x = odom->pose.pose.position.z;
    odomData.pose.pose.position.y = odom->pose.pose.position.x;
    odomData.pose.pose.position.z = odom->pose.pose.position.y;
  }

  estimateTwistFromPose(odomData);

  // publish odometry messages
  odomData.header.frame_id = worldFrame;
  odomData.child_frame_id = childFrame;
  pubOdometryPointer->publish(odomData);
  publishKeyPoseIfNeeded(odomData);

  previousOdom = odomData;
  previousOdomValid = true;
  odometryReceived = true;
  lastOdometryWallTime = ros::WallTime::now();

  // publish tf messages
  odomTrans.stamp_ = odom->header.stamp;
  odomTrans.frame_id_ = worldFrame;
  odomTrans.child_frame_id_ = childFrame;
  odomTrans.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
  odomTrans.setOrigin(tf::Vector3(odomData.pose.pose.position.x, odomData.pose.pose.position.y, odomData.pose.pose.position.z));

  if (sendTF) {
    if (!reverseTF) {
      tfBroadcasterPointer->sendTransform(odomTrans);
    } else {
      tfBroadcasterPointer->sendTransform(
          tf::StampedTransform(odomTrans.inverse(), odom->header.stamp, childFrame, worldFrame));
    }
  }
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn)
{
  if (!flipRegisteredScan) {
    sensor_msgs::PointCloud2 laserCloudOut = *laserCloudIn;
    laserCloudOut.header.frame_id = worldFrame;
    pubLaserCloudPointer->publish(laserCloudOut);
    cloudReceived = true;
    lastCloudWallTime = ros::WallTime::now();
    return;
  }

  laserCloud->clear();
  pcl::fromROSMsg(*laserCloudIn, *laserCloud);

  if (flipRegisteredScan) {
    int laserCloudSize = laserCloud->points.size();
    for (int i = 0; i < laserCloudSize; i++) {
      float temp = laserCloud->points[i].x;
      laserCloud->points[i].x = laserCloud->points[i].z;
      laserCloud->points[i].z = laserCloud->points[i].y;
      laserCloud->points[i].y = temp;
    }
  }

  // publish registered scan messages
  sensor_msgs::PointCloud2 laserCloud2;
  pcl::toROSMsg(*laserCloud, laserCloud2);
  laserCloud2.header.stamp = laserCloudIn->header.stamp;
  laserCloud2.header.frame_id = worldFrame;
  pubLaserCloudPointer->publish(laserCloud2);
  cloudReceived = true;
  lastCloudWallTime = ros::WallTime::now();
}

void readinessTimerHandler(const ros::WallTimerEvent&)
{
  const ros::WallTime now = ros::WallTime::now();
  const bool inputsFresh = odometryReceived && cloudReceived &&
      (now - lastOdometryWallTime).toSec() <= inputTimeout &&
      (now - lastCloudWallTime).toSec() <= inputTimeout;

  if (!inputsFresh) {
    firstCompleteInputTime = ros::WallTime();
    if (alignmentReady) {
      alignmentReady = false;
      std_msgs::Bool readyMessage;
      readyMessage.data = false;
      pubAlignmentReadyPointer->publish(readyMessage);
    }
    return;
  }

  if (firstCompleteInputTime.isZero()) firstCompleteInputTime = now;
  if (!alignmentReady &&
      (now - firstCompleteInputTime).toSec() >= readinessStableDuration) {
    alignmentReady = true;
    std_msgs::Bool readyMessage;
    readyMessage.data = true;
    pubAlignmentReadyPointer->publish(readyMessage);
    ROS_INFO("DCL exploration input ready: odometry and cloud stable for %.1f seconds",
             readinessStableDuration);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "loamInterface");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("stateEstimationTopic", stateEstimationTopic);
  nhPrivate.getParam("registeredScanTopic", registeredScanTopic);
  nhPrivate.param<string>("outputStateEstimationTopic", outputStateEstimationTopic,
                          outputStateEstimationTopic);
  nhPrivate.param<string>("outputRegisteredScanTopic", outputRegisteredScanTopic,
                          outputRegisteredScanTopic);
  nhPrivate.param<string>("keyPoseTopic", keyPoseTopic, keyPoseTopic);
  nhPrivate.param<string>("alignmentReadyTopic", alignmentReadyTopic, alignmentReadyTopic);
  nhPrivate.param<string>("worldFrame", worldFrame, worldFrame);
  nhPrivate.param<string>("childFrame", childFrame, childFrame);
  nhPrivate.getParam("flipStateEstimation", flipStateEstimation);
  nhPrivate.getParam("flipRegisteredScan", flipRegisteredScan);
  nhPrivate.getParam("sendTF", sendTF);
  nhPrivate.getParam("reverseTF", reverseTF);
  nhPrivate.param<bool>("publishKeyPose", publishKeyPose, publishKeyPose);
  nhPrivate.param<bool>("estimateTwist", estimateTwist, estimateTwist);
  nhPrivate.param<double>("keyPoseDistance", keyPoseDistance, keyPoseDistance);
  nhPrivate.param<double>("keyPoseAngle", keyPoseAngle, keyPoseAngle);
  nhPrivate.param<double>("readinessStableDuration", readinessStableDuration,
                          readinessStableDuration);
  nhPrivate.param<double>("inputTimeout", inputTimeout, inputTimeout);

  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry> (stateEstimationTopic, 5, odometryHandler);

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2> (registeredScanTopic, 5, laserCloudHandler);

  ros::Publisher pubOdometry = nh.advertise<nav_msgs::Odometry> (outputStateEstimationTopic, 5);
  pubOdometryPointer = &pubOdometry;

  tf::TransformBroadcaster tfBroadcaster;
  tfBroadcasterPointer = &tfBroadcaster;

  ros::Publisher pubLaserCloud =
      nh.advertise<sensor_msgs::PointCloud2> (outputRegisteredScanTopic, 5);
  pubLaserCloudPointer = &pubLaserCloud;

  ros::Publisher pubKeyPose = nh.advertise<nav_msgs::Odometry> (keyPoseTopic, 5);
  pubKeyPosePointer = &pubKeyPose;

  ros::Publisher pubAlignmentReady = nh.advertise<std_msgs::Bool> (alignmentReadyTopic, 1, true);
  pubAlignmentReadyPointer = &pubAlignmentReady;
  std_msgs::Bool initialReadyMessage;
  initialReadyMessage.data = false;
  pubAlignmentReady.publish(initialReadyMessage);

  ros::WallTimer readinessTimer = nh.createWallTimer(ros::WallDuration(0.1), readinessTimerHandler);

  ros::spin();

  return 0;
}
