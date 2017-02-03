#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

aruco::MarkerDetector MDetector;
vector<aruco::Marker> Markers;
cv::Mat frame;

cv::Vec3f position_robot_1, position_robot_2, position_robot_3;
tf::Quaternion orientation_robot_1, orientation_robot_2, orientation_robot_3;

double scale = 0.01;

tf::Transform getTf(const cv::Mat &Rvec, const cv::Mat &Tvec)
{
    cv::Mat rot(3, 3, CV_32FC1);
    cv::Rodrigues(Rvec, rot);

    cv::Mat rotate_to_sys(3, 3, CV_32FC1);
    /**
    /* Fixed the rotation to meet the ROS system
    /* Doing a basic rotation around X with theta=PI
    /* By Sahloul
    /* See http://en.wikipedia.org/wiki/Rotation_matrix for details
    */

    //	1	0	0
    //	0	-1	0
    //	0	0	-1
    rotate_to_sys.at<float>(0,0) = 1.0;
    rotate_to_sys.at<float>(0,1) = 0.0;
    rotate_to_sys.at<float>(0,2) = 0.0;
    rotate_to_sys.at<float>(1,0) = 0.0;
    rotate_to_sys.at<float>(1,1) = -1.0;
    rotate_to_sys.at<float>(1,2) = 0.0;
    rotate_to_sys.at<float>(2,0) = 0.0;
    rotate_to_sys.at<float>(2,1) = 0.0;
    rotate_to_sys.at<float>(2,2) = -1.0;
    rot = rot*rotate_to_sys.t();

    tf::Matrix3x3 tf_rot(rot.at<float>(0,0), rot.at<float>(0,1), rot.at<float>(0,2),
                         rot.at<float>(1,0), rot.at<float>(1,1), rot.at<float>(1,2),
                         rot.at<float>(2,0), rot.at<float>(2,1), rot.at<float>(2,2));

    tf::Vector3 tf_orig(Tvec.at<float>(0,0), Tvec.at<float>(1,0), Tvec.at<float>(2,0));

    return tf::Transform(tf_rot, tf_orig);
}

tf::Transform arucoMarker2Tf(const aruco::Marker &marker)
{
    cv::Mat rot(3, 3, CV_64FC1);
    cv::Mat Rvec64;
    marker.Rvec.convertTo(Rvec64, CV_64FC1);
    cv::Rodrigues(Rvec64, rot);
    cv::Mat tran64;
    marker.Tvec.convertTo(tran64, CV_64FC1);

    cv::Mat rotate_to_ros(3, 3, CV_64FC1);
    // -1 0 0
    // 0 0 1
    // 0 1 0
    rotate_to_ros.at<double>(0,0) = -1.0;
    rotate_to_ros.at<double>(0,1) = 0.0;
    rotate_to_ros.at<double>(0,2) = 0.0;
    rotate_to_ros.at<double>(1,0) = 0.0;
    rotate_to_ros.at<double>(1,1) = 0.0;
    rotate_to_ros.at<double>(1,2) = 1.0;
    rotate_to_ros.at<double>(2,0) = 0.0;
    rotate_to_ros.at<double>(2,1) = 1.0;
    rotate_to_ros.at<double>(2,2) = 0.0;
    rot = rot*rotate_to_ros.t();

    tf::Matrix3x3 tf_rot(rot.at<double>(0,0), rot.at<double>(0,1), rot.at<double>(0,2),
                         rot.at<double>(1,0), rot.at<double>(1,1), rot.at<double>(1,2),
                         rot.at<double>(2,0), rot.at<double>(2,1), rot.at<double>(2,2));

    tf::Vector3 tf_orig(tran64.at<double>(0,0), tran64.at<double>(1,0), tran64.at<double>(2,0));


    return tf::Transform(tf_rot, tf_orig);
}
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_bridge::toCvShare(msg, "bgr8")->image.copyTo(frame);

        MDetector.detect(frame, Markers);

        //for each marker, draw info and its boundaries in the image
        for (unsigned int i=0;i<Markers.size();i++) {

            cv::Mat m33(3,3,CV_32FC1);
            cv::Rodrigues(Markers[i].Rvec, m33)  ;
/*
            tf::Matrix3x3 m( m33.at<double>(0, 0), m33.at<double>(0, 1), m33.at<double>(0, 2),
                             m33.at<double>(1, 0), m33.at<double>(1, 1), m33.at<double>(1, 2),
                             m33.at<double>(2, 0), m33.at<double>(2, 1), m33.at<double>(2, 2));
*/
            tf::Quaternion quad_orientation;
            //m.getRotation(quad_orientation);

            tf::Transform tf_robot = getTf(Markers[i].Rvec, Markers[i].Tvec);
            //tf::Transform tf_robot = arucoMarker2Tf(Markers[i]);
            quad_orientation = tf_robot.getRotation();

            ROS_INFO("orientation %f, %f, %f, %f", quad_orientation.getX(), quad_orientation.getY(), quad_orientation.getZ(), quad_orientation.getW() );
            //ROS_INFO("orientation %f, %f, %f", Markers[i].Rvec.at<double>(0), Markers[i].Rvec.at<double>(1), Markers[i].Rvec.at<double>(2) );

            cv::Vec3f position = cv::Vec3f(Markers[i].getCenter().x * scale, Markers[i].getCenter().y * scale, 1.0);

            switch(Markers[i].id) {
                case 10:
                    position_robot_1 = position;
                    orientation_robot_1 = quad_orientation;
                    break;
                case 20:
                    position_robot_2 = position;
                    orientation_robot_2 = quad_orientation;
                    break;
                case 30:
                    position_robot_3 = position;
                    orientation_robot_3 = quad_orientation;
                    break;
                default:
                    // No correct marker
                    break;
            }

            Markers[i].draw(frame, cv::Scalar(0,0,255),2);

        }

        cv::imshow("Camera_image", frame);
        cv::waitKey(10);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void send_robot_odometry(cv::Vec3f position, tf::Quaternion orientation, ros::Publisher &odom_pub, tf::TransformBroadcaster &odom_broadcaster){
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;

    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = position[0];
    odom_trans.transform.translation.y = position[1];
    odom_trans.transform.translation.z = position[2];


    odom_trans.transform.rotation.x = orientation.getX();
    odom_trans.transform.rotation.y = orientation.getY();
    odom_trans.transform.rotation.z = orientation.getZ();
    odom_trans.transform.rotation.w = orientation.getW();

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    //set the position
    odom.pose.pose.position.x = position[0];
    odom.pose.pose.position.y = position[1];
    odom.pose.pose.position.z = position[2];

    odom.pose.pose.orientation.x = orientation.getX();
    odom.pose.pose.orientation.y = orientation.getY();
    odom.pose.pose.orientation.z = orientation.getZ();
    odom.pose.pose.orientation.w = orientation.getW();

    //publish the message
    odom_pub.publish(odom);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    // create a OpenCV window with camera image
    cv::namedWindow("Camera_image");
    cv::startWindowThread();

    // create publishers for odometry
    ros::Publisher odom_pub_robot_1 = nh.advertise<nav_msgs::Odometry>("/robot_1/odom", 1);
    ros::Publisher odom_pub_robot_2 = nh.advertise<nav_msgs::Odometry>("/robot_2/odom", 1);
    ros::Publisher odom_pub_robot_3 = nh.advertise<nav_msgs::Odometry>("/robot_3/odom", 1);

    tf::TransformBroadcaster odom_broadcaster_robot_1;
    tf::TransformBroadcaster odom_broadcaster_robot_2;
    tf::TransformBroadcaster odom_broadcaster_robot_3;

    // Subscribe for camera images
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/robot_map_camera/image_raw", 1, imageCallback);

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    // Define frame rate
    ros::Rate r(100.0);

    while(nh.ok()){
        ros::spinOnce();               // check for incoming messages

        send_robot_odometry(position_robot_1, orientation_robot_1, odom_pub_robot_1, odom_broadcaster_robot_1);
        send_robot_odometry(position_robot_2, orientation_robot_2, odom_pub_robot_2, odom_broadcaster_robot_2);
        send_robot_odometry(position_robot_3, orientation_robot_3, odom_pub_robot_3, odom_broadcaster_robot_3);

        last_time = current_time;
        r.sleep();
    }

    //ros::spin();
    cv::destroyWindow("markers");
}