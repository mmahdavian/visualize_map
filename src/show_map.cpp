#include <iostream>
#include "ros/ros.h"
#include <vector>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <opencv2/core/core.hpp>
//#include </home/mohammad/Mohammad_ws/ORB-SLAM/ORB_SLAM3/include/Converter.h>
#include <Eigen/Dense>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;

//    geometry_msgs::PoseStamped pose;

class map_show{

private:
     ros::NodeHandle n;
//     ros::Subscriber object_sub;
//     ros::Subscriber cam_sub;
//     ros::Publisher marker_pub_cam;
     ros::Publisher marker_pub_obj;
     ros::Publisher marker_pub_cam_arr;

     message_filters::Subscriber<sensor_msgs::PointCloud> Mo_sub;
     message_filters::Subscriber<geometry_msgs::PoseStamped> Kf_sub;
     typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud,geometry_msgs::PoseStamped> MySyncPolicy;
     typedef message_filters::Synchronizer<MySyncPolicy> Sync;
     boost::shared_ptr<Sync> sync;


public:
     std::vector<geometry_msgs::Point32> all_objects;        
     float x_obj;
     float y_obj;
     float z_obj;
     float x_cam;
     float y_cam;
     float z_cam;
     float R[4];
     std::vector<visualization_msgs::Marker> all_cam_markers;
  //   std::vector<visualization_msgs::Marker> all_markers;
     visualization_msgs::MarkerArray Markerarr;
     visualization_msgs::MarkerArray Posesarr;
     geometry_msgs::PoseStamped pose;
     
     float init_height = 1;
 
map_show():
n("~"){
     
     Mo_sub.subscribe(n,"/map_objects",1);
     Kf_sub.subscribe(n,"/keyframe_pose",1);
     sync.reset(new Sync(MySyncPolicy(10), Mo_sub,Kf_sub));
     sync->registerCallback(boost::bind(&map_show::Objects,this,_1));
     sync->registerCallback(boost::bind(&map_show::Camera_poses,this,_2));
    
//     object_sub = n.subscribe("/map_objects",1,&map_show::Objects , this);
//     cam_sub = n.subscribe("/keyframe_pose",1,&map_show::Camera_poses , this);
     marker_pub_obj = n.advertise<visualization_msgs::MarkerArray>("objects_marker_array", 1);
//     marker_pub_cam = n.advertise<visualization_msgs::Marker>("camera_pose_marker", 1);
     marker_pub_cam_arr = n.advertise<visualization_msgs::MarkerArray>("camera_pose_marker_array", 1);

}
~map_show(){}

void Objects(const sensor_msgs::PointCloud::ConstPtr& msg1){        
     all_objects = msg1->points;
     Markerarr.markers.resize(all_objects.size());
     for(size_t i=0 ; i<all_objects.size(); i++){
          geometry_msgs::Point32 obj;
          obj = all_objects[i];
          x_obj = obj.x;
          y_obj = obj.y;
          z_obj = obj.z;

          float orb_to_ros[3][3] = {{0,0,1},
                                    {-1,0,0},
                                    {0,-1,0}};
         
          cv::Mat ORB_ROS = cv::Mat(3, 3, CV_32FC1, &orb_to_ros);
          float object_point[3][1] = {{x_obj},{y_obj},{z_obj}};
          cv::Mat OBJ_Point = cv::Mat(3, 1, CV_32FC1, &object_point); 

          cv::Mat OBJ_transformed = ORB_ROS * OBJ_Point;
          float x_obj_tr = OBJ_transformed.at<float>(0,0);
          float y_obj_tr = OBJ_transformed.at<float>(1,0);
          float z_obj_tr = OBJ_transformed.at<float>(2,0);

          Markerarr.markers[i].header.stamp = ros::Time();
          Markerarr.markers[i].header.frame_id = "world";
          Markerarr.markers[i].ns = "object";
          Markerarr.markers[i].id = i;
          Markerarr.markers[i].type = visualization_msgs::Marker::CUBE;
          Markerarr.markers[i].action = visualization_msgs::Marker::ADD;
          Markerarr.markers[i].pose.position.x = z_obj;
          Markerarr.markers[i].pose.position.y = -x_obj;
          Markerarr.markers[i].pose.position.z = -y_obj+init_height;
          Markerarr.markers[i].pose.orientation.x = 0;
          Markerarr.markers[i].pose.orientation.y = 0;
          Markerarr.markers[i].pose.orientation.z = 0;
          Markerarr.markers[i].pose.orientation.w = 0;
          Markerarr.markers[i].scale.x = 0.1;
          Markerarr.markers[i].scale.y = 0.1;
          Markerarr.markers[i].scale.z = 0.1;
          Markerarr.markers[i].color.r = 0.0f;
          Markerarr.markers[i].color.g = 1.0f;
          Markerarr.markers[i].color.b = 0.0f;
          Markerarr.markers[i].color.a = 1.0f;
          Markerarr.markers[i].lifetime = ros::Duration();
     }
     marker_pub_obj.publish(Markerarr);
}

void Camera_poses(const geometry_msgs::PoseStamped::ConstPtr& msg2){
     x_cam = msg2->pose.position.x;
     y_cam = msg2->pose.position.y;
     z_cam = msg2->pose.position.z;

     R[0] = msg2->pose.orientation.x;
     R[1] = msg2->pose.orientation.y;
     R[2] = msg2->pose.orientation.z;
     R[3] = msg2->pose.orientation.w;

     float Rot[3][3] = {{1-2*(R[1]*R[1])-2*(R[2]*R[2]),2*R[0]*R[1]+2*R[2]*R[3],2*R[0]*R[2]-2*R[1]*R[3]},
                        {2*R[0]*R[1]-2*R[2]*R[3],1-2*(R[0]*R[0])-2*(R[2]*R[2]),2*R[1]*R[2]+2*R[0]*R[3]},
                        {2*R[0]*R[2]+2*R[1]*R[3],2*R[1]*R[2]-2*R[0]*R[3],1-2*(R[0]*R[0])-2*(R[1]*R[1])}};

     cv::Mat Rotation = cv::Mat(3, 3, CV_32FC1, &Rot);
     //cout<<"Rotation is: "<<Rotation<<endl;

     float tcw[3][1] = {{-x_cam},{-y_cam},{-z_cam}};
     cv::Mat tcw_ros = cv::Mat(3, 1, CV_32FC1, &tcw);

     cv::Mat tcw_orb = Rotation*tcw_ros;

     float proj[3][4];

     proj[0][0] = Rot[0][0];
     proj[0][1] = Rot[0][1];
     proj[0][2] = Rot[0][2];
     proj[1][0] = Rot[1][0];
     proj[1][1] = Rot[1][1];
     proj[1][2] = Rot[1][2];
     proj[2][0] = Rot[2][0];
     proj[2][1] = Rot[2][1];
     proj[2][2] = Rot[2][2];
     proj[0][3] = tcw_orb.at<float> (0,0);
     proj[1][3] = tcw_orb.at<float> (1,0);
     proj[2][3] = tcw_orb.at<float> (2,0);

     cv::Mat Proj = cv::Mat(3, 4, CV_32FC1, &proj);   //same as ORB_slam
     cv::Mat transform = TransformFromMat(Proj);    //gets transformed to rviz format
     cv::Mat Rwc = transform.rowRange(0,3).colRange(0,3);
     cv::Mat twc = transform.rowRange(0,3).col(3);
     vector<float> q = toQuaternion(Rwc);

     tf::Transform new_transform;
     new_transform.setOrigin(tf::Vector3(twc.at<float>(0, 0), twc.at<float>(0, 1), twc.at<float>(0, 2)));

     tf::Quaternion quaternion(q[0], q[1], q[2], q[3]);
     new_transform.setRotation(quaternion);
     tf::poseTFToMsg(new_transform, pose.pose);

     if (twc.at<float>(0, 0) != 0){
     save_poses(twc.at<float>(0, 0),twc.at<float>(1,0),twc.at<float>(2,0),q);
     }
     visualize_cam();
     
}

void save_poses(float x_cam, float y_cam, float z_cam, vector<float> q){

     visualization_msgs::Marker marker_cam;
     marker_cam.header.stamp = ros::Time();
     marker_cam.header.frame_id = "world";
     marker_cam.ns = "camera";
     marker_cam.id = 1;
     marker_cam.type = visualization_msgs::Marker::ARROW;
     marker_cam.action = visualization_msgs::Marker::ADD;
    // cout<<x_cam<<" "<<y_cam<<" "<<z_cam<<endl;
     marker_cam.pose.position.x = x_cam;
     marker_cam.pose.position.y = y_cam;
     marker_cam.pose.position.z = z_cam+init_height;
     marker_cam.pose.orientation.x = q[0];
     marker_cam.pose.orientation.y = q[1];
     marker_cam.pose.orientation.z = q[2];
     marker_cam.pose.orientation.w = q[3];
     marker_cam.scale.x = 0.2;
     marker_cam.scale.y = 0.05;
     marker_cam.scale.z = 0.05;
     marker_cam.color.r = 0.0f;
     marker_cam.color.g = 1.0f;
     marker_cam.color.b = 0.0f;
     marker_cam.color.a = 1.0f;
     marker_cam.lifetime = ros::Duration();
     all_cam_markers.push_back(marker_cam);
     cout<<"size of cam poses are: "<<all_cam_markers.size()<<endl; 
}


cv::Mat TransformFromMat(cv::Mat position_mat) {
     cv::Mat rotation(3,3,CV_32F);
     cv::Mat translation(3,1,CV_32F);

     rotation = position_mat.rowRange(0,3).colRange(0,3);
     translation = position_mat.rowRange(0,3).col(3);

     float orb_to_ros[3][3] = {{0,0,1},
                               {-1,0,0},
                               {0,-1,0}};


     cv::Mat ORB_ROS = cv::Mat(3, 3, CV_32FC1, &orb_to_ros);

     //Transform from orb coordinate system to ros coordinate system on camera coordinates
     cv::Mat camera_rotation = ORB_ROS * ((ORB_ROS*rotation).t());
     cv::Mat camera_translation = -ORB_ROS * ((ORB_ROS*rotation).t())*(ORB_ROS*translation);
     cv::Mat Transform_matrix(3,4,CV_32F);
     cv::hconcat(camera_rotation, camera_translation, Transform_matrix);

//     cout<<position_mat<<endl;
//     cv::Mat Transform_matrix = ORB_ROS * position_mat;
     return Transform_matrix;
}

std::vector<float> toQuaternion(const cv::Mat &M)
{
    Eigen::Matrix<double,3,3> eigMat = toMatrix3d(M);
    Eigen::Quaterniond q(eigMat);

    std::vector<float> v(4);
    v[0] = q.x();
    v[1] = q.y();
    v[2] = q.z();
    v[3] = q.w();

    return v;
}

Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3)
{
    Eigen::Matrix<double,3,3> M;

    M << cvMat3.at<float>(0,0), cvMat3.at<float>(0,1), cvMat3.at<float>(0,2),
         cvMat3.at<float>(1,0), cvMat3.at<float>(1,1), cvMat3.at<float>(1,2),
         cvMat3.at<float>(2,0), cvMat3.at<float>(2,1), cvMat3.at<float>(2,2);

    return M;
}

/*tf::Transform TransformFromMat (cv::Mat position_mat) {
  cv::Mat rotation(3,3,CV_32F);
  cv::Mat translation(3,1,CV_32F);

  rotation = position_mat.rowRange(0,3).colRange(0,3);
  translation = position_mat.rowRange(0,3).col(3);

  tf::Matrix3x3 tf_camera_rotation (rotation.at<float> (0,0), rotation.at<float> (0,1), rotation.at<float> (0,2),
                                    rotation.at<float> (1,0), rotation.at<float> (1,1), rotation.at<float> (1,2),
                                    rotation.at<float> (2,0), rotation.at<float> (2,1), rotation.at<float> (2,2)
                                   );

  tf::Vector3 tf_camera_translation (translation.at<float> (0), translation.at<float> (1), translation.at<float> (2));

  //Coordinate transformation matrix from orb coordinate system to ros coordinate system
  const tf::Matrix3x3 tf_orb_to_ros (0, 0, 1,
                                    -1, 0, 0,
                                     0,-1, 0);

  //Transform from orb coordinate system to ros coordinate system on camera coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  //Inverse matrix
  tf_camera_rotation = tf_camera_rotation.transpose();
  tf_camera_translation = -(tf_camera_rotation*tf_camera_translation);

  //Transform from orb coordinate system to ros coordinate system on map coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  return tf::Transform (tf_camera_rotation, tf_camera_translation);
}
*/
/*
void visualize_obj(float x_obj,float y_obj,float z_obj)
{
     visualization_msgs::Marker marker_obj;
     marker_obj.header.stamp = ros::Time();
     marker_obj.header.frame_id = "map";
     marker_obj.ns = "object";
     marker_obj.id = 0;
     marker_obj.type = visualization_msgs::Marker::CUBE;
     marker_obj.action = visualization_msgs::Marker::ADD;
     marker_obj.pose.position.x = x_obj;
     marker_obj.pose.position.y = y_obj;
     marker_obj.pose.position.z = z_obj;
     marker_obj.pose.orientation.x = 0;
     marker_obj.pose.orientation.y = 0;
     marker_obj.pose.orientation.z = 0;
     marker_obj.pose.orientation.w = 0;
     marker_obj.scale.x = 0.5;
     marker_obj.scale.y = 0.5;
     marker_obj.scale.z = 0.5;
     marker_obj.color.r = 0.0f;
     marker_obj.color.g = 1.0f;
     marker_obj.color.b = 0.0f;
     marker_obj.color.a = 1.0f;
     marker_obj.lifetime = ros::Duration();
     all_markers.push_back(marker_obj);
     marker_pub.publish(all_markers);
}



void visualize_cam(float x_cam, float y_cam, float z_cam, vector<float> q)
{

     visualization_msgs::Marker marker_cam;
     marker_cam.header.stamp = ros::Time();
     marker_cam.header.frame_id = "world";
     marker_cam.ns = "camera";
     marker_cam.id = 1;
     marker_cam.type = visualization_msgs::Marker::ARROW;
     marker_cam.action = visualization_msgs::Marker::ADD;
    // cout<<x_cam<<" "<<y_cam<<" "<<z_cam<<endl;
     marker_cam.pose.position.x = x_cam;
     marker_cam.pose.position.y = y_cam;
     marker_cam.pose.position.z = z_cam+init_height;
     marker_cam.pose.orientation.x = q[0];
     marker_cam.pose.orientation.y = q[1];
     marker_cam.pose.orientation.z = q[2];
     marker_cam.pose.orientation.w = q[3];
     marker_cam.scale.x = 0.2;
     marker_cam.scale.y = 0.05;
     marker_cam.scale.z = 0.05;
     marker_cam.color.r = 0.0f;
     marker_cam.color.g = 1.0f;
     marker_cam.color.b = 0.0f;
     marker_cam.color.a = 1.0f;
     marker_cam.lifetime = ros::Duration();
     marker_pub_cam.publish(marker_cam);
}
*/

void visualize_cam()
{
     Posesarr.markers.resize(all_cam_markers.size());
     for(size_t i=0;i<all_cam_markers.size();i++){

          visualization_msgs::Marker cur_cam = all_cam_markers[i];
          Posesarr.markers[i].header.stamp = ros::Time();
          Posesarr.markers[i].header.frame_id = "world";
          Posesarr.markers[i].ns = "camera";
          Posesarr.markers[i].id = i;
          Posesarr.markers[i].type = visualization_msgs::Marker::ARROW;
          Posesarr.markers[i].action = visualization_msgs::Marker::ADD;
          Posesarr.markers[i].pose.position.x = cur_cam.pose.position.x;
          Posesarr.markers[i].pose.position.y = cur_cam.pose.position.y;
          Posesarr.markers[i].pose.position.z = cur_cam.pose.position.z;
          Posesarr.markers[i].pose.orientation.x = cur_cam.pose.orientation.x;
          Posesarr.markers[i].pose.orientation.y = cur_cam.pose.orientation.y;
          Posesarr.markers[i].pose.orientation.z = cur_cam.pose.orientation.z;
          Posesarr.markers[i].pose.orientation.w = cur_cam.pose.orientation.w;
          Posesarr.markers[i].scale.x = cur_cam.scale.x;
          Posesarr.markers[i].scale.y = cur_cam.scale.y;
          Posesarr.markers[i].scale.z = cur_cam.scale.z;
          Posesarr.markers[i].color.r = 1.0f;
          Posesarr.markers[i].color.g = 0.0f;
          Posesarr.markers[i].color.b = 0.0f;
          Posesarr.markers[i].color.a = 1.0f;
          Posesarr.markers[i].lifetime = ros::Duration();
     }
     marker_pub_cam_arr.publish(Posesarr);
}


};

int main(int argc, char** argv) {

     ros::init(argc, argv, "map_visualizer");
     map_show rs;
     ros::Rate r(30);
     while (ros::ok()){
            ros::spinOnce();
            r.sleep();
     }
     return 0;
}
