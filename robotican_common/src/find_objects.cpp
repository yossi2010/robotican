


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <std_srvs/SetBool.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <robotican_common/FindObjectDynParamConfig.h>
#include <dynamic_reconfigure/server.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

using namespace cv;

bool debug_vision=false;

bool update_cb(std_srvs::SetBool::Request  &req,std_srvs::SetBool::Response &res);
bool find_object(Mat input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,Point3d *obj,std::string frame);
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
void dynamicParamCallback(robotican_common::FindObjectDynParamConfig &config, uint32_t level);
void arm_msgCallback(const boost::shared_ptr<const geometry_msgs::PoseStamped>& point_ptr);


std::vector<moveit_msgs::CollisionObject> col_objects;
moveit::planning_interface::PlanningSceneInterface *planning_scene_interface_ptr;
tf::TransformListener *listener_ptr;

bool timeout=true;
bool update=false;

std::string object_name;
double object_r,object_h;
ros::Time detect_t;

std::string depth_topic;
bool have_object=false;

ros::Publisher object_pub;
image_transport::Publisher result_image_pub;
image_transport::Publisher object_image_pub;
image_transport::Publisher bw_image_pub;

//red
int minH=3,maxH=160;
int minS=70,maxS=255;

int minV=10,maxV=255;
int minA=200,maxA=50000;
int gaussian_ksize=0;
int gaussian_sigma=0;
int morph_size=0;

int inv_H=1;

bool update_cb(std_srvs::SetBool::Request  &req,
                 std_srvs::SetBool::Response &res) {

    update=req.data;
    res.success=true;
    if (update)  res.message="update collision is ON";
    else res.message="update collision is OFF";
    return true;
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {



    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    pcl::fromROSMsg (*input, cloud);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudp (new pcl::PointCloud<pcl::PointXYZRGBA> (cloud));

     if (cloudp->empty()) {

         ROS_WARN("empty cloud");
         return;
     }

    sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);
    pcl::toROSMsg (*input, *image_msg);
    image_msg->header.stamp = input->header.stamp;
    image_msg->header.frame_id = input->header.frame_id;

cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
 Mat result=cv_ptr->image;

    Point3d obj;
    have_object= find_object(result,cloudp,&obj,input->header.frame_id);

    waitKey(1);



    if (have_object) {


        geometry_msgs::PoseStamped target_pose2;
        target_pose2.header.frame_id=input->header.frame_id;
        target_pose2.header.stamp=ros::Time::now();
        target_pose2.pose.position.x =obj.x;
        target_pose2.pose.position.y = obj.y;
        target_pose2.pose.position.z = obj.z+object_r;
       //  std::printf("---------> OBJECT: [%f , %f , %f]\n",target_pose2.pose.position.x,target_pose2.pose.position.y,target_pose2.pose.position.z);
       target_pose2.pose.orientation.w=1;
        object_pub.publish(target_pose2);




    }


}

void arm_msgCallback(const boost::shared_ptr<const geometry_msgs::PoseStamped>& point_ptr)
{

    try
    {
           geometry_msgs::PoseStamped base_object_pose;
        listener_ptr->transformPose("base_footprint", *point_ptr, base_object_pose);
        base_object_pose.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0);
       // base_object_pose.header.frame_id="base_footprint";
      //  base_object_pose.header.stamp=ros::Time::now();

detect_t=ros::Time::now();

     // col_objects[0].header.stamp=ros::Time::now();
    //  col_objects[0].primitive_poses.clear();
     if(col_objects[0].primitive_poses.size()==0) {
         col_objects[0].primitive_poses.push_back(base_object_pose.pose);
     }

     else  {
         col_objects[0].primitive_poses[0]=base_object_pose.pose;
     }
       col_objects[0].operation = col_objects[0].ADD;

       if (update) planning_scene_interface_ptr->addCollisionObjects(col_objects);

       if (timeout) {
           ROS_DEBUG("Found object");
           timeout=false;
       }
    }
    catch (tf::TransformException &ex)
    {
        printf ("Failure %s\n", ex.what()); //Print exception which was caught
    }
}

bool find_object(Mat input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudp,Point3d *pr,std::string frame) {

    Mat hsv,filtered,bw,mask;

    cv_bridge::CvImage out_msg;
    out_msg.header.stamp=ros::Time::now();
    out_msg.header.frame_id=  frame;

    cvtColor(input,hsv,CV_BGR2HSV);


    if (inv_H) {
        Mat lower_hue_range;
        Mat upper_hue_range;
        inRange(hsv, cv::Scalar(0, minS, minV), cv::Scalar(minH, maxS, maxV), lower_hue_range);
        inRange(hsv, cv::Scalar(maxH, minS, minV), cv::Scalar(179, maxS, maxV), upper_hue_range);
        // Combine the above two images

        addWeighted(lower_hue_range, 1.0, upper_hue_range, 1.0, 0.0, mask);
    }
    else{
        //if not red use:
        inRange(hsv,Scalar(minH,minS,minV),Scalar(maxH,maxS,maxV),mask);
    }
    hsv.copyTo(filtered,mask);
    cvtColor(filtered,filtered,CV_HSV2BGR);

    out_msg.image    = filtered;
    out_msg.encoding = "bgr8";
    object_image_pub.publish(out_msg.toImageMsg());

    mask.copyTo(bw);
    if (gaussian_ksize>0) {
        if (gaussian_ksize % 2 == 0) gaussian_ksize++;
        GaussianBlur( bw, bw, Size(gaussian_ksize,gaussian_ksize), gaussian_sigma , 0);
    }


    if (morph_size>0) {
        Mat element = getStructuringElement( MORPH_ELLIPSE, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
        morphologyEx( bw, bw, MORPH_CLOSE, element, Point(-1,-1), 1 );
    }



    out_msg.image    = bw;
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    bw_image_pub.publish(out_msg.toImageMsg());


    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    findContours(bw, contours,hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    double largest_area=0;
    int largest_contour_index=0;
    for( int i = 0; i< contours.size(); i++ )
    {
        double area0 = abs(contourArea(contours[i]));
        if(area0>largest_area){
            largest_area=area0;
            largest_contour_index=i;
        }
    }
    bool ok=false;
    if ((largest_area>minA)&&(largest_area<maxA)) {

        //
        drawContours(input, contours, (int)largest_contour_index,  Scalar(255,0,0), 3, 8, hierarchy, 0);
        Moments mu=moments( contours[largest_contour_index], true );
        Point2f mc = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
        circle( input, mc, 4, Scalar(0,0,255), -1, 8, 0 );
        int pcl_index = ((int)(mc.y)*input.cols) + (int)(mc.x);
        circle( input, mc, 8, Scalar(0,255,0), -1, 8, 0 );

        pr->x=cloudp->points[pcl_index].x;
        pr->y=cloudp->points[pcl_index].y;
        pr->z=cloudp->points[pcl_index].z;
        char str[100];
        if (isnan (pr->x) || isnan (pr->y) || isnan (pr->z) ) {
            sprintf(str,"NaN");
            ok=false;
        }
        else {
            sprintf(str,"[%.3f,%.3f,%.3f] A=%lf",pr->x,pr->y,pr->z,largest_area);
            ok=true;
        }
        putText( input, str, mc, CV_FONT_HERSHEY_COMPLEX, 1, Scalar(255,255,255), 1, 8);

    }


    out_msg.image    = input;
    out_msg.encoding = "bgr8";
    result_image_pub.publish(out_msg.toImageMsg());

    return ok;
}

void dynamicParamCallback(robotican_common::FindObjectDynParamConfig &config, uint32_t level) {
    minH = config.H_min;
    maxH = config.H_max;

    minS = config.S_min;
    maxS = config.S_max;

    minV = config.V_min;
    maxV = config.V_max;

    minA = config.A_min;
    maxA = config.A_max;

    gaussian_ksize = config.gaussian_ksize;
    gaussian_sigma = config.gaussian_sigma;

    morph_size = config.morph_size;

    inv_H = config.invert_Hue;
}


void on_trackbar( int, void* ){}


int main(int argc, char **argv) {

    ros::init(argc, argv, "find_objects_node");
    ros::NodeHandle n("~");
    ROS_INFO("Hello");

    std::string object_id;
    n.param<double>("object_r", object_r, 0.025);
    n.param<double>("object_h", object_h, 0.15);
    n.param<std::string>("object_id", object_id, "can");
    n.param<std::string>("object_name", object_name, "object");
    n.param<std::string>("depth_topic", depth_topic, "/kinect2/qhd/points");

    dynamic_reconfigure::Server<robotican_common::FindObjectDynParamConfig> dynamicServer;
    dynamic_reconfigure::Server<robotican_common::FindObjectDynParamConfig>::CallbackType callbackFunction;

    callbackFunction = boost::bind(&dynamicParamCallback, _1, _2);
    dynamicServer.setCallback(callbackFunction);

    image_transport::ImageTransport it_(n);

    result_image_pub = it_.advertise("result", 1);
    object_image_pub = it_.advertise("hsv_filterd", 1);
    bw_image_pub = it_.advertise("bw", 1);

string uc="/update_collision/"+object_name;
ros::ServiceServer service = n.advertiseService(uc, update_cb);

    ros::Subscriber pcl_sub = n.subscribe(depth_topic, 1, cloud_cb);
string topic="/detected_objects/"+object_name;
    object_pub=n.advertise<geometry_msgs::PoseStamped>(topic, 2, true);

    tf::TransformListener listener;
    listener_ptr=&listener;

    message_filters::Subscriber<geometry_msgs::PoseStamped> point_sub_;
    point_sub_.subscribe(n, topic, 10);

    tf::MessageFilter<geometry_msgs::PoseStamped> tf_filter(point_sub_, listener, "base_footprint", 10);
    tf_filter.registerCallback( boost::bind(arm_msgCallback, _1) );

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface_ptr=&planning_scene_interface;

    moveit_msgs::CollisionObject can_collision_object;
    can_collision_object.header.frame_id = "base_footprint";
    can_collision_object.id = object_id;
    shape_msgs::SolidPrimitive object_primitive;
    object_primitive.type = object_primitive.CYLINDER;
    object_primitive.dimensions.resize(2);
    object_primitive.dimensions[0] = object_h;
    object_primitive.dimensions[1] = object_r;

    can_collision_object.primitives.push_back(object_primitive);
    col_objects.push_back(can_collision_object);


    if (debug_vision) {
        namedWindow("Trackbars",CV_WINDOW_AUTOSIZE);              // trackbars window
        createTrackbar( "H min", "Trackbars", &minH, 180, on_trackbar );
        createTrackbar( "H max", "Trackbars", &maxH, 180, on_trackbar );
        createTrackbar( "S min", "Trackbars", &minS, 255, on_trackbar );
        createTrackbar( "S max", "Trackbars", &maxS, 255, on_trackbar );
        createTrackbar( "V min", "Trackbars", &minV, 255, on_trackbar );
        createTrackbar( "V max", "Trackbars", &maxV, 255, on_trackbar );
        createTrackbar( "gaussian_ksize", "Trackbars", &gaussian_ksize, 255, on_trackbar );
        createTrackbar( "gaussian_sigma", "Trackbars", &gaussian_sigma, 255, on_trackbar );
        createTrackbar( "A min", "Trackbars", &minA, 50000, on_trackbar );
        createTrackbar( "A max", "Trackbars", &maxA, 50000, on_trackbar );
        createTrackbar( "morph_size", "Trackbars", &morph_size, 50, on_trackbar );
        createTrackbar( "invert_Hue", "Trackbars", &inv_H, 1, on_trackbar );
    }


    ROS_INFO("Ready to find objects!");
ros::Rate r(10);

while (ros::ok()) {

        double dt=(ros::Time::now()-detect_t).toSec();
        if ((dt>2.0)&&(!timeout)&&(update)) {
            timeout=true;
            ROS_DEBUG("Object detection timeout");
           col_objects[0].operation = col_objects[0].REMOVE;
            planning_scene_interface_ptr->addCollisionObjects(col_objects);

        }
        r.sleep();
ros::spinOnce();
}

    return 0;
}

