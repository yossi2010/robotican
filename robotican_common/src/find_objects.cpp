


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Empty.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <robotican_common/FindObjectDynParamConfig.h>
#include <dynamic_reconfigure/server.h>


using namespace cv;

bool debug_vision=false;

bool find_object(Mat input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,Point3d *obj,std::string frame);
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
void dynamicParamCallback(robotican_common::FindObjectDynParamConfig &config, uint32_t level);



std::string object_name;


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

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {



/*
    int image_w=0,image_h=0;
   //   std::cout << input->width <<"   "<< input->height  <<std::endl;
    if ((input->width==960*540)||((input->width==960)&&(input->height==540))) {
        image_w=960;
        image_h=540;
    }
    else if ((input->width==640*480)||((input->width==640)&&(input->height==480))) {
        image_w=640;
        image_h=480;
    }
    else if (input->width==1920*1080) {
        image_w=1920;
        image_h=1080;
    }
    else if (input->width==512*424) {
        image_w=512;
        image_h=424;
    }
    else {
        ROS_ERROR("Unknown image resolutuin");
        return;
    }
    */
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
         /*
    Mat result = Mat(image_h, image_w, CV_8UC3);
    if (!cloudp->empty()) {
        for (int h=0; h<image_h; h++) {
            for (int w=0; w<image_w; w++) {
                int pcl_index = (h*image_w) + w;
                pcl::PointXYZRGBA point = cloudp->at(pcl_index);
                if (point.z>0.1) {
                    // Eigen::Vector3i rgb = point.getRGBVector3i();
                    result.at<cv::Vec3b>(h,w)[0] = point.b;
                    result.at<cv::Vec3b>(h,w)[1] = point.g;
                    result.at<cv::Vec3b>(h,w)[2] = point.r;
                }
                else {
                    result.at<cv::Vec3b>(h,w)[0]=0;
                    result.at<cv::Vec3b>(h,w)[1]=0;
                    result.at<cv::Vec3b>(h,w)[2]=0;
                }
            }
        }

    }
    else {
        ROS_WARN("empty cloud");
        return;
    }
*/

    Point3d obj;
    have_object= find_object(result,cloudp,&obj,input->header.frame_id);

    waitKey(1);



    if (have_object) {


        geometry_msgs::PoseStamped target_pose2;
        target_pose2.header.frame_id=input->header.frame_id;
        target_pose2.header.stamp=ros::Time::now();
        target_pose2.pose.position.x =obj.x;
        target_pose2.pose.position.y = obj.y;
        target_pose2.pose.position.z = obj.z;
       //  std::printf("---------> OBJECT: [%f , %f , %f]\n",target_pose2.pose.position.x,target_pose2.pose.position.y,target_pose2.pose.position.z);
       target_pose2.pose.orientation.w=1;
        object_pub.publish(target_pose2);
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

    ros::Subscriber pcl_sub = n.subscribe(depth_topic, 1, cloud_cb);
string topic="/detected_objects/"+object_name;
    object_pub=n.advertise<geometry_msgs::PoseStamped>(topic, 2, true);

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



        ros::spin();


    return 0;
}

