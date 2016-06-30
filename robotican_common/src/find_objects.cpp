


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Empty.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


using namespace cv;

bool debug_vision=true;
void imageCb(const sensor_msgs::ImageConstPtr& msg);
bool find_object(Mat input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,Point3d *obj);
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);

tf::StampedTransform obj_transform;


int image_w=0,image_h=0;

bool have_object=false;
ros::Publisher object_pub;

image_transport::Publisher result_image_pub;
image_transport::Publisher object_image_pub;
image_transport::Publisher bw_image_pub;

image_transport::Subscriber image_sub;
;
//red
int minH=3,maxH=160;
int minS=70,maxS=255;
int minV=70,maxV=255;

int minA=1000,maxA=50000;
int gaussian_ksize=0;
int gaussian_sigma=0;
int morph_size=0;
int inv_H=1;

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Mat bgr=cv_ptr->image;

    if (image_w==0) image_w=bgr.cols;
    if (image_h==0) image_h=bgr.rows;

}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {

    if ((image_w==0)||(image_h==0)) return;

    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    pcl::fromROSMsg (*input, cloud);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudp (new pcl::PointCloud<pcl::PointXYZRGBA> (cloud));

    Mat result;

    result = Mat(image_h, image_w, CV_8UC3);

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


    Point3d obj;
    have_object= find_object(result,cloudp,&obj);

    waitKey(1);



    if (have_object) {

        // obj_tf_ok=have_object;

        obj_transform.setOrigin( tf::Vector3(obj.z,-obj.x,-obj.y) );
        obj_transform.stamp_=ros::Time::now();

        geometry_msgs::PoseStamped target_pose2;
        target_pose2.header.frame_id="kinect2_depth_frame";
        target_pose2.header.stamp=ros::Time::now();
        target_pose2.pose.position.x =obj_transform.getOrigin().x();
        target_pose2.pose.position.y = obj_transform.getOrigin().y();
        target_pose2.pose.position.z = obj_transform.getOrigin().z();
        tf::quaternionTFToMsg( obj_transform.getRotation(), target_pose2.pose.orientation);
        object_pub.publish(target_pose2);
    }


}

bool find_object(Mat input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudp,Point3d *pr) {

    Mat hsv,filtered,bw,mask;

    cv_bridge::CvImage out_msg;
    out_msg.header.stamp=ros::Time::now();
    out_msg.header.frame_id="kinect2_depth_optical_frame";

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
        if (isnan (pr->x) || isnan (pr->y) || isnan (pr->z) ) sprintf(str,"NaN");
        else sprintf(str,"[%.3f,%.3f,%.3f] A=%lf",pr->x,pr->y,pr->z,largest_area);
        putText( input, str, mc, CV_FONT_HERSHEY_COMPLEX, 1, Scalar(255,255,255), 1, 8);
        ok=true;
    }


    out_msg.image    = input;
    out_msg.encoding = "bgr8";
    result_image_pub.publish(out_msg.toImageMsg());

    return ok;
}

void on_trackbar( int, void* ){}




int main(int argc, char **argv) {

    ros::init(argc, argv, "find_objects_node");
    // ros::AsyncSpinner spinner(2);
    // spinner.start();
    ros::NodeHandle n;
    ROS_INFO("Hello");
    // n.getParam("distance_from_object", distance_from_object);

    image_transport::ImageTransport it_(n);

    image_sub = it_.subscribe("kinect2/hd/image_color", 1,imageCb);
    result_image_pub = it_.advertise("find_objects/result", 1);
    object_image_pub = it_.advertise("find_objects/hsv_filterd", 1);
    bw_image_pub = it_.advertise("find_objects/bw", 1);

    ros::Subscriber pcl_sub = n.subscribe("kinect2/hd/points", 1, cloud_cb);

     object_pub=n.advertise<geometry_msgs::PoseStamped>("object", 2, true);

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
    tf::TransformBroadcaster br;

    ros::Rate r(50); // 50 hz

    ROS_INFO("Ready to find objects!");

    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    obj_transform.setRotation(q);
    obj_transform.frame_id_="kinect2_depth_frame";
    obj_transform.child_frame_id_="object";

    while (ros::ok())
    {

        if (have_object) {
            obj_transform.stamp_=ros::Time::now();
            br.sendTransform(obj_transform);
        }

        ros::spinOnce();

        r.sleep();
    }

    return 0;
}

