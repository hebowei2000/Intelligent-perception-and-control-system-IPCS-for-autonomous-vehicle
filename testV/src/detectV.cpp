// ros
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
// #include <message_filters/time_synchronizer.h>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
// std
#include <vector>
#include <time.h>
// cv & dnn
#include <opencv2/opencv.hpp>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
// self
#include <testV_msgs/Signal.h>
using namespace std;
using namespace message_filters;
using namespace sensor_msgs;
using namespace darknet_ros_msgs;
using namespace cv;
using namespace testV_msgs;
float thisDis=0,nextDis=0,dist=0;
bool isdanger(){//每隔0.1s探测一次
    dist=nextDis-thisDis;
    thisDis=nextDis;
    return dist>0.5?1:0;
}
void callback(const ImageConstPtr& img, const BoundingBoxes::ConstPtr& bbox){
    //用来更新距离
    // ROS_INFO("深度图时间戳: %d", img->header.stamp);
    // ROS_INFO("bbox图时间戳: %d", bbox->header.stamp);
    try
    {

        vector<BoundingBox* > carBox;
        
        int maxH = 0;
        int index=0;
        int LENG=(sizeof(bbox->bounding_boxes) / sizeof(bbox->bounding_boxes[0]));
        for(int i = 0;i<LENG;i++){
            if(bbox->bounding_boxes[i].Class=="car"&&bbox->bounding_boxes[i].probability>0.9){//确定为车并且置信度要高于一定值
                BoundingBox mid=bbox->bounding_boxes[i];
                carBox.push_back(&mid);
                int Hei=bbox->bounding_boxes[i].ymax-bbox->bounding_boxes[i].ymin;
                //同时取得bbox最大所对应的车辆，该车辆极有可能是距离相机最近的车
                if(Hei>maxH){
                    maxH=Hei;
                    index = i;
                }
            }
        }
        Mat a = cv_bridge::toCvShare(img, "bgr8")->image;
        //只计算和摄像机最近的车
        Rect rect(bbox->bounding_boxes[index].xmin,bbox->bounding_boxes[index].ymin,bbox->bounding_boxes[index].xmax,bbox->bounding_boxes[index].ymax);
        Mat b = a(rect);//这里可以考虑用大津阈值分割出两个部分，求高部分的均值
        
        Scalar meanTor = mean(b);
        nextDis=meanTor.val[0];       
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img->encoding.c_str());
    }
}
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "detect_velocity");//计算速度
	ros::NodeHandle nh;
	Subscriber<Image> depthSub(nh,"/camera/depth/image_raw",1);
    Subscriber<BoundingBoxes> bboxSub(nh,"/darknet_ros/bounding_boxes",1);
    ros::Publisher danPub=nh.advertise<Signal>("/testV/isDanger",1);//向底层发布危险信号   
    // ros::Timer timer=nh.createTimer(ros::Duration(0.1),msgMove);
    //同步消息
    //这里不能指定回调函数，注意这里是BoundingBoxes
    // TimeSynchronizer<Image, BoundingBoxes> sync(depthSub, bboxSub, 10);
    typedef sync_policies::ApproximateTime<Image, BoundingBoxes> MySyncPolicy;
    
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), depthSub, bboxSub);//合并到一起
    sync.registerCallback(boost::bind(&callback, _1, _2));
    ros::Rate loop_rate(10);//一秒判断10次
    while(nh.ok()){
        ros::spinOnce();
        if(isdanger()){
            Signal sig;
            sig.distance = thisDis;
            sig.velocity = dist;
            sig.direction = 0;
            danPub.publish(sig);//向底层发送通知消息
        }
        else{
            ROS_INFO("Safe!");
        }
        loop_rate.sleep();
    }
    return 0;

    
}
