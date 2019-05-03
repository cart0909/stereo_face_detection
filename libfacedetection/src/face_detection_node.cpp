#include <chrono>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <cv_bridge/cv_bridge.h>
#include "facedetectcnn.h"

//define the buffer size. Do not change the size!
#define DETECT_BUFFER_SIZE 0x20000

using namespace sensor_msgs;

class Node {
public:
    Node(ros::NodeHandle& nh) {
        pub_face = nh.advertise<Image>("/face/image_raw", 1000);
        pub_bbox = nh.advertise<PointCloud>("/face/bbox", 1000);
        pBuffer = (unsigned char *)malloc(DETECT_BUFFER_SIZE);
        ROS_ASSERT_MSG(pBuffer, "Can not alloc buffer.");
    }

    ~Node() {
        free(pBuffer);
    }

    void Callback(const sensor_msgs::ImageConstPtr& img_msg) {
        cv::Mat image = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
        ///////////////////////////////////////////
        // CNN face detection
        // Best detection rate
        //////////////////////////////////////////
        //!!! The input image must be a BGR one (three-channel) instead of RGB
        //!!! DO NOT RELEASE pResults !!!
        pResults = facedetect_cnn(pBuffer, (unsigned char*)(image.ptr(0)), image.cols, image.rows, (int)image.step);
//        printf("%d faces detected.\n", (pResults ? *pResults : 0));

        if(!pResults)
            return;

        //print the detection results
        // n = (pResults ? *pResults : 0)
        int i = 0;
        short * p = ((short*)(pResults+1))+142*i;
        int x = p[0];
        int y = p[1];
        int w = p[2];
        int h = p[3];
        int confidence = p[4];
        int angle = p[5];

        if(confidence < 80)
            return;

        if(x < 0)
            x = 0;
        else if(x >= image.cols)
            x = image.cols - 1;

        if(y < 0)
            y = 0;
        else if(y >= image.rows)
            y = image.rows - 1;

        if(x + w >= image.cols)
            w = image.cols - 1 - x;

        if(y + h >= image.rows)
            h = image.rows - 1 - y;

//        printf("face_rect=[%d, %d, %d, %d], confidence=%d, angle=%d\n", x,y,w,h,confidence, angle);

        cv::Rect face_bbox(x, y, w, h);
        cv::Mat face_img = image(face_bbox).clone();
        cv::cvtColor(face_img, face_img, CV_BGR2GRAY);

        cv_bridge::CvImage face_img_msg;
        face_img_msg.header = img_msg->header;
        face_img_msg.encoding = "mono8";
        face_img_msg.image = face_img;
        pub_face.publish(face_img_msg.toImageMsg());

        PointCloud face_bbox_msg;
        face_bbox_msg.header = img_msg->header;
        geometry_msgs::Point32 pt;
        pt.x = x;
        pt.y = y;
        pt.z = 0;
        face_bbox_msg.points.emplace_back(pt);
        pt.x = w;
        pt.y = h;
        pt.z = 0;
        face_bbox_msg.points.emplace_back(pt);
        pub_bbox.publish(face_bbox_msg);
    }

    int* pResults = nullptr;
    unsigned char * pBuffer = nullptr;
    ros::Publisher pub_face, pub_bbox;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "face_detection_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh("~");
    Node node(nh);
    ros::Subscriber sub_img = nh.subscribe<Image>("/rect_rgb/image_raw", 2, boost::bind(&Node::Callback, &node, _1));
    ros::spin();
    return 0;
}
