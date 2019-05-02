#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "facedetectcnn.h"

//define the buffer size. Do not change the size!
#define DETECT_BUFFER_SIZE 0x20000

using namespace sensor_msgs;

class Node {
public:
    Node(ros::NodeHandle& nh) {
        pub_face = nh.advertise<Image>("/face/image_raw", 10);
        pBuffer = (unsigned char *)malloc(DETECT_BUFFER_SIZE);
        ROS_ASSERT_MSG(pBuffer, "Can not alloc buffer.");
    }

    ~Node() {
        free(pBuffer);
    }

    void Callback(const sensor_msgs::ImageConstPtr& img_msg) {
        cv::Mat image = cv_bridge::toCvCopy(img_msg, "mono8")->image;
        ///////////////////////////////////////////
        // CNN face detection
        // Best detection rate
        //////////////////////////////////////////
        //!!! The input image must be a BGR one (three-channel) instead of RGB
        //!!! DO NOT RELEASE pResults !!!
        pResults = facedetect_cnn(pBuffer, (unsigned char*)(image.ptr(0)), image.cols, image.rows, (int)image.step);
        printf("%d faces detected.\n", (pResults ? *pResults : 0));
        cv::Mat result_cnn = image.clone();
        //print the detection results
        for(int i = 0; i < (pResults ? *pResults : 0); i++)
        {
            short * p = ((short*)(pResults+1))+142*i;
            int x = p[0];
            int y = p[1];
            int w = p[2];
            int h = p[3];
            int confidence = p[4];
            int angle = p[5];

            printf("face_rect=[%d, %d, %d, %d], confidence=%d, angle=%d\n", x,y,w,h,confidence, angle);
            rectangle(result_cnn, cv::Rect(x, y, w, h), cv::Scalar(0, 255, 0), 2);
        }
        cv::imshow("result_cnn", result_cnn);

        cv::waitKey(1);
    }

    int* pResults = nullptr;
    unsigned char * pBuffer = nullptr;
    ros::Publisher pub_face;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "face_detection_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh("~");
    Node node(nh);
    ros::Subscriber sub_img = nh.subscribe<Image>("/rect_rgb/image_raw", 10, boost::bind(&Node::Callback, &node, _1));
    ros::spin();
    return 0;
}
