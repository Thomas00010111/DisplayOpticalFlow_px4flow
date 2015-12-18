#include "ros/ros.h"
#include "opencv2/video/tracking.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "px_comm/OpticalFlow.h"

/*some helpful urls:
 * [1] https://github.com/Itseez/opencv/blob/03e74330fa95306407f521beecdc3ae059f0cf1c/samples/cpp/fback.cpp
 * [2] http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
 * [3] http://wiki.ros.org/image_transport/Tutorials/SubscribingToImages
*/

using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

Mat gray_prev, flow, gray;
Mat uflow, cflow;
cv_bridge::CvImageConstPtr cv_ptr, cv_ptr_prev;
bool init = true;

//taken from [1]
static void drawOptFlowMap(const Mat& flow, Mat& cflowmap, int step,
                    double, const Scalar& color)
{
    for(int y = 0; y < cflowmap.rows; y += step)
        for(int x = 0; x < cflowmap.cols; x += step)
        {
            const Point2f& fxy = flow.at<Point2f>(y, x);
            line(cflowmap, Point(x,y), Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),
                 color);
            circle(cflowmap, Point(x,y), 2, color, -1);
        }
}

void recImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if (init)
    {
        ROS_INFO("Init");
        cv_ptr_prev = cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::BGR8);
        cvtColor(cv_ptr_prev->image, gray_prev, COLOR_BGR2GRAY);
        init=false;
        return;
    }

    cv_ptr = cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::BGR8);
    cvtColor(cv_ptr->image, gray, COLOR_BGR2GRAY);
    calcOpticalFlowFarneback(gray_prev, gray, uflow, 0.5, 3, 15, 3, 5, 1.2, 0);

    cvtColor(gray_prev, cflow, COLOR_GRAY2BGR);
    drawOptFlowMap(uflow, cflow, 16, 1.5, Scalar(0, 255, 0));

    imshow(OPENCV_WINDOW, cflow);

    std::swap(gray_prev, gray);
}



int main(int argc, char **argv) {

	//setup listener	
        ros::init(argc, argv, "listener");
	ros::NodeHandle n;

        namedWindow(OPENCV_WINDOW);
        startWindowThread();

        image_transport::ImageTransport it(n);
        image_transport::Subscriber sub = it.subscribe("/px4flow/camera_image", 10, recImageCallback);

        ros::spin();
        
        destroyWindow(OPENCV_WINDOW);
	return 0;
}
