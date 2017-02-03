#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace aruco;

int main(int argc,char **argv)
{
    try
    {
        VideoCapture cap(0); // open the default camera
        if(!cap.isOpened())  // check if we succeeded
            return -1;

        Mat edges;
        namedWindow("frame", 1 );

        aruco::MarkerDetector MDetector;
        vector<Marker> Markers;
        for(;;)
        {
            Mat frame;
            cap >> frame; // get a new frame from camera
            imshow("frame", frame);

            //Ok, let's detect
            MDetector.detect(frame, Markers);
            //for each marker, draw info and its boundaries in the image
            for (unsigned int i=0;i<Markers.size();i++) {
                cout<<Markers[i]<<endl;
                Markers[i].draw(frame,Scalar(0,0,255),2);
            }
            cv::imshow("in", frame);

            if(waitKey(30) >= 0)
                break;
        }
    }
    catch (std::exception &ex)
    {
        cout<<"Exception :"<<ex.what()<<endl;
    }
}
 