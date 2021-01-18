#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/stereo.hpp>
#include "opencv2/core/utility.hpp"
#include "opencv2/video/tracking.hpp"

//static const std::string OPENCV_WINDOW = "Image window";
using namespace cv;
using namespace ros;
using namespace std;
using namespace cv::stereo;
enum { STEREO_BINARY_BM, STEREO_BINARY_SGM };
int frame_number=0;
cv::Mat currentleft,currentright;
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_left,image_sub_right;
  image_transport::Publisher image_pub_left,image_pub_right;
  cv::Mat L_input_image,R_input_image;
  ros::Time L_image_time,R_image_time;
public:
  ImageConverter()
  : it_(nh_)
  {
      // Subscrive to input video feed and publish output video feed
    image_sub_left = it_.subscribe("/left/image_raw", 1,
     &ImageConverter::imageCbleft, this);
    image_sub_right = it_.subscribe("/right/image_raw", 1,
      &ImageConverter::imageCbright, this);
    image_pub_left = it_.advertise("/left/image_mono_crop", 1);
    image_pub_right = it_.advertise("/right/image_mono_crop", 1);
/*
    Mat M1 = (Mat1d(3, 3) << 4.241222823159629e+02, 0, 3.808757832118394e+02, 0, 4.253507465077519e+02, 2.415503963486819e+02, 0, 0, 1);  //M1
    Mat M2 = (Mat1d(3, 3) << 4.279950560407619e+02, 0, 3.560625127393157e+02, 0, 4.269467283509290e+02, 2.378487325222138e+02, 0, 0, 1);  //M2
    Mat D1 = (Mat1d(1, 4) << -0.295639862369312,0.081195391520932,9.303430510914474e-04,8.871065775310512e-04);         //D1
    Mat D2 = (Mat1d(1, 4) <<  -0.298723672183145,0.087365579013266, 6.771145987843409e-04,-0.001045529523078);         //D2
    Mat R = (Mat1d(3, 3) << 0.999436578873115,-0.014451722263272,-0.030293110339312,0.014073497936550,0.999820792320651,-0.012661749450053,0.030470665667727,0.012228285527057,0.999460858446610);  //R
    Mat T = (Mat1d(3, 1) << -2.603989366196675e+02,-6.073232915040237,-9.472208027788101);  //T
*/

/*
	//claibe3
    Mat M1 = (Mat1d(3, 3) <<4.219793564182257e+02, 0, 3.815933416567996e+02, 0, 4.233917160811687e+02, 2.422558547246247e+02, 0, 0, 1);  //M1
    Mat M2 = (Mat1d(3, 3) << 4.272880000664518e+02, 0, 3.536349006950120e+02, 0,4.286467129776005e+02, 2.327775016969991e+02, 0, 0, 1);  //M2
    Mat D1 = (Mat1d(1, 4) << -0.312141318819769,0.110816285528975,6.277559651545402e-04,7.461325892843807e-05);         //D1
    Mat D2 = (Mat1d(1, 4) <<  -0.303765650849966,0.099483812206590, -3.033499801908697e-05,5.154642791875892e-04);         //D2
    Mat R = (Mat1d(3, 3) << 0.999622929050293,-0.009316814388365,-0.025830150726737,0.009251874434512,0.999953735346454,-0.002632486680468,0.025853482093544,0.002392516735067,0.999662879838654);  //R
    Mat T = (Mat1d(3, 1) << -2.591873680320928e+02,-1.847244087539134,-3.750709353589202);  //T
*/

//calib2
        Mat M1 = (Mat1d(3, 3) << 4.250258563372763e+02, 0, 3.860151866550880e+02, 0, 4.267976260903337e+02, 2.419130336743440e+02, 0, 0, 1);  //M1
    Mat M2 = (Mat1d(3, 3) << 4.313364265799752e+02, 0, 3.548956286992647e+02, 0,4.327527965378035e+02, 2.325508916495161e+02, 0, 0, 1);  //M2
    Mat D1 = (Mat1d(1, 4) << -0.288105327549552,0.074578284234601, 7.784489598138802e-04,-2.277853975035461e-04);         //D1
    Mat D2 = (Mat1d(1, 4) << -0.300267420221178,0.090544063693053, 3.330220891093334e-05,8.989607188457415e-05);         //D2
    Mat R = (Mat1d(3, 3) << 0.999795205274114,-0.009288477716175,-0.017979757857331,0.009235691998760,0.999952799780068,-0.003016654654974,0.018006929338346,0.002849981354228,0.999833800239862);  //R
    Mat T = (Mat1d(3, 1) << -2.590512646644321e+02,-1.557279545456558,-4.706934914838741);  //T


      Size img_size = Size(752,480);
      
      std::cout<<"init camera matrix"<<std::endl;
      Rect roi1, roi2;
      Mat Q;
      
      std::cout<<R<<std::endl;
      std::cout<<T<<std::endl;
      Mat R1, P1, R2, P2;
      stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );
      std::cout<<"R1R2P2P2"<<std::endl;

      Mat map11, map12, map21, map22;
      initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
      initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);
      std::cout<<"initUndistortRectifyMapx"<<std::endl;


      int kernel_size = 5, number_of_disparities = 128, aggregation_window = 7, sgmP1 = 100, sgmP2 = 1000;
      float scale = 32;
      int algo = STEREO_BINARY_BM;
      int binary_descriptor_type = 3;

      //cv::namedWindow(OPENCV_WINDOW);
      ros::Rate loop_rate(10);
      while (ros::ok())
      {
        ros::Duration diff=L_image_time-R_image_time;
       
        /*std::cout<<"!R_input_image.empty(): "<<!R_input_image.empty()<<std::endl; // debug for image sync*/
        if(!R_input_image.empty() && !L_input_image.empty())
        {
          if(fabs((float)diff.toSec())< 0.0001) //if image comesin with less than 1ms difference. 
          {
             std::cout<<"diff: "<<fabs((float)diff.toSec())<<std::endl; // debug for image sync
           Mat img1r, img2r;
           remap(L_input_image, img1r, map11, map12, INTER_LINEAR);
           remap(R_input_image, img2r, map21, map22, INTER_LINEAR);

           imshow("R_input_image",img2r(roi2));
           imshow("L_input_image",img1r(roi1));



           Mat imgDisparity16S2 = Mat(img1r(roi1).rows, img1r(roi1).cols, CV_16S);
           Mat imgDisparity8U2 = Mat(img1r(roi1).rows, img1r(roi1).cols, CV_8UC1);
           Ptr<cv::stereo::StereoBinarySGBM> sgbm = cv::stereo::StereoBinarySGBM::create(0, number_of_disparities, kernel_size);
        // setting the penalties for sgbm
           sgbm->setP1(sgmP1);
           sgbm->setP2(sgmP2);
           sgbm->setMinDisparity(0);
           sgbm->setUniquenessRatio(5);
           sgbm->setSpeckleWindowSize(200);
           sgbm->setSpeckleRange(0);
           sgbm->setDisp12MaxDiff(1);
           sgbm->setBinaryKernelType(binary_descriptor_type);
           sgbm->setSpekleRemovalTechnique(CV_SPECKLE_REMOVAL_AVG_ALGORITHM);
           sgbm->setSubPixelInterpolationMethod(CV_SIMETRICV_INTERPOLATION);
           sgbm->compute(img1r(roi1), img2r(roi2), imgDisparity16S2);
        
          //imgDisparity16S2.convertTo(imgDisparity8U2, CV_8UC1, scale);
        
           double minVal; double maxVal;
           minMaxLoc(imgDisparity16S2, &minVal, &maxVal);
           imgDisparity16S2.convertTo(imgDisparity8U2, CV_8UC1, 255 / (maxVal - minVal));
        //show the disparity image
           imshow("Windowsgm", imgDisparity8U2);



          vector<Point2f> points1, points2;        //vectors to store the coordinates of the feature points
          featureDetection(img1r(roi1), points1);        //detect features in img_1
          vector<uchar> status;
          featureTracking(img1r(roi1),img2r(roi2),points1,points2, status); //track those features to img_2
          cv::Mat drawimage;
          img1r(roi1).copyTo(drawimage);
          cvtColor(drawimage,drawimage,CV_GRAY2BGR);
          for(int i=0;i<points1.size();i++)
          {
            line(drawimage, points1[i], points2[i], Scalar(0,255,0),1 );
          }
          imshow("drawimage",drawimage);

           cv::waitKey(1);
         }
       }

       ros::spinOnce();
       loop_rate.sleep();
     }



   }

   ~ImageConverter()
   {
      //cv::destroyWindow(OPENCV_WINDOW);
   }


   void featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status)  { 

//this function automatically gets rid of points for which tracking fails

  vector<float> err;          
  Size winSize=Size(21,21);                                               
  TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);

  calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);

  //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
  int indexCorrection = 0;
  for( int i=0; i<status.size(); i++)
     {  Point2f pt = points2.at(i- indexCorrection);
      if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))  {
          if((pt.x<0)||(pt.y<0))  {
            status.at(i) = 0;
          }
          points1.erase (points1.begin() + (i - indexCorrection));
          points2.erase (points2.begin() + (i - indexCorrection));
          indexCorrection++;
      }

     }

}


void featureDetection(Mat img_1, vector<Point2f>& points1)  {   //uses FAST as of now, modify parameters as necessary
  vector<KeyPoint> keypoints_1;
  int fast_threshold = 20;
  bool nonmaxSuppression = true;
  FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
  KeyPoint::convert(keypoints_1, points1, vector<int>());
}


   void imageCbleft(const sensor_msgs::ImageConstPtr& msg)
   {
    cv_bridge::CvImagePtr cv_ptr;
    L_image_time = msg->header.stamp;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    L_input_image=cv_ptr->image;
    cvtColor(L_input_image,L_input_image,CV_RGB2GRAY);
/*      cv::Mat inputimage=cv_ptr->image;

      // Update GUI Window
      //cv::flip(inputimage,inputimage,-1);
      cv::Mat crop;
      cvtColor(inputimage,inputimage,CV_RGB2GRAY);
      L_input_image=inputimage;

      cv::resize(inputimage,crop,cv::Size(752,480));
      cv::imshow("left", crop);
      currentleft=crop;

      //char key=cv::waitKey(1);

      
      //cv_ptr->image=crop;
      cv_bridge::CvImage out_msg;
      out_msg.header   = cv_ptr->header; // Same timestamp and tf frame as input image
      out_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
      out_msg.image    = crop; // Your cv::Mat

      // Output modified video stream
      image_pub_left.publish(out_msg.toImageMsg());*/
  }


  void imageCbright(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    R_image_time = msg->header.stamp;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    R_input_image=cv_ptr->image;
    cvtColor(R_input_image,R_input_image,CV_RGB2GRAY);
/*      cv::Mat inputimage=cv_ptr->image;
      // Update GUI Window

      cv::Mat crop;

      cvtColor(inputimage,inputimage,CV_RGB2GRAY);
      cv::resize(inputimage,crop,cv::Size(752,480));
      cv::imshow("right", crop);
      currentright=crop;
      char key=cv::waitKey(1);
      //cv_ptr->image=crop;
      cv_bridge::CvImage out_msg;
      out_msg.header   = cv_ptr->header; // Same timestamp and tf frame as input image
      out_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
      out_msg.image    = crop; // Your cv::Mat

      // Output modified video stream
      image_pub_right.publish(out_msg.toImageMsg());*/
  }
};
int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereo_matching_node");
  ImageConverter ic;
  //ros::spinOnce();
  return 0;
}
