#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <iwh_arm/Vclassify.h>
#include <std_msgs/UInt8.h>
#include <vision/GuiMsg.h>

using namespace cv;

VideoCapture cap;
iwh_arm::Vclassify v_classify;	//arm ->
vision::GuiMsg       GuiMsg;	//gui ->
int flag = 0;
int button;
int x;
Mat img, img_copy, img_thre, img_blur, img_morph, img_copyROI;
std::vector<std::vector<Point>> contours;
std::vector<RotatedRect> candidates;
Point2f pts[4];
RotatedRect rot_rect;
float asd = 0.f;
Matx<uchar, 3, 3> mask;   //mask

#define LEFT	60
#define TOP		100
#define RIGHT	280
#define BOTTOM	360

void camera_marker();

void camera_marker_callback(const std_msgs::UInt8& msg)
{
   x = (int)msg.data;
   printf("x %d", x);
   cap.open(0);

   if(cap.isOpened())   printf("camera open~!~~!~!!\n");
   else            	printf("camera is not opened");
}

void camera_marker()
{
   
   mask << 0, 1, 0, 
         1, 1, 1, 
         0, 1, 0;
   //while(1)
   //{
      if(x == 1)
      {
         cap.read(img);

         if (img.empty())
         {
            printf("이미지 안 불러와져 img");
            //break;
         }

         img.copyTo(img_copy);
		 
		img.copyTo(img_copy);
		img_copyROI = img_copy(Rect_<float>(Point2f(LEFT, TOP), Point2f(RIGHT, BOTTOM)));
         cvtColor(img_copyROI, img_copyROI, COLOR_BGR2GRAY);
         threshold(img_copyROI, img_thre, 0, 255, THRESH_BINARY_INV | THRESH_OTSU);
         blur(img_thre, img_blur, Size(10, 10));
         morphologyEx(img_blur, img_morph, MORPH_DILATE, mask, Point(-1, -1), 6);

         //외곽선 찾은 후 최소 사각형 구하기----------------------------
		 //findContours(img_morph, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
         findContours(img_morph, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

         for (int i = 0; i < (int)contours.size(); i++)
         {
            rot_rect = minAreaRect(contours[i]);
            candidates.push_back(rot_rect);
         }
         //외곽선 찾은 후 최소 사각형 구하기----------------------------

         //최소 사각형 그리기 -----------------------------------------
         for (int i = 0; i < (int)candidates.size(); i++)
         {
            rot_rect.points(pts);
            for (int j = 0; j < 4; j++) {
               line(img_morph, pts[j], pts[(j + 1) % 4], Scalar(255), 2);
               circle(img_morph, pts[j], 3, Scalar(128), 3);
            }
         }
         //최소 사각형 그리기 -----------------------------------------
         
         //printf("minArea\n");

         //sorting--------------
         Point2f tmp;
         for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 3; j++) {
               if (pts[j].y > pts[j + 1].y) {
                  tmp = pts[j];
                  pts[j] = pts[j + 1];
                  pts[j + 1] = tmp;
               }
            }
         }
         if (pts[0].x > pts[1].x) {
            tmp = pts[0];
            pts[0] = pts[1];
            pts[1] = tmp;
         }
         if (pts[2].x > pts[3].x) {
            tmp = pts[2];
            pts[2] = pts[3];
            pts[3] = tmp;
         }
         //sorting--------------
         //printf("sorting\n");

         //warping--------------
         Size warpsize((pts[1].x - pts[0].x), (pts[3].y - pts[0].y));
         Mat img_wrap;
         Mat img_ROI;

         Point2f warp_pts[4];
         warp_pts[0] = Point2f(0, 0);
         warp_pts[1] = Point2f(warpsize.width, 0);
         warp_pts[2] = Point2f(0, warpsize.height);
         warp_pts[3] = Point2f(warpsize.width, warpsize.height);

         Mat trans = getPerspectiveTransform(pts, warp_pts);
         warpPerspective(img_thre, img_wrap, trans, warpsize, INTER_CUBIC);
         //warping--------------
         //printf("warping\n");
		 imshow("img_wrap",img_wrap);

         //ROI 검출-------------
         Mat img_wrapcp;
         img_wrap.copyTo(img_wrapcp);   //워핑 복사본 만들기
         img_ROI = img_wrapcp(Rect_<float>(Point2f(warp_pts[0].x, warp_pts[0].y),
            Point2f(warp_pts[3].x, warp_pts[3].y)));   //영역 만들기
         //ROI 검출-------------
         //printf("roi\n");

         Mat img_marker, img_markerThre;
         img_ROI.copyTo(img_marker);
         threshold(img_marker, img_markerThre, 0, 255, THRESH_BINARY_INV | THRESH_OTSU);
		 imshow("img_morph",img_morph);

         //마커 영역만 잡기----------------
         int left=0, right=0, top=0, bottom=0;
         for (int i = 0; i < img_markerThre.cols; i++) {
            if (img_markerThre.at<uchar>(img_markerThre.rows / 2, i) == 0) {
               right = i;
            }
         }

         for (int i = img_markerThre.cols - 1; i > 0; i--) {
            if (img_markerThre.at<uchar>(img_markerThre.rows / 2, i) == 0) {
               left = i;
            }
         }

         for (int i = 0; i < img_markerThre.rows; i++) {
            if (img_markerThre.at<uchar>(i, img_markerThre.cols / 2) == 0) {
               bottom = i;
            }
         }

         for (int i = img_markerThre.rows - 1; i > 0; i--) {
            if (img_markerThre.at<uchar>(i, img_markerThre.cols / 2) == 0) {
               top = i;
            }
         }
         //cout << "top " << top << "bottom " << bottom << "left " << left << "right " << right << endl;
         
         int height, width, diff;
         height = bottom - top;
         width = right - left;
         //diff = height - width;

         std::cout << "height" << height << "width" << width << std::endl;      

         circle(img_morph, rot_rect.center, 5, Scalar(255), 2);   //회전 사각형 중심점 그리기
         printf("marker_cam\n\n");
         //imshow("marker_cam", img_morph);

         //마커 영역만 잡기----------------

         Mat img_markerThreCopy, img_markerROI;
         
         img_markerThre.copyTo(img_markerThreCopy);
         if (60 < height && height < 90 && 60 < width && width < 90) {
            img_markerROI = img_markerThreCopy(Rect_<float>(Point2f(left, top), Point2f(right, bottom)));

            Mat img_markerROIResize;
            resize(img_markerROI, img_markerROIResize, Size(160, 160));

            // check_marker ------------------------------------
            int m_arrMarker160by160[160][160] = { 0 };

            for (int y = 0; y < 160; y++) {
               for (int x = 0; x < 160; x++) {
                  if (img_markerROIResize.at<uchar>(y, x) == 255)      m_arrMarker160by160[y][x] = 1;   //흰색
                  if (img_markerROIResize.at<uchar>(y, x) == 0)      m_arrMarker160by160[y][x] = 0;   //검정색
               }
            }
            int m_arrMarker[16] = { 0 };
            
            int m_arrNum[16] = { 0 };

            for (int y = 40; y < 60; y++) {
               for (int x = 40; x < 60; x++) {
                  m_arrMarker[0] += m_arrMarker160by160[y][x];
               }
               for (int x = 60; x < 80; x++) {
                  m_arrMarker[1] += m_arrMarker160by160[y][x];
               }
               for (int x = 80; x < 100; x++) {
                  m_arrMarker[2] += m_arrMarker160by160[y][x];
               }
               for (int x = 100; x < 120; x++) {
                  m_arrMarker[3] += m_arrMarker160by160[y][x];
               }
            }
            for (int y = 60; y < 80; y++) {
               for (int x = 40; x < 60; x++) {
                  m_arrMarker[4] += m_arrMarker160by160[y][x];
               }
               for (int x = 60; x < 80; x++) {
                  m_arrMarker[5] += m_arrMarker160by160[y][x];
               }
               for (int x = 80; x < 100; x++) {
                  m_arrMarker[6] += m_arrMarker160by160[y][x];
               }
               for (int x = 100; x < 120; x++) {
                  m_arrMarker[7] += m_arrMarker160by160[y][x];
               }
            }
            for (int y = 80; y < 100; y++) {
               for (int x = 40; x < 60; x++) {
                  m_arrMarker[8] += m_arrMarker160by160[y][x];
               }
               for (int x = 60; x < 80; x++) {
                  m_arrMarker[9] += m_arrMarker160by160[y][x];
               }
               for (int x = 80; x < 100; x++) {
                  m_arrMarker[10] += m_arrMarker160by160[y][x];
               }
               for (int x = 100; x < 120; x++) {
                  m_arrMarker[11] += m_arrMarker160by160[y][x];
               }
            }
            for (int y = 100; y < 120; y++) {
               for (int x = 40; x < 60; x++) {
                  m_arrMarker[12] += m_arrMarker160by160[y][x];
               }
               for (int x = 60; x < 80; x++) {
                  m_arrMarker[13] += m_arrMarker160by160[y][x];
               }
               for (int x = 80; x < 100; x++) {
                  m_arrMarker[14] += m_arrMarker160by160[y][x];
               }
               for (int x = 100; x < 120; x++) {
                  m_arrMarker[15] += m_arrMarker160by160[y][x];
               }
            }

            int sum = 0;
            for (int i = 0; i < 16; i++){   
               if (m_arrMarker[i] < 200)   m_arrMarker[i] = 0;      //검정
               else                        m_arrMarker[i] = 1;      //흰색
            }

            for (int i = 0; i < 16; i++)
            {
               if (m_arrMarker[i] == 1)   sum += 0;
               else                        sum += pow(2, i);
            }

            /*if(0 < rot_rect.center.x && rot_rect.center.x < 79 && 0 < rot_rect.center.y && rot_rect.center.y < 79)         v_classify.x = 0;
            if(80 < rot_rect.center.x && rot_rect.center.x < 159 && 0 < rot_rect.center.y && rot_rect.center.y < 79)      v_classify.x = 1;
            if(160 < rot_rect.center.x && rot_rect.center.x < 239 && 0 < rot_rect.center.y && rot_rect.center.y < 79)      v_classify.x = 2;
            if(240 < rot_rect.center.x && rot_rect.center.x < 319 && 0 < rot_rect.center.y && rot_rect.center.y < 79)      v_classify.x = 3;

            if(0 < rot_rect.center.x && rot_rect.center.x < 79 && 80 < rot_rect.center.y && rot_rect.center.y < 159)      v_classify.x = 4;
            if(80 < rot_rect.center.x && rot_rect.center.x < 159 && 80 < rot_rect.center.y && rot_rect.center.y < 159)      v_classify.x = 5;
            if(160 < rot_rect.center.x && rot_rect.center.x < 239 && 80 < rot_rect.center.y && rot_rect.center.y < 159)      v_classify.x = 6;
            if(240 < rot_rect.center.x && rot_rect.center.x < 319 && 80 < rot_rect.center.y && rot_rect.center.y < 159)      v_classify.x = 7;

            if(0 < rot_rect.center.x && rot_rect.center.x < 79 && 160 < rot_rect.center.y && rot_rect.center.y < 239)      v_classify.x = 8;
            if(80 < rot_rect.center.x && rot_rect.center.x < 159 && 160 < rot_rect.center.y && rot_rect.center.y < 239)      v_classify.x = 9;
            if(160 < rot_rect.center.x && rot_rect.center.x < 239 && 160 < rot_rect.center.y && rot_rect.center.y < 239)   v_classify.x = 10;
            if(240 < rot_rect.center.x && rot_rect.center.x < 319 && 160 < rot_rect.center.y && rot_rect.center.y < 239)   v_classify.x = 11;*/

            
            if (sum == 231 || sum == 4914 || sum == 19656 || sum == 59136)
            {
               v_classify.city = 1;
               flag = 1;
               printf("판교\n");
               x = 0;
               //break;
            }
            else if (sum == 1809 || sum == 35040 || sum == 29760 || sum == 558)
            {
               v_classify.city = 2;
               flag = 1;
               printf("문래\n");
               x = 0;
               //break;
            }
            else if (sum == 5217 || sum == 37472 || sum == 1609 || sum == 34344)
            {
               v_classify.city = 3;
               flag = 1;
               printf("구리\n");
               x = 0;
               //break;
            }
            // else if (sum == 1099 || sum == 4449 || sum == 53792 || sum == 34440)
            // {
            //    v_classify.city = 4;
            //    flag = 1;
            //    printf("인천\n");
            //    break;
            // }

            
            
         }
         // check_marker ------------------------------------
   
         
         //imshow("trans", trans);

         waitKey(100) == 27;   
      }
      
      
   //}
}

void sub_callback(const vision::GuiMsg& msg)
{
   if(msg.button == 1)
   {
      button = 1;
      GuiMsg.machine = 3;      //camera2
      GuiMsg.m_state = 1;      //on
      printf("start");
   }
   if(msg.button == 0)
   {
      button = 0;
      //stop = 1;
      GuiMsg.m_state = 3;      //stop
      GuiMsg.machine = 3;      //camera2
      printf("stop");
   }
   
}


int main(int argc, char** argv)
{

   ros::init(argc, argv, "camera2_node");
   ros::NodeHandle nh;

   // -- 동작 수행 --
   ros::Subscriber  sub = nh.subscribe("T_EndLoc_topic", 1, camera_marker_callback);
   ros::Publisher     pub = nh.advertise<iwh_arm::Vclassify>("V_Classify_topic", 1);
   
   // -- GUI와 통신 --
   ros::Subscriber  sub1 = nh.subscribe("StartStop_Topic", 1, sub_callback);
   ros::Publisher     pub1 = nh.advertise<vision::GuiMsg>("Gui_Topic", 1);
   
   ros::Rate loop_rate(20);

   GuiMsg.machine = 3;      //camera2

   printf("hi\n");

   while (ros::ok())   //roscore
   {
      if(button == 1)   
      {   
         camera_marker();
      }

      pub1.publish(GuiMsg);

      if(flag == 1)
      {
         pub.publish(v_classify);
         flag = 0;
      }
      ros::spinOnce();
      loop_rate.sleep();
   }

   return 0;
}