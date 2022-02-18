	#include <string>
	#include <ros/ros.h>
	#include <ros/console.h>
	#include <opencv2/opencv.hpp>
	#include <math.h>
	#include <std_msgs/String.h>
	#include <std_msgs/Empty.h>
	#include <swiftpro/status.h>
	#include <vision/GuiMsg.h>


	#define LEFT	300
	#define TOP		280
	#define RIGHT	370
	#define BOTTOM	340

	using namespace cv;

	VideoCapture cap;
	swiftpro::status status;
	vision::GuiMsg GuiMsg;
	int flag = 0;
	//int ok = 0;
	int button;
	int flag_color = 0;
	int stop = -1;

	void sub_callback(const vision::GuiMsg& msg)
	{
		if(msg.button == 1)
		{
			button = 1;
			printf("start %d\n", button);
		}
		if(msg.button == 0)
		{
			button = 0;
			GuiMsg.m_state = 3;		//stop
			GuiMsg.machine = 0;		//camera1
			printf("stop %d\n", button);
		}
	}

	/*void camara_box()
	{
		if(button == 1)
		{
			printf("111 \n");
			Mat img, img_gray;
			Mat img_file, img_file_gray, img_file_resize;
			Mat img_box, img_box_thre, img_box_color;

			//img_file = imread("/home/yay/catkin_ws/src/RosForSwiftAndSwiftPro/swiftpro/img/img_file1.PNG");

			cap.read(img_file);

			resize(img_file, img_file_resize, Size(640, 480));
			
				//printf("222 \n");

				cap.read(img);			//img.size() 640x480

				if (img.empty()) {
					printf("이미지 안 불러와져 img");
					break;
				}

				if (img_file.empty()) {
					printf("이미지 안 불러와져 img_file");
					break;
				}

				//std::cout << img_file.size() << std::endl;

				cvtColor(img, img_gray, COLOR_BGR2GRAY);
				cvtColor(img_file_resize, img_file_gray, COLOR_BGR2GRAY);
				absdiff(img_file_gray, img_gray, img_box);				
				threshold(img_box, img_box_thre, 15, 255, THRESH_BINARY);

				cvtColor(img_box_thre, img_box_color, COLOR_GRAY2BGR);
				bitwise_and(img, img_box_color, img_box_color);

				Mat img_box_color_ROI, img_box_color_copy;
				img_box_color.copyTo(img_box_color_copy);
			
				img_box_color_ROI = img_box_color_copy(Rect_<float>(Point2f(LEFT + 1, TOP - 1), Point2f(RIGHT + 1, BOTTOM - 1)));		

				circle(img, Point(LEFT, TOP), 3, Scalar(255, 0, 0), 3);
				circle(img, Point(RIGHT, BOTTOM), 3, Scalar(255,0,0), 3);
			
				//int check = 0;		//물체가 잇나?
				//printf("333 \n");

				for (int j = 0; j < img_box_color_ROI.rows; j++)
				{
					for (int i = 0; i < img_box_color_ROI.cols; i++)
					{
						if (img_box_color_ROI.at<uchar>(j, i) != 0)
						{
							ok = 1;
						
						}
					}
				}

				
				if(ok == 1)
				{
					printf("box ok\n");
					status.status = 1;
					flag = 1;
					break;
				}


				imshow("img", img_box_thre);
				imshow("img_file", img_file_gray);
				imshow("img_box_color_ROI",img_box_color_ROI);

				//printf("444 \n");		

				//if (waitKey(100) == 27)  break;
		}
	}*/


	int main(int argc, char** argv)
	{
		ros::init(argc, argv, "yay_opencv_box");
		ros::NodeHandle nh;
		//ros::MultiThreadedSpinner spin(2);

		// -- 동작 수행 --
		ros::Publisher 	 pub = nh.advertise<swiftpro::status>("V_Check_topic", 1);

		// -- GUI와 통신 --
		ros::Subscriber  sub = nh.subscribe("StartStop_Topic", 1, sub_callback);
		ros::Publisher 	 pub1 = nh.advertise<vision::GuiMsg>("Gui_Topic", 1);


		ros::Rate loop_rate(20);

		Mat img, img_gray, img_copy;
		Mat img_file, img_file_gray, img_file_resize;
		Mat img_box, img_box_thre, img_box_color;
		Mat img_thre, img_blur, img_morph;

		std::vector<std::vector<Point>> contours;
		std::vector<RotatedRect> candidates;
		Point2f pts[4];
		RotatedRect rot_rect;

		Matx<uchar, 3, 3> mask;	//mask
		mask << 0, 1, 0,
				1, 1, 1,
				0, 1, 0;

		cap.open(1);

		if(cap.isOpened())	printf("camera open~!~~!~!!\n");

		printf("hi\n");
		for(int i=0; i<30; i++)
		{
			cap.read(img_file);
		}
		while (ros::ok())	//roscore
		{	
			int sum = 0;
			if(button == 1)
			{
				GuiMsg.m_state = 1;
				GuiMsg.machine = 0;
				//printf("111 \n");
				//img_file = imread("/home/yay/catkin_ws/src/RosForSwiftAndSwiftPro/swiftpro/img/img_file1.PNG");

				resize(img_file, img_file_resize, Size(640, 480));
			
				//printf("222 \n");
				//while(1)
				//{
				cap.read(img);			//img.size() 640x480

				
				if (img.empty()) {
					printf("이미지 안 불러와져 img");
					break;
				}

				if (img_file.empty()) {
					printf("이미지 안 불러와져 img_file");
					break;
				}

				
				//std::cout << img_file.size() << std::endl;

				// cvtColor(img, img_gray, COLOR_BGR2GRAY);
				// cvtColor(img_file_resize, img_file_gray, COLOR_BGR2GRAY);
				// absdiff(img_file_gray, img_gray, img_box);				
				// threshold(img_box, img_box_thre, 15, 255, THRESH_BINARY);

				// cvtColor(img_box_thre, img_box_color, COLOR_GRAY2BGR);
				// bitwise_and(img, img_box_color, img_box_color);

				Mat img_box_color_ROI;
			
				img_box_color_ROI = img(Rect_<float>(Point2f(LEFT + 1, TOP - 1), Point2f(RIGHT + 1, BOTTOM - 1)));		// 카메라 설치 후 roi 영역 재설정할 것

				img.copyTo(img_copy);
				circle(img_copy, Point(LEFT, TOP), 1, Scalar(255, 0, 0), 1);
				circle(img_copy, Point(RIGHT, BOTTOM), 1, Scalar(255,0,0), 1);
				
				//외곽선 찾은 후 최소 사각형 구하기----------------------------
				cvtColor(img_box_color_ROI, img_box_color_ROI, COLOR_BGR2GRAY);
				threshold(img_box_color_ROI, img_thre, 0, 255, THRESH_BINARY_INV | THRESH_OTSU);
				blur(img_thre, img_blur, Size(10, 10));
				morphologyEx(img_blur, img_morph, MORPH_DILATE, mask, Point(-1, -1), 6);
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
				
				printf("minArea\n");

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
				printf("sorting\n");

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
				printf("warping\n");

				//ROI 검출-------------
				Mat img_wrapcp;
				img_wrap.copyTo(img_wrapcp);	//워핑 복사본 만들기
				img_ROI = img_wrapcp(Rect_<float>(Point2f(warp_pts[0].x, warp_pts[0].y),
					Point2f(warp_pts[3].x, warp_pts[3].y)));	//영역 만들기
				//ROI 검출-------------
				printf("roi\n");

				Mat img_marker, img_markerThre;
				img_ROI.copyTo(img_marker);
				threshold(img_marker, img_markerThre, 0, 255, THRESH_BINARY_INV | THRESH_OTSU);

					//printf("%d\n", flag_color);
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

					Mat img_markerThreCopy, img_markerROI;
					int m_arrMarker160by160[80][80] = { 0 };
			
					img_markerThre.copyTo(img_markerThreCopy);

					img_markerROI = img_markerThreCopy(Rect_<float>(Point2f(left, top), Point2f(right, bottom)));

					int height, width, diff;
					height = bottom - top;
					width = right - left;
					imshow("img_markerROI",img_markerROI);

					std::cout << "height " << height << "width " << width << std::endl;

					if(10 < height && 10 < width){
						Mat img_markerROIResize;
						resize(img_markerROI, img_markerROIResize, Size(80, 80));

						for (int y = 0; y < 80; y++) {
							for (int x = 0; x < 80; x++) {
								if (img_markerROIResize.at<uchar>(y, x) == 255)		m_arrMarker160by160[y][x] = 1;	//흰색
								if (img_markerROIResize.at<uchar>(y, x) == 0)		m_arrMarker160by160[y][x] = 0;	//검정색
							}
						}

						int m_arrMarker[16] = { 0 };
						int m_arrNum[16] = { 0 };

						for(int y = 0; y < 10; y++){
							for(int x = 0; x < 10; x++){
								m_arrMarker[0] += m_arrMarker160by160[y][x];
							}
							for(int x = 10; x < 20; x++){
								m_arrMarker[1] += m_arrMarker160by160[y][x];
							}
							for(int x = 60; x < 70; x++){
								m_arrMarker[2] += m_arrMarker160by160[y][x];
							}
							for(int x = 70; x < 80; x++){
								m_arrMarker[3] += m_arrMarker160by160[y][x];
							}
						}

						for(int y = 10; y < 20; y++){
							for(int x = 0; x < 10; x++){
								m_arrMarker[4] += m_arrMarker160by160[y][x];
							}
							for(int x = 10; x < 20; x++){
								m_arrMarker[5] += m_arrMarker160by160[y][x];
							}
							for(int x = 60; x < 70; x++){
								m_arrMarker[6] += m_arrMarker160by160[y][x];
							}
							for(int x = 70; x < 80; x++){
								m_arrMarker[7] += m_arrMarker160by160[y][x];
							}
						}

						for(int y = 60; y < 70; y++){
							for(int x = 0; x < 10; x++){
								m_arrMarker[8] += m_arrMarker160by160[y][x];
							}
							for(int x = 10; x < 20; x++){
								m_arrMarker[9] += m_arrMarker160by160[y][x];
							}
							for(int x = 60; x < 70; x++){
								m_arrMarker[10] += m_arrMarker160by160[y][x];
							}
							for(int x = 70; x < 80; x++){
								m_arrMarker[11] += m_arrMarker160by160[y][x];
							}
						}

						for(int y = 70; y < 80; y++){
							for(int x = 0; x < 10; x++){
								m_arrMarker[12] += m_arrMarker160by160[y][x];
							}
							for(int x = 10; x < 20; x++){
								m_arrMarker[13] += m_arrMarker160by160[y][x];
							}
							for(int x = 60; x < 70; x++){
								m_arrMarker[14] += m_arrMarker160by160[y][x];
							}
							for(int x = 70; x < 80; x++){
								m_arrMarker[15] += m_arrMarker160by160[y][x];
							}
						}

						
						for (int i = 0; i < 16; i++){	
							if (m_arrMarker[i] < 50)	m_arrMarker[i] = 0;		//검정
							else                        m_arrMarker[i] = 1;		//흰색
						}

						for (int i = 0; i < 16; i++)
						{
							std::cout << m_arrMarker[i] << '\t';
						}
						std::cout << std::endl;

						for (int i = 0; i < 16; i++)
						{
							if (m_arrMarker[i] == 0)	sum += 0;
							else						sum += pow(2, i);
						}
						std::cout << "sum " << sum << std::endl;
					}
					
					if(sum == 1632)
					{
						printf("box ok\n");
						status.status = 1;
						pub.publish(status);
					}


					imshow("img_copy", img_copy);
					imshow("img_morph", img_morph);
					imshow("img_box_color_ROI",img_box_color_ROI);

					//printf("444 \n");		

					if (waitKey(100) == 27)  break;
					//}
				}
				
			
				pub1.publish(GuiMsg);
				ros::spinOnce();
				loop_rate.sleep();
			}

			return 0;


		}