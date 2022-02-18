#include <iostream>
#include <opencv2/opencv.hpp>
//#include <windows.h>
#include <math.h>
#include <thread>
using namespace cv;
using namespace std;

//
bool m_bThFlag;
VideoCapture cap(0);
int check_marker(int, Mat);

void ThreadFunc()
{
	Mat img, img_copy, img_thre, img_blur, img_morph;
	vector<vector<Point>> contours;
	vector<RotatedRect> candidates;
	Point2f pts[4];
	RotatedRect rot_rect;
	float asd = 0.f;

	Matx<uchar, 3, 3> mask;	//mask
	mask << 0, 1, 0,
			1, 1, 1,
			0, 1, 0;

	if (!cap.isOpened())
	{
		printf("응 카메라 안 열림2");
		exit(0);
	}

	while (1)
	{
		if (!m_bThFlag) break;

		cap.read(img);

		//img = imread("marker.PNG");

		if (img.empty()) {
			printf("이미지 안 불러와져");
			break;
		}

		img.copyTo(img_copy);
		cvtColor(img_copy, img_copy, COLOR_BGR2GRAY);
		threshold(img_copy, img_thre, 0, 255, THRESH_BINARY_INV | THRESH_OTSU);
		blur(img_thre, img_blur, Size(10, 10));
		morphologyEx(img_blur, img_morph, MORPH_DILATE, mask, Point(-1, -1), 6);

		//외곽선 찾은 후 최소 사각형 구하기----------------------------
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

		//ROI 검출-------------
		Mat img_wrapcp;
		img_wrap.copyTo(img_wrapcp);	//워핑 복사본 만들기
		img_ROI = img_wrapcp(Rect_<float>(Point2f(warp_pts[0].x, warp_pts[0].y),
			Point2f(warp_pts[3].x, warp_pts[3].y)));	//영역 만들기
		//ROI 검출-------------

		Mat img_marker, img_markerThre;
		img_ROI.copyTo(img_marker);
		threshold(img_marker, img_markerThre, 0, 255, THRESH_BINARY_INV | THRESH_OTSU);

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

		cout << "height" << height << "width" << width << endl;		//160-180

		circle(img_morph, rot_rect.center, 5, Scalar(255), 2);	//회전 사각형 중심점 그리기

		imshow("morph", img_morph);

		//마커 영역만 잡기----------------

		Mat img_markerThreCopy, img_markerROI;
		int flag = -1;

		img_markerThre.copyTo(img_markerThreCopy);
		//imshow("roi", img_markerThreCopy);
		if (160 < height && height < 180 && 160 < width && width < 180) {
			img_markerROI = img_markerThreCopy(Rect_<float>(Point2f(left + 1, top - 1), Point2f(right + 1, bottom - 1)));
			//imshow("roi", img_markerROI);

			Mat img_markerROIResize;
			resize(img_markerROI, img_markerROIResize, Size(160, 160));

			//imshow("roi_resize", img_markerROIResize);
			flag = check_marker(1, img_markerROIResize);
		}
		
		if (waitKey(100) == 27)  break;

	}

}

int check_marker(int flag, Mat img_markerROIResize)
{
	int m_arrMarker160by160[160][160] = { 0 };
	for (int y = 0; y < 160; y++) {
		for (int x = 0; x < 160; x++) {
			if (img_markerROIResize.at<uchar>(y, x) == 255)		m_arrMarker160by160[y][x] = 1;	//흰색
			if (img_markerROIResize.at<uchar>(y, x) == 0)		m_arrMarker160by160[y][x] = 0;	//검정색
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
		if (m_arrMarker[i] < 200)	m_arrMarker[i] = 0;		//검정
		else                        m_arrMarker[i] = 1;		//흰색
	}

	for (int i = 0; i < 16; i++)
	{
		if (m_arrMarker[i] == 1)	sum += 0;
		else                        sum += pow(2, i);
	}
	
	if (sum == 231 || sum == 4914 || sum == 19656 || sum == 59136)
	{
		cout << "판교" << endl;
	}
	else if (sum == 1809 || sum == 35040 || sum == 29760 || sum == 558)
	{
		cout << "문래" << endl;
	}
	else if (sum == 5217 || sum == 37472 || sum == 1609 || sum == 34344)
	{
		cout << "구리" << endl;
	}
	else if (sum == 1099 || sum == 4449 || sum == 53792 || sum == 34440)
	{
		cout << "인천" << endl;
	}

	//for (int i = 0; i < 16; i++)			cout << m_arrMarker[i] << '\t';
	cout << endl;

	return 0;
}

int main()
{
	if (!cap.isOpened()) { printf("Can't open the camera");  return -1; }
	m_bThFlag = true;
	std::thread t1(&ThreadFunc);
	
	t1.join();	
	//while (1) { waitKey(); }

	return 0;
}