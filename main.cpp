#include <stdio.h>
#include <time.h>
#include <iostream>
#include <unistd.h>
#include <sys/time.h>
#include "dxl.h"
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;
using namespace cv::dnn;
using namespace cv::ml;

string src = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)160, height=(int)120, format=(string)NV12, framerate=(fraction)20/1 ! \
     nvvidconv flip-method=0 ! video/x-raw, width=(int)160, height=(int)120, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";

const float CONFIDENCE_THRESHOLD = 0.7;//인식률 임계값
const float NMS_THRESHOLD = 0.5;
const int NUM_CLASSES = 5;//클래스 갯수
const Scalar colors[] = {
{0, 255, 255},
{255, 255, 0},
{0, 255, 0},
{255, 0, 0}
};
const auto NUM_COLORS = sizeof(colors) / sizeof(colors[0]);
int get_area=0;//관심영역 얻기위한 변수 
Mat my_Pretreatment(Mat *frame_);//객체를 받아 전처리 기능 수행 해주는 함수
void my_lane_recognize(Mat *gray_dst,double *pa,double*pb);//전처리 된 Mat 객체를 받아 레이블링 후 면적이 가장 큰 객체의 무게중심 반환
void object_recognize(int input,int box_size);//클래스 번호
void Traffic_Light_recognize(int get_num,int box_size);//신호등 인식 때
void sign_recognize(int get_num,int box_size);//표지판 인식 때
int Traffic_Light_value=0;//모터 제어하는 함수에 필요한 변수
int base_Left_speed = 80, base_Right_speed = 80;
int main()
{
	double prev_value=0.0;
	int prev_x=0,prev_y=0;
	vector<string> class_names = { "blue","yellow","red","no_Straight","right" };
	auto net = readNetFromDarknet("chodata.cfg", "chodata_final.weights");
	net.setPreferableBackend(DNN_BACKEND_CUDA);
	net.setPreferableTarget(DNN_TARGET_CUDA);
	
	auto output_names = net.getUnconnectedOutLayersNames();
	Mat blob;
	vector<Mat> detections;

	dxl_open();//모터 제어 하기위한 함수 설정
	VideoCapture cap1(src, CAP_GSTREAMER);
	if (!cap1.isOpened())
	{
		cerr << "Camera open failed!" << endl;
		return -1;
	}
	struct timeval start, end1;
	double diff1;
	Mat frame, gray;
	int nkey;
	int a=0;
	
	while (true)
	{
		gettimeofday(&start, NULL);
		base_Left_speed = 80, base_Right_speed = 80;//기본 모터 속도 초기화
		get_area=0;//기본 관심영역 초기화
		

		if (kbhit())nkey = getch();//키보드로부터 값이 입력될 때만 호출 뒤 nkey변수에 저장

		cap1 >> frame;//카메라로부터 영상 받아 옴
		if (frame.empty())//영상 받아오기 실패일 때
		{
			cerr << "frame empty!" << endl;
			break;
		}

		gray=my_Pretreatment(&frame);//Mat 객체 전처리 
		blobFromImage(frame, blob, 1 / 255.f, Size(224, 224), cv::Scalar(),true, false, CV_32F);
		//입력 영상으로부터 4차원 블롭객체 생성 및 반환
		net.setInput(blob);//네트워크 입력 설정
		net.forward(detections, output_names);//네트워크 실행 및 레이어 결과 반환
		vector<int> indices[NUM_CLASSES];//객체 갯수 저장 할 int자료형의 vector클래스 생성
		vector<Rect> boxes[NUM_CLASSES];//영역 저장 할 Rect자료형의 vector클래스 생성
		vector<float> scores[NUM_CLASSES];//인식률 저장 할 float자료형의 vector클래스 생성


		for (auto& output : detections)//출력 레이어에 따라 반복
		{
			const auto num_boxes = output.rows;//검출 객체 갯수 저장
			for (int i = 0; i < num_boxes; i++)//반복문
			{
				auto x = output.at<float>(i, 0) * frame.cols;//검출 객체 영역 중심 x좌표 저장
				auto y = output.at<float>(i, 1) * frame.rows;//검출 객체 영역 중심 y좌표 저장
				auto width = output.at<float>(i, 2) * frame.cols;//검출 객체 영역 너비 저장
				auto height = output.at<float>(i, 3) * frame.rows;//검출 객체 영역 높이 저장
				Rect rect(x - width / 2, y - height / 2, width, height);//검출 객체 영역 사각형으로 저장
				for (int c = 0; c < NUM_CLASSES; c++)//클래스 갯수에 따라 반복
				{
					auto confidence = *output.ptr<float>(i, 5 + c);//인식률 저장
					if (confidence >= CONFIDENCE_THRESHOLD)//지정해놓은 인식률보다 높으면 참
					{
						boxes[c].push_back(rect);//사각형 영역 배열에 저장
						scores[c].push_back(confidence);//인식률 배열에 저장
						
						if(c==0||c==1||c==2)Traffic_Light_recognize(c,rect.area());//신호등 인식 
						else sign_recognize(c,rect.area());//표지판 인식
						
					}
				}
			}
		}

	for (int c = 0; c < NUM_CLASSES; c++)NMSBoxes(boxes[c], scores[c], 0.0, NMS_THRESHOLD, indices[c]);
	//검출된 정보들을 가지고 가장 잘 맞는정보 필터링
		for (int c = 0; c < NUM_CLASSES; c++)//클래스 갯수에 따라 반복
		{
			for (int i = 0; i < indices[c].size(); ++i)//검출 갯수에 따라 반복
			{
				const auto color = colors[c % NUM_COLORS];//색 저장
				auto idx = indices[c][i];//검출 객체 번호 저장
				const auto& rect = boxes[c][idx];//검출 객체 사각형 영역 저장
				rectangle(frame, Point(rect.x, rect.y), Point(rect.x + rect.width, 
				rect.y + rect.height), color, 3);//사각형 그려주기

				string label_str = class_names[c] +
				": " + format("%.02lf",scores[c][idx]);
				int baseline;//글자 길이를 저장할 변수
				auto label_bg_sz = getTextSize(label_str, 
				FONT_HERSHEY_COMPLEX_SMALL, 1, 1, &baseline);
				
				putText(frame, label_str, Point(rect.x,rect.height), 
				FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 0, 255));//영상에 글자 출력
			}
		}
		
		Mat dst;//관심영역 저장 할 Mat 객체
		double plus_x=0.0;//x좌표 더할 변수
		double plus_y=0.0;//y좌표 더할 변수
	
		if(get_area==0)//기본 관심영역
		{
			dst=gray(Rect(gray.cols/4,gray.rows/4,gray.cols/2,gray.rows*3/4));
			plus_x=gray.cols/4.0;//원본영상에서 관심영역이 이동한 x좌표
			plus_y=gray.rows/4.0;//원본영상에서 관심영역이 이동한 y좌표
		}
		else if(get_area==1)//직진 금지 표지판 인식 했을 때
		{
			dst=gray(Rect(gray.cols/4,gray.rows/4,gray.cols/4,gray.rows*3/4));
			plus_x=gray.cols/4.0;
			plus_y=gray.rows/4.0;
		}
		else if(get_area==2)//오른쪽 표지판 인식 했을 때
		{
		    dst=gray(Rect(gray.cols/2,gray.rows/4,gray.cols/2,gray.rows*3/4));
		    plus_x=gray.cols/2.0;
			plus_y=gray.rows/4.0;     	
		}
	    if(a==1)//급커브 길일 때
 	    {
			dst=gray(Rect(gray.cols/4,gray.rows*3/4,gray.cols/2,gray.rows/4));
			base_Left_speed = 70, base_Right_speed =70;
			plus_x=gray.cols/4.0;
			plus_y=gray.rows*3/4;
        }
		double pa=0,pb=0;//차선 인식후 무게중심의 x좌표와 y좌표의 값 저장 할 변수
		double error = 0.0;//모터 제어 값 저장 할 변수

		my_lane_recognize(&dst,&pa,&pb);//차선 인식 함수 
		
		pa+=plus_x;//이동한 x좌표만큼 더해줌
		pb+=plus_y;//이동한 y좌표만큼 더해줌

		error = (gray.cols / 2 - pa)/1.4;//제어값 계산

		if(abs(prev_value-error)>20)//(이전 제어값-현재 제어값)의 절대값이 20보다 클 경우(무게중심이 튀었을 경우)
		{
			error=prev_value;//이전 제어값을 현재 제어값에 대입 
			pa=prev_x;//이전 무게중심 x좌표를 현재 무게중심 x좌표에 대입
			pb=prev_y;	//이전 무게중심 y좌표를 현재 무게중심 y좌표에 대입
		}

		//현재 제어 값 들을 이전 제어값을 불러와주는 변수에 대입 
		prev_value=error;
		prev_x=pa;
		prev_y=pb;

		circle(frame, Point2d(pa, pb), 3, Scalar(0,0,255), -1);//차선 인식하는 원 그리기
		

		if(abs(error)>15)a=1;//현재 제어값의 절대값이 15이상일 경우(급 커브일 때)
		else a=0;
		

		if(Traffic_Light_value==0)dxl_set_velocity(base_Left_speed-error,-(base_Right_speed+error));
		else dxl_set_velocity(0,0);//red를 한번이라도 인식하면 호출 후 속도 0 유지->blue를 인식 했을 때 다시 동작

		imshow("src",frame);
		
		waitKey(1);
		gettimeofday(&end1, NULL);
		
        diff1 = end1.tv_sec + end1.tv_usec / 1000000.0 - start.tv_sec - start.tv_usec / 1000000.0;
		
		if (nkey == 'q')break;
		cout <<diff1<<endl;//ms단위로 실행시간 출력
	}
	dxl_close();//모터 제어 설정 종료
	return 0;
}
Mat my_Pretreatment(Mat *frame_)//객체를 받아 전처리 기능 수행 해주는 함수
{
	Mat gray;

	cvtColor(*frame_, gray, 6);//color -> gray
	GaussianBlur(gray,gray,Size()2.0);//블러링(잡음 제거)
	threshold(gray, gray, 170, 255, THRESH_BINARY);//이진화
	
	return gray;
}
void my_lane_recognize(Mat *gray_dst,double *pa,double*pb)//전처리 된 Mat 객체를 받아 레이블링 후 면적이 가장 큰 객체의 무게중심 반환
{
	Mat labels, status, centroids;

	int cnt = connectedComponentsWithStats(*gray_dst, labels, status, centroids);
		
		int num = 0;
		int get_max_labels = 0;
		for (int i = 1;i < cnt;i++)//배경(0번) 제외 가장 큰 면적 찾기
		{
			int* p = status.ptr<int>(i);//i번째 행의 주소를 저장
			if (p[4] > get_max_labels)//면적 비교 
			{
				get_max_labels = p[4];
				num = i;
			}
		}
		double* p = centroids.ptr<double>(num);//가장 큰 면적인 num행의 주소를 저장
		*pa=p[0];//무게중심의 x좌표 
		*pb=p[1];//무게중심의 y좌표
}
void Traffic_Light_recognize(int get_num,int box_size)//신호등을 인식 했을 때
{
	if(get_num==0&&box_size<3000) 
	{
		cout<<"blue 인식"<<endl;
		Traffic_Light_value=0;//모터 제어하는 함수에 필요한 변수
	}
	else if(get_num==1)
	{	
		cout<<"yellow 인식"<<endl;
		base_Left_speed = 40; //왼쪽 모터의 기본속도 조절
		base_Right_speed = 40;//오른쪽 모터의 기본속도 조절
	}
	else if(get_num==2&&box_size<1500) 
	{
		cout<<"red 인식"<<endl;
		Traffic_Light_value=1;
	}
}
void sign_recognize(int get_num,int box_size)//표지판 인식 했을 때
{
	if(get_num==3&&box_size<4000)
	{
		cout<<"직진금지 표지판 인식"<<endl;
		get_area=1;//관심영역 변경에 필요한 변수
    }
	else if(get_num==4&&box_size<4000)
	{
		cout<<"오른쪽 표지판  인식"<<endl;
		get_area=2;
	}
}
