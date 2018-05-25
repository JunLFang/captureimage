/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/captureimage>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <thread>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

//#include<System.h>

#include <opencv2/opencv.hpp>
#include "camera.h"
#include "utility.h"
#include <unistd.h>
#include <mutex>

using namespace std;
using namespace mynteye;
using namespace cv;

//void Draw2dMap(Mat &pose, Mat &background, double timestamp);
std::vector<cv::Mat> g_photo_left, g_photo_right;
std::mutex mtx;
bool bRun = true;
void WriteImage(){
	int index = 0;
	while (bRun){
		std::vector<cv::Mat> photo_left,photo_right;
		mtx.lock();
		photo_left = g_photo_left;
		g_photo_left.clear();
		photo_right = g_photo_right;
		photo_right.clear();
		mtx.unlock();
		usleep(1);
		for(auto photo:photo_left){
			cv::imwrite(std::string("photo")+to_string(index)+".png",photo);
			index++;
		}
		for(auto photo:photo_right){
			cv::imwrite(std::string("photo")+to_string(index)+".png",photo);
			index++;
		}
	}
}

int main(int argc, char **argv)
{
    Mat background(1200,1200,CV_32F);
    int slam_flag=1;
    std::uint32_t timestamp;
    float time;
    vector<IMUData> imudatas;
    // Open camera
    std::cout << "Open Camera: "<< std::endl;
    stringstream device;
    device << argv[3];
    mynteye::Camera cam;
   // cam.SetMode(MODE_GPU);
//    cout<<"device :"<<device.str()<<"/n"<<endl;

    mynteye::InitParameters params(device.str());
    std::cout << "Open Camera: "<< device.str()<< std::endl;

    cam.Open(params);
    std::cout << "Open Camera: "  << std::endl;
    if (!cam.IsOpened()) {
        std::cerr << "Error: Open camera failed" << std::endl;
        return 1;
    }
    std::cout << "\033[1;32mPress ESC/Q on Windows to terminate\033[0m\n";

    while(cam.Grab() != ErrorCode::SUCCESS) {
        std::cout << "Warning: Grab failed <" << ">" << std::endl;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    //ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    //ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);


    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;


	int insert_count = 25;
    cv::Mat img_left;
    cv::Mat img_right;
    int n=0;
	int empty_count = 0;
	std::thread thread_save(WriteImage);

    while(1)
    {

        cam.Grab();
        cam.RetrieveImage(img_left, View::VIEW_LEFT_UNRECTIFIED);
        cam.RetrieveImage(img_right, View::VIEW_RIGHT_UNRECTIFIED);
        cam.RetrieveIMUData(imudatas,timestamp);
		string time_stamp=to_string(timestamp);
		time_stamp.append(".jpg");
        time = static_cast<float>(static_cast<float>(timestamp)/10000.0);

		if(!img_left.empty()){

			flip(img_left,img_left,-1);
			if(n%insert_count == 0){
				mtx.lock();
				g_photo_left.push_back(img_left);
				mtx.unlock();
			}
			imshow("Left image",img_left);
			empty_count = 0;
		}else{
			empty_count++;
		}

		if(!img_right.empty()){
			flip(img_right,img_right,-1);
			if(n%insert_count == 0){
				mtx.lock();
				g_photo_right.push_back(img_right);
				mtx.unlock();
			}
			imshow("Right image",img_right);
			empty_count = 0;
		}else{
			empty_count++;
		}
		if((empty_count>20)||(27 == waitKey(1e3/50))){
			break;
		}
//		cv::imwrite("/home/jimmy/VSLAM/Testimage/"+time_stamp,img_left);
		++n;
	}

    cam.Close();


    return 0;

}

