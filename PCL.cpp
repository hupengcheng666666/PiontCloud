// PCL.cpp : 定义控制台应用程序的入口点。
//

//包含文件*******************************************************************************************************************************__Include
#include "stdafx.h"
#include <iostream>
#include <Windows.h>
#include <Kinect.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#define NOMINMAX

//命名空间*******************************************************************************************************************************__Namespace
using namespace cv;
using namespace std;

//安全释放函数建立**********************************************************************************************************************__SafeRelease
template<class Interface>//安全释放
inline void SafeRelease(Interface *& pInterfaceToRelease)
{

	if (pInterfaceToRelease != NULL){
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

//————————————————————————————————————————主函数————————————————————————————————————————//
//——————————————————————————————————————程序入口—————————————————————————————————————————//

int main()
{
	//Kinect初始化***********************************************************************************************************************__Sensor
	IKinectSensor* pSensor;
	HRESULT hResult = S_OK;
	hResult = GetDefaultKinectSensor(&pSensor);
	if (FAILED(hResult)){
		std::cerr << "Error : GetDefaultKinectSensor" << std::endl;
		return -1;
	}

	//打开Kinect
	hResult = pSensor->Open();//打开kinect
	if (FAILED(hResult)){
		std::cerr << "Error : IKinectSensor::Open()" << std::endl;
		return -1;
	}



	//获取彩色图得源*******************************************************************************************************************__Source
	IColorFrameSource* pColorSource;//彩色图
	hResult = pSensor->get_ColorFrameSource(&pColorSource);
	if (FAILED(hResult)){
		std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
		return -1;
	}

	//获取深色图得源
	IDepthFrameSource* pDepthSource;//深度图
	hResult = pSensor->get_DepthFrameSource(&pDepthSource);
	if (FAILED(hResult)){
		std::cerr << "Error : IKinectSensor::get_DepthFrameSource()" << std::endl;
		return -1;
	}



	// 创建彩色图读口*****************************************************************************************************************__Reader
	IColorFrameReader* pColorReader;
	hResult = pColorSource->OpenReader(&pColorReader);
	if (FAILED(hResult)){
		std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	// 创建深色图读口
	IDepthFrameReader* pDepthReader;
	hResult = pDepthSource->OpenReader(&pDepthReader);
	if (FAILED(hResult)){
		std::cerr << "Error : IDepthFrameSource::OpenReader()" << std::endl;
		return -1;
	}



	// 获取彩色图像帧属性**********************************************************************************************************__Color Description
	IFrameDescription* pColorDescription;
	hResult = pColorSource->get_FrameDescription(&pColorDescription);
	if (FAILED(hResult)){
		std::cerr << "Error : IColorFrameSource::get_FrameDescription()" << std::endl;
		return -1;
	}
	int colorWidth = 0;
	int colorHeight = 0;
	pColorDescription->get_Width(&colorWidth); // 1920
	pColorDescription->get_Height(&colorHeight); // 1080
	std::vector<RGBQUAD> colorBuffer(colorWidth * colorHeight);

	//彩色图准备
	/*unsigned int colorBufferSize = colorWidth * colorHeight * 4 * sizeof(unsigned char);
	cv::Mat colorBuffer(colorHeight, colorWidth, CV_8UC4);
	cv::Mat colorMat(colorHeight / 2, colorWidth / 2, CV_8UC4);*/
	//::namedWindow("Color");



	// 获取深色图像帧属性***********************************************************************************************************__Depth Description
	IFrameDescription* pDepthDescription;
	hResult = pDepthSource->get_FrameDescription(&pDepthDescription);
	if (FAILED(hResult)){
		std::cerr << "Error : IDepthFrameSource::get_FrameDescription()" << std::endl;
		return -1;
	}
	int depthWidth = 0;
	int depthHeight = 0;
	pDepthDescription->get_Width(&depthWidth); // 512
	pDepthDescription->get_Height(&depthHeight); // 424

	//深色图准备
	unsigned int depthBufferSize = depthWidth * depthHeight * sizeof(unsigned short);
	cv::Mat depthBufferMat(depthHeight, depthWidth, CV_16UC1);
	cv::Mat depthMat(depthHeight, depthWidth, CV_8UC1);
	//cv::namedWindow("Depth");


	//映射初始化*********************************************************************************************************************__Mapper
	ICoordinateMapper* pCoordinateMapper;
	hResult = pSensor->get_CoordinateMapper(&pCoordinateMapper);
	if (FAILED(hResult)){
		std::cerr << "Error : IKinectSensor::get_CoordinateMapper()" << std::endl;
		return -1;
	}



	//点云准备：定义文件路径名称*****************************************************************************************************__Could File Name
	//string dst_img_name = "F://PCL//点云1__4_1//PCL//could";
	//char chari[10000];
	//int n = 0;



	//可视化：创建点云查看器***************************************************************************************************************__Could Viewer
	pcl::visualization::CloudViewer viewer("Point Cloud Viewer");



	//循环**************************************************************************************************************************__While
	while (1)
	{
		/************************************获取彩色帧*********************************/
		IColorFrame* pColorFrame = nullptr;
		hResult = pColorReader->AcquireLatestFrame(&pColorFrame);
		if (SUCCEEDED(hResult)){
			hResult = pColorFrame->CopyConvertedFrameDataToArray(colorBuffer.size()* sizeof(RGBQUAD), reinterpret_cast<BYTE*>(&colorBuffer[0]), ColorImageFormat::ColorImageFormat_Bgra);
			//if (SUCCEEDED(hResult)){
			//	cv::resize(colorBufferMat, colorMat, cv::Size(), 0.5, 0.5);//尺寸改变
			//}
		}


		/*************************************获取深度帧******************************/
		IDepthFrame* pDepthFrame = nullptr;
		hResult = pDepthReader->AcquireLatestFrame(&pDepthFrame);
		if (SUCCEEDED(hResult)){
			hResult = pDepthFrame->AccessUnderlyingBuffer(&depthBufferSize, reinterpret_cast<UINT16**>(&depthBufferMat.data));
			if (SUCCEEDED(hResult)){
				depthBufferMat.convertTo(depthMat, CV_16U, -255.0f / 8000.0f, 255.0f);//类型转换  , -255.0f / 8000.0f, 255.0f
			}
		}



		/******************************创建云点—得到信息****************************/
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>());
		pointcloud->width = static_cast<uint32_t>(depthWidth);
		pointcloud->height = static_cast<uint32_t>(depthHeight);
		pointcloud->is_dense = true;



		/*****************************映射********************************************/
		for (int y = 0; y < depthHeight; y++){

			for (int x = 0; x < depthWidth; x++){
				//创建点（包含X,Y,G,R,G,B）
				pcl::PointXYZRGB point;    
				//创建深度空间点
				DepthSpacePoint depthSpacePoint = {
					static_cast<float>(x), static_cast<float>(y)
				};

				//UINT16 depth = depthBufferMat.data[y * depthWidth + x];
				UINT16 depth = depthMat.data[y * depthWidth + x];
				//创建彩色空间点
				ColorSpacePoint colorSpacePoint = {
					0.0f, 0.0f
				};
				// 坐标映射颜色的深度空间,设置PointCloud RGB
				pCoordinateMapper->MapDepthPointToColorSpace(depthSpacePoint, depth, &colorSpacePoint);
				int colorX = static_cast<int>(std::floor(colorSpacePoint.X + 0.5f));
				int colorY = static_cast<int>(std::floor(colorSpacePoint.Y + 0.5f));
				if ((0 <= colorX) && (colorX < colorWidth) && (0 <= colorY) && (colorY < colorHeight)){

					RGBQUAD color = colorBuffer[colorY * colorWidth + colorX];
					point.b = color.rgbBlue;
					point.g = color.rgbGreen;
					point.r = color.rgbRed;

				}

				// 创建相机空间点
				CameraSpacePoint cameraSpacePoint = {
					0.0f, 0.0f, 0.0f
				};
				//深度相机坐标映射空间, 设置PointCloud XYZ
				pCoordinateMapper->MapDepthPointToCameraSpace(depthSpacePoint, depth, &cameraSpacePoint);
				if ((0 <= colorX) && (colorX < colorWidth) && (0 <= colorY) && (colorY < colorHeight)){

					point.x = cameraSpacePoint.X;
					point.y = cameraSpacePoint.Y;
					point.z = cameraSpacePoint.Z;

				}
				//插入点
				pointcloud->push_back(point);
			}
		}


		/*****************************在查看器中显示点云*****************************/
		viewer.showCloud(pointcloud);


		/****************************点云数据输出保存******************************/
		//sprintf_s(chari, "%04d", n);
		//dst_img_name += "cloud_";
		//dst_img_name += chari;
		//dst_img_name += ".pcd";


		///***************************保持输出**************************************/
		//pcl::io::savePCDFile(dst_img_name, *pointcloud);

		//dst_img_name = "F://PCL//点云1__4_1//PCL//could";

		//n++;

		///*****************************读取云点数据**********************************/
		/*pcl::PCDReader reader;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
		reader.read(dst_img_name, *cloud);
		std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl;*/


		///*****************************可视化云点数据******************************/
		//pcl::visualization::PCLVisualizer viewer("Cloud Viewer"); //窗口显示第pit个聚类
		//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> color(cloud); //显示点云RGB信息
		//viewer.addPointCloud(cloud, color);
		//viewer.setBackgroundColor(0, 255, 255);


		/************************************释放帧**********************************/
		SafeRelease(pColorFrame);
		SafeRelease(pDepthFrame);


		/*********************************显示图像***********************************/
		/*cv::imshow("Color", colorBuffer);
		cv::imshow("Depth", depthBufferMat);*/


		/*****************************结束条件（Esc）********************************/
		if (cv::waitKey(1) == VK_ESCAPE){
			break;


		}
		
	}



	//程序结束—释放*******************************************************************************************************************__End
	SafeRelease(pColorSource);
	SafeRelease(pDepthSource);
	SafeRelease(pColorReader);
	SafeRelease(pDepthReader);
	SafeRelease(pColorDescription);
	SafeRelease(pDepthDescription);
	SafeRelease(pCoordinateMapper);
	if (pSensor){
		pSensor->Close();
	}
	SafeRelease(pSensor);
	cv::destroyAllWindows();
	return 0;
}