#ifdef SendMessage
#undef SendMessage
#endif

#include <RobotRaconteur.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

#include "edu__rpi__cats__sensors__kinect2_tracker.h"
#include "edu__rpi__cats__sensors__kinect2_tracker_stubskel.h"

#include <Windows.h>
#include <Kinect.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <boost/enable_shared_from_this.hpp>
#include <map>

#pragma once

class Kinect2_impl : public edu::rpi::cats::sensors::kinect2_tracker::HandTracker, public boost::enable_shared_from_this < Kinect2_impl >
{
public:

	Kinect2_impl();
	~Kinect2_impl();

	HRESULT StartupKinect();
	HRESULT ShutdownKinect();


	virtual uint8_t EnableSensors();
	virtual uint8_t DisableSensors();


	virtual void start_recording(std::string record_name);
	virtual void stop_recording();

	virtual RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::ImageHeader > getImageHeader();
	virtual RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::ImageHeader > getDepthImageHeader();

	virtual RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image > getCurrentImage();
	virtual RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image16 > getCurrentDepthImage();
	
	
	virtual RR_SHARED_PTR<edu::rpi::cats::sensors::kinect2_tracker::HandData > getLeftHand();

	virtual RR_SHARED_PTR<edu::rpi::cats::sensors::kinect2_tracker::HandData > getRightHand();

private:
	RGBQUAD *color_image_data;
	uint16_t *depth_image_data;

	int color_image_width, color_image_height;
	int depth_image_width, depth_image_height;

	ColorSpacePoint left_color_center, right_color_center;
	std::vector<cv::Point> left_color_points, right_color_points;
	DepthSpacePoint left_depth_center, right_depth_center;
	CameraSpacePoint left_center, right_center;

	bool _recording;
	int record_idx;
	std::string record_base;
	std::ofstream record_file;
	boost::posix_time::ptime record_t0;

	
	IKinectSensor *kinect;
	ICoordinateMapper *coordinate_mapper;
	IMultiSourceFrameReader *multi_reader;
	//DepthSpacePoint *color_points_in_depth_frame;
	CameraSpacePoint *color_points_in_camera_frame;
	WAITABLE_HANDLE h_event;
	boost::mutex mtx_;
	boost::thread t1;

	int FindHandCentersInColorImage();
	int MapHandCentersToDepthImage();
	int MapBlobsToCameraSpace();
	void MultiSourceFrameArrived(IMultiSourceFrameArrivedEventArgs* pArgs);

	void backgroundPollingThread();
	void write_data();

	template<class Interface> inline void SafeRelease(Interface *& pInterfaceToRelease)
	{
		if (pInterfaceToRelease != NULL)
		{
			pInterfaceToRelease->Release();
			pInterfaceToRelease = NULL;
		}
	}
};