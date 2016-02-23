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

	ColorSpacePoint left_color_point, right_color_point;
	DepthSpacePoint left_depth_point, right_depth_point;
	CameraSpacePoint left_point, right_point;

	
	IKinectSensor *kinect;
	ICoordinateMapper *coordinate_mapper;
	IMultiSourceFrameReader *multi_reader;
	DepthSpacePoint *color_points_in_depth_frame;
	WAITABLE_HANDLE h_event;
	boost::mutex mtx_;
	boost::thread t1;

	int FindHandCentersInColorImage();
	int MapHandCentersToDepthImage();
	void MultiSourceFrameArrived(IMultiSourceFrameArrivedEventArgs* pArgs);

	void backgroundPollingThread();

	template<class Interface> inline void SafeRelease(Interface *& pInterfaceToRelease)
	{
		if (pInterfaceToRelease != NULL)
		{
			pInterfaceToRelease->Release();
			pInterfaceToRelease = NULL;
		}
	}
};