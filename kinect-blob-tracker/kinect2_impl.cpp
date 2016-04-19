#include "kinect2_impl.h"

Kinect2_impl::Kinect2_impl() : edu::rpi::cats::sensors::kinect2_tracker::HandTracker()
{

	this->multi_reader = NULL;
	HRESULT hr = StartupKinect();
	if (FAILED(hr))
	{
		std::cout << "Failed to Startup Kinect: error code " << HRESULT_CODE(hr) << std::endl;
		return;
	}
	// Allocate memory for the different image streams
	this->depth_image_data = new uint16_t[this->depth_image_width * this->depth_image_height];
	this->color_image_data = new RGBQUAD[this->color_image_width * this->color_image_height];
	//this->color_points_in_depth_frame = new DepthSpacePoint[this->color_image_width * this->color_image_height];
	this->color_points_in_camera_frame = new CameraSpacePoint[this->color_image_width * this->color_image_height];

	this->left_color_center = ColorSpacePoint();
	this->left_depth_center = DepthSpacePoint();
	this->left_center = CameraSpacePoint();

	this->right_color_center = ColorSpacePoint();
	this->right_depth_center = DepthSpacePoint();
	this->right_center = CameraSpacePoint();

	this->_recording = false;

	t1 = boost::thread(boost::bind(&Kinect2_impl::backgroundPollingThread, this));

}

Kinect2_impl::~Kinect2_impl()
{
	stop_recording();
	ShutdownKinect();
	delete depth_image_data;
	delete color_image_data;
}

HRESULT Kinect2_impl::StartupKinect()
{
	HRESULT hr = NULL;

	// Attempt access to default Kinect-2 sensor
	std::cout << "Looking for Default Kinect Sensor" << std::endl;
	hr = GetDefaultKinectSensor(&this->kinect);
	if (FAILED(hr))
		return hr;

	// If Kinect is found, initialize
	if (this->kinect)
	{
		std::cout << "Found Kinect, Initializing..." << std::endl;

		hr = this->kinect->Open();
		if FAILED(hr) { return hr; }

		// Get Color Frame Information
		std::cout << "Getting Color Frame Info...";
		IFrameDescription *color_frame_description = NULL;
		IColorFrameSource *color_frame_source = NULL;
		hr = this->kinect->get_ColorFrameSource(&color_frame_source);
		if FAILED(hr) { return hr; }
		color_frame_source->get_FrameDescription(&color_frame_description);
		if FAILED(hr) { return hr; }
		color_frame_description->get_Width(&this->color_image_width);
		color_frame_description->get_Height(&this->color_image_height);
		SafeRelease(color_frame_source);
		SafeRelease(color_frame_description);
		std::cout << "Success" << std::endl;

		// Get Depth Frame Information
		std::cout << "Getting Depth Frame Info...";
		IFrameDescription *depth_frame_description = NULL;
		IDepthFrameSource *depth_frame_source = NULL;
		hr = this->kinect->get_DepthFrameSource(&depth_frame_source);
		if FAILED(hr) { return hr; }
		hr = depth_frame_source->get_FrameDescription(&depth_frame_description);
		if FAILED(hr) { return hr; }
		depth_frame_description->get_Width(&this->depth_image_width);
		depth_frame_description->get_Height(&this->depth_image_height);
		SafeRelease(depth_frame_source);
		SafeRelease(depth_frame_description);
		std::cout << "Success" << std::endl;

		std::cout << "Getting Coordinate Mapper...";
		hr = this->kinect->get_CoordinateMapper(&this->coordinate_mapper);
		if FAILED(hr) { return hr; }

		std::cout << "Success" << std::endl;

	}

	return hr;
}

HRESULT Kinect2_impl::ShutdownKinect()
{
	HRESULT hr = E_FAIL;

	this->DisableSensors();

	t1.interrupt();
	t1.join();

	SafeRelease(coordinate_mapper);

	if (kinect)
		hr = kinect->Close();
	SafeRelease(kinect);
	return hr;
}



uint8_t Kinect2_impl::EnableSensors()
{
	// Open MultiSource Reader with user-specified sources open
	if SUCCEEDED(this->kinect->OpenMultiSourceFrameReader(FrameSourceTypes_Color | FrameSourceTypes_Depth, &this->multi_reader))
	{
		multi_reader->SubscribeMultiSourceFrameArrived(&this->h_event);
		return 1;
	}
	else
		return 0;
}

uint8_t Kinect2_impl::DisableSensors()
{
	HRESULT hr;
	// Close MultiSource Reader
	if (multi_reader != NULL)
	{
		hr = this->multi_reader->put_IsPaused(true);
		if SUCCEEDED(hr)
			std::cout << "Successfully Paused Multi-Source Reader" << std::endl;

		SafeRelease(this->multi_reader);
	}

	return 1;
}


void Kinect2_impl::start_recording(std::string record_name)
{
	std::string folder_name = "./" + record_name + "_images";
	boost::filesystem::path images_folder(folder_name);
	if (boost::filesystem::create_directory(images_folder))
		std::cout << "Saving images to folder " << folder_name << std::endl;

	this->record_base = folder_name + "/" + record_name;
	this->record_file = std::ofstream(this->record_base + ".csv");
	this->record_idx = 0;
	this->_recording = true;
	this->record_t0 = boost::posix_time::microsec_clock::local_time();
}

void Kinect2_impl::stop_recording()
{
	this->_recording = false;

	this->record_file.close();
}

void Kinect2_impl::write_data()
{
	boost::lock_guard<boost::mutex> guard(mtx_);

	//cv::Mat color_image_mat = cv::Mat(this->color_image_height, this->color_image_width, CV_8UC4, this->color_image_data);
	cv::Mat depth_image_mat = cv::Mat(this->depth_image_height, this->depth_image_width, CV_16UC1, this->depth_image_data);

	std::stringstream ss;
	std::string color_image_name, depth_image_name;

	ss << this->record_base << "_color_" << this->record_idx << ".png";
	ss >> color_image_name;
	ss.clear();
	ss << this->record_base << "_depth_" << this->record_idx << ".png";
	ss >> depth_image_name;

	//cv::imwrite(color_image_name, color_image_mat);
	//cv::imwrite(depth_image_name, depth_image_mat);

	boost::posix_time::time_duration t_elapsed = boost::posix_time::microsec_clock::local_time() - this->record_t0;

	this->record_file << t_elapsed.total_microseconds() / 1000000.f;
	this->record_file << ", " << this->left_center.X << ", " << this->left_center.Y << ", " << this->left_center.Z;
	this->record_file << ", " << this->right_center.X << ", " << this->right_center.Y << ", " << this->right_center.Z << std::endl;

	this->record_idx++;
}

RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::ImageHeader > Kinect2_impl::getImageHeader()
{
	boost::lock_guard<boost::mutex> guard(mtx_);

	RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::ImageHeader> I(new edu::rpi::cats::sensors::camera_interface::ImageHeader());

	I->width = this->color_image_width;
	I->height = this->color_image_height;
	I->channels = 4;
	I->step = 1;

	return I;

}
RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::ImageHeader > Kinect2_impl::getDepthImageHeader()
{
	boost::lock_guard<boost::mutex> guard(mtx_);

	RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::ImageHeader> I(new edu::rpi::cats::sensors::camera_interface::ImageHeader());

	I->width = this->depth_image_width;
	I->height = this->depth_image_height;
	I->channels = 1;
	I->step = 2;

	return I;

}

RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image > Kinect2_impl::getCurrentImage()
{
	boost::lock_guard<boost::mutex> guard(mtx_);

	RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image> I(new edu::rpi::cats::sensors::camera_interface::Image());

	I->width = this->color_image_width;
	I->height = this->color_image_height;
	I->channels = 4;
	I->data = RobotRaconteur::AttachRRArrayCopy<uint8_t>(reinterpret_cast<uint8_t *>(this->color_image_data), I->width * I->height * I->channels);
	
	return I;

}
RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image16 > Kinect2_impl::getCurrentDepthImage()
{
	boost::lock_guard<boost::mutex> guard(mtx_);

	RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image16> I(new edu::rpi::cats::sensors::camera_interface::Image16());

	I->width = this->depth_image_width;
	I->height = this->depth_image_height;
	I->channels = 1;
	I->data = RobotRaconteur::AttachRRArrayCopy<uint16_t>(reinterpret_cast<uint16_t *>(this->depth_image_data), I->width * I->height * I->channels);

	return I;

}

RR_SHARED_PTR<edu::rpi::cats::sensors::kinect2_tracker::HandData >  Kinect2_impl::getLeftHand()
{
	boost::lock_guard<boost::mutex> guard(mtx_);

	RR_SHARED_PTR<edu::rpi::cats::sensors::kinect2_tracker::HandData> H(new edu::rpi::cats::sensors::kinect2_tracker::HandData());

	H->color_x = this->left_color_center.X;
	H->color_y = this->left_color_center.Y;
	H->depth_x = this->left_depth_center.X;
	H->depth_y = this->left_depth_center.Y;
	H->x = this->left_center.X;
	H->y = this->left_center.Y;
	H->z = this->left_center.Z;

	return H;
}

RR_SHARED_PTR<edu::rpi::cats::sensors::kinect2_tracker::HandData >  Kinect2_impl::getRightHand()
{
	boost::lock_guard<boost::mutex> guard(mtx_);

	RR_SHARED_PTR<edu::rpi::cats::sensors::kinect2_tracker::HandData> H(new edu::rpi::cats::sensors::kinect2_tracker::HandData());

	H->color_x = this->right_color_center.X;
	H->color_y = this->right_color_center.Y;
	H->depth_x = this->right_depth_center.X;
	H->depth_y = this->right_depth_center.Y;
	H->x = this->right_center.X;
	H->y = this->right_center.Y;
	H->z = this->right_center.Z;

	return H;
}

int Kinect2_impl::FindHandCentersInColorImage()
{
	// Set up cv::Mat wrapper around color image data and convert to BGR format
	cv::Mat color_image_mat = cv::Mat(this->color_image_height, this->color_image_width, CV_8UC4, this->color_image_data);
	cv::Mat color_image_bgr;
	cv::cvtColor(color_image_mat, color_image_bgr, CV_BGRA2BGR);

	// Convert from BGR to HSV
	cv::Mat color_image_hsv, color_image_hsv_thresh;
	cv::cvtColor(color_image_bgr, color_image_hsv, CV_BGR2HSV);

	// Threshold and segment out bluish-purple color
	cv::inRange(color_image_hsv, cv::Scalar(100, 100, 128), cv::Scalar(110, 200, 255), color_image_hsv_thresh);
	cv::erode(color_image_hsv_thresh, color_image_hsv_thresh, cv::Mat());
	cv::dilate(color_image_hsv_thresh, color_image_hsv_thresh, cv::Mat());

	color_image_mat.release();

	// Display thresholded color image
	cv::Mat thresh_resize;
	cv::resize(color_image_hsv_thresh, thresh_resize, cv::Size(), 0.25, 0.25);
	cv::imshow("thresh", thresh_resize);
	cv::waitKey(1);

	// Detect connected components and stats about each particular label
	cv::Mat cc_labels, cc_stats, cc_centroids;
	int n_cc = cv::connectedComponentsWithStats(color_image_hsv_thresh, cc_labels, cc_stats, cc_centroids);
	
	// Find largest two blobs (not counting the background)
	if (n_cc >= 3)
	{
		int max_a1 = 0, max_a2 = 0;
		int max_i1 = 0, max_i2 = 0;
		for (int i = 1; i < n_cc; i++)
		{
			if (cc_stats.at<int>(i, cv::CC_STAT_AREA) > max_a1)
			{
				max_a2 = max_a1;
				max_i2 = max_i1;
				max_a1 = cc_stats.at<int>(i, cv::CC_STAT_AREA);
				max_i1 = i;
			}
			else if (cc_stats.at<int>(i, cv::CC_STAT_AREA) > max_a2)
			{
				max_a2 = cc_stats.at<int>(i, cv::CC_STAT_AREA);
				max_i2 = i;
			}
		}
		// Determine which blob is associated with the left or right hand
		int left_i, right_i;
		if (cc_centroids.at<double>(max_i1, 0) < cc_centroids.at<double>(max_i2, 0))
		{
			left_i = max_i1;
			right_i = max_i2;
		}
		else
		{
			left_i = max_i2;
			right_i = max_i1;
		}

		// Save detected centroids of each blob and associated pixels for each blob
		this->left_color_center.X = cc_centroids.at<double>(left_i, 0);
		this->left_color_center.Y = cc_centroids.at<double>(left_i, 1);
		this->right_color_center.X = cc_centroids.at<double>(right_i, 0);
		this->right_color_center.Y = cc_centroids.at<double>(right_i, 1);

		this->left_color_points.clear();
		for (int i = cc_stats.at<int>(left_i, cv::CC_STAT_TOP);
			i < cc_stats.at<int>(left_i, cv::CC_STAT_TOP) + cc_stats.at<int>(left_i, cv::CC_STAT_HEIGHT); i++)
		{
			for (int j = cc_stats.at<int>(left_i, cv::CC_STAT_LEFT);
				j < cc_stats.at<int>(left_i, cv::CC_STAT_LEFT) + cc_stats.at<int>(left_i, cv::CC_STAT_WIDTH); j++)
			{
				if (cc_labels.at<int>(i, j) == left_i)
					this->left_color_points.push_back(cv::Point(j, i));
			}
		}

		this->right_color_points.clear();
		for (int i = cc_stats.at<int>(right_i, cv::CC_STAT_TOP);
			i < cc_stats.at<int>(right_i, cv::CC_STAT_TOP) + cc_stats.at<int>(right_i, cv::CC_STAT_HEIGHT); i++)
		{
			for (int j = cc_stats.at<int>(right_i, cv::CC_STAT_LEFT);
				j < cc_stats.at<int>(right_i, cv::CC_STAT_LEFT) + cc_stats.at<int>(right_i, cv::CC_STAT_WIDTH); j++)
			{
				if (cc_labels.at<int>(i, j) == right_i)
					this->right_color_points.push_back(cv::Point(j, i));
			}
		}

	}
	else
		return -1;

	return 0;
}

int Kinect2_impl::MapHandCentersToDepthImage()
{
	/*UINT left_color_idx, right_color_idx;
	UINT left_depth_idx, right_depth_idx;
	UINT n_points;

	// Left point
	left_color_idx = (UINT)(this->left_color_center.Y) * (UINT)(this->color_image_width) + (UINT)this->left_color_center.X;
	this->left_depth_center = this->color_points_in_depth_frame[left_color_idx];
	left_depth_idx = (UINT)(this->left_depth_center.Y) * (UINT)(this->depth_image_width) + (UINT)this->left_depth_center.X;
	if ((left_depth_idx < 0) || (left_depth_idx >= this->depth_image_height * this->depth_image_width))
		return -1;
	this->coordinate_mapper->MapDepthPointToCameraSpace(this->color_points_in_depth_frame[left_color_idx], this->depth_image_data[left_depth_idx], &this->left_center);

	// Right point
	right_color_idx = (UINT)(this->right_color_center.Y) * (UINT)(this->color_image_width) + (UINT)this->right_color_center.X;
	this->right_depth_center = this->color_points_in_depth_frame[right_color_idx];
	right_depth_idx = (UINT)(this->right_depth_center.Y) * (UINT)(this->depth_image_width) + (UINT)this->right_depth_center.X;
	if ((right_depth_idx < 0) || (right_depth_idx >= this->depth_image_height * this->depth_image_width))
		return -1;
	this->coordinate_mapper->MapDepthPointToCameraSpace(this->color_points_in_depth_frame[right_color_idx], this->depth_image_data[right_depth_idx], &this->right_center);
	*/
	return 0;
}

int Kinect2_impl::MapBlobsToCameraSpace()
{

	CameraSpacePoint left_center_cs = CameraSpacePoint();
	CameraSpacePoint right_center_cs = CameraSpacePoint();
	int n_points = this->left_color_points.size();

	for (int i = 0; i < this->left_color_points.size(); i++)
	{
		int k = (UINT)(this->left_color_points.at(i).y) * (UINT)(this->color_image_width) + (UINT)this->left_color_points.at(i).x;
		if (this->color_points_in_camera_frame[k].Z >= 0.4 && this->color_points_in_camera_frame[k].Z <= 4)
		{
			left_center_cs.X += this->color_points_in_camera_frame[k].X;
			left_center_cs.Y += this->color_points_in_camera_frame[k].Y;
			left_center_cs.Z += this->color_points_in_camera_frame[k].Z;
		}
		else
			n_points--;
	}

	if (n_points == 0)
	{
		this->left_center = CameraSpacePoint();
		return -1;
	}
	else
	{
		this->left_center.X = left_center_cs.X / n_points;
		this->left_center.Y = left_center_cs.Y / n_points;
		this->left_center.Z = left_center_cs.Z / n_points;
	}

	n_points = this->right_color_points.size();
	for (int i = 0; i < this->right_color_points.size(); i++)
	{
		int k = (UINT)(this->right_color_points.at(i).y) * (UINT)(this->color_image_width) + (UINT)this->right_color_points.at(i).x;
		if (this->color_points_in_camera_frame[k].Z >= 0.4 && this->color_points_in_camera_frame[k].Z <= 4)
		{
			right_center_cs.X += this->color_points_in_camera_frame[k].X;
			right_center_cs.Y += this->color_points_in_camera_frame[k].Y;
			right_center_cs.Z += this->color_points_in_camera_frame[k].Z;
		}
		else
			n_points--;
	}

	if (n_points == 0)
	{
		this->right_center = CameraSpacePoint();
		return -1;
	}
	else
	{
		this->right_center.X = right_center_cs.X / n_points;
		this->right_center.Y = right_center_cs.Y / n_points;
		this->right_center.Z = right_center_cs.Z / n_points;
	}

	return 0;
}

void Kinect2_impl::MultiSourceFrameArrived(IMultiSourceFrameArrivedEventArgs* pArgs)
{
	HRESULT hr;
	IMultiSourceFrameReference *multi_frame_reference = NULL;
	IMultiSourceFrame *multi_frame = NULL;
	bool color_frame_acquired, depth_frame_acquired;
	
	hr = pArgs->get_FrameReference(&multi_frame_reference);
	if FAILED(hr)
	{
		//std::cout << "Could not access multi-source reference.  Failed" << std::endl;
		SafeRelease(multi_frame_reference);
		return;
	}

	multi_frame_reference->AcquireFrame(&multi_frame);

	//std::cout << "Looking at Color Frame Data" << std::endl;
	IColorFrameReference *color_frame_reference = NULL;
	IColorFrame *color_frame = NULL;
	hr = multi_frame->get_ColorFrameReference(&color_frame_reference);
	// Acquire Frame
	if SUCCEEDED(hr)
		hr = color_frame_reference->AcquireFrame(&color_frame);

	if SUCCEEDED(hr)
	{
		//std::cout << "Copying to buffer...";
		UINT buffer_size = this->color_image_width * this->color_image_height * sizeof(RGBQUAD);
		hr = color_frame->CopyConvertedFrameDataToArray(buffer_size, reinterpret_cast<BYTE *>(this->color_image_data), ColorImageFormat_Bgra);
		//std::cout << ((SUCCEEDED(hr)) ? "Success" : "Failed") << std::endl;
	}

	SafeRelease(color_frame);
	SafeRelease(color_frame_reference);
	color_frame_acquired = SUCCEEDED(hr);
	
	//std::cout << "Looking at Depth Frame Data" << std::endl;
	IDepthFrameReference *depth_frame_reference = NULL;
	IDepthFrame *depth_frame = NULL;
	hr = multi_frame->get_DepthFrameReference(&depth_frame_reference);
	if (SUCCEEDED(hr))
		hr = depth_frame_reference->AcquireFrame(&depth_frame);

	if (SUCCEEDED(hr))
	{
		//std::cout << "Copying to buffer...";
		UINT buffer_size = this->depth_image_width * this->depth_image_height;
		hr = depth_frame->CopyFrameDataToArray(buffer_size, reinterpret_cast<UINT16 *>(this->depth_image_data));
		//std::cout << ((SUCCEEDED(hr)) ? "Success" : "Failed") << std::endl;
	}
	SafeRelease(depth_frame);
	SafeRelease(depth_frame_reference);
	depth_frame_acquired = SUCCEEDED(hr);

	SafeRelease(multi_frame);
	SafeRelease(multi_frame_reference);

	if (color_frame_acquired && depth_frame_acquired)
	{
		//std::cout << "Attempting to map color frame into depth space...";
		//hr = this->coordinate_mapper->MapColorFrameToDepthSpace(this->depth_image_height * this->depth_image_width, this->depth_image_data,
		//	this->color_image_height * this->color_image_width, this->color_points_in_depth_frame);
		hr = this->coordinate_mapper->MapColorFrameToCameraSpace(this->depth_image_height * this->depth_image_width, this->depth_image_data,
														this->color_image_height * this->color_image_width, this->color_points_in_camera_frame);
		//std::cout << ((SUCCEEDED(hr)) ? "Success" : "Failed") << std::endl;
		if (FindHandCentersInColorImage() < 0)
		{
			this->left_color_center = ColorSpacePoint();
			this->left_depth_center = DepthSpacePoint();
			this->left_center = CameraSpacePoint();

			this->right_color_center = ColorSpacePoint();
			this->right_depth_center = DepthSpacePoint();
			this->right_center = CameraSpacePoint();
		}
		else
		{
			if (MapBlobsToCameraSpace() < 0)
			{
				this->left_color_center = ColorSpacePoint();
				this->left_depth_center = DepthSpacePoint();
				this->left_center = CameraSpacePoint();

				this->right_color_center = ColorSpacePoint();
				this->right_depth_center = DepthSpacePoint();
				this->right_center = CameraSpacePoint();
			}
		}
		if (this->_recording)
			this->write_data();
	}

}

void Kinect2_impl::backgroundPollingThread()
{
	HRESULT hr;
	DWORD res;
	std::cout << "Starting up background thread" << std::endl;
	while (true)
	{
		try
		{
			res = WaitForSingleObjectEx(reinterpret_cast<HANDLE>(h_event), 1000, false);
			if (multi_reader != NULL)
			{
				IMultiSourceFrameArrivedEventArgs *pArgs = NULL;
				hr = this->multi_reader->GetMultiSourceFrameArrivedEventData(this->h_event, &pArgs);
				if (SUCCEEDED(hr))
					MultiSourceFrameArrived(pArgs);
				else
					std::cout << "Dropped Frame" << std::endl;
				SafeRelease(pArgs);

			}
			boost::this_thread::interruption_point();
		}
		catch (...) {
			break;
		}
	}

	std::cout << "Exiting Background Thread" << std::endl;
}