#include "r200.h"

void ConvertDepthToRGBUsingHistogram(const uint16_t depthImage[], int width, int height, const uint8_t nearColor[3], const uint8_t farColor[3], uint8_t rgbImage[])
{
    // Produce a cumulative histogram of depth values
    int histogram[256 * 256] = {1};
    for (int i = 0; i < width * height; ++i)
    {
        if (auto d = depthImage[i]) ++histogram[d];
    }
    for (int i = 1; i < 256 * 256; i++)
    {
        histogram[i] += histogram[i - 1];
    }

    // Remap the cumulative histogram to the range 0..256
    for (int i = 1; i < 256 * 256; i++)
    {
        histogram[i] = (histogram[i] << 8) / histogram[256 * 256 - 1];
    }

    // Produce RGB image by using the histogram to interpolate between two colors
    auto rgb = rgbImage;
    for (int i = 0; i < width * height; i++)
    {
        if (uint16_t d = depthImage[i]) // For valid depth values (depth > 0)
        {
            auto t = histogram[d]; // Use the histogram entry (in the range of 0..256) to interpolate between nearColor and farColor
            *rgb++ = ((256 - t) * nearColor[0] + t * farColor[0]) >> 8;
            *rgb++ = ((256 - t) * nearColor[1] + t * farColor[1]) >> 8;
            *rgb++ = ((256 - t) * nearColor[2] + t * farColor[2]) >> 8;
        }
        else // Use black pixels for invalid values (depth == 0)
        {
            *rgb++ = 0;
            *rgb++ = 0;
            *rgb++ = 0;
        }
    }
}

R200Grabber::R200Grabber(bool log, float min_distance, float max_distance): 
	p_(std::shared_ptr<DSAPI>(DSCreate(DS_DS4_PLATFORM), DSDestroy)), 
	cnan_(0), qnan_(std::numeric_limits<float>::quiet_NaN()), min_distance_mm_(min_distance), max_distance_mm_(max_distance)
{

	std::cout << "number of connected cameras " << DSGetNumberOfCameras(true) << std::endl;
	serial_ = DSGetCameraSerialNumber(0);
	std::cout << "serial_ " << serial_ << std::endl;

	if(!p_->openDevice())
		std::cout << "device could not be opened" << std::endl;

	if(!p_->probeConfiguration())
		std::cout << "probe failed" << std::endl;

	if(log)
		p_->setLoggingLevelAndFile(DS_LOG_TRACE, "log.txt");

	std::cout << "Firmware version: " << p_->getFirmwareVersionString() << std::endl;
		std::cout << "Software version: " << p_->getSoftwareVersionString() << std::endl;

	p_->enableZ(true);
	p_->enableLeft(true);
	p_->enableRight(true);
	p_->enableLRCrop(true);	
	dresolutions_num_ = p_->getLRZNumberOfResolutionModes(true);
	p_third_ = p_->accessThird();
	p_third_->enableThird(true);
	cresolutions_num_ = p_third_->getThirdNumberOfResolutionModes(true);

	p_hardware_ = p_->accessHardware();
	p_->accessHardware()->setDepthControlParameters(DSGetDepthControlPreset(DS_DEPTHCONTROL_PRESET_OPTIMIZED));

	/*
	if(p_->isCalibrationValid()){
		p_->getCalibRectParameters(params_);
	}
	

	dsettings_.resize(dresolutions_num_);
	csettings_.resize(cresolutions_num_);

	int w, h, fps;
	DSPixelFormat pix;

	std::cout << "supported camera formats: " << dresolutions_num_ << std::endl;
	for(size_t i = 0; i < dresolutions_num_; ++i)
	{
	    p_->getLRZResolutionMode(false, i, w, h, fps, pix);
	    dsettings_[i].w = w;
	    dsettings_[i].h = h;
	    dsettings_[i].fps = fps;
	    dsettings_[i].pix = pix;
	    std::cout << "\t" << i << " : width " << dsettings_[i].w << " height " << dsettings_[i].h << " fps " << dsettings_[i].fps << " pixel format " << dsettings_[i].pix << std::endl;
	    //std::cout << "\t" << i << " : width " << w << " height " << h << " fps " << fps << " pixel format " << pix << std::endl;
	}

	dmode_ = INT_MAX;
	while(!(dmode_ < dresolutions_num_ && dmode_ >= 0)){
		std::cout << "insert mode" << std::endl;
		std::cin >> dmode_;
	}

	std::cout << "supported camera formats: " << cresolutions_num_ << std::endl;
	for(size_t i = 0; i < cresolutions_num_; ++i)
	{
	    p_third_->getThirdResolutionMode(false, i, w, h, fps, pix);
	    csettings_[i].w = w;
	    csettings_[i].h = h;
	    csettings_[i].fps = fps;
	    csettings_[i].pix = pix;
	    std::cout << "\t" << i << " : width " << csettings_[i].w << " height " << csettings_[i].h << " fps " << csettings_[i].fps << " pixel format " << csettings_[i].pix << std::endl;
	    //std::cout << "\t" << i << " : width " << w << " height " << h << " fps " << fps << " pixel format " << pix << std::endl;
	}

	cmode_ = INT_MAX;
	while(!(cmode_ < cresolutions_num_ && cmode_ >= 0)){
		std::cout << "insert mode" << std::endl;
		std::cin >> cmode_;
	}

	p_->setLRZResolutionMode(true, dsettings_[dmode_].w, dsettings_[dmode_].h, dsettings_[dmode_].fps, dsettings_[dmode_].pix); 
	p_third_->setThirdResolutionMode(true, csettings_[cmode_].w, csettings_[cmode_].h, csettings_[cmode_].fps, csettings_[cmode_].pix);
	*/
	p_->setLRZResolutionMode(true, 628, 468, 90, DS_LUMINANCE8); 
	p_third_->setThirdResolutionMode(true, 1920, 1080, 30, DS_RGB8);
	

	height_ = p_->lrHeight(); 
	width_ = p_->lrWidth();

	zheight_ = p_->zHeight();
	zwidth_ = p_->zWidth();

	cwidth_ = p_third_->thirdWidth();
	pixel_width_ = cwidth_ * 3;
	cheight_ = p_third_->thirdHeight();

	setAutoGain();
	p_third_->getCalibExtrinsicsZToRectThird(translation_);
	p_third_->getCalibIntrinsicsRectThird(cintrinsics_);

	p_->getCalibIntrinsicsZ(dintrinsiscs_);
	prepareMake3D(628, 468);
	prepareMakeW2C(628, 468);
	scale_ = p_->getZUnits() / 1000.0;

	if(p_->startCapture()){
		std::cout << "started capture" << std::endl;
	}

	// advance to first valid state
	p_->grab();
}

void * R200Grabber::getColorImageRGB(){
	cv::Mat tmp(cv::Size(1920,1080), CV_8UC3);
	tmp.data = (uint8_t *)p_third_->getThirdImage();

	cv::cvtColor(tmp, tmp, CV_BGR2RGB);
	return (void *)tmp.data;
}

void R200Grabber::setAutoWhiteBalance(bool auto_while_balance){
	if(p_hardware_) 
		p_hardware_->setUseAutoWhiteBalance(auto_while_balance, DS_THIRD_IMAGER);
}

void R200Grabber::setAutoExposure(bool auto_exposure){
	if(p_hardware_) 
		p_hardware_->setAutoExposure(DS_BOTH_IMAGERS, auto_exposure);
}

void R200Grabber::prepareMake3D(const int w, const int h)
{
    float * pm1 = dcolmap.data();
    float * pm2 = drowmap.data();
    for(int i = 0; i < w; i++)
    {
        *pm1++ = (i - dintrinsiscs_.rpx ) / dintrinsiscs_.rfx;
    }
    for (int i = 0; i < h; i++)
    {
        *pm2++ = (i - dintrinsiscs_.rpy ) / dintrinsiscs_.rfy;
    }
}

void R200Grabber::prepareMakeW2C(const int w, const int h)
{
    int * pm1 = ccolmap.data();
    int * pm2 = crowmap.data();
    for(int i = 0; i < w; i++)
    {
        *pm1++ = (int)(   (((float)i - dintrinsiscs_.rpx ) / dintrinsiscs_.rfx ) * cintrinsics_.rfx + cintrinsics_.rpx) * 3;
    }
    for (int i = 0; i < h; i++)
    {
        *pm2++ = (int)((((float)i - dintrinsiscs_.rpy ) / dintrinsiscs_.rfy ) * cintrinsics_.rfy + cintrinsics_.rpy) * cwidth_ * 3;
	}
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr R200Grabber::allocCloud() const
{
	return pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>(zwidth_, zheight_));
}

// Not registered cloud
void R200Grabber::getCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{		
	if(!cloud)
	{
		cloud = allocCloud();
	}
	else
	{
		if(cloud->width != zwidth_ || cloud->height != zheight_){
			std::cout << "wrong cloud size in getCloud(). Correct size is 628x484" << std::endl;
			cloud = nullptr;
		}
	}

	uint16_t * depth = getDepthImage();
	uint8_t * color = (uint8_t *)getColorImageBGR();
	pcl::PointXYZRGB * itP = &cloud->points[0];
	bool is_dense = true;
	
	for(int y = 0; y < zheight_; ++y) 
	{
		const unsigned int offset = y * zwidth_;
		const uint16_t * itD = depth + offset;
		const float dy = drowmap(y);
		const int cdy = crowmap(y);

		for(size_t x = 0; x < zwidth_; ++x, ++itP, ++itD)
		{
			const float depth_value = *itD ; //* scale_;
			if(depth_value >= min_distance_mm_ && depth_value <= max_distance_mm_)
			{
				const float rx = dcolmap(x) * depth_value;
            	const float ry = dy * depth_value;               
				itP->z = depth_value;
				itP->x = rx;
				itP->y = ry;

				const int coffset = cdy  + ccolmap(x);
				itP->r = color[coffset + 0];
				itP->g = color[coffset + 1];
				itP->b = color[coffset + 2];

			}else{
				itP->z = qnan_;
				itP->x = qnan_;
				itP->y = qnan_;

				itP->b = cnan_;
				itP->g = cnan_;
				itP->r = cnan_;
				is_dense = false;
			}
		}
	}
	cloud->is_dense = is_dense;
}

// Registered cloud
void R200Grabber::getCloudT(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{		
	if(!cloud)
	{
		cloud = allocCloud();
	}
	else
	{
		if(cloud->width != zwidth_ || cloud->height != zheight_){
			std::cout << "wrong cloud size in getCloud(). Correct size is 628x484" << std::endl;
			cloud = nullptr;
		}
	}

	uint16_t * depth = getDepthImage();
	uint8_t * color = (uint8_t *)getColorImageBGR();
	pcl::PointXYZRGB * itP = &cloud->points[0];
	bool is_dense = true;
	for(int y = 0; y < zheight_; ++y) 
	{
		const unsigned int offset = y * zwidth_;
		const uint16_t * itD = depth + offset;
		const float dy = drowmap(y);
		//const int cdy = crowmap(y);

		for(size_t x = 0; x < zwidth_; ++x, ++itP, ++itD)
		{
			const float depth_value = *itD;
			if(depth_value >= min_distance_mm_ && depth_value <= max_distance_mm_)
			{
				const float rx = dcolmap(x) * depth_value;
            	const float ry = dy * depth_value;               
				itP->z = depth_value;
				itP->x = rx;
				itP->y = ry;

				// Translation to register color and depth. There is no rotation since the frames are rectified
				const float tx = rx + translation_[0];
				const float ty = ry + translation_[1];

				const int cx = (tx / depth_value) * cintrinsics_.rfx + cintrinsics_.rpx;
				const int cy = (ty / depth_value) * cintrinsics_.rfy + cintrinsics_.rpy;

				const int coffset = cy * pixel_width_ + cx * 3;
				itP->r = color[coffset + 0];
				itP->g = color[coffset + 1];
				itP->b = color[coffset + 2];

			}else{
				itP->z = qnan_;
				itP->x = qnan_;
				itP->y = qnan_;

				itP->b = cnan_;
				itP->g = cnan_;
				itP->r = cnan_;
				is_dense = false;
			}
		}
	}
	cloud->is_dense = is_dense;
}

