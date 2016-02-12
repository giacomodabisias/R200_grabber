#include "DSAPI.h"
#include <iostream>
#include <chrono>
#include <unistd.h>
#include <climits>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "DSAPI/DSDepthControlParametersUtil.h"

struct Setting
{
	Setting(): w(0), h(0), fps(0) {}
	int w, h, fps;
	DSPixelFormat pix;
};

inline void ConvertDepthToRGBUsingHistogram(const uint16_t depthImage[], int width, int height, const uint8_t nearColor[3], const uint8_t farColor[3], uint8_t rgbImage[])
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

inline void printRT(const float * rotation, const float * translation){

	std::cout << "rotation:" << std::endl;
	for(size_t i = 0; i < 3; ++i){
		std::cout << "\t" << rotation[i*3] << ", " << rotation[i*3+1] << ", " << rotation[i*3+2] << std::endl;
	}
	std::cout << std::endl << "translation:" << std::endl;
	std::cout << "\t" << translation[0] << "," << translation[1] << "," << translation[2] << std::endl;
	std::cout << std::endl;
}

class R200Grabber {

public:
	R200Grabber(bool log = true, float min_distance = 600, float max_distance = 3000);

	bool hasData(){
		return p_->grab();
	}

	double * getRegistrationTranslation(){
		return translation_;
	}

	void set_auto_exposure(bool auto_exposure){
		if(p_hardware_) 
			p_hardware_->setAutoExposure(DS_BOTH_IMAGERS, auto_exposure);
	}

	void set_auto_white_balance(bool auto_while_balance){
		if(p_hardware_) 
			p_hardware_->setUseAutoWhiteBalance(auto_while_balance, DS_THIRD_IMAGER);
	}

	void * getColorImage(){
		return p_third_->getThirdImage();
	}

	uint16_t * getDepthImage(){
		return p_->getZImage();
	}

	void * getLImage(){
		return p_->getLImage();
	}

	void * getRImage(){
		return p_->getRImage();
	}

	double getFrameTime() const{
		return p_->getFrameTime();
	}

	int getFrameNumber(){
		return p_->getFrameNumber();
	}

	cv::Size getColorSize() const {
		return cv::Size(cwidth_, cheight_);
	}

	cv::Size getDepthSize() const {
		return cv::Size(zwidth_, zheight_);
	}

	cv::Size getLRSize() const {
		return cv::Size(width_, height_);
	}

	void setImageExposure(float exposure){
		p_hardware_->setImagerExposure(exposure, DS_BOTH_IMAGERS);
	}

	void setImageGain(float gain){
		p_hardware_->setImagerGain(gain, DS_BOTH_IMAGERS);
	}

	void setAutoGain(){
		p_hardware_->setAutoExposure(DS_BOTH_IMAGERS, true);
	}

	DSPixelFormat getPixelTypeColor() const {
		return csettings_[cmode_].pix;
	}

	std::shared_ptr<DSAPI> getDSAPI(){
		return p_;
	}

	void getCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
	void getCloudT(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr allocCloud() const;

private:

	void prepareMake3D(const int w, const int h)
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

	void getColorXY(int x, int y, int & x1, int & y1) const 
	{
		x1 = x + translation_[0];
		y1 = y + translation_[1];
	}

	void prepareMakeW2C(const int w, const int h)
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


	void makeD2CMap(){
		for(size_t i = 0; i < 628; ++i)
			ymap_[i] = i + translation_[0];
		for(size_t j = 0; j < 468; ++j)
			xmap_[j] = j + translation_[1];
	}



	std::shared_ptr<DSAPI> p_;
	DSThird * p_third_;
	DSHardware * p_hardware_;
	uint32_t serial_;
	int dresolutions_num_, cresolutions_num_;
	std::vector<Setting> dsettings_, csettings_;
	int height_, width_, zheight_, zwidth_, cheight_, cwidth_, cmode_, dmode_;
	double translation_[3];
	float qnan_;
	uint8_t cnan_;
	short xmap_[468];
	short ymap_[628];
	Eigen::Matrix<int,628,1> ccolmap;
	Eigen::Matrix<int,468,1> crowmap;
	Eigen::Matrix<float,628,1> dcolmap;
	Eigen::Matrix<float,468,1> drowmap;
	DSCalibIntrinsicsRectified dintrinsiscs_;
	DSCalibIntrinsicsRectified cintrinsics_;
	float max_distance_mm_, min_distance_mm_;
	float scale_;
};

inline R200Grabber::R200Grabber(bool log, float min_distance, float max_distance): 
	p_(std::shared_ptr<DSAPI>(DSCreate(DS_DS4_PLATFORM), DSDestroy)), 
	cnan_(0),
	qnan_(std::numeric_limits<float>::quiet_NaN()), min_distance_mm_(min_distance), max_distance_mm_(max_distance)
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
	}*/

	//p_->setLRZResolutionMode(true, dsettings_[dmode_].w, dsettings_[dmode_].h, dsettings_[dmode_].fps, dsettings_[dmode_].pix); 
	//p_third_->setThirdResolutionMode(true, csettings_[cmode_].w, csettings_[cmode_].h, csettings_[cmode_].fps, csettings_[cmode_].pix);
	p_->setLRZResolutionMode(true, 628, 468, 90, DS_LUMINANCE8); 
	p_third_->setThirdResolutionMode(true, 1920, 1080, 30, DS_RGB8);


	height_ = p_->lrHeight(); 
	width_ = p_->lrWidth();

	zheight_ = p_->zHeight();
	zwidth_ = p_->zWidth();

	cwidth_ = p_third_->thirdWidth();
	cheight_ = p_third_->thirdHeight();

	setAutoGain();
	p_third_->getCalibExtrinsicsZToRectThird(translation_);
	//std::cout << translation_[0] << " " << translation_[1] << " " << translation_[2] << std::endl;
	p_third_->getCalibIntrinsicsRectThird(cintrinsics_);

	makeD2CMap();
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

inline pcl::PointCloud<pcl::PointXYZRGB>::Ptr R200Grabber::allocCloud() const
{
	return pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>(zwidth_, zheight_));
}

inline void R200Grabber::getCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
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
	uint8_t * color = (uint8_t *)getColorImage();
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


inline void R200Grabber::getCloudT(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
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
	uint8_t * color = (uint8_t *)getColorImage();
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

				const float tx = rx + translation_[0];
				const float ty = ry + translation_[1];

				const int cx = (tx / depth_value) * cintrinsics_.rfx + cintrinsics_.rpx;
				const int cy = (ty / depth_value) * cintrinsics_.rfy + cintrinsics_.rpy;


				const int coffset = cy * cwidth_ * 3 + cx * 3;
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
