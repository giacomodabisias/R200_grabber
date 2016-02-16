#pragma once
#include "DSAPI.h"
#include <iostream>
#include <unistd.h>
#include <climits>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "DSAPI/DSDepthControlParametersUtil.h"

struct Setting
{
	Setting(): w(0), h(0), fps(0) {}
	int w, h, fps;
	DSPixelFormat pix;
};

class R200Grabber {

public:
	R200Grabber(bool log = true, float min_distance = 600, float max_distance = 3000);

	bool hasData(){
		return p_->grab();
	}

	double * getRegistrationTranslation(){
		return translation_;
	}

	void setAutoExposure(bool auto_exposure);
	void setAutoWhiteBalance(bool auto_while_balance);

	void * getColorImageRGB();

	void * getColorImageBGR(){
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

	void prepareMake3D(const int w, const int h);
	void prepareMakeW2C(const int w, const int h);

	std::shared_ptr<DSAPI> p_;
	DSThird * p_third_;
	DSHardware * p_hardware_;
	uint32_t serial_;
	int dresolutions_num_, cresolutions_num_;
	std::vector<Setting> dsettings_, csettings_;
	int height_, width_, zheight_, zwidth_, cheight_, cwidth_, cmode_, dmode_, pixel_width_;
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
