#include "r200.h"
#include <pcl/visualization/cloud_viewer.h>

void
KeyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void * data)
{
  R200Grabber * g = (R200Grabber*)data;
  std::shared_ptr<DSAPI> p = g->getDSAPI();
  if(event.keyDown())
  {
    if(event.getKeySym() == "o")
    {
      p->accessHardware()->setDepthControlParameters(DSGetDepthControlPreset(DS_DEPTHCONTROL_PRESET_OFF));
      std::cout << "activated DS_DEPTHCONTROL_PRESET_OFF" << std::endl;
    }
    if(event.getKeySym() == "p")
    {
      p->accessHardware()->setDepthControlParameters(DSGetDepthControlPreset(DS_DEPTHCONTROL_PRESET_LOW));
      std::cout << "activated DS_DEPTHCONTROL_PRESET_LOW" << std::endl;
    }
    if(event.getKeySym() == "k")
    {
      p->accessHardware()->setDepthControlParameters(DSGetDepthControlPreset(DS_DEPTHCONTROL_PRESET_MEDIUM));
      std::cout << "activated DS_DEPTHCONTROL_PRESET_MEDIUM" << std::endl;
    }
    if(event.getKeySym() == "l")
    {
      p->accessHardware()->setDepthControlParameters(DSGetDepthControlPreset(DS_DEPTHCONTROL_PRESET_HIGH));
      std::cout << "activated DS_DEPTHCONTROL_PRESET_HIGH" << std::endl;
    }
    if(event.getKeySym() == "m")
    {
      p->accessHardware()->setDepthControlParameters(DSGetDepthControlPreset(DS_DEPTHCONTROL_PRESET_OPTIMIZED));
      std::cout << "activated DS_DEPTHCONTROL_PRESET_OPTIMIZED" << std::endl;
    }
  }
}

int main(int argc, char * argv[])
{
	std::cout << "press q to exit" << std::endl;
	R200Grabber grabber;
	const uint8_t nearColor[] = {255, 0, 0};
	const uint8_t farColor[] = {20, 40, 255};
	uint8_t cdepth[628 * 468 * 3];

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = grabber.allocCloud();
	grabber.getCloudT(cloud);

	cloud->sensor_orientation_.w() = 0.0;
	cloud->sensor_orientation_.x() = 1.0;
	cloud->sensor_orientation_.y() = 0.0;
	cloud->sensor_orientation_.z() = 0.0;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->registerKeyboardCallback(KeyboardEventOccurred, (void*)&grabber);

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");


	cv::Mat limage(grabber.getLRSize(), CV_8UC1);
    cv::Mat rimage(grabber.getLRSize(), CV_8UC1);
    cv::Mat dimage(grabber.getDepthSize(), CV_8UC3);
    cv::Mat cimage(grabber.getColorSize(), CV_8UC4);

	double ftime;
	int frame;
	int dwidth = grabber.getDepthSize().width;
	int dheight = grabber.getDepthSize().height;
	bool first = true;

	while(!viewer->wasStopped())
	{
		if(grabber.hasData() ){

			std::chrono::high_resolution_clock::time_point tnow = std::chrono::high_resolution_clock::now();
		    ftime = grabber.getFrameTime();
		    frame = grabber.getFrameNumber();
		    
		    //std::cout << "Frame " << frame << " @ " << std::fixed << ftime << std::endl;
		    
		    const uint16_t * pd = grabber.getDepthImage();
		    const void * pl = grabber.getLImage();
		    const void * pr = grabber.getRImage();
		    const void * pc = grabber.getColorImage();

		    ConvertDepthToRGBUsingHistogram(pd, dwidth, dheight, nearColor, farColor, cdepth);

    		//std::cout << "delta " << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now()-tnow).count() * 1000 << std::endl;

		    limage.data = (uint8_t *)pl;
		    rimage.data = (uint8_t *)pr;
		    dimage.data = (uint8_t *)cdepth;
		    cimage.data = (uint8_t *)pc;
/*
    		cv::imshow("limage", limage);
    		cv::imshow("rimage", rimage);
    		cv::imshow("cimage", cimage);
    		cv::imshow("dimage", dimage);
*/

    		grabber.getCloudT(cloud);
    		/*
    		if(first){
    			for(auto & p : cloud->points)
    			std::cout << p.x << " " << p.y << " " << p.z << " " << (int)p.r << " " << (int)p.g << " " << (int)p.b << std::endl;
    			first = false;
    		}*/
    		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    		viewer->updatePointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
    		viewer->spinOnce();
		}
	}

	return 0;
}
