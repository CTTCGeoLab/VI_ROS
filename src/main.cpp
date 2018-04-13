/*main.cpp
 *
 *Created on: Jan 8, 2018
 *Author: David Calero
 *Author: Enric Fernandez
 *Author: CTTC
 */

#include <sys/stat.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>


static const std::string OPENCV_bgr 	= "BGR";
static const std::string OPENCV_ir		= "IR";
static const std::string OPENCV_ir_comp	= "IR_comp";
static const std::string OPENCV_depth 	= "DEPTH";
static const std::string OPENCV_ndvi 	= "NDVI";
//static const std::string OPENCV_diff 	= "DIFF";

bool save_rgb;
bool save_ir;
bool save_ir_comp;
bool save_ndvi;
bool save_depth;
bool display_images;
//bool publish_ndvi;

class NDVIConverter
{
  ros::NodeHandle nh_, priv_nh;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_bgr;
  image_transport::Subscriber image_sub_ir;
  image_transport::Subscriber image_sub_depth;

  ros::Subscriber ndvipointcloud;

  //PUBLISHER
  image_transport::Publisher image_ndvi;
  ros::Publisher ndvipointcloudfiltered1;
  ros::Publisher ndvipointcloudfiltered2;

  std::string path;
  const std::string name_bgr 	= "BGR_";
  const std::string name_ir 	= "IR_";
  //const std::string name_ir_comp= "IR_COMP";
  const std::string name_ndvi 	= "NDVI_";
  const std::string name_depth 	= "Depth_";
  const std::string ext_png 	= ".png";

  cv_bridge::CvImageConstPtr cv_ptr_vis; 	// CVSHARE
  cv_bridge::CvImageConstPtr cv_ptr_nir; 	// CVSHARE
  cv_bridge::CvImageConstPtr cv_ptr_depth; 	// CVSHARE

  std::stringstream color_timestamp ;
  std::stringstream ir_timestamp ;
  std::stringstream depth_timestamp ;

  //char yyyymmdd[16] ;

  //pixel values
  uchar *ptr_pix_nir;
  uchar *ptr_pix_vis;
  uchar *ptr_pix_ndvi;
  uchar *ptr_pix_depth;
  uchar *ptr_pix_nir_comp;

  //Are there sync data to be processed?
  bool nir_rx 		= false;
  bool vis_rx 		= false;
  bool depth_rx 	= false;
  bool ndvi_running = false;

  //How many samples have been processed?
  int cnt_rgb 	= 0;
  int cnt_ir 	= 0;
  int cnt_depth	= 0;

  bool ndvi_one_channel = false;

  ros::Time t_stamp;

public:
  NDVIConverter() : it_(nh_)
  {
	  // READ PARAMS FROM LAUNCH FILE
	  priv_nh = ros::NodeHandle("~");

	  // SET PARAMS FROM LAUNCH FILE OR a default value
	  priv_nh.param("save_rgb", save_rgb, true);
	  priv_nh.param("save_ir", save_ir, true);
	  priv_nh.param("save_ir_comp", save_ir_comp, true);
	  priv_nh.param("save_ndvi", save_ndvi, true);
	  priv_nh.param("save_depth", save_depth, true);
	 // priv_nh.param("publish_ndvi", publish_ndvi, true);
	  priv_nh.param("visualize", display_images, true);
	  priv_nh.param("save_path", path, std::string(""));

	  //Get current Time
	  std::time_t rawtime ;
	  std::time ( &rawtime ) ;
	  const std::tm* timeinfo = std::localtime ( &rawtime ) ;


	  //std::strftime( yyyymmdd, sizeof(yyyymmdd), "%Y%m%d", timeinfo ) ;

	  std::cout << "NDVI PATH: "<< path << std::endl;


	// Subscribe to input RGB registered image
	  image_sub_bgr = it_.subscribe("kinect2/sd/image_color_rect", 2, &NDVIConverter::imageCb_bgr, this);

	// Subscribe to input IR image
	  image_sub_ir = it_.subscribe("kinect2/sd/image_ir_rect", 2, &NDVIConverter::imageCb_ir, this);

	  // Subscribe to input Depth image
	  image_sub_depth = it_.subscribe("kinect2/sd/image_depth_rect", 2, &NDVIConverter::imageCb_depth, this);

	  if (display_images)
	  {
		  cv::namedWindow(OPENCV_bgr);
		  cv::namedWindow(OPENCV_ir);
		  cv::namedWindow(OPENCV_depth);
		  cv::namedWindow(OPENCV_ir_comp);
		  cv::namedWindow(OPENCV_ndvi);
	  }

	 // Publish NDVI image
	  image_ndvi = it_.advertise("ndvi/sd/image_ndvi", 1);

	  //Subscribe to NDVI pointCloud
	  ndvipointcloud = nh_.subscribe("ndvi/sd/points", 1, &NDVIConverter::Point_Cloud_Callback, this);

	  //Publish NDVI PointCloud filtered
	  ndvipointcloudfiltered1 = nh_.advertise<sensor_msgs::PointCloud2>("ndvi/sd/pointsfiltered1", 1);
	  ndvipointcloudfiltered2 = nh_.advertise<sensor_msgs::PointCloud2>("ndvi/sd/pointsfiltered2", 1);
  }

  ~NDVIConverter()
  {
	 image_sub_bgr.shutdown();
	 image_sub_ir.shutdown();
	 image_sub_depth.shutdown();

	// if (publish_ndvi)
	 {
	    image_ndvi.shutdown();
	 }
	 ndvipointcloud.shutdown();
	 ndvipointcloudfiltered1.shutdown();
	 ndvipointcloudfiltered2.shutdown();
	  if (display_images)
	  {
		cv::destroyWindow(OPENCV_bgr);
		cv::destroyWindow(OPENCV_ir);
		cv::destroyWindow(OPENCV_ir_comp);
		cv::destroyWindow(OPENCV_ndvi);
		cv::destroyWindow(OPENCV_depth);
	  }
  }

  void imageCb_bgr(const sensor_msgs::ImageConstPtr& msg)
  {
	//printf("RGB IMAGE CALLBACK \n");
	if (ndvi_running == false)
	{
		char nsec_str[10];
		std::sprintf(nsec_str, "%09u", msg->header.stamp.nsec);

		color_timestamp.str("");
		color_timestamp.clear();
		color_timestamp << msg->header.stamp.sec << nsec_str ;

		std::stringstream filename;
		filename << path << name_bgr << color_timestamp.str() << ext_png;

		//filename << path /*<< yyyymmdd << "/"*/ << name_bgr << color_timestamp.str() << ext_png;

		//std::cout << "RGB PATH: "<< filename.str() << std::endl;
		t_stamp.sec = msg->header.stamp.sec;
		t_stamp.nsec = msg->header.stamp.nsec;

		try
		{
			cv_ptr_vis = cv_bridge::toCvShare(msg, msg->encoding);	 //No modificar toCvShare
			//color_timestamp = msg->header.stamp.sec*1000000000 + msg->header.stamp.nsec;
			vis_rx = true;
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		// Update GUI Window
		if (display_images)
		{
			cv::imshow(OPENCV_bgr, cv_ptr_vis->image);
			cv::waitKey(1);
		}
		//Write image
		if (save_rgb)
		{
			if (cv::imwrite(filename.str(), cv_ptr_vis->image)==false)
			{
				printf("ERROR at saving RGB IMAGE\n");
			}
		}
	}
  }

  void imageCb_ir(const sensor_msgs::ImageConstPtr& msg)
  {
	//printf("IR IMAGE CALLBACK \n");

	if (ndvi_running == false)
	{
		char nsec_str[10];
		std::sprintf(nsec_str, "%09u", msg->header.stamp.nsec);

		ir_timestamp.str("");
		ir_timestamp.clear();
		ir_timestamp << msg->header.stamp.sec << nsec_str;

		std::stringstream filename;
		filename << path  << name_ir << ir_timestamp.str() << ext_png;

		try
		{
			cv_ptr_nir 	= cv_bridge::toCvShare(msg, msg->encoding);
			nir_rx 		= true;
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		// Update GUI Window
		if (display_images)
		{
			cv::imshow(OPENCV_ir, cv_ptr_nir->image);
			cv::waitKey(1);
		}

		//Write image
		if (save_ir)
		{
			if (cv::imwrite(filename.str(), cv_ptr_nir->image)==false)
			{
				printf("ERROR at saving IR IMAGE\n");
			}
		}
	}
  }

  void imageCb_depth(const sensor_msgs::ImageConstPtr& msg)
  {
	//printf("DEPTH IMAGE CALLBACK\n");
	  if (ndvi_running == false)
	  {

		char nsec_str[10];
		std::sprintf(nsec_str, "%09u", msg->header.stamp.nsec);

		depth_timestamp.str("");
		depth_timestamp.clear();
		depth_timestamp << msg->header.stamp.sec << nsec_str ;

		std::stringstream filename;
		filename << path << name_depth << depth_timestamp.str() << ext_png;


		try
		{
			cv_ptr_depth 	= cv_bridge::toCvShare(msg, msg->encoding);
			depth_rx 		= true;
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		// Update GUI Window
		 if (display_images)
		 {
			cv::imshow(OPENCV_depth, cv_ptr_depth->image);
			cv::waitKey(1);
		 }

		//Write image
		 if (save_depth)
		 {
			if (cv::imwrite(filename.str(), cv_ptr_depth->image)==false)
			{
				printf("ERROR at saving DEPTH IMAGE\n");
			}
		 }
	  }
  }

 /* void compute_diff()
	{
	  if (nir_rx && vis_rx)
	  {
		  if (color_timestamp == ir_timestamp)
		  {
				mtx_vis.lock();//LOCK MUTEX
				mtx_nir.lock();//LOCK MUTEX

				//1. CONVERT BGR TO GRAY
				cv::Mat cv_ptr_gray;
				cv::cvtColor(cv_ptr_vis->image, cv_ptr_gray, cv::COLOR_BGR2GRAY);
				//cv::imshow("MONOCHROME", cv_ptr_mono);
				//cv::waitKey(3);

				//2. SCALE NIR FROM CV_16UC1 TO CV_8UC1
				cv::Mat cv_ptr_nir_scaled;
				cv_ptr_nir->image.convertTo(cv_ptr_nir_scaled, CV_8U, 0.00390625); // scale factor: 1/256 = 0.00390625
				//cv::imshow("NIR_SCALE", cv_ptr_nir_scaled);
				//cv::waitKey(3);

				float diff_timestamp = color_timestamp;
				nir_rx = false;
				vis_rx = false;

				mtx_vis.unlock();//UNLOCK MUTEX
				mtx_nir.unlock();//UNLOCK MUTEX

				//3. Extract canny edges from GRAY
				cv::Canny( cv_ptr_gray, cv_ptr_gray, 50, 150, 3 );
				//cv::imshow("GRAY_CANNY", cv_ptr_gray);
				//cv::waitKey(3);

				//4. Extract canny edges NIR
				cv::Canny( cv_ptr_nir_scaled, cv_ptr_nir_scaled, 50, 150, 3 );
				//cv::imshow("NIR_CANNY", cv_ptr_nir_scaled);
				//cv::waitKey(3);

				//5. MAKE DIFFERENCE BETWEEN IR AND GRAY
				cv::Mat result_image;
				cv::absdiff( cv_ptr_gray, cv_ptr_nir_scaled, result_image );

				//6. PLOT IMAGE DIFFERENCES
				cv::imshow(OPENCV_diff, result_image);
				cv::waitKey(3);
		  }
		}
	}
*/


  /*uchar ndvi_coloring(float ndvi){
	  if(ndvi < 0.0)
		  return 0; //black
	  else if()
	  else()
  }*/


  //to change a pixel value in function of its depth
 /*float pixel_DepthAdjustment(uchar * pix_nir, uchar * pix_depth){
	  float pix_nir_depthcorrected; //value of a NIR pixel when depth correction is applied
	  float nir=float(pix_nir[0]), depth=float(pix_depth[0]);
	  pix_nir_depthcorrected = nir + depth;
	  return 0.0;
  }*/


  cv::Mat compensate_IR(cv::Mat nir_mat, cv::Mat depth_mat)
  {
		//1. Create output IR compensated
		cv::Mat nir_compensated;
		nir_compensated.create(nir_mat.rows,nir_mat.cols,  CV_8UC1);

		float DN = 0.0f;
		float value = 0.0f;
		float depth_meters = 0.0f;
		for (int i = 0 ; i< nir_compensated.rows; i++ )
		{
			ptr_pix_nir 		= nir_mat.ptr(i);				//input: nir
			//ptr_pix_depth 		= depth_mat.ptr(i); 			//input::depth
			ptr_pix_nir_comp 	= nir_compensated.ptr(i);		//output: ndvi

			for (int j = 0 ; j< nir_compensated.cols; j++ )
			{
				//depth_meters = (float)(1.0 / ((double)(depth_mat.at<ushort>(i,j)) * -0.0030711016 + 3.3309495161));
				depth_meters = (float) ((float)depth_mat.at<ushort>(i,j))/1000;
				DN = -67.432 * ((float) (depth_meters*depth_meters*depth_meters)) + 387.58 * ((float) (depth_meters*depth_meters)) - 745.17 * ((float) (depth_meters)) + 508.65;
				value = ((float)ptr_pix_nir[0]) * (140 / DN);
				nir_compensated.at<uchar>(i,j) = (uchar)(value);
				ptr_pix_nir++;
				ptr_pix_depth++;
				ptr_pix_nir_comp++;
				if ((i == (nir_compensated.rows/2)) && (j== (nir_compensated.cols/2)))
				{
					printf ("Center depth Value:%u\n",depth_mat.at<ushort>(i,j));
					printf ("Center DN Value:%f\n",DN);
					printf ("Center compensated Value:%f\n",value);
				}
			}
		}

		//2. PLOT IR COMPENSATED IMAGE
		if (display_images)
		{
			cv::imshow(OPENCV_ir_comp, nir_compensated);
			cv::waitKey(1);
		}
		return nir_compensated;
  }

  void compute_ndvi()
  {

	if ((nir_rx) && (vis_rx) && (depth_rx) && (color_timestamp.str().compare(ir_timestamp.str())==0) && (color_timestamp.str().compare(depth_timestamp.str())==0))
	{
		ndvi_running = true;
		nir_rx = false;
		vis_rx = false;
		depth_rx = false;

		//1. SCALE NIR FROM CV_16UC1 TO CV_8UC1
		cv::Mat cv_ptr_nir_scaled;
		cv_ptr_nir->image.convertTo(cv_ptr_nir_scaled, CV_8U, 0.00390625); // scale factor: 1/256 = 0.00390625

		//2. Extract RED channel from BGR
		cv::Mat planes[3];
		cv::split(cv_ptr_vis->image,planes);  // planes[2] is the red channel

		//3. FUNCTION TO COMPENSATE IR
		cv::Mat cv_ptr_nir_compensated = compensate_IR(cv_ptr_nir_scaled, cv_ptr_depth->image);

		//5. GENERATE NDVI IMAGE
		cv::Mat ndvi;
		if (ndvi_one_channel)
		{
			ndvi.create(cv_ptr_nir_compensated.rows,cv_ptr_nir_compensated.cols,  CV_8UC1);

			for (int i = 0 ; i< planes[2].rows; i++ )
			{
				ptr_pix_vis = planes[2].ptr(i);			//input: vis
				ptr_pix_nir = cv_ptr_nir_compensated.ptr(i);	//input: nir
				ptr_pix_ndvi = ndvi.ptr(i);				//output: ndvi

				for (int j = 0 ; j< planes[2].cols; j++ )
				{
					float value = (((float)ptr_pix_nir[0] - (float)ptr_pix_vis[0])/((float)ptr_pix_nir[0] + (float)ptr_pix_vis[0]));
					value++;
					ndvi.at<uchar>(i,j) = (uchar)(value * 127);
					ptr_pix_vis++;
					ptr_pix_nir++;
					ptr_pix_ndvi++;
				}
			}
		}
		else
		{
			ndvi.create(cv_ptr_nir_compensated.rows,cv_ptr_nir_compensated.cols,  CV_8UC3);

			for (int i = 0 ; i< planes[2].rows; i++ )
			{
				ptr_pix_vis = planes[2].ptr(i);			//input: vis
				ptr_pix_nir = cv_ptr_nir_compensated.ptr(i);	//input: nir
				ptr_pix_ndvi = ndvi.ptr(i);				//output: ndvi

				for (int j = 0 ; j< planes[2].cols; j++ )
				{
					float value = (((float)ptr_pix_nir[0] - (float)ptr_pix_vis[0])/((float)ptr_pix_nir[0] + (float)ptr_pix_vis[0]));
					value++;
					uchar ndvi_value = (uchar)(value * 127);
					ndvi.at<cv::Vec3b>(i,j)[0] = ndvi_value;
					ndvi.at<cv::Vec3b>(i,j)[1] = ndvi_value;
					ndvi.at<cv::Vec3b>(i,j)[2] = ndvi_value;
					ptr_pix_vis++;
					ptr_pix_nir++;
					ptr_pix_ndvi++;
				}
			}
		}

		//4. PLOT NDVI IMAGE
		if (display_images)
		{
			cv::imshow(OPENCV_ndvi, ndvi);
			cv::waitKey(1);
		}

		//5. SAVE NDVI IMAGE
		if (save_ndvi)
		{
			std::stringstream filename;
			filename << path << name_ndvi << color_timestamp.str() << ext_png;
			if (cv::imwrite(filename.str(), ndvi)==false)
			{
				printf("ERROR at saving NDVI IMAGE\n");
			}
		}

		//6. Publish NDVI IMAGE
		//if (publish_ndvi)
		{
			if (ndvi_one_channel)
			{
				sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", ndvi).toImageMsg();
				msg->header.stamp.sec = t_stamp.sec;
				msg->header.stamp.nsec = t_stamp.nsec;
				msg->header.frame_id = "kinect2_ir_optical_frame";
				image_ndvi.publish(msg);
			}else
			{
				sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", ndvi).toImageMsg();
				msg->header.stamp.sec = t_stamp.sec;
				msg->header.stamp.nsec = t_stamp.nsec;
				msg->header.frame_id = "kinect2_ir_optical_frame";
				image_ndvi.publish(msg);
			}
		}
		ndvi_running = false;
	}

  }

  void Point_Cloud_Callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {

	  int cnt = 0;
	  // Input: Convert sensor_msgs to PointCloud
	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	  pcl::fromROSMsg (*cloud_msg, *pointcloud);

	  //Output1 (Filter by color)
	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered1 (new pcl::PointCloud<pcl::PointXYZRGB>);
	  pcl::PointIndices::Ptr inliers1(new pcl::PointIndices());
	  pcl::ExtractIndices <pcl::PointXYZRGB> extract1;

	  for (size_t i = 0; i < pointcloud->size(); i++)
	  {
		  if (pointcloud->points[i].r == 254)
		  {

			  inliers1->indices.push_back(i);
			  cnt++;
		  }
	  }
	  //printf("POINTCLOUD filtered points %d\n",cnt);
	  extract1.setInputCloud(pointcloud);
	  extract1.setIndices(inliers1);
	  extract1.setNegative(true);
	  extract1.filter(*cloud_filtered1);

	  // Convert to ROS data type
	  sensor_msgs::PointCloud2 output1;
	  pcl::toROSMsg(*cloud_filtered1, output1);

	  // Publish the data
	  ndvipointcloudfiltered1.publish (output1);

	  //Output2 (Filter by distance)
	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	  pcl::PointIndices::Ptr inliers2(new pcl::PointIndices());
	  pcl::ExtractIndices <pcl::PointXYZRGB> extract2;

	  for (size_t i = 0; i < cloud_filtered1->size(); i++)
	  {
		  if ((cloud_filtered1->points[i].z < 0.75) || (cloud_filtered1->points[i].z > 1.5))
		  {

			  inliers2->indices.push_back(i);
			  cnt++;
		  }
	  }
	  extract2.setInputCloud(cloud_filtered1);
	  extract2.setIndices(inliers2);
	  extract2.setNegative(true);
	  extract2.filter(*cloud_filtered2);

	  // Convert to ROS data type
	  sensor_msgs::PointCloud2 output2;
	  pcl::toROSMsg(*cloud_filtered2, output2);

	  // Publish the data
	  ndvipointcloudfiltered2.publish (output2);
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, OPENCV_ndvi);
  NDVIConverter ndvi;
  while (ros::ok())
  {
	  ndvi.compute_ndvi();
	  ros::spinOnce();
  }
  return 0;
}
