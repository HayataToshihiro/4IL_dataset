#include <ros/ros.h>
#include <cstdlib> 

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/OccupancyGrid.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <tf/tf.h>

class mkdataset{
	private:
		ros::NodeHandle nh;
		/*subscribe*/
		ros::Subscriber sub_rmground;
		ros::Subscriber sub_ground;
		ros::Subscriber sub_human;
		/*publish*/
		ros::Publisher pub_grid;
		ros::Publisher pub_image;
		/*cloud*/
		pcl::PointCloud<pcl::PointXYZI>::Ptr rmground {new pcl::PointCloud<pcl::PointXYZI>};
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr ground {new pcl::PointCloud<pcl::PointXYZINormal>};
		pcl::PointCloud<pcl::PointXYZI>::Ptr human {new pcl::PointCloud<pcl::PointXYZI>};
		/*grid*/
		nav_msgs::OccupancyGrid grid;
		nav_msgs::OccupancyGrid grid_all_minusone;
		/*publish infomations*/
		std::string pub_frameid;
		ros::Time pub_stamp;
		/*const values*/
		const double w = 20.0;	//x[m]
		const double h = 20.0;	//y[m]
		const double resolution = 0.1;	//[m]
		const int image_w = int(w/resolution);
		const int image_h = int(h/resolution);
		/* flags */
		bool firstcallback_rmg = true;
		bool firstcallback_gnd = true;
		bool firstcallback_hum = true;
		/*image*/
		sensor_msgs::ImagePtr image_ros;
	public:
		mkdataset();
		void Initialization(void);
		void CallbackRmGround(const sensor_msgs::PointCloud2ConstPtr& msg);
		void CallbackGround(const sensor_msgs::PointCloud2ConstPtr& msg);
		void CallbackHuman(const sensor_msgs::PointCloud2ConstPtr& msg);
		void ExtractPCInRange(pcl::PointCloud<pcl::PointXYZI>::Ptr pc);
		void InputGrid(void);
		int MeterpointToIndex(double x, double y);
		int MeterpointToPixel_x(double x);
		int MeterpointToPixel_y(double y);
		void Publication(void);
};


mkdataset::mkdataset()
{
	sub_rmground = nh.subscribe("/rm_ground", 1, &mkdataset::CallbackRmGround, this);
	sub_ground = nh.subscribe("/ground", 1, &mkdataset::CallbackGround, this);
	sub_human = nh.subscribe("/human", 1, &mkdataset::CallbackHuman, this);
	pub_grid = nh.advertise<nav_msgs::OccupancyGrid>("/occupancygrid/lidar", 1);
	pub_image = nh.advertise<sensor_msgs::Image>("/image", 1);
	Initialization();
}

void mkdataset::Initialization(void)/*{{{*/
{
	// std::cout<<"initialization"<<std::endl;
	grid.info.resolution = resolution;
	grid.info.width = w/resolution + 1;
	grid.info.height = h/resolution + 1;
	grid.info.origin.position.x = -w/2.0;
	grid.info.origin.position.y = -h/2.0;
	grid.info.origin.position.z = 0.0;
	grid.info.origin.orientation.x = 0.0;
	grid.info.origin.orientation.y = 0.0;
	grid.info.origin.orientation.z = 0.0;
	grid.info.origin.orientation.w = 1.0;
	for(int i=0;i<grid.info.width*grid.info.height;i++)	grid.data.push_back(-1);
	// frame_id is same as the one of subscribed pc
	grid_all_minusone = grid;

}/*}}}*/

void mkdataset::CallbackRmGround(const sensor_msgs::PointCloud2ConstPtr &msg)/*{{{*/
{
	// std::cout<<"callbackrmground"<<std::endl;
	firstcallback_rmg = false;
	pcl::fromROSMsg(*msg, *rmground);

	ExtractPCInRange(rmground);

	pub_frameid = msg->header.frame_id;
	pub_stamp = msg->header.stamp;

	InputGrid();
	Publication();
}/*}}}*/

void mkdataset::CallbackGround(const sensor_msgs::PointCloud2ConstPtr &msg)/*{{{*/
{
	// std::cout<<"callbacground"<<std::endl;
	firstcallback_gnd = false;
	pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_pc {new pcl::PointCloud<pcl::PointXYZI>};
	pcl::fromROSMsg(*msg, *tmp_pc);
	
	ExtractPCInRange(tmp_pc);
	
	pcl::copyPointCloud(*tmp_pc, *ground);
}/*}}}*/

void mkdataset::CallbackHuman(const sensor_msgs::PointCloud2ConstPtr &msg)/*{{{*/
{
	// std::cout<<"callbackhuman"<<std::endl;
	firstcallback_hum = false;
	pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_pc {new pcl::PointCloud<pcl::PointXYZI>};
	pcl::fromROSMsg(*msg, *tmp_pc);
	
	ExtractPCInRange(tmp_pc);
	
	pcl::copyPointCloud(*tmp_pc, *human);
}/*}}}*/
void mkdataset::ExtractPCInRange(pcl::PointCloud<pcl::PointXYZI>::Ptr pc)/*{{{*/
{
	// std::cout<<"extractpcinrange"<<std::endl;
	pcl::PassThrough<pcl::PointXYZI> pass;
	pass.setInputCloud(pc);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-w/2.0, w/2.0);
	pass.filter(*pc);
	pass.setInputCloud(pc);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-h/2.0, h/2.0);
	pass.filter(*pc);
}/*}}}*/

void mkdataset::InputGrid(void)
{
	std::cout<<"imputgrid"<<std::endl;
	grid = grid_all_minusone;
	cv::Mat image(cv::Size(image_w,image_h), CV_8UC3, cv::Scalar(0,0,0));


	//ground
	// std::cout<<ground->points.size()<<std::endl;
 	// if(ground->points.size()){
	// 	for(size_t i=0;i<ground->points.size();i++){
	// 		grid.data[MeterpointToIndex(ground->points[i].x, ground->points[i].y)] = 0;
	// 		cv::Vec3b *src = image.ptr<cv::Vec3b>(MeterpointToPixel_y(ground->points[i].x));
	// 		src[MeterpointToPixel_x(ground->points[i].y)][0]=0;
	// 		src[MeterpointToPixel_x(ground->points[i].y)][1]=0;
	// 		src[MeterpointToPixel_x(ground->points[i].y)][2]=0;
	// 	}
	// }

	//obstacle
	// std::cout<<rmground->points.size()<<std::endl;
	if(rmground->points.size()){
		for(size_t i=0;i<rmground->points.size();i++){
			// grid.data[MeterpointToIndex(rmground->points[i].x, rmground->points[i].y)] = 100;
			
			int px_x = MeterpointToPixel_x(rmground->points[i].y);
			int px_y = MeterpointToPixel_y(rmground->points[i].x);
			if(px_x>=200||px_y>=200){
				std::cout << "px_x = " << px_x << std::endl;
				std::cout << "px_y = " << px_y << std::endl;
			}
			cv::Vec3b *src = image.ptr<cv::Vec3b>(px_y);
			src[px_x][0] = 255;
			// src[0][0] = 255;
			// image.at<cv::Vec3d>(px_y, px_x)[0] = 255;

		}
	}
	//human
	// std::cout<<human->points.size()<<std::endl;
	// if(human->points.size()){
	// 	for(size_t i=0;i<human->points.size();i++){
	// 		grid.data[MeterpointToIndex(human->points[i].x, human->points[i].y)] = 100;
	// 		cv::Vec3b *src = image.ptr<cv::Vec3b>(MeterpointToPixel_y(human->points[i].x));
	// 		src[MeterpointToPixel_x(human->points[i].y)][1]=255;
	// 	}
	// }
		// std::cout<<"333333333"<<std::endl;
		// std::cout<<image.size()<<std::endl;
		// std::cout<<image.channels()<<std::endl;
	image_ros = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
		// std::cout<<"444444444"<<std::endl;
}

int mkdataset::MeterpointToIndex(double x, double y)
{
	int x_ = x/grid.info.resolution + grid.info.width/2.0;
	int y_ = y/grid.info.resolution + grid.info.height/2.0;
	int index = y_*grid.info.width + x_;
	return index;
}

int mkdataset::MeterpointToPixel_x(double x)
{
	int x_ = -x/resolution + grid.info.width/2.0 - 0.5;
	return x_;
}
int mkdataset::MeterpointToPixel_y(double y)
{
	int y_ = -y/resolution + grid.info.height/2.0 - 0.5;
	return y_;
}
void mkdataset::Publication(void)
{
	grid.header.frame_id = pub_frameid;
	grid.header.stamp = pub_stamp;
	image_ros->header.frame_id = pub_frameid;
	image_ros->header.stamp = pub_stamp;
	pub_grid.publish(grid);
	pub_image.publish(image_ros);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mkdataset");
	std::cout << "= mkdataset =" << std::endl;
	
	mkdataset mkdataset;

	ros::spin();
}
