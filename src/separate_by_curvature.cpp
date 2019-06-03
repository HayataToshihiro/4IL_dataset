#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <tf/tf.h>

class SeparateByCurvature{
	private:
		ros::NodeHandle nh;
		
		/*subscribe*/
		ros::Subscriber sub;
		
		/*publish*/
		ros::Publisher pub_smooth;
		ros::Publisher pub_rough;
		
		/*cloud*/
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr pcl_in {new pcl::PointCloud<pcl::PointXYZINormal>};
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr pcl_smooth {new pcl::PointCloud<pcl::PointXYZINormal>};
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr pcl_rough {new pcl::PointCloud<pcl::PointXYZINormal>};

		sensor_msgs::PointCloud2 smooth;
		sensor_msgs::PointCloud2 rough;
		
		/*publish infomations*/
		ros::Time pub_stamp;
		std::string pub_frameid;	
		/*const values*/
		const double threshold_curvature = 3.0e-4;
		const double w = 20.0;
		const double h = 20.0;
	
	public:
		SeparateByCurvature();
		void CallbackPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg);
		void ExtractPCInRange(pcl::PointCloud<pcl::PointXYZI>::Ptr pc);
		void Separate(void);
		int MeterpointToIndex(double x, double y);
		void Publication(void);
};


SeparateByCurvature::SeparateByCurvature()
{
	sub = nh.subscribe("/points", 1, &SeparateByCurvature::CallbackPointCloud, this);
	pub_smooth = nh.advertise<sensor_msgs::PointCloud2>("/points/smooth", 1);
	pub_rough = nh.advertise<sensor_msgs::PointCloud2>("/points/rough", 1);
}


void SeparateByCurvature::CallbackPointCloud(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_pc {new pcl::PointCloud<pcl::PointXYZI>};
	pcl::fromROSMsg(*msg, *tmp_pc);
	
	ExtractPCInRange(tmp_pc);
	
	pcl::copyPointCloud(*tmp_pc, *pcl_in);
	Separate();
	
	pcl::toROSMsg(*pcl_smooth, smooth);
	pcl::toROSMsg(*pcl_rough, rough);
	pub_frameid = msg->header.frame_id;
	pub_stamp = msg->header.stamp;
	Publication();
}

void SeparateByCurvature::ExtractPCInRange(pcl::PointCloud<pcl::PointXYZI>::Ptr pc)
{
	pcl::PassThrough<pcl::PointXYZI> pass;
	pass.setInputCloud(pc);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-w/2.0, w/2.0);
	pass.filter(*pc);
	pass.setInputCloud(pc);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-h/2.0, h/2.0);
	pass.filter(*pc);
}

void SeparateByCurvature::Separate(void)
{
	for(size_t i=0;i<pcl_in->points.size();i++){
		// std::cout << "pcl_in->points[i].curvature = " << pcl_in->points[i].curvature << std::endl;
		// std::cout << "pcl_in->points[i].intensity = " << pcl_in->points[i].intensity << std::endl;
		if(pcl_in->points[i].curvature>threshold_curvature){
			pcl_rough->push_back(pcl_in->points[i]);
		}else{
			pcl_rough->push_back(pcl_in->points[i]);
		}
	}
}


void SeparateByCurvature::Publication(void)
{
	smooth.header.frame_id = pub_frameid;
	rough.header.frame_id = pub_frameid;
	smooth.header.stamp = pub_stamp;
	rough.header.stamp = pub_stamp;
	pub_smooth.publish(smooth);
	pub_rough.publish(rough);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "separate_by_curvature");
	std::cout << "= separate_by_curvature =" << std::endl;
	
	SeparateByCurvature separate_by_curvature;

	ros::spin();
}
