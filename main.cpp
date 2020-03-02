//特征点匹配V_7：实现通过键盘鼠标实现点云匹配的过程,算法改为NDT
//解决方法： 首先通过键盘操作第二个点云的姿态，之后人为选出关心区域，做NDT，得到精确结果
//需要改进：采用两段式方法，先通过匹配点手动选择进行粗匹配，之后通过ICP或者NDT进行细匹配。同时界面设计完善，考虑ubuntu和windows双平台。
// 	    目前准备采用自己的拼接的点云来进行测试，需要自己拼接点云。
#include <iostream>
#include <vector>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
//#include <pcl/registration/icp.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h> 
#include <pcl/filters/approximate_voxel_grid.h>
#define  MOVE_STEP 0.1
using std::cout;
using std::endl;
typedef pcl::PointXYZRGB PointT;
pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
pcl::PointCloud<PointT>::Ptr cloud1(new pcl::PointCloud<PointT>());
//pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>()); //!!
pcl::PointCloud<PointT>::Ptr cloud_mix(new pcl::PointCloud<PointT>());
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("viewer1"));//!!
pcl::PointCloud<PointT>::Ptr clicked_points_3d(new pcl::PointCloud<PointT>);//框选点云
pcl::PointCloud<PointT>::Ptr clicked_points_3d_1(new pcl::PointCloud<PointT>);//框选点云
int num = 0;
double move_x=0;
double move_y=0;
double angle_y=0;
double angle_z=0;
bool change_flag=0;
int txt_gray_lvl=1.0;
bool sel_mod_flag=0;

Eigen::Matrix4d final_tranfirm_matrix=Eigen::Matrix4d::Identity();

//打印矩阵
void
print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  cout<<"Rotation matrix :\n"<<endl;
  cout<<"    " <<matrix (0, 0)<<" "<< matrix (0, 1)<<" "<<matrix (0, 2)<<endl;
  cout<<"R = " <<matrix (1, 0)<<" "<< matrix (1, 1)<<" "<<matrix (1, 2)<<endl;
  cout<<"    " <<matrix (2, 0)<<" "<< matrix (2, 1)<<" "<<matrix (2, 2)<<endl;
  cout<<"\nTranslation vector :\n"<<endl;
  cout<<"t = "<< matrix (0, 3)<<" "<<matrix (1, 3)<<" "<< matrix (2, 3)<<endl<<endl;
}
//选取区域回调函数
void ppp_callback(const pcl::visualization::AreaPickingEvent& event, void *args)
{
  struct callback_args * data =(struct callback_args *) args;
  std::vector<int> indices;
  if(event.getPointsIndices(indices)==-1)
  {
    return;
  }
  //将选取的目标点云和源点云分开放入两个点云以备后续匹配
  for(size_t i=0;i<indices.size();i++)
  {
    if(cloud_mix->points[indices[i]].r==255&&cloud_mix->points[indices[i]].g==255&&cloud_mix->points[indices[i]].b==255)
    {
      clicked_points_3d->points.push_back(cloud_mix->at(indices[i]));
    } 
    if(cloud_mix->points[indices[i]].r==255&&cloud_mix->points[indices[i]].g==0&&cloud_mix->points[indices[i]].b==0)
    {
       clicked_points_3d_1->points.push_back(cloud_mix->at(indices[i]));
    }
     
  }
  cout<<"select "<<clicked_points_3d->size()<<" target point"<<endl;
  cout<<"select "<<clicked_points_3d_1->size()<<" source point"<<endl;
  if(clicked_points_3d->size()==0||clicked_points_3d_1->size()==0)
  {
    cout<<"fail to select point cloud,please try again"<<endl;
    return;
  }
  pcl::visualization::PointCloudColorHandlerCustom<PointT> green (clicked_points_3d,0,255,0);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> blue (clicked_points_3d_1,0,0,255);
  
  //对输入矩阵降采样
  pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>);
  pcl::ApproximateVoxelGrid<PointT> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize(0.2,0.2,0.2);
  approximate_voxel_filter.setInputCloud(clicked_points_3d_1);
  approximate_voxel_filter.filter(*filtered_cloud);
  cout<<"Filtered cloud contains "<<filtered_cloud->size()<<
	" datd points from selected cloud"<<endl;
  //对选取的两个点云进行匹配
//   pcl::IterativeClosestPoint<PointT,PointT> local_icp;
//   local_icp.setInputTarget(clicked_points_3d);
//   local_icp.setInputSource(clicked_points_3d_1);
//   local_icp.setMaxCorrespondenceDistance(1);
//   local_icp.setMaximumIterations(100);
//   local_icp.setTransformationEpsilon(1e-8);
//   local_icp.align(*clicked_points_3d_1);

//对选取的两个点云进行匹配 
  //配置点云
  pcl::NormalDistributionsTransform<PointT,PointT> ndt;
  ndt.setTransformationEpsilon(0.1);
  ndt.setStepSize(0.5);
  ndt.setResolution(5.0);
  ndt.setMaximumIterations(10);
  ndt.setInputSource(filtered_cloud);
  ndt.setInputTarget(clicked_points_3d);
  //设置初始值
  Eigen::AngleAxisf init_rotation(0,Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(0,0,0);
  Eigen::Matrix4f init_guess=(init_translation*init_rotation).matrix();
  cout<<"haha2"<<endl;
  //开始配准
  ndt.align(*filtered_cloud,init_guess);
  cout<<"haha"<<endl;
  if(ndt.hasConverged())
  {
    cout<<"haha1"<<endl;
    cout<<"NDT has converged,score is "<< ndt.getFitnessScore()<<endl<<endl;
    Eigen::Matrix4d transform_matrix;
    transform_matrix=ndt.getFinalTransformation().cast<double>();
    pcl::transformPointCloud(*cloud1,*cloud1,transform_matrix);
    final_tranfirm_matrix=transform_matrix*final_tranfirm_matrix;
    cout<<"Final result is:"<<endl;
    print4x4Matrix(final_tranfirm_matrix);
    //将选取的点云变换颜色以方便观察
    for(size_t i=0;i<indices.size();i++)
    {
      if(indices[i]<cloud->size())
      {
	cloud->points[indices[i]].r=0;
	cloud->points[indices[i]].b=0;
      } 	
      else
      {
        cloud1->points[indices[i]-cloud->size()].r=0;
	cloud1->points[indices[i]-cloud->size()].b=255;
        cloud1->points[indices[i]-cloud->size()].g=0;
      }
    }
    *cloud_mix=*cloud+*cloud1;
    viewer->updatePointCloud(cloud_mix,"bunny_source");
   
  }	
}
//键盘的回调函数
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,void* nothing )
{
  static bool space=0;
  if(event.keyDown())
  {
    Eigen::Affine3f key_tranform=Eigen::Affine3f::Identity();
    //选择点云模式切换
     if(event.getKeySym()=="x"||event.getKeySym()=="X")
    {
      sel_mod_flag=!sel_mod_flag;
      if(sel_mod_flag)
	viewer->updateText("Now you can select points using mouse",10,100, 18, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl,"select");
      else
	viewer->updateText("Press 'x' to turn on selection",10,100, 18, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl,"select");     
    }
    //姿态和位置调整功能切换
    if(event.getKeySym()=="space")
    {
      space=1-space;
      cout<<"space:"<<space<<endl;
      move_x=0;
      move_y=0;
      angle_y=0;
      angle_z=0;
    }
    //控制键功能
    if(event.getKeySym()=="Up")
    {
      move_x=space>0?MOVE_STEP:0;
      move_y=0;
      angle_y=space>0?0:-M_PI*0.01;
      angle_z=0;
      change_flag=1;
      
    } 
    if(event.getKeySym()=="Down")
    {
      move_x=space>0?-MOVE_STEP:0;
      move_y=0;
      angle_y=space>0?0:M_PI*0.01;
      angle_z=0;
      change_flag=1;
    }
    if(event.getKeySym()=="Left")
    {
      move_y=space>0?MOVE_STEP:0;
      move_x=0;
      angle_y=0;
      angle_z=space>0?0:M_PI*0.05;
      change_flag=1;
    }      
    if(event.getKeySym()=="Right")
    {
      move_y=space>0?-MOVE_STEP:0;
      move_x=0; 
      angle_y=0;
      angle_z=space>0?0:-M_PI*0.05;
      change_flag=1;
    }
    if(change_flag)
    {
      key_tranform.rotate(Eigen::AngleAxisf(angle_y,Eigen::Vector3f::UnitY()));
      key_tranform.rotate(Eigen::AngleAxisf(angle_z,Eigen::Vector3f::UnitZ()));
      key_tranform.translation()<<move_x,move_y,0;
      pcl::transformPointCloud(*cloud1,*cloud1,key_tranform);
      final_tranfirm_matrix=key_tranform.matrix().cast<double>()*final_tranfirm_matrix;
      *cloud_mix=*cloud+*cloud1;
      viewer->updatePointCloud(cloud_mix,"bunny_source");
      change_flag=0;
    }
    
  }
  
  //viewer->addPointCloud(cloud1,cloud1_h, "bunny_source1");
}
  
//执行程序
void run(int argc,char** argv)
{
  std::cout << "You can press 's' to turn on select mode, then press 'Q' to exit" << std::endl;
  std::vector<int> pcdfile_indices=pcl::console::parse_file_extension_argument(argc,argv,"pcd");
  if(pcdfile_indices.size()!=2)
  {
    cout<<"ERROR:Please input the target and source point cloud files [.pcd]"<<endl;
    return ;
  }
  std::string filename(argv[pcdfile_indices[0]]);
  if (pcl::io::loadPCDFile(filename, *cloud))
  {
    std::cerr << "ERROR: Cannot open file " << filename << "! Aborting..." << std::endl;
    return;
  }
  std::cout <<"Target point cloud has "<< cloud->points.size()<<" points."<< std::endl;
  std::string filename1(argv[pcdfile_indices[1]]);
  if (pcl::io::loadPCDFile(filename1, *cloud1))
  {
    std::cerr << "ERROR: Cannot open file " << filename << "! Aborting..." << std::endl;
    return;
  }
  std::cout <<"Source point cloud has "<<cloud1->points.size() <<" points."<< std::endl;
  for(int i=0;i<cloud->size();i++)
  {
    cloud->points[i].r=255;
    cloud->points[i].g=255;
    cloud->points[i].b=255;
  }
  for(int i=0;i<cloud1->size();i++)
  {
    cloud1->points[i].r=255;
    cloud1->points[i].g=0;
    cloud1->points[i].b=0;
  }
//  *cloud2=*cloud1;  //!!
  *cloud_mix=*cloud+*cloud1;
  // Display pointcloud:
  //pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud1_h (cloud1,20,230,20);
  viewer->addPointCloud(cloud_mix, "bunny_source");
  viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
//  viewer->addCoordinateSystem(5.0,"aix");
  viewer->setCameraPosition(-50,0,0,0,0,1);
  viewer->addText("White: Target point cloud\nRed: Source point cloud\nGreen: Selected target point cloud\nBlue: Selected source point cloud",
		10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl,"text1");
  viewer->addText("Press 'x' to turn on selection",10,100, 18, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl,"select");
  viewer->registerAreaPickingCallback(ppp_callback, (void*)&cloud_mix);
  viewer->registerKeyboardCallback(&keyboardEventOccurred, (void*)NULL);
  while (!viewer->wasStopped())  //!!
  {
    viewer->spinOnce(100);
 //   viewer1->spinOnce(100);//!!
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
}
int main(int argc,char** argv)
{
  run(argc,argv);
}
