//ros and in-code package includes
#include <ros/ros.h>
#include <find_edge/brick.h> 
#include <detect/cord.h> 
#include "find_edge/FeatureCloud.h" 
#include "find_edge/TemplateAlignment.h"
#include "find_edge/pcdFileStore.h"
#include "find_edge/BrickDimension.h"
#include <sensor_msgs/PointCloud2.h> 
//in-code includes
#include <boost/bind.hpp> 
#include <thread>
#include <mutex>
#include <atomic>
#include <Eigen/Dense>
//pcl includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/point_cloud.h>


#define LOADFILE "BRICKS/listFileClassPcd.txt"
#define RADIUS 0.08
#define COMPONENTS 6 
#define BOUNDBOX 6 
#define NAME 0 
#define X_MIN 1 
#define Y_MIN 2 
#define X_MAX 3
#define Y_MAX 4
#define CEN_X 5
#define CEN_Y 6
#define PI 3.141592653589

//pcl::io::savePCDFile

typedef struct{
    int name;
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ( new pcl::PointCloud<pcl::PointXYZ> ());
    int boundBox[BOUNDBOX]; 
    float initPosition[COMPONENTS]; 
    float initRotation[COMPONENTS]; 
    float x; 
    float y;
    float z;
}detectedBrick;


//const float CAMERA_POSITION[COMPONENTS]=[]; 
//const float CAMERA_ANGLE=; 
StoreClass STORE_PCD_TEMPLATES(LOADFILE);
atomic<bool> semaforo (false);
mutex mtx;
mutex mtx_brick_map;
mutex mtx_brick_struct;

void cord_callback(const detect::cord::ConstPtr&, detectedBrick*); 
void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr&, pcl::PointCloud<pcl::PointXYZ>::Ptr); 
void computePosition( pcl::PointCloud<pcl::PointXYZ>::ConstPtr, detectedBrick*, const ros::Publisher*); 
//void computePosition( pcl::PointCloud<pcl::PointXYZ>::ConstPtr, vector<detectedBrick>*, const ros::Publisher*); 
//find_edge::brick getMSG(const detectedBrick);


int main(int argc, char* argv[]){
  
  ros::init(argc,argv,"detect");
  ros::NodeHandle node;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr brick_map (new pcl::PointCloud<pcl::PointXYZ>);

  ros::Publisher posPub = node.advertise<find_edge::brick>("motion/plan", 11);
  ros::Subscriber cloudSub=node.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 2, boost::bind(cloud_callback, _1, *(&brick_map)));
    
  ros::Rate loop(10);
  while(brick_map->empty()){
    ros::spinOnce();
    cout<<"loop\n";
    loop.sleep();
  }
  cloudSub.shutdown();

  atomic<int> ctr_detectedBrick (1); 
  atomic<int> itr_detectedBrick (0); 

  detectedBrick *detected_Bricks=(new detectedBrick[itr_detectedBrick]());
  //vector<detectedBrick> detected_Bricks;
  //detected_Bricks.clear();

  vector<thread> thr_compute;
  
  thr_compute.clear();

  ros::Subscriber coordSub=node.subscribe<detect::cord>("/kinects/coordinate", 11, boost::bind(cord_callback, _1, &detected_Bricks[itr_detectedBrick]));

  while(ros::ok()){
    ros::spinOnce();
    
    if(semaforo){
      cout<<"itr "<<itr_detectedBrick<<endl;
      semaforo=false;
      mtx_brick_struct.lock();
      thr_compute.push_back(thread(boost::bind(computePosition,brick_map,&detected_Bricks[itr_detectedBrick],&posPub)));
      mtx_brick_struct.unlock();
      ctr_detectedBrick+=1;
      itr_detectedBrick+=1;
    }
  }

  coordSub.shutdown();

  for(thread & th : thr_compute){
      th.join();
  }

  delete detected_Bricks;
  return 0;
}

void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg, pcl::PointCloud<pcl::PointXYZ>::Ptr map){
    mtx_brick_map.lock();
    pcl::fromROSMsg (*msg, *map);
    mtx_brick_map.unlock();
}

void computePosition(pcl::PointCloud<pcl::PointXYZ>::ConstPtr map, detectedBrick* Brick, const ros::Publisher* pub){
  

  mtx.lock();
  cout<<"my name is "<<Brick->name<<" and im at ";
  for(int i=0;i<COMPONENTS;++i){
    cout<<Brick->boundBox[i]<<" ";
  }
  cout<<endl;
  mtx.unlock();

  FeatureCloud feature_Brick;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr brick (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::search::OrganizedNeighbor<pcl::PointXYZ>::Ptr tree (new pcl::search::OrganizedNeighbor<pcl::PointXYZ>());


  mtx_brick_map.lock();
  cout<<"copio\n";
  pcl::copyPointCloud(*map,*cloud_map);
  mtx_brick_map.unlock();
  
 
  
  pcl::PointXYZ point=cloud_map->at(Brick->boundBox[CEN_X-1],Brick->boundBox[CEN_Y-1]);

  mtx.lock();
  cout<<Brick->boundBox[CEN_X-1]<<" "<<Brick->boundBox[CEN_Y-1]<<endl;
  cout<<"point at "<<point<<endl;
  
  cout<<"number of point of "<<Brick->name<<" in cloudMap "<<cloud_map->size()<<endl;

  mtx.unlock();
   

  pcl::Indices indices;
  vector<float> k_distances;
  
  tree->setInputCloud(cloud_map);
  tree->radiusSearch(point,RADIUS,indices,k_distances);

  if(!cloud_map->empty()){

  for(const auto& idx : indices){
    brick->push_back((*cloud_map)[idx]);
  }
  mtx.lock();
  cout<<"number of point in pointcloud "<<brick->size()<<endl;
  mtx.unlock();
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01); // 1cm

  int nr_points = (int) brick->size ();
  while (brick->size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (brick);
    seg.segment (*inliers, *coefficients);
    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (brick);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *brick= *cloud_f;
   
   }
   *cloud_map=*brick;


 
    
     mtx.lock();
  cout<<"number of point in pointcloud2 "<<brick->size()<<endl;
  Eigen::Vector4f centroid_3d;
  int val=compute3DCentroid(*cloud_map,centroid_3d);
  cout<<"centroid "<<centroid_3d<<endl;
  cout<<"val "<<val<<endl;
  mtx.unlock();
  feature_Brick.setInputCloud(cloud_map);
  
  // per trovare il centroide del cloud riferito al brick
  //Brick->initPosition[0]=CAMERA_POSITION[0]-position[0]; // per ora sbagliato ma si rifrisce a x
  //Brick->initPosition[1]=CAMERA_POSITION[1]-position[1]; // per ora sbagliato ma si riferisce a y
  //Brick->initPosition[2]=Brick->z/2; // setto centroide del blocco ad altezza del blocco/2
  
  
  int vect_DIM = STORE_PCD_TEMPLATES.getDim(Brick->name);

  TemplateAlignment template_align; // creo templatecloud per poi usare la funzione interna che mi torna
                                    // le rotazioni

  for (std::size_t i = 0; i < vect_DIM; ++i) {
    FeatureCloud fc = STORE_PCD_TEMPLATES.getCloud(Brick->name,i);
    template_align.addTemplateCloud(fc);//carico vettore di .pcd
  }
  template_align.setTargetCloud(feature_Brick); //carico cloud brick

  TemplateAlignment::Result best_alignment;
  int best_index = template_align.findBestAlignment(best_alignment); // computo e mi torna indice del miglior risultato
  const FeatureCloud &best_template = STORE_PCD_TEMPLATES.getCloud(Brick->name,best_index); // non serve
  
  

  Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3, 3>(0, 0);
  Eigen::Vector3f translation = best_alignment.final_transformation.block<3, 1>(0, 3);
  // risultati di traslazione e rotazione si riferiscono a .pcd best index
  mtx.lock();
  cout<<"translation "<<translation<<endl<<endl;
  cout<<"rotation "<<rotation<<endl;
   // find_edge::brick msg=getMSG(*Brick);
    //pub.publish(msg);
  mtx.lock();
  }else {
    mtx.lock();
  cout<<"empty map "<<Brick->name<<endl;
  mtx.lock();
  }

}

void cord_callback(const detect::cord::ConstPtr& msg, detectedBrick* Brick){

  mtx_brick_struct.lock();

    Brick->name=(int)msg->coordinate[NAME];
    cout<<Brick->name<<endl;

    Brick->boundBox[X_MIN-1]=(int)msg->coordinate[X_MIN]; 
    Brick->boundBox[Y_MIN-1]=(int)msg->coordinate[Y_MIN];
    cout<<Brick->boundBox[X_MIN-1]<<" "<<Brick->boundBox[Y_MIN-1];


    Brick->boundBox[X_MAX-1]=(int)msg->coordinate[X_MAX];
    Brick->boundBox[Y_MAX-1]=(int)msg->coordinate[Y_MAX];
    cout<<" "<<Brick->boundBox[X_MAX-1]<<" "<<Brick->boundBox[Y_MAX-1]<<" ";

    Brick->boundBox[CEN_X-1]=(int)msg->coordinate[CEN_X]; 
    Brick->boundBox[CEN_Y-1]=(int)msg->coordinate[CEN_Y];
    cout<< Brick->boundBox[CEN_X-1]<<" "<<Brick->boundBox[CEN_Y-1]<<" ";

    Brick->x=BRICKS[Brick->name].x;
    Brick->y=BRICKS[Brick->name].y;
    Brick->z=BRICKS[Brick->name].z;
  mtx_brick_struct.unlock();  
    semaforo=true;
  
}
/*
find_edge::brick getMSG(const detectedBrick BRICK){
  
  find_edge::brick msg;

  for (int i=0;i<COMPONENTS;++i){
  msg.initPosition[i]=BRICK.initPosition[i]; 
  msg.finalPosition[i]=BRICK.finalPosition[i];
  msg.initRotation[i]=BRICK.initRotation[i];
  msg.finalRotation[i]=BRICK.finalRotation[i];
  }
  msg.x=BRICK.x;
  msg.y=BRICK.y;
  msg.z=BRICK.z;

  return msg;
}
*/


/*

Class A
  ciao *

Class B:extend A
  int1
  int2
  int3


A* prova(ciao);

 prova (new B[]);
 */