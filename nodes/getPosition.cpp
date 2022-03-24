#include <find_edge/brick.h> // tipo messaggio per motion
#include <find_edge/cord.h> // tipo messaggio per cord ricevute da yolo
#include "find_edge/FeatureCloud.h" // classe featureCloude per salvare cloud Brick
#include "find_edge/TemplateAlignment.h" // classe per fare confronto
#include "find_edge/pcdFileStore.h"  // classe per salvare .pcd  vector<vector<FeaureCloud>>
#include "find_edge/BrickDimension.h" // HardCoded dimension dei brick
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>  // tipo messaggio per pointCloud2
#include <pcl_conversions/pcl_conversions.h>  // ridondante contenuta anche in FeatureCloud e TemplateCloud
#include <boost/bind.hpp> // per passare una funzione con due argomenti ad una chiamata che prende solo funzione
                          // ros::subscriber e publischer

#define LOADFILE "listFileClassPcd.txt" //.txt per caricare .pcd delle classi
#define COMPONENTS 3 // numero componeti di un vettore
#define BOUNDBOX 4 // valori del boundbox
#define NAME 0 // posizione della classe nel messaggio da yolo cord.msg
#define X_MIN 1 // posizione x_min boundbox da messaggio yolo
#define Y_MIN 2 // vedi sopra
#define X_MAX 3 // vedi sopra
#define Y_MAX 4 // vedi sopra


//struct per definizione bricks
typedef struct{
    int name; //classe del blocco
    FeatureCloud cloud; // nuvola che rappresenta il blocco
    float boundBox[BOUNDBOX]; // boundbox del brick
    float initPosition[COMPONENTS]; // posizione iniziale da computare
    float finalPosition[COMPONENTS]; // posizione finale da computare half-HardCoded
    float initRotation[COMPONENTS]; // rotazione iniziale da computare
    float finalRotation[COMPONENTS]; // rotazione finale HardCoded --> ogni blocco ha un fronte,retro, sopra, sotto
    float x; // dimensione x del blocco
    float y; // dimensione y del blocco
    float z; // dimensione z del blocco
}detectedBrick;

//int ctr_detectedBrick=1;
//int itr_detectedBrick=0;
//detectedBrick *detected_Bricks=new detectedBrick[ctr_detectedBrick]();

//const float CAMERA_POSITION[COMPONENTS]=[]; // posizione della camera su gazebo
                                              // utilizzata per computare la distanza dei blocchi dalla
                                              // posizione del braccio che e' [x=0,y=0,z=0]

//const float CAMERA_ANGLE=; // rotazione della camera rispetto al piano, serve per computare distanza come per coordinate

StoreClass STORE_PCD_TEMPLATES(LOADFILE); //classe alla cui creazione viene passato
                                                //il file .txt conntenete a sua volta i nomi
                                                //.txt contenenti i file .pcd per ogni classe
                                                // vector<vector<FeatureCloud>> e' quello
                                                //che la classe contiene 


/*****SE LE CALLBACK SONO CON SINGOLO PARAMETRO DEVO DICHIARARE DELLE VARIABILI GLOBALI****/
                /*****MI FAREBBE PARECCHIO SCHIFO*****/
//void cordCallback(const find_edge::cord::ConstPtr&); // callback con singolo argomento
void cord_Callback(const find_edge::cord::ConstPtr&, detectedBrick* ); // callBack per la gestione e ricezione dei messaggi
                                                      // da parte del nodo che compie riconoscimento,
                                                      // ricevo messaggio contenete un singolo array di float[5] 
                                                      //contenete in pos 0 nome [classe 0-10], [1-4] boundbox
                                                      //vedi define NAME X_MIN......



//void depthCallback(const sensor_msgs::PointCloud2::ConstPtr& ); // singolo argomento
void depth_Callback(const sensor_msgs::PointCloud2::ConstPtr&, detectedBrick* ); // callBack per estrarre i cloud corrispondenti ai blocchi
                                                                                // esegue anche chiamata a computePosition
                                                                                // per copiare i dati sul messaggio da publicare



void computePosition(detectedBrick*); // funzione che computa rotazione assoluta, e posizione fianle per il blocco passato
find_edge::brick getMSG(const detectedBrick);

int main(int argc, char* argv[]){
  
  ros::init(argc,argv,"detect");
  ros::NodeHandle node;

  FeatureCloud prova;   //test
  TemplateAlignment prova2;//test

  int ctr_detectedBrick=1; // contatore per numero messaggi ricevuti da yolo
  int itr_detectedBrick=0;// iteratore per gestire la creazione dinamica di struct detectedBrick 
                          // viene aumentato alla ricezione di un nuovo messaggio da parte di yolo
                          // se uso subscriber con 1 argomento diventa globale

  detectedBrick *detected_Bricks=new detectedBrick[ctr_detectedBrick](); // array dinamico per la creazione di
                                                            // struct per poi poter scrivere messaggio
                                                            // da inviare a motion plan
                                                            // se uso subscriber con 1 argomento diventa globale
  
  //subscriber callback con due argomenti
  //ros::Subscriber coordSub=node.subscribe<const find_edge::cord >("/kinect/coordinates", 11, boost::bind(cord_Callback, _1, detected_Bricks[itr_detectedBrick]));
  //ros::Subscriber coordSub=node.subscribe("/kinect/coordinates",11,cordCallback); // singolo argomento

  while(ros::ok()){
    ros::spinOnce();
    ctr_detectedBrick+=1;
    itr_detectedBrick+=1;
  }
  //coordSub.shutdown();// chiudo subscriber al topic di yolo succede dopo che ho ricevuto una volta tutti i messaggi
                      // ricevo tutti i messaggi solo una volta perche' una volta inviati i messaggi da yolo il nodo
                      // chude il topic (parte di gio) --> esco dal loop
  itr_detectedBrick=0; // pongo iteratore a 0 per ricominciare le chiamare
  
  ros::Subscriber depthSub=node.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 2, boost::bind(depth_Callback, _1, &detected_Bricks[itr_detectedBrick])); 
  //ros::Subscriber depthSub=node.subscribe("/camera/depth/points", 1, depth_Callback); // mi collego al topic del kinect 
                                                                                      // e passo alla callback il brick su 
                                                                                      // cui trovare le info di posizione
                                                                                      
ros::Publisher posPub = node.advertise<find_edge::cord>("motion/plan", 11); // topic dove publico ad ogni ciclo le info del cubo appena computato
 /****MUTEX*****/
  while(itr_detectedBrick < ctr_detectedBrick){  
    ros::spinOnce(); // faccio un giro leggendo dal topic che pubblica pointCloud2 e faccio tutte le cose del caso
    find_edge::brick msg=getMSG(detected_Bricks[itr_detectedBrick]); // copio i dati dall'ultimo brick elaborato al tipo messaggio
    posPub.publish(msg); // pubblico il messaggio
    itr_detectedBrick+=1;
  }
  depthSub.shutdown();
  //posPub.shutdown(); // prima di chudere topic devo aspettare che tutti i messaggi siano stati letti
    delete detected_Bricks;
    return 0;
}

void cord_Callback(const find_edge::cord::ConstPtr& msg, detectedBrick* Brick){
                        // classe del blocco da yolo
    //detected_Bricks[itr-1].name=(int)msg->coordinate[NAME];// senza argomento detected_Bricks globale
    Brick->name=(int)msg->coordinate[NAME];// con argomento passo singolo brick

        // punti del boundbox trovati da yolo
    //detected_Bricks[itr-1].boundBox[X_MIN-1]=msg->coordinate[X_MIN]; 
    Brick->boundBox[X_MIN-1]=msg->coordinate[X_MIN]; 
    //detected_Bricks[itr-1].boundBox[Y_MIN-1]=msg->coordinate[Y_MIN];
    Brick->boundBox[Y_MIN-1]=msg->coordinate[Y_MIN];

    //detected_Bricks[itr-1].boundBox[X_MAX-1]=msg->coordinate[X_MAX];
    Brick->boundBox[X_MAX-1]=msg->coordinate[X_MAX];
    //detected_Bricks[itr-1].boundBox[Y_MAX-1]=msg->coordinate[Y_MAX];
    Brick->boundBox[Y_MAX-1]=msg->coordinate[Y_MAX];

        // dimensioni del blocco ottenute da BRICKS -> HardCoded vedi BrickDimension.h
    //detected_Bricks[itr-1].x=BRICKS[detected_Bricks[itr-1].name].x;
    Brick->x=BRICKS[Brick->name].x;
    //detected_Bricks[itr-1].y=BRICKS[detected_Bricks[itr-1].name].y;
    Brick->y=BRICKS[Brick->name].y;
    //detected_Bricks[itr-1].z=BRICKS[detected_Bricks[itr-1].name].z;
    Brick->z=BRICKS[Brick->name].z;
  
}



void depth_Callback(const sensor_msgs::PointCloud2::ConstPtr& msg, detectedBrick* Brick){

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg (*msg, cloud);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudBrick;
        /****COPIA PCL2 DEL BRICK A FEATURECLOUD DI DETECTEDBRICK*****/ 

    //filtro il cloud che ho appena estratto tramite il boundBox per avere meno punti da elaborare aka piu' veloce    
    const float voxel_grid_size = 0.005f;
    pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
    vox_grid.setInputCloud(cloudBrick);
    vox_grid.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(
          new pcl::PointCloud<pcl::PointXYZ>);
    vox_grid.filter(*tempCloud);
    cloudBrick = tempCloud;
    
    //copio il cloud filtrato sul cloud del brick
    Brick->cloud.setInputCloud(cloudBrick);
    computePosition(Brick); //trovo rotazione e posizione del Brick e salvo il tutto nella struct corrispondente
}


void computePosition(detectedBrick* Brick){

  //Eigen::Matrix<4,1> position;
  //pcl::compute3dCentroid({argomento},&position); // per trovare il centroide del cloud riferito al brick
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
  template_align.setTargetCloud(Brick->cloud); //carico cloud brick

  TemplateAlignment::Result best_alignment;
  int best_index = template_align.findBestAlignment(best_alignment); // computo e mi torna indice del miglior risultato
  const FeatureCloud &best_template = STORE_PCD_TEMPLATES.getCloud(Brick->name,best_index); // non serve
  
  {

  Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3, 3>(0, 0);
  Eigen::Vector3f translation = best_alignment.final_transformation.block<3, 1>(0, 3);
  // risultati di traslazione e rotazione si riferiscono a .pcd best index
    } 
  
}


// semplice copia dei valori per tornare il messaggio del brick passato
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
