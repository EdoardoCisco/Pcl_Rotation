#ifndef __PCDFILESTORE_H__
#define __PCDFILESTORE_H__

#include <find_edge/FeatureCloud.h>
#include <string.h>
#include <vector>


class StoreClass{
    private:
        const vector<vector<FeatureCloud>> pcdLoadedSample;
        pcl::PointCloud<pcl::PointXYZ>::Ptr _ptr;
        vector<vector<FeatureCloud>> loadPcdFile(string)const;
        // devo inserire dei valori che mi indicano le rotazini dei blocchi dei .pcd
        // lo implemento leggendo dal file, tipo il nome e' la sua rotazione
    public:
        StoreClass(string); // passo il .txt definito in getPosition
        ~StoreClass();
        int getDim(int); // ritorna diensione del vettore di una classe
        FeatureCloud getCloud(int,int) const; // ritorna .pcd
        //vector<FeatureCloud> getVector(int); // ritorna vettore dei .pcd di una classe
        // metodo che mi da la rotazione corrispondente al getCloud()
};
// .pcd sono file contenenti il cloud del brick a rotazini prestabilite

#endif
