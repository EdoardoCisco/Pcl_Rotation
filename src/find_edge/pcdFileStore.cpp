#include <find_edge/pcdFileStore.h>
#include <find_edge/FeatureCloud.h>
#include <vector>
#include <fstream>
#include <string>

using namespace std;

StoreClass::StoreClass(string _loadFile):pcdLoadedSample(loadPcdFile(_loadFile)){}

StoreClass::~StoreClass() {}


vector<vector<FeatureCloud>> StoreClass::loadPcdFile(string _loadFile)const{

    vector<vector<FeatureCloud>> pcdLoadedSample;
    pcdLoadedSample.clear();
    ifstream fileList(_loadFile);
    string fileName;
    string pcdFileName;
    int classNum=0;

    while (fileList.good()) { 
        getline(fileList, fileName);
        if (fileName.empty() || fileName.at(0) == '#') continue;

        ifstream pcdClassFile(fileName);
        vector<FeatureCloud> classTemplate;
        FeatureCloud template_cloud;

        while(pcdClassFile.good()){
            getline(pcdClassFile, pcdFileName);
            if (pcdFileName.empty() || pcdFileName.at(0) == '#') continue;
            
            template_cloud.loadInputCloud(pcdFileName);
            classTemplate.push_back(template_cloud);
        }
        pcdClassFile.close();
        pcdLoadedSample.push_back(classTemplate);
        classTemplate.clear();
    }
    fileList.close();

    return pcdLoadedSample;
}

int StoreClass::getDim(int vect){
    return pcdLoadedSample[vect].size();
}

FeatureCloud StoreClass::getCloud(int vector,int file) const {
    return pcdLoadedSample[vector][file];
}

/*
 vector<FeatureCloud> StoreClass::getVector(int vector)
    vector<FeatureCloud> vect;
    //ancora da implementare meglio usare ptr

    return vect;
 }
*/