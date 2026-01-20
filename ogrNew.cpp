/*
 * ogrNew.cpp
 * Function: to extract the road networks from .shp
 *
 *  Created on: 16 June 2024
 *      Author: Xinjie ZHOU
 */
#include <ogrsf_frmts.h>
#include <gdal.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <map>
#include <unordered_map>
#include <set>
#include <unordered_set>
#include <stack>
#include <string>
#include <stdlib.h>
#include <vector>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

using namespace std;

vector<string> split(const string &s, const string &seperator);
void NodePointProcess(char * sourceFile, char * layerName, string outputFile);
void EdgePolylineProcess(char * sourceFile, char * layerName, string outputFile);
void GetRoadNetwork(string edgeFile, string nodeFile, string sourcePath);
void GetRoadNetworkAggregation(string sourcePath, string targetPath, vector<string>& datasets, string tragetName);
void RoadNetworkPreprocess(string graph_path);
template <class T>
pair<int, unsigned long long int> DFS_CC(T & Edges, unordered_set<int> & set_A, set<int> & set_LCC, int nodenum);
double ComputeDiameter(vector<pair<double,double>> & points);
double EuclideanDis(pair<double,double> s, pair<double,double> t);

bool ifNew=true;

int main(int argc, char** argv){
    if( argc < 4){//
        printf("usage:\n<arg1> source path, e.g /data/TrajectoryData/map/\n");
        printf("<arg2> target path, e.g. /data/xzhouby/datasets/map/Guangdong/\n");
        printf("<arg3> name of dataset, e.g. Guangdong\n");
        printf("<arg4> aggregate road networks? (optional), 0: No, 1: Yes. Default: 0\n");
        printf("<arg5> name of aggregated datasets (optional), eg. guangdong1 guangdong2 guangzhou\n");
        exit(0);
    }
    bool ifAggregate=false;
    string sourcePath;
    string targetPath;
    vector<string> datasets;
    string dataSet;
    if(argc > 1) {
        cout << "argc: " << argc << endl;
        cout << "argv[1] (source path): " << argv[1] << endl;
        sourcePath = argv[1];

        cout << "argv[2] (target path): " << argv[2] << endl;
        targetPath = argv[2];

        cout << "argv[3] (Dataset): " << argv[3] << endl;//dataset
        dataSet=argv[3];

        if(argc>4){
            cout << "argv[4] (if aggregate road networks): " << argv[4] << endl;
            ifAggregate = stoi(argv[4]);
            if(argc>5){
                for(int i=5;i<argc;++i){
                    cout << "argv["<<i<<"] (Dataset): " << argv[i] << endl;//dataset
                    datasets.emplace_back(argv[i]);
                }
            }
        }

    }

    GDALAllRegister();  //register all the format drivers
    cout << "GDAL All Registed!" << endl;

    if(!ifAggregate){//only one dataset
        string dataset="fujian";
        dataset=dataSet;
        string inPath=sourcePath+dataset+"/road/";
        cout<<"Extract one road network."<<endl;
        /// Step 1: get the original vertex information
        cout<<"Step 1: get the original vertex information"<<endl;
        string layer1="N"+dataset+"_point";
        string source1=inPath+layer1+".shp";
        char* char_array1 = new char[source1.length() + 1];
        strcpy(char_array1, source1.c_str());
        char* char_layer1 = new char[layer1.length() + 1];
        strcpy(char_layer1, layer1.c_str());
        NodePointProcess(char_array1,char_layer1,inPath+dataset+".node");
        delete[] char_array1; delete[] char_layer1;

        /// Step 2: get the original edge information
        cout<<"Step 2: get the original edge information"<<endl;
        string layer2="R"+dataset+"_polyline";
        string source2=inPath+layer2+".shp";
        char* char_array2 = new char[source2.length() + 1];
        strcpy(char_array2, source2.c_str());
        char* char_layer2 = new char[layer2.length() + 1];
        strcpy(char_layer2, layer2.c_str());
        EdgePolylineProcess(char_array2,char_layer2,inPath+dataset+".edge");
        delete[] char_array2; delete[] char_layer2;

        /// Step 3: get the original road network
        cout<<"Step 3: get the original road network"<<endl;
        //For single data source
        GetRoadNetwork(inPath+dataset+".edge",inPath+dataset+".node",targetPath+dataset);
    }
    else{//aggregation of multiple datasets
        string dataset="fujian";
        string inPath;
        cout<<"Aggregate multiple road networks"<<endl;
        for(int i=0;i<datasets.size();++i){
            dataset=datasets[i];
            inPath=sourcePath+dataset+"/road/";

            /// Step 1: get the original vertex information
            cout<<"Step 1: get the original vertex information"<<endl;
            string layer1="N"+dataset+"_point";
            string source1=inPath+layer1+".shp";
            char* char_array1 = new char[source1.length() + 1];
            strcpy(char_array1, source1.c_str());
            char* char_layer1 = new char[layer1.length() + 1];
            strcpy(char_layer1, layer1.c_str());
            NodePointProcess(char_array1,char_layer1,inPath+dataset+".node");
            delete[] char_array1; delete[] char_layer1;

            /// Step 2: get the original edge information
            cout<<"Step 2: get the original edge information"<<endl;
            string layer2="R"+dataset+"_polyline";
            string source2=inPath+layer2+".shp";
            char* char_array2 = new char[source2.length() + 1];
            strcpy(char_array2, source2.c_str());
            char* char_layer2 = new char[layer2.length() + 1];
            strcpy(char_layer2, layer2.c_str());
            EdgePolylineProcess(char_array2,char_layer2,inPath+dataset+".edge");
            delete[] char_array2; delete[] char_layer2;
        }


        /// Step 3: get the original road network
        cout<<"Step 3: get the original road network"<<endl;
        //For multiple data sources
        GetRoadNetworkAggregation(sourcePath, targetPath, datasets, dataSet);


    }

    /// Step 4: get the largest connected component
    cout<<"Step 4: compute the largest connected component"<<endl;
    RoadNetworkPreprocess(targetPath+dataSet);

    return 0;
}

void GetRoadNetworkAggregation(string sourcePath, string targetPath, vector<string>& datasets, string targetName){
    ifstream IFOut(targetPath + targetName + "_Distance.gr");
    if (IFOut.is_open() && !ifNew) {//already exist
        cout << "File " << sourcePath + targetName + "_Distance.gr" << " already exists."<< endl;
      IFOut.close();
    }
    else{
        IFOut.close();
        vector<string> vs;
        string line;
        unsigned long long int ID1, ID2;
        long long int edgeID;
        int weightD, weightT;
        int lineNum = 0;
        vector<tuple<int, int, int, int>> edges;//ID1,ID2,length (m),travel time (s)
        map<int, int> speedClass;
        double speedMap[7] = {33.33, 27.77, 22.22, 16.66, 11.11, 8.33,
                              2.77};// m/s, corresponding to 120, 100, 80, 60, 40, 30, 10 km/h
        int speedClassTemp = 0;
        int direction;
        int lineNum2 = 0;
        map<unsigned long long int, int> IDMap;//map vertex ID from old to new
        map<long long int, pair<int,int>> EdgeToNodeMap;//map old edge ID to its endpoints' new ID
        map<long long int, int> EdgeIDMap;//map edge ID to new ID
//        set<long long int> EdgeIDs;//collection of edge ID
        int newID = 0;
        double speedTemp;
        map<int, pair<double,double>> nodeGPS;// new ID to its coordinate
        double minLon=INT16_MAX, minLat=INT16_MAX;
        double maxLon=-INT16_MAX, maxLat=-INT16_MAX;
        int edgeIDnew=0;

        for(int di=0;di<datasets.size();++di){
            string sourcePath1 = sourcePath+datasets[di]+"/road/";
            string nodeFile=sourcePath1 + datasets[di] + ".node";
            ifstream nodeIF(nodeFile, ios::in);
            if (!nodeIF.is_open()) {
                cout << "Open node file failed!" << nodeFile << endl;
                exit(1);
            }
            getline(nodeIF, line);
            getline(nodeIF, line);
            vs = split(line, " \t");
            lineNum = 0;
            if (vs.size() == 1) {
                lineNum = stoi(vs[0]);
                cout << "Node number: " << lineNum << endl;
            } else {
                cout << "Wrong syntax! " << line << endl;
                exit(1);
            }
            double lat, lon;
            lineNum2=0;
            while(getline(nodeIF,line))
            {
                if(line=="") continue;
                vs.clear();
                vs = split(line," \t");
                if(vs.size()<4){
                    cout<<"Wrong syntax! "<<line<<endl; exit(1);
                }
                ID1=stoull(vs[0]), lon=stod(vs[1]), lat=stod(vs[2]), ID2=stoull(vs[vs.size()-1]);
                //!!! a vertex has two ID, ID1 and ID2
                if(IDMap.find(ID1)==IDMap.end()){//if not found ID1
                    if(IDMap.find(ID2)==IDMap.end()){//if not found ID2
                        nodeGPS.insert({newID, make_pair(lon,lat)});
                        IDMap.insert({ID1,newID});
                        newID++;
                    }else{//if found ID2
                        IDMap.insert({ID1,IDMap[ID2]});
                    }

                }else{//if found ID1
                    cout<<"Wrong! found "<<ID2<<" "<<ID1<<" "<<lon<<" "<<lat<<" "<<newID<<" "<<lineNum2<<endl; exit(1);
                }

                if(minLon>lon) minLon=lon;
                if(minLat>lat) minLat=lat;
                if(maxLon<lon) maxLon=lon;
                if(maxLat<lat) maxLat=lat;

                lineNum2++;
            }
            if(lineNum!=lineNum2){
                cout<<"Inconsistent ID number."<<endl; exit(1);
            }

            string edgeFile=sourcePath1 + datasets[di] + ".edge";
            ifstream edgeIF(edgeFile, ios::in);
            if (!edgeIF.is_open()) {
                cout << "Open edge file failed!" << edgeFile << endl;
                exit(1);
            }

            getline(edgeIF, line);
            getline(edgeIF, line);
            vs = split(line, " \t");

            if (vs.size() == 1) {
                lineNum = stoi(vs[0]);
                cout << "Edge number: " << lineNum << endl;
            } else {
                cout << "Wrong syntax! " << line << endl;
                exit(1);
            }

            while (getline(edgeIF, line)) {
                if (line == "") continue;
                vs.clear();
                vs = split(line, " \t");
                if (vs.size() < 11) {
                    cout << "Wrong syntax! " << line << endl;
                    exit(1);
                }
                ID1 = stoull(vs[3]), ID2 = stoull(vs[4]), weightD = stoi(vs[2]), speedClassTemp = stoi(vs[5]);
                direction = stoi(vs[1]);
                edgeID = stoll(vs[0]);

                double weightEuc= EuclideanDis(nodeGPS[IDMap[ID1]],nodeGPS[IDMap[ID2]]);
                if(weightD<weightEuc){
                    int newWeiD=max(weightEuc+2,weightEuc*1.05);
                    cout<<"Invalid edge distance! "<<ID1<<" "<<ID2<<" "<<weightD<<" "<<weightEuc<<" "<< newWeiD<<endl;
                    weightD=newWeiD;
                }
//            EdgeMap.insert({edgeID, make_pair(ID1,ID2)});

                if(EdgeIDMap.find(edgeID)==EdgeIDMap.end()){//if not found
                    EdgeIDMap.insert({edgeID,edgeIDnew});
                    edgeIDnew++;
                    EdgeToNodeMap.insert({edgeID, make_pair(IDMap[ID1],IDMap[ID2])});
                }else{
                    cout<<"Already exist! "<<edgeID<<endl; exit(1);
                }

                speedTemp=speedMap[speedClassTemp - 2];

                weightT = ceil((double) weightD / speedTemp);
//        cout<<weightT<<" "<<speedTemp<<" "<<weightD<<endl;
                // speedclass: 2: 120 km/h; 3: 100 km/h; 4: 80 km/h; 5: 60 km/h; 6: 40 km/h; 7: 30 km/h; 8: 10 km/h
                if (speedClass.find(speedClassTemp) == speedClass.end()) {//if not found
                    speedClass.insert({speedClassTemp, 1});
                } else {
                    speedClass[speedClassTemp]++;
                }

                if(direction == 1 || direction == 0){
                    edges.emplace_back(IDMap[ID1], IDMap[ID2], weightD, weightT);
                    edges.emplace_back(IDMap[ID2], IDMap[ID1], weightD, weightT);
                }else if(direction == 2){
                    edges.emplace_back(IDMap[ID1], IDMap[ID2], weightD, weightT);
                }else if(direction == 3){
                    edges.emplace_back(IDMap[ID2], IDMap[ID1], weightD, weightT);
                }else{
                    cout<<"Wrong direction! "<<direction<<endl; exit(1);
                }

                lineNum2++;
            }


        }


        if(nodeGPS.size()!=newID){
            cout<<"Inconsistent node number! "<<newID<<" "<<nodeGPS.size()<<endl; exit(1);
        }

        cout << "Node number: " << nodeGPS.size() << " ; Edge number: " << edges.size() << " ("<<edgeIDnew+1<<")"<< endl;
        cout<<"Original GPS range: [ "<< minLon<<" "<<maxLon<<" ]; [ "<<minLat<<" "<<maxLat<<" ]"<<endl;
        cout<<"Speed class: ";
        for (auto it = speedClass.begin(); it != speedClass.end(); ++it) {
            cout << it->first << " " << it->second << " ;\t";
        }
        cout << endl;

        ofstream OF1(targetPath + targetName + "_Distance.gr");
        if (!OF1.is_open()) {
            cout << "Open file failed!" << sourcePath + targetName + "_Distance.gr" << endl;
            exit(1);
        }
        OF1 << newID << " " << edges.size() << endl;
        for (auto it = edges.begin(); it != edges.end(); ++it) {
            OF1 << get<0>(*it) << " " << get<1>(*it) << " " << get<2>(*it) << endl;
//        cout << get<0>(*it) << " " << get<1>(*it) << " " << get<2>(*it) <<" "<<get<3>(*it)<< endl;
        }
        OF1.close();

        ofstream OF2(targetPath + targetName + "_Time.gr");
        if (!OF2.is_open()) {
            cout << "Open file failed!" << sourcePath + targetName + "_Time.gr" << endl;
            exit(1);
        }
        OF2 << newID << " " << edges.size() << endl;
        for (auto it = edges.begin(); it != edges.end(); ++it) {
            OF2 << get<0>(*it) << " " << get<1>(*it) << " " << get<3>(*it) << endl;
        }
        OF2.close();

        ofstream OF3(targetPath + targetName + "_NodeIDMap");
        if (!OF3.is_open()) {
            cout << "Open file failed!" << sourcePath + targetName + "_NodeIDMap" << endl;
            exit(1);
        }
        OF3 << newID << endl;
        for (auto it = IDMap.begin(); it != IDMap.end(); ++it) {
            OF3 << it->first << " " << it->second << endl;
        }
        OF3.close();
        cout<<"Finish graph generation."<<endl;

        ofstream OF4(targetPath + targetName + "_Coordinate.co");
        if (!OF4.is_open()) {
            cout << "Open file failed!" << sourcePath + targetName + "_Coordinate.co" << endl;
            exit(1);
        }
        OF4 << nodeGPS.size() << endl;
        for (auto it=nodeGPS.begin();it!=nodeGPS.end(); ++it) {
            OF4 << it->first << " " << int(1000000*it->second.first) << " " << int(1000000*it->second.second) << endl;
        }
        OF4.close();

        ofstream OF5(targetPath + targetName + "_EdgeToNodeMap");
        if (!OF5.is_open()) {
            cout << "Open file failed!" << sourcePath + targetName + "_EdgeToNodeMap" << endl;
            exit(1);
        }
        OF5 << EdgeToNodeMap.size() << endl;
        for (auto it = EdgeToNodeMap.begin(); it != EdgeToNodeMap.end(); ++it) {
            OF5 << it->first << " " << it->second.first<<" "<<it->second.second << endl;
        }
        OF5.close();

        ofstream OF6(targetPath + targetName + "_EdgeIDMap");
        if (!OF6.is_open()) {
            cout << "Open file failed!" << sourcePath + targetName + "_EdgeIDMap" << endl;
            exit(1);
        }
        OF6 << EdgeIDMap.size() << endl;
        for (auto it = EdgeIDMap.begin(); it != EdgeIDMap.end(); ++it) {
            OF6 << it->first << " " << it->second << endl;
        }
        OF6.close();

        cout<<"Done."<<endl;
    }

}


// function of getting the road network by mapping original vertex ID to 0-start ID
void GetRoadNetwork(string edgeFile, string nodeFile, string sourcePath) {
    vector<string> vs;
    string line;
    unsigned long long int ID1, ID2;
    long long int edgeID;
    int weightD, weightT;
    vector<tuple<int, int, int, int>> edges;//ID1,ID2,length (m),travel time (s)
    map<int, int> speedClass;
    double speedMap[7] = {33.33, 27.77, 22.22, 16.66, 11.11, 8.33,
                          2.77};// m/s, corresponding to 120, 100, 80, 60, 40, 30, 10 km/h
    int speedClassTemp = 0;
    int direction;
    int lineNum2 = 0;
    map<unsigned long long int, int> IDMap;//map vertex ID from old to new
    map<long long int, pair<int,int>> EdgeToNodeMap;//map edge ID to its endpoints' new ID
    map<long long int, int> EdgeIDMap;//map edge ID to new ID
//    set<long long int> EdgeIDs;//collection of edge ID
    int newID = 0;
    double speedTemp;
    double minLon=INT16_MAX, minLat=INT16_MAX;
    double maxLon=-INT16_MAX, maxLat=-INT16_MAX;
    int edgeIDnew=0;
    int lineNum = 0;
    // Read node file
    map<int, pair<double,double>> nodeGPS;// map coordinate to its new node ID
    ifstream nodeIF(nodeFile, ios::in);
    if (!nodeIF.is_open()) {
        cout << "Open node file failed!" << nodeFile << endl;
        exit(1);
    }
    getline(nodeIF, line);
    getline(nodeIF, line);
    vs = split(line, " \t");
    lineNum = 0;
    if (vs.size() == 1) {
        lineNum = stoi(vs[0]);
        cout << "Node number: " << lineNum << endl;
    } else {
        cout << "Wrong syntax! " << line << endl;
        exit(1);
    }
    double lat, lon;
    lineNum2=0;
    newID=0;
    while(getline(nodeIF,line))
    {
        if(line=="") continue;
        vs.clear();
        vs = split(line," \t");
        if(vs.size()<4){
            cout<<"Wrong syntax! "<<line<<endl; exit(1);
        }
        ID1=stoull(vs[0]), lon=stod(vs[1]), lat=stod(vs[2]), ID2=stoull(vs[vs.size()-1]);
        //!!! a vertex has two ID, ID1 and ID2
        if(IDMap.find(ID1)==IDMap.end()){//if not found ID1
            if(IDMap.find(ID2)==IDMap.end()){//if not found ID2
                nodeGPS.insert({newID, make_pair(lon,lat)});
                IDMap.insert({ID1,newID});
                newID++;
            }else{//if found ID2
                IDMap.insert({ID1,IDMap[ID2]});
            }

        }else{//if found ID1
            cout<<"Wrong! found "<<ID2<<" "<<ID1<<" "<<lon<<" "<<lat<<" "<<newID<<" "<<lineNum2<<endl; exit(1);
        }

        if(minLon>lon) minLon=lon;
        if(minLat>lat) minLat=lat;
        if(maxLon<lon) maxLon=lon;
        if(maxLat<lat) maxLat=lat;

        lineNum2++;
    }
    if(lineNum!=lineNum2){
        cout<<"Inconsistent ID number."<<endl; exit(1);
    }
    nodeIF.close();
    if(nodeGPS.size()!=newID){
        cout<<"Inconsistent node number! "<<newID<<" "<<nodeGPS.size()<<endl; exit(1);
    }
    /// Read edge file
    ifstream edgeIF(edgeFile, ios::in);
    if (!edgeIF.is_open()) {
        cout << "Open edge file failed!" << edgeFile << endl;
        exit(1);
    }
    getline(edgeIF, line);
    getline(edgeIF, line);
    vs = split(line, " \t");
    if (vs.size() == 1) {
        lineNum = stoi(vs[0]);
        cout << "Edge number: " << lineNum << endl;
    } else {
        cout << "Wrong syntax! " << line << endl;
        exit(1);
    }
    while (getline(edgeIF, line)) {
        if (line == "") continue;
        vs.clear();
        vs = split(line, " \t");
        if (vs.size() < 11) {
            cout << "Wrong syntax! " << line << endl;
            exit(1);
        }
        ID1 = stoull(vs[3]), ID2 = stoull(vs[4]), weightD = stoi(vs[2]), speedClassTemp = stoi(vs[5]);
        direction = stoi(vs[1]);
        edgeID = stoll(vs[0]);

        double weightEuc= EuclideanDis(nodeGPS[IDMap[ID1]],nodeGPS[IDMap[ID2]]);
//        cout<<ID1<<" "<<ID2<<" "<<weightD<<" "<<weightEuc<<endl;
        if(weightD<weightEuc){
            int newWeiD=max(weightEuc+2,weightEuc*1.05);
            cout<<"Invalid edge distance! "<<ID1<<" "<<ID2<<" "<<weightD<<" "<<weightEuc<<" "<< newWeiD<<endl;
            weightD=newWeiD;
        }

//        EdgeMap.insert({edgeID, make_pair(ID1,ID2)});


        if(EdgeIDMap.find(edgeID)==EdgeIDMap.end()){//if not found
            EdgeIDMap.insert({edgeID,edgeIDnew});
            edgeIDnew++;
            EdgeToNodeMap.insert({edgeID, make_pair(IDMap[ID1],IDMap[ID2])});
        }else{
            cout<<"Already exist! "<<edgeID<<endl; exit(1);
        }
//        EdgeMap.insert({edgeID, make_pair(IDMap[ID1],IDMap[ID2])});

        speedTemp=speedMap[speedClassTemp - 2];

        weightT = ceil((double) weightD / speedTemp);
//        cout<<weightT<<" "<<speedTemp<<" "<<weightD<<endl;
        // speedclass: 2: 120 km/h; 3: 100 km/h; 4: 80 km/h; 5: 60 km/h; 6: 40 km/h; 7: 30 km/h; 8: 10 km/h
        if (speedClass.find(speedClassTemp) == speedClass.end()) {//if not found
            speedClass.insert({speedClassTemp, 1});
        } else {
            speedClass[speedClassTemp]++;
        }

        if(direction == 1 || direction == 0){
            edges.emplace_back(IDMap[ID1], IDMap[ID2], weightD, weightT);
            edges.emplace_back(IDMap[ID2], IDMap[ID1], weightD, weightT);
        }else if(direction == 2){
            edges.emplace_back(IDMap[ID1], IDMap[ID2], weightD, weightT);
        }else if(direction == 3){
            edges.emplace_back(IDMap[ID2], IDMap[ID1], weightD, weightT);
        }else{
            cout<<"Wrong direction! "<<direction<<endl; exit(1);
        }

        lineNum2++;
    }
    edgeIF.close();


    cout << "Node number: " << nodeGPS.size() << " ; Edge number: " << edges.size() << " ("<<edgeIDnew+1<<")"<< endl;
    cout<<"Original Longitude range: "<< minLon<<" "<<maxLon<<" ; Latitude range: "<<minLat<<" "<<maxLat<<endl;
    cout<<"Speed class: ";
    for (auto it = speedClass.begin(); it != speedClass.end(); ++it) {
        cout << it->first << " " << it->second << " ;\t";
    }
    cout << endl;

    ofstream OF1(sourcePath + "_Distance.gr");
    if (!OF1.is_open()) {
        cout << "Open file failed!" << sourcePath + "_Distance.gr" << endl;
        exit(1);
    }
    OF1 << newID << " " << edges.size() << endl;
    for (auto it = edges.begin(); it != edges.end(); ++it) {
        OF1 << get<0>(*it) << " " << get<1>(*it) << " " << get<2>(*it) << endl;
//        cout << get<0>(*it) << " " << get<1>(*it) << " " << get<2>(*it) <<" "<<get<3>(*it)<< endl;
    }
    OF1.close();

    ofstream OF2(sourcePath + "_Time.gr");
    if (!OF2.is_open()) {
        cout << "Open file failed!" << sourcePath + "_Time.gr" << endl;
        exit(1);
    }
    OF2 << newID << " " << edges.size() << endl;
    for (auto it = edges.begin(); it != edges.end(); ++it) {
        OF2 << get<0>(*it) << " " << get<1>(*it) << " " << get<3>(*it) << endl;
    }
    OF2.close();

    ofstream OF3(sourcePath + "_NodeIDMap");
    if (!OF3.is_open()) {
        cout << "Open file failed!" << sourcePath + "_NodeIDMap" << endl;
        exit(1);
    }
    OF3 << newID << endl;
    for (auto it = IDMap.begin(); it != IDMap.end(); ++it) {
        OF3 << it->first << " " << it->second << endl;
    }
    OF3.close();
    cout<<"Finish graph generation."<<endl;



    ofstream OF4(sourcePath + "_Coordinate.co");
    if (!OF4.is_open()) {
        cout << "Open file failed!" << sourcePath + "_Coordinate.co" << endl;
        exit(1);
    }
    OF4 << nodeGPS.size() << endl;
    for (auto it=nodeGPS.begin();it!=nodeGPS.end(); ++it) {
        OF4 << it->first << " " << int(1000000*it->second.first) << " " << int(1000000*it->second.second) << endl;
    }
    OF4.close();

    ofstream OF5(sourcePath + "_EdgeToNodeMap");
    if (!OF5.is_open()) {
        cout << "Open file failed!" << sourcePath + "_EdgeToNodeMap" << endl;
        exit(1);
    }
    OF5 << EdgeToNodeMap.size() << endl;
    for (auto it = EdgeToNodeMap.begin(); it != EdgeToNodeMap.end(); ++it) {
        OF5 << it->first << " " << it->second.first<<" "<<it->second.second << endl;
    }
    OF5.close();

    ofstream OF6(sourcePath + "_EdgeIDMap");
    if (!OF6.is_open()) {
        cout << "Open file failed!" << sourcePath + "_EdgeIDMap" << endl;
        exit(1);
    }
    OF6 << EdgeIDMap.size() << endl;
    for (auto it = EdgeIDMap.begin(); it != EdgeIDMap.end(); ++it) {
        OF6 << it->first << " " << it->second << endl;
    }
    OF6.close();

    cout<<"Done."<<endl;
}
// function of extracting edge information from polyline.shp
void EdgePolylineProcess(char * sourceFile, char * layerName, string outputFile){
    ifstream ifile(outputFile);
    if(!ifile.is_open()) {//not open
        ifile.close();
        GDALDataset *poDS;  //Data source
        poDS = (GDALDataset*) GDALOpenEx(sourceFile, GDAL_OF_VECTOR, NULL, NULL, NULL);

        if(poDS == NULL)
        {
            cout << "Open shp file failed!" << endl;
            exit(1);
        }
        cout << "Data source open success!" << endl;

        OGRLayer *poLayer;
//    poLayer = poDS->GetLayerByName("Nfujian_point");
        poLayer = poDS->GetLayerByName(layerName);
        //feature is a geometry and a set of attributes
        OGRFeature *poFeature;
        poLayer->ResetReading();    //Start at the beginning of the layer
        cout << "Feature number:" << poLayer->GetFeatureCount() << endl;
        auto layerDefn=poLayer->GetLayerDefn();
        for(int i=0;i<layerDefn->GetFieldCount();++i){
            cout<<layerDefn->GetFieldDefn(i)->GetNameRef()<<" ";
        }
        cout<<endl;
        ofstream ofile(outputFile);
        if(!ofile.is_open()){
            cout << "Open file failed!" << outputFile << endl;
            exit(1);
        }

        ofile << "ID\tDirection\tLength\tSNodeID\tENodeID\tSpeedClass\tSpdLmtS2E\tSpdLmtE2S\tToll\tLaneNum\tWidth\tKingNum\tKing"<<endl;
        ofile << poLayer->GetFeatureCount()<<endl;//feature number
        while((poFeature = poLayer->GetNextFeature()) != NULL)
        {
            ofile <<poFeature->GetFieldAsString(1);			//ID

            ofile << "\t"<< poFeature->GetFieldAsString(5);	//Direction
            auto temp=poFeature->GetFieldAsDouble(12) * 1000;
            ofile << "\t"<< (int)temp;	                    //Length

            ofile << "\t" << poFeature->GetFieldAsString(9);    //Source ID
            ofile << "\t" << poFeature->GetFieldAsString(10);   //End ID

            ofile << "\t" << poFeature->GetFieldAsString(24);	//Speed Limit class, 1: >130 km/h; 2: (100 km/h, 130 km/h]; 3: (90 km/h, 100 km/h]; 4: (70 km/h, 90 km/h]; 5: (50 km/h, 70 km/h]; 6: (30 km/h, 50 km/h]; 7: [11 km/h, 30 km/h]; 8: <11 km/h
            ofile << "\t" << poFeature->GetFieldAsString(33);	//Speed Limit from source to end, 0.1 km/s
            ofile << "\t" << poFeature->GetFieldAsString(34);	//Speed Limit from end to source, 0.1 km/h
            ofile << "\t" << poFeature->GetFieldAsString(6);	//Toll fee
            ofile << "\t" << poFeature->GetFieldAsString(27);	//Lane Number
            ofile << "\t" << poFeature->GetFieldAsString(4);	//Width
            ofile << "\t" << poFeature->GetFieldAsString(2);	//KindNumber
            ofile << "\t" << poFeature->GetFieldAsString(3);	//Kind

            ofile<<endl;
            OGRFeature::DestroyFeature(poFeature);
        }
        GDALClose(poFeature);
        ofile.close();
        cout<<"Write Done.\n"<<endl;
    }
    else{
        cout<<"File "<<outputFile<<" already exist."<<endl;
        ifile.close();
    }

}
// function of extracting node information from point.shp
void NodePointProcess(char * sourceFile, char * layerName, string outputFile){
    ifstream ifile(outputFile);
    if(!ifile.is_open()){//not open
        ifile.close();
        GDALDataset *poDS;  //Data source
//    string sourcePath="/Users/zhouxj/Documents/1-Research/Datasets/NavInfo/map/fujian/road/";
//    poDS = (GDALDataset*) GDALOpenEx("./beijing/road/Nbeijing_point.shp", GDAL_OF_VECTOR, NULL, NULL, NULL);
        poDS = (GDALDataset*) GDALOpenEx(sourceFile, GDAL_OF_VECTOR, NULL, NULL, NULL);
//    poDS = (GDALDataset*) GDALOpenEx("/Users/zhouxj/Documents/1-Research/Datasets/NavInfo/map/fujian/road/Nfujian_point.shp", GDAL_OF_VECTOR, NULL, NULL, NULL);

        if(poDS == NULL)
        {
            cout << "Open shp file failed!" << endl;
            exit(1);
        }
        cout << "Data source open success!" << endl;

        int layerNum = poDS->GetLayerCount();   //a dataset may have many layers
        cout << "Layer number:" << layerNum << endl;

        OGRLayer *poLayer;
//    poLayer = poDS->GetLayerByName("Nfujian_point");
        poLayer = poDS->GetLayerByName(layerName);

        //feature is a geometry and a set of attributes
        OGRFeature *poFeature;
        poLayer->ResetReading();    //Start at the beginning of the layer
        cout << "Feature number:" << poLayer->GetFeatureCount() << endl;

        auto layerDefn=poLayer->GetLayerDefn();
        for(int i=0;i<layerDefn->GetFieldCount();++i){
            cout<<layerDefn->GetFieldDefn(i)->GetNameRef()<<" ";
        }
        cout<<endl;

        stringstream ss;
        map<string, pair<double, double> > mip;
        map<string, pair<double, double> >::iterator imip;

        ofstream ofile(outputFile);
        if(!ofile.is_open()){
            cout << "Open file failed!" << outputFile << endl;
            exit(1);
        }

        ofile << "ID\tlatitudinal\tlongitudinal\tlight_flag\tnode_lid.size\tnode_lid\tCrossFlag\tCross_lid.size\tCross_lid\tMainNodeid\tsubnodeid\tsubnodeid2\tadjoin_nid"<<endl;
        ofile << poLayer->GetFeatureCount()<<endl;//feature number

        string id;
        int	crossFlag;
        string stmp, stmp2;
        vector<string> vs;
        vector<string>::iterator ivs;
        while((poFeature = poLayer->GetNextFeature()) != NULL)
        {
            //Defn contains the definition of all the fields
            OGRFeatureDefn *poFDefn = poLayer->GetLayerDefn();
//        for(iField = 0; iField < poFDefn->GetFieldCount(); iField++)
            //       {
//        OGRFieldDefn * poFieldDefn = poFDefn->GetFieldDefn(iField);
            id = poFeature->GetFieldAsString(1);

            ofile << id;//ID

            OGRGeometry *poGeometry;
            poGeometry = poFeature->GetGeometryRef();
//        cout << "Geometry Type:" << poGeometry->getGeometryType() << endl;
            if(poGeometry != NULL && wkbFlatten(poGeometry->getGeometryType()) == wkbMultiPoint)
            {
                OGRMultiPoint *poMultiPoint = (OGRMultiPoint*)poGeometry;
                //          cout << "Number in the MultiPoint:" << poMultiPoint->getNumGeometries() << endl;

                OGRGeometry *pG;
                pG = poMultiPoint->getGeometryRef(0);
                OGRPoint *pP = (OGRPoint*)pG;
                ofile << "\t" << pP->getX()<< "\t" << pP->getY();//longi,lati
//            pP->flattenTo2D();
                //           cout <<  setprecision(15) << pP->getY() << ", " << pP->getX() << endl;
                //           mip[id] = make_pair(pP->getY(), pP->getX());

            }
            else
                cout << "No Point Geometry" << endl;

            ofile << "\t" << poFeature->GetFieldAsString(5);//light_flag, 0: no light, 1: with light

            stmp = poFeature->GetFieldAsString(12);
            vs = split(stmp, "|");
            ofile << "\t" << vs.size();
            for(ivs = vs.begin(); ivs != vs.end(); ivs++)
                ofile << "\t" << *ivs;//node_lid
            vs.clear();
/*            if(poFieldDefn->GetType() == OFTInteger)
                cout << poFeature->GetFieldAsInteger(iField) << ", ";
            else if(poFieldDefn->GetType() == OFTInteger64)
                cout << poFeature->GetFieldAsInteger64(iField) << ", ";
            else if(poFieldDefn->GetType() == OFTReal)
                cout << setprecision(15) << poFeature->GetFieldAsDouble(iField) << ", ";
            else if(poFieldDefn->GetType() == OFTString)
                cout << poFeature->GetFieldAsString(iField) << ", ";
            else
                cout << poFeature->GetFieldAsString(iField) << ", ";*/
//        }



            crossFlag = poFeature->GetFieldAsInteger(4);//Cross flag
            if(crossFlag == 0)//if not an intersection
            {
                ofile << "\t" << crossFlag << "\t" << 0 << "\t" << 0 << "\t" << 0;
            }
            else if(crossFlag == 1)
            {
                ofile << "\t" << crossFlag << "\t" << 0 << "\t" << poFeature->GetFieldAsInteger(7) << "\t" << 0;
            }
            else if(crossFlag == 2)
            {
                stmp = poFeature->GetFieldAsString(6);
                vs = split(stmp, "|");
                ofile << "\t" << crossFlag << "\t" << vs.size();
                for(ivs = vs.begin(); ivs != vs.end(); ivs++)
                    ofile << "\t" << *ivs;
                vs.clear();
                ofile << "\t" << poFeature->GetFieldAsInteger(7) << "\t" << 0;
            }
            else if(crossFlag == 3)
            {
                stmp = poFeature->GetFieldAsString(6);
                vs = split(stmp, "|");
                ofile << "\t" << crossFlag << "\t" << vs.size();
                for(ivs = vs.begin(); ivs != vs.end(); ivs++)
                    ofile << "\t" << *ivs;
                vs.clear();

                ofile << "\t" << poFeature->GetFieldAsInteger(7);
                stmp = poFeature->GetFieldAsString(8);
                stmp2 = poFeature->GetFieldAsString(9);
                if(stmp2 != "")
                    stmp += "|" + stmp2;
                vs = split(stmp, "|");
                ofile << "\t" << vs.size();
                for(ivs = vs.begin(); ivs != vs.end(); ivs++)
                    ofile << "\t" << *ivs;
                vs.clear();
            }

            ofile << "\t" << poFeature->GetFieldAsString(11);


            ofile << endl;
            OGRFeature::DestroyFeature(poFeature);
        }

/*	cout << "Writing nodes" << endl;
    for(imip = mip.begin(); imip != mip.end(); imip++)
        ofile << setprecision(15) << (*imip).first << "\t" << (*imip).second.first << "\t" << (*imip).second.second << endl;*/

        GDALClose(poFeature);
        ofile.close();
        cout<<"Write Done.\n"<<endl;
    }
    else{
        cout<<"File "<<outputFile<<" already exist."<<endl;
        ifile.close();
    }

}

void RoadNetworkPreprocess(string graph_path){
    vector<unordered_map<int,int>> NeighborMap; //adjacency list of original graph, map version
    vector<unordered_map<int,int>> NeighborMap2; //adjacency list of original graph, map version

    string gFile=graph_path+"_Time.gr";
    string coFile=graph_path+"_Coordinate.co";

    int node_num;
    unsigned long long int edge_num;
    vector<pair<int,int>> Coordinate;//coordinates of vertex

    /// read edges
    ifstream inGraph(gFile, ios::in);
    if (!inGraph) { // if not exist
        cout << "Fail to open file " << gFile << endl;
        exit(1);
    }
    cout<<"Reading graph "<<gFile<<endl;
    string line;
    vector<string> vs;
    int ID1,ID2,weight;
    // time graph
    getline(inGraph,line);
    vs.clear();
    boost::split(vs,line,boost::is_any_of(" "));
    node_num=stoi(vs[0]), edge_num=stoi(vs[1]);
    cout<<"Node number: "<<node_num<<" , edge number: "<<edge_num<<endl;
    NeighborMap.assign(node_num,unordered_map<int,int>());
    unsigned long long int edgeNum=0;
    while(getline(inGraph,line)){
        if(line.empty()) continue;
        istringstream iss(line);
        if (!(iss >> ID1 >> ID2 >> weight)){
            cout<<"Wrong input syntax!"<<endl;
            exit(1);
        }

        if(ID1>=0 && ID1<node_num && ID2>=0 && ID2<node_num && weight>0){
//            NeighborMap[ID1].insert({ID2,weight});
            if(NeighborMap[ID1].find(ID2)==NeighborMap[ID1].end()){//if not found
                NeighborMap[ID1].insert({ID2,weight});
                NeighborMap[ID2].insert({ID1,weight});
                edgeNum+=2;
            }
        }else{
            cout<<"Graph data is wrong! "<<ID1<<" "<<ID2<<" "<<weight<<endl;
        }
    }
    inGraph.close();
    cout<<"Finished."<<endl;
    cout<<"New edge number: "<<edgeNum<<endl;

    gFile=graph_path+"_Distance.gr";
    ifstream inGraph2(gFile, ios::in);
    if (!inGraph2) { // if not exist
        cout << "Fail to open file " << gFile << endl;
        exit(1);
    }
    cout<<"Reading graph "<<gFile<<endl;
    // time graph
    getline(inGraph2,line);
    vs.clear();
    boost::split(vs,line,boost::is_any_of(" "));
    node_num=stoi(vs[0]), edge_num=stoi(vs[1]);
    cout<<"Node number: "<<node_num<<" , edge number: "<<edge_num<<endl;
    NeighborMap2.assign(node_num,unordered_map<int,int>());
    edgeNum=0;
    while(getline(inGraph2,line)){
        if(line.empty()) continue;
        istringstream iss(line);
        if (!(iss >> ID1 >> ID2 >> weight)){
            cout<<"Wrong input syntax!"<<endl;
            exit(1);
        }

        if(ID1>=0 && ID1<node_num && ID2>=0 && ID2<node_num && weight>0){
//            NeighborMap[ID1].insert({ID2,weight});
            if(NeighborMap2[ID1].find(ID2)==NeighborMap2[ID1].end()){//if not found
                NeighborMap2[ID1].insert({ID2,weight});
                NeighborMap2[ID2].insert({ID1,weight});
                edgeNum+=2;
            }
        }else{
            cout<<"Graph data is wrong! "<<ID1<<" "<<ID2<<" "<<weight<<endl;
        }
    }
    inGraph2.close();
    cout<<"Finished."<<endl;
    cout<<"New edge number: "<<edgeNum<<endl;
    /// read coordinates
    ifstream inCoord(coFile, ios::in);
    if (!inCoord) { // if not exist
        cout << "Fail to open file " << coFile << endl;
        exit(1);
    }
    cout<<"Reading graph coordinates."<<endl;


    int co1,co2;

    getline(inCoord,line);
    vs.clear();
    boost::split(vs,line,boost::is_any_of(" "));
    int nodeNum=stoi(vs[0]);
    if(nodeNum != node_num){
        cout<<"Inconsistent node number! "<<node_num<<" "<<nodeNum<<endl; exit(1);
    }
    Coordinate.assign(node_num,pair<int,int>());
    while(getline(inCoord,line)){
        if(line.empty()) continue;
        istringstream iss(line);
        if (!(iss >> ID1 >> co1 >> co2)){
            cout<<"Wrong input syntax!"<<endl;
            exit(1);
        }

        if(ID1>=0 && ID1<node_num ){
            Coordinate[ID1].first=co1, Coordinate[ID1].second=co2;
        }else{
            cout<<"Graph data is wrong! "<<ID1<<" "<<co1<<" "<<co2<<endl;
        }
    }

    inCoord.close();
    cout<<"Finished."<<endl;


    unordered_set<int> vertices; vertices.clear();
    for(int i=0;i<node_num;++i){
        vertices.insert(i);
    }
    set<int> verticesFinal;
    pair<int, unsigned long long int> LCC;
    LCC = DFS_CC(NeighborMap,vertices,verticesFinal,node_num);
    map<int,int> IDMap;
    int ID=0;
    for(auto it=verticesFinal.begin();it!=verticesFinal.end();++it){
        IDMap.insert({*it,ID});
        ++ID;
    }
    if(IDMap.size()!=LCC.first){
        cout<<"Wrong! Inconsistent! "<<IDMap.size()<<" "<<LCC.first<<endl; exit(1);
    }
    /// Write ID map
    ofstream OF(graph_path+".IDMap", ios::out);
    if(!OF.is_open()){
        cout<<"Cannot open file "<<endl; exit(1);
    }
    OF<<IDMap.size()<<endl;
    for(auto it=IDMap.begin();it!=IDMap.end();++it){
        OF<<it->first<<" "<<it->second<<endl;//from old id to new id of LCC
    }
    OF.close();
    /// Write graph edges
    string wGraph=graph_path;
    wGraph=graph_path+".time";//+"2";
    ofstream outGraph(wGraph, ios::out);
    if(!outGraph.is_open()){
        cout<<"Cannot open file "<<wGraph<<endl; exit(1);
    }
    cout<<"Writing graph edges. "<<wGraph<<endl;
    outGraph<<LCC.first<<" "<<LCC.second<<endl;

    for(auto it=verticesFinal.begin();it!=verticesFinal.end();++it){
        ID1=*it;
        for(auto it2=NeighborMap[ID1].begin();it2!=NeighborMap[ID1].end();++it2){
            ID2=it2->first, weight=it2->second;
            outGraph<<IDMap[ID1]<<" "<<IDMap[ID2]<<" "<<weight<<endl;
        }
    }
    outGraph.close();
    cout<<"Finished."<<endl;

    wGraph=graph_path+".dis";//+"2";
    ofstream outGraph2(wGraph, ios::out);
    if(!outGraph2.is_open()){
        cout<<"Cannot open file "<<wGraph<<endl; exit(1);
    }
    cout<<"Writing graph edges. "<<wGraph<<endl;
    outGraph2<<LCC.first<<" "<<LCC.second<<endl;

    for(auto it=verticesFinal.begin();it!=verticesFinal.end();++it){
        ID1=*it;
        if(NeighborMap[ID1].size()!=NeighborMap2[ID1].size()){
            cout<<"Inconsistent neighbors between the distance graph and time graph. "<<ID1<<" "<<NeighborMap[ID1].size()<<" "<<NeighborMap2[ID1].size()<<endl; exit(1);
        }
        for(auto it2=NeighborMap2[ID1].begin();it2!=NeighborMap2[ID1].end();++it2){
            ID2=it2->first, weight=it2->second;
            outGraph2<<IDMap[ID1]<<" "<<IDMap[ID2]<<" "<<weight<<endl;
        }
    }
    outGraph2.close();
    cout<<"Finished."<<endl;
    /// Write graph coordinates
    string wCoord=graph_path+".dis.co";
    wCoord=graph_path+".time.co";
    ofstream outCoord(wCoord, ios::out);
    if(!outCoord.is_open()){
        cout<<"Cannot open file "<<wCoord<<endl; exit(1);
    }
    cout<<"Writing graph coordinates. "<<wCoord <<endl;
    outCoord<<LCC.first<<endl;
    pair<double,double> maxLon=make_pair(-999999999,0);
    pair<double,double> maxLat=make_pair(0,-999999999);
    pair<double,double> minLon=make_pair(999999999,0);
    pair<double,double> minLat=make_pair(0,999999999);
    for(auto it=verticesFinal.begin();it!=verticesFinal.end();++it){
        ID1=*it;
        outCoord<<IDMap[ID1]<<" "<<Coordinate[ID1].first<<" "<<Coordinate[ID1].second<<endl;
        if(Coordinate[ID1].first>maxLon.first) {
            maxLon.first=Coordinate[ID1].first; maxLon.second=Coordinate[ID1].second;
        }
        if(Coordinate[ID1].second>maxLat.second) {
            maxLat.second=Coordinate[ID1].second; maxLat.first=Coordinate[ID1].first;
        }
        if(Coordinate[ID1].first<minLon.first) {
            minLon.first=Coordinate[ID1].first; minLon.second=Coordinate[ID1].second;
        }
        if(Coordinate[ID1].second<minLat.second) {
            minLat.second=Coordinate[ID1].second; minLat.first=Coordinate[ID1].first;
        }
    }
    outCoord.close();
    minLon.first/=1000000, minLon.second/=1000000, maxLon.first/=1000000, maxLon.second/=1000000;
    minLat.first/=1000000, minLat.second/=1000000, maxLat.first/=1000000, maxLat.second/=1000000;
    cout<<"Final Longitude range: "<<minLon.first<<" "<<maxLon.first<<" ; Latitude range: "<<minLat.second<<" "<<maxLat.second<<endl;
    vector<pair<double,double>> points;
    points.push_back(minLon); points.push_back(maxLon); points.push_back(minLat); points.push_back(maxLat);
    cout << "Maximal diameter: "<< ComputeDiameter(points)/1000<< " km."<<endl;
    cout<<"Finished."<<endl;
}

template <class T>
pair<int, unsigned long long int> DFS_CC(T & Edges, unordered_set<int> & set_A, set<int> & set_LCC, int nodenum) {
    /// DFS for connected component
    stack<int> stack_A;
//    set<int> set_A;//nodes waiting for visited
    unordered_set<int> set_B;//nodes visited for current component
    set_B.clear();
    int item_id,temp_id;
    vector<bool> flag_visited(nodenum,false);
    bool flag_finish = false;
    unsigned long long int temp_num = 0;
    int component_i = 0;
    pair<unordered_set<int>,unsigned long long int> LCC;
    vector<int> CCs;//the vertex size of each connected component

//    for(int i=0;i<nodenum;++i){
//        set_A.insert(i);
//    }
    int seed = *set_A.begin();
    stack_A.push(seed);
    set_A.erase(seed);
    set_B.insert(seed);
    flag_visited[seed] = true;
    //Get the connected components by DFS
    while(!set_A.empty()) {//if not finish
        temp_num = 0;
        while (!stack_A.empty()) {
            item_id = stack_A.top();
            stack_A.pop();
            for (auto it = Edges[item_id].begin(); it != Edges[item_id].end(); ++it) {
                temp_id = it->first;
                temp_num += 1;
                if (!flag_visited[temp_id]) {//if not visited
                    stack_A.push(temp_id);
                    set_A.erase(temp_id);
                    set_B.insert(temp_id);
                    flag_visited[temp_id] = true;
                }
            }
        }
        if (set_B.size() > LCC.first.size()) {
            LCC.first.clear();
            LCC.first = set_B;
            LCC.second = temp_num;// /2
        }
        assert(!set_B.empty());
        CCs.push_back(set_B.size());
//        if(!set_B.empty() && set_B.size() < mcc.first.size()){
//            cout<<"Smaller connected component with vertex size "<<set_B.size()<<": ";
//            for(auto it=set_B.begin();it!=set_B.end();++it){
//                cout<<*it<<" ";
//            }
//            cout<<"; degree: ";
//            for(auto it=set_B.begin();it!=set_B.end();++it){
//                cout<<Edges[*it].size()<<" ";
//            }
//            cout<<endl;
//        }
        ++component_i;
        set_B.clear();
        if (!set_A.empty()) {
            stack_A.push(*set_A.begin());
            set_B.insert(*set_A.begin());
            flag_visited[*set_A.begin()] = true;
            set_A.erase(*set_A.begin());
        } else {
            break;
        }
    }
    if(component_i==1){
        cout<<"This graph has only one connected component. ";
        cout<<"Nodes size of graph: "<< LCC.first.size() << " ; ";
        cout<<"Edges size of graph: "<< LCC.second << endl;
    }else{
        cout<<"!!! This graph has "<< component_i <<" connected component!"<<endl;
        cout<<"Nodes size of the largest connected component is: "<<LCC.first.size()<<endl;
        cout<<"Edges size of the largest connected component is: "<<LCC.second<<endl;
    }
    for(auto it=LCC.first.begin();it!=LCC.first.end();++it){
        set_LCC.insert(*it);
    }
//    std::sort(CCs.begin(), CCs.end());
//    cout<<"Degree-1 vertex in LCC: ";
//    for(auto it=set_LCC.begin();it!=set_LCC.end();++it){
//        if(NeighborMap[*it].size()==1){
//            cout<<*it<<"("<<NewToOldID[*it]<<") ";
//        }
//    }
//    cout<<endl;
    return make_pair(LCC.first.size(),LCC.second);
}

double ComputeDiameter(vector<pair<double,double>> & points){
    double maxDis=0;
    for(int i=0;i<points.size();++i){
        for(int j=i+1;j<points.size();++j){
            double temp = EuclideanDis(points[i],points[j]);
            if(temp>maxDis) maxDis=temp;
        }

    }
    return maxDis;
}

//function of computing Euclidean distance
//double EuclideanDis(pair<double,double> s, pair<double,double> t)
//{
//    double d=111.319;//distance in equator per degree, kilometer
//    double coe;
//    double temp=(s.second+t.second)/2;
//    temp=temp*3.1415926/180;
////    temp=temp*3.1415926/(180*1000000);
//    coe=cos(temp);
//    double y1 = s.second * d, y2 = t.second * d;
//    double x1 = s.first * d * coe, x2 = t.first * d * coe;
//    double xx = x1 - x2, yy = y1 - y2;
//    return sqrt(xx * xx + yy * yy);//Euclidean Distance in meter
//}
////correct version, credit from https://stackoverflow.com/questions/27928/calculate-distance-between-two-latitude-longitude-points-haversine-formula
double EuclideanDis(pair<double,double> s, pair<double,double> t){
    int R=6371;//km
    double p=3.1415926/180;
    double lon1=s.first;
    double lat1=s.second;
    double lon2=t.first;
    double lat2=t.second;
    double a=0.5-cos((lat2-lat1)*p)/2+cos(lat1*p)*cos(lat2*p)*(1-cos((lon2-lon1)*p))/2;
    double d=2000*R*asin(sqrt(a));//in meter
    return d;
}

vector<string> split(const string &s, const string &seperator)
{
	vector<string> result;
	typedef string::size_type string_size;
	string_size i = 0;
		    
	while(i != s.size())
	{
		int flag = 0;
		while(i != s.size() && flag == 0)
		{
			flag = 1;
			for(string_size x = 0; x < seperator.size(); ++x)
			if(s[i] == seperator[x])
			{
				++i;
				flag = 0;
				break;
			}
		}
						    
		flag = 0;
		string_size j = i;
		while(j != s.size() && flag == 0)
		{
			for(string_size x = 0; x < seperator.size(); ++x)
				if(s[j] == seperator[x])
				{
					flag = 1;
					break;
				}
				if(flag == 0) 
					++j;
		}
									    
		if(i != j)
		{
			result.push_back(s.substr(i, j-i));
			i = j;														    
		}
										  
	}
			  
	return result;
}
