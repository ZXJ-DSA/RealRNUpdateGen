/*
 * trajectory.cpp
 * Function: to extract trajectory, update, query
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
#include <string>
#include <vector>
#include <chrono>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

using namespace std;

struct Timer
{
    std::chrono::high_resolution_clock::time_point t1, t2;//varibles for time record
    std::chrono::duration<double> time_span;
    void start()
    {
        t1 = std::chrono::high_resolution_clock::now();
    }
    void stop()
    {
        t2 = std::chrono::high_resolution_clock::now();
    }
    double GetRuntime()//return time in second
    {
        time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);//std::chrono::nanoseconds
        return time_span.count();
    }
};

vector<string> split(const string &s, const string &seperator);
void TrajectoryExtract(string sourceFile, string outputFile, vector<string>& sourceFiles);
void GetTargetTrajectory(vector<string> sourceFiles, string outputFile, pair<double,double> lonP, pair<double,double> latP);
void GetEdgeUpdates(string graphFile, string edgeNodeFile, string edgeIDFile, string trajectoryFile, string outputFile, unsigned long long int startT, unsigned long long int endT);
void GetStreamUpdatesAndQueriesLCC(string graphFile, string edgeNodeFile, string edgeIDFile, string nodeIDFile, string trajectoryFile, unsigned long long int startT, unsigned long long int endT);
void GetBatchUpdatesLCC(string graphFile, string edgeNodeFile, string edgeIDFile, string updateFile, string outputFile, int batchInterval, pair<unsigned long long int, unsigned long long int> timeRange);
void GetBatchUpdatesLCCs(string graphFile, string edgeNodeFile, string edgeIDFile, string updateFile, int batchInterval,  vector<int>& dayIDs, vector<pair<unsigned long long int, unsigned long long int>>& timeRanges);
double EuclideanDis(pair<double,double> s, pair<double,double> t);//return distance in meter
bool ifNew=false;

int main(int argc, char** argv)
{
    if( argc < 5 || argc > 10){
        printf("usage:\n<arg1> trajectory source path, e.g. /data/TrajectoryData/CennaviData/BasicTrajectory/m=01/\n");
        printf("<arg2> trajectory target path, e.g. /data/xzhouby/datasets/trajectoryData/m=01/\n");
        printf("<arg3> dataset, e.g. Guangdong\n");
        printf("<arg4> graph path, e.g. /data/xzhouby/datasets/map/Guangdong/Guangdong\n");
        printf("<arg5> minimum longitude (optional), e.g. 109.589\n");
        printf("<arg6> maximum longitude (optional), e.g. 117.25\n");
        printf("<arg7> minimum latitude (optional), e.g. 20.0833\n");
        printf("<arg8> maximum latitude (optional), e.g. 25.6667\n");
        printf("<arg9> process again (optional), 0: No, 1: Yes. default: 0\n");
        exit(0);
    }
    string sourcePath="/Users/zhouxj/Documents/1-Research/Datasets/NavInfo/trajectory/m=01/d=01/BASIS_TRAJECTORY_2016_";
    string graphFile="/Users/zhouxj/Documents/1-Research/Datasets/NavInfo/map/";
//    string traFile="/Users/zhouxj/Documents/1-Research/Datasets/NavInfo/trajectory/m=01/d=01/";
    vector<string> sourceFiles;
    string dataset;
//    string updatePath;
    double minLon=73.33;
    double maxLon=135.05;
    double minLat=3.51;
    double maxLat=53.33;

    sourcePath=argv[1];

    string targetPath=argv[2];
    dataset=argv[3];
    graphFile=argv[4];

    if(argc>5){
        minLon=stod(argv[5]);
        if(argc>6) {
            maxLon = stod(argv[6]);
        }
        if(argc>7) {
            minLat = stod(argv[7]);
        }
        if(argc>8) {
            maxLat = stod(argv[8]);
        }
        if(argc>9) {
            ifNew = stoi(argv[9]);
        }
    }

    /// Step 1: Get extracted trajectories
    cout<<"Step 1: extract valid trajectories (From Dec 1st 2015 to Feb 1st 2016, GMT+8)."<<endl;
    for(int di=1;di<=5;++di) {
        string dayPath = sourcePath+"d=0"+ to_string(di)+"/BASIS_TRAJECTORY_2016_";
        string dayPath2 = targetPath+"d=0"+ to_string(di)+"/BASIS_TRAJECTORY_2016_";
        for (int i = 0; i < 30; ++i) {
            string path1 = dayPath + to_string(i);
            TrajectoryExtract(path1, dayPath2 + to_string(i) + ".valid", sourceFiles);
//            cout << "File " << i << " done." << endl;
//        sourceFiles.push_back(targetPath + "/BASIS_TRAJECTORY_2016_" + to_string(i) + ".extract");
        }
//        cout<<endl;
    }
    cout<<endl;

    /// Step 2: Get target trajectories
    cout<<"Step 2: obtain target trajectories that have start point or end point locating at certain area."<<endl;
//    if(sourceFiles.empty()){
//        int di=5;
//        for(int i=0;i<30;++i){
//            sourceFiles.push_back(targetPath + "d=0"+to_string(di)+"/BASIS_TRAJECTORY_2016_" + to_string(i) + ".valid");
//        }
//    }

//    GetTargetTrajectory(sourceFiles, targetPath + "/"+dataset+".trajectory", make_pair(109.5,117.25), make_pair(20.0833,25.6667));//Guangdong
    cout<<"Extracted trajectory file number: "<<sourceFiles.size()<<endl;

    GetTargetTrajectory(sourceFiles, targetPath +dataset+".trajectory", make_pair(minLon,maxLon), make_pair(minLat,maxLat));



    /// Step 3: Get the queries and edge updates
    cout<<"\nStep 3: obtain the queries and edge updates on the LCC during a certain period."<<endl;
    unsigned long long int startT=1451577600;//2016-01-01 00:00
    unsigned long long int endT=1452009600;//2016-01-05 23:59
//    string updateOutput=graphFile;
//    GetEdgeUpdates(graphFile, graphFile+"_EdgeToNodeMap", graphFile+"_EdgeIDMap", targetPath+dataset+".trajectory", targetPath+dataset+".EdgeUpdates", startT, endT);
    GetStreamUpdatesAndQueriesLCC(graphFile, graphFile+"_EdgeToNodeMap", graphFile+"_EdgeIDMap", graphFile+".IDMap", targetPath+dataset+".trajectory", startT, endT);

    /// Step 4: Get final batch edge updates
    cout<<"\nStep 4: obtain the final batch updates of edges on the LCC."<<endl;
    string updateFile=graphFile+".edgeUpdates";
    vector<int> dayIDs;
    dayIDs.emplace_back(20160101); dayIDs.emplace_back(20160102); dayIDs.emplace_back(20160103); dayIDs.emplace_back(20160104);dayIDs.emplace_back(20160105);
    vector<pair<unsigned long long int, unsigned long long int>> dayIntervals;
    dayIntervals.emplace_back(1451577600,1451664000); dayIntervals.emplace_back(1451664000,1451750400);
    dayIntervals.emplace_back(1451750400,1451836800); dayIntervals.emplace_back(1451836800,1451923200);
    dayIntervals.emplace_back(1451923200,1452009600);
    vector<int> updateIntervals;
    updateIntervals.push_back(300); updateIntervals.push_back(900);
//    updateIntervals.push_back(120); updateIntervals.push_back(600);

    for(int j=0;j<updateIntervals.size();++j){
        int updateInterval=updateIntervals[j];
        cout<<"Update interval: "<<updateInterval<<" s."<<endl;
        GetBatchUpdatesLCCs(graphFile, graphFile+"_EdgeToNodeMap", graphFile+"_EdgeIDMap", updateFile, updateInterval, dayIDs, dayIntervals);
        cout<<endl;
    }


    cout<<"Done."<<endl;
    return 0;
}
//function of obtaining the final edge update batches
void GetBatchUpdatesLCC(string graphFile, string edgeNodeFile, string edgeIDFile, string updateFile, string outputFile, int batchInterval, pair<unsigned long long int, unsigned long long int> timeRange){
    Timer tt;
    tt.start();
    vector<string> vs;
    string line;
    map<long long int, pair<int, int>> EdgeToNode;//from old edge ID to new vertex ID
    vector<long long int> EdgeIDMapV;//map from new edge ID to old edge ID
    vector<map<unsigned long long int,int>> EdgeTrajectory;//new edge ID, time, edge weight
    vector<pair<int,int>> Edges;//new edge ID, distance, travel time
    vector<map<int,vector<int>>> batchUpdates;//batch updates, <time slot, <new edge ID, vector<edge weights>>
    vector<map<pair<int,int>,int>> batchUpdatesFinal;
    vector<map<int,int>> NeighborsMap;
    unsigned long long int time;
    int ID1,ID2,weightT;
    int node_num, edge_num;
    long long int edgeID;

    int slotNum=(timeRange.second-timeRange.first)/batchInterval;
    cout<<"Time slot number: "<<slotNum<<" ; time range: [ "<<timeRange.first<<" "<<timeRange.second<<" ]"<<endl;
    batchUpdates.assign(slotNum,map<int,vector<int>>());

    // Step 1: read edge ID map
    ifstream IF1(edgeIDFile, ios::in);
    if (!IF1.is_open()) {
        cout << "Open file failed!" << edgeIDFile << endl;
        exit(1);
    }
    getline(IF1,line);
    vs.clear();
    boost::split(vs,line,boost::is_any_of(" "));
    edge_num=stoi(vs[0]);
    cout<<"Edge number: "<<edge_num<<endl;
    EdgeIDMapV.assign(edge_num,-1);
    while(getline(IF1,line)){
        if(line.empty()) continue;
        istringstream iss(line);
        if (!(iss >> edgeID >> ID2)){
            cout<<"Wrong input syntax!"<<endl;
            exit(1);
        }
        if(ID2>=0 && ID2<edge_num){
            EdgeIDMapV[ID2]=edgeID;//from new edge ID to old edge ID
        }
        else{
            cout<<"Wrong edge ID "<<ID2<<endl; exit(1);
        }

    }
    IF1.close();

    // Step 2: read edge to node map
    Edges.assign(edge_num,pair<int,int>());
    EdgeTrajectory.assign(edge_num,map<unsigned long long int,int>());
    cout<<"Reading edge to node map..."<<endl;
    ifstream IF2(edgeNodeFile, ios::in);
    if (!IF2.is_open()) {
        cout << "Open file failed!" << edgeNodeFile << endl;
        exit(1);
    }
    getline(IF2,line);
    vs.clear();
    boost::split(vs,line,boost::is_any_of(" "));
    edge_num=stoi(vs[0]);
    while(getline(IF2,line)){
        if(line.empty()) continue;
        istringstream iss(line);
        if (!(iss >> edgeID >> ID1 >> ID2)){
            cout<<"Wrong input syntax!"<<endl;
            exit(1);
        }
        if(EdgeToNode.find(edgeID)==EdgeToNode.end()){//if not found
            EdgeToNode.insert({edgeID,make_pair(ID1,ID2)});//from old edge ID to new node ID 1
        }else{
            cout<<"Wrong. Already exist. "<<edgeID<<" "<<ID1<<" "<<ID2<<endl; exit(1);
        }
    }
    IF2.close();
    if(EdgeToNode.size() != edge_num){
        cout<<"Inconsistent edge number. "<<EdgeToNode.size() <<" "<< edge_num<<endl; exit(1);
    }

    // Step 3: read time graph of LCC
    ifstream IF3(graphFile+".time", ios::in);
    if (!IF3.is_open()) {
        cout << "Open file failed!" << graphFile+".time" << endl;
        exit(1);
    }
    getline(IF3,line);
    vs.clear();
    boost::split(vs,line,boost::is_any_of(" "));
    node_num=stoi(vs[0]), edge_num=stoi(vs[1]);
    cout<<"Node number: "<<node_num<<" , edge number: "<<edge_num<<endl;
    NeighborsMap.assign(node_num,map<int,int>());
    while(getline(IF3,line)){
        if(line.empty()) continue;
        istringstream iss(line);
        if (!(iss >> ID1 >> ID2 >> weightT)){
            cout<<"Wrong input syntax!"<<endl;
            exit(1);
        }

        if(ID1>=0 && ID1<node_num && ID2>=0 && ID2<node_num && weightT>0){
            NeighborsMap[ID1].insert({ID2,weightT});
        }else{
            cout<<"Graph data is wrong! "<<ID1<<" "<<ID2<<" "<<weightT<<endl;
        }
    }
    IF3.close();

    // Step 4: read node ID map of LCC
    ifstream IF4(graphFile+".IDMap", ios::in);
    if (!IF4.is_open()) {
        cout << "Open file failed!" << graphFile+".IDMap" << endl;
        exit(1);
    }
    getline(IF4,line);
    vs.clear();
    boost::split(vs,line,boost::is_any_of(" "));
    node_num=stoi(vs[0]);
    cout<<"Node number: "<<node_num<<endl;
    map<int,int> oldToNewNodeID;
    while(getline(IF4,line)){
        if(line.empty()) continue;
        istringstream iss(line);
        if (!(iss >> ID1 >> ID2)){
            cout<<"Wrong input syntax!"<<endl;
            exit(1);
        }

        if(ID2>=0 && ID2<node_num){
            oldToNewNodeID.insert({ID1,ID2});
        }else{
            cout<<"Graph data is wrong! "<<ID1<<" "<<ID2<<endl;
        }
    }
    IF4.close();

    // Step 5: read edge updates
    ifstream IF(updateFile, ios::in);
    if (!IF.is_open()) {
        cout << "Open file failed!" << updateFile << endl;
        exit(1);
    }
    cout<<"Update File "<<updateFile<<endl;
    getline(IF,line);
    vs.clear();
    boost::split(vs,line,boost::is_any_of(" "));
    int edgeNum= stoi(vs[0]);
//        EdgeTrajectory.assign(edgeNum,map<unsigned long long int,int>());
    for(int i=0;i<edgeNum;++i){
        getline(IF,line);

        vs.clear();
        boost::split(vs,line,boost::is_any_of(" "));
        int temp=stoi(vs[2]);//edge update number
//            cout<<line<<endl;
//            cout<<temp<<endl;
        vector<pair<int,unsigned long long int>> weights;
        for(int j=0;j<temp;++j){
            time = stoull(vs[2*j+3]);
            weightT = stoi(vs[2*j+4]);
            weights.emplace_back(weightT, time);
        }
        sort(weights.begin(),weights.end());
        for(int j=0;j<weights.size();++j){//arrange the updates to different time slots
            time = weights[j].second;
            weightT = weights[j].first;
            double ratio=j+1;
            ratio/=temp;
            if(ratio > 0.95 && temp>3){//remove the top-5% slowest trajectory
//                    cout<<j<<" "<<temp<<" "<<ratio<<endl;
                break;
            }
            int slotID=(time - timeRange.first)/batchInterval;
            if(slotID>=0 && slotID<slotNum){
                if(batchUpdates[slotID].find(i)==batchUpdates[slotID].end()){//if not found
                    batchUpdates[slotID].insert({i,vector<int>()});
                }
                batchUpdates[slotID][i].push_back(weightT);
            }
        }
    }
    IF.close();


    // Step 6: obtain valid updates: remove invalid edges and compute the average edge weights
    batchUpdatesFinal.assign(slotNum,map<pair<int,int>,int>());
    unsigned long long int updateNum=0;
    int maxUpdateNum=0;
    int minUpdateNum=INT32_MAX;
    map<pair<int,int>,int> existingUpdates;
    for(int i=0;i<batchUpdates.size();++i){
        for(auto it=batchUpdates[i].begin();it!=batchUpdates[i].end();++it){
            int eID=it->first;//new edge ID
            edgeID=EdgeIDMapV[eID];//old edge ID
            if(EdgeToNode.find(edgeID)!=EdgeToNode.end()){//if found
                ID1=EdgeToNode[edgeID].first, ID2=EdgeToNode[edgeID].second;//new node ID 1
                if(oldToNewNodeID.find(ID1)!=oldToNewNodeID.end() && oldToNewNodeID.find(ID2)!=oldToNewNodeID.end()){
                    ID1=oldToNewNodeID[ID1], ID2=oldToNewNodeID[ID2];//new node ID 2
                    if(ID1>ID2){
                        int temp=ID1;
                        ID1=ID2, ID2=temp;
                    }
                    if(NeighborsMap[ID1].find(ID2)!=NeighborsMap[ID1].end()){//if found
                        //compute average edge weight
                        int tempSum=0;
                        for(auto it2=it->second.begin();it2!=it->second.end();++it2){
                            tempSum+=*it2;
                        }
                        tempSum=ceil(tempSum/it->second.size());
                        if(tempSum<1){
                            tempSum=1;
                            cout<<"Smaller than 1 "<<tempSum<<endl;
                        }

                        if(existingUpdates.find(make_pair(ID1,ID2))!=existingUpdates.end()){//if found
                            int oldW=existingUpdates[make_pair(ID1,ID2)];
                            double tempChange=tempSum-oldW;
                            if(tempChange>0){
                                if(tempChange > 20 || tempChange/oldW > 1){//if the time change is larger than 20 seconds or the edge increase ratio is larger than 100%
                                    batchUpdatesFinal[i].insert({make_pair(ID1,ID2),tempSum});
                                    existingUpdates[make_pair(ID1,ID2)]=tempSum;
                                }
                            }else if(tempChange<0){
                                if(tempChange > 20 || -tempChange/oldW > 0.5){//if the time change is larger than 20 seconds or the edge decrease ratio is larger than 50%
                                    batchUpdatesFinal[i].insert({make_pair(ID1,ID2),tempSum});
                                    existingUpdates[make_pair(ID1,ID2)]=tempSum;
                                }
                            }

                        }else{//if not found
                            int oldW=NeighborsMap[ID1][ID2];
                            double tempChange=tempSum-oldW;
                            if(tempChange>0){
                                if(tempChange > 20 || tempChange/oldW > 1){//if the time change is larger than 20 seconds or the edge increase ratio is larger than 100%
                                    batchUpdatesFinal[i].insert({make_pair(ID1,ID2),tempSum});
                                    existingUpdates[make_pair(ID1,ID2)]=tempSum;
                                }
                            }else if(tempChange<0){
                                if(tempChange > 20 || -tempChange/oldW > 0.5){//if the time change is larger than 20 seconds or the edge decrease ratio is larger than 50%
                                    batchUpdatesFinal[i].insert({make_pair(ID1,ID2),tempSum});
                                    existingUpdates[make_pair(ID1,ID2)]=tempSum;
                                }
                            }
                        }

                    }
                }

            }
            else{
                cout<<"Wrong. No edge "<<edgeID<<endl; exit(1);
            }

        }


        updateNum+=batchUpdatesFinal[i].size();
        if(maxUpdateNum<batchUpdatesFinal[i].size()){
            maxUpdateNum=batchUpdatesFinal[i].size();
        }
        if(minUpdateNum>batchUpdatesFinal[i].size()){
            minUpdateNum=batchUpdatesFinal[i].size();
        }
    }
    cout<<"Total update number: "<<updateNum<<" ; average update number: "<<updateNum/slotNum<<" ; maximum update number: "<<maxUpdateNum<<" ; minimum update number: "<<minUpdateNum<<endl;
    ofstream OF1(outputFile+"Info", ios::out);
    if (!OF1.is_open()) {
        cout << "Open file failed!" << outputFile+"Info" << endl;
        exit(1);
    }
    OF1<<slotNum<<endl;
    for(int i=0;i<batchUpdatesFinal.size();++i){
        OF1<<i<<" "<<batchUpdatesFinal[i].size()<<endl;
    }
    OF1.close();

    ofstream OF(outputFile, ios::out);
    if (!OF.is_open()) {
        cout << "Open file failed!" << outputFile << endl;
        exit(1);
    }

    OF<<slotNum<<" "<<batchInterval<<endl;
    for(int i=0;i<batchUpdatesFinal.size();++i){
        OF<<batchUpdatesFinal[i].size();
        for(auto it=batchUpdatesFinal[i].begin();it!=batchUpdatesFinal[i].end();++it){
            ID1=it->first.first, ID2=it->first.second, weightT=it->second;
            OF<<" "<<ID1<<" "<<ID2<<" "<<weightT;

        }
        OF<<endl;
    }
    OF.close();

    tt.stop();
    cout<<"CPU elapsed time: "<<tt.GetRuntime()<<" s."<<endl;
}

void GetBatchUpdatesLCCs(string graphFile, string edgeNodeFile, string edgeIDFile, string updateFile, int batchInterval, vector<int>& dayIDs, vector<pair<unsigned long long int, unsigned long long int>>& timeRanges){

    Timer tt;
    tt.start();
    vector<string> vs;
    string line;
    map<long long int, pair<int, int>> EdgeToNode;//from old edge ID to new vertex ID
    vector<long long int> EdgeIDMapV;//map from new edge ID to old edge ID
    vector<map<unsigned long long int,int>> EdgeTrajectory;//new edge ID, time, edge weight
    vector<pair<int,int>> Edges;//new edge ID, distance, travel time
    vector<vector<pair<int,unsigned long long int>>> EdgeUpdates;//new edge ID, edge weight, time stamp
    vector<map<int,vector<int>>> batchUpdates;//batch updates, <time slot, <new edge ID, vector<edge weights>>
    vector<map<pair<int,int>,int>> batchUpdatesFinal;
    vector<map<int,int>> NeighborsMap;
    unsigned long long int time;
    int ID1,ID2,weightT;
    int node_num, edge_num;
    long long int edgeID;



    /// Step 1: read edge ID map
    ifstream IF1(edgeIDFile, ios::in);
    if (!IF1.is_open()) {
        cout << "Open file failed!" << edgeIDFile << endl;
        exit(1);
    }
    getline(IF1,line);
    vs.clear();
    boost::split(vs,line,boost::is_any_of(" "));
    edge_num=stoi(vs[0]);
    cout<<"Edge number: "<<edge_num<<endl;
    EdgeIDMapV.assign(edge_num,-1);
    while(getline(IF1,line)){
        if(line.empty()) continue;
        istringstream iss(line);
        if (!(iss >> edgeID >> ID2)){
            cout<<"Wrong input syntax!"<<endl;
            exit(1);
        }
        if(ID2>=0 && ID2<edge_num){
            EdgeIDMapV[ID2]=edgeID;//from new edge ID to old edge ID
        }
        else{
            cout<<"Wrong edge ID "<<ID2<<endl; exit(1);
        }

    }
    IF1.close();

    /// Step 2: read edge to node map
    Edges.assign(edge_num,pair<int,int>());
    EdgeTrajectory.assign(edge_num,map<unsigned long long int,int>());
    cout<<"Reading edge to node map..."<<endl;
    ifstream IF2(edgeNodeFile, ios::in);
    if (!IF2.is_open()) {
        cout << "Open file failed!" << edgeNodeFile << endl;
        exit(1);
    }
    getline(IF2,line);
    vs.clear();
    boost::split(vs,line,boost::is_any_of(" "));
    edge_num=stoi(vs[0]);
    while(getline(IF2,line)){
        if(line.empty()) continue;
        istringstream iss(line);
        if (!(iss >> edgeID >> ID1 >> ID2)){
            cout<<"Wrong input syntax!"<<endl;
            exit(1);
        }
        if(EdgeToNode.find(edgeID)==EdgeToNode.end()){//if not found
            EdgeToNode.insert({edgeID,make_pair(ID1,ID2)});//from old edge ID to new node ID 1
        }else{
            cout<<"Wrong. Already exist. "<<edgeID<<" "<<ID1<<" "<<ID2<<endl; exit(1);
        }
    }
    IF2.close();
    if(EdgeToNode.size() != edge_num){
        cout<<"Inconsistent edge number. "<<EdgeToNode.size() <<" "<< edge_num<<endl; exit(1);
    }

    /// Step 3: read time graph of LCC
    ifstream IF3(graphFile+".time", ios::in);
    if (!IF3.is_open()) {
        cout << "Open file failed!" << graphFile+".time" << endl;
        exit(1);
    }
    getline(IF3,line);
    vs.clear();
    boost::split(vs,line,boost::is_any_of(" "));
    node_num=stoi(vs[0]), edge_num=stoi(vs[1]);
    cout<<"Node number: "<<node_num<<" , edge number: "<<edge_num<<endl;
    NeighborsMap.assign(node_num,map<int,int>());
    while(getline(IF3,line)){
        if(line.empty()) continue;
        istringstream iss(line);
        if (!(iss >> ID1 >> ID2 >> weightT)){
            cout<<"Wrong input syntax!"<<endl;
            exit(1);
        }

        if(ID1>=0 && ID1<node_num && ID2>=0 && ID2<node_num && weightT>0){
            NeighborsMap[ID1].insert({ID2,weightT});
        }else{
            cout<<"Graph data is wrong! "<<ID1<<" "<<ID2<<" "<<weightT<<endl;
        }
    }
    IF3.close();

    /// Step 4: read node ID map of LCC
    ifstream IF4(graphFile+".IDMap", ios::in);
    if (!IF4.is_open()) {
        cout << "Open file failed!" << graphFile+".IDMap" << endl;
        exit(1);
    }
    getline(IF4,line);
    vs.clear();
    boost::split(vs,line,boost::is_any_of(" "));
    node_num=stoi(vs[0]);
    cout<<"Node number: "<<node_num<<endl;
    map<int,int> oldToNewNodeID;
    while(getline(IF4,line)){
        if(line.empty()) continue;
        istringstream iss(line);
        if (!(iss >> ID1 >> ID2)){
            cout<<"Wrong input syntax!"<<endl;
            exit(1);
        }

        if(ID2>=0 && ID2<node_num){
            oldToNewNodeID.insert({ID1,ID2});
        }else{
            cout<<"Graph data is wrong! "<<ID1<<" "<<ID2<<endl;
        }
    }
    IF4.close();



    /// Step 5: read edge updates
    ifstream IF(updateFile, ios::in);
    if (!IF.is_open()) {
        cout << "Open file failed!" << updateFile << endl;
        exit(1);
    }
    cout<<"Update File "<<updateFile<<endl;
    getline(IF,line);
    getline(IF,line);
    vs.clear();
    boost::split(vs,line,boost::is_any_of(" "));
    int edgeNum= stoi(vs[0]);
//    edgeNum=edge_num;
    EdgeUpdates.assign(edgeNum,vector<pair<int,unsigned long long int>>());

    for(int i=0;i<edgeNum;++i){
        getline(IF,line);

        vs.clear();
        boost::split(vs,line,boost::is_any_of(" "));
        int euNum=stoi(vs[4]);//edge update number
//            cout<<line<<endl;
//            cout<<temp<<endl;
        vector<pair<int,unsigned long long int>> weights;
        for(int j=0;j<euNum;++j){
            time = stoull(vs[2*j+5]);
            weightT = stoi(vs[2*j+6]);
            weights.emplace_back(weightT, time);
        }
        sort(weights.begin(),weights.end());

        for(int j=0;j<weights.size();++j){//arrange the updates to different time slots
            time = weights[j].second;
            weightT = weights[j].first;
            double ratio=j+1;
            ratio/=euNum;
            if(ratio > 0.95 && euNum>3){//remove the top-5% slowest trajectory
//                    cout<<j<<" "<<temp<<" "<<ratio<<endl;
                continue;
            }

            EdgeUpdates[i].emplace_back(weightT,time);

        }
    }
    IF.close();




    /// Step 6: obtain valid updates: remove invalid edges and compute the average edge weights

    for(int di=0;di<timeRanges.size();++di){
        cout<<"Date: "<<dayIDs[di]<<endl;
        string outputFile=graphFile+"_"+ to_string(dayIDs[di])+"_"+to_string(batchInterval)+".batchUpdates";
        ifstream IF1(outputFile+"Info", ios::out);
        if (IF1.is_open() && !ifNew) {//if open
            cout << "File already exist." << endl;
            IF1.close();
        }else{
            IF1.close();
            pair<unsigned long long int, unsigned long long int> timeRange=timeRanges[di];
            int slotNum=(timeRange.second-timeRange.first)/batchInterval;
            cout<<"Time slot number: "<<slotNum<<" ; time range: [ "<<timeRange.first<<" "<<timeRange.second<<" ]"<<endl;
            batchUpdates.assign(slotNum,map<int,vector<int>>());
            for(int i=0;i<edgeNum;++i){
                for(int j=0;j<EdgeUpdates[i].size();++j){//arrange the updates to different time slots
                    time = EdgeUpdates[i][j].second;
                    weightT = EdgeUpdates[i][j].first;

                    int slotID=(time - timeRange.first)/batchInterval;
                    if(slotID>=0 && slotID<slotNum){
                        if(batchUpdates[slotID].find(i)==batchUpdates[slotID].end()){//if not found
                            batchUpdates[slotID].insert({i,vector<int>()});
                        }
                        batchUpdates[slotID][i].push_back(weightT);
                    }
                }
            }

            batchUpdatesFinal.assign(slotNum,map<pair<int,int>,int>());
            unsigned long long int updateNum=0;
            int maxUpdateNum=0;
            int minUpdateNum=INT32_MAX;
            map<pair<int,int>,int> existingUpdates;
            for(int i=0;i<batchUpdates.size();++i){
                for(auto it=batchUpdates[i].begin();it!=batchUpdates[i].end();++it){
                    int eID=it->first;//new edge ID
                    edgeID=EdgeIDMapV[eID];//old edge ID
                    if(EdgeToNode.find(edgeID)!=EdgeToNode.end()){//if found
                        ID1=EdgeToNode[edgeID].first, ID2=EdgeToNode[edgeID].second;//new node ID 1
                        if(oldToNewNodeID.find(ID1)!=oldToNewNodeID.end() && oldToNewNodeID.find(ID2)!=oldToNewNodeID.end()){
                            ID1=oldToNewNodeID[ID1], ID2=oldToNewNodeID[ID2];//new node ID 2
                            if(ID1>ID2){
                                int temp=ID1;
                                ID1=ID2, ID2=temp;
                            }
                            if(NeighborsMap[ID1].find(ID2)!=NeighborsMap[ID1].end()){//if found
                                //compute average edge weight
                                int tempSum=0;
                                for(auto it2=it->second.begin();it2!=it->second.end();++it2){
                                    tempSum+=*it2;
                                }
                                tempSum=ceil(tempSum/it->second.size());
                                if(tempSum<1){
                                    tempSum=1;
                                    cout<<"Smaller than 1 "<<tempSum<<endl;
                                }

                                if(existingUpdates.find(make_pair(ID1,ID2))!=existingUpdates.end()){//if found
                                    int oldW=existingUpdates[make_pair(ID1,ID2)];
                                    double tempChange=tempSum-oldW;
                                    if(tempChange>0){
                                        if(tempChange > 20 || tempChange/oldW > 1){//if the time change is larger than 20 seconds or the edge increase ratio is larger than 100%
                                            batchUpdatesFinal[i].insert({make_pair(ID1,ID2),tempSum});
                                            existingUpdates[make_pair(ID1,ID2)]=tempSum;
                                        }
                                    }else if(tempChange<0){
                                        if(tempChange > 20 || -tempChange/oldW > 0.5){//if the time change is larger than 20 seconds or the edge decrease ratio is larger than 50%
                                            batchUpdatesFinal[i].insert({make_pair(ID1,ID2),tempSum});
                                            existingUpdates[make_pair(ID1,ID2)]=tempSum;
                                        }
                                    }

                                }else{//if not found
                                    int oldW=NeighborsMap[ID1][ID2];
                                    double tempChange=tempSum-oldW;
                                    if(tempChange>0){
                                        if(tempChange > 20 || tempChange/oldW > 1){//if the time change is larger than 20 seconds or the edge increase ratio is larger than 100%
                                            batchUpdatesFinal[i].insert({make_pair(ID1,ID2),tempSum});
                                            existingUpdates[make_pair(ID1,ID2)]=tempSum;
                                        }
                                    }else if(tempChange<0){
                                        if(tempChange > 20 || -tempChange/oldW > 0.5){//if the time change is larger than 20 seconds or the edge decrease ratio is larger than 50%
                                            batchUpdatesFinal[i].insert({make_pair(ID1,ID2),tempSum});
                                            existingUpdates[make_pair(ID1,ID2)]=tempSum;
                                        }
                                    }
                                }

                            }
                        }

                    }
                    else{
                        cout<<"Wrong. No edge "<<edgeID<<endl; exit(1);
                    }
                }

                updateNum+=batchUpdatesFinal[i].size();
                if(maxUpdateNum<batchUpdatesFinal[i].size()){
                    maxUpdateNum=batchUpdatesFinal[i].size();
                }
                if(minUpdateNum>batchUpdatesFinal[i].size()){
                    minUpdateNum=batchUpdatesFinal[i].size();
                }
            }
            cout<<"Total update number: "<<updateNum<<" ; average update number: "<<updateNum/slotNum<<" ; maximum update number: "<<maxUpdateNum<<" ; minimum update number: "<<minUpdateNum<<endl;

            string outputFile=graphFile+"_"+ to_string(dayIDs[di])+"_"+to_string(batchInterval)+".batchUpdates";
            ofstream OF1(outputFile+"Info", ios::out);
            if (!OF1.is_open()) {
                cout << "Open file failed!" << outputFile+"Info" << endl;
                exit(1);
            }
            OF1<<slotNum<<endl;
            for(int i=0;i<batchUpdatesFinal.size();++i){
                OF1<<i<<" "<<batchUpdatesFinal[i].size()<<endl;
            }
            OF1.close();

            ofstream OF(outputFile, ios::out);
            if (!OF.is_open()) {
                cout << "Open file failed!" << outputFile << endl;
                exit(1);
            }

            OF<<slotNum<<" "<<batchInterval<<endl;
            for(int i=0;i<batchUpdatesFinal.size();++i){
                OF<<batchUpdatesFinal[i].size();
                for(auto it=batchUpdatesFinal[i].begin();it!=batchUpdatesFinal[i].end();++it){
                    ID1=it->first.first, ID2=it->first.second, weightT=it->second;
                    OF<<" "<<ID1<<" "<<ID2<<" "<<weightT;

                }
                OF<<endl;
            }
            OF.close();
        }
    }

    tt.stop();
    cout<<"CPU elapsed time: "<<tt.GetRuntime()<<" s."<<endl;
}

//function of obtaining the edge updates of new edgeID
void GetEdgeUpdates(string graphFile, string edgeNodeFile, string edgeIDFile, string trajectoryFile, string outputFile, unsigned long long int startT, unsigned long long int endT){
    ifstream IFOut(outputFile);
    if (IFOut.is_open()) {//if exist
        cout << "File " << outputFile << " already exist."<< endl;
        IFOut.close();
    }
    else{
        IFOut.close();
        Timer tt;
        tt.start();
        vector<string> vs;
        string line;

        map<pair<int, int>, long long int> NodeToEdge;//from vertex ID to old edge ID
        map<long long int, int> EdgeIDMap;//map from old edge ID to new edge ID
//        map<int, int> NodeIDMap;//map from old vertex ID to new vertex ID
        vector<map<unsigned long long int,int>> EdgeTrajectory;//new edge ID, time, edge weight
        vector<pair<int,int>> Edges;//new edge ID, distance, travel time

        vector<string> carID;
        vector<int> carType;//1: private car; 2: taxi; 0: others
        vector<vector<long long int>> trajectory;
        vector<vector<unsigned long long int>> trajectoryTime;
        vector<vector<int>> trajectorySpeed;
        vector<vector<int>> trajectoryCity;
        vector<int> linkNum;
        vector<int> travelDis, travelTime, travelSpeed;
        vector<pair<double,double>> startPoint;//gps
        vector<pair<double,double>> endPoint;//gps
        vector<unsigned long long int> startTime, endTime;
        int lineNum = 0;
        int tNum=0;
        vector<unsigned long long int> traTemp;
        vector<int> traTemp2;
        vector<long long int> traTemp3;

        double lon, lat;
        int node_num, edge_num;
        int ID1, ID2, weightD, weightT;
        long long int edgeID;
        int edgeIDNew;
        unsigned long long int timeStart;

        cout<<"Reading edge ID map..."<<endl;
        // edge ID map
        ifstream IF(edgeIDFile, ios::in);
        if (!IF.is_open()) {
            cout << "Open file failed!" << edgeIDFile << endl;
            exit(1);
        }
        getline(IF,line);
        vs.clear();
        boost::split(vs,line,boost::is_any_of(" "));
        edge_num=stoi(vs[0]);
        while(getline(IF,line)){
            if(line.empty()) continue;
            istringstream iss(line);
            if (!(iss >> edgeID >> ID2)){
                cout<<"Wrong input syntax!"<<endl;
                exit(1);
            }
            if(EdgeIDMap.find(edgeID)==EdgeIDMap.end()){
                EdgeIDMap.insert({edgeID,ID2});
            }else{
                cout<<"Wrong. Already exist. "<<edgeID<<endl; exit(1);
            }
        }
        IF.close();
        if(EdgeIDMap.size() != edge_num){
            cout<<"Inconsistent edge number. "<<EdgeIDMap.size() <<" "<< edge_num<<endl; exit(1);
        }
        Edges.assign(edge_num,pair<int,int>());
        EdgeTrajectory.assign(edge_num,map<unsigned long long int,int>());
        cout<<"Reading edge to node map..."<<endl;

        // edge to node map
        ifstream IF2(edgeNodeFile, ios::in);
        if (!IF2.is_open()) {
            cout << "Open file failed!" << edgeNodeFile << endl;
            exit(1);
        }
        getline(IF2,line);
        vs.clear();
        boost::split(vs,line,boost::is_any_of(" "));
        edge_num=stoi(vs[0]);
        while(getline(IF2,line)){
            if(line.empty()) continue;
            istringstream iss(line);
            if (!(iss >> edgeID >> ID1 >> ID2)){
                cout<<"Wrong input syntax!"<<endl;
                exit(1);
            }
            if(NodeToEdge.find(make_pair(ID1,ID2))==NodeToEdge.end()){//if not found
                NodeToEdge.insert({make_pair(ID1,ID2),edgeID});//from new node ID to old edge ID
            }else{
                cout<<"Wrong. Already exist. "<<ID1<<" "<<ID2<<endl; exit(1);
            }
        }
        IF2.close();
        if(NodeToEdge.size() != edge_num){
            cout<<"Inconsistent edge number. "<<NodeToEdge.size() <<" "<< edge_num<<endl; exit(1);
        }

        cout<<"Reading road network..."<<endl;
        // distance graph
        ifstream IF3(graphFile+"_Distance.gr", ios::in);
        if (!IF3.is_open()) {
            cout << "Open file failed!" << graphFile+"_Distance.gr" << endl;
            exit(1);
        }
        getline(IF3,line);
        vs.clear();
        boost::split(vs,line,boost::is_any_of(" "));
        node_num=stoi(vs[0]), edge_num=stoi(vs[1]);
        cout<<"Node number: "<<node_num<<" , edge number: "<<edge_num<<endl;
        while(getline(IF3,line)){
            if(line.empty()) continue;
            istringstream iss(line);
            if (!(iss >> ID1 >> ID2 >> weightD)){
                cout<<"Wrong input syntax!"<<endl;
                exit(1);
            }
//        if(ID1==23 && ID2==22){
//            cout<<"here. "<<weightD<<endl;
//        }
            if(ID1>=0 && ID1<node_num && ID2>=0 && ID2<node_num && weightD>0){
                if(NodeToEdge.find(make_pair(ID1,ID2)) == NodeToEdge.end()){//if not found
                    if(NodeToEdge.find(make_pair(ID2,ID1)) != NodeToEdge.end()){//if found
                        edgeID = NodeToEdge[make_pair(ID2,ID1)];
                        if(EdgeIDMap.find(edgeID)!=EdgeIDMap.end()){//if found
                            Edges[EdgeIDMap[edgeID]].first=weightD;
                        }else{
                            cout<<"Not found edge ID. "<<edgeID<<endl; exit(1);
                        }

                    }else{
//                        cout<<"Edge not exists. "<<ID1<<" "<<ID2<<endl; exit(1);
                    }
                }else{//if found
                    edgeID = NodeToEdge[make_pair(ID1,ID2)];
                    if(EdgeIDMap.find(edgeID)!=EdgeIDMap.end()){//if found
                        Edges[EdgeIDMap[edgeID]].first=weightD;
                    }else{
                        cout<<"Not found edge ID. "<<edgeID<<endl; exit(1);
                    }
                }

            }else{
                cout<<"Graph data is wrong! "<<ID1<<" "<<ID2<<" "<<weightD<<endl;
            }
        }
        IF3.close();

        // time graph
        ifstream IF4(graphFile+"_Time.gr", ios::in);
        if (!IF4.is_open()) {
            cout << "Open file failed!" << graphFile+"_Time.gr" << endl;
            exit(1);
        }
        getline(IF4,line);
        vs.clear();
        boost::split(vs,line,boost::is_any_of(" "));
        node_num=stoi(vs[0]), edge_num=stoi(vs[1]);
        cout<<"Node number: "<<node_num<<" , edge number: "<<edge_num<<endl;
        while(getline(IF4,line)){
            if(line.empty()) continue;
            istringstream iss(line);
            if (!(iss >> ID1 >> ID2 >> weightT)){
                cout<<"Wrong input syntax!"<<endl;
                exit(1);
            }

            if(ID1>=0 && ID1<node_num && ID2>=0 && ID2<node_num && weightT>0){
                if(NodeToEdge.find(make_pair(ID1,ID2)) == NodeToEdge.end()){//if not found
                    if(NodeToEdge.find(make_pair(ID2,ID1)) != NodeToEdge.end()){//if found
                        edgeID = NodeToEdge[make_pair(ID2,ID1)];
                        if(EdgeIDMap.find(edgeID)!=EdgeIDMap.end()){//if found
                            Edges[EdgeIDMap[edgeID]].second=weightT;
                        }else{
                            cout<<"Not found edge ID. "<<edgeID<<endl; exit(1);
                        }
                    }else{
//                        cout<<"Edge not exists. "<<ID1<<" "<<ID2<<endl;
                    }
                }else{//if found
                    edgeID = NodeToEdge[make_pair(ID1,ID2)];
                    if(EdgeIDMap.find(edgeID)!=EdgeIDMap.end()){//if found
                        Edges[EdgeIDMap[edgeID]].second=weightT;
                    }else{
                        cout<<"Not found edge ID. "<<edgeID<<endl; exit(1);
                    }
                }

            }else{
                cout<<"Graph data is wrong! "<<ID1<<" "<<ID2<<" "<<weightT<<endl;
            }
        }
        IF4.close();

        for(int i=0;i<Edges.size();++i){
            if(Edges[i].first<1 && Edges[i].second!=0){
                cout<<"Wrong dis. "<<i<<" "<<Edges[i].first<<" "<<Edges[i].second<<endl;
                exit(1);
            }
        }

        // read trajectory
        cout<<"Reading trajectories..."<<endl;

        ifstream IF5(trajectoryFile, ios::in);
        if (!IF5.is_open()) {
            cout << "Open file failed!" << trajectoryFile << endl; exit(1);
        }

        getline(IF5, line);
        getline(IF5, line);
        vs.clear();
        boost::split(vs,line,boost::is_any_of(" "));
//        vs=split(line,",");//link ID of trajectory
        if (vs.size() != 1) {
            cout << "Wrong syntax! " <<vs.size() <<" : "<< line << endl;
            exit(1);
        }
        tNum= stoi(vs[0]);
        lineNum = 0;
        map<int,int> cityFreq;

        while (getline(IF5, line)) {
            if (line == "") continue;
            vs.clear();
            boost::split(vs,line,boost::is_any_of(" "));
//        vs=split(line,",");//link ID of trajectory
            if (vs.size() < 17) {
                cout << "Wrong syntax! " <<vs.size() <<" : "<< line << endl;
                exit(1);
            }
            if(stoll(vs[5])>endT || stoll(vs[6])<startT){
                continue;
            }

//        lon=stod(vs[7]), lat=stod(vs[8]);
//        lon=stod(vs[9]), lat=stod(vs[10]);
            lineNum++;

//        carID.emplace_back(vs[0]), carType.emplace_back(stoi(vs[1]));
//        travelDis.emplace_back(stoi(vs[2])), travelTime.emplace_back(stoi(vs[3])), travelSpeed.emplace_back(stoi(vs[4]));
            startTime.emplace_back(stoll(vs[5])), endTime.emplace_back(stoll(vs[6]));
//        startPoint.emplace_back(stod(vs[7]),stod(vs[8]));
//        endPoint.emplace_back(stod(vs[9]), stod(vs[10]));

            int tempInt;
            tempInt=stoi(vs[11]);
            linkNum.emplace_back(tempInt);

            int index_i=12;
            traTemp3.clear();
            for(int i=0;i<tempInt;++i){
                traTemp3.push_back(stoll(vs[index_i]));
                index_i++;
            }
            trajectory.emplace_back(traTemp3);


            tempInt=stoi(vs[index_i]);
            index_i++;
            traTemp.clear();
            for(int i=0;i<tempInt;++i){
                traTemp.push_back(stoull(vs[index_i]));
                index_i++;
            }
            trajectoryTime.emplace_back(traTemp);

            tempInt=stoi(vs[index_i]);
            index_i++;
            traTemp2.clear();
            for(int i=0;i<tempInt;++i){
                traTemp2.push_back(stoi(vs[index_i]));
                index_i++;
            }
            trajectorySpeed.emplace_back(traTemp2);

            tempInt=stoi(vs[index_i]);
            index_i++;
            traTemp2.clear();
            for(int i=0;i<tempInt;++i){
                int temp=stoi(vs[index_i]);
                traTemp2.push_back(temp);
                index_i++;
                if(cityFreq.find(temp)==cityFreq.end()){//if not found
                    cityFreq.insert({temp,1});
                }else{
                    cityFreq[temp]++;
                }
            }
            trajectoryCity.emplace_back(traTemp2);

//        cout<<tempInt<<":";
//        for(int i=0; i<tempInt; ++i){
//            cout<<" "<<traTemp2[i];
//        }
//        cout<<endl;


        }
        IF5.close();
        if(lineNum!=trajectory.size()){
            cout<<"Inconsistent trajectory number! "<<lineNum<<" "<<trajectory.size()<<" "<<tNum<<endl; exit(1);
        }

        cout << "Trajectory number: " << trajectory.size() <<" "<< trajectoryTime.size()<<" "<<trajectorySpeed.size()<<endl;
        cout<<"City number: "<<cityFreq.size()<<" . ";
        vector<pair<int,int>> cityFreqR;
        for(auto it=cityFreq.begin();it!=cityFreq.end();++it){
            cityFreqR.emplace_back(it->second,it->first);
//        cout<<it->first<<": "<<it->second<<" ; ";
        }
        sort(cityFreqR.begin(),cityFreqR.end());
        for(auto it=cityFreqR.begin();it!=cityFreqR.end();++it){
            cout<<it->second<<": "<<it->first<<" ; ";
        }
        cout<<endl;
        unsigned long long minTime=INT64_MAX, maxTime=0;
        for(int i=0;i<trajectory.size();++i){
            for(int j=0;j<trajectory[i].size();++j){
                edgeID=trajectory[i][j];
                if(edgeID<0){
                    edgeID=-edgeID;
                }
//            cout<<edgeID<<endl;
                if(EdgeIDMap.find(edgeID) != EdgeIDMap.end()){//if found
                    edgeIDNew=EdgeIDMap[edgeID];
                    timeStart=trajectoryTime[i][j];
                    if(timeStart<startT || timeStart>endT){
                        continue;
                    }
                    if(j==trajectory[i].size()-1){
                        weightT=endTime[i]-trajectoryTime[i][j];
                    }else{
                        weightT=trajectoryTime[i][j+1]-trajectoryTime[i][j];
                    }
                    if(weightT<1){
                        weightT=1;
                    }
                    int originalW=Edges[edgeIDNew].second;
                    if(weightT<0.83*originalW){//if exceeds the speed limit over 20%
                        continue;
                    }
//                double speedTemp=trajectorySpeed[i][j];//km/h to m/s
//                speedTemp=speedTemp/3.6;
//                if(speedTemp<=0){
//                    speedTemp=1;
//                }
                    weightD = Edges[edgeIDNew].first;
//                if(weightD<1){
//                    cout<<"wrong dis. "<<weightD<<endl;
//                }
//                weightT = ceil((double) weightD / speedTemp);
                    if(weightT<1){
                        cout<<"wrong time. "<<weightT<<" "<<weightD<<" "<<i<<" "<<j<<endl; exit(1);
                    }
                    EdgeTrajectory[edgeIDNew].insert({timeStart,weightT});
                    if(minTime>timeStart) minTime=timeStart;
                    if(maxTime<timeStart) maxTime=timeStart;
                }
                else{
//                cout<<"Not found!"<<endl; exit(1);
                }
            }
        }
        cout<<"Time range: [ "<<minTime<<" "<<maxTime<<" ] , equals to "<<(double)(maxTime-minTime)/(60*60)<<" hours."<<endl;

        ofstream OF(outputFile, ios::out);
        if (!OF.is_open()) {
            cout << "Open file failed!" << outputFile << endl;
            exit(1);
        }
        unsigned long long int edgeUpdateNum=0;
        OF<<EdgeTrajectory.size()<<endl;//edge number
        for(int i=0;i<EdgeTrajectory.size();++i){
            OF<<Edges[i].first<<" "<<Edges[i].second<<" " <<EdgeTrajectory[i].size();//original edge distance and time, edge update size
            edgeUpdateNum+=EdgeTrajectory[i].size();
            for(auto it=EdgeTrajectory[i].begin();it!=EdgeTrajectory[i].end();++it){
                OF<<" "<<it->first<<" "<<it->second;//update time and edge weight
            }
            OF<<endl;
        }
        OF.close();
        tt.stop();
        cout<<"Edge update number: "<<edgeUpdateNum<<endl;
        cout<<"CPU elapsed time: "<<tt.GetRuntime()<<" s."<<endl;
    }
}

void GetStreamUpdatesAndQueriesLCC(string graphFile, string edgeNodeFile, string edgeIDFile, string nodeIDFile, string trajectoryFile, unsigned long long int startT, unsigned long long int endT){
    ifstream IFOut(graphFile+".realQueries");
    if (IFOut.is_open() ) {//if exist && !ifNew
        cout << "File " << graphFile+".realQueries" << " already exist."<< endl;
        IFOut.close();
        ifstream IF(graphFile+".realQueries", ios::in);
        if (!IF.is_open() ) {
            cout << "Open file failed!" << graphFile+".realQueries" << endl;
            exit(1);
        }
        string line;
        vector<string> vs;
        vector<pair<long long int, long long int>> dayIntervals;
        dayIntervals.emplace_back(1451577600,1451664000); dayIntervals.emplace_back(1451664000,1451750400);
        dayIntervals.emplace_back(1451750400,1451836800); dayIntervals.emplace_back(1451836800,1451923200);
        dayIntervals.emplace_back(1451923200,1452009600);
        getline(IF, line);
        vs.clear();
        boost::split(vs,line,boost::is_any_of(" "));
//        vs=split(line,",");//link ID of trajectory
        if (vs.size() != 1) {
            cout << "Wrong syntax! " <<vs.size() <<" : "<< line << endl;
            exit(1);
        }
        int qNum=0;
        long long int timeStamp=0;
        cout<<"Total query number: "<<stoi(vs[0])<<endl;
        while (getline(IF, line)) {
            if (line == "") continue;
            vs.clear();
            boost::split(vs,line,boost::is_any_of(" "));
//        vs=split(line,",");//link ID of trajectory
            if (vs.size() != 5) {
                cout << "Wrong syntax! " <<vs.size() <<" : "<< line << endl;
                exit(1);
            }
            timeStamp=stoll(vs[0]);
            if(timeStamp >= dayIntervals[4].first && timeStamp < dayIntervals[4].second){
                qNum++;
            }
        }
        IF.close();
        cout<<"Query number of day 5: "<<qNum<<endl;
    }
    else{
        IFOut.close();
        Timer tt;
        tt.start();
        vector<string> vs;
        string line;

        map<pair<int, int>, long long int> NodeToEdge;//from LCC vertex ID to old edge ID
        map<long long int, pair<int,int>> EdgeToNodeMap;//map old edge ID to its endpoints' LCC vertex ID
        map<long long int, int> EdgeIDMap;//map from old edge ID to new edge ID
        map<int, long long int> EdgeIDMapToOld;//map from new edge ID to old edge ID
        map<int, int> NodeIDMap;//map from new vertex ID to LCC vertex ID, only new vertex number
        vector<map<unsigned long long int,int>> EdgeUpdates;//new edge ID (not LCC ID), time, edge weight
        map<long long int, vector<pair<pair<int,int>, int>>> StreamUpdates;// time stamp, <ID1, ID2, weight>
        vector<pair<int,int>> Edges;//new edge ID, distance, travel time of the LCC
        vector<pair<long long int,tuple<int,int,int,int>>> Queries;//store the queries generated by trajectories, <time stamp, <ID1,ID2,carType,trajectoryLength>>>
        vector<pair<double,double>> Coordinate;//coordinate of the LCC

        vector<string> carID;
        vector<int> carType;//1: private car; 2: taxi; 0: others
        vector<vector<long long int>> trajectory;
        vector<vector<unsigned long long int>> trajectoryTime;
        vector<vector<int>> trajectorySpeed;
        vector<vector<int>> trajectoryCity;
        vector<int> linkNum;
        vector<int> travelDis, travelTime, travelSpeed;
        vector<pair<double,double>> startPoint;//gps
        vector<pair<double,double>> endPoint;//gps
        vector<unsigned long long int> startTime, endTime;
        long long int startEdge, endEdge;

        int lineNum = 0;
        int tNum=0;
        vector<unsigned long long int> traTemp;
        vector<int> traTemp2;
        vector<long long int> traTemp3;

        double lon, lat;
        int node_num, edge_num;
        int edgeNumBefore;
        int ID1, ID2, weightD, weightT;
        long long int edgeID;
        int edgeIDNew;
        unsigned long long int timeStamp;

        cout<<"Reading edge ID map (from old edge ID to new edge ID)..."<<endl;
        /// edge ID map
        ifstream IF(edgeIDFile, ios::in);
        if (!IF.is_open()) {
            cout << "Open file failed!" << edgeIDFile << endl;
            exit(1);
        }
        getline(IF,line);
        vs.clear();
        boost::split(vs,line,boost::is_any_of(" "));
        edgeNumBefore=stoi(vs[0]);
        while(getline(IF,line)){
            if(line.empty()) continue;
            istringstream iss(line);
            if (!(iss >> edgeID >> ID2)){
                cout<<"Wrong input syntax!"<<endl;
                exit(1);
            }
            if(EdgeIDMap.find(edgeID)==EdgeIDMap.end()){
                EdgeIDMap.insert({edgeID,ID2});
            }else{
                cout<<"Wrong. Already exist. "<<edgeID<<endl; exit(1);
            }
            if(EdgeIDMapToOld.find(ID2)==EdgeIDMapToOld.end()){
                EdgeIDMapToOld.insert({ID2,edgeID});
            }else{
                cout<<"Wrong. Already exist. "<<ID2<<endl; exit(1);
            }
        }
        IF.close();
        if(EdgeIDMap.size() != edgeNumBefore){
            cout<<"Inconsistent edge number. "<<EdgeIDMap.size() <<" "<< edgeNumBefore<<endl; exit(1);
        }
        Edges.assign(edgeNumBefore,pair<int,int>());
        EdgeUpdates.assign(edgeNumBefore,map<unsigned long long int,int>());

        cout<<"Reading node id map (from new vertex ID to LCC vertex ID)..."<<endl;
        /// node id map
        ifstream IF1(nodeIDFile, ios::in);
        if (!IF1.is_open()) {
            cout << "Open file failed!" << nodeIDFile << endl;
            exit(1);
        }
        getline(IF1,line);
        vs.clear();
        boost::split(vs,line,boost::is_any_of(" "));
        node_num=stoi(vs[0]);
        while(getline(IF1,line)){
            if(line.empty()) continue;
            istringstream iss(line);
            if (!(iss >> ID1 >> ID2)){
                cout<<"Wrong input syntax!"<<endl;
                exit(1);
            }
            NodeIDMap.insert({ID1,ID2});//new vertex id to LCC vertex id
        }
        IF1.close();

        cout<<"Reading edge to node map (map from old edge ID to LCC vertex ID)..."<<endl;
        /// edge to node map
        ifstream IF2(edgeNodeFile, ios::in);
        if (!IF2.is_open()) {
            cout << "Open file failed!" << edgeNodeFile << endl;
            exit(1);
        }
        getline(IF2,line);
        vs.clear();
        boost::split(vs,line,boost::is_any_of(" "));
        if(edgeNumBefore!=stoi(vs[0])){
            cout<<"Inconsistent original edge number "<<edgeNumBefore<<" "<<stoi(vs[0])<<endl; exit(1);
        }
        while(getline(IF2,line)){
            if(line.empty()) continue;
            istringstream iss(line);
            if (!(iss >> edgeID >> ID1 >> ID2)){
                cout<<"Wrong input syntax!"<<endl;
                exit(1);
            }
            if(NodeIDMap.find(ID1)!=NodeIDMap.end() && NodeIDMap.find(ID2)!=NodeIDMap.end()){
                ID1=NodeIDMap[ID1], ID2=NodeIDMap[ID2];
                EdgeToNodeMap.insert({edgeID, make_pair(ID1,ID2)});//from original edge id to final vertex id
                if(NodeToEdge.find(make_pair(ID1,ID2))==NodeToEdge.end()){//if not found
                    NodeToEdge.insert({make_pair(ID1,ID2),edgeID});//from LCC node ID to old edge ID
                }else{
                    cout<<"Wrong. Already exist. "<<ID1<<" "<<ID2<<endl; exit(1);
                }
            }

        }
        IF2.close();

        cout<<"Reading road network..."<<endl;
        // distance graph
        ifstream IF3(graphFile+".dis", ios::in);
        if (!IF3.is_open()) {
            cout << "Open file failed!" << graphFile+".dis" << endl;
            exit(1);
        }
        getline(IF3,line);
        vs.clear();
        boost::split(vs,line,boost::is_any_of(" "));
        if(node_num!=stoi(vs[0])){
            cout<<"Inconsistent node number "<<node_num<<" "<<stoi(vs[0])<<endl; exit(1);
        }
        edge_num=stoi(vs[1]);
        cout<<"Node number: "<<node_num<<" , edge number: "<<edge_num<<endl;
        if(NodeToEdge.size()*2 != edge_num){
            cout<<"Inconsistent edge number. "<<NodeToEdge.size()*2 <<" "<< edge_num<<endl; exit(1);
        }
        while(getline(IF3,line)){
            if(line.empty()) continue;
            istringstream iss(line);
            if (!(iss >> ID1 >> ID2 >> weightD)){
                cout<<"Wrong input syntax!"<<endl;
                exit(1);
            }
//        if(ID1==23 && ID2==22){
//            cout<<"here. "<<weightD<<endl;
//        }
            if(ID1>=0 && ID1<node_num && ID2>=0 && ID2<node_num && weightD>0){
                if(NodeToEdge.find(make_pair(ID1,ID2)) == NodeToEdge.end()){//if not found
                    if(NodeToEdge.find(make_pair(ID2,ID1)) != NodeToEdge.end()){//if found
                        edgeID = NodeToEdge[make_pair(ID2,ID1)];
                        if(EdgeIDMap.find(edgeID)!=EdgeIDMap.end()){//if found
                            Edges[EdgeIDMap[edgeID]].first=weightD;
                        }else{
                            cout<<"Not found edge ID. "<<edgeID<<endl; exit(1);
                        }

                    }else{
                        cout<<"Edge not exists. "<<ID1<<" "<<ID2<<endl; exit(1);
                    }
                }else{//if found
                    edgeID = NodeToEdge[make_pair(ID1,ID2)];
                    if(EdgeIDMap.find(edgeID)!=EdgeIDMap.end()){//if found
                        Edges[EdgeIDMap[edgeID]].first=weightD;
                    }else{
                        cout<<"Not found edge ID. "<<edgeID<<endl; exit(1);
                    }
                }

            }else{
                cout<<"Graph data is wrong! "<<ID1<<" "<<ID2<<" "<<weightD<<endl;
            }
        }
        IF3.close();

        /// time graph
        ifstream IF4(graphFile+".time", ios::in);
        if (!IF4.is_open()) {
            cout << "Open file failed!" << graphFile+".time" << endl;
            exit(1);
        }
        getline(IF4,line);
        vs.clear();
        boost::split(vs,line,boost::is_any_of(" "));
        node_num=stoi(vs[0]), edge_num=stoi(vs[1]);
        cout<<"Node number: "<<node_num<<" , edge number: "<<edge_num<<endl;
        while(getline(IF4,line)){
            if(line.empty()) continue;
            istringstream iss(line);
            if (!(iss >> ID1 >> ID2 >> weightT)){
                cout<<"Wrong input syntax!"<<endl;
                exit(1);
            }

            if(ID1>=0 && ID1<node_num && ID2>=0 && ID2<node_num && weightT>0){
                if(NodeToEdge.find(make_pair(ID1,ID2)) == NodeToEdge.end()){//if not found
                    if(NodeToEdge.find(make_pair(ID2,ID1)) != NodeToEdge.end()){//if found
                        edgeID = NodeToEdge[make_pair(ID2,ID1)];
                        if(EdgeIDMap.find(edgeID)!=EdgeIDMap.end()){//if found
                            Edges[EdgeIDMap[edgeID]].second=weightT;
                        }else{
                            cout<<"Not found edge ID. "<<edgeID<<endl; exit(1);
                        }
                    }else{
                        cout<<"Edge not exists. "<<ID1<<" "<<ID2<<endl; exit(1);
                    }
                }else{//if found
                    edgeID = NodeToEdge[make_pair(ID1,ID2)];
                    if(EdgeIDMap.find(edgeID)!=EdgeIDMap.end()){//if found
                        Edges[EdgeIDMap[edgeID]].second=weightT;
                    }else{
                        cout<<"Not found edge ID. "<<edgeID<<endl; exit(1);
                    }
                }

            }else{
                cout<<"Graph data is wrong! "<<ID1<<" "<<ID2<<" "<<weightT<<endl;
            }
        }
        IF4.close();

        // time graph coordinates
        ifstream IF42(graphFile+".time.co", ios::in);
        if (!IF42.is_open()) {
            cout << "Open file failed!" << graphFile+".time.co" << endl;
            exit(1);
        }
        getline(IF42,line);
        vs.clear();
        boost::split(vs,line,boost::is_any_of(" "));
        node_num=stoi(vs[0]);
        cout<<"Node number: "<<node_num<<endl;
        int longitude, latitude;
        Coordinate.assign(node_num,pair<int,int>());
        while(getline(IF42,line)){
            if(line.empty()) continue;
            istringstream iss(line);
            if (!(iss >> ID1 >> longitude >> latitude)){
                cout<<"Wrong input syntax!"<<endl;
                exit(1);
            }

            if(ID1>=0 && ID1<node_num){
                Coordinate[ID1].first=longitude; Coordinate[ID1].first/=1000000;
                Coordinate[ID1].second=latitude; Coordinate[ID1].second/=1000000;
            }else{
                cout<<"Coordinate data is wrong! "<<ID1<<" "<<longitude<<" "<<latitude<<endl;
            }
        }
        IF42.close();

//        for(int i=0;i<Edges.size();++i){
//            if(Edges[i].first<1 && Edges[i].second!=0){
//                cout<<"Wrong dis. "<<i<<" "<<Edges[i].first<<" "<<Edges[i].second<<endl;
//                exit(1);
//            }
//        }

        // read trajectory
        cout<<"Reading trajectories..."<<endl;

        ifstream IF5(trajectoryFile, ios::in);
        if (!IF5.is_open()) {
            cout << "Open file failed!" << trajectoryFile << endl; exit(1);
        }

        getline(IF5, line);
        getline(IF5, line);
        vs.clear();
        boost::split(vs,line,boost::is_any_of(" "));
//        vs=split(line,",");//link ID of trajectory
        if (vs.size() != 1) {
            cout << "Wrong syntax! " <<vs.size() <<" : "<< line << endl;
            exit(1);
        }
        tNum= stoi(vs[0]);
        lineNum = 0;
        map<int,int> cityFreq;

        while (getline(IF5, line)) {
            if (line == "") continue;
            vs.clear();
            boost::split(vs,line,boost::is_any_of(" "));
//        vs=split(line,",");//link ID of trajectory
            if (vs.size() < 17) {
                cout << "Wrong syntax! " <<vs.size() <<" : "<< line << endl;
                exit(1);
            }
            if(stoll(vs[5])>endT || stoll(vs[6])<startT){
                continue;
            }

//        lon=stod(vs[7]), lat=stod(vs[8]);
//        lon=stod(vs[9]), lat=stod(vs[10]);
            lineNum++;

//        carID.emplace_back(vs[0]), carType.emplace_back(stoi(vs[1]));
//        travelDis.emplace_back(stoi(vs[2])), travelTime.emplace_back(stoi(vs[3])), travelSpeed.emplace_back(stoi(vs[4]));
            startTime.emplace_back(stoll(vs[5])), endTime.emplace_back(stoll(vs[6]));
            startPoint.emplace_back(stod(vs[7]),stod(vs[8]));
            endPoint.emplace_back(stod(vs[9]), stod(vs[10]));

            int tempInt;
            tempInt=stoi(vs[11]);
            linkNum.emplace_back(tempInt);

            int index_i=12;
            traTemp3.clear();
            for(int i=0;i<tempInt;++i){
                traTemp3.push_back(stoll(vs[index_i]));
                index_i++;
            }
            trajectory.emplace_back(traTemp3);
            startEdge=traTemp3[0];
            endEdge=traTemp3[traTemp3.size()-1];

            if(startEdge<0) startEdge=-startEdge;
            if(endEdge<0) endEdge=-endEdge;

            if(stoll(vs[5]) >= startT && stoll(vs[5]) < endT){
                if(EdgeToNodeMap.find(startEdge)!=EdgeToNodeMap.end() && EdgeToNodeMap.find(endEdge)!=EdgeToNodeMap.end()){//if found
                    ID1=EdgeToNodeMap[startEdge].first, ID2=EdgeToNodeMap[startEdge].second;//the vertex id of start edge
                    double dis1= EuclideanDis(Coordinate[ID1],startPoint[startPoint.size()-1]);
                    double dis2= EuclideanDis(Coordinate[ID2],startPoint[startPoint.size()-1]);
                    pair<int,int> query;

                    if(dis1<=dis2){//check which endpoint of start edge is closer to the start point
                        query.first=ID1;
                        if(dis1>100){
                            cout<<"seems wrong. "<<ID1<<" "<<dis2<<" "<<dis1<<endl; exit(1);
                        }
                    }else{
                        query.first=ID2;
                        if(dis2>100){
                            cout<<"seems wrong. "<<ID2<<" "<<dis1<<" "<<dis2<<endl; exit(1);
                        }
                    }
                    ID1=EdgeToNodeMap[endEdge].first, ID2=EdgeToNodeMap[endEdge].second;
                    dis1= EuclideanDis(Coordinate[ID1],endPoint[endPoint.size()-1]);
                    dis2= EuclideanDis(Coordinate[ID2],endPoint[endPoint.size()-1]);
                    if(dis1<=dis2){
                        query.second=ID1;
                        if(dis1>100){
                            cout<<"seems wrong. "<<ID1<<" "<<dis2<<" "<<dis1<<endl; exit(1);
                        }
                    }else{
                        query.second=ID2;
                        if(dis2>100){
                            cout<<"seems wrong. "<<ID2<<" "<<dis1<<" "<<dis2<<endl; exit(1);
                        }
                    }
                    Queries.emplace_back(stoll(vs[5]),make_tuple(query.first, query.second, stoi(vs[1]), stoi(vs[2])));//obtain queries
                }
            }

            tempInt=stoi(vs[index_i]);
            index_i++;
            traTemp.clear();
            for(int i=0;i<tempInt;++i){
                traTemp.push_back(stoull(vs[index_i]));
                index_i++;
            }
            trajectoryTime.emplace_back(traTemp);

            tempInt=stoi(vs[index_i]);
            index_i++;
            traTemp2.clear();
            for(int i=0;i<tempInt;++i){
                traTemp2.push_back(stoi(vs[index_i]));
                index_i++;
            }
            trajectorySpeed.emplace_back(traTemp2);

            tempInt=stoi(vs[index_i]);
            index_i++;
            traTemp2.clear();
            for(int i=0;i<tempInt;++i){
                int temp=stoi(vs[index_i]);
                traTemp2.push_back(temp);
                index_i++;
                if(cityFreq.find(temp)==cityFreq.end()){//if not found
                    cityFreq.insert({temp,1});
                }else{
                    cityFreq[temp]++;
                }
            }
            trajectoryCity.emplace_back(traTemp2);

//        cout<<tempInt<<":";
//        for(int i=0; i<tempInt; ++i){
//            cout<<" "<<traTemp2[i];
//        }
//        cout<<endl;


        }
        IF5.close();
        if(lineNum!=trajectory.size()){
            cout<<"Inconsistent trajectory number! "<<lineNum<<" "<<trajectory.size()<<" "<<tNum<<endl; exit(1);
        }

        cout << "Trajectory number: " << trajectory.size() <<" "<< trajectoryTime.size()<<" "<<trajectorySpeed.size()<<endl;
        cout << "Query number: "<<Queries.size()<<endl;
        cout<<"City number: "<<cityFreq.size()<<" . ";
        vector<pair<int,int>> cityFreqR;
        for(auto it=cityFreq.begin();it!=cityFreq.end();++it){
            cityFreqR.emplace_back(it->second,it->first);
//        cout<<it->first<<": "<<it->second<<" ; ";
        }
        sort(cityFreqR.begin(),cityFreqR.end());
        for(auto it=cityFreqR.begin();it!=cityFreqR.end();++it){
            cout<<it->second<<": "<<it->first<<" ; ";
        }
        cout<<endl;
        unsigned long long minTime=INT64_MAX, maxTime=0;
        map<pair<int,long long int>,int> residualEdgeInfo;//map from <edgeID, time stamp> to its count number
        for(int i=0;i<trajectory.size();++i){
            for(int j=0;j<trajectory[i].size();++j){
                edgeID=trajectory[i][j];
                if(edgeID<0){
                    edgeID=-edgeID;
                }
//            cout<<edgeID<<endl;
                if(EdgeToNodeMap.find(edgeID) != EdgeToNodeMap.end()) {//if found, means that edgeID is in LCC
                    if (EdgeIDMap.find(edgeID) != EdgeIDMap.end()) {//if found, old
                        edgeIDNew = EdgeIDMap[edgeID];
                        timeStamp = trajectoryTime[i][j];
                        if (timeStamp < startT || timeStamp > endT) {
                            continue;
                        }
                        if (j == trajectory[i].size() - 1) {
                            weightT = endTime[i] - trajectoryTime[i][j];
                        } else {
                            weightT = trajectoryTime[i][j + 1] - trajectoryTime[i][j];
                        }

                        int originalW = Edges[edgeIDNew].second;
                        double tempW = 0.83 * originalW;
                        if (weightT < tempW) {//if exceeds the speed limit over 20%
                            weightT = tempW;
                        }
                        weightD = Edges[edgeIDNew].first;
                        double minW = ceil((double) weightD / 33.33);//120km/h
                        if (weightT < minW) {
                            weightT = minW;
                        }
                        if (weightT < 1) {
                            if (EdgeToNodeMap.find(edgeID) != EdgeToNodeMap.end()) {//if found
                                cout << "wrong time. " << EdgeToNodeMap[edgeID].first << " "
                                     << EdgeToNodeMap[edgeID].second << " : " << weightT << " " << weightD << " " << i
                                     << " " << j << endl;
                                exit(1);
                            } else {
                                cout << "wrong time. not found edgeID " << edgeID << " : " << weightT << " " << weightD
                                     << " " << i << " " << j << endl;
                                exit(1);
                            }
                        }
                        if (EdgeUpdates[edgeIDNew].find(timeStamp) == EdgeUpdates[edgeIDNew].end()) {//if not found
                            EdgeUpdates[edgeIDNew].insert({timeStamp, weightT});
                        } else {//if found
                            if (residualEdgeInfo.find(make_pair(edgeIDNew, timeStamp)) ==
                                residualEdgeInfo.end()) {//if not found
                                residualEdgeInfo.insert({make_pair(edgeIDNew, timeStamp), 2});
                                EdgeUpdates[edgeIDNew][timeStamp] += weightT;
                                EdgeUpdates[edgeIDNew][timeStamp] /= 2;
                            } else {//if found
                                EdgeUpdates[edgeIDNew][timeStamp] *= residualEdgeInfo[make_pair(edgeIDNew, timeStamp)];
                                EdgeUpdates[edgeIDNew][timeStamp] += weightT;
                                residualEdgeInfo[make_pair(edgeIDNew, timeStamp)] += 1;
                                EdgeUpdates[edgeIDNew][timeStamp] /= residualEdgeInfo[make_pair(edgeIDNew, timeStamp)];
                            }
                        }
                        StreamUpdates[timeStamp].emplace_back(EdgeToNodeMap[edgeID], weightT);
                        if (minTime > timeStamp) minTime = timeStamp;
                        if (maxTime < timeStamp) maxTime = timeStamp;
                    }
                    else{
                        cout<<"Not found in EdgeIDMap "<<edgeID<<endl; exit(1);
                    }
                }
                else{
//                cout<<"Not found!"<<endl; exit(1);
                }
            }
        }
        cout<<"Repeated edge update number: "<<residualEdgeInfo.size()<<endl;
        cout<<"Time range: [ "<<minTime<<" "<<maxTime<<" ] , equals to "<<(double)(maxTime-minTime)/(60*60)<<" hours."<<endl;

        ifstream IFOut1(graphFile+".edgeUpdates");
        if (IFOut1.is_open() && !ifNew) {
            cout << "File " << graphFile+".edgeUpdates already exist." << endl;
            IFOut1.close();
        }
        else{
            cout<<"Storing edge updates"<<endl;
            ofstream OF(graphFile+".edgeUpdates", ios::out);
            if (!OF.is_open()) {
                cout << "Open file failed!" << graphFile+".edgeUpdates" << endl;
                exit(1);
            }
            unsigned long long int edgeUpdateNum=0;
            OF<<"vertexID1 vertexID2 spatialDis timeDis updateSize timeStamp1 timeDis1"<<endl;
            OF<<EdgeUpdates.size()<<endl;//edge number
            int edgeNum=0;
            for(int i=0;i<EdgeUpdates.size();++i){
                if(EdgeIDMapToOld.find(i)==EdgeIDMapToOld.end()){
                    OF<< "0 0 0 0 0"<<endl;
//                    cout<<"Not found edge "<<i<<" in EdgeIDMapToOld"<<endl;
//                    exit(1);
                }else if(EdgeToNodeMap.find(EdgeIDMapToOld[i])==EdgeToNodeMap.end()){
                    OF<< "0 0 0 0 0"<<endl;
//                    cout<<"Not found edge "<<EdgeIDMapToOld[i]<<" in EdgeToNodeMap "<<i<<endl;
//                    exit(1);
                }else{
                    edgeNum++;
                    OF<< EdgeToNodeMap[EdgeIDMapToOld[i]].first<<" "<<EdgeToNodeMap[EdgeIDMapToOld[i]].second<<" "<<Edges[i].first<<" "<<Edges[i].second<<" " <<EdgeUpdates[i].size();//original edge distance and time, edge update size
                    edgeUpdateNum+=EdgeUpdates[i].size();
                    for(auto it=EdgeUpdates[i].begin();it!=EdgeUpdates[i].end();++it){
                        int w1=Edges[i].second*0.83, w2=Edges[i].first/33.3;
                        if(it->second<w1 && it->second<w2){
                            cout<<"Invalid update "<<i<<" "<<EdgeIDMapToOld[i]<<" "<<EdgeToNodeMap[EdgeIDMapToOld[i]].first<<" "<<EdgeToNodeMap[EdgeIDMapToOld[i]].second<<" "<<Edges[i].first<<"("<<w2<<") "<<Edges[i].second<<"("<<w1<<") "<<it->second<<endl;
                            it->second=max(w1, w2);
//                            exit(1);
                        }
                        OF<<" "<<it->first<<" "<<it->second;//update time and edge weight
                    }
                    OF<<endl;
                }

            }
            OF.close();
            tt.stop();
            if(edgeNum*2 != edge_num){
                cout<<"Inconsistent edge number for edgeUpdate file. "<<edgeNum<<" "<<edge_num<<endl;
                exit(1);
            }
            cout<<"Edge update number: "<<edgeUpdateNum<<endl;
        }

        ifstream IFOut2(graphFile+".streamUpdates");
        if (IFOut2.is_open() && !ifNew) {
            cout << "File " << graphFile+".streamUpdates already exist." << endl;
            IFOut2.close();
        }else{
            cout<<"Storing stream updates"<<endl;
            ofstream OF2(graphFile+".streamUpdates", ios::out);
            if (!OF2.is_open()) {
                cout << "Open file failed!" << graphFile+".streamUpdates" << endl;
                exit(1);
            }
            OF2<<StreamUpdates.size()<<endl;//time stamp number
            for(auto it=StreamUpdates.begin();it!=StreamUpdates.end();++it) {
                OF2 << it->first << " " << it->second.size();
                for (int i = 0; i < it->second.size(); ++i) {
                    OF2<<" "<<it->second[i].first.first << " " << it->second[i].first.second << " "<< it->second[i].second;// ID1, ID2, weight
                }
                OF2<<endl;
            }
            OF2.close();
            tt.stop();
            cout<<"Stream update number: "<<StreamUpdates.size()<<endl;
        }


        cout<<"Storing queries"<<endl;
        ofstream OF3(graphFile+".realQueries", ios::out);
        if (!OF3.is_open() && !ifNew) {
            cout << "Open file failed!" << graphFile+".realQueries" << endl;
            exit(1);
        }
        sort(Queries.begin(),Queries.end());
        OF3<<Queries.size()<<endl;//edge number
        for(int i=0;i<Queries.size();++i){
            OF3<<Queries[i].first<<" "<<get<0>(Queries[i].second)<<" " <<get<1>(Queries[i].second)<<" "<<get<2>(Queries[i].second)<<" "<<get<3>(Queries[i].second)<<endl;//time stamp, ID1, ID2, carType, travelDis
        }
        OF3.close();
        tt.stop();
        cout<<"Real query number: "<<Queries.size()<<endl;

        cout<<"CPU elapsed time: "<<tt.GetRuntime()<<" s."<<endl;
    }


}

//function of getting target trajectories of certain GPS range
void GetTargetTrajectory(vector<string> sourceFiles, string outputFile, pair<double,double> lonP, pair<double,double> latP) {
    ifstream IFOut(outputFile);
    if(IFOut.is_open()){//if exist
        cout<<"File "<<outputFile<<" already exist."<<endl;
        IFOut.close();
//        cout<<"Reading trajectories..."<<endl;

        ifstream IF5(outputFile, ios::in);
        if (!IF5.is_open()) {
            cout << "Open file failed!" << outputFile << endl; exit(1);
        }
        string line;
        vector<string> vs;
        vector<int> dayIDs;
        dayIDs.emplace_back(20160101); dayIDs.emplace_back(20160102); dayIDs.emplace_back(20160103); dayIDs.emplace_back(20160104);dayIDs.emplace_back(20160105);
        vector<pair<unsigned long long int, unsigned long long int>> dayIntervals;
        dayIntervals.emplace_back(1451577600,1451664000); dayIntervals.emplace_back(1451664000,1451750400);
        dayIntervals.emplace_back(1451750400,1451836800); dayIntervals.emplace_back(1451836800,1451923200);
        dayIntervals.emplace_back(1451923200,1452009600);

        getline(IF5, line);
        getline(IF5, line);
        vs.clear();
        boost::split(vs,line,boost::is_any_of(" "));
//        vs=split(line,",");//link ID of trajectory
        if (vs.size() != 1) {
            cout << "Wrong syntax! " <<vs.size() <<" : "<< line << endl;
            exit(1);
        }
        int tNum= stoi(vs[0]);
        int lineNum = 0;
        vector<int> tNumPerDay(5,0);
        long long int startT, endT;
        while (getline(IF5, line)) {
            if (line == "") continue;
            vs.clear();
            boost::split(vs,line,boost::is_any_of(" "));
//        vs=split(line,",");//link ID of trajectory
            if (vs.size() < 17) {
                cout << "Wrong syntax! " <<vs.size() <<" : "<< line << endl;
                exit(1);
            }
            startT=stoll(vs[5]);
            endT=stoll(vs[6]);
            if((startT >= dayIntervals[4].first && startT < dayIntervals[4].second) || (endT >= dayIntervals[4].first && endT < dayIntervals[4].second)){
                tNumPerDay[4]++;
            }

//        lon=stod(vs[7]), lat=stod(vs[8]);
//        lon=stod(vs[9]), lat=stod(vs[10]);
            lineNum++;


        }
        IF5.close();

        cout << "Overall trajectory number: " << lineNum <<" ; trajectory of day 5: "<< tNumPerDay[4]<<endl;
//        exit(0);

    }
    else{//if not exist
        cout<<"Longitude range: "<< lonP.first<<" "<<lonP.second<<" ; Latitude range: "<<latP.first<<" "<<latP.second<<endl;
        IFOut.close();
        Timer tt;
        tt.start();
        vector<string> vs;
        string line;
        long long int EdgeID;
        unsigned long long int TimeID;//Time to 1970, unit: s
        vector<string> carID;
        vector<int> carType;//1: private car; 2: taxi; 0: others
        vector<vector<long long int>> trajectory;
        vector<vector<unsigned long long int>> trajectoryTime;
        vector<vector<int>> trajectorySpeed;
        vector<vector<int>> trajectoryCity;
        vector<int> linkNum;
        vector<int> travelDis, travelTime, travelSpeed;
        vector<pair<double,double>> startPoint;//gps
        vector<pair<double,double>> endPoint;//gps
        vector<unsigned long long int> startTime, endTime;
        int lineNum = 0;
        int tNum=0;
        vector<unsigned long long int> traTemp;
        vector<int> traTemp2;
        vector<long long int> traTemp3;

        unsigned long long int minTime=INT64_MAX, maxTime=0;
        double minLon=INT16_MAX, minLat=INT16_MAX;
        double maxLon=-INT16_MAX, maxLat=-INT16_MAX;
        double lon, lat;

        cout<<"Target Longitude range: "<< lonP.first<<" "<<lonP.second<<" ; Latitude range: "<<latP.first<<" "<<latP.second<<endl;
        cout<<"Reading trajectories..."<<endl;


        for(int fi=0;fi<sourceFiles.size();++fi){
            string sourceFile=sourceFiles[fi];
            ifstream IF(sourceFile, ios::in);
            if (!IF.is_open()) {
                cout << "Open file failed!" << sourceFile << endl;
                continue;
            }

            getline(IF, line);
            getline(IF, line);
            vs.clear();
            boost::split(vs,line,boost::is_any_of(" "));
//        vs=split(line,",");//link ID of trajectory
            if (vs.size() != 1) {
                cout << "Wrong syntax! " <<vs.size() <<" : "<< line << endl;
                exit(1);
            }
            tNum= stoi(vs[0]);
            bool flagFind=false;
            lineNum = 0;

            while (getline(IF, line)) {
                if (line == "") continue;
                vs.clear();
                boost::split(vs,line,boost::is_any_of(" "));
//        vs=split(line,",");//link ID of trajectory
                if (vs.size() < 17) {
                    cout << "Wrong syntax! " <<vs.size() <<" : "<< line << endl;
                    exit(1);
                }

                flagFind=false;
                lon=stod(vs[7]), lat=stod(vs[8]);
                if(lon>lonP.first && lon<lonP.second && lat>latP.first && lat<latP.second){
//            cout<<"s: "<<lon<<" "<<lat<<endl;
                    flagFind=true;
                }
                lon=stod(vs[9]), lat=stod(vs[10]);
                if(lon>lonP.first && lon<lonP.second && lat>latP.first && lat<latP.second){
//            cout<<"t: "<<lon<<" "<<lat<<endl;
                    flagFind=true;
                }
                lineNum++;
                if(!flagFind) continue;

                carID.emplace_back(vs[0]), carType.emplace_back(stoi(vs[1]));
                travelDis.emplace_back(stoi(vs[2])), travelTime.emplace_back(stoi(vs[3])), travelSpeed.emplace_back(stoi(vs[4]));
                startTime.emplace_back(stoll(vs[5])), endTime.emplace_back(stoll(vs[6]));
                startPoint.emplace_back(stod(vs[7]),stod(vs[8]));
                endPoint.emplace_back(stod(vs[9]), stod(vs[10]));

                int tempInt;
                tempInt=stoi(vs[11]);
                linkNum.emplace_back(tempInt);

                int index_i=12;
                traTemp3.clear();
                for(int i=0;i<tempInt;++i){
                    traTemp3.push_back(stoll(vs[index_i]));
                    index_i++;
                }
                trajectory.emplace_back(traTemp3);


                tempInt=stoi(vs[index_i]);
                index_i++;
                traTemp.clear();
                for(int i=0;i<tempInt;++i){
                    traTemp.push_back(stoull(vs[index_i]));
                    index_i++;
                }
                trajectoryTime.emplace_back(traTemp);

                tempInt=stoi(vs[index_i]);
                index_i++;
                traTemp2.clear();
                for(int i=0;i<tempInt;++i){
                    traTemp2.push_back(stoi(vs[index_i]));
                    index_i++;
                }
                trajectorySpeed.emplace_back(traTemp2);

                tempInt=stoi(vs[index_i]);
                index_i++;
                traTemp2.clear();
                for(int i=0;i<tempInt;++i){
                    traTemp2.push_back(stoi(vs[index_i]));
                    index_i++;
                }
                trajectoryCity.emplace_back(traTemp2);

//        cout<<tempInt<<":";
//        for(int i=0; i<tempInt; ++i){
//            cout<<" "<<traTemp2[i];
//        }
//        cout<<endl;

                if(minTime>startTime[startTime.size()-1]) minTime=startTime[startTime.size()-1];
                if(maxTime<endTime[endTime.size()-1]) maxTime=endTime[endTime.size()-1];
                if(minLon>startPoint[startPoint.size()-1].first) minLon=startPoint[startPoint.size()-1].first;
                if(minLon>endPoint[endPoint.size()-1].first) minLon=endPoint[endPoint.size()-1].first;
                if(minLat>startPoint[startPoint.size()-1].second) minLat=startPoint[startPoint.size()-1].second;
                if(minLat>endPoint[endPoint.size()-1].second) minLat=endPoint[endPoint.size()-1].second;
                if(maxLon<startPoint[startPoint.size()-1].first) maxLon=startPoint[startPoint.size()-1].first;
                if(maxLon<endPoint[endPoint.size()-1].first) maxLon=endPoint[endPoint.size()-1].first;
                if(maxLat<startPoint[startPoint.size()-1].second) maxLat=startPoint[startPoint.size()-1].second;
                if(maxLat<endPoint[endPoint.size()-1].second) maxLat=endPoint[endPoint.size()-1].second;

            }
            IF.close();
            if(lineNum!=tNum){
                cout<<"Inconsistent trajectory number! "<<lineNum<<" "<<tNum<<" "<<trajectory.size()<<endl; exit(1);
            }
//            cout<<"File "<<fi<<" done. "<< trajectory.size() << endl;
        }


        cout << "Trajectory number: " << trajectory.size() << " ; Time range: [ "<< minTime <<" "<<maxTime<<" ] s; GPS range: [ "<< minLon<<" "<<maxLon<<" ] [ "<<minLat<<" "<<maxLat<<" ]"<<endl;

        ofstream OF(outputFile, ios::out);
        if (!OF.is_open()) {
            cout << "Open file failed!" << outputFile << endl;
            exit(1);
        }
        OF << "carID carType travelDis travelTime travelSpeed start_time end_time start_gps_lon start_gps_lat end_gps_lon end_gps_lat linkNum links time_of_links.size time_of_links speed_of_links.size speed_of_links city_of_links.size city_of_links"<<endl;
        OF<<trajectory.size()<<endl;
        for(int i=0;i<trajectory.size();++i){
            OF<<carID[i]<<" ";
            OF<<carType[i]<<" ";
            OF<<travelDis[i]<<" "<<travelTime[i]<<" "<<travelSpeed[i]<<" ";
            OF<<startTime[i]<<" "<<endTime[i]<<" ";
            OF<<startPoint[i].first<<" "<<startPoint[i].second<<" "<<endPoint[i].first<<" "<<endPoint[i].second<<" ";
            OF<<trajectory[i].size();
            for(int j=0;j<trajectory[i].size();++j){
                OF<<" "<<trajectory[i][j];
            }
            OF<<" "<<trajectoryTime[i].size();
            for(int j=0;j<trajectoryTime[i].size();++j){
                OF<<" "<<trajectoryTime[i][j];
            }
            OF<<" "<<trajectorySpeed[i].size();
            for(int j=0;j<trajectorySpeed[i].size();++j){
                OF<<" "<<trajectorySpeed[i][j];
            }
            OF<<" "<<trajectoryCity[i].size();
            for(int j=0;j<trajectoryCity[i].size();++j){
                OF<<" "<<trajectoryCity[i][j];
            }
            OF<<endl;
        }
        OF.close();
        tt.stop();
        cout<<"CPU elapsed time: "<<tt.GetRuntime()<<" s."<<endl;
    }


}

//function of extracting useful information from original trajectory file
void TrajectoryExtract(string sourceFile, string outputFile, vector<string>& sourceFiles) {
    ifstream IFOut(outputFile);
    if(IFOut.is_open()){//if exist
//        cout<<"File "<<outputFile<<" already exist."<<endl;
//        string line;
//        getline(IFOut, line);
//        getline(IFOut, line);
//        vector<string> vs;
//        boost::split(vs,line,boost::is_any_of(" "));
//        int num=stoi(vs[0]);
//        cout<<"Trajectory number: "<<num<<endl;
        IFOut.close();
        sourceFiles.push_back(outputFile);
    }
    else{//if not open
        IFOut.close();
        ifstream IF(sourceFile, ios::in);
        if (!IF.is_open()) {//if not open
//            cout << "Open file failed! " << sourceFile << endl;
//        exit(1);
        }
        else{
            cout << "Processing " << sourceFile << endl;
            Timer tt;
            tt.start();
            vector<string> vs;
            string line;
            unsigned long long int EdgeID;
            unsigned long long int TimeID;//Time to 1970, unit: s
            vector<string> carID;
            vector<int> carType;//1: private car; 2: taxi; 0: others
            vector<vector<string>> trajectory;
            vector<vector<string>> trajectoryTime;
            vector<vector<string>> trajectorySpeed;
            vector<vector<string>> trajectoryCity;
            vector<int> linkNum;
            vector<int> travelDis, travelTime, travelSpeed;
            vector<pair<double,double>> startPoint;//gps
            vector<pair<double,double>> endPoint;//gps
            vector<unsigned long long int> startTime, endTime;
            int lineNum = 0;
            vector<string> traTemp;

            unsigned long long int minTime=INT64_MAX, maxTime=0;
            double minLon=INT16_MAX, minLat=INT16_MAX;
            double maxLon=-INT16_MAX, maxLat=-INT16_MAX;

            cout<<"Reading trajectories..."<<endl;
            long long int invalidNum=0;
            while (getline(IF, line)) {
                if (line == "") continue;
                vs.clear();
                boost::split(vs,line,boost::is_any_of(","));
//        vs=split(line,",");//link ID of trajectory
                if (vs.size() < 28) {
                    cout << "Wrong syntax! " <<vs.size() <<" : "<< line << endl;
                    exit(1);
                }

                if(stoll(vs[26])<1448899200 || stoll(vs[27])>1454256000){//if the start time stamp is before December 1st 2015 or the end time stamp is after February 1st 2016
//                cout<<lineNum<<": "<<minTime<<" "<< startTime[startTime.size()-1]<<endl;
                    invalidNum++;
                    continue;
                }

                carID.emplace_back(vs[2]), carType.emplace_back(stoi(vs[3]));
                linkNum.emplace_back(stoi(vs[11]));

                traTemp.clear();
                boost::split(traTemp,vs[4],boost::is_any_of("|"));
//        traTemp=split(vs[4],"|");//link ID of trajectory
                trajectory.emplace_back(traTemp);
                if(linkNum[linkNum.size()-1] != traTemp.size()){
                    cout<<"Incorrect link number: "<<linkNum[linkNum.size()-1]<<" "<<traTemp.size()<<endl; exit(1);
                }

                traTemp.clear();
                boost::split(traTemp,vs[7],boost::is_any_of("|"));
//        traTemp=split(vs[7],"|");//traveled city
                trajectoryCity.emplace_back(traTemp);

                traTemp.clear();
                boost::split(traTemp,vs[8],boost::is_any_of("|"));
//        traTemp=split(vs[8],"|");//time of each link
                trajectoryTime.emplace_back(traTemp);

                traTemp.clear();
                boost::split(traTemp,vs[9],boost::is_any_of("|"));
//        traTemp=split(vs[8],"|");//time of each link
                trajectorySpeed.emplace_back(traTemp);

                travelDis.emplace_back(stoi(vs[14])), travelTime.emplace_back(stoi(vs[15])), travelSpeed.emplace_back(stoi(vs[16]));

                startPoint.emplace_back(stod(vs[22]),stod(vs[23]));
                endPoint.emplace_back(stod(vs[24]), stod(vs[25]));
                startTime.emplace_back(stoll(vs[26])), endTime.emplace_back(stoll(vs[27]));
                if(minTime>startTime[startTime.size()-1]) minTime=startTime[startTime.size()-1];
                if(maxTime<endTime[endTime.size()-1]) maxTime=endTime[endTime.size()-1];
                if(minLon>startPoint[startPoint.size()-1].first) minLon=startPoint[startPoint.size()-1].first;
                if(minLon>endPoint[endPoint.size()-1].first) minLon=endPoint[endPoint.size()-1].first;
                if(minLat>startPoint[startPoint.size()-1].second) minLat=startPoint[startPoint.size()-1].second;
                if(minLat>endPoint[endPoint.size()-1].second) minLat=endPoint[endPoint.size()-1].second;
                if(maxLon<startPoint[startPoint.size()-1].first) maxLon=startPoint[startPoint.size()-1].first;
                if(maxLon<endPoint[endPoint.size()-1].first) maxLon=endPoint[endPoint.size()-1].first;
                if(maxLat<startPoint[startPoint.size()-1].second) maxLat=startPoint[startPoint.size()-1].second;
                if(maxLat<endPoint[endPoint.size()-1].second) maxLat=endPoint[endPoint.size()-1].second;
                lineNum++;
            }
            IF.close();
            if(trajectory.size()!=lineNum){
                cout<<"Inconsistent trajectory number! "<<trajectory.size()<<" "<<lineNum<<endl; exit(1);
            }

            cout << "Trajectory number: " << lineNum << " ; Time range: [ "<< minTime <<" "<<maxTime<<" ] s; GPS range: [ "<< minLon<<" "<<maxLon<<" ] [ "<<minLat<<" "<<maxLat<<" ]"<<endl;
            cout << "Invalid trajectory number: "<<invalidNum<<endl;

            ofstream OF(outputFile, ios::out);
            if (!OF.is_open()) {
                cout << "Open file failed!" << outputFile << endl;
                exit(1);
            }
            OF << "carID carType travelDis travelTime travelSpeed start_time end_time start_gps_lon start_gps_lat end_gps_lon end_gps_lat linkNum links time_of_links.size time_of_links speed_of_links.size speed_of_links city_of_links.size city_of_links"<<endl;
            OF<<lineNum<<endl;
            for(int i=0;i<lineNum;++i){
//            if(startTime[i]<1400000000) continue;
                OF<<carID[i]<<" ";
                OF<<carType[i]<<" ";
                OF<<travelDis[i]<<" "<<travelTime[i]<<" "<<travelSpeed[i]<<" ";
                OF<<startTime[i]<<" "<<endTime[i]<<" ";
                OF<<startPoint[i].first<<" "<<startPoint[i].second<<" "<<endPoint[i].first<<" "<<endPoint[i].second<<" ";
                OF<<trajectory[i].size();
                for(int j=0;j<trajectory[i].size();++j){
                    OF<<" "<<trajectory[i][j];
                }
                OF<<" "<<trajectoryTime[i].size();
                for(int j=0;j<trajectoryTime[i].size();++j){
                    OF<<" "<<trajectoryTime[i][j];
                }
                OF<<" "<<trajectorySpeed[i].size();
                for(int j=0;j<trajectorySpeed[i].size();++j){
                    OF<<" "<<trajectorySpeed[i][j];
                }
                OF<<" "<<trajectoryCity[i].size();
                for(int j=0;j<trajectoryCity[i].size();++j){
                    OF<<" "<<trajectoryCity[i][j];
                }
                OF<<endl;
            }
            OF.close();
            tt.stop();
            sourceFiles.push_back(outputFile);
            cout<<"CPU elapsed time: "<<tt.GetRuntime()<<" s."<<endl;
        }
    }


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


//function of computing Euclidean distance, longitude and latitude
double EuclideanDis(pair<double,double> s, pair<double,double> t)
{
    double d=111.319;//distance in equator per degree, kilometer
    double coe;
    double temp=(s.second+t.second)/2;
    temp=temp*3.1415926/180;
//    temp=temp*3.1415926/(180*1000000);
    coe=cos(temp);
    double y1 = s.second * d, y2 = t.second * d;
    double x1 = s.first * d * coe, x2 = t.first * d * coe;
    double xx = x1 - x2, yy = y1 - y2;
    return sqrt(xx * xx + yy * yy);//Euclidean Distance in meter
}