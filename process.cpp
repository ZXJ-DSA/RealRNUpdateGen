/*
 * process.cpp
 * Function: to generate csv files for visualizing the road network, updates, and queries
 *
 *  Created on: 16 June 2024
 *      Author: Xinjie ZHOU
 */
#include <ogrsf_frmts.h>
#include <gdal.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string.h>
#include <fstream>
#include <map>
#include <string>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

using namespace std;

vector<string> split(const string &s, const string &seperator);
void ProcessPartitionedGraph(string sourcePath, string dataset, string graphFile, string coordFile, int& node_num, int& edge_num, vector<pair<double,double>>& Coord, int pNum);
void ProcessWholeGraph(string sourcePath, string dataset, string graphFile, string coordFile, int& node_num, int& edge_num, vector<vector<pair<int,int>>>& Neighbors, vector<pair<double,double>>& Coord);
void ReadGraphPartitions(string filename, int& node_num, int& edge_num, int& partiNum);
void ReadGraph(string& filename, int& node_num, int& edge_num, vector<vector<pair<int,int>>>& Neighbors);
void ReadCoordinate(string& filename, int& node_num, vector<pair<double,double>>& Coord);
void WriteEdgeOverlayCSVFile(string filename, vector<vector<pair<int,int>>>& Neighbors, vector<pair<double,double>>& Coord);
void WriteEdgePartiCSVFile(int pid, string filename, vector<vector<pair<int,int>>>& Neighbors, vector<pair<double,double>>& Coord);
void WriteNodeOverlayCSVFile(string filename, vector<vector<pair<int,int>>>& Neighbors, vector<pair<double,double>>& Coord);
void WriteEdgeCSVFile(string filename, vector<vector<pair<int,int>>>& Neighbors, vector<pair<double,double>>& Coord);
void WriteNodeCSVFile(string filename, vector<vector<pair<int,int>>>& Neighbors, vector<pair<double,double>>& Coord);
void QueryToNodeCSV(string graphFile, vector<pair<double,double>>& Coord, string dayName, unsigned long long startT, unsigned long long endT);
void UpdateToNodeCSV(string graphFile, vector<pair<double,double>>& Coord, string dayName, unsigned long long startT, unsigned long long endT);
void StatisticCompute(string filename, unsigned long long startT, unsigned long long endT);
bool ifNew=false;
vector<vector<int>> PartiVertex;//<partition_number,<in-partition vertices>>, in increasing vertex order, higher-rank vertex first
vector<vector<int>> BoundVertex;//boundary vertices of each partition
vector<vector<pair<int,int>>> Neighbor;//original graph
vector<vector<pair<int,int>>> NeighborsParti;//<node_number,<in-partition adjacency lists>>
vector<vector<pair<int,int>>> NeighborsOverlay;//<node_number,<adjacency lists of overlay graph>>
vector<pair<int,bool>> PartiTag;//<node_number,<partition_id,if_boundary>>, for PMHL

int main(int argc, char** argv){
    if( argc < 3 || argc > 6){//
        printf("usage:\n<arg1> source path, e.g /data/xzhouby/datasets/map/\n");
        printf("<arg2> dataset, e.g. Guangdong\n");
        printf("<arg3> trajectory path (optional), e.g. /data/xzhouby/datasets/trajectoryData/m=01/Guangdong.trajectory\n");
        printf("<arg4> if regenerate data (optional), 0: No, 1: Yes, default: 0\n");
        printf("<arg5> partition number (optional), e.g. 8\n");
        exit(0);
    }

    string dataset=argv[2];
    string sourcePath=argv[1];
    string graphFile=argv[1]+dataset+"/"+dataset+".time";
    string coordFile=graphFile+".co";
    string trajectoryPath;
    string partiPath;
    int pNum=0;
    if(argc>3){
        trajectoryPath=argv[3];
        if(argc>4){
            ifNew=stoi(argv[4]);
        }
        if(argc>5){
            pNum=stoi(argv[5]);
        }
    }


    int node_num=0, edge_num=0;
    vector<vector<pair<int,int>>> Neighbors;
    vector<pair<double,double>> Coord;

    /// Show the whole road network
//    ProcessWholeGraph(graphFile,coordFile,node_num,edge_num,Neighbors,Coord);
    /// Show partitions of road network
    ProcessPartitionedGraph(sourcePath,dataset,graphFile,coordFile,node_num,edge_num,Coord,pNum);

    // Step 4: Output query CSV file
    unsigned long long startT=1451923200;//2016-01-05 00:00
    unsigned long long endT=1452009600;//2016-01-05 23:59
    QueryToNodeCSV(sourcePath+dataset+"/"+dataset, Coord, "d5", startT, endT);

    // Step 5: Output update CSV file
    UpdateToNodeCSV(sourcePath+dataset+"/"+dataset, Coord, "d5", startT, endT);

    // Step 6: Compute statistic information
//    StatisticCompute(sourcePath+dataset+"/"+dataset+".realQueries", startT, endT);

    return 0;
}

void ProcessPartitionedGraph(string sourcePath, string dataset, string graphFile, string coordFile, int& node_num, int& edge_num,  vector<pair<double,double>>& Coord, int pNum){
    // Step 1: Read road networks
    ReadCoordinate(coordFile,node_num,Coord);
    // Read partitions
    int partiNum=0;
    ReadGraphPartitions(sourcePath+dataset+"/partitions/"+dataset+"_NC_"+to_string(pNum),node_num,edge_num,partiNum);

    for(int i=0;i<partiNum;++i){
        // Step 2: Output the edge CSV file
        WriteEdgePartiCSVFile(i,sourcePath+dataset+"/partitions/"+dataset+"_NC_"+to_string(pNum)+"/"+dataset+"_edge_"+to_string(partiNum)+"_"+to_string(i)+".csv", NeighborsParti, Coord);
    }

    //write overlay edges
    WriteEdgeOverlayCSVFile(sourcePath+dataset+"/partitions/"+dataset+"_NC_"+to_string(pNum)+"/"+dataset+"_edge_"+to_string(partiNum)+"_overlay.csv", NeighborsOverlay, Coord);
    WriteNodeOverlayCSVFile(sourcePath+dataset+"/partitions/"+dataset+"_NC_"+to_string(pNum)+"/"+dataset+"_node_"+to_string(partiNum)+"_overlay.csv", NeighborsOverlay, Coord);
}

void ProcessWholeGraph(string sourcePath, string dataset, string graphFile, string coordFile, int& node_num, int& edge_num, vector<vector<pair<int,int>>>& Neighbors, vector<pair<double,double>>& Coord){
    // Step 1: Read road networks
    ReadCoordinate(coordFile,node_num,Coord);
    // Read whole road network
    ReadGraph(graphFile,node_num,edge_num,Neighbors);

    // Step 2: Output the edge CSV file
    WriteEdgeCSVFile(graphFile+"_edge.csv", Neighbors, Coord);
    // Step 3: Output the node CSV file
    WriteNodeCSVFile(graphFile+"_node.csv", Neighbors, Coord);

    // Step 4: Output query CSV file
    unsigned long long startT=1451923200;//2016-01-05 00:00
    unsigned long long endT=1452009600;//2016-01-05 23:59
    QueryToNodeCSV(sourcePath+dataset+"/"+dataset, Coord, "d5", startT, endT);

    // Step 5: Output update CSV file
    UpdateToNodeCSV(sourcePath+dataset+"/"+dataset, Coord, "d5", startT, endT);

    // Step 6: Compute statistic information
    StatisticCompute(sourcePath+dataset+"/"+dataset+".realQueries", startT, endT);
}

void ReadGraphPartitions(string filename, int& node_num, int& edge_num, int& partiNum){
    //read the partitioned graphs
//    vector<vector<int>> PartiVertex;//<partition_number,<in-partition vertices>>, in increasing vertex order, higher-rank vertex first
//    vector<vector<int>> BoundVertex;//boundary vertices of each partition
//    vector<vector<pair<int,int>>> Neighbor;//original graph
//    vector<vector<pair<int,int>>> NeighborsParti;//<node_number,<in-partition adjacency lists>>
//    vector<vector<pair<int,int>>> NeighborsOverlay;//<node_number,<adjacency lists of overlay graph>>
//    vector<pair<int,bool>> PartiTag;//<node_number,<partition_id,if_boundary>>, for PMHL
//    int partiNum=0;

    Neighbor.assign(node_num,vector<pair<int,int>>());
    NeighborsParti.assign(node_num, vector<pair<int,int>>());
    NeighborsOverlay.assign(node_num,vector<pair<int,int>>());
    PartiTag.assign(node_num, make_pair(-1,false));

    ifstream IF1(filename+"/subgraph_vertex");
    if(!IF1){
        cout<<"Cannot open file "<<filename+"/subgraph_vertex"<<endl;
        exit(1);
    }

    int pnum2;
    IF1>>pnum2;
    partiNum = pnum2;
    cout<<"Partition number: "<<pnum2<<endl;
    PartiVertex.assign(partiNum,vector<int>());
    BoundVertex.assign(partiNum,vector<int>());
    for(int k=0;k<pnum2;k++){
        int vernum,ID;
        IF1>>vernum;
        for(int i=0;i<vernum;i++){
            IF1>>ID;
            assert(ID>=0);
            /*if(ID>=nodenum)
                cout<<"ID "<<ID<<" ,partition ID "<<k<<endl;*/

            if(ID>=0 && ID<node_num){
                if(PartiTag[ID].first==-1){
                    PartiTag[ID].first=k;
                    PartiVertex[k].push_back(ID);
                }else{
                    cout<<"vertex already in one partition!"<<ID<<" "<<PartiTag[ID].first<<" "<<k<<endl; exit(1);
                }
            }else{
                cout<<"Wrong vertex ID! "<<ID<<endl; exit(1);
            }

        }
    }
    //further check that each vertex is in one and only one partition
    for(int vid=0;vid<node_num;vid++){
        if(PartiTag[vid].first==-1){
            cout<<"vertex "<<vid<<" not within any partition"<<endl; exit(1);
        }
    }


    ifstream IF(filename+"/subgraph_edge");
    if(!IF){
        cout<<"Cannot open file "<<filename+"/subgraph_edge"<<endl;
        exit(1);
    }

    int pnum1;
    IF>>pnum1;
    for(int k=0;k<pnum1;k++){
        int edgenum0,ID1,ID2,weight;
        IF>>edgenum0;
        for(int i=0;i<edgenum0;i++){
            IF>>ID1>>ID2>>weight;
            if(ID1>=0 && ID1 <node_num && ID2>=0 && ID2 <node_num && weight>0){
                NeighborsParti[ID1].emplace_back(ID2,weight);
                Neighbor[ID1].emplace_back(ID2,weight);
//                if(NeighborMap[ID1].find(ID2)==NeighborMap[ID1].end()){//not found
//                    NeighborMap[ID1].insert(make_pair(ID2,weight));
//                }else{
//                    cout<<"Wrong for subgraph_edge! edge ("<<ID1<<", "<<ID2<<") already exist!"<<endl; exit(1);
//                }
            }else{
                cout<<"Wrong for subgraph_edge! "<<ID1<<" "<<ID2<<" "<<weight<<endl; exit(1);
            }


        }
    }

    //read the cut edges
    ifstream IF2(filename+"/cut_edges");
    if(!IF2){
        cout<<"Cannot open file "<<filename+"/cut_edges"<<endl;
        exit(1);
    }

    int ednum,ID1,ID2,weight;
    int boundaryNum=0;
    IF2>>ednum;
    for(int i=0;i<ednum;i++){
        IF2>>ID1>>ID2>>weight;

        if(ID1>=0 && ID1 <node_num && ID2>=0 && ID2 <node_num && weight>0){
            if(PartiTag[ID1].first==PartiTag[ID2].first){
                cout<<"two end points of cut edge are in the same partition"<<endl; exit(1);
            }

            if(!PartiTag[ID1].second){//if not boundary
                PartiTag[ID1].second=true;
                boundaryNum++;
                BoundVertex[PartiTag[ID1].first].emplace_back(ID1);
            }
            if(!PartiTag[ID2].second){
                PartiTag[ID2].second=true;
                boundaryNum++;
                BoundVertex[PartiTag[ID2].first].emplace_back(ID2);
            }

            Neighbor[ID1].emplace_back(ID2,weight);
            NeighborsOverlay[ID1].emplace_back(ID2,weight);
        }else{
            cout<<"Wrong for cut_edge! "<<ID1<<" "<<ID2<<" "<<weight<<endl; exit(1);
        }
    }


    vector<int> bNums;
//    partiRootsV.assign(partiNum,vector<int>());//partition root vertex
    for(int pid=0;pid<partiNum;++pid){
        bNums.emplace_back(BoundVertex[pid].size());
    }

    cout<<"Overall boundary vertex number: "<<boundaryNum<<" ; Average boundary vertex number for each partition: "<<boundaryNum/partiNum<<" ; Maximum boundary number: "<<*max_element(bNums.begin(),bNums.end())<<endl;

    //further check the edges
    unsigned long long calEdgeNum=0;
    for(int i=0;i<Neighbor.size();i++){
        calEdgeNum+=Neighbor[i].size();
    }
    if(edge_num == 0){
        edge_num = calEdgeNum;
    }else{
        if(edge_num!=calEdgeNum){
            cout<<"Inconsistent edge number! "<<edge_num<<" "<<calEdgeNum<<endl; exit(1);
        }
    }
    cout<<"calculated edge number "<<calEdgeNum<<", graph edge number "<<edge_num<<endl;
    cout<<"Partition data finish reading!"<<endl;
}

void StatisticCompute(string filename, unsigned long long startT, unsigned long long endT){
    cout<<"Reading queries..."<<endl;

    ifstream IF5(filename, ios::in);
    if (!IF5.is_open()) {
        cout << "Open file failed!" << filename << endl; exit(1);
    }
    string line;
    vector<string> vs;

    getline(IF5, line);
    vs.clear();
    boost::split(vs,line,boost::is_any_of(" "));
//        vs=split(line,",");//link ID of trajectory
    if (vs.size() != 1) {
        cout << "Wrong syntax! " <<vs.size() <<" : "<< line << endl;
        exit(1);
    }
    int qNum= stoi(vs[0]);
    unsigned long long lineNum = 0;
    unsigned long long timeStamp;
    int ID1, ID2, carType, travelDis;
    //compute the statistics of car type
    vector<int> carTypeNum(3,0);//1: private car; 2: taxi; 0: others
    //compute queries with more than 500 km
    vector<pair<long long int,tuple<int,int,int,int>>> longQueries;
    unsigned long long travelDisAll=0;
    while (getline(IF5, line)) {
        if (line == "") continue;
        vs.clear();
        boost::split(vs, line, boost::is_any_of(" "));
        if (vs.size()!=5){
            cout<<"Wrong input syntax!"<<endl;
            exit(1);
        }
        timeStamp=stoull(vs[0]), ID1=stoi(vs[1]), ID2=stoi(vs[2]), carType=stoi(vs[3]), travelDis=stoi(vs[4]);
        travelDisAll+=travelDis;
        if(carType>=0 && carType<=2){
            carTypeNum[carType]++;
        }
        else{
            cout<<"Wrong carType "<<carType<<endl; exit(1);
        }
        if(travelDis>300000){
            longQueries.emplace_back(timeStamp, make_tuple(ID1,ID2,carType,travelDis));
        }
    }

    cout<<"Overall query number: "<<qNum<<" ; long-distance (travel distance is longer than 300 km) query number: "<<longQueries.size()<<endl;
    cout<<"Average travel distance: "<<travelDisAll/qNum<<" m"<<endl;
    cout<<"Number of different car types: private car "<<carTypeNum[1]<<" ; taxi "<<carTypeNum[2]<<" ; others "<<carTypeNum[0]<<endl;
}

void UpdateToNodeCSV(string filename, vector<pair<double,double>>& Coord, string dayName, unsigned long long startT, unsigned long long endT){
    ifstream IFOut(filename+"_"+dayName+".realUpdate.csv");
    if (IFOut.is_open() && !ifNew) {
        cout << "File " << filename+".realUpdate.csv already exist." << endl;
        IFOut.close();
    }
    else{
        IFOut.close();
        ifstream IF(filename+".streamUpdates", ios::in);
        if (!IF.is_open()) {
            cout << "Open file failed!" << filename << endl;
            exit(1);
        }
        cout<<"Covert update to CSV"<<endl;
        vector<pair<unsigned long long int,pair<int,int>>> realUpdates;
        vector<map<int,int>> updateMap;//ID1, <ID2, number>
        updateMap.assign(Coord.size(),map<int,int>());

        vector<string> vs;
        string line;
        unsigned long long int timeStamp;
        int ID1, ID2, weight;
        getline(IF,line);
        vs.clear();
        boost::split(vs,line,boost::is_any_of(" "));
        unsigned long long int uNum=stoull(vs[0]);
        cout<<"Time stamp number: "<<uNum<<endl;
        for(int i=0;i<uNum;++i){
            getline(IF,line);
            vs.clear();
            boost::split(vs,line,boost::is_any_of(" "));
            timeStamp= stoull(vs[0]);
            if(timeStamp>=startT && timeStamp<endT) {
                int lNum = stoi(vs[1]);
                if(vs.size()<3*lNum+2){
                    cout<<"Wrong. "<<vs.size()<<" "<<lNum<<endl; exit(1);
                }
                for (int j = 0; j < lNum; ++j) {
                    ID1=stoi(vs[3*j+2]), ID2=stoi(vs[3*j+3]);
                    realUpdates.emplace_back(timeStamp, make_pair(ID1,ID2));

                    if(updateMap[ID1].find(ID2)==updateMap[ID1].end()){//if not found
                        updateMap[ID1].insert({ID2,1});
                    }else{//if found
                        updateMap[ID1][ID2]++;
                    }
                }
            }
        }
        /*cout<<"Update number: "<<uNum<<endl;
         while(getline(IF,line)){
            if(line.empty()) continue;
            istringstream iss(line);
            if (!(iss >> timeStamp >> ID1 >> ID2 >> weight)){
                cout<<"Wrong input syntax!"<<endl;
                exit(1);
            }
            if(timeStamp>=startT && timeStamp<endT){
                realUpdates.emplace_back(timeStamp, make_pair(ID1,ID2));

                if(updateMap[ID1].find(ID2)==updateMap[ID1].end()){//if not found
                    updateMap[ID1].insert({ID2,1});
                }else{//if found
                    updateMap[ID1][ID2]++;
                }
            }
        }*/
        IF.close();
        cout<<"Real update number: "<<realUpdates.size()<<endl;
//        ofstream OF(filename+"_"+dayName+".realUpdate.csv", ios::out);
//        if (!OF.is_open()) {
//            cout << "Open file failed!" << filename+".realUpdate.csv" << endl;
//            exit(1);
//        }
//
//        OF<<"updateID,x_lon,x_lat,y_lon,y_lat,timeStamp"<<endl;
//        unsigned long long int edgeID=0;
//        for(int i=0;i<realUpdates.size();++i){
//            ID1=realUpdates[i].second.first; ID2=realUpdates[i].second.second;
//            timeStamp=realUpdates[i].first;
//
//            OF<<i<<","<<Coord[ID1].first<<","<<Coord[ID1].second<<","<<Coord[ID2].first<<","<<Coord[ID2].second<<","<<timeStamp<<endl;
//        }
//
//        OF.close();

        ofstream OF3(filename+"_"+dayName+".updateNum.csv", ios::out);
        if (!OF3.is_open()) {
            cout << "Open file failed!" << filename+".updateNum.csv" << endl;
            exit(1);
        }

        OF3<<"x_lon,x_lat,y_lon,y_lat,num"<<endl;
        for(int i=0;i<updateMap.size();++i){
            if(!updateMap[i].empty()){
                for(auto it=updateMap[i].begin();it!=updateMap[i].end();++it){
                    ID2=it->first;
                    OF3<<Coord[i].first<<","<<Coord[i].second<<","<<Coord[ID2].first<<","<<Coord[ID2].second<<","<<it->second<<endl;
                }
            }

        }

        OF3.close();
        cout<<"Write done."<<endl;
    }


}

void QueryToNodeCSV(string filename, vector<pair<double,double>>& Coord, string dayName, unsigned long long startT, unsigned long long endT){
    ifstream IFOut(filename+"_"+dayName+".queryNum.csv");
    if (IFOut.is_open() && !ifNew) {
        cout << "File " << filename+".queryNum.csv already exist." << endl;
        IFOut.close();
    }
    else{
        IFOut.close();
        ifstream IF(filename+".realQueries", ios::in);
        if (!IF.is_open()) {
            cout << "Open file failed!" << filename << endl;
            exit(1);
        }
        cout<<"Covert query to CSV"<<endl;
        vector<pair<unsigned long long int,tuple<int,int,int,int>>> realQueries;
        vector<map<int,int>> queryMap;//ID1, <ID2, number>
        queryMap.assign(Coord.size(),map<int,int>());
        vector<pair<int,int>> vertexQNum;//ID, source number, target number
        vertexQNum.assign(Coord.size(),pair<int,int>());
        vector<int> carTypeNum(3,0);//1: private car; 2: taxi; 0: others
        //compute queries with more than 500 km
        vector<pair<long long int,tuple<int,int,int,int>>> longQueries;
        unsigned long long travelDisAll=0;
        vector<string> vs;
        string line;
        unsigned long long int timeStamp;
        int ID1, ID2, carType, travelDis;
        getline(IF,line);
        vs.clear();
        boost::split(vs,line,boost::is_any_of(" "));
        unsigned long long int qNum=stoull(vs[0]);
        cout<<"Query number: "<<qNum<<endl;
//    unsigned long long int queryNumReal=0;
        while(getline(IF,line)){
            if(line.empty()) continue;
            istringstream iss(line);
            if (!(iss >> timeStamp >> ID1 >> ID2 >> carType >> travelDis)){
                cout<<"Wrong input syntax!"<<endl;
                exit(1);
            }
            travelDisAll+=travelDis;
            if(carType>=0 && carType<=2){
                carTypeNum[carType]++;
            }
            else{
                cout<<"Wrong carType "<<carType<<endl; exit(1);
            }
            if(travelDis>300000){
                longQueries.emplace_back(timeStamp, make_tuple(ID1,ID2,carType,travelDis));
            }
            if(timeStamp>=startT && timeStamp<endT){
                realQueries.emplace_back(timeStamp, make_tuple(ID1,ID2,carType,travelDis));

                if(queryMap[ID1].find(ID2)==queryMap[ID1].end()){//if not found
                    queryMap[ID1].insert({ID2,1});
                }else{//if found
                    queryMap[ID1][ID2]++;
                }

                vertexQNum[ID1].first++;
                vertexQNum[ID2].second++;
            }

        }
        IF.close();
        cout<<"Real query number: "<<realQueries.size()<<endl;
        cout<<"Overall query number: "<<qNum<<" ; long-distance (travel distance is longer than 300 km) query number: "<<longQueries.size()<<endl;
        cout<<"Average travel distance: "<<travelDisAll/qNum<<" m"<<endl;
        cout<<"Number of different car types: private car "<<carTypeNum[1]<<" ; taxi "<<carTypeNum[2]<<" ; others "<<carTypeNum[0]<<endl;


        /*ofstream OF(filename+"_"+dayName+".realQuery.csv", ios::out);
        if (!OF.is_open()) {
            cout << "Open file failed!" << filename+".realQuery.csv" << endl;
            exit(1);
        }
        OF<<"queryID,x_lon,x_lat,y_lon,y_lat,timeStamp"<<endl;
        unsigned long long int edgeID=0;
        for(int i=0;i<realQueries.size();++i){
            ID1=get<0>(realQueries[i].second); ID2=get<1>(realQueries[i].second);
            timeStamp=realQueries[i].first;

            OF<<i<<","<<Coord[ID1].first<<","<<Coord[ID1].second<<","<<Coord[ID2].first<<","<<Coord[ID2].second<<","<<timeStamp<<endl;
        }
        OF.close();*/

        /*ofstream OF2(filename+"_"+dayName+".vertexQueryNum.csv", ios::out);
        if (!OF2.is_open()) {
            cout << "Open file failed!" << filename+".vertexQueryInfo.csv" << endl;
            exit(1);
        }
        OF2<<"nodeID,lon,lat,sourceNum,targetNum"<<endl;
        int num1, num2;
        for(int i=0;i<vertexQNum.size();++i){
            num1=vertexQNum[i].first; num2=vertexQNum[i].second;
            if(num1>0 || num2>0){
                OF2<<i<<","<<Coord[i].first<<","<<Coord[i].second<<","<<num1<<","<<num2<<endl;
            }
        }
        OF2.close();*/

        ofstream OF3(filename+"_"+dayName+".queryNum.csv", ios::out);
        if (!OF3.is_open()) {
            cout << "Open file failed!" << filename+".queryNum.csv" << endl;
            exit(1);
        }

        OF3<<"x_lon,x_lat,y_lon,y_lat,num"<<endl;
        for(int i=0;i<queryMap.size();++i){
            if(!queryMap[i].empty()){
                for(auto it=queryMap[i].begin();it!=queryMap[i].end();++it){
                    ID2=it->first;
                    OF3<<Coord[i].first<<","<<Coord[i].second<<","<<Coord[ID2].first<<","<<Coord[ID2].second<<","<<it->second<<endl;
                }
            }

        }

        OF3.close();
        cout<<"Write done."<<endl;
    }

}

void WriteEdgePartiCSVFile(int pid, string filename, vector<vector<pair<int,int>>>& Neighbors, vector<pair<double,double>>& Coord){
    ifstream IF(filename);
    if(IF.is_open() && !ifNew){//already exists
        cout<<"File "<<filename<<" already exists."<<endl;
        IF.close();
    }
    else{
        IF.close();
        ofstream OF(filename, ios::out);
        if (!OF.is_open()) {
            cout << "Open file failed!" << filename << endl;
            exit(1);
        }
        vector<string> vs;
        string line;
        int ID1, ID2, weightT;

        OF<<"edgeID,x_lon,x_lat,y_lon,y_lat,weight"<<endl;
        unsigned long long int edgeID=0;
        for(int i=0;i<Neighbors.size();++i){
            ID1=i;
            if(PartiTag[ID1].first!=pid){
                continue;
            }
            for(int j=0;j<Neighbors[i].size();++j){
                ID2=Neighbors[i][j].first;
                weightT=Neighbors[i][j].second;
                OF<<edgeID<<","<<Coord[ID1].first<<","<<Coord[ID1].second<<","<<Coord[ID2].first<<","<<Coord[ID2].second<<","<<weightT<<endl;
                edgeID++;
            }
        }

        OF.close();
        cout<<"Write done."<<endl;
    }

}

void WriteEdgeOverlayCSVFile(string filename, vector<vector<pair<int,int>>>& Neighbors, vector<pair<double,double>>& Coord){
    ifstream IF(filename);
    if(IF.is_open() && !ifNew){//already exists
        cout<<"File "<<filename<<" already exists."<<endl;
        IF.close();
    }
    else{
        IF.close();
        ofstream OF(filename, ios::out);
        if (!OF.is_open()) {
            cout << "Open file failed!" << filename << endl;
            exit(1);
        }
        vector<string> vs;
        string line;
        int ID1, ID2, weightT;

        OF<<"edgeID,x_lon,x_lat,y_lon,y_lat,weight"<<endl;
        unsigned long long int edgeID=0;
        for(int i=0;i<Neighbors.size();++i){
            ID1=i;
            if(!PartiTag[ID1].second){
                continue;
            }
            for(int j=0;j<Neighbors[i].size();++j){
                ID2=Neighbors[i][j].first;
                weightT=Neighbors[i][j].second;
                OF<<edgeID<<","<<Coord[ID1].first<<","<<Coord[ID1].second<<","<<Coord[ID2].first<<","<<Coord[ID2].second<<","<<weightT<<endl;
                edgeID++;
            }
        }

        OF.close();
        cout<<"Write done."<<endl;
    }

}

void WriteNodeOverlayCSVFile(string filename, vector<vector<pair<int,int>>>& Neighbors, vector<pair<double,double>>& Coord){
    ifstream IF(filename);
    if(IF.is_open()  && !ifNew){//already exists
        cout<<"File "<<filename<<" already exists."<<endl;
        IF.close();
    }
    else {
        IF.close();
        ofstream OF(filename, ios::out);
        if (!OF.is_open()) {
            cout << "Open file failed!" << filename << endl;
            exit(1);
        }
        vector<string> vs;
        string line;
        int ID1, ID2, weightT;

        OF << "nodeID,lon,lat" << endl;
        for (int i = 0; i < Neighbors.size(); ++i) {
            ID1 = i;
            if(PartiTag[ID1].second){
                OF << ID1 << "," << Coord[ID1].first << "," << Coord[ID1].second << endl;
            }
        }

        OF.close();
        cout << "Write done." << endl;
    }
}

void WriteEdgeCSVFile(string filename, vector<vector<pair<int,int>>>& Neighbors, vector<pair<double,double>>& Coord){
    ifstream IF(filename);
    if(IF.is_open() && !ifNew){//already exists
        cout<<"File "<<filename<<" already exists."<<endl;
        IF.close();
    }
    else{
        IF.close();
        ofstream OF(filename, ios::out);
        if (!OF.is_open()) {
            cout << "Open file failed!" << filename << endl;
            exit(1);
        }
        vector<string> vs;
        string line;
        int ID1, ID2, weightT;

        OF<<"edgeID,x_lon,x_lat,y_lon,y_lat,weight"<<endl;
        unsigned long long int edgeID=0;
        for(int i=0;i<Neighbors.size();++i){
            ID1=i;
            for(int j=0;j<Neighbors[i].size();++j){
                ID2=Neighbors[i][j].first;
                weightT=Neighbors[i][j].second;
                OF<<edgeID<<","<<Coord[ID1].first<<","<<Coord[ID1].second<<","<<Coord[ID2].first<<","<<Coord[ID2].second<<","<<weightT<<endl;
                edgeID++;
            }
        }

        OF.close();
        cout<<"Write done."<<endl;
    }

}

void WriteNodeCSVFile(string filename, vector<vector<pair<int,int>>>& Neighbors, vector<pair<double,double>>& Coord){
    ifstream IF(filename);
    if(IF.is_open()  && !ifNew){//already exists
        cout<<"File "<<filename<<" already exists."<<endl;
        IF.close();
    }
    else {
        IF.close();
        ofstream OF(filename, ios::out);
        if (!OF.is_open()) {
            cout << "Open file failed!" << filename << endl;
            exit(1);
        }
        vector<string> vs;
        string line;
        int ID1, ID2, weightT;

        OF << "nodeID,lon,lat" << endl;
        for (int i = 0; i < Neighbors.size(); ++i) {
            ID1 = i;
            OF << ID1 << "," << Coord[ID1].first << "," << Coord[ID1].second << endl;
        }

        OF.close();
        cout << "Write done." << endl;
    }
}

void ReadGraph(string& filename, int& node_num, int& edge_num, vector<vector<pair<int,int>>>& Neighbors){
    ifstream IF(filename, ios::in);
    if (!IF.is_open()) {
        cout << "Open file failed!" << filename << endl;
        exit(1);
    }
    vector<string> vs;
    string line;
    int ID1, ID2, weightT;
    getline(IF,line);
    vs.clear();
    boost::split(vs,line,boost::is_any_of(" "));
    if(node_num==0){
        node_num=stoi(vs[0]);
    }else if(node_num!=stoi(vs[0])){
        cout<<"Seem Wrong!!! "<<node_num<<" "<<stoi(vs[0])<<endl;
        node_num=stoi(vs[0]);
    }

    edge_num=stoi(vs[1]);
    cout<<"Node number: "<<node_num<<" , edge number: "<<edge_num<<endl;
    Neighbors.assign(node_num,vector<pair<int,int>>());
    while(getline(IF,line)){
        if(line.empty()) continue;
        istringstream iss(line);
        if (!(iss >> ID1 >> ID2 >> weightT)){
            cout<<"Wrong input syntax!"<<endl;
            exit(1);
        }

        if(ID1>=0 && ID1<node_num && ID2>=0 && ID2<node_num && weightT>0){
            Neighbors[ID1].emplace_back(ID2,weightT);
        }else{
            cout<<"Graph data is wrong! "<<ID1<<" "<<ID2<<" "<<weightT<<endl;
        }
    }
    IF.close();
}

void ReadCoordinate(string& filename, int& node_num, vector<pair<double,double>>& Coord){
    ifstream IF(filename, ios::in);
    if (!IF.is_open()) {
        cout << "Open file failed!" << filename << endl;
        exit(1);
    }
    vector<string> vs;
    string line;
    int ID, lon, lat;
    getline(IF,line);
    vs.clear();
    boost::split(vs,line,boost::is_any_of(" "));
    if(node_num==0){
        node_num=stoi(vs[0]);
    }else if(node_num!=stoi(vs[0])){
        cout<<"Seem Wrong!!! "<<node_num<<" "<<stoi(vs[0])<<endl;
        node_num=stoi(vs[0]);
    }
    cout<<"Node number: "<<node_num<<endl;
    Coord.assign(node_num,pair<int,int>());
    while(getline(IF,line)){
        if(line.empty()) continue;
        istringstream iss(line);
        if (!(iss >> ID >> lon >> lat)){
            cout<<"Wrong input syntax!"<<endl;
            exit(1);
        }

        if(ID>=0 && ID<node_num){
            Coord[ID].first=(double)lon/1000000;
            Coord[ID].second=(double)lat/1000000;
        }else{
            cout<<"Graph data is wrong! "<<ID<<" "<<lon<<" "<<lat<<endl;
        }
    }
    IF.close();
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
