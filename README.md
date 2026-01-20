# Extract Road Network, Trajectory, Query, and Updates
There are two projects.
 
## orgNew.cpp
Target at extracting original road networks, including sole road network or aggregated road network.

```
usage:
<arg1> source path, e.g /data/TrajectoryData/map/
<arg2> target path, e.g. /data/xzhouby/datasets/map/
<arg3> name of dataset, e.g. Guangdong
<arg4> aggregate road networks? (optional), 0: No, 1: Yes. Default: 0
<arg5> name of aggregated datasets (optional), eg. guangdong1 guangdong2 guangzhou
```

## trajectory.cpp
Target at extracting trajectories, queries, and updates related to certain road network.

```
usage:
<arg1> trajectory source path, e.g. /data/TrajectoryData/CennaviData/BasicTrajectory/m=01/
<arg2> trajectory target path, e.g. /home/xzhouby/datasets/trajectoryData/
<arg3> dataset, e.g. Guangdong
<arg4> graph path, e.g. /home/xzhouby/datasets/map/Guangdong/Guangdong
<arg5> minimum longitude (optional), e.g. 109.5
<arg6> maximum longitude (optional), e.g. 117.25
<arg7> minimum latitude (optional), e.g. 20.0833
<arg8> maximum latitude (optional), e.g. 25.6667
```


## process.cpp
Target at converting the update and query files to CSV files for QGIS.

```
usage:
<arg1> source path, e.g /data/TrajectoryData/map/
<arg2> dataset, e.g. Guangdong
```