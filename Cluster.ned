package uavswarm;

module Cluster
{
    parameters:
        int cid;
        int nc;
        int m;

    gates:
        inout up[];

    submodules:
        uav[nc]: UAV {
            clusterId = parent.cid;
            idInCluster = index;
            globalId = parent.cid * parent.nc + index;
            isLeader = (index == 0);
            @display("p=100,100,m,5,60,60");
        }

    connections:
        for i=0..nc-2, for j=i+1..nc-1 {
            uav[i].port++ <--> Link <--> uav[j].port++;
        }

        for k=0..m-2 {
            uav[0].up++ <--> up++;
        }
}