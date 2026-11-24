package uavswarm;

network Swarm
{
    parameters:
        int numClusters = default(10);
        int clusterSize = default(10);

    submodules:
        cluster[numClusters]: Cluster {
            cid = index;
            nc = parent.clusterSize;
            m = parent.numClusters;
            @display("p=100,100,m,5,280,220");
        }

    connections:
        for i=0..numClusters-2, for j=i+1..numClusters-1 {
            cluster[i].up++ <--> Link <--> cluster[j].up++;
        }
}