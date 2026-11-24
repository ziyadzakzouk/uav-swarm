namespace uavswarm;

packet StateMsg
{
    int srcId;          // global id of the sender
    simtime_t sentAt;   // timestamp of the state
    double gnssX;       // reported GNSS position
    double gnssY;
    double insVelX;     // INS-derived velocity
    double insVelY;
    bool truthSpoofed;
}