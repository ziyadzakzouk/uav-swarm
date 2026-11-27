namespace uavswarm;

//
// Stage 2.2, Step 4: evidence package sent by an observer to its cluster leader.
//
packet EvidenceMsg
{
    int observerId;          // global id of the observer (UAV i)
    int targetId;            // global id of the suspected neighbour (UAV j)
    int clusterId;           // cluster of the observer
    simtime_t timestamp;     // time of the sample that raised the event

    double targetPosX;       // target's reported GNSS position p_j(t)
    double targetPosY;
    double targetVelX;       // target's reported INS velocity v_j(t)
    double targetVelY;

    double obsPosX;          // observer's own position p_i(t)
    double obsPosY;
    double obsVelX;          // observer's own INS velocity v_i(t)
    double obsVelY;

    double rMot;             // motion residual
    double rRel;             // relative-position residual
    double anomaly;          // A_ij(t)
    bool   viaCusum;         // which rule fired: true = CUSUM, false = persistence
    double cusumValue;       // C_t when it fired
    int    persistCount;     // consecutive samples above tau when it fired

    uint64_t signature;      // modelled signature (keyed hash of the fields above)

    bool truthTargetSpoofed; // ground truth, scoring only (never used by the logic)
}