#include <omnetpp.h>
#include <map>
#include "Vec2.h"
#include "StateMsg_m.h"
#include <algorithm>

using namespace omnetpp;

namespace uavswarm {

class UAV : public cSimpleModule
{
  protected:
    // identity
    int globalId, clusterId, idInCluster;
    bool isLeader;

    // parameters
    double dt, speed, gnssSigma, insVelNoise;

    // ground truth (the simulator knows this, the UAV logic must not use it)
    Vec2 startPos, trueVel;

    // INS state
    Vec2 insPos, insVel, insBias;
    simtime_t lastTick;

    // latest GNSS fix
    Vec2 gnssPos;
    simtime_t gnssTime;          // when gnssPos was taken

    cMessage *stateTimer = nullptr;

    // Stage 2.1 parameters
    double w1, w2, sigmaM, sigmaR, sigmaS;
    simtime_t maxGap;            // Δt_max
    int anchorWindow;            // W
    double anchorGate;           // g
    simtime_t holdTime;          // T_hold

    struct Anchor {
        bool valid = false;
        simtime_t t;             // anchor time t_a
        Vec2 pos;                // p(t_a)
        Vec2 q;                  // q_ij(t_a)
        Vec2 sumMot;             // Σ v_j(t_{l-1})·Δt_l
        Vec2 sumRel;             // Σ (v_j − v_i)(t_{l-1})·Δt_l
        int windowCount = 0;     // samples in the current window
        double windowMax = 0;    // max statistic in the current window
        bool suspect = false;    // a window failed the gate
    };

    struct Sample {
        simtime_t t;
        Vec2 pos;                // reported GNSS position
        Vec2 vel;                // reported INS velocity
    };
    struct NeighborState {
        bool hasPrev = false;
        Anchor anchor;           // replaces lastQ
        Sample last;             // p_j(t_{l-1}), v_j(t_{l-1})
        Vec2 ownVelAtLast;       // v_i(t_{l-1})
        double lastA = 0;        // latest A_ij, used by Stage 2.2
        // Stage 2.2 state
        int persistCount = 0;        // consecutive samples with A > tau
        double cusum = 0;            // C_t

        // scoring only (ground truth, never used by the detection logic)
        bool truthSeen = false;
        simtime_t truthSince;        // first message that carried a spoofed fix
        bool detectedPers = false;
        bool detectedCusum = false;
    };
    std::map<int, NeighborState> neighbors;
    Anchor selfAnchor;           // anchor for the UAV's own track

    // Stage 2.2 parameters
    double tau;                  // persistence threshold (Eq. 4.10)
    int    persistK;             // k consecutive samples
    double cusumKappa;           // CUSUM slack (kappa)
    double cusumH;               // CUSUM alarm limit (h)

    simsignal_t persFalseSig, persDelaySig, cusumFalseSig, cusumDelaySig;
    simsignal_t cusumStatSig, persistRunSig, obsSpoofedEventSig;

    void resetDetectors(NeighborState &s);
    void raiseEvent(NeighborState &s, int targetId, bool viaCusum,
                    bool targetSpoofed, simtime_t t);

    void resetAnchor(Anchor &a, simtime_t t, const Vec2 &pos, const Vec2 &q);
    bool anchorShouldAdvance(Anchor &a, simtime_t now, double stat);


        simsignal_t selfResSig, motResSig, relResSig, anomalySig;

        Vec2 ownPositionAt(simtime_t t) const;

        // attack injection (ground truth; UAV logic must not use it)
            enum AttackType { ATTACK_NONE, ATTACK_JUMP, ATTACK_RAMP };
            bool isSpoofed;
            AttackType attackType;
            simtime_t attackStart;
            Vec2 attackDir;                 // unit vector of the false offset
            double jumpSize, rampRate, maxOffset;

            simsignal_t spoofOffsetSig, anomSpoofedSig, anomCleanSig, anomObsSpoofedSig;

            Vec2 spoofOffsetAt(simtime_t t) const;
            bool underAttack(simtime_t t) const;

    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;

    void onTick();
    void broadcastState();
    void onStateReceived(StateMsg *msg);

  public:
    virtual ~UAV();
};

Define_Module(UAV);

UAV::~UAV()
{
    cancelAndDelete(stateTimer);
}

void UAV::initialize()
{
    globalId    = par("globalId");
    clusterId   = par("clusterId");
    idInCluster = par("idInCluster");
    isLeader    = par("isLeader");

    dt          = par("stateInterval").doubleValue();
    speed       = par("speed");
    gnssSigma   = par("gnssSigma");
    insVelNoise = par("insVelNoise");

    w1          = par("w1");
    w2          = par("w2");
    sigmaM      = par("sigmaM");
    sigmaR      = par("sigmaR");
    maxGap       = par("maxGap").doubleValue();
    double insBiasSigma = par("insBiasSigma");

    isSpoofed   = par("isSpoofed");
    attackStart = par("attackStart").doubleValue();
    jumpSize    = par("jumpSize");
    rampRate    = par("rampRate");
    maxOffset   = par("maxOffset");

    sigmaS       = par("sigmaS");
    anchorWindow = par("anchorWindow");
    anchorGate   = par("anchorGate");
    holdTime     = par("holdTime").doubleValue();

    tau        = par("tau");
    persistK   = par("persistK");
    cusumKappa = par("cusumKappa");
    cusumH     = par("cusumH");

    persFalseSig       = registerSignal("persistenceFalse");
    persDelaySig       = registerSignal("persistenceDelay");
    cusumFalseSig      = registerSignal("cusumFalse");
    cusumDelaySig      = registerSignal("cusumDelay");
    cusumStatSig       = registerSignal("cusumStat");
    persistRunSig      = registerSignal("persistRun");
    obsSpoofedEventSig = registerSignal("eventWhileObserverSpoofed");

    double ang = par("attackAngle").doubleValue() * M_PI / 180.0;  // deg -> rad
    attackDir  = Vec2(std::cos(ang), std::sin(ang));

    std::string t = par("attackType").stdstringValue();
    if      (t == "none") attackType = ATTACK_NONE;
    else if (t == "jump") attackType = ATTACK_JUMP;
    else if (t == "ramp") attackType = ATTACK_RAMP;
    else throw cRuntimeError("Unknown attackType '%s'", t.c_str());

    spoofOffsetSig = registerSignal("spoofOffset");
    anomSpoofedSig = registerSignal("anomalyTargetSpoofed");
    anomCleanSig   = registerSignal("anomalyTargetClean");
    anomObsSpoofedSig = registerSignal("anomalyObserverSpoofed");

    // formation: clusters on a 5-column grid (300 m apart),
    // UAVs inside a cluster on a 5-column grid (20 m apart)
    startPos = Vec2((clusterId % 5) * 300.0 + (idInCluster % 5) * 20.0,
                    (clusterId / 5) * 300.0 + (idInCluster / 5) * 20.0);
    trueVel  = Vec2(speed, 0);

    // INS starts aligned with the true state, then drifts
    insBias = Vec2(normal(0, insBiasSigma), normal(0, insBiasSigma));
    insPos  = startPos;
    insVel  = trueVel + insBias;
    lastTick = 0;

    // initial GNSS fix so ownPositionAt() is valid before the first tick

    gnssPos  = startPos + Vec2(normal(0, gnssSigma), normal(0, gnssSigma));
    gnssTime = 0;

    selfResSig  = registerSignal("selfResidual");
    motResSig   = registerSignal("motionResidual");
    relResSig   = registerSignal("relResidual");
    anomalySig  = registerSignal("anomaly");

    // random start phase so 100 UAVs don't all transmit at the same instant
    stateTimer = new cMessage("stateTimer");
    scheduleAt(uniform(0, dt), stateTimer);

    if (isLeader)
        getDisplayString().setTagArg("i", 1, "red");
}

bool UAV::underAttack(simtime_t t) const
{
    return isSpoofed && attackType != ATTACK_NONE && t >= attackStart;
}

Vec2 UAV::spoofOffsetAt(simtime_t t) const
{
    if (!underAttack(t))
        return Vec2();                                  // zero offset

    double mag = 0;
    if (attackType == ATTACK_JUMP)
        mag = jumpSize;                                 // constant from attackStart on
    else if (attackType == ATTACK_RAMP)
        mag = std::min(rampRate * (t - attackStart).dbl(), maxOffset);

    return attackDir * mag;
}

// Make (t, pos, q) the new trusted reference and clear the accumulated movement.
void UAV::resetAnchor(Anchor &a, simtime_t t, const Vec2 &pos, const Vec2 &q)
{
    a.valid       = true;
    a.t           = t;
    a.pos         = pos;
    a.q           = q;
    a.sumMot      = Vec2();
    a.sumRel      = Vec2();
    a.windowCount = 0;
    a.windowMax   = 0;
    a.suspect     = false;
}

// Called once per sample with that sample's statistic.
// Returns true if the anchor should move to the current sample.
bool UAV::anchorShouldAdvance(Anchor &a, simtime_t now, double stat)
{
    a.windowCount++;
    a.windowMax = std::max(a.windowMax, stat);

    // the anchor can only move at the end of a complete window
    if (a.windowCount < anchorWindow)
        return false;

    // Rule 2: judge the window that just finished
    bool clean = (a.windowMax <= anchorGate);
    a.windowCount = 0;            // start a new window either way
    a.windowMax   = 0;

    if (!clean) {
        a.suspect = true;         // inconsistency: keep the anchor where it is
        return false;
    }

    a.suspect = false;            // the latest window is consistent again

    // Rule 3: move only when the anchor has reached its maximum age
    return (now - a.t) >= holdTime;
}

// Clear the Stage 2.2 state (used whenever the anchor is re-initialised by Rule 1).
void UAV::resetDetectors(NeighborState &s)
{
    s.persistCount = 0;
    s.cusum        = 0;
}

// A suspicion event for neighbor targetId. Step C will build the evidence
// package here; for now we only score the event against the ground truth.
void UAV::raiseEvent(NeighborState &s, int targetId, bool viaCusum,
                     bool targetSpoofed, simtime_t t)
{
    EV_INFO << "UAV " << globalId << ": " << (viaCusum ? "CUSUM" : "persistence")
            << " event against UAV " << targetId << " at t=" << t << "\n";

    // ---- scoring only (ground truth, never fed back into detection) ----
    if (targetSpoofed) {
        bool &done = viaCusum ? s.detectedCusum : s.detectedPers;
        if (!done) {                                   // first detection only
            done = true;
            emit(viaCusum ? cusumDelaySig : persDelaySig, (t - s.truthSince).dbl());
        }
    }
    else if (underAttack(simTime())) {
        // target is clean but this observer is the victim: the relative residual
        // is expected to fire. Phase 3 has to resolve this, so it is not a false alarm.
        emit(obsSpoofedEventSig, viaCusum ? 1.0 : 0.0);
    }
    else {
        emit(viaCusum ? cusumFalseSig : persFalseSig, 1.0);   // both clean: false alarm
    }

    // restart the rule so a new event needs fresh evidence
    if (viaCusum) s.cusum = 0;
    else          s.persistCount = 0;
}

void UAV::handleMessage(cMessage *msg)
{
    if (msg->isSelfMessage()) {
        onTick();                       // our own timer fired
    }
    else {
        onStateReceived(check_and_cast<StateMsg *>(msg));
        delete msg;                     // we own every received message
    }
}

void UAV::onTick()
{
    simtime_t now = simTime();
    double elapsed = (now - lastTick).dbl();
    lastTick = now;


    // INS: keep the previous velocity v_i(t_{l-1}), then take a new reading
    Vec2 prevVel = insVel;
       insPos = insPos + prevVel * elapsed;
       insVel = trueVel + insBias + Vec2(normal(0, insVelNoise), normal(0, insVelNoise));

    // GNSS: truth plus noise plus (if attacked) the spoofer's false offset
    Vec2 truePos = startPos + trueVel * now.dbl();
    Vec2 offset  = spoofOffsetAt(now);
    gnssPos  = truePos + Vec2(normal(0, gnssSigma), normal(0, gnssSigma)) + offset;
    gnssTime = now;

    if (isSpoofed)
        emit(spoofOffsetSig, offset.norm());           // ground-truth record

    if (!selfAnchor.valid) {
            resetAnchor(selfAnchor, now, gnssPos, Vec2());            // Rule 1
        }
        else {
            selfAnchor.sumMot = selfAnchor.sumMot + prevVel * elapsed;
            Vec2 selfPred = selfAnchor.pos + selfAnchor.sumMot;       // Eq. 4.1
            double rSelf  = (gnssPos - selfPred).norm();              // Eq. 4.2
            emit(selfResSig, rSelf);

            if (anchorShouldAdvance(selfAnchor, now, rSelf / sigmaS))
                resetAnchor(selfAnchor, now, gnssPos, Vec2());
        }

    broadcastState();
    scheduleAt(now + dt, stateTimer);
}

Vec2 UAV::ownPositionAt(simtime_t t) const
{
    // extrapolate our latest fix to time t using our INS velocity
    return gnssPos + insVel * (t - gnssTime).dbl();
}

void UAV::broadcastState()
{
    StateMsg *tmpl = new StateMsg("state");
    tmpl->setSrcId(globalId);
    tmpl->setSentAt(simTime());
    tmpl->setGnssX(gnssPos.x);
    tmpl->setGnssY(gnssPos.y);
    tmpl->setInsVelX(insVel.x);
    tmpl->setInsVelY(insVel.y);
    tmpl->setByteLength(64);   // modeled packet size
    tmpl->setTruthSpoofed(underAttack(simTime()));

    int n = gateSize("port");
    for (int i = 0; i < n; i++)
        send(tmpl->dup(), "port$o", i); // one copy per neighbor
    delete tmpl;                        // the template itself is never sent
}

void UAV::onStateReceived(StateMsg *msg)
{
    Sample cur;
    cur.t   = msg->getSentAt();
    cur.pos = Vec2(msg->getGnssX(), msg->getGnssY());
    cur.vel = Vec2(msg->getInsVelX(), msg->getInsVelY());

    Vec2 q = cur.pos - ownPositionAt(cur.t);          // q_ij(t), Eq. 4.4
    int srcId = msg->getSrcId();
    NeighborState &s = neighbors[srcId];

    // scoring only: remember when the target first sent a spoofed fix
    bool targetSpoofed = msg->getTruthSpoofed();
    if (targetSpoofed && !s.truthSeen) {
        s.truthSeen  = true;
        s.truthSince = cur.t;
    }

    if (!s.hasPrev) {
        resetAnchor(s.anchor, cur.t, cur.pos, q);     // Rule 1: first message
        resetDetectors(s);
    }
    else {
        simtime_t gap = cur.t - s.last.t;
        if (gap <= 0)
            return;                                   // out of order: ignore

        if (gap > maxGap) {
            resetAnchor(s.anchor, cur.t, cur.pos, q); // Rule 1: after an interruption
            resetDetectors(s);                        // no evidence across a gap
        }
        else {
            double d = gap.dbl();

            // extend the sums by one more interval (L grows by one)
            s.anchor.sumMot = s.anchor.sumMot + s.last.vel * d;
            s.anchor.sumRel = s.anchor.sumRel + (s.last.vel - s.ownVelAtLast) * d;

            // Eq. 4.3: motion residual
            Vec2 predPos = s.anchor.pos + s.anchor.sumMot;
            double rMot  = (cur.pos - predPos).norm();

            // Eq. 4.5-4.6: relative-position residual
            Vec2 qHat   = s.anchor.q + s.anchor.sumRel;
            double rRel = (q - qHat).norm();

            // Eq. 4.7
            double A = w1 * rMot / sigmaM + w2 * rRel / sigmaR;
            s.lastA = A;

            emit(motResSig, rMot);
            emit(relResSig, rRel);
            emit(anomalySig, A);

            // scoring only (ground truth): three separate cases
            if (msg->getTruthSpoofed())
                emit(anomSpoofedSig, A);          // target is spoofed
            else if (underAttack(simTime()))
                emit(anomObsSpoofedSig, A);       // target clean, but this observer is spoofed
            else
                emit(anomCleanSig, A);            // both clean

            // ---------------- Stage 2.2: event-triggered detection ----------------
            // Step 1: persistence rule, A > tau for k consecutive samples
            s.persistCount = (A > tau) ? s.persistCount + 1 : 0;
            emit(persistRunSig, (double)s.persistCount);
            if (s.persistCount >= persistK)
                raiseEvent(s, srcId, false, targetSpoofed, cur.t);

            // Step 2: CUSUM rule, Eq. 4.9 (kept across anchor moves so slow drift adds up)
            s.cusum = std::max(0.0, s.cusum + A - cusumKappa);
            emit(cusumStatSig, s.cusum);
            if (s.cusum > cusumH)
                raiseEvent(s, srcId, true, targetSpoofed, cur.t);

            // Step 6, rules 2 and 3
            if (anchorShouldAdvance(s.anchor, cur.t, A))
                resetAnchor(s.anchor, cur.t, cur.pos, q);
        }
    }

    s.last         = cur;
    s.ownVelAtLast = insVel;
    s.hasPrev      = true;
}

}  // namespace uavswarm
