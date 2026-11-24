package uavswarm;

simple UAV
{
    parameters:
        int globalId;
        int clusterId;
        int idInCluster;
        bool isLeader = default(false);

        // motion
        double speed = default(10);                // m/s, formation flies along +x

        // timing-
        double stateInterval @unit(s) = default(0.5s);   // the Δt of Section 3.2.3

        // sensor models
        double gnssSigma = default(1.5);           // m, GNSS noise std-dev per axis
        double insVelNoise = default(0.05);        // m/s, INS velocity noise per sample
        double insBiasSigma = default(0.02);       // m/s, constant INS velocity bias spread

        // Stage 2.1: anomaly statistic
        double w1 = default(0.5);
        double w2 = default(0.5);
        double sigmaM = default(1.39);
        double sigmaR = default(1.97);
        double maxGap @unit(s) = default(1.5s);   // ~3 broadcast periods

        // attack injection (ground truth, configured from omnetpp.ini)
        bool isSpoofed = default(false);            // is this UAV a spoofing victim?
        string attackType = default("none");        // "none", "jump" or "ramp"
        double attackStart @unit(s) = default(100s);
        double attackAngle @unit(deg) = default(90deg); // direction of the false offset
        double jumpSize = default(20);                  // m, size of the "jump" offset
        double rampRate = default(1.0);                 // m/s, growth rate of the "ramp" offset
        double maxOffset = default(200);                // m, cap for the ramp


        // Stage 2.1: anchored reference (Steps 1 and 6)
        double sigmaS = default(1.39);
        int    anchorWindow = default(10);         // W: samples per clean-window check (10 x 0.5 s = 5 s)
        double anchorGate = default(7.0);
        double holdTime @unit(s) = default(20s);   // T_hold: maximum anchor age

        // Stage 2.2: event-triggered detection
        double tau = default(4.44);          // persistence threshold (mu_A + 3 sigma_A)
        int    persistK = default(6);        // k consecutive samples above tau
        double cusumKappa = default(2.2);    // slack, just above benign mean of A
        double cusumH = default(60);         // alarm limit

		@signal[anomalyObserverSpoofed](type=double);
		@statistic[anomalyObserverSpoofed](record=stats,histogram,vector);
		
		 @signal[cusumStat](type=double);                 // C_t every sample (for tuning h)
        @statistic[cusumStat](record=max,stats);
        @signal[persistRun](type=double);                // current run length above tau (for tuning k)
        @statistic[persistRun](record=max,histogram);
        @signal[eventWhileObserverSpoofed](type=double); // events raised BY a spoofed observer
        @statistic[eventWhileObserverSpoofed](record=count);

        @signal[persistenceFalse](type=double);
        @signal[persistenceDelay](type=double);
        @signal[cusumFalse](type=double);
        @signal[cusumDelay](type=double);
        @statistic[persistenceFalse](record=count);
        @statistic[persistenceDelay](record=stats);
        @statistic[cusumFalse](record=count);
        @statistic[cusumDelay](record=stats);

        @signal[spoofOffset](type=double);
        @signal[anomalyTargetSpoofed](type=double);
        @signal[anomalyTargetClean](type=double);
        @statistic[spoofOffset](record=vector);
        @statistic[anomalyTargetSpoofed](record=stats,histogram,vector);
        @statistic[anomalyTargetClean](record=stats,histogram,vector);

        @signal[selfResidual](type=double);
        @signal[motionResidual](type=double);
        @signal[relResidual](type=double);
        @signal[anomaly](type=double);
        @statistic[selfResidual](record=stats,histogram,vector);
        @statistic[motionResidual](record=stats,histogram,vector);
        @statistic[relResidual](record=stats,histogram,vector);
        @statistic[anomaly](record=stats,histogram,vector);

        @display("i=misc/node_vs");

    gates:
        inout port[];
        inout up[];
}