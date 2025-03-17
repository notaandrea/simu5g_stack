//
//                  Simu5G
//
// Authors: Giovanni Nardini, Giovanni Stea, Antonio Virdis (University of Pisa)
//
// This file is part of a software released under the license included in file
// "license.pdf". Please read LICENSE and README files before using it.
// The above files and the present reference are part of the software itself,
// and cannot be removed from it.
//

#include <assert.h>
#include "stack/phy/LtePhyUe.h"


#include "stack/ip2nic/IP2Nic.h"
#include "stack/mac/LteMacEnb.h"
#include "stack/phy/packet/LteFeedbackPkt.h"
#include "stack/phy/feedback/LteDlFeedbackGenerator.h"
#include "common/LteCommon.h"

namespace simu5g {

Define_Module(LtePhyUe);

using namespace inet;

simsignal_t LtePhyUe::distanceSignal_ = registerSignal("distance");
simsignal_t LtePhyUe::servingCellSignal_ = registerSignal("servingCell");

LtePhyUe::~LtePhyUe()
{
    cancelAndDelete(handoverStarter_);
    cancelAndDelete(handoverTrigger_);
    delete das_;
}

struct NRMCSelem
{
    LteMod mod_;       /// modulation (Qm)
    double coderate_;  /// code rate (R)

    NRMCSelem(LteMod mod = _QPSK, double coderate = 0.0) : mod_(mod), coderate_(coderate)
    {
    }

};

class NRMcsTable
{
  public:
    /**
     * <CQI Index [0-15]> , <Modulation> , <Code Rate x 1024>
     * This table contains values taken from table 7.2.3-1 (TS 38.214)
     */
    CQIelem cqiTable[MAXCQI + 1] = {cqiTable[0] = CQIelem(_QPSK, 0.0),
                                    cqiTable[1] = CQIelem(_QPSK, 78.0),
                                    cqiTable[2] = CQIelem(_QPSK, 193.0),
                                    cqiTable[3] = CQIelem(_QPSK, 449.0),
                                    cqiTable[4] = CQIelem(_16QAM, 378.0),
                                    cqiTable[5] = CQIelem(_16QAM, 490.0),
                                    cqiTable[6] = CQIelem(_16QAM, 616.0),
                                    cqiTable[7] = CQIelem(_64QAM, 466.0),
                                    cqiTable[8] = CQIelem(_64QAM, 567.0),
                                    cqiTable[9] = CQIelem(_64QAM, 666.0),
                                    cqiTable[10] = CQIelem(_64QAM, 772.0),
                                    cqiTable[11] = CQIelem(_64QAM, 873.0),
                                    cqiTable[12] = CQIelem(_256QAM, 711.0),
                                    cqiTable[13] = CQIelem(_256QAM, 797.0),
                                    cqiTable[14] = CQIelem(_256QAM, 885.0),
                                    cqiTable[15] = CQIelem(_256QAM, 948.0)};

    NRMCSelem table[CQI2ITBSSIZE] =    {table[0] = NRMCSelem(_QPSK, 120.0),
                                        table[1] = NRMCSelem(_QPSK, 193.0),
                                        table[2] = NRMCSelem(_QPSK, 308.0),
                                        table[3] = NRMCSelem(_QPSK, 449.0),
                                        table[4] = NRMCSelem(_QPSK, 602.0),
                                        table[5] = NRMCSelem(_16QAM, 378.0),
                                        table[6] = NRMCSelem(_16QAM, 434.0),
                                        table[7] = NRMCSelem(_16QAM, 490.0),
                                        table[8] = NRMCSelem(_16QAM, 553.0),
                                        table[9] = NRMCSelem(_16QAM, 616.0),
                                        table[10] = NRMCSelem(_16QAM, 658.0),
                                        table[11] = NRMCSelem(_64QAM, 466.0),
                                        table[12] = NRMCSelem(_64QAM, 517.0),
                                        table[13] = NRMCSelem(_64QAM, 567.0),
                                        table[14] = NRMCSelem(_64QAM, 616.0),
                                        table[15] = NRMCSelem(_64QAM, 666.0),
                                        table[16] = NRMCSelem(_64QAM, 719.0),
                                        table[17] = NRMCSelem(_64QAM, 772.0),
                                        table[18] = NRMCSelem(_64QAM, 822.0),
                                        table[19] = NRMCSelem(_64QAM, 873.0),
                                        table[20] = NRMCSelem(_256QAM, 682.5),
                                        table[21] = NRMCSelem(_256QAM, 711.0),
                                        table[22] = NRMCSelem(_256QAM, 754.0),
                                        table[23] = NRMCSelem(_256QAM, 797.0),
                                        table[24] = NRMCSelem(_256QAM, 841.0),
                                        table[25] = NRMCSelem(_256QAM, 885.0),
                                        table[26] = NRMCSelem(_256QAM, 916.5),
                                        table[27] = NRMCSelem(_256QAM, 948.0)};

    CQIelem getCqiElem(int i)
    {
        return cqiTable[i];
    }

    unsigned int getMinIndex(LteMod mod);
    unsigned int getMaxIndex(LteMod mod);

    /// MCS table seek operator
    NRMCSelem& at(Tbs tbs)
    {
        return table[tbs];
    }

};


void LtePhyUe::initialize(int stage)
{
    LtePhyBase::initialize(stage);

    if (stage == inet::INITSTAGE_LOCAL) {
        isNr_ = false;        // this might be true only if this module is a NrPhyUe
        nodeType_ = UE;
        enableHandover_ = par("enableHandover");
        handoverLatency_ = par("handoverLatency").doubleValue();
        handoverDetachment_ = handoverLatency_ / 2.0;                      // TODO: make this configurable from NED
        handoverAttachment_ = handoverLatency_ - handoverDetachment_;
        dynamicCellAssociation_ = par("dynamicCellAssociation");
        if (par("minRssiDefault").boolValue())
            minRssi_ = binder_->phyPisaData.minSnr();
        else
            minRssi_ = par("minRssi").doubleValue();

        currentMasterRssi_ = -999.0;
        candidateMasterRssi_ = -999.0;
        hysteresisTh_ = 0;
        hysteresisFactor_ = 10;
        handoverDelta_ = 0.00001;

        dasRssiThreshold_ = 1.0e-5;
        das_ = new DasFilter(this, binder_, nullptr, dasRssiThreshold_);

        hasCollector = par("hasCollector");

        if (!hasListeners(averageCqiDlSignal_))
            throw cRuntimeError("no phy listeners");

        WATCH(nodeType_);
        WATCH(masterId_);
        WATCH(candidateMasterId_);
        WATCH(dasRssiThreshold_);
        WATCH(currentMasterRssi_);
        WATCH(candidateMasterRssi_);
        WATCH(hysteresisTh_);
        WATCH(hysteresisFactor_);
        WATCH(handoverDelta_);
    }
    else if (stage == inet::INITSTAGE_PHYSICAL_ENVIRONMENT) {
        txPower_ = ueTxPower_;

        lastFeedback_ = 0;

        handoverStarter_ = new cMessage("handoverStarter");

        // get the reference to the MAC module
        mac_ = check_and_cast<LteMacUe *>(gate(upperGateOut_)->getPathEndGate()->getOwnerModule());

        rlcUm_.reference(this, "rlcUmModule", true);
        pdcp_.reference(this, "pdcpRrcModule", true);
        ip2nic_.reference(this, "ip2nicModule", true);
        fbGen_.reference(this, "feedbackGeneratorModule", true);

        // get local id
        if (isNr_)
            nodeId_ = MacNodeId(hostModule->par("nrMacNodeId").intValue());
        else
            nodeId_ = MacNodeId(hostModule->par("macNodeId").intValue());
        EV << "Local MacNodeId: " << nodeId_ << endl;
    }
    else if (stage == inet::INITSTAGE_PHYSICAL_LAYER) {
        // get serving cell from configuration
        // TODO find a more elegant way
        if (isNr_)
            masterId_ = MacNodeId(hostModule->par("nrMasterId").intValue());
        else
            masterId_ = MacNodeId(hostModule->par("masterId").intValue());
        candidateMasterId_ = masterId_;

        // find the best candidate master cell
        if (dynamicCellAssociation_) {
            // this is a fictitious frame that needs to compute the SINR
            LteAirFrame *frame = new LteAirFrame("cellSelectionFrame");
            UserControlInfo *cInfo = new UserControlInfo();

            // get the list of all eNodeBs in the network
            std::vector<EnbInfo *> *enbList = binder_->getEnbList();
            for (auto enbInfo : *enbList) {
                // the NR phy layer only checks signal from gNBs
                if (isNr_ && enbInfo->nodeType != GNODEB)
                    continue;

                // the LTE phy layer only checks signal from eNBs
                if (!isNr_ && enbInfo->nodeType != ENODEB)
                    continue;

                MacNodeId cellId = enbInfo->id;
                LtePhyBase *cellPhy = check_and_cast<LtePhyBase *>(enbInfo->eNodeB->getSubmodule("cellularNic")->getSubmodule("phy"));
                double cellTxPower = cellPhy->getTxPwr();
                Coord cellPos = cellPhy->getCoord();

                // check whether the BS supports the carrier frequency used by the UE
                double ueCarrierFrequency = primaryChannelModel_->getCarrierFrequency();
                LteChannelModel *cellChannelModel = cellPhy->getChannelModel(ueCarrierFrequency);
                if (cellChannelModel == nullptr)
                    continue;

                // build a control info
                cInfo->setSourceId(cellId);
                cInfo->setTxPower(cellTxPower);
                cInfo->setCoord(cellPos);
                cInfo->setFrameType(BROADCASTPKT);
                cInfo->setDirection(DL);

                // get RSSI from the BS
                double rssi = 0;
                std::vector<double> rssiV = primaryChannelModel_->getRSRP(frame, cInfo);
                for (auto value : rssiV)
                    rssi += value;
                rssi /= rssiV.size();   // compute the mean over all RBs

                EV << "LtePhyUe::initialize - RSSI from cell " << cellId << ": " << rssi << " dB (current candidate cell " << candidateMasterId_ << ": " << candidateMasterRssi_ << " dB)" << endl;

                if (rssi > candidateMasterRssi_) {
                    candidateMasterId_ = cellId;
                    candidateMasterRssi_ = rssi;
                }
            }
            delete cInfo;
            delete frame;

            // binder calls
            // if dynamicCellAssociation selected a different master
            if (candidateMasterId_ != NODEID_NONE && candidateMasterId_ != masterId_) {
                binder_->unregisterNextHop(masterId_, nodeId_);
                binder_->registerNextHop(candidateMasterId_, nodeId_);
            }
            masterId_ = candidateMasterId_;
            // set serving cell
            if (isNr_)
                hostModule->par("nrMasterId").setIntValue(num(masterId_));
            else
                hostModule->par("masterId").setIntValue(num(masterId_));
            currentMasterRssi_ = candidateMasterRssi_;
            updateHysteresisTh(candidateMasterRssi_);
        }

        EV << "LtePhyUe::initialize - Attaching to eNodeB " << masterId_ << endl;

        das_->setMasterRuSet(masterId_);
        emit(servingCellSignal_, (long)masterId_);

        if (masterId_ == NODEID_NONE)
            masterMobility_ = nullptr;
        else {
            cModule *masterModule = binder_->getModuleByMacNodeId(masterId_);
            masterMobility_ = check_and_cast<IMobility *>(masterModule->getSubmodule("mobility"));
        }
    }
    else if (stage == inet::INITSTAGE_NETWORK_CONFIGURATION) {
        // get cellInfo at this stage because the next hop of the node is registered in the IP2Nic module at the INITSTAGE_NETWORK_LAYER
        if (masterId_ != NODEID_NONE) {
            cellInfo_ = getCellInfo(binder_, nodeId_);
            int index = intuniform(0, binder_->phyPisaData.maxChannel() - 1);
            if (cellInfo_ != nullptr) {
                cellInfo_->lambdaInit(nodeId_, index);
                cellInfo_->channelUpdate(nodeId_, intuniform(1, binder_->phyPisaData.maxChannel2()));
            }
        }
        else
            cellInfo_ = nullptr;
    }
}

void LtePhyUe::handleSelfMessage(cMessage *msg)
{
    if (msg->isName("handoverStarter"))
        triggerHandover();
    else if (msg->isName("handoverTrigger")) {
        doHandover();
        delete msg;
        handoverTrigger_ = nullptr;
    }
}

void LtePhyUe::handoverHandler(LteAirFrame *frame, UserControlInfo *lteInfo)
{
    lteInfo->setDestId(nodeId_);
    if (!enableHandover_) {
        // Even if handover is not enabled, this call is necessary
        // to allow Reporting Set computation.
        if (getNodeTypeById(lteInfo->getSourceId()) == ENODEB && lteInfo->getSourceId() == masterId_) {
            // Broadcast message from my master enb
            das_->receiveBroadcast(frame, lteInfo);
        }

        delete frame;
        delete lteInfo;
        return;
    }

    frame->setControlInfo(lteInfo);
    double rssi;

    if (getNodeTypeById(lteInfo->getSourceId()) == ENODEB && lteInfo->getSourceId() == masterId_) {
        // Broadcast message from my master enb
        rssi = das_->receiveBroadcast(frame, lteInfo);
    }
    else {
        // Broadcast message from not-master enb
        rssi = 0;
        std::vector<double> rssiV = primaryChannelModel_->getSINR(frame, lteInfo);
        for (auto value : rssiV)
            rssi += value;
        rssi /= rssiV.size();
    }

    EV << "UE " << nodeId_ << " broadcast frame from " << lteInfo->getSourceId() << " with RSSI: " << rssi << " at " << simTime() << endl;

    if (lteInfo->getSourceId() != masterId_ && rssi < minRssi_) {
        EV << "Signal too weak from a candidate master - minRssi[" << minRssi_ << "]" << endl;
        delete frame;
        return;
    }

    if (rssi > candidateMasterRssi_ + hysteresisTh_) {
        if (lteInfo->getSourceId() == masterId_) {
            // receiving even stronger broadcast from current master
            currentMasterRssi_ = rssi;
            candidateMasterId_ = masterId_;
            candidateMasterRssi_ = rssi;
            hysteresisTh_ = updateHysteresisTh(currentMasterRssi_);
            cancelEvent(handoverStarter_);
        }
        else {
            // broadcast from another master with higher RSSI
            candidateMasterId_ = lteInfo->getSourceId();
            candidateMasterRssi_ = rssi;
            hysteresisTh_ = updateHysteresisTh(rssi);
            binder_->addHandoverTriggered(nodeId_, masterId_, candidateMasterId_);

            // schedule self message to evaluate handover parameters after
            // all broadcast messages have arrived
            if (!handoverStarter_->isScheduled()) {
                // all broadcast messages are scheduled at the very same time, a small delta
                // guarantees the ones belonging to the same turn have been received
                scheduleAt(simTime() + handoverDelta_, handoverStarter_);
            }
        }
    }
    else {
        if (lteInfo->getSourceId() == masterId_) {
            if (rssi >= minRssi_) {
                currentMasterRssi_ = rssi;
                candidateMasterRssi_ = rssi;
                hysteresisTh_ = updateHysteresisTh(rssi);
            }
            else { // lost connection from current master
                if (candidateMasterId_ == masterId_) { // trigger detachment
                    candidateMasterId_ = NODEID_NONE;
                    currentMasterRssi_ = -999.0;
                    candidateMasterRssi_ = -999.0; // set candidate RSSI very bad as we currently do not have any.
                                                   // this ensures that each candidate with is at least as 'bad'
                                                   // as the minRssi_ has a chance.

                    hysteresisTh_ = updateHysteresisTh(0);
                    binder_->addHandoverTriggered(nodeId_, masterId_, candidateMasterId_);

                    if (!handoverStarter_->isScheduled()) {
                        // all broadcast messages are scheduled at the very same time, a small delta
                        // guarantees the ones belonging to the same turn have been received
                        scheduleAt(simTime() + handoverDelta_, handoverStarter_);
                    }
                }
                // else do nothing, a stronger RSSI from another nodeB has been found already
            }
        }
    }

    delete frame;
}

void LtePhyUe::triggerHandover()
{
    ASSERT(masterId_ != candidateMasterId_);

    EV << "####Handover starting:####" << endl;
    EV << "current master: " << masterId_ << endl;
    EV << "current RSSI: " << currentMasterRssi_ << endl;
    EV << "candidate master: " << candidateMasterId_ << endl;
    EV << "candidate RSSI: " << candidateMasterRssi_ << endl;
    EV << "############" << endl;

    if (candidateMasterRssi_ == 0)
        EV << NOW << " LtePhyUe::triggerHandover - UE " << nodeId_ << " lost its connection to eNB " << masterId_ << ". Now detaching... " << endl;
    else if (masterId_ == NODEID_NONE)
        EV << NOW << " LtePhyUe::triggerHandover - UE " << nodeId_ << " is starting attachment procedure to eNB " << candidateMasterId_ << "... " << endl;
    else
        EV << NOW << " LtePhyUe::triggerHandover - UE " << nodeId_ << " is starting handover to eNB " << candidateMasterId_ << "... " << endl;

    binder_->addUeHandoverTriggered(nodeId_);

    // inform the UE's IP2Nic module to start holding downstream packets
    ip2nic_->triggerHandoverUe(candidateMasterId_);
    binder_->removeHandoverTriggered(nodeId_);

    // inform the eNB's IP2Nic module to forward data to the target eNB
    if (masterId_ != NODEID_NONE && candidateMasterId_ != NODEID_NONE) {
        IP2Nic *enbIp2Nic = check_and_cast<IP2Nic *>(getSimulation()->getModule(binder_->getOmnetId(masterId_))->getSubmodule("cellularNic")->getSubmodule("ip2nic"));
        enbIp2Nic->triggerHandoverSource(nodeId_, candidateMasterId_);
    }

    double handoverLatency;
    if (masterId_ == NODEID_NONE)                                                // attachment only
        handoverLatency = handoverAttachment_;
    else if (candidateMasterId_ == NODEID_NONE)                                                // detachment only
        handoverLatency = handoverDetachment_;
    else
        handoverLatency = handoverDetachment_ + handoverAttachment_;

    handoverTrigger_ = new cMessage("handoverTrigger");
    scheduleAt(simTime() + handoverLatency, handoverTrigger_);
}

void LtePhyUe::doHandover()
{
    // if masterId_ == 0, it means the UE was not attached to any eNodeB, so it only has to perform attachment procedures
    // if candidateMasterId_ == 0, it means the UE is detaching from its eNodeB, so it only has to perform detachment procedures

    if (masterId_ != NODEID_NONE) {
        // Delete Old Buffers
        deleteOldBuffers(masterId_);

        // amc calls
        LteAmc *oldAmc = getAmcModule(masterId_);
        oldAmc->detachUser(nodeId_, UL);
        oldAmc->detachUser(nodeId_, DL);
    }

    if (candidateMasterId_ != NODEID_NONE) {
        LteAmc *newAmc = getAmcModule(candidateMasterId_);
        assert(newAmc != nullptr);
        newAmc->attachUser(nodeId_, UL);
        newAmc->attachUser(nodeId_, DL);
    }

    // binder calls
    if (masterId_ != NODEID_NONE)
        binder_->unregisterNextHop(masterId_, nodeId_);

    if (candidateMasterId_ != NODEID_NONE) {
        binder_->registerNextHop(candidateMasterId_, nodeId_);
        das_->setMasterRuSet(candidateMasterId_);
    }
    binder_->updateUeInfoCellId(nodeId_, candidateMasterId_);

    // @author Alessandro Noferi
    if (hasCollector) {
        binder_->moveUeCollector(nodeId_, masterId_, candidateMasterId_);
    }

    // change masterId and notify handover to the MAC layer
    MacNodeId oldMaster = masterId_;
    masterId_ = candidateMasterId_;
    mac_->doHandover(candidateMasterId_);  // do MAC operations for handover
    currentMasterRssi_ = candidateMasterRssi_;
    hysteresisTh_ = updateHysteresisTh(currentMasterRssi_);

    // update NED parameter
    if (isNr_)
        hostModule->par("nrMasterId").setIntValue(num(masterId_));
    else
        hostModule->par("masterId").setIntValue(num(masterId_));

    // update reference to master node's mobility module
    if (masterId_ == NODEID_NONE)
        masterMobility_ = nullptr;
    else {
        cModule *masterModule = binder_->getModuleByMacNodeId(masterId_);
        masterMobility_ = check_and_cast<IMobility *>(masterModule->getSubmodule("mobility"));
    }

    // update cellInfo
    if (masterId_ != NODEID_NONE)
        cellInfo_->detachUser(nodeId_);

    if (candidateMasterId_ != NODEID_NONE) {
        CellInfo *oldCellInfo = cellInfo_;
        LteMacEnb *newMacEnb = check_and_cast<LteMacEnb *>(getSimulation()->getModule(binder_->getOmnetId(candidateMasterId_))->getSubmodule("cellularNic")->getSubmodule("mac"));
        CellInfo *newCellInfo = newMacEnb->getCellInfo();
        newCellInfo->attachUser(nodeId_);
        cellInfo_ = newCellInfo;
        if (oldCellInfo == nullptr) {
            // first time the UE is attached to someone
            int index = intuniform(0, binder_->phyPisaData.maxChannel() - 1);
            cellInfo_->lambdaInit(nodeId_, index);
            cellInfo_->channelUpdate(nodeId_, intuniform(1, binder_->phyPisaData.maxChannel2()));
        }
    }

    // update DL feedback generator
    fbGen_->handleHandover(masterId_);

    // collect stat
    emit(servingCellSignal_, (long)masterId_);

    if (masterId_ == NODEID_NONE)
        EV << NOW << " LtePhyUe::doHandover - UE " << nodeId_ << " detached from the network" << endl;
    else
        EV << NOW << " LtePhyUe::doHandover - UE " << nodeId_ << " has completed handover to eNB " << masterId_ << "... " << endl;
    binder_->removeUeHandoverTriggered(nodeId_);

    // inform the UE's IP2Nic module to forward held packets
    ip2nic_->signalHandoverCompleteUe();

    // inform the eNB's IP2Nic module to forward data to the target eNB
    if (oldMaster != NODEID_NONE && candidateMasterId_ != NODEID_NONE) {
        IP2Nic *enbIp2Nic = check_and_cast<IP2Nic *>(getSimulation()->getModule(binder_->getOmnetId(masterId_))->getSubmodule("cellularNic")->getSubmodule("ip2nic"));
        enbIp2Nic->signalHandoverCompleteTarget(nodeId_, oldMaster);
    }
}

// TODO: ***reorganize*** method
void LtePhyUe::handleAirFrame(cMessage *msg)
{
    UserControlInfo *lteInfo = dynamic_cast<UserControlInfo *>(msg->removeControlInfo());

    connectedNodeId_ = masterId_;
    LteAirFrame *frame = check_and_cast<LteAirFrame *>(msg);
    EV << "LtePhy: received new LteAirFrame with ID " << frame->getId() << " from channel" << endl;

    int sourceId = binder_->getOmnetId(lteInfo->getSourceId());
    if (sourceId == 0) {
        // source has left the simulation
        delete msg;
        return;
    }

    // check if the air frame was sent on a correct carrier frequency
    double carrierFreq = lteInfo->getCarrierFrequency();
    LteChannelModel *channelModel = getChannelModel(carrierFreq);
    if (channelModel == nullptr) {
        EV << "Received packet on carrier frequency not supported by this node. Delete it." << endl;
        delete lteInfo;
        delete frame;
        return;
    }

    //Update coordinates of this user
    if (lteInfo->getFrameType() == HANDOVERPKT) {
        // check if the message is on another carrier frequency or handover is already in process
        if (carrierFreq != primaryChannelModel_->getCarrierFrequency() || (handoverTrigger_ != nullptr && handoverTrigger_->isScheduled())) {
            EV << "Received handover packet on a different carrier frequency than the primary cell. Delete it." << endl;
            delete lteInfo;
            delete frame;
            return;
        }

        handoverHandler(frame, lteInfo);
        return;
    }

    // Check if the frame is for us ( MacNodeId matches )
    if (lteInfo->getDestId() != nodeId_) {
        EV << "ERROR: Frame is not for us. Delete it." << endl;
        EV << "Packet Type: " << phyFrameTypeToA((LtePhyFrameType)lteInfo->getFrameType()) << endl;
        EV << "Frame MacNodeId: " << lteInfo->getDestId() << endl;
        EV << "Local MacNodeId: " << nodeId_ << endl;
        delete lteInfo;
        delete frame;
        return;
    }

    /*
     * This could happen if the ue associates with a new master while the old one
     * already scheduled a packet for him: the packet is in the air while the ue changes master.
     * Event timing:      TTI x: packet scheduled and sent for UE (tx time = 1ms)
     *                     TTI x+0.1: ue changes master
     *                     TTI x+1: packet from old master arrives at ue
     */
    if (lteInfo->getSourceId() != masterId_) {
        EV << "WARNING: frame from an old master during handover: deleted " << endl;
        EV << "Source MacNodeId: " << lteInfo->getSourceId() << endl;
        EV << "Master MacNodeId: " << masterId_ << endl;
        delete frame;
        return;
    }

    // send H-ARQ feedback up
    if (lteInfo->getFrameType() == HARQPKT || lteInfo->getFrameType() == GRANTPKT || lteInfo->getFrameType() == RACPKT) {
        handleControlMsg(frame, lteInfo);
        return;
    }
    if ((lteInfo->getUserTxParams()) != nullptr) {
        int cw = lteInfo->getCw();
        if (lteInfo->getUserTxParams()->readCqiVector().size() == 1)
            cw = 0;
        double cqi = lteInfo->getUserTxParams()->readCqiVector()[cw];
        emit(averageCqiDlSignal_, cqi);
        recordCqi(cqi, DL);
    }

    //----------------------------------------------------------------------------------- MY CODE -------------------------------------------------------------------------------------------------
    if ((lteInfo->getUserTxParams()) != nullptr) {
        int cw = lteInfo->getCw();
        if (lteInfo->getUserTxParams()->readCqiVector().size() == 1)
            cw = 0;
        double cqi = lteInfo->getUserTxParams()->readCqiVector()[cw];
        //FIND MCS STARTING FROM THE CQI

        // CQI threshold table selection
        NRMcsTable mcsTable;

        CQIelem entry = mcsTable.getCqiElem(cqi);
        LteMod mod = entry.mod_;
        double rate = entry.rate_;

        // Select the ranges for searching in the McsTable (extended reporting supported)
        unsigned int min = mcsTable.getMinIndex(mod);
        unsigned int max = mcsTable.getMaxIndex(mod);

        // Initialize the working variables at the minimum value.
        NRMCSelem ret = mcsTable.at(min);
        // Initialize the MCS Index at the minimum value
        unsigned int index = min;

        // Search in the McsTable from min to max until the rate exceeds
        // the coderate in an entry of the table.
        for (unsigned int i = min; i <= max; i++) {
            NRMCSelem elem = mcsTable.at(i);
            if (elem.coderate_ <= rate){
                ret = elem;
                index = i;
            }else{
                break;
            }
        }
        emit(averageMCSDlSignal_, index);
        recordMCS(index, DL);
    }

    // apply decider to received packet
    bool result = true;
    RemoteSet r = lteInfo->getUserTxParams()->readAntennaSet();
    if (r.size() > 1) {
        // DAS
        for (auto it : r) {
            EV << "LtePhy: Receiving Packet from antenna " << it << "\n";

            /*
             * On UE set the sender position
             * and tx power to the sender das antenna
             */

            RemoteUnitPhyData data;
            data.txPower = lteInfo->getTxPower();
            data.m = getRadioPosition();
            frame->addRemoteUnitPhyDataVector(data);
        }
        // apply analog models For DAS
        result = channelModel->isErrorDas(frame, lteInfo);
    }
    else {
        result = channelModel->isError(frame, lteInfo);
    }

    // update statistics
    if (result)
        numAirFrameReceived_++;
    else
        numAirFrameNotReceived_++;

    EV << "Handled LteAirframe with ID " << frame->getId() << " with result "
       << (result ? "RECEIVED" : "NOT RECEIVED") << endl;

    auto pkt = check_and_cast<inet::Packet *>(frame->decapsulate());

    // here frame has to be destroyed since it is no longer useful
    delete frame;

    // attach the decider result to the packet as control info
    lteInfo->setDeciderResult(result);
    *(pkt->addTagIfAbsent<UserControlInfo>()) = *lteInfo;
    delete lteInfo;

    // send decapsulated message along with result control info to upperGateOut_
    send(pkt, upperGateOut_);

    if (getEnvir()->isGUI())
        updateDisplayString();
}

void LtePhyUe::handleUpperMessage(cMessage *msg)
{
    auto pkt = check_and_cast<inet::Packet *>(msg);
    auto lteInfo = pkt->getTag<UserControlInfo>();

    MacNodeId dest = lteInfo->getDestId();
    if (dest != masterId_) {
        // UE is not sending to its master!!
        throw cRuntimeError("LtePhyUe::handleUpperMessage  Ue preparing to send message to %hu instead of its master (%hu)", num(dest), num(masterId_));
    }

    double carrierFreq = lteInfo->getCarrierFrequency();
    LteChannelModel *channelModel = getChannelModel(carrierFreq);
    if (channelModel == nullptr)
        throw cRuntimeError("LtePhyUe::handleUpperMessage - Carrier frequency [%f] not supported by any channel model", carrierFreq);

    if (lteInfo->getFrameType() == DATAPKT && (channelModel->isUplinkInterferenceEnabled() || channelModel->isD2DInterferenceEnabled())) {
        // Store the RBs used for data transmission to the binder (for UL interference computation)
        RbMap rbMap = lteInfo->getGrantedBlocks();
        Remote antenna = MACRO;  // TODO fix for multi-antenna
        binder_->storeUlTransmissionMap(channelModel->getCarrierFrequency(), antenna, rbMap, nodeId_, mac_->getMacCellId(), this, UL);
    }

    if (lteInfo->getFrameType() == DATAPKT && lteInfo->getUserTxParams() != nullptr) {
        double cqi = lteInfo->getUserTxParams()->readCqiVector()[lteInfo->getCw()];
        if (lteInfo->getDirection() == UL) {
            emit(averageCqiUlSignal_, cqi);
            recordCqi(cqi, UL);
        }
        else if (lteInfo->getDirection() == D2D)
            emit(averageCqiD2DSignal_, cqi);
    }

    LtePhyBase::handleUpperMessage(msg);
}

void LtePhyUe::emitMobilityStats()
{
    // emit serving cell id
    emit(servingCellSignal_, (long)masterId_);

    if (masterMobility_) {
        // emit distance from current serving cell
        inet::Coord masterPos = masterMobility_->getCurrentPosition();
        double distance = getRadioPosition().distance(masterPos);
        emit(distanceSignal_, distance);
    }
}

double LtePhyUe::updateHysteresisTh(double v)
{
    if (hysteresisFactor_ == 0)
        return 0;
    else
        return v / hysteresisFactor_;
}

void LtePhyUe::deleteOldBuffers(MacNodeId masterId)
{
    // Delete Mac Buffers

    // delete macBuffer[nodeId_] at old master
    LteMacEnb *masterMac = check_and_cast<LteMacEnb *>(getMacByMacNodeId(binder_, masterId));
    masterMac->deleteQueues(nodeId_);

    // delete queues for master at this ue
    mac_->deleteQueues(masterId_);

    // Delete Rlc UM Buffers

    // delete UmTxQueue[nodeId_] at old master
    LteRlcUm *masterRlcUm = check_and_cast<LteRlcUm *>(getRlcByMacNodeId(binder_, masterId, UM));
    masterRlcUm->deleteQueues(nodeId_);

    // delete queues for master at this ue
    rlcUm_->deleteQueues(nodeId_);

    // Delete PDCP Entities
    // delete pdcpEntities[nodeId_] at old master
    LtePdcpRrcEnb *masterPdcp = check_and_cast<LtePdcpRrcEnb *>(getPdcpByMacNodeId(binder_, masterId));
    masterPdcp->deleteEntities(nodeId_);

    // delete queues for master at this ue
    pdcp_->deleteEntities(masterId_);
}

DasFilter *LtePhyUe::getDasFilter()
{
    return das_;
}

void LtePhyUe::sendFeedback(LteFeedbackDoubleVector fbDl, LteFeedbackDoubleVector fbUl, FeedbackRequest req)
{
    Enter_Method("SendFeedback");
    EV << "LtePhyUe: feedback from Feedback Generator" << endl;

    //Create a feedback packet
    auto fbPkt = makeShared<LteFeedbackPkt>();
    //Set the feedback
    fbPkt->setLteFeedbackDoubleVectorDl(fbDl);
    fbPkt->setLteFeedbackDoubleVectorDl(fbUl);
    fbPkt->setSourceNodeId(nodeId_);

    auto pkt = new Packet("feedback_pkt");
    pkt->insertAtFront(fbPkt);

    UserControlInfo *uinfo = new UserControlInfo();
    uinfo->setSourceId(nodeId_);
    uinfo->setDestId(masterId_);
    uinfo->setFrameType(FEEDBACKPKT);
    uinfo->setIsCorruptible(false);
    // create LteAirFrame and encapsulate a feedback packet
    LteAirFrame *frame = new LteAirFrame("feedback_pkt");
    frame->encapsulate(check_and_cast<cPacket *>(pkt));
    uinfo->feedbackReq = req;
    uinfo->setDirection(UL);
    simtime_t signalLength = TTI;
    uinfo->setTxPower(txPower_);
    // initialize frame fields

    frame->setSchedulingPriority(airFramePriority_);
    frame->setDuration(signalLength);

    uinfo->setCoord(getRadioPosition());

    //TODO access speed data Update channel index
    lastFeedback_ = NOW;

    // send one feedback packet for each carrier
    for (auto& cm : channelModel_) {
        double carrierFrequency = cm.first;
        LteAirFrame *carrierFrame = frame->dup();
        UserControlInfo *carrierInfo = uinfo->dup();
        carrierInfo->setCarrierFrequency(carrierFrequency);
        carrierFrame->setControlInfo(carrierInfo);

        EV << "LtePhy: " << nodeTypeToA(nodeType_) << " with id "
           << nodeId_ << " sending feedback to the air channel for carrier " << carrierFrequency << endl;
        sendUnicast(carrierFrame);
    }

    delete frame;
    delete uinfo;
}

void LtePhyUe::recordMCS(unsigned int sample, Direction dir)
{
    if (dir == DL) {
        mcsDlSamples_.push_back(sample);
        mcsDlSum_ += sample;
        mcsDlCount_++;
    }
    if (dir == UL) {
        mcsUlSamples_.push_back(sample);
        mcsUlSum_ += sample;
        mcsUlCount_++;
    }
}

void LtePhyUe::recordTBS(unsigned int sample, Direction dir)
{
    if (dir == DL) {
        tbsDlSamples_.push_back(sample);
        tbsDlSum_ += sample;
        tbsDlCount_++;
    }
    if (dir == UL) {
        tbsUlSamples_.push_back(sample);
        tbsUlSum_ += sample;
        tbsUlCount_++;
    }
}

void LtePhyUe::recordCqi(unsigned int sample, Direction dir)
{
    if (dir == DL) {
        cqiDlSamples_.push_back(sample);
        cqiDlSum_ += sample;
        cqiDlCount_++;
    }
    if (dir == UL) {
        cqiUlSamples_.push_back(sample);
        cqiUlSum_ += sample;
        cqiUlCount_++;
    }
}

double LtePhyUe::getAverageMCS(Direction dir)
{
    if (dir == DL) {
        if (mcsDlCount_ == 0)
            return 0;
        return (double)mcsDlSum_ / mcsDlCount_;
    }
    if (dir == UL) {
        if (mcsUlCount_ == 0)
            return 0;
        return (double)mcsUlSum_ / mcsUlCount_;
    }
    throw cRuntimeError("Direction %d is not handled.", dir);
}

double LtePhyUe::getAverageTBS(Direction dir)
{
    if (dir == DL) {
        if (tbsDlCount_ == 0)
            return 0;
        return (double)tbsDlSum_ / tbsDlCount_;
    }
    if (dir == UL) {
        if (tbsUlCount_ == 0)
            return 0;
        return (double)tbsUlSum_ / tbsUlCount_;
    }
    throw cRuntimeError("Direction %d is not handled.", dir);
}

double LtePhyUe::getAverageCqi(Direction dir)
{
    if (dir == DL) {
        if (cqiDlCount_ == 0)
            return 0;
        return (double)cqiDlSum_ / cqiDlCount_;
    }
    if (dir == UL) {
        if (cqiUlCount_ == 0)
            return 0;
        return (double)cqiUlSum_ / cqiUlCount_;
    }
    throw cRuntimeError("Direction %d is not handled.", dir);
}

double LtePhyUe::getVarianceMCS(Direction dir)
{
    double avgMCS = getAverageMCS(dir);
    double err, sum = 0;

    if (dir == DL) {
        for (short & mcsDlSample : mcsDlSamples_) {
            err = avgMCS - mcsDlSample;
            sum += (err * err);
        }
        return sum / mcsDlSamples_.size();
    }
    if (dir == UL) {
        for (short & mcsUlSample : mcsUlSamples_) {
            err = avgMCS - mcsUlSample;
            sum += (err * err);
        }
        return sum / mcsUlSamples_.size();
    }
    throw cRuntimeError("Direction %d is not handled.", dir);
}

double LtePhyUe::getVarianceTBS(Direction dir)
{
    double avgTBS = getAverageTBS(dir);
    double err, sum = 0;

    if (dir == DL) {
        for (short & tbsDlSample : tbsDlSamples_) {
            err = avgTBS - tbsDlSample;
            sum += (err * err);
        }
        return sum / tbsDlSamples_.size();
    }
    if (dir == UL) {
        for (short & tbsUlSample : tbsUlSamples_) {
            err = avgTBS - tbsUlSample;
            sum += (err * err);
        }
        return sum / tbsUlSamples_.size();
    }
    throw cRuntimeError("Direction %d is not handled.", dir);
}

double LtePhyUe::getVarianceCqi(Direction dir)
{
    double avgCqi = getAverageCqi(dir);
    double err, sum = 0;

    if (dir == DL) {
        for (short & cqiDlSample : cqiDlSamples_) {
            err = avgCqi - cqiDlSample;
            sum += (err * err);
        }
        return sum / cqiDlSamples_.size();
    }
    if (dir == UL) {
        for (short & cqiUlSample : cqiUlSamples_) {
            err = avgCqi - cqiUlSample;
            sum += (err * err);
        }
        return sum / cqiUlSamples_.size();
    }
    throw cRuntimeError("Direction %d is not handled.", dir);
}

void LtePhyUe::finish()
{
    if (getSimulation()->getSimulationStage() != CTX_FINISH) {
        // do this only during the deletion of the module during the simulation

        // do this only if this PHY layer is connected to a serving base station
        if (masterId_ != NODEID_NONE) {
            // clear buffers
            deleteOldBuffers(masterId_);

            // amc calls
            LteAmc *amc = getAmcModule(masterId_);
            if (amc != nullptr) {
                amc->detachUser(nodeId_, UL);
                amc->detachUser(nodeId_, DL);
            }

            // binder call
            binder_->unregisterNextHop(masterId_, nodeId_);

            // cellInfo call
            cellInfo_->detachUser(nodeId_);
        }
    }
}

} //namespace

