//
// Generated file, do not edit! Created by opp_msgtool 6.1 from stack/mac/packet/LteHarqFeedback.msg.
//

#ifndef __SIMU5G_LTEHARQFEEDBACK_M_H
#define __SIMU5G_LTEHARQFEEDBACK_M_H

#if defined(__clang__)
#  pragma clang diagnostic ignored "-Wreserved-id-macro"
#endif
#include <omnetpp.h>

// opp_msgtool version check
#define MSGC_VERSION 0x0601
#if (MSGC_VERSION!=OMNETPP_VERSION)
#    error Version mismatch! Probably this file was generated by an earlier version of opp_msgtool: 'make clean' should help.
#endif


namespace simu5g {

class LteHarqFeedback;
class LteHarqFeedbackMirror;

}  // namespace simu5g

#include "inet/common/INETDefs_m.h" // import inet.common.INETDefs

#include "inet/common/packet/chunk/Chunk_m.h" // import inet.common.packet.chunk.Chunk

#include "common/LteCommon_m.h" // import common.LteCommon


namespace simu5g {

/**
 * Class generated from <tt>stack/mac/packet/LteHarqFeedback.msg:19</tt> by opp_msgtool.
 * <pre>
 * class LteHarqFeedback extends inet::FieldsChunk
 * {
 *     // H-ARQ acid to which this fb is addressed
 *     unsigned char acid;
 *     // H-ARQ cw id to which this fb is addressed
 *     unsigned char cw;
 *     // H-ARQ feedback: true for ACK, false for NACK
 *     bool result;
 *     // Id of the pdu to which the feedback is addressed
 *     long fbMacPduId;
 *     // if this flag is true, then the feedback refers to a D2D connection
 *     bool d2dFeedback = false;
 *     chunkLength = inet::B(1); // TODO: size 0
 * }
 * </pre>
 */
class LteHarqFeedback : public ::inet::FieldsChunk
{
  protected:
    unsigned char acid = 0;
    unsigned char cw = 0;
    bool result = false;
    long fbMacPduId = 0;
    bool d2dFeedback = false;

  private:
    void copy(const LteHarqFeedback& other);

  protected:
    bool operator==(const LteHarqFeedback&) = delete;

  public:
    LteHarqFeedback();
    LteHarqFeedback(const LteHarqFeedback& other);
    virtual ~LteHarqFeedback();
    LteHarqFeedback& operator=(const LteHarqFeedback& other);
    virtual LteHarqFeedback *dup() const override {return new LteHarqFeedback(*this);}
    virtual void parsimPack(omnetpp::cCommBuffer *b) const override;
    virtual void parsimUnpack(omnetpp::cCommBuffer *b) override;

    virtual unsigned char getAcid() const;
    virtual void setAcid(unsigned char acid);

    virtual unsigned char getCw() const;
    virtual void setCw(unsigned char cw);

    virtual bool getResult() const;
    virtual void setResult(bool result);

    virtual long getFbMacPduId() const;
    virtual void setFbMacPduId(long fbMacPduId);

    virtual bool getD2dFeedback() const;
    virtual void setD2dFeedback(bool d2dFeedback);
};

inline void doParsimPacking(omnetpp::cCommBuffer *b, const LteHarqFeedback& obj) {obj.parsimPack(b);}
inline void doParsimUnpacking(omnetpp::cCommBuffer *b, LteHarqFeedback& obj) {obj.parsimUnpack(b);}

/**
 * Class generated from <tt>stack/mac/packet/LteHarqFeedback.msg:34</tt> by opp_msgtool.
 * <pre>
 * class LteHarqFeedbackMirror extends LteHarqFeedback
 * {
 *     d2dFeedback = true;
 * 
 *     // Id of the D2D Transmitter
 *     MacNodeId d2dSenderId;
 *     // Id of the D2D Receiver
 *     MacNodeId d2dReceiverId;
 *     // Length (in bytes) of the corresponding PDU
 *     long pduLength;
 * }
 * </pre>
 */
class LteHarqFeedbackMirror : public ::simu5g::LteHarqFeedback
{
  protected:
    MacNodeId d2dSenderId;
    MacNodeId d2dReceiverId;
    long pduLength = 0;

  private:
    void copy(const LteHarqFeedbackMirror& other);

  protected:
    bool operator==(const LteHarqFeedbackMirror&) = delete;

  public:
    LteHarqFeedbackMirror();
    LteHarqFeedbackMirror(const LteHarqFeedbackMirror& other);
    virtual ~LteHarqFeedbackMirror();
    LteHarqFeedbackMirror& operator=(const LteHarqFeedbackMirror& other);
    virtual LteHarqFeedbackMirror *dup() const override {return new LteHarqFeedbackMirror(*this);}
    virtual void parsimPack(omnetpp::cCommBuffer *b) const override;
    virtual void parsimUnpack(omnetpp::cCommBuffer *b) override;

    virtual const MacNodeId& getD2dSenderId() const;
    virtual MacNodeId& getD2dSenderIdForUpdate() { handleChange();return const_cast<MacNodeId&>(const_cast<LteHarqFeedbackMirror*>(this)->getD2dSenderId());}
    virtual void setD2dSenderId(const MacNodeId& d2dSenderId);

    virtual const MacNodeId& getD2dReceiverId() const;
    virtual MacNodeId& getD2dReceiverIdForUpdate() { handleChange();return const_cast<MacNodeId&>(const_cast<LteHarqFeedbackMirror*>(this)->getD2dReceiverId());}
    virtual void setD2dReceiverId(const MacNodeId& d2dReceiverId);

    virtual long getPduLength() const;
    virtual void setPduLength(long pduLength);
};

inline void doParsimPacking(omnetpp::cCommBuffer *b, const LteHarqFeedbackMirror& obj) {obj.parsimPack(b);}
inline void doParsimUnpacking(omnetpp::cCommBuffer *b, LteHarqFeedbackMirror& obj) {obj.parsimUnpack(b);}


}  // namespace simu5g


namespace omnetpp {

template<> inline simu5g::LteHarqFeedback *fromAnyPtr(any_ptr ptr) { return check_and_cast<simu5g::LteHarqFeedback*>(ptr.get<cObject>()); }
template<> inline simu5g::LteHarqFeedbackMirror *fromAnyPtr(any_ptr ptr) { return check_and_cast<simu5g::LteHarqFeedbackMirror*>(ptr.get<cObject>()); }

}  // namespace omnetpp

#endif // ifndef __SIMU5G_LTEHARQFEEDBACK_M_H

