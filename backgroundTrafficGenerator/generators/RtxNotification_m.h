//
// Generated file, do not edit! Created by opp_msgtool 6.1 from stack/backgroundTrafficGenerator/generators/RtxNotification.msg.
//

#ifndef __SIMU5G_RTXNOTIFICATION_M_H
#define __SIMU5G_RTXNOTIFICATION_M_H

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

class RtxNotification;

}  // namespace simu5g


namespace simu5g {

/**
 * Class generated from <tt>stack/backgroundTrafficGenerator/generators/RtxNotification.msg:15</tt> by opp_msgtool.
 * <pre>
 * message RtxNotification
 * {
 *     int bytes;       // number of bytes to be retransmitted
 *     int direction;   // DL or UL
 * }
 * </pre>
 */
class RtxNotification : public ::omnetpp::cMessage
{
  protected:
    int bytes = 0;
    int direction = 0;

  private:
    void copy(const RtxNotification& other);

  protected:
    bool operator==(const RtxNotification&) = delete;

  public:
    RtxNotification(const char *name=nullptr, short kind=0);
    RtxNotification(const RtxNotification& other);
    virtual ~RtxNotification();
    RtxNotification& operator=(const RtxNotification& other);
    virtual RtxNotification *dup() const override {return new RtxNotification(*this);}
    virtual void parsimPack(omnetpp::cCommBuffer *b) const override;
    virtual void parsimUnpack(omnetpp::cCommBuffer *b) override;

    virtual int getBytes() const;
    virtual void setBytes(int bytes);

    virtual int getDirection() const;
    virtual void setDirection(int direction);
};

inline void doParsimPacking(omnetpp::cCommBuffer *b, const RtxNotification& obj) {obj.parsimPack(b);}
inline void doParsimUnpacking(omnetpp::cCommBuffer *b, RtxNotification& obj) {obj.parsimUnpack(b);}


}  // namespace simu5g


namespace omnetpp {

template<> inline simu5g::RtxNotification *fromAnyPtr(any_ptr ptr) { return check_and_cast<simu5g::RtxNotification*>(ptr.get<cObject>()); }

}  // namespace omnetpp

#endif // ifndef __SIMU5G_RTXNOTIFICATION_M_H

