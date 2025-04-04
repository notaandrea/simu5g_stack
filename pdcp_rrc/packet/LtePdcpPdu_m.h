//
// Generated file, do not edit! Created by opp_msgtool 6.1 from stack/pdcp_rrc/packet/LtePdcpPdu.msg.
//

#ifndef __SIMU5G_LTEPDCPPDU_M_H
#define __SIMU5G_LTEPDCPPDU_M_H

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

class LtePdcpPdu;

}  // namespace simu5g

#include "inet/common/packet/chunk/Chunk_m.h" // import inet.common.packet.chunk.Chunk


namespace simu5g {

/**
 * Class generated from <tt>stack/pdcp_rrc/packet/LtePdcpPdu.msg:20</tt> by opp_msgtool.
 * <pre>
 * //
 * // This is the PDCP message flowing through LTE stack.
 * //
 * class LtePdcpPdu extends inet::FieldsChunk
 * {
 *     chunkLength = inet::B(1); // TODO: size 0
 * }
 * </pre>
 */
class LtePdcpPdu : public ::inet::FieldsChunk
{
  protected:

  private:
    void copy(const LtePdcpPdu& other);

  protected:
    bool operator==(const LtePdcpPdu&) = delete;

  public:
    LtePdcpPdu();
    LtePdcpPdu(const LtePdcpPdu& other);
    virtual ~LtePdcpPdu();
    LtePdcpPdu& operator=(const LtePdcpPdu& other);
    virtual LtePdcpPdu *dup() const override {return new LtePdcpPdu(*this);}
    virtual void parsimPack(omnetpp::cCommBuffer *b) const override;
    virtual void parsimUnpack(omnetpp::cCommBuffer *b) override;
};

inline void doParsimPacking(omnetpp::cCommBuffer *b, const LtePdcpPdu& obj) {obj.parsimPack(b);}
inline void doParsimUnpacking(omnetpp::cCommBuffer *b, LtePdcpPdu& obj) {obj.parsimUnpack(b);}


}  // namespace simu5g


namespace omnetpp {

template<> inline simu5g::LtePdcpPdu *fromAnyPtr(any_ptr ptr) { return check_and_cast<simu5g::LtePdcpPdu*>(ptr.get<cObject>()); }

}  // namespace omnetpp

#endif // ifndef __SIMU5G_LTEPDCPPDU_M_H

