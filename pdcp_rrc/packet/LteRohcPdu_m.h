//
// Generated file, do not edit! Created by opp_msgtool 6.1 from stack/pdcp_rrc/packet/LteRohcPdu.msg.
//

#ifndef __SIMU5G_LTEROHCPDU_M_H
#define __SIMU5G_LTEROHCPDU_M_H

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

class LteRohcPdu;

}  // namespace simu5g

#include "inet/common/INETDefs_m.h" // import inet.common.INETDefs

#include "inet/common/packet/chunk/Chunk_m.h" // import inet.common.packet.chunk.Chunk

// cplusplus {{
   using namespace inet;
// }}


namespace simu5g {

/**
 * Class generated from <tt>stack/pdcp_rrc/packet/LteRohcPdu.msg:30</tt> by opp_msgtool.
 * <pre>
 * //
 * // This is the RObust Header Compression PDU
 * // 
 * // ROHC is modeled by reducing the compressed headers to a lower size
 * // as indicated by the headerCompressedSize_ parameter. The additional ROHC header
 * // is added an allows to restore the compressed headers to their full size when
 * // decompressing.
 * //
 * class LteRohcPdu extends inet::FieldsChunk
 * {
 *     inet::B origSizeTransportHeader;
 *     inet::B origSizeIpHeader;
 * }
 * </pre>
 */
class LteRohcPdu : public ::inet::FieldsChunk
{
  protected:
    ::inet::B origSizeTransportHeader = B(-1);
    ::inet::B origSizeIpHeader = B(-1);

  private:
    void copy(const LteRohcPdu& other);

  protected:
    bool operator==(const LteRohcPdu&) = delete;

  public:
    LteRohcPdu();
    LteRohcPdu(const LteRohcPdu& other);
    virtual ~LteRohcPdu();
    LteRohcPdu& operator=(const LteRohcPdu& other);
    virtual LteRohcPdu *dup() const override {return new LteRohcPdu(*this);}
    virtual void parsimPack(omnetpp::cCommBuffer *b) const override;
    virtual void parsimUnpack(omnetpp::cCommBuffer *b) override;

    virtual ::inet::B getOrigSizeTransportHeader() const;
    virtual void setOrigSizeTransportHeader(::inet::B origSizeTransportHeader);

    virtual ::inet::B getOrigSizeIpHeader() const;
    virtual void setOrigSizeIpHeader(::inet::B origSizeIpHeader);
};

inline void doParsimPacking(omnetpp::cCommBuffer *b, const LteRohcPdu& obj) {obj.parsimPack(b);}
inline void doParsimUnpacking(omnetpp::cCommBuffer *b, LteRohcPdu& obj) {obj.parsimUnpack(b);}


}  // namespace simu5g


namespace omnetpp {

template<> inline simu5g::LteRohcPdu *fromAnyPtr(any_ptr ptr) { return check_and_cast<simu5g::LteRohcPdu*>(ptr.get<cObject>()); }

}  // namespace omnetpp

#endif // ifndef __SIMU5G_LTEROHCPDU_M_H

