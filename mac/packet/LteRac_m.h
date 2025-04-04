//
// Generated file, do not edit! Created by opp_msgtool 6.1 from stack/mac/packet/LteRac.msg.
//

#ifndef __SIMU5G_LTERAC_M_H
#define __SIMU5G_LTERAC_M_H

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

class LteRac;

}  // namespace simu5g

#include "inet/common/INETDefs_m.h" // import inet.common.INETDefs

#include "inet/common/packet/chunk/Chunk_m.h" // import inet.common.packet.chunk.Chunk


namespace simu5g {

/**
 * Class generated from <tt>stack/mac/packet/LteRac.msg:17</tt> by opp_msgtool.
 * <pre>
 * class LteRac extends inet::FieldsChunk
 * {
 *     // meaningful only for DL (response) RAC packets : if true RAC request has been admitted.
 *     bool success;
 *     chunkLength = inet::B(1); // TODO: size 0
 * }
 * </pre>
 */
class LteRac : public ::inet::FieldsChunk
{
  protected:
    bool success = false;

  private:
    void copy(const LteRac& other);

  protected:
    bool operator==(const LteRac&) = delete;

  public:
    LteRac();
    LteRac(const LteRac& other);
    virtual ~LteRac();
    LteRac& operator=(const LteRac& other);
    virtual LteRac *dup() const override {return new LteRac(*this);}
    virtual void parsimPack(omnetpp::cCommBuffer *b) const override;
    virtual void parsimUnpack(omnetpp::cCommBuffer *b) override;

    virtual bool getSuccess() const;
    virtual void setSuccess(bool success);
};

inline void doParsimPacking(omnetpp::cCommBuffer *b, const LteRac& obj) {obj.parsimPack(b);}
inline void doParsimUnpacking(omnetpp::cCommBuffer *b, LteRac& obj) {obj.parsimUnpack(b);}


}  // namespace simu5g


namespace omnetpp {

template<> inline simu5g::LteRac *fromAnyPtr(any_ptr ptr) { return check_and_cast<simu5g::LteRac*>(ptr.get<cObject>()); }

}  // namespace omnetpp

#endif // ifndef __SIMU5G_LTERAC_M_H

