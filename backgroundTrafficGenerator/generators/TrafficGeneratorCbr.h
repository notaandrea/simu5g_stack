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

#ifndef TRAFFICGENERATORCBR_H_
#define TRAFFICGENERATORCBR_H_

#include "stack/backgroundTrafficGenerator/generators/TrafficGeneratorBase.h"

namespace simu5g {

//
// TrafficGeneratorCbr
//
class TrafficGeneratorCbr : public TrafficGeneratorBase
{
  protected:

    // message length (excluding headers)
    unsigned int size_[2];

    // inter-arrival time
    double period_[2];

    void initialize(int stage) override;

    // -- re-implemented functions from the base class -- //

    // generate a new message with length size_
    unsigned int generateTraffic(Direction dir) override;

    // returns the period_
    simtime_t getNextGenerationTime(Direction dir) override;

  public:

    // returns the average traffic generated by this BG UE in the given direction (in Bps)
    double getAvgLoad(Direction dir) override;

};

} //namespace

#endif

