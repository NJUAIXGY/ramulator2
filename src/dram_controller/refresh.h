#ifndef     RAMULATOR_CONTROLLER_REFRESH_H
#define     RAMULATOR_CONTROLLER_REFRESH_H

#include <cstdint>
#include <vector>
#include <string>

#include "base/base.h"


namespace Ramulator {

struct RefreshScheduleHint {
  bool valid = false;
  uint64_t next_refresh_deadline_cycles = 0;
  uint64_t refresh_interval_cycles = 0;
};

class IRefreshManager {
  RAMULATOR_REGISTER_INTERFACE(IRefreshManager, "RefreshManager", "Refresh Manager Interface.");

  public:
    virtual void tick() = 0;
    virtual RefreshScheduleHint query_refresh_schedule_hint() const {
      return RefreshScheduleHint {};
    }
};

}        // namespace Ramulator


#endif   // RAMULATOR_CONTROLLER_REFRESH_H
