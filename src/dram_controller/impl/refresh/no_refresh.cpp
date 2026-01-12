#include "base/base.h"
#include "dram_controller/refresh.h"

namespace Ramulator {

class NoRefresh final : public IRefreshManager, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IRefreshManager, NoRefresh, "None", "Disable periodic DRAM refresh (NO-OP)")
public:
  void init() override {}
  void setup(IFrontEnd* /*frontend*/, IMemorySystem* /*memory_system*/) override {}
  void tick() override {}
};

} // namespace Ramulator

