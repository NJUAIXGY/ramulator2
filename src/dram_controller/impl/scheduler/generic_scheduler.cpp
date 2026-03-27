#include <vector>

#include "base/base.h"
#include "dram_controller/controller.h"
#include "dram_controller/scheduler.h"

namespace Ramulator {

namespace {
constexpr int kReadyScratchpadIdx = 0;
}

class FRFCFS : public IScheduler, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IScheduler, FRFCFS, "FRFCFS", "FRFCFS DRAM Scheduler.")
  private:
    IDRAMController* m_ctrl = nullptr;

  public:
    void init() override { };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      m_ctrl = cast_parent<IDRAMController>();
    };

    ReqBuffer::iterator compare(ReqBuffer::iterator req1, ReqBuffer::iterator req2) override {
      bool ready1 = req1->scratchpad[kReadyScratchpadIdx] != 0;
      bool ready2 = req2->scratchpad[kReadyScratchpadIdx] != 0;

      if (ready1 ^ ready2) {
        if (ready1) {
          return req1;
        } else {
          return req2;
        }
      }

      // Fallback to FCFS
      if (req1->arrive <= req2->arrive) {
        return req1;
      } else {
        return req2;
      } 
    }

    ReqBuffer::iterator get_best_request(ReqBuffer& buffer) override {
      if (buffer.size() == 0) {
        return buffer.end();
      }

      for (auto& req : buffer) {
        ControllerCommandState command_state {};
        if (m_ctrl->query_command_state(req.final_command, req.addr_vec,
                                        command_state)) {
          req.command = command_state.next_command;
          req.scratchpad[kReadyScratchpadIdx] =
              command_state.next_command_ready ? 1 : 0;
        } else {
          req.command = -1;
          req.scratchpad[kReadyScratchpadIdx] = 0;
        }
      }

      auto candidate = buffer.begin();
      for (auto next = std::next(buffer.begin(), 1); next != buffer.end(); next++) {
        candidate = compare(candidate, next);
      }
      return candidate;
    }
};

}       // namespace Ramulator
