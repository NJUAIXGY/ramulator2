#include <vector>

#include "base/base.h"
#include "dram_controller/bh_controller.h"
#include "dram_controller/bh_scheduler.h"
#include "dram_controller/impl/plugin/blockhammer/blockhammer.h"

namespace Ramulator {

namespace {
constexpr int kReadyScratchpadIdx = 0;
}

class BlockingScheduler : public IBHScheduler, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IBHScheduler, BlockingScheduler, "BlockingScheduler", "Blocking DRAM Scheduler.")

  private:
    IBHDRAMController* m_ctrl = nullptr;
    IBlockHammer* m_bh;

    int m_clk = -1;

    // stats
    int s_num_blacklist = 0;

    bool m_is_debug; 

  public:
    void init() override {
    }

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      m_ctrl = cast_parent<IBHDRAMController>();
      m_bh = m_ctrl->get_plugin<IBlockHammer>();
      if (!m_bh) {
        std::cout << "BlockHammer scheduler requires BlockHammer plugin enabled!" << std::endl;
        std::exit(0); 
      }
    }

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
      while (candidate != buffer.end() && !m_bh->is_act_safe(*candidate)) {
        candidate++;
      }

      if (candidate == buffer.end()) {
        return buffer.end();
      }

      // std::next(candidate, 1)
      for (auto next = std::next(buffer.begin(), 1); next != buffer.end(); next++) {
        if (!m_bh->is_act_safe(*next)) {
          continue;
        }
        candidate = compare(candidate, next);
      }
      return candidate;
    }

    virtual void tick() override {
      m_clk++;
    }
};

}       // namespace Ramulator
