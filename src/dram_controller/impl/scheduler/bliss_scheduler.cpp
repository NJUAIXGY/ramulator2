#include <vector>

#include "base/base.h"
#include "dram_controller/bh_controller.h"
#include "dram_controller/bh_scheduler.h"
#include "dram_controller/impl/plugin/bliss/bliss.h"

namespace Ramulator {

class BLISSScheduler : public IBHScheduler, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IBHScheduler, BLISSScheduler, "BLISS", "BLISS Scheduler.")

  private:
    IBHDRAMController* m_ctrl = nullptr;
    IDRAM* m_dram;
    IBLISS* m_bliss;

    int m_clk = -1;

    int m_req_rd = -1;
    int m_req_wr = -1;

    bool m_is_debug; 

    const int SAFE_IDX = 0;
    const int READY_IDX = 1;

  public:
    void init() override { }

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      m_ctrl = cast_parent<IBHDRAMController>();
      m_dram = m_ctrl->m_dram;
      m_bliss = m_ctrl->get_plugin<IBLISS>();

      m_req_rd = m_dram->m_requests("read");
      m_req_wr = m_dram->m_requests("write");

      if (!m_bliss) {
        throw ConfigurationError("[Ramulator::BLISSScheduler] Implementation requires BLISS plugin to be active.");
      }
    }

    ReqBuffer::iterator compare(ReqBuffer::iterator req1, ReqBuffer::iterator req2) override {
      bool safe1 = req1->scratchpad[SAFE_IDX];
      bool safe2 = req2->scratchpad[SAFE_IDX];
      
      if (safe1 ^ safe2) {
        if (safe1) {
          return req1;
        } else {
          return req2;
        }
      }

      bool ready1 = req1->scratchpad[READY_IDX];
      bool ready2 = req2->scratchpad[READY_IDX];

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
          req.scratchpad[READY_IDX] =
              command_state.next_command_ready ? 1 : 0;
        } else {
          req.command = -1;
          req.scratchpad[READY_IDX] = 0;
        }

        // Check if the request is safe to issue
        bool blisted = m_bliss->is_blacklisted(req.source_id);
        bool isrw = req.type_id == m_req_rd || req.type_id == m_req_wr;
        bool safe = !isrw || !blisted;
        req.scratchpad[SAFE_IDX] = safe;
      }

      auto candidate = buffer.begin();
      for (auto next = std::next(buffer.begin(), 1); next != buffer.end(); next++) {
        candidate = compare(candidate, next);
      }
      return candidate;
    }

    virtual void tick() override {
      m_clk++;
    }
};

}       // namespace Ramulator
