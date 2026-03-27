#include <vector>

#include "base/base.h"
#include "dram_controller/bh_controller.h"
#include "dram_controller/bh_scheduler.h"
#include "dram_controller/impl/plugin/prac/prac.h"

namespace Ramulator {

class PRACScheduler : public IBHScheduler, public Implementation {
RAMULATOR_REGISTER_IMPLEMENTATION(IBHScheduler, PRACScheduler, "PRACScheduler", "PRAC Scheduler.")

private:
    IBHDRAMController* m_ctrl;
    IPRAC* m_prac;

    std::unordered_map<int, int> lut_cycles_needed;

    Clk_t m_clk = 0;

    bool m_is_debug = false; 

    const int FITS_IDX = 0;
    const int READY_IDX = 1;

public:
    void init() override {
        m_is_debug = param<bool>("debug").default_val(false);
    }

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
        m_ctrl = cast_parent<IBHDRAMController>();
        m_prac = m_ctrl->get_plugin<IPRAC>();

        if (!m_prac) {
            std::cout << "[RAMULATOR::PRACSched] Need PRAC plugin!" << std::endl;
            std::exit(0);
        }
    }

    ReqBuffer::iterator compare(ReqBuffer::iterator req1, ReqBuffer::iterator req2) override {
        bool fits1 = req1->scratchpad[FITS_IDX];
        bool fits2 = req2->scratchpad[FITS_IDX];

        if (fits1 ^ fits2) {
            if (fits1) {
                return req1;
            }
            else {
                return req2;
            }
        }

        bool ready1 = req1->scratchpad[READY_IDX];
        bool ready2 = req2->scratchpad[READY_IDX];

        if (ready1 ^ ready2) {
            if (ready1) {
                return req1;
            }
            else {
                return req2;
            }
        }

        if (req1->arrive <= req2->arrive) {
            return req1;
        }
        else {
            return req2;
        } 
    }

    ReqBuffer::iterator get_best_request(ReqBuffer& buffer) override {
        if (buffer.size() == 0) {
            return buffer.end();
        }

        Clk_t next_recovery = m_prac->next_recovery_cycle();
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
            req.scratchpad[FITS_IDX] = m_clk + m_prac->min_cycles_with_preall(req) < next_recovery;
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
