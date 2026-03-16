#ifndef RAMULATOR_CONTROLLER_BANK_MACHINE_H
#define RAMULATOR_CONTROLLER_BANK_MACHINE_H

#include <cstdint>

#include "base/base.h"
#include "dram/dram.h"
#include "dram_controller/bank_state_scoreboard.h"

namespace Ramulator {

class BankMachine {
 public:
  enum class TransitionType {
    kInvalid = 0,
    kOpen,
    kAccess,
    kClose,
    kRefresh,
    kOther,
  };

  struct IssuePlan {
    int issue_command = -1;
    bool completes_request = false;
    bool forced_autoprecharge = false;
    TransitionType transition = TransitionType::kInvalid;
  };

  void setup(IDRAM* dram);

  void attach_scoreboard(const BankStateScoreboard* scoreboard) {
    m_scoreboard = scoreboard;
  }

  void configure_scoreboard_autoprecharge(uint32_t cap, int cmd_rd, int cmd_wr,
                                          int cmd_rda, int cmd_wra);

  IssuePlan build_issue_plan(const Request& req, const ProbeResult& probe,
                             Clk_t clk) const;

 private:
  IDRAM* m_dram = nullptr;
  const BankStateScoreboard* m_scoreboard = nullptr;
  uint32_t m_scoreboard_autoprecharge_cap = 0;
  int m_cmd_rd = -1;
  int m_cmd_wr = -1;
  int m_cmd_rda = -1;
  int m_cmd_wra = -1;

  bool should_force_autoprecharge(const Request& req,
                                  const ProbeResult& probe, Clk_t clk) const;
  TransitionType classify_transition(int issue_command) const;
};

}  // namespace Ramulator

#endif  // RAMULATOR_CONTROLLER_BANK_MACHINE_H
