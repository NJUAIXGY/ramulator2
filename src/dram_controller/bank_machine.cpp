#include "dram_controller/bank_machine.h"

namespace Ramulator {

void BankMachine::setup(IDRAM* dram) { m_dram = dram; }

void BankMachine::configure_scoreboard_autoprecharge(uint32_t cap, int cmd_rd,
                                                      int cmd_wr, int cmd_rda,
                                                      int cmd_wra) {
  m_scoreboard_autoprecharge_cap = cap;
  m_cmd_rd = cmd_rd;
  m_cmd_wr = cmd_wr;
  m_cmd_rda = cmd_rda;
  m_cmd_wra = cmd_wra;
}

BankMachine::IssuePlan BankMachine::build_issue_plan(
    const Request& req, const SchedulingState& scheduling_state,
    Clk_t clk) const {
  IssuePlan plan {};
  if (!m_dram) {
    return plan;
  }

  plan.issue_command = req.command;
  plan.completes_request = (req.command == req.final_command);

  const int forced_autoprecharge_command =
      resolve_forced_autoprecharge_command(req, scheduling_state.bank_state,
                                           clk);
  if (forced_autoprecharge_command >= 0) {
    plan.issue_command = forced_autoprecharge_command;
    plan.completes_request = true;
    plan.forced_autoprecharge = true;
  }

  plan.transition = classify_transition(plan.issue_command);
  return plan;
}

int BankMachine::resolve_forced_autoprecharge_command(
    const Request& req, const BankStateSnapshot& bank_state,
    Clk_t clk) const {
  if (!m_dram) return -1;
  if (m_scoreboard_autoprecharge_cap == 0) return -1;
  if (m_scoreboard && m_scoreboard->valid()) {
    const ForcedAutoprechargeDecision decision =
        m_scoreboard->evaluate_forced_autoprecharge(
            m_dram, req, bank_state, clk, m_scoreboard_autoprecharge_cap);
    return decision.force ? decision.issue_command : -1;
  }

  if (req.command != req.final_command) return -1;
  if (!bank_state.valid || !bank_state.row_hit) return -1;
  if (bank_state.refreshing || bank_state.refresh_recovery) return -1;
  const auto cmd_meta = m_dram->m_command_meta(req.command);
  if (!cmd_meta.is_accessing || cmd_meta.is_closing) return -1;

  int target_command = -1;
  if (req.type_id == Request::Type::Read && req.command == m_cmd_rd) {
    target_command = m_cmd_rda;
  } else if (req.type_id == Request::Type::Write && req.command == m_cmd_wr) {
    target_command = m_cmd_wra;
  } else {
    return -1;
  }

  if (bank_state.col_accesses_on_row + 1 <
      static_cast<uint64_t>(m_scoreboard_autoprecharge_cap)) {
    return -1;
  }
  if (!m_dram->check_ready(target_command, req.addr_vec)) {
    return -1;
  }

  return target_command;
}

BankMachine::TransitionType BankMachine::classify_transition(
    int issue_command) const {
  if (!m_dram || issue_command < 0) {
    return TransitionType::kInvalid;
  }

  const auto meta = m_dram->m_command_meta(issue_command);
  if (meta.is_refreshing) {
    return TransitionType::kRefresh;
  }
  if (meta.is_opening) {
    return TransitionType::kOpen;
  }
  if (meta.is_accessing) {
    return TransitionType::kAccess;
  }
  if (meta.is_closing) {
    return TransitionType::kClose;
  }
  return TransitionType::kOther;
}

}  // namespace Ramulator
