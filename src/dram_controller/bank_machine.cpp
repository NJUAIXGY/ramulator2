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
    const Request& req, const ProbeResult& probe, Clk_t clk) const {
  IssuePlan plan {};
  if (!m_dram) {
    return plan;
  }

  plan.issue_command = req.command;
  plan.completes_request = (req.command == req.final_command);

  if (should_force_autoprecharge(req, probe, clk)) {
    if (req.type_id == Request::Type::Read) {
      plan.issue_command = m_cmd_rda;
    } else if (req.type_id == Request::Type::Write) {
      plan.issue_command = m_cmd_wra;
    }
    plan.completes_request = true;
    plan.forced_autoprecharge = true;
  }

  plan.transition = classify_transition(plan.issue_command);
  return plan;
}

bool BankMachine::should_force_autoprecharge(const Request& req,
                                             const ProbeResult& probe,
                                             Clk_t clk) const {
  if (!m_dram) return false;
  if (m_scoreboard_autoprecharge_cap == 0) return false;
  if (req.command != req.final_command) return false;
  if (!probe.valid || !probe.row_hit || probe.refreshing) return false;

  const auto cmd_meta = m_dram->m_command_meta(req.command);
  if (!cmd_meta.is_accessing || cmd_meta.is_closing) return false;

  int target_command = -1;
  if (req.type_id == Request::Type::Read && req.command == m_cmd_rd) {
    target_command = m_cmd_rda;
  } else if (req.type_id == Request::Type::Write && req.command == m_cmd_wr) {
    target_command = m_cmd_wra;
  } else {
    return false;
  }

  if (probe.col_accesses_on_row + 1 <
      static_cast<uint64_t>(m_scoreboard_autoprecharge_cap)) {
    return false;
  }
  if (m_scoreboard && m_scoreboard->valid()) {
    if (!m_scoreboard->is_command_ready(m_dram, target_command, req.addr_vec,
                                        clk)) {
      return false;
    }
  } else if (!m_dram->check_ready(target_command, req.addr_vec)) {
    return false;
  }

  return true;
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
