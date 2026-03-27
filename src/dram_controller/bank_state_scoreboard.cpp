#include "dram_controller/bank_state_scoreboard.h"

#include <algorithm>
#include <stdexcept>
#include <string_view>

namespace Ramulator {

namespace {

ProbeResult probe_result_from_bank_state(const BankStateSnapshot& bank_state) {
  ProbeResult result {};
  if (!bank_state.valid) {
    return result;
  }

  result.valid = true;
  result.row_open = bank_state.row_open;
  result.row_hit = bank_state.row_hit;
  result.refreshing = bank_state.refreshing || bank_state.refresh_recovery;
  result.col_accesses_on_row = bank_state.col_accesses_on_row;
  result.autoprecharge_armed = bank_state.autoprecharge_armed;
  return result;
}

}  // namespace

int BankStateScoreboard::try_level_index(IDRAM* dram, const char* level_name) {
  try {
    return dram->m_levels(level_name);
  } catch (const std::out_of_range&) {
    return -1;
  }
}

int BankStateScoreboard::try_command_index(IDRAM* dram,
                                           const char* command_name) {
  try {
    return dram->m_commands(command_name);
  } catch (const std::out_of_range&) {
    return -1;
  }
}

int BankStateScoreboard::addr_component(const AddrVec_t& addr_vec,
                                        int level_idx) {
  if (level_idx < 0) return -1;
  const size_t idx = static_cast<size_t>(level_idx);
  if (idx >= addr_vec.size()) return -1;
  return addr_vec[idx];
}

Clk_t BankStateScoreboard::clamp_non_negative(Clk_t value) {
  return value < 0 ? 0 : value;
}

Clk_t BankStateScoreboard::safe_add(Clk_t base, Clk_t delta) {
  return clamp_non_negative(base) + clamp_non_negative(delta);
}

Clk_t BankStateScoreboard::try_timing_value(IDRAM* dram,
                                            const char* timing_name) {
  try {
    const int value = dram->m_timing_vals(timing_name);
    if (value <= 0) return 0;
    return static_cast<Clk_t>(value);
  } catch (const std::out_of_range&) {
    return 0;
  }
}

bool BankStateScoreboard::resolve_iteration_bounds(const AddrVec_t& addr_vec,
                                                   int level_idx,
                                                   int level_size, int& begin,
                                                   int& end) const {
  if (level_idx < 0) {
    begin = 0;
    end = 1;
    return true;
  }
  if (level_size <= 0) {
    begin = 0;
    end = 0;
    return false;
  }

  const int value = addr_component(addr_vec, level_idx);
  if (value < 0) {
    begin = 0;
    end = level_size;
    return true;
  }
  if (value >= level_size) {
    begin = 0;
    end = 0;
    return false;
  }
  begin = value;
  end = value + 1;
  return true;
}

bool BankStateScoreboard::lookup_bank_components(const AddrVec_t& addr_vec,
                                                 int& rank, int& bg,
                                                 int& bank) const {
  rank = 0;
  bg = 0;
  bank = 0;

  if (!m_valid) return false;
  if (m_level_channel >= 0) {
    const int channel = addr_component(addr_vec, m_level_channel);
    if (channel >= 0 && channel != m_channel_id) return false;
  }

  if (m_level_rank >= 0) {
    rank = addr_component(addr_vec, m_level_rank);
    if (rank < 0 || rank >= m_num_ranks) return false;
  }
  if (m_level_bankgroup >= 0) {
    bg = addr_component(addr_vec, m_level_bankgroup);
    if (bg < 0 || bg >= m_num_bankgroups) return false;
  }
  if (m_level_bank >= 0) {
    bank = addr_component(addr_vec, m_level_bank);
    if (bank < 0 || bank >= m_num_banks) return false;
  }

  return true;
}

size_t BankStateScoreboard::flatten_index(int rank, int bg, int bank) const {
  return static_cast<size_t>(
      bank + bg * m_num_banks + rank * m_num_bankgroups * m_num_banks);
}

RefreshScopeKind BankStateScoreboard::command_scope_to_refresh_scope(
    IDRAM* dram, int command) const {
  try {
    const int scope_level = dram->m_command_scopes(command);
    const std::string_view scope_name = dram->m_levels(scope_level);
    if (scope_name == "bank") return RefreshScopeKind::kBank;
    if (scope_name == "bankgroup") return RefreshScopeKind::kBankGroup;
    if (scope_name == "rank") return RefreshScopeKind::kRank;
    if (scope_name == "channel") return RefreshScopeKind::kChannel;
  } catch (const std::out_of_range&) {
    return RefreshScopeKind::kNone;
  }
  return RefreshScopeKind::kNone;
}

void BankStateScoreboard::cache_command_indices(IDRAM* dram) {
  m_cmd_act = try_command_index(dram, "ACT");
  m_cmd_pre = try_command_index(dram, "PRE");
  m_cmd_prea = try_command_index(dram, "PREA");
  m_cmd_rd = try_command_index(dram, "RD");
  m_cmd_wr = try_command_index(dram, "WR");
  m_cmd_rda = try_command_index(dram, "RDA");
  m_cmd_wra = try_command_index(dram, "WRA");
  m_cmd_refab = try_command_index(dram, "REFab");
}

void BankStateScoreboard::cache_timing_values(IDRAM* dram) {
  m_timing_nrcd = try_timing_value(dram, "nRCD");
  m_timing_nras = try_timing_value(dram, "nRAS");
  m_timing_nrp = try_timing_value(dram, "nRP");
  m_timing_nrc = try_timing_value(dram, "nRC");
  m_timing_nrtp = try_timing_value(dram, "nRTP");
  m_timing_nwtr = try_timing_value(dram, "nWTR");
  m_timing_nrtw = try_timing_value(dram, "nRTW");
  m_timing_nrrds = try_timing_value(dram, "nRRDS");
  m_timing_nfaw = try_timing_value(dram, "nFAW");
  m_timing_nccds = try_timing_value(dram, "nCCDS");
  m_timing_ncwl = try_timing_value(dram, "nCWL");
  m_timing_nbl = try_timing_value(dram, "nBL");
  m_timing_nwr = try_timing_value(dram, "nWR");
  m_timing_nrfc = try_timing_value(dram, "nRFC");
  m_timing_nrefi = try_timing_value(dram, "nREFI");
}

bool BankStateScoreboard::is_read_like_command(int command) const {
  return command == m_cmd_rd || command == m_cmd_rda;
}

bool BankStateScoreboard::is_write_like_command(int command) const {
  return command == m_cmd_wr || command == m_cmd_wra;
}

bool BankStateScoreboard::any_open_row_in_scope(const AddrVec_t& addr_vec) const {
  if (!m_valid) return false;

  if (m_level_channel >= 0) {
    const int channel = addr_component(addr_vec, m_level_channel);
    if (channel >= 0 && channel != m_channel_id) return false;
  }

  int rank_begin = 0;
  int rank_end = 1;
  int bg_begin = 0;
  int bg_end = 1;
  int bank_begin = 0;
  int bank_end = 1;
  if (!resolve_iteration_bounds(addr_vec, m_level_rank, m_num_ranks, rank_begin,
                                rank_end))
    return false;
  if (!resolve_iteration_bounds(addr_vec, m_level_bankgroup, m_num_bankgroups,
                                bg_begin, bg_end))
    return false;
  if (!resolve_iteration_bounds(addr_vec, m_level_bank, m_num_banks, bank_begin,
                                bank_end))
    return false;

  for (int rank = rank_begin; rank < rank_end; ++rank) {
    for (int bg = bg_begin; bg < bg_end; ++bg) {
      for (int bank = bank_begin; bank < bank_end; ++bank) {
        const BankStateEntry& entry = m_bank_entries[flatten_index(rank, bg, bank)];
        if (entry.open_row_valid) {
          return true;
        }
      }
    }
  }

  return false;
}

RefreshStateSnapshot BankStateScoreboard::snapshot_refresh_state(
    const AddrVec_t& addr_vec, Clk_t clk) const {
  RefreshStateSnapshot result {};
  if (!m_valid) {
    return result;
  }

  int rank = 0;
  int bg = 0;
  int bank = 0;
  if (lookup_bank_components(addr_vec, rank, bg, bank)) {
    const BankStateEntry& entry = m_bank_entries[flatten_index(rank, bg, bank)];
    result.valid = true;
    result.pending = entry.refresh_pending;
    result.active = entry.refreshing;
    result.recovery = entry.refresh_recovery;
    result.owner_scope = entry.refresh_owner_scope;
    result.epoch = entry.refresh_epoch;
    const Clk_t blocked_until =
        std::max(entry.refresh_blocked_until, entry.recovery_blocked_until);
    if (blocked_until > clk) {
      result.horizon_cycles = static_cast<uint64_t>(blocked_until - clk);
    }
    return result;
  }

  const bool scope_known = m_refresh_scope_state.kind != RefreshScopeKind::kNone ||
                           m_refresh_scope_state.pending ||
                           m_refresh_scope_state.active ||
                           m_refresh_scope_state.epoch != 0;
  if (!scope_known) {
    return result;
  }

  result.valid = true;
  result.pending = m_refresh_scope_state.pending;
  result.active = m_refresh_scope_state.active;
  result.recovery = false;
  result.owner_scope = m_refresh_scope_state.kind;
  result.epoch = m_refresh_scope_state.epoch;
  if (m_refresh_scope_state.active && m_refresh_scope_state.enter_cycle >= 0 &&
      m_timing_nrfc > 0) {
    const Clk_t scope_blocked_until =
        safe_add(m_refresh_scope_state.enter_cycle, m_timing_nrfc);
    if (scope_blocked_until > clk) {
      result.horizon_cycles = static_cast<uint64_t>(scope_blocked_until - clk);
    }
  }
  return result;
}

BankStateSnapshot BankStateScoreboard::snapshot_bank_state(
    const AddrVec_t& addr_vec, Clk_t clk) const {
  BankStateSnapshot result {};
  if (!m_valid) {
    return result;
  }

  int rank = 0;
  int bg = 0;
  int bank = 0;
  if (!lookup_bank_components(addr_vec, rank, bg, bank)) {
    return result;
  }

  const BankStateEntry& entry = m_bank_entries[flatten_index(rank, bg, bank)];
  const int row = addr_component(addr_vec, m_level_row);

  result.valid = true;
  result.open_row_valid = entry.open_row_valid;
  result.open_row = entry.open_row_valid ? entry.open_row : -1;
  result.row_open = entry.open_row_valid && !entry.refreshing;
  result.row_hit =
      result.row_open && row >= 0 && entry.open_row_valid && entry.open_row == row;
  result.refresh_pending = entry.refresh_pending;
  result.refreshing = entry.refreshing;
  result.refresh_recovery = entry.refresh_recovery;
  result.autoprecharge_armed = entry.autoprecharge_armed;
  result.inflight_accesses = entry.inflight_accesses;
  result.col_accesses_on_row = entry.col_accesses_on_row;
  if (entry.open_row_valid && entry.open_since_cycle >= 0 &&
      clk >= entry.open_since_cycle) {
    result.open_age_cycles = static_cast<uint64_t>(clk - entry.open_since_cycle);
  }
  result.act_ready_in_cycles = entry.next_act_ready_at > clk
                                   ? static_cast<uint64_t>(entry.next_act_ready_at - clk)
                                   : 0;
  result.pre_ready_in_cycles = entry.next_pre_ready_at > clk
                                   ? static_cast<uint64_t>(entry.next_pre_ready_at - clk)
                                   : 0;
  result.read_ready_in_cycles =
      entry.next_read_ready_at > clk
          ? static_cast<uint64_t>(entry.next_read_ready_at - clk)
          : 0;
  result.write_ready_in_cycles =
      entry.next_write_ready_at > clk
          ? static_cast<uint64_t>(entry.next_write_ready_at - clk)
          : 0;
  result.refresh_epoch = entry.refresh_epoch;
  result.refresh_owner_scope = entry.refresh_owner_scope;
  return result;
}

BankStateScoreboard::ReadyResult BankStateScoreboard::evaluate_command_ready(
    IDRAM* dram, int command, const AddrVec_t& addr_vec, Clk_t clk) const {
  ReadyResult result {};
  result.block_reason = ReadyBlockReason::kNone;
  if (!m_valid || !dram) {
    result.block_reason = ReadyBlockReason::kScoreboardMiss;
    return result;
  }
  if (command < 0) {
    result.block_reason = ReadyBlockReason::kInvalidCommand;
    return result;
  }

  if (command == m_cmd_prea || command == m_cmd_refab) {
    bool has_target = false;
    for_each_target_rank_const(addr_vec,
                               [&](const RankTimingEntry& rank_entry, int) {
                                 if (has_target &&
                                     result.block_reason !=
                                         ReadyBlockReason::kNone) {
                                   return;
                                 }
                                 has_target = true;
                                 if (clk < rank_entry.refresh_active_until) {
                                   result.block_reason =
                                       ReadyBlockReason::kRankRefreshActive;
                                   return;
                                 }
                                 if (clk < rank_entry.refresh_recovery_until) {
                                   result.block_reason =
                                       ReadyBlockReason::kRankRefreshRecovery;
                                   return;
                                 }
                                 if (command == m_cmd_prea &&
                                     clk < rank_entry.next_prea_ready_at) {
                                   result.block_reason =
                                       ReadyBlockReason::kRankPrechargeTiming;
                                   return;
                                 }
                                 if (command == m_cmd_refab &&
                                     clk < rank_entry.next_ref_ready_at) {
                                   result.block_reason =
                                       ReadyBlockReason::kRankRefreshTiming;
                                 }
                               });
    if (!has_target) {
      result.block_reason = ReadyBlockReason::kAddressDecodeMiss;
      return result;
    }
    if (result.block_reason != ReadyBlockReason::kNone) {
      return result;
    }
    if (command == m_cmd_refab && any_open_row_in_scope(addr_vec)) {
      result.block_reason = ReadyBlockReason::kRefreshScopeOpenRows;
      return result;
    }
    result.ready = true;
    return result;
  }

  const bool is_bank_local_tracked =
      command == m_cmd_act || command == m_cmd_pre ||
      is_read_like_command(command) || is_write_like_command(command);
  if (!is_bank_local_tracked) {
    result.ready = true;
    result.block_reason = ReadyBlockReason::kNone;
    return result;
  }

  int rank = 0;
  int bg = 0;
  int bank = 0;
  if (!lookup_bank_components(addr_vec, rank, bg, bank)) {
    result.block_reason = ReadyBlockReason::kAddressDecodeMiss;
    return result;
  }

  const BankStateEntry& entry = m_bank_entries[flatten_index(rank, bg, bank)];
  const auto meta = dram->m_command_meta(command);

  if (!meta.is_refreshing) {
    if (clk < entry.refresh_blocked_until) {
      result.block_reason = ReadyBlockReason::kBankRefreshActive;
      return result;
    }
    if (clk < entry.recovery_blocked_until) {
      result.block_reason = ReadyBlockReason::kBankRefreshRecovery;
      return result;
    }
  }

  const int row = addr_component(addr_vec, m_level_row);
  if (command == m_cmd_act && entry.open_row_valid) {
    result.block_reason = ReadyBlockReason::kBankOpen;
    return result;
  }
  if (is_read_like_command(command) || is_write_like_command(command)) {
    if (!entry.open_row_valid) {
      result.block_reason = ReadyBlockReason::kBankClosed;
      return result;
    }
    if (row < 0 || entry.open_row != row) {
      result.block_reason = ReadyBlockReason::kRowConflict;
      return result;
    }
  }
  if (command == m_cmd_pre && !entry.open_row_valid) {
    result.block_reason = ReadyBlockReason::kBankClosed;
    return result;
  }

  if (command == m_cmd_act) {
    if (clk < m_layer_timing.next_act_ready_at) {
      result.block_reason = ReadyBlockReason::kActivateWindow;
      return result;
    }
    if (m_timing_nfaw > 0) {
      size_t window_acts = 0;
      for (const Clk_t act_clk : m_layer_timing.recent_act_cycles) {
        if (safe_add(act_clk, m_timing_nfaw) > clk) {
          window_acts++;
        }
      }
      if (window_acts >= 4) {
        result.block_reason = ReadyBlockReason::kFourActivateWindow;
        return result;
      }
    }
    if (clk < entry.next_act_ready_at) {
      result.block_reason = ReadyBlockReason::kBankTimingAct;
      return result;
    }
    result.ready = true;
    result.block_reason = ReadyBlockReason::kNone;
    return result;
  }
  if (command == m_cmd_pre) {
    if (clk < entry.next_pre_ready_at) {
      result.block_reason = ReadyBlockReason::kBankTimingPre;
      return result;
    }
    result.ready = true;
    result.block_reason = ReadyBlockReason::kNone;
    return result;
  }
  if (is_read_like_command(command)) {
    if (clk < m_layer_timing.next_col_issue_ready_at) {
      result.block_reason = ReadyBlockReason::kColumnBusTiming;
      return result;
    }
    if (clk < m_layer_timing.next_read_data_ready_at) {
      result.block_reason = ReadyBlockReason::kReadDataTiming;
      return result;
    }
    if (clk < m_layer_timing.next_read_ready_at) {
      result.block_reason = ReadyBlockReason::kReadTurnaroundTiming;
      return result;
    }
    if (clk < entry.next_read_ready_at) {
      result.block_reason = ReadyBlockReason::kBankTimingRead;
      return result;
    }
    result.ready = true;
    result.block_reason = ReadyBlockReason::kNone;
    return result;
  }
  if (is_write_like_command(command)) {
    if (clk < m_layer_timing.next_col_issue_ready_at) {
      result.block_reason = ReadyBlockReason::kColumnBusTiming;
      return result;
    }
    if (clk < m_layer_timing.next_write_data_ready_at) {
      result.block_reason = ReadyBlockReason::kWriteDataTiming;
      return result;
    }
    if (clk < m_layer_timing.next_write_ready_at) {
      result.block_reason = ReadyBlockReason::kWriteTurnaroundTiming;
      return result;
    }
    if (clk < entry.next_write_ready_at) {
      result.block_reason = ReadyBlockReason::kBankTimingWrite;
      return result;
    }
    result.ready = true;
    result.block_reason = ReadyBlockReason::kNone;
    return result;
  }

  result.ready = true;
  result.block_reason = ReadyBlockReason::kNone;
  return result;
}

void BankStateScoreboard::init_from_dram_org(IDRAM* dram, int channel_id) {
  m_channel_id = channel_id;

  m_level_channel = try_level_index(dram, "channel");
  m_level_rank = try_level_index(dram, "rank");
  m_level_tier = try_level_index(dram, "tier");
  if (m_level_rank < 0 && m_level_tier >= 0) {
    // Some DRAM orgs (e.g., Mono3DTiered) expose "tier" instead of "rank".
    // Treat tier as a rank-like dimension for bank/row state tracking.
    m_level_rank = m_level_tier;
  }
  m_level_bankgroup = try_level_index(dram, "bankgroup");
  m_level_bank = try_level_index(dram, "bank");
  m_level_row = try_level_index(dram, "row");

  if (m_level_bank < 0 || m_level_row < 0) {
    m_valid = false;
    m_bank_entries.clear();
    return;
  }

  const int num_ranks = dram->get_level_size("rank");
  const int num_tiers = dram->get_level_size("tier");
  if (num_ranks > 0) {
    m_num_ranks = num_ranks;
  } else if (num_tiers > 0) {
    m_num_ranks = num_tiers;
  } else {
    m_num_ranks = 1;
  }
  m_num_bankgroups = std::max(1, dram->get_level_size("bankgroup"));
  m_num_banks = std::max(1, dram->get_level_size("bank"));

  const size_t total_entries =
      static_cast<size_t>(m_num_ranks) * static_cast<size_t>(m_num_bankgroups) *
      static_cast<size_t>(m_num_banks);
  m_rank_entries.assign(static_cast<size_t>(m_num_ranks), RankTimingEntry {});
  m_bank_entries.assign(total_entries, BankStateEntry {});
  cache_command_indices(dram);
  cache_timing_values(dram);
  m_layer_timing = LayerTimingEntry {};
  m_refresh_scope_state = RefreshScopeState {};
  m_valid = true;
}

void BankStateScoreboard::on_refresh_scope_pending(
    RefreshScopeKind scope_kind, const AddrVec_t& scope_addr_vec, Clk_t clk) {
  if (!m_valid) return;
  m_refresh_scope_state.kind = scope_kind;
  m_refresh_scope_state.pending = true;
  m_refresh_scope_state.active = false;
  m_refresh_scope_state.scope_addr_vec = scope_addr_vec;
  m_refresh_scope_state.enter_cycle = clk;

  for_each_target_bank(scope_addr_vec, [&](BankStateEntry& entry, int, int, int) {
    entry.refresh_pending = true;
    entry.refresh_owner_scope = scope_kind;
  });
}

void BankStateScoreboard::on_refresh_scope_pending_from_command(
    IDRAM* dram, int command, const AddrVec_t& scope_addr_vec, Clk_t clk) {
  const RefreshScopeKind scope_kind =
      command_scope_to_refresh_scope(dram, command);
  on_refresh_scope_pending(scope_kind, scope_addr_vec, clk);
}

void BankStateScoreboard::on_refresh_enter(RefreshScopeKind scope_kind,
                                           const AddrVec_t& scope_addr_vec,
                                           Clk_t clk) {
  if (!m_valid) return;
  m_refresh_scope_state.kind = scope_kind;
  m_refresh_scope_state.pending = false;
  m_refresh_scope_state.active = true;
  m_refresh_scope_state.epoch++;
  m_refresh_scope_state.scope_addr_vec = scope_addr_vec;
  m_refresh_scope_state.enter_cycle = clk;

  const Clk_t refresh_until = safe_add(clk, m_timing_nrfc);
  for_each_target_rank(scope_addr_vec, [&](RankTimingEntry& rank_entry, int) {
    rank_entry.refresh_active_until =
        std::max(rank_entry.refresh_active_until, refresh_until);
    rank_entry.refresh_recovery_until =
        std::max(rank_entry.refresh_recovery_until, refresh_until);
    rank_entry.next_prea_ready_at =
        std::max(rank_entry.next_prea_ready_at, refresh_until);
    const Clk_t next_ref_by_interval = safe_add(clk, m_timing_nrefi);
    rank_entry.next_ref_ready_at =
        std::max(rank_entry.next_ref_ready_at, next_ref_by_interval);
  });

  for_each_target_bank(scope_addr_vec, [&](BankStateEntry& entry, int, int, int) {
    entry.refresh_pending = false;
    entry.refreshing = true;
    entry.refresh_recovery = false;
    entry.refresh_epoch = m_refresh_scope_state.epoch;
    entry.last_refresh_enter_cycle = clk;
    entry.refresh_owner_scope = scope_kind;
    entry.refresh_blocked_until = std::max(entry.refresh_blocked_until, refresh_until);
    entry.recovery_blocked_until =
        std::max(entry.recovery_blocked_until, refresh_until);
    entry.next_act_ready_at = std::max(entry.next_act_ready_at, refresh_until);
    entry.next_pre_ready_at = std::max(entry.next_pre_ready_at, refresh_until);
    entry.next_read_ready_at = std::max(entry.next_read_ready_at, refresh_until);
    entry.next_write_ready_at =
        std::max(entry.next_write_ready_at, refresh_until);
    entry.open_row_valid = false;
    entry.open_row = -1;
    entry.col_accesses_on_row = 0;
    entry.autoprecharge_armed = false;
  });
}

void BankStateScoreboard::on_refresh_exit(RefreshScopeKind scope_kind,
                                          const AddrVec_t& scope_addr_vec,
                                          Clk_t clk) {
  if (!m_valid) return;
  m_refresh_scope_state.kind = scope_kind;
  m_refresh_scope_state.pending = false;
  m_refresh_scope_state.active = false;
  m_refresh_scope_state.scope_addr_vec = scope_addr_vec;
  m_refresh_scope_state.exit_cycle = clk;

  for_each_target_rank(scope_addr_vec, [&](RankTimingEntry& rank_entry, int) {
    rank_entry.refresh_active_until = std::max(rank_entry.refresh_active_until, clk);
    rank_entry.refresh_recovery_until =
        std::max(rank_entry.refresh_recovery_until, clk);
  });

  for_each_target_bank(scope_addr_vec, [&](BankStateEntry& entry, int, int, int) {
    entry.refresh_pending = false;
    entry.refreshing = false;
    entry.refresh_recovery = true;
    entry.last_refresh_exit_cycle = clk;
    entry.refresh_owner_scope = scope_kind;
    entry.refresh_blocked_until = std::max(entry.refresh_blocked_until, clk);
    entry.recovery_blocked_until =
        std::max(entry.recovery_blocked_until, clk);
    entry.open_row_valid = false;
    entry.open_row = -1;
    entry.col_accesses_on_row = 0;
    entry.autoprecharge_armed = false;
  });
}

void BankStateScoreboard::on_issue_command(IDRAM* dram, int command,
                                           const AddrVec_t& addr_vec,
                                           Clk_t clk) {
  if (!m_valid) return;

  const auto meta = dram->m_command_meta(command);
  if (meta.is_refreshing) {
    const RefreshScopeKind scope_kind =
        command_scope_to_refresh_scope(dram, command);
    on_refresh_enter(scope_kind, addr_vec, clk);
    return;
  }

  const int row = addr_component(addr_vec, m_level_row);
  const int rank = (m_level_rank >= 0) ? addr_component(addr_vec, m_level_rank) : 0;
  auto update_rank_timing = [&](auto&& fn) {
    if (rank < 0 || rank >= m_num_ranks) {
      return;
    }
    fn(m_rank_entries[static_cast<size_t>(rank)]);
  };

  if (command == m_cmd_act) {
    m_layer_timing.next_act_ready_at =
        std::max(m_layer_timing.next_act_ready_at, safe_add(clk, m_timing_nrrds));
    if (m_timing_nfaw > 0) {
      while (!m_layer_timing.recent_act_cycles.empty()) {
        const Clk_t oldest = m_layer_timing.recent_act_cycles.front();
        if (safe_add(oldest, m_timing_nfaw) > clk) {
          break;
        }
        m_layer_timing.recent_act_cycles.pop_front();
      }
      m_layer_timing.recent_act_cycles.push_back(clk);
    }
    update_rank_timing([&](RankTimingEntry& rank_entry) {
      rank_entry.next_ref_ready_at =
          std::max(rank_entry.next_ref_ready_at, safe_add(clk, m_timing_nrc));
    });
  } else if (command == m_cmd_pre || command == m_cmd_prea) {
    update_rank_timing([&](RankTimingEntry& rank_entry) {
      rank_entry.next_ref_ready_at =
          std::max(rank_entry.next_ref_ready_at, safe_add(clk, m_timing_nrp));
    });
  } else if (command == m_cmd_rda) {
    m_layer_timing.next_col_issue_ready_at = std::max(
        m_layer_timing.next_col_issue_ready_at, safe_add(clk, m_timing_nccds));
    m_layer_timing.next_write_ready_at = std::max(
        m_layer_timing.next_write_ready_at, safe_add(clk, m_timing_nrtw));
    m_layer_timing.next_read_data_ready_at =
        std::max(m_layer_timing.next_read_data_ready_at,
                 safe_add(clk, m_timing_nbl));
    update_rank_timing([&](RankTimingEntry& rank_entry) {
      rank_entry.next_ref_ready_at = std::max(
          rank_entry.next_ref_ready_at,
          safe_add(clk, m_timing_nrtp + m_timing_nrp));
    });
  } else if (command == m_cmd_wra) {
    m_layer_timing.next_col_issue_ready_at = std::max(
        m_layer_timing.next_col_issue_ready_at, safe_add(clk, m_timing_nccds));
    m_layer_timing.next_read_ready_at = std::max(
        m_layer_timing.next_read_ready_at, safe_add(clk, m_timing_nwtr));
    m_layer_timing.next_write_data_ready_at =
        std::max(m_layer_timing.next_write_data_ready_at,
                 safe_add(clk, m_timing_nbl));
    update_rank_timing([&](RankTimingEntry& rank_entry) {
      rank_entry.next_ref_ready_at =
          std::max(rank_entry.next_ref_ready_at,
                   safe_add(clk, m_timing_ncwl + m_timing_nbl + m_timing_nwr +
                                    m_timing_nrp));
    });
  } else if (command == m_cmd_rd) {
    m_layer_timing.next_col_issue_ready_at = std::max(
        m_layer_timing.next_col_issue_ready_at, safe_add(clk, m_timing_nccds));
    m_layer_timing.next_write_ready_at = std::max(
        m_layer_timing.next_write_ready_at, safe_add(clk, m_timing_nrtw));
    m_layer_timing.next_read_data_ready_at =
        std::max(m_layer_timing.next_read_data_ready_at,
                 safe_add(clk, m_timing_nbl));
  } else if (command == m_cmd_wr) {
    m_layer_timing.next_col_issue_ready_at = std::max(
        m_layer_timing.next_col_issue_ready_at, safe_add(clk, m_timing_nccds));
    m_layer_timing.next_read_ready_at = std::max(
        m_layer_timing.next_read_ready_at, safe_add(clk, m_timing_nwtr));
    m_layer_timing.next_write_data_ready_at =
        std::max(m_layer_timing.next_write_data_ready_at,
                 safe_add(clk, m_timing_nbl));
  }

  if (command == m_cmd_prea) {
    update_rank_timing([&](RankTimingEntry& rank_entry) {
      rank_entry.next_prea_ready_at =
          std::max(rank_entry.next_prea_ready_at, safe_add(clk, m_timing_nrp));
    });
  }

  for_each_target_bank(addr_vec, [&](BankStateEntry& entry, int, int, int) {
    entry.last_issue_cycle = clk;

    if (entry.refreshing) {
      entry.refreshing = false;
      entry.refresh_recovery = true;
      entry.last_refresh_exit_cycle = clk;
    } else if (entry.refresh_recovery) {
      entry.refresh_recovery = false;
    }

    if (meta.is_opening) {
      entry.open_row_valid = row >= 0;
      entry.open_row = row;
      entry.col_accesses_on_row = 0;
      entry.open_since_cycle = clk;
      entry.autoprecharge_armed = false;
      entry.next_read_ready_at =
          std::max(entry.next_read_ready_at, safe_add(clk, m_timing_nrcd));
      entry.next_write_ready_at =
          std::max(entry.next_write_ready_at, safe_add(clk, m_timing_nrcd));
      entry.next_pre_ready_at =
          std::max(entry.next_pre_ready_at, safe_add(clk, m_timing_nras));
      entry.next_act_ready_at =
          std::max(entry.next_act_ready_at, safe_add(clk, m_timing_nrc));
    }

    if (meta.is_accessing) {
      entry.inflight_accesses++;
      if (entry.open_row_valid) {
        entry.col_accesses_on_row++;
      }
      if (meta.is_closing) {
        entry.autoprecharge_armed = true;
      }

      if (is_read_like_command(command)) {
        entry.next_pre_ready_at =
            std::max(entry.next_pre_ready_at, safe_add(clk, m_timing_nrtp));
        entry.next_write_ready_at =
            std::max(entry.next_write_ready_at, safe_add(clk, m_timing_nrtw));
        if (command == m_cmd_rda) {
          entry.next_act_ready_at = std::max(
              entry.next_act_ready_at,
              safe_add(clk, m_timing_nrtp + m_timing_nrp));
        }
      } else if (is_write_like_command(command)) {
        const Clk_t wr_to_pre =
            safe_add(clk, m_timing_ncwl + m_timing_nbl + m_timing_nwr);
        entry.next_pre_ready_at = std::max(entry.next_pre_ready_at, wr_to_pre);
        entry.next_read_ready_at =
            std::max(entry.next_read_ready_at, safe_add(clk, m_timing_nwtr));
        if (command == m_cmd_wra) {
          entry.next_act_ready_at = std::max(
              entry.next_act_ready_at, safe_add(wr_to_pre, m_timing_nrp));
        }
      }
    }

    if (meta.is_closing) {
      entry.open_row_valid = false;
      entry.open_row = -1;
      entry.col_accesses_on_row = 0;
      entry.autoprecharge_armed = false;
      entry.next_act_ready_at =
          std::max(entry.next_act_ready_at, safe_add(clk, m_timing_nrp));
    }
  });
}

void BankStateScoreboard::on_request_completed(const Request& req, Clk_t clk) {
  (void)clk;
  if (!m_valid) return;
  int rank = 0;
  int bg = 0;
  int bank = 0;
  if (!lookup_bank_components(req.addr_vec, rank, bg, bank)) return;
  BankStateEntry& entry = m_bank_entries[flatten_index(rank, bg, bank)];
  if (entry.inflight_accesses > 0) {
    entry.inflight_accesses--;
  }
}

SchedulingState BankStateScoreboard::resolve_scheduling_state(
    IDRAM* dram, int final_command, const AddrVec_t& addr_vec,
    Clk_t clk) const {
  SchedulingState result {};
  if (!m_valid || !dram || final_command < 0) {
    result.ready_block_reason = ReadyBlockReason::kScoreboardMiss;
    return result;
  }

  result.refresh_state = snapshot_refresh_state(addr_vec, clk);
  result.bank_state = snapshot_bank_state(addr_vec, clk);
  result.next_command = get_prereq_command(dram, final_command, addr_vec);
  result.prereq_scoreboard_miss = (result.next_command < 0);
  if (result.next_command >= 0) {
    const ReadyResult ready =
        evaluate_command_ready(dram, result.next_command, addr_vec, clk);
    result.next_command_ready = ready.ready;
    result.ready_block_reason = ready.block_reason;
  } else {
    result.ready_block_reason = ReadyBlockReason::kScoreboardMiss;
  }

  const auto meta = dram->m_command_meta(final_command);
  if (meta.is_refreshing) {
    result.row_state.valid = false;
    result.row_state.refreshing =
        result.refresh_state.pending || result.refresh_state.active ||
        result.refresh_state.recovery || result.bank_state.refreshing ||
        result.bank_state.refresh_recovery;
  } else {
    result.row_state = probe_result_from_bank_state(result.bank_state);
    result.rowstate_scoreboard_miss = !result.bank_state.valid;
  }

  result.valid = (result.next_command >= 0) || result.bank_state.valid ||
                 result.row_state.valid || result.row_state.refreshing ||
                 result.refresh_state.valid;
  return result;
}

ProbeResult BankStateScoreboard::probe(IDRAM* dram, int final_command,
                                       const AddrVec_t& addr_vec) const {
  (void)dram;
  (void)final_command;
  return probe_result_from_bank_state(snapshot_bank_state(addr_vec, 0));
}

int BankStateScoreboard::get_prereq_command(IDRAM* dram, int final_command,
                                            const AddrVec_t& addr_vec) const {
  if (!m_valid) return -1;
  if (!dram) return -1;
  if (final_command < 0) return -1;

  // All-bank refresh (rank-scope) requires all banks closed. If any bank in the
  // scope is open, issue PREA as the prerequisite.
  if (final_command == m_cmd_refab) {
    if (m_cmd_prea >= 0 && any_open_row_in_scope(addr_vec)) {
      return m_cmd_prea;
    }
    return final_command;
  }

  if (final_command == m_cmd_prea) {
    return final_command;
  }

  const bool is_access =
      is_read_like_command(final_command) || is_write_like_command(final_command);
  if (is_access) {
    int rank = 0;
    int bg = 0;
    int bank = 0;
    if (!lookup_bank_components(addr_vec, rank, bg, bank)) {
      return -1;
    }
    const BankStateEntry& entry = m_bank_entries[flatten_index(rank, bg, bank)];
    const int row = addr_component(addr_vec, m_level_row);
    if (entry.open_row_valid && row >= 0 && entry.open_row == row) {
      return final_command;  // row hit
    }
    if (entry.open_row_valid) {
      return m_cmd_pre >= 0 ? m_cmd_pre : -1;  // row conflict
    }
    return m_cmd_act >= 0 ? m_cmd_act : -1;  // row miss
  }

  if (final_command == m_cmd_act) {
    int rank = 0;
    int bg = 0;
    int bank = 0;
    if (!lookup_bank_components(addr_vec, rank, bg, bank)) {
      return -1;
    }
    const BankStateEntry& entry = m_bank_entries[flatten_index(rank, bg, bank)];
    if (entry.open_row_valid) {
      return m_cmd_pre >= 0 ? m_cmd_pre : -1;
    }
    return final_command;
  }

  if (final_command == m_cmd_pre) {
    return final_command;
  }

  // Unknown/untracked command: assume no prereq needed.
  return final_command;
}

TelemetryResult BankStateScoreboard::query_telemetry(
    IDRAM* dram, int final_command, const AddrVec_t& addr_vec,
    Clk_t clk) const {
  TelemetryResult result {};
  const BankStateSnapshot bank_state = snapshot_bank_state(addr_vec, clk);
  if (!bank_state.valid) {
    return result;
  }
  const ProbeResult probe_result = probe_result_from_bank_state(bank_state);
  const ReadyResult ready =
      evaluate_command_ready(dram, final_command, addr_vec, clk);
  const RefreshStateSnapshot refresh_state =
      snapshot_refresh_state(addr_vec, clk);

  result.valid = true;
  result.bank_state = bank_state;
  result.row_hit = probe_result.row_hit;
  result.row_open = probe_result.row_open;
  result.refreshing = probe_result.refreshing;
  result.bank_busy = (bank_state.inflight_accesses > 0) || !ready.ready;
  result.inflight_accesses = bank_state.inflight_accesses;
  result.col_accesses_on_row = bank_state.col_accesses_on_row;
  result.autoprecharge_armed = bank_state.autoprecharge_armed;
  result.open_row = bank_state.open_row_valid ? bank_state.open_row : -1;
  result.refresh_epoch = refresh_state.epoch;
  result.refresh_horizon_cycles = refresh_state.horizon_cycles;
  result.ready_block_reason = ready.block_reason;
  result.refresh_state = refresh_state;
  return result;
}

RefreshStateSnapshot BankStateScoreboard::snapshot_global_refresh_state(
    Clk_t clk) const {
  RefreshStateSnapshot result {};
  if (!m_valid) {
    return result;
  }

  const bool scope_known = m_refresh_scope_state.kind != RefreshScopeKind::kNone ||
                           m_refresh_scope_state.pending ||
                           m_refresh_scope_state.active ||
                           m_refresh_scope_state.epoch != 0;
  if (scope_known) {
    result.valid = true;
    result.pending = m_refresh_scope_state.pending;
    result.active = m_refresh_scope_state.active;
    result.owner_scope = m_refresh_scope_state.kind;
    result.epoch = m_refresh_scope_state.epoch;
    if (m_refresh_scope_state.active && m_refresh_scope_state.enter_cycle >= 0 &&
        m_timing_nrfc > 0) {
      const Clk_t scope_blocked_until =
          safe_add(m_refresh_scope_state.enter_cycle, m_timing_nrfc);
      if (scope_blocked_until > clk) {
        result.horizon_cycles =
            static_cast<uint64_t>(scope_blocked_until - clk);
      }
    }
  }

  for (const auto& entry : m_bank_entries) {
    const bool entry_known = entry.refresh_pending || entry.refreshing ||
                             entry.refresh_recovery || entry.refresh_epoch != 0;
    if (!entry_known) {
      continue;
    }

    result.valid = true;
    result.pending = result.pending || entry.refresh_pending;
    result.active = result.active || entry.refreshing;
    result.recovery = result.recovery || entry.refresh_recovery;
    if (result.owner_scope == RefreshScopeKind::kNone &&
        entry.refresh_owner_scope != RefreshScopeKind::kNone) {
      result.owner_scope = entry.refresh_owner_scope;
    }
    result.epoch = std::max<uint64_t>(result.epoch, entry.refresh_epoch);
    const Clk_t blocked_until =
        std::max(entry.refresh_blocked_until, entry.recovery_blocked_until);
    if (blocked_until > clk) {
      result.horizon_cycles = std::max<uint64_t>(
          result.horizon_cycles,
          static_cast<uint64_t>(blocked_until - clk));
    }
  }

  return result;
}

bool BankStateScoreboard::is_command_ready(IDRAM* dram, int command,
                                           const AddrVec_t& addr_vec,
                                           Clk_t clk) const {
  return evaluate_command_ready(dram, command, addr_vec, clk).ready;
}

ForcedAutoprechargeDecision
BankStateScoreboard::evaluate_forced_autoprecharge(
    IDRAM* dram, const Request& req, const BankStateSnapshot& bank_state,
    Clk_t clk, uint32_t autoprecharge_cap) const {
  ForcedAutoprechargeDecision result {};
  if (!m_valid || !dram) {
    return result;
  }

  result.valid = true;
  if (autoprecharge_cap == 0) {
    return result;
  }
  if (req.command != req.final_command) {
    return result;
  }
  if (!bank_state.valid || !bank_state.row_hit) {
    return result;
  }
  if (bank_state.refreshing || bank_state.refresh_recovery) {
    return result;
  }

  const auto cmd_meta = dram->m_command_meta(req.command);
  if (!cmd_meta.is_accessing || cmd_meta.is_closing) {
    return result;
  }

  if (bank_state.col_accesses_on_row + 1 <
      static_cast<uint64_t>(autoprecharge_cap)) {
    return result;
  }

  if (req.type_id == Request::Type::Read && req.command == m_cmd_rd) {
    result.issue_command = m_cmd_rda;
  } else if (req.type_id == Request::Type::Write &&
             req.command == m_cmd_wr) {
    result.issue_command = m_cmd_wra;
  } else {
    return result;
  }

  if (result.issue_command < 0) {
    return result;
  }

  const ReadyResult ready =
      evaluate_command_ready(dram, result.issue_command, req.addr_vec, clk);
  result.block_reason = ready.block_reason;
  result.force = ready.ready;
  if (!result.force) {
    result.issue_command = -1;
  }
  return result;
}

ShadowDiffResult BankStateScoreboard::diff_against_dram(
    IDRAM* dram, int final_command, const AddrVec_t& addr_vec) const {
  ShadowDiffResult result {};
  const ProbeResult probe_result =
      probe_result_from_bank_state(snapshot_bank_state(addr_vec, 0));
  if (!probe_result.valid) {
    return result;
  }

  result.valid = true;
  result.refreshing = probe_result.refreshing;
  result.scoreboard_row_hit = probe_result.row_hit;
  result.scoreboard_row_open = probe_result.row_open;
  result.dram_row_hit = dram->check_rowbuffer_hit(final_command, addr_vec);
  result.dram_row_open = dram->check_node_open(final_command, addr_vec);
  result.row_hit_match = (result.scoreboard_row_hit == result.dram_row_hit);
  result.row_open_match = (result.scoreboard_row_open == result.dram_row_open);
  return result;
}

size_t BankStateScoreboard::count_refreshing_banks() const {
  if (!m_valid) return 0;
  size_t count = 0;
  for (const auto& entry : m_bank_entries) {
    if (entry.refreshing) count++;
  }
  return count;
}

size_t BankStateScoreboard::count_refresh_pending_banks() const {
  if (!m_valid) return 0;
  size_t count = 0;
  for (const auto& entry : m_bank_entries) {
    if (entry.refresh_pending) count++;
  }
  return count;
}

size_t BankStateScoreboard::count_open_banks() const {
  if (!m_valid) return 0;
  size_t count = 0;
  for (const auto& entry : m_bank_entries) {
    if (entry.open_row_valid && !entry.refreshing) count++;
  }
  return count;
}

size_t BankStateScoreboard::count_inflight_banks() const {
  if (!m_valid) return 0;
  size_t count = 0;
  for (const auto& entry : m_bank_entries) {
    if (entry.inflight_accesses > 0) count++;
  }
  return count;
}

size_t BankStateScoreboard::count_autoprecharge_armed_banks() const {
  if (!m_valid) return 0;
  size_t count = 0;
  for (const auto& entry : m_bank_entries) {
    if (entry.autoprecharge_armed) count++;
  }
  return count;
}

uint64_t BankStateScoreboard::max_open_row_age_cycles(Clk_t clk) const {
  if (!m_valid) return 0;
  uint64_t max_age = 0;
  for (const auto& entry : m_bank_entries) {
    if (!entry.open_row_valid || entry.open_since_cycle < 0 ||
        clk < entry.open_since_cycle) {
      continue;
    }
    max_age =
        std::max<uint64_t>(max_age, static_cast<uint64_t>(clk - entry.open_since_cycle));
  }
  return max_age;
}

}  // namespace Ramulator
