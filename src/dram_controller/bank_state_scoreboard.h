#ifndef RAMULATOR_CONTROLLER_BANK_STATE_SCOREBOARD_H
#define RAMULATOR_CONTROLLER_BANK_STATE_SCOREBOARD_H

#include <cstdint>
#include <deque>
#include <vector>

#include "base/base.h"
#include "dram/dram.h"

namespace Ramulator {

struct ProbeResult {
  bool valid = false;
  bool row_hit = false;
  bool row_open = false;
  bool refreshing = false;
  uint64_t col_accesses_on_row = 0;
  bool autoprecharge_armed = false;
};

struct TelemetryResult {
  bool valid = false;
  bool row_hit = false;
  bool row_open = false;
  bool refreshing = false;
  bool bank_busy = false;
  uint64_t inflight_accesses = 0;
  uint64_t col_accesses_on_row = 0;
  bool autoprecharge_armed = false;
  int open_row = -1;
  uint64_t refresh_epoch = 0;
  uint64_t refresh_horizon_cycles = 0;
};

enum class RefreshScopeKind {
  kNone = 0,
  kBank,
  kBankGroup,
  kRank,
  kChannel,
};

struct RefreshScopeState {
  RefreshScopeKind kind = RefreshScopeKind::kNone;
  bool pending = false;
  bool active = false;
  uint64_t epoch = 0;
  AddrVec_t scope_addr_vec {};
  Clk_t enter_cycle = -1;
  Clk_t exit_cycle = -1;
};

struct RankTimingEntry {
  Clk_t next_prea_ready_at = 0;
  Clk_t next_ref_ready_at = 0;
  Clk_t refresh_active_until = 0;
  Clk_t refresh_recovery_until = 0;
};

struct LayerTimingEntry {
  Clk_t next_act_ready_at = 0;
  Clk_t next_col_issue_ready_at = 0;
  Clk_t next_read_ready_at = 0;
  Clk_t next_write_ready_at = 0;
  Clk_t next_read_data_ready_at = 0;
  Clk_t next_write_data_ready_at = 0;
  std::deque<Clk_t> recent_act_cycles {};
};

struct BankStateEntry {
  bool open_row_valid = false;
  int open_row = -1;

  uint64_t inflight_accesses = 0;
  uint64_t col_accesses_on_row = 0;
  Clk_t open_since_cycle = -1;
  Clk_t last_issue_cycle = -1;
  bool autoprecharge_armed = false;

  Clk_t next_act_ready_at = 0;
  Clk_t next_pre_ready_at = 0;
  Clk_t next_read_ready_at = 0;
  Clk_t next_write_ready_at = 0;
  Clk_t refresh_blocked_until = 0;
  Clk_t recovery_blocked_until = 0;

  bool refresh_pending = false;
  bool refreshing = false;
  bool refresh_recovery = false;
  uint64_t refresh_epoch = 0;
  Clk_t last_refresh_enter_cycle = -1;
  Clk_t last_refresh_exit_cycle = -1;
  RefreshScopeKind refresh_owner_scope = RefreshScopeKind::kNone;
};

struct ShadowDiffResult {
  bool valid = false;
  bool row_hit_match = true;
  bool row_open_match = true;
  bool scoreboard_row_hit = false;
  bool dram_row_hit = false;
  bool scoreboard_row_open = false;
  bool dram_row_open = false;
  bool refreshing = false;
};

class BankStateScoreboard {
 public:
  void init_from_dram_org(IDRAM* dram, int channel_id);

  bool valid() const { return m_valid; }

  void on_issue_command(IDRAM* dram, int command, const AddrVec_t& addr_vec,
                        Clk_t clk);
  void on_request_completed(const Request& req, Clk_t clk);
  void on_refresh_scope_pending_from_command(IDRAM* dram, int command,
                                             const AddrVec_t& scope_addr_vec,
                                             Clk_t clk);
  void on_refresh_scope_pending(RefreshScopeKind scope_kind,
                                const AddrVec_t& scope_addr_vec, Clk_t clk);
  void on_refresh_enter(RefreshScopeKind scope_kind,
                        const AddrVec_t& scope_addr_vec, Clk_t clk);
  void on_refresh_exit(RefreshScopeKind scope_kind,
                       const AddrVec_t& scope_addr_vec, Clk_t clk);

  ProbeResult probe(IDRAM* dram, int final_command,
                    const AddrVec_t& addr_vec) const;
  // Controller-owned prerequisite command resolution based on scoreboard state.
  //
  // Returns:
  //   - a DRAM command id to issue next (ACT/PRE/RD/WR/PREA/REF...)
  //   - or -1 if the scoreboard cannot answer (caller may fallback to DRAM).
  int get_prereq_command(IDRAM* dram, int final_command,
                         const AddrVec_t& addr_vec) const;
  TelemetryResult query_telemetry(IDRAM* dram, int final_command,
                                  const AddrVec_t& addr_vec, Clk_t clk) const;
  bool is_command_ready(IDRAM* dram, int command, const AddrVec_t& addr_vec,
                        Clk_t clk) const;
  ShadowDiffResult diff_against_dram(IDRAM* dram, int final_command,
                                     const AddrVec_t& addr_vec) const;

  size_t count_refreshing_banks() const;
  size_t count_refresh_pending_banks() const;
  const RefreshScopeState& refresh_scope_state() const {
    return m_refresh_scope_state;
  }

 private:
  bool m_valid = false;
  int m_channel_id = -1;

  int m_level_channel = -1;
  int m_level_rank = -1;
  int m_level_tier = -1;
  int m_level_bankgroup = -1;
  int m_level_bank = -1;
  int m_level_row = -1;

  int m_num_ranks = 1;
  int m_num_bankgroups = 1;
  int m_num_banks = 1;

  int m_cmd_act = -1;
  int m_cmd_pre = -1;
  int m_cmd_prea = -1;
  int m_cmd_rd = -1;
  int m_cmd_wr = -1;
  int m_cmd_rda = -1;
  int m_cmd_wra = -1;
  int m_cmd_refab = -1;

  Clk_t m_timing_nrcd = 0;
  Clk_t m_timing_nras = 0;
  Clk_t m_timing_nrp = 0;
  Clk_t m_timing_nrc = 0;
  Clk_t m_timing_nrtp = 0;
  Clk_t m_timing_nwtr = 0;
  Clk_t m_timing_nrtw = 0;
  Clk_t m_timing_nrrds = 0;
  Clk_t m_timing_nfaw = 0;
  Clk_t m_timing_nccds = 0;
  Clk_t m_timing_ncwl = 0;
  Clk_t m_timing_nbl = 0;
  Clk_t m_timing_nwr = 0;
  Clk_t m_timing_nrfc = 0;
  Clk_t m_timing_nrefi = 0;

  RefreshScopeState m_refresh_scope_state {};
  LayerTimingEntry m_layer_timing {};
  std::vector<RankTimingEntry> m_rank_entries;
  std::vector<BankStateEntry> m_bank_entries;

  static int try_level_index(IDRAM* dram, const char* level_name);
  static int try_command_index(IDRAM* dram, const char* command_name);
  static int addr_component(const AddrVec_t& addr_vec, int level_idx);
  static Clk_t clamp_non_negative(Clk_t value);
  static Clk_t safe_add(Clk_t base, Clk_t delta);
  static Clk_t try_timing_value(IDRAM* dram, const char* timing_name);

  bool resolve_iteration_bounds(const AddrVec_t& addr_vec, int level_idx,
                                int level_size, int& begin,
                                int& end) const;
  bool lookup_bank_components(const AddrVec_t& addr_vec, int& rank, int& bg,
                              int& bank) const;
  size_t flatten_index(int rank, int bg, int bank) const;
  bool any_open_row_in_scope(const AddrVec_t& addr_vec) const;

  template <typename Fn>
  void for_each_target_bank(const AddrVec_t& addr_vec, Fn&& fn) {
    if (!m_valid) return;

    if (m_level_channel >= 0) {
      const int channel = addr_component(addr_vec, m_level_channel);
      if (channel >= 0 && channel != m_channel_id) return;
    }

    int rank_begin = 0;
    int rank_end = 1;
    int bg_begin = 0;
    int bg_end = 1;
    int bank_begin = 0;
    int bank_end = 1;
    if (!resolve_iteration_bounds(addr_vec, m_level_rank, m_num_ranks,
                                  rank_begin, rank_end))
      return;
    if (!resolve_iteration_bounds(addr_vec, m_level_bankgroup, m_num_bankgroups,
                                  bg_begin, bg_end))
      return;
    if (!resolve_iteration_bounds(addr_vec, m_level_bank, m_num_banks,
                                  bank_begin, bank_end))
      return;

    for (int rank = rank_begin; rank < rank_end; ++rank) {
      for (int bg = bg_begin; bg < bg_end; ++bg) {
        for (int bank = bank_begin; bank < bank_end; ++bank) {
          fn(m_bank_entries[flatten_index(rank, bg, bank)], rank, bg, bank);
        }
      }
    }
  }

  template <typename Fn>
  void for_each_target_rank(const AddrVec_t& addr_vec, Fn&& fn) {
    if (!m_valid) return;

    if (m_level_channel >= 0) {
      const int channel = addr_component(addr_vec, m_level_channel);
      if (channel >= 0 && channel != m_channel_id) return;
    }

    int rank_begin = 0;
    int rank_end = 1;
    if (!resolve_iteration_bounds(addr_vec, m_level_rank, m_num_ranks,
                                  rank_begin, rank_end))
      return;

    for (int rank = rank_begin; rank < rank_end; ++rank) {
      fn(m_rank_entries[static_cast<size_t>(rank)], rank);
    }
  }

  template <typename Fn>
  void for_each_target_rank_const(const AddrVec_t& addr_vec, Fn&& fn) const {
    if (!m_valid) return;

    if (m_level_channel >= 0) {
      const int channel = addr_component(addr_vec, m_level_channel);
      if (channel >= 0 && channel != m_channel_id) return;
    }

    int rank_begin = 0;
    int rank_end = 1;
    if (!resolve_iteration_bounds(addr_vec, m_level_rank, m_num_ranks,
                                  rank_begin, rank_end))
      return;

    for (int rank = rank_begin; rank < rank_end; ++rank) {
      fn(m_rank_entries[static_cast<size_t>(rank)], rank);
    }
  }

  RefreshScopeKind command_scope_to_refresh_scope(IDRAM* dram,
                                                  int command) const;
  void cache_command_indices(IDRAM* dram);
  void cache_timing_values(IDRAM* dram);
  bool is_read_like_command(int command) const;
  bool is_write_like_command(int command) const;
};

}  // namespace Ramulator

#endif  // RAMULATOR_CONTROLLER_BANK_STATE_SCOREBOARD_H
