#include "dram_controller/controller.h"
#include "dram_controller/bank_state_scoreboard.h"
#include "memory_system/memory_system.h"

#include <algorithm>
#include <array>
#include <deque>
#include <set>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

namespace Ramulator {

namespace {

constexpr int kShmemTrafficClassScratchpadIdx = 4;
constexpr size_t kNumExternalTrafficClasses = 3;

enum class ExternalTrafficClass : int {
  kForeground = 0,
  kBackground = 1,
  kShadow = 2,
};

using TrafficClassCounterArray = std::array<size_t, kNumExternalTrafficClasses>;
using TrafficClassAverageArray = std::array<float, kNumExternalTrafficClasses>;

constexpr std::array<ExternalTrafficClass, kNumExternalTrafficClasses>
    kTrafficClasses = {ExternalTrafficClass::kForeground,
                       ExternalTrafficClass::kBackground,
                       ExternalTrafficClass::kShadow};

ExternalTrafficClass get_external_traffic_class(const Request& req) {
  const int raw = req.scratchpad[kShmemTrafficClassScratchpadIdx];
  if (raw == static_cast<int>(ExternalTrafficClass::kBackground)) {
    return ExternalTrafficClass::kBackground;
  }
  if (raw == static_cast<int>(ExternalTrafficClass::kShadow)) {
    return ExternalTrafficClass::kShadow;
  }
  return ExternalTrafficClass::kForeground;
}

constexpr size_t traffic_class_index(ExternalTrafficClass traffic_class) {
  return static_cast<size_t>(traffic_class);
}

const char* traffic_class_stat_prefix(ExternalTrafficClass traffic_class) {
  switch (traffic_class) {
    case ExternalTrafficClass::kForeground:
      return "foreground";
    case ExternalTrafficClass::kBackground:
      return "background";
    case ExternalTrafficClass::kShadow:
      return "shadow";
  }
  return "foreground";
}

bool is_valid_source_id(const Request& req, size_t num_cores) {
  return req.source_id >= 0 && static_cast<size_t>(req.source_id) < num_cores;
}

ControllerRefreshScope controller_refresh_scope_from_scoreboard(
    RefreshScopeKind scope_kind) {
  switch (scope_kind) {
    case RefreshScopeKind::kBank:
      return ControllerRefreshScope::kBank;
    case RefreshScopeKind::kBankGroup:
      return ControllerRefreshScope::kBankGroup;
    case RefreshScopeKind::kRank:
      return ControllerRefreshScope::kRank;
    case RefreshScopeKind::kChannel:
      return ControllerRefreshScope::kChannel;
    case RefreshScopeKind::kNone:
    default:
      return ControllerRefreshScope::kNone;
  }
}

ControllerReadyBlockReason controller_ready_block_reason_from_scoreboard(
    ReadyBlockReason reason) {
  switch (reason) {
    case ReadyBlockReason::kScoreboardMiss:
      return ControllerReadyBlockReason::kScoreboardMiss;
    case ReadyBlockReason::kInvalidCommand:
      return ControllerReadyBlockReason::kInvalidCommand;
    case ReadyBlockReason::kAddressDecodeMiss:
      return ControllerReadyBlockReason::kAddressDecodeMiss;
    case ReadyBlockReason::kRankRefreshActive:
      return ControllerReadyBlockReason::kRankRefreshActive;
    case ReadyBlockReason::kRankRefreshRecovery:
      return ControllerReadyBlockReason::kRankRefreshRecovery;
    case ReadyBlockReason::kRankPrechargeTiming:
      return ControllerReadyBlockReason::kRankPrechargeTiming;
    case ReadyBlockReason::kRankRefreshTiming:
      return ControllerReadyBlockReason::kRankRefreshTiming;
    case ReadyBlockReason::kRefreshScopeOpenRows:
      return ControllerReadyBlockReason::kRefreshScopeOpenRows;
    case ReadyBlockReason::kBankRefreshActive:
      return ControllerReadyBlockReason::kBankRefreshActive;
    case ReadyBlockReason::kBankRefreshRecovery:
      return ControllerReadyBlockReason::kBankRefreshRecovery;
    case ReadyBlockReason::kBankOpen:
      return ControllerReadyBlockReason::kBankOpen;
    case ReadyBlockReason::kBankClosed:
      return ControllerReadyBlockReason::kBankClosed;
    case ReadyBlockReason::kRowConflict:
      return ControllerReadyBlockReason::kRowConflict;
    case ReadyBlockReason::kActivateWindow:
      return ControllerReadyBlockReason::kActivateWindow;
    case ReadyBlockReason::kFourActivateWindow:
      return ControllerReadyBlockReason::kFourActivateWindow;
    case ReadyBlockReason::kBankTimingAct:
      return ControllerReadyBlockReason::kBankTimingAct;
    case ReadyBlockReason::kBankTimingPre:
      return ControllerReadyBlockReason::kBankTimingPre;
    case ReadyBlockReason::kColumnBusTiming:
      return ControllerReadyBlockReason::kColumnBusTiming;
    case ReadyBlockReason::kReadDataTiming:
      return ControllerReadyBlockReason::kReadDataTiming;
    case ReadyBlockReason::kWriteDataTiming:
      return ControllerReadyBlockReason::kWriteDataTiming;
    case ReadyBlockReason::kReadTurnaroundTiming:
      return ControllerReadyBlockReason::kReadTurnaroundTiming;
    case ReadyBlockReason::kWriteTurnaroundTiming:
      return ControllerReadyBlockReason::kWriteTurnaroundTiming;
    case ReadyBlockReason::kBankTimingRead:
      return ControllerReadyBlockReason::kBankTimingRead;
    case ReadyBlockReason::kBankTimingWrite:
      return ControllerReadyBlockReason::kBankTimingWrite;
    case ReadyBlockReason::kNone:
    default:
      return ControllerReadyBlockReason::kNone;
  }
}

ControllerReadyBlockReason choose_ready_block_reason(
    ControllerReadyBlockReason current,
    ControllerReadyBlockReason candidate) {
  if (current != ControllerReadyBlockReason::kNone) {
    return current;
  }
  return candidate;
}

}  // namespace

class TieredM3DController final : public IDRAMController, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(
      IDRAMController, TieredM3DController, "TieredM3D",
      "A tier-aware DRAM controller that models shared channel issue/access "
      "resources together with per-tier access-port budgets.")

 private:
  std::deque<Request>
      pending;  // A queue for read/write requests waiting for completion.

  ReqBuffer m_active_buffer;    // Buffer for requests being served
  ReqBuffer m_priority_buffer;  // Buffer for high-priority requests
  ReqBuffer m_read_buffer;      // Read request buffer
  ReqBuffer m_write_buffer;     // Write request buffer

  int m_bank_addr_idx = -1;
  int m_tier_addr_idx = -1;
  size_t m_num_tiers = 1;

  float m_wr_low_watermark;
  float m_wr_high_watermark;
  bool m_is_write_mode = false;

  // Legacy knob: previously used for both "issue width" and "access ports".
  uint32_t m_bank_parallel_ports = 1;
  // New knobs (TieredM3D):
  // - m_cmd_issue_width: total #commands issued per controller cycle.
  // - m_shared_access_ports: shared #accessing commands per controller cycle.
  // - m_tier_access_ports: per-tier #accessing commands per controller cycle.
  // - m_vertical_transfer_ports: remote-tier accesses per controller cycle.
  uint32_t m_cmd_issue_width = 1;
  uint32_t m_shared_access_ports = 1;
  uint32_t m_tier_access_ports = 1;
  uint32_t m_vertical_transfer_ports = 0;
  bool m_size_aware_timing = true;
  uint32_t m_local_burst_transfer_cycles = 1;
  uint32_t m_vertical_transfer_cycles_per_burst = 1;
  uint32_t m_vertical_copy_cycles_per_burst = 1;
  uint32_t m_cross_tier_hop_latency_cycles = 0;
  uint32_t m_vertical_copy_hop_latency_cycles = 0;
  uint32_t m_cross_tier_source_endpoint_cycles_per_burst = 0;
  uint32_t m_cross_tier_destination_endpoint_cycles_per_burst = 0;
  uint32_t m_vertical_copy_source_endpoint_cycles_per_burst = 0;
  uint32_t m_vertical_copy_destination_endpoint_cycles_per_burst = 0;
  uint32_t m_vertical_link_ports_per_hop = 0;
  uint32_t m_vertical_link_cycles_per_burst = 1;
  uint32_t m_transfer_unit_bytes = 1;
  enum class WriteCompletionMode { kPosted, kData };
  WriteCompletionMode m_write_completion_mode = WriteCompletionMode::kPosted;
  std::multiset<Clk_t> m_active_vertical_transfer_releases;
  std::vector<std::multiset<Clk_t>> m_active_vertical_link_releases;
  ControllerReadyBlockReason m_last_ready_block_reason =
      ControllerReadyBlockReason::kNone;

  bool m_qos_enable = true;
  bool m_allow_foreground_read_interrupt_write_mode = true;
  uint32_t m_background_cmd_budget_per_cycle = 1;
  uint32_t m_shadow_cmd_budget_per_cycle = 1;
  Clk_t m_non_foreground_starvation_threshold_cycles = 64;
  bool m_shadow_scoreboard_enable = true;
  bool m_shadow_scoreboard_debug_overlay_enable = false;
  bool m_shadow_scoreboard_fail_fast = false;
  bool m_shadow_scoreboard_log_mismatch = false;
  BankStateScoreboard m_shadow_scoreboard;
  Clk_t m_refresh_window_cycles = -1;

  size_t s_row_hits = 0;
  size_t s_row_misses = 0;
  size_t s_row_conflicts = 0;
  size_t s_read_row_hits = 0;
  size_t s_read_row_misses = 0;
  size_t s_read_row_conflicts = 0;
  size_t s_write_row_hits = 0;
  size_t s_write_row_misses = 0;
  size_t s_write_row_conflicts = 0;

  size_t m_num_cores = 0;
  std::vector<size_t> s_read_row_hits_per_core;
  std::vector<size_t> s_read_row_misses_per_core;
  std::vector<size_t> s_read_row_conflicts_per_core;

  size_t s_num_read_reqs = 0;
  size_t s_num_write_reqs = 0;
  size_t s_num_other_reqs = 0;
  size_t s_num_foreground_read_reqs = 0;
  size_t s_num_foreground_write_reqs = 0;
  size_t s_num_background_read_reqs = 0;
  size_t s_num_background_write_reqs = 0;
  size_t s_num_shadow_read_reqs = 0;
  size_t s_num_shadow_write_reqs = 0;
  size_t s_tier_local_read_reqs = 0;
  size_t s_tier_local_write_reqs = 0;
  size_t s_cross_tier_read_reqs = 0;
  size_t s_cross_tier_write_reqs = 0;
  size_t s_vertical_copy_read_reqs = 0;
  size_t s_vertical_copy_write_reqs = 0;
  size_t s_explicit_source_tier_hint_reqs = 0;
  size_t s_explicit_destination_tier_hint_reqs = 0;
  size_t s_fallback_source_tier_reqs = 0;
  size_t s_fallback_destination_tier_reqs = 0;
  size_t s_queue_len = 0;
  size_t s_read_queue_len = 0;
  size_t s_write_queue_len = 0;
  size_t s_priority_queue_len = 0;
  float s_queue_len_avg = 0;
  float s_read_queue_len_avg = 0;
  float s_write_queue_len_avg = 0;
  float s_priority_queue_len_avg = 0;

  TrafficClassCounterArray m_counted_queue_occupancy = {0, 0, 0};
  TrafficClassCounterArray s_queue_len_by_class = {0, 0, 0};
  TrafficClassAverageArray s_queue_len_avg_by_class = {0.0f, 0.0f, 0.0f};
  TrafficClassCounterArray s_num_completed_reqs_by_class = {0, 0, 0};
  TrafficClassCounterArray s_queue_wait_cycles_by_class = {0, 0, 0};
  TrafficClassAverageArray s_avg_queue_wait_cycles_by_class = {0.0f, 0.0f,
                                                               0.0f};
  TrafficClassCounterArray s_max_queue_wait_cycles_by_class = {0, 0, 0};
  TrafficClassCounterArray s_qos_budget_blocked_events_by_class = {0, 0, 0};
  TrafficClassCounterArray s_qos_age_promotions_by_class = {0, 0, 0};
  size_t s_foreground_qos_wins = 0;
  size_t s_write_mode_enter_count = 0;
  size_t s_write_mode_exit_count = 0;
  size_t s_write_mode_fg_interrupt_count = 0;

  size_t s_read_latency = 0;
  float s_avg_read_latency = 0;

  size_t s_max_cmds_issued_per_cycle = 0;
  size_t s_max_access_cmds_issued_per_cycle = 0;
  size_t s_max_shared_access_cmds_issued_per_cycle = 0;
  size_t s_max_vertical_transfer_cmds_issued_per_cycle = 0;
  size_t s_cmds_issued_total = 0;
  size_t s_access_cmds_issued_total = 0;
  size_t s_local_tier_access_cmds_issued_total = 0;
  size_t s_shared_access_cmds_issued_total = 0;
  size_t s_vertical_transfer_cmds_issued_total = 0;
  size_t s_cross_tier_cmds_issued_total = 0;
  size_t s_vertical_copy_cmds_issued_total = 0;
  size_t s_vertical_port_busy_cycles = 0;
  size_t s_cross_tier_transfer_cycles_total = 0;
  size_t s_vertical_copy_transfer_cycles_total = 0;
  size_t s_cross_tier_hop_latency_cycles_total = 0;
  size_t s_vertical_copy_hop_latency_cycles_total = 0;
  size_t s_cross_tier_source_endpoint_cycles_total = 0;
  size_t s_cross_tier_destination_endpoint_cycles_total = 0;
  size_t s_vertical_copy_source_endpoint_cycles_total = 0;
  size_t s_vertical_copy_destination_endpoint_cycles_total = 0;
  size_t s_cross_tier_hops_total = 0;
  size_t s_vertical_copy_hops_total = 0;
  float s_avg_cmds_issued_per_cycle = 0;
  float s_avg_access_cmds_issued_per_cycle = 0;
  float s_avg_shared_access_cmds_issued_per_cycle = 0;
  float s_avg_vertical_transfer_cmds_issued_per_cycle = 0;
  std::vector<size_t> s_tier_access_cmds_issued_total;
  std::vector<size_t> s_max_tier_access_cmds_issued_per_cycle;
  std::vector<float> s_avg_tier_access_cmds_issued_per_cycle;
  std::vector<std::vector<size_t>> s_cross_tier_reqs_by_src_dst;
  std::vector<std::vector<size_t>> s_vertical_copy_reqs_by_src_dst;
  std::vector<size_t> s_cross_tier_reqs_by_hops;
  std::vector<size_t> s_vertical_copy_reqs_by_hops;
  std::vector<size_t> s_vertical_link_busy_cycles;
  size_t s_shadow_scoreboard_diff_checks = 0;
  size_t s_shadow_scoreboard_rowhit_mismatches = 0;
  size_t s_shadow_scoreboard_rowopen_mismatches = 0;
  size_t s_shadow_scoreboard_prereq_checks = 0;
  size_t s_shadow_scoreboard_prereq_mismatches = 0;
  size_t s_controller_prereq_scoreboard_misses = 0;
  size_t s_controller_rowstate_scoreboard_misses = 0;
  size_t s_shadow_scoreboard_ready_checks = 0;
  size_t s_shadow_scoreboard_ready_mismatches = 0;
  size_t s_shadow_scoreboard_ready_oracle_blocked_while_scoreboard_ready = 0;
  size_t s_shadow_scoreboard_ready_oracle_ready_while_scoreboard_blocked = 0;
  size_t s_shadow_scoreboard_ready_mismatch_open = 0;
  size_t s_shadow_scoreboard_ready_mismatch_access = 0;
  size_t s_shadow_scoreboard_ready_mismatch_close = 0;
  size_t s_shadow_scoreboard_ready_mismatch_other = 0;
  size_t s_shadow_scoreboard_ready_blocked_by_scoreboard = 0;
  size_t s_shadow_scoreboard_ready_blocked_by_oracle = 0;

 public:
  void init() override {
    m_wr_low_watermark =
        param<float>("wr_low_watermark")
            .desc("Threshold for switching back to read mode.")
            .default_val(0.2f);
    m_wr_high_watermark =
        param<float>("wr_high_watermark")
            .desc("Threshold for switching to write mode.")
            .default_val(0.8f);

    m_bank_parallel_ports =
        param<uint32_t>("bank_parallel_ports_per_layer")
            .desc(
                "Max number of DRAM commands issued per controller cycle; "
                "RD/WR-like accessing commands are additionally constrained to "
                "distinct banks within the channel.")
            .default_val(1);

    const uint32_t legacy_cmd_issue_width =
        param<uint32_t>("cmd_issue_width_per_layer")
            .desc(
                "Legacy alias for cmd_issue_width_per_channel in tier-aware controllers.")
            .default_val(m_bank_parallel_ports);
    m_cmd_issue_width =
        param<uint32_t>("cmd_issue_width_per_channel")
            .desc(
                "Max number of DRAM commands issued per controller cycle for the whole channel.")
            .default_val(legacy_cmd_issue_width);

    const uint32_t legacy_shared_access_ports =
        param<uint32_t>("access_ports_per_layer")
            .desc(
                "Legacy alias for shared_access_ports_per_channel in tier-aware controllers.")
            .default_val(m_bank_parallel_ports);
    m_shared_access_ports =
        param<uint32_t>("shared_access_ports_per_channel")
            .desc(
                "Max number of accessing DRAM commands (RD/WR/RDA/WRA) issued per channel cycle "
                "across all tiers; accessing commands are additionally constrained to distinct banks.")
            .default_val(legacy_shared_access_ports);
    m_tier_access_ports =
        param<uint32_t>("tier_access_ports")
            .desc(
                "Max number of accessing DRAM commands issued per tier per controller cycle.")
            .default_val(1);
    m_vertical_transfer_ports =
        param<uint32_t>("vertical_transfer_ports")
            .desc(
                "Max number of remote-tier accessing commands that can use the shared "
                "vertical path per channel cycle (0=unlimited/disabled).")
            .default_val(0);
    m_size_aware_timing =
        param<bool>("size_aware_timing")
            .desc(
                "Enable request-size-aware completion timing and multi-cycle vertical "
                "path occupancy in TieredM3D.")
            .default_val(true);
    m_local_burst_transfer_cycles =
        param<uint32_t>("local_burst_transfer_cycles")
            .desc(
                "Additional completion cycles charged per extra transfer unit beyond "
                "the first for read/data-write requests.")
            .default_val(1);
    m_vertical_transfer_cycles_per_burst =
        param<uint32_t>("vertical_transfer_cycles_per_burst")
            .desc(
                "Cycles that one remote transfer unit occupies a vertical port and "
                "adds to read/data-write completion latency.")
            .default_val(1);
    m_vertical_copy_cycles_per_burst =
        param<uint32_t>("vertical_copy_cycles_per_burst")
            .desc(
                "Cycles that one vertical-copy transfer unit occupies a vertical "
                "port and adds to completion latency.")
            .default_val(m_vertical_transfer_cycles_per_burst);
    m_cross_tier_hop_latency_cycles =
        param<uint32_t>("cross_tier_hop_latency_cycles")
            .desc(
                "Additional fixed latency charged per vertical hop for cross-tier "
                "requests (0=disable hop-aware latency).")
            .default_val(0);
    m_vertical_copy_hop_latency_cycles =
        param<uint32_t>("vertical_copy_hop_latency_cycles")
            .desc(
                "Additional fixed latency charged per vertical hop for vertical-copy "
                "requests (0=disable hop-aware latency).")
            .default_val(m_cross_tier_hop_latency_cycles);
    m_cross_tier_source_endpoint_cycles_per_burst =
        param<uint32_t>("cross_tier_source_endpoint_cycles_per_burst")
            .desc(
                "Additional completion cycles charged per transfer unit for the "
                "source-side endpoint stage of cross-tier requests.")
            .default_val(0);
    m_cross_tier_destination_endpoint_cycles_per_burst =
        param<uint32_t>("cross_tier_destination_endpoint_cycles_per_burst")
            .desc(
                "Additional completion cycles charged per transfer unit for the "
                "destination-side endpoint stage of cross-tier requests.")
            .default_val(0);
    m_vertical_copy_source_endpoint_cycles_per_burst =
        param<uint32_t>("vertical_copy_source_endpoint_cycles_per_burst")
            .desc(
                "Additional completion cycles charged per transfer unit for the "
                "source-side endpoint stage of vertical-copy requests.")
            .default_val(m_cross_tier_source_endpoint_cycles_per_burst);
    m_vertical_copy_destination_endpoint_cycles_per_burst =
        param<uint32_t>("vertical_copy_destination_endpoint_cycles_per_burst")
            .desc(
                "Additional completion cycles charged per transfer unit for the "
                "destination-side endpoint stage of vertical-copy requests.")
            .default_val(m_cross_tier_destination_endpoint_cycles_per_burst);
    m_vertical_link_ports_per_hop =
        param<uint32_t>("vertical_link_ports_per_hop")
            .desc(
                "Per-hop vertical link budget for adjacency-link topology "
                "(0=use legacy global vertical_transfer_ports pool).")
            .default_val(0);
    m_vertical_link_cycles_per_burst =
        param<uint32_t>("vertical_link_cycles_per_burst")
            .desc(
                "Cycles that one transfer unit occupies each traversed vertical "
                "adjacency link when per-hop topology is enabled.")
            .default_val(m_vertical_transfer_cycles_per_burst);

    const std::string write_mode =
        param<std::string>("write_completion_mode")
            .desc(
                "When a write request is considered complete: posted (next cycle) or data (nCWL+nBL).")
            .default_val("posted");
    if (write_mode == "posted") {
      m_write_completion_mode = WriteCompletionMode::kPosted;
    } else if (write_mode == "data") {
      m_write_completion_mode = WriteCompletionMode::kData;
    } else {
      throw ConfigurationError(
          "Invalid write_completion_mode \"{}\" (expected \"posted\" or \"data\").",
          write_mode);
    }

    m_qos_enable =
        param<bool>("qos_enable")
            .desc("Enable shared-aware foreground/background/shadow QoS in TieredM3D.")
            .default_val(true);
    m_allow_foreground_read_interrupt_write_mode =
        param<bool>("qos_allow_foreground_read_interrupt_write_mode")
            .desc("Allow a foreground read to bypass background/shadow write-drain when QoS prefers it.")
            .default_val(true);
    m_background_cmd_budget_per_cycle =
        param<uint32_t>("qos_background_cmd_budget_per_cycle")
            .desc("Max background commands issued per cycle while foreground demand exists (0=unlimited).")
            .default_val(1);
    m_shadow_cmd_budget_per_cycle =
        param<uint32_t>("qos_shadow_cmd_budget_per_cycle")
            .desc("Max shadow commands issued per cycle while foreground demand exists (0=unlimited).")
            .default_val(1);
    m_non_foreground_starvation_threshold_cycles =
        (Clk_t)param<uint32_t>("qos_non_foreground_starvation_threshold_cycles")
            .desc("Promote background/shadow traffic after waiting this many controller cycles (0=disable).")
            .default_val(64);
    m_shadow_scoreboard_enable =
        param<bool>("shadow_scoreboard_enable")
            .desc("Enable controller-owned bank-state scoreboard.")
            .default_val(true);
    m_shadow_scoreboard_debug_overlay_enable =
        param<bool>("shadow_scoreboard_debug_overlay_enable")
            .desc(
                "Enable optional DRAM-oracle debug overlay for scoreboard prereq/ready/row-state cross-checks.")
            .default_val(false);
    m_shadow_scoreboard_fail_fast =
        param<bool>("shadow_scoreboard_fail_fast")
            .desc(
                "Abort when optional DRAM-oracle debug overlay disagrees with controller scoreboard (row-state/prereq/ready).")
            .default_val(false);
    m_shadow_scoreboard_log_mismatch =
        param<bool>("shadow_scoreboard_log_mismatch")
            .desc(
                "Log mismatches between controller scoreboard and optional DRAM-oracle debug overlay.")
            .default_val(false);

    // Queue depths: keep defaults small for legacy behavior, but allow YAML to
    // scale buffers so the controller (not the injector) becomes the bottleneck.
    m_active_buffer.max_size =
        (size_t)param<uint32_t>("active_buffer_max_size")
            .desc("Max entries in active buffer (requests being served).")
            .default_val((uint32_t)m_active_buffer.max_size);
    m_read_buffer.max_size =
        (size_t)param<uint32_t>("read_buffer_max_size")
            .desc("Max entries in read request buffer.")
            .default_val((uint32_t)m_read_buffer.max_size);
    m_write_buffer.max_size =
        (size_t)param<uint32_t>("write_buffer_max_size")
            .desc("Max entries in write request buffer.")
            .default_val((uint32_t)m_write_buffer.max_size);
    m_priority_buffer.max_size =
        (size_t)param<uint32_t>("priority_buffer_max_size")
            .desc("Max entries in priority buffer.")
            .default_val(512u * 3u + 32u);

    m_scheduler = create_child_ifce<IScheduler>();
    m_refresh = create_child_ifce<IRefreshManager>();
    m_rowpolicy = create_child_ifce<IRowPolicy>();

    if (m_config["plugins"]) {
      YAML::Node plugin_configs = m_config["plugins"];
      for (YAML::iterator it = plugin_configs.begin(); it != plugin_configs.end();
           ++it) {
        m_plugins.push_back(create_child_ifce<IControllerPlugin>(*it));
      }
    }
  };

  void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
    m_dram = memory_system->get_ifce<IDRAM>();
    m_bank_addr_idx = m_dram->m_levels("bank");
    try {
      m_tier_addr_idx = m_dram->m_levels("tier");
    } catch (const std::out_of_range&) {
      m_tier_addr_idx = -1;
    }
    const int configured_tiers = m_dram->get_level_size("tier");
    m_num_tiers = configured_tiers > 0 ? static_cast<size_t>(configured_tiers) : 1;
    const uint64_t tx_bytes_u64 =
        (uint64_t)std::max(1, m_dram->m_internal_prefetch_size) *
        (uint64_t)std::max(8, m_dram->m_channel_width) / 8ull;
    m_transfer_unit_bytes =
        static_cast<uint32_t>(std::max<uint64_t>(1ull, tx_bytes_u64));

    if (m_shadow_scoreboard_enable) {
      m_shadow_scoreboard.init_from_dram_org(m_dram, m_channel_id);
    }
    m_refresh_window_cycles = detect_refresh_window_cycles();

    m_num_cores = frontend->get_num_cores();
    s_tier_access_cmds_issued_total.assign(m_num_tiers, 0);
    s_max_tier_access_cmds_issued_per_cycle.assign(m_num_tiers, 0);
    s_avg_tier_access_cmds_issued_per_cycle.assign(m_num_tiers, 0.0f);
    s_cross_tier_reqs_by_src_dst.assign(
        m_num_tiers, std::vector<size_t>(m_num_tiers, 0));
    s_vertical_copy_reqs_by_src_dst.assign(
        m_num_tiers, std::vector<size_t>(m_num_tiers, 0));
    s_cross_tier_reqs_by_hops.assign(std::max<size_t>(1, m_num_tiers), 0);
    s_vertical_copy_reqs_by_hops.assign(std::max<size_t>(1, m_num_tiers), 0);
    s_vertical_link_busy_cycles.assign(
        m_num_tiers > 1 ? (m_num_tiers - 1) : 0, 0);
    m_active_vertical_link_releases.assign(
        m_num_tiers > 1 ? (m_num_tiers - 1) : 0,
        std::multiset<Clk_t> {});

    s_read_row_hits_per_core.resize(m_num_cores, 0);
    s_read_row_misses_per_core.resize(m_num_cores, 0);
    s_read_row_conflicts_per_core.resize(m_num_cores, 0);

    register_stat(s_row_hits).name("row_hits_{}", m_channel_id);
    register_stat(s_row_misses).name("row_misses_{}", m_channel_id);
    register_stat(s_row_conflicts).name("row_conflicts_{}", m_channel_id);
    register_stat(s_read_row_hits).name("read_row_hits_{}", m_channel_id);
    register_stat(s_read_row_misses).name("read_row_misses_{}", m_channel_id);
    register_stat(s_read_row_conflicts)
        .name("read_row_conflicts_{}", m_channel_id);
    register_stat(s_write_row_hits).name("write_row_hits_{}", m_channel_id);
    register_stat(s_write_row_misses).name("write_row_misses_{}", m_channel_id);
    register_stat(s_write_row_conflicts)
        .name("write_row_conflicts_{}", m_channel_id);

    for (size_t core_id = 0; core_id < m_num_cores; core_id++) {
      register_stat(s_read_row_hits_per_core[core_id])
          .name("read_row_hits_core_{}", core_id);
      register_stat(s_read_row_misses_per_core[core_id])
          .name("read_row_misses_core_{}", core_id);
      register_stat(s_read_row_conflicts_per_core[core_id])
          .name("read_row_conflicts_core_{}", core_id);
    }

    register_stat(s_num_read_reqs).name("num_read_reqs_{}", m_channel_id);
    register_stat(s_num_write_reqs).name("num_write_reqs_{}", m_channel_id);
    register_stat(s_num_other_reqs).name("num_other_reqs_{}", m_channel_id);
    register_stat(s_num_foreground_read_reqs)
        .name("num_foreground_read_reqs_{}", m_channel_id);
    register_stat(s_num_foreground_write_reqs)
        .name("num_foreground_write_reqs_{}", m_channel_id);
    register_stat(s_num_background_read_reqs)
        .name("num_background_read_reqs_{}", m_channel_id);
    register_stat(s_num_background_write_reqs)
        .name("num_background_write_reqs_{}", m_channel_id);
    register_stat(s_num_shadow_read_reqs)
        .name("num_shadow_read_reqs_{}", m_channel_id);
    register_stat(s_num_shadow_write_reqs)
        .name("num_shadow_write_reqs_{}", m_channel_id);
    register_stat(s_tier_local_read_reqs)
        .name("tier_local_read_reqs_{}", m_channel_id);
    register_stat(s_tier_local_write_reqs)
        .name("tier_local_write_reqs_{}", m_channel_id);
    register_stat(s_cross_tier_read_reqs)
        .name("cross_tier_read_reqs_{}", m_channel_id);
    register_stat(s_cross_tier_write_reqs)
        .name("cross_tier_write_reqs_{}", m_channel_id);
    register_stat(s_vertical_copy_read_reqs)
        .name("vertical_copy_read_reqs_{}", m_channel_id);
    register_stat(s_vertical_copy_write_reqs)
        .name("vertical_copy_write_reqs_{}", m_channel_id);
    register_stat(s_explicit_source_tier_hint_reqs)
        .name("explicit_source_tier_hint_reqs_{}", m_channel_id);
    register_stat(s_explicit_destination_tier_hint_reqs)
        .name("explicit_destination_tier_hint_reqs_{}", m_channel_id);
    register_stat(s_fallback_source_tier_reqs)
        .name("fallback_source_tier_reqs_{}", m_channel_id);
    register_stat(s_fallback_destination_tier_reqs)
        .name("fallback_destination_tier_reqs_{}", m_channel_id);
    register_stat(s_queue_len).name("queue_len_{}", m_channel_id);
    register_stat(s_read_queue_len).name("read_queue_len_{}", m_channel_id);
    register_stat(s_write_queue_len).name("write_queue_len_{}", m_channel_id);
    register_stat(s_priority_queue_len)
        .name("priority_queue_len_{}", m_channel_id);
    register_stat(s_queue_len_avg).name("queue_len_avg_{}", m_channel_id);
    register_stat(s_read_queue_len_avg)
        .name("read_queue_len_avg_{}", m_channel_id);
    register_stat(s_write_queue_len_avg)
        .name("write_queue_len_avg_{}", m_channel_id);
    register_stat(s_priority_queue_len_avg)
        .name("priority_queue_len_avg_{}", m_channel_id);

    for (const auto traffic_class : kTrafficClasses) {
      const size_t idx = traffic_class_index(traffic_class);
      const char* prefix = traffic_class_stat_prefix(traffic_class);
      register_stat(s_queue_len_by_class[idx])
          .name("{}_queue_len_{}", prefix, m_channel_id);
      register_stat(s_queue_len_avg_by_class[idx])
          .name("{}_queue_len_avg_{}", prefix, m_channel_id);
      register_stat(s_num_completed_reqs_by_class[idx])
          .name("num_{}_completed_reqs_{}", prefix, m_channel_id);
      register_stat(s_queue_wait_cycles_by_class[idx])
          .name("{}_queue_wait_cycles_{}", prefix, m_channel_id);
      register_stat(s_avg_queue_wait_cycles_by_class[idx])
          .name("avg_{}_queue_wait_cycles_{}", prefix, m_channel_id);
      register_stat(s_max_queue_wait_cycles_by_class[idx])
          .name("max_{}_queue_wait_cycles_{}", prefix, m_channel_id);
    }
    register_stat(s_qos_budget_blocked_events_by_class[traffic_class_index(
                      ExternalTrafficClass::kBackground)])
        .name("background_qos_budget_blocked_events_{}", m_channel_id);
    register_stat(s_qos_budget_blocked_events_by_class[traffic_class_index(
                      ExternalTrafficClass::kShadow)])
        .name("shadow_qos_budget_blocked_events_{}", m_channel_id);
    register_stat(s_qos_age_promotions_by_class[traffic_class_index(
                      ExternalTrafficClass::kBackground)])
        .name("background_qos_age_promotions_{}", m_channel_id);
    register_stat(s_qos_age_promotions_by_class[traffic_class_index(
                      ExternalTrafficClass::kShadow)])
        .name("shadow_qos_age_promotions_{}", m_channel_id);
    register_stat(s_foreground_qos_wins)
        .name("foreground_qos_wins_{}", m_channel_id);
    register_stat(s_write_mode_enter_count)
        .name("write_mode_enter_count_{}", m_channel_id);
    register_stat(s_write_mode_exit_count)
        .name("write_mode_exit_count_{}", m_channel_id);
    register_stat(s_write_mode_fg_interrupt_count)
        .name("write_mode_fg_interrupt_count_{}", m_channel_id);

    register_stat(s_read_latency).name("read_latency_{}", m_channel_id);
    register_stat(s_avg_read_latency).name("avg_read_latency_{}", m_channel_id);

    register_stat(s_max_cmds_issued_per_cycle)
        .name("max_cmds_issued_per_cycle_{}", m_channel_id);
    register_stat(s_max_access_cmds_issued_per_cycle)
        .name("max_access_cmds_issued_per_cycle_{}", m_channel_id);
    register_stat(s_max_shared_access_cmds_issued_per_cycle)
        .name("max_shared_access_cmds_issued_per_cycle_{}", m_channel_id);
    register_stat(s_max_vertical_transfer_cmds_issued_per_cycle)
        .name("max_vertical_transfer_cmds_issued_per_cycle_{}", m_channel_id);
    register_stat(s_cmds_issued_total).name("cmds_issued_total_{}", m_channel_id);
    register_stat(s_access_cmds_issued_total)
        .name("access_cmds_issued_total_{}", m_channel_id);
    register_stat(s_local_tier_access_cmds_issued_total)
        .name("local_tier_access_cmds_issued_total_{}", m_channel_id);
    register_stat(s_shared_access_cmds_issued_total)
        .name("shared_access_cmds_issued_total_{}", m_channel_id);
    register_stat(s_vertical_transfer_cmds_issued_total)
        .name("vertical_transfer_cmds_issued_total_{}", m_channel_id);
    register_stat(s_cross_tier_cmds_issued_total)
        .name("cross_tier_cmds_issued_total_{}", m_channel_id);
    register_stat(s_vertical_copy_cmds_issued_total)
        .name("vertical_copy_cmds_issued_total_{}", m_channel_id);
    register_stat(s_vertical_port_busy_cycles)
        .name("vertical_port_busy_cycles_{}", m_channel_id);
    register_stat(s_cross_tier_transfer_cycles_total)
        .name("cross_tier_transfer_cycles_total_{}", m_channel_id);
    register_stat(s_vertical_copy_transfer_cycles_total)
        .name("vertical_copy_transfer_cycles_total_{}", m_channel_id);
    register_stat(s_cross_tier_hop_latency_cycles_total)
        .name("cross_tier_hop_latency_cycles_total_{}", m_channel_id);
    register_stat(s_vertical_copy_hop_latency_cycles_total)
        .name("vertical_copy_hop_latency_cycles_total_{}", m_channel_id);
    register_stat(s_cross_tier_source_endpoint_cycles_total)
        .name("cross_tier_source_endpoint_cycles_total_{}", m_channel_id);
    register_stat(s_cross_tier_destination_endpoint_cycles_total)
        .name("cross_tier_destination_endpoint_cycles_total_{}", m_channel_id);
    register_stat(s_vertical_copy_source_endpoint_cycles_total)
        .name("vertical_copy_source_endpoint_cycles_total_{}", m_channel_id);
    register_stat(s_vertical_copy_destination_endpoint_cycles_total)
        .name("vertical_copy_destination_endpoint_cycles_total_{}", m_channel_id);
    register_stat(s_cross_tier_hops_total)
        .name("cross_tier_hops_total_{}", m_channel_id);
    register_stat(s_vertical_copy_hops_total)
        .name("vertical_copy_hops_total_{}", m_channel_id);
    register_stat(s_avg_cmds_issued_per_cycle)
        .name("avg_cmds_issued_per_cycle_{}", m_channel_id);
    register_stat(s_avg_access_cmds_issued_per_cycle)
        .name("avg_access_cmds_issued_per_cycle_{}", m_channel_id);
    register_stat(s_avg_shared_access_cmds_issued_per_cycle)
        .name("avg_shared_access_cmds_issued_per_cycle_{}", m_channel_id);
    register_stat(s_avg_vertical_transfer_cmds_issued_per_cycle)
        .name("avg_vertical_transfer_cmds_issued_per_cycle_{}", m_channel_id);
    for (size_t tier = 0; tier < m_num_tiers; tier++) {
      register_stat(s_tier_access_cmds_issued_total[tier])
          .name("tier_access_cmds_issued_total_t{}_{}", tier, m_channel_id);
      register_stat(s_max_tier_access_cmds_issued_per_cycle[tier])
          .name("max_tier_access_cmds_issued_per_cycle_t{}_{}", tier,
                m_channel_id);
      register_stat(s_avg_tier_access_cmds_issued_per_cycle[tier])
          .name("avg_tier_access_cmds_issued_per_cycle_t{}_{}", tier,
                m_channel_id);
      for (size_t dst_tier = 0; dst_tier < m_num_tiers; dst_tier++) {
        register_stat(s_cross_tier_reqs_by_src_dst[tier][dst_tier])
            .name("cross_tier_reqs_src_t{}_dst_t{}_{}", tier, dst_tier,
                  m_channel_id);
        register_stat(s_vertical_copy_reqs_by_src_dst[tier][dst_tier])
            .name("vertical_copy_reqs_src_t{}_dst_t{}_{}", tier, dst_tier,
                  m_channel_id);
      }
    }
    for (size_t hops = 1; hops < std::max<size_t>(1, m_num_tiers); hops++) {
      register_stat(s_cross_tier_reqs_by_hops[hops])
          .name("cross_tier_reqs_hop_{}_{}", hops, m_channel_id);
      register_stat(s_vertical_copy_reqs_by_hops[hops])
          .name("vertical_copy_reqs_hop_{}_{}", hops, m_channel_id);
    }
    for (size_t link = 0; link < s_vertical_link_busy_cycles.size(); link++) {
      register_stat(s_vertical_link_busy_cycles[link])
          .name("vertical_link_busy_cycles_l{}_{}", link, m_channel_id);
    }

    register_stat(s_shadow_scoreboard_diff_checks)
        .name("shadow_scoreboard_diff_checks_{}", m_channel_id);
    register_stat(s_shadow_scoreboard_rowhit_mismatches)
        .name("shadow_scoreboard_rowhit_mismatches_{}", m_channel_id);
    register_stat(s_shadow_scoreboard_rowopen_mismatches)
        .name("shadow_scoreboard_rowopen_mismatches_{}", m_channel_id);
    register_stat(s_shadow_scoreboard_prereq_checks)
        .name("shadow_scoreboard_prereq_checks_{}", m_channel_id);
    register_stat(s_shadow_scoreboard_prereq_mismatches)
        .name("shadow_scoreboard_prereq_mismatches_{}", m_channel_id);
    register_stat(s_controller_prereq_scoreboard_misses)
        .name("controller_prereq_scoreboard_misses_{}", m_channel_id);
    register_stat(s_controller_rowstate_scoreboard_misses)
        .name("controller_rowstate_scoreboard_misses_{}", m_channel_id);
    register_stat(s_shadow_scoreboard_ready_checks)
        .name("shadow_scoreboard_ready_checks_{}", m_channel_id);
    register_stat(s_shadow_scoreboard_ready_mismatches)
        .name("shadow_scoreboard_ready_mismatches_{}", m_channel_id);
    register_stat(s_shadow_scoreboard_ready_oracle_blocked_while_scoreboard_ready)
        .name("shadow_scoreboard_ready_oracle_blocked_while_scoreboard_ready_{}",
              m_channel_id);
    register_stat(s_shadow_scoreboard_ready_oracle_ready_while_scoreboard_blocked)
        .name("shadow_scoreboard_ready_oracle_ready_while_scoreboard_blocked_{}",
              m_channel_id);
    register_stat(s_shadow_scoreboard_ready_mismatch_open)
        .name("shadow_scoreboard_ready_mismatch_open_{}", m_channel_id);
    register_stat(s_shadow_scoreboard_ready_mismatch_access)
        .name("shadow_scoreboard_ready_mismatch_access_{}", m_channel_id);
    register_stat(s_shadow_scoreboard_ready_mismatch_close)
        .name("shadow_scoreboard_ready_mismatch_close_{}", m_channel_id);
    register_stat(s_shadow_scoreboard_ready_mismatch_other)
        .name("shadow_scoreboard_ready_mismatch_other_{}", m_channel_id);
    register_stat(s_shadow_scoreboard_ready_blocked_by_scoreboard)
        .name("shadow_scoreboard_ready_blocked_by_scoreboard_{}", m_channel_id);
    register_stat(s_shadow_scoreboard_ready_blocked_by_oracle)
        .name("shadow_scoreboard_ready_blocked_by_oracle_{}", m_channel_id);
  };

  bool send(Request& req) override {
    req.final_command = m_dram->m_request_translations(req.type_id);

    if (req.type_id == Request::Type::Read) {
      auto compare_addr = [req](const Request& wreq) {
        return wreq.addr == req.addr;
      };
      if (std::find_if(m_write_buffer.begin(), m_write_buffer.end(),
                       compare_addr) != m_write_buffer.end()) {
        req.arrive = m_clk;
        req.depart = m_clk + 1;
        note_accepted_request_stats(req);
        pending.push_back(req);
        adjust_counted_queue_occupancy(req, +1);
        return true;
      }
    }

    bool is_success = false;
    req.arrive = m_clk;
    if (req.type_id == Request::Type::Read) {
      is_success = m_read_buffer.enqueue(req);
    } else if (req.type_id == Request::Type::Write) {
      is_success = m_write_buffer.enqueue(req);
    } else {
      throw std::runtime_error("Invalid request type!");
    }
    if (!is_success) {
      req.arrive = -1;
      return false;
    }

    note_accepted_request_stats(req);
    adjust_counted_queue_occupancy(req, +1);
    return true;
  };

  bool priority_send(Request& req) override {
    req.final_command = m_dram->m_request_translations(req.type_id);
    req.arrive = m_clk;
    const bool ok = m_priority_buffer.enqueue(req);
    if (ok) {
      adjust_counted_queue_occupancy(req, +1);
      if (controller_scoreboard_enabled() &&
          m_dram->m_command_meta(req.final_command).is_refreshing) {
        m_shadow_scoreboard.on_refresh_scope_pending_from_command(
            m_dram, req.final_command, req.addr_vec, m_clk);
      }
    }
    return ok;
  }

  int get_prereq_command(int final_command,
                         const AddrVec_t& addr_vec) const override {
    return resolve_scheduling_state(final_command, addr_vec).next_command;
  }

  bool query_command_state(int final_command, const AddrVec_t& addr_vec,
                           ControllerCommandState& result) const override {
    result = ControllerCommandState {};
    const SchedulingState scheduling_state =
        resolve_scheduling_state(final_command, addr_vec);
    if (scheduling_state.next_command < 0) {
      return false;
    }

    result.valid = true;
    result.next_command = scheduling_state.next_command;
    result.next_command_ready = scheduling_state.next_command_ready;
    result.ready_block_reason = controller_ready_block_reason_from_scoreboard(
        scheduling_state.ready_block_reason);
    return true;
  }

  bool is_command_ready(int command, const AddrVec_t& addr_vec) const override {
    if (command < 0) return false;
    if (controller_scoreboard_enabled()) {
      return m_shadow_scoreboard.is_command_ready(m_dram, command, addr_vec,
                                                 m_clk);
    }

    if (!m_dram) return false;
    return m_dram->check_ready(command, addr_vec);
  }

  bool probe_rowbuffer(int final_command, const AddrVec_t& addr_vec,
                       int& result) const override {
    result = 0;
    if (!controller_scoreboard_enabled()) {
      return false;
    }
    const ProbeResult probe =
        m_shadow_scoreboard.probe(m_dram, final_command, addr_vec);
    if (!probe.valid) {
      return false;
    }
    result = probe.row_hit ? 1 : (probe.row_open ? 2 : 0);
    return true;
  }

  bool query_telemetry(int final_command, const AddrVec_t& addr_vec,
                       ControllerTelemetry& result) const override {
    result = ControllerTelemetry {};
    if (!controller_scoreboard_enabled()) {
      return false;
    }

    const TelemetryResult telemetry = m_shadow_scoreboard.query_telemetry(
        m_dram, final_command, addr_vec, m_clk);
    if (!telemetry.valid) {
      return false;
    }

    const uint64_t queue_occupancy =
        static_cast<uint64_t>(m_active_buffer.size()) +
        static_cast<uint64_t>(m_read_buffer.size()) +
        static_cast<uint64_t>(m_write_buffer.size()) +
        static_cast<uint64_t>(m_priority_buffer.size());
    const uint64_t queue_capacity =
        static_cast<uint64_t>(m_active_buffer.max_size) +
        static_cast<uint64_t>(m_read_buffer.max_size) +
        static_cast<uint64_t>(m_write_buffer.max_size) +
        static_cast<uint64_t>(m_priority_buffer.max_size);

    result.valid = true;
    result.rowbuffer_state =
        telemetry.row_hit ? 1 : (telemetry.row_open ? 2 : 0);
    result.bank_busy = telemetry.bank_busy;
    result.refreshing = telemetry.refreshing;
    result.refresh_horizon_cycles = telemetry.refresh_horizon_cycles;
    result.queue_occupancy = queue_occupancy;
    result.queue_capacity = queue_capacity;
    result.queue_pressure_permille =
        queue_capacity == 0
            ? 0
            : static_cast<uint32_t>(
                  (queue_occupancy * 1000ULL + queue_capacity / 2ULL) /
                  queue_capacity);
    result.refresh_epoch = telemetry.refresh_epoch;
    result.refresh_mode = ControllerRefreshMode::kUnknown;
    return true;
  }

  bool query_telemetry_summary_v2(
      ControllerTelemetrySummaryV2& result) const override {
    result = ControllerTelemetrySummaryV2 {};
    if (!controller_scoreboard_enabled()) {
      return false;
    }

    const RefreshStateSnapshot refresh_state =
        m_shadow_scoreboard.snapshot_global_refresh_state(m_clk);
    const RefreshScheduleHint refresh_hint =
        m_refresh ? m_refresh->query_refresh_schedule_hint()
                  : RefreshScheduleHint {};
    const uint64_t queue_active_occupancy =
        static_cast<uint64_t>(m_active_buffer.size());
    const uint64_t queue_read_occupancy =
        static_cast<uint64_t>(m_read_buffer.size());
    const uint64_t queue_write_occupancy =
        static_cast<uint64_t>(m_write_buffer.size());
    const uint64_t queue_priority_occupancy =
        static_cast<uint64_t>(m_priority_buffer.size());
    const uint64_t queue_pending_occupancy =
        static_cast<uint64_t>(pending.size());
    const uint64_t queue_total_occupancy =
        queue_active_occupancy + queue_read_occupancy + queue_write_occupancy +
        queue_priority_occupancy;
    const uint64_t queue_active_capacity =
        static_cast<uint64_t>(m_active_buffer.max_size);
    const uint64_t queue_read_capacity =
        static_cast<uint64_t>(m_read_buffer.max_size);
    const uint64_t queue_write_capacity =
        static_cast<uint64_t>(m_write_buffer.max_size);
    const uint64_t queue_priority_capacity =
        static_cast<uint64_t>(m_priority_buffer.max_size);
    const uint64_t queue_total_capacity =
        queue_active_capacity + queue_read_capacity + queue_write_capacity +
        queue_priority_capacity;
    const uint64_t queue_counted_foreground_occupancy =
        static_cast<uint64_t>(m_counted_queue_occupancy[traffic_class_index(
            ExternalTrafficClass::kForeground)]);
    const uint64_t queue_counted_background_occupancy =
        static_cast<uint64_t>(m_counted_queue_occupancy[traffic_class_index(
            ExternalTrafficClass::kBackground)]);
    const uint64_t queue_counted_shadow_occupancy =
        static_cast<uint64_t>(m_counted_queue_occupancy[traffic_class_index(
            ExternalTrafficClass::kShadow)]);

    result.valid = true;
    result.controller_count = 1;
    result.clk = static_cast<uint64_t>(m_clk);
    result.queue_total_occupancy = queue_total_occupancy;
    result.queue_total_capacity = queue_total_capacity;
    result.queue_total_pressure_permille =
        queue_total_capacity == 0
            ? 0
            : static_cast<uint32_t>(
                  (queue_total_occupancy * 1000ULL + queue_total_capacity / 2ULL) /
                  queue_total_capacity);
    result.queue_active_occupancy = queue_active_occupancy;
    result.queue_active_capacity = queue_active_capacity;
    result.queue_read_occupancy = queue_read_occupancy;
    result.queue_read_capacity = queue_read_capacity;
    result.queue_write_occupancy = queue_write_occupancy;
    result.queue_write_capacity = queue_write_capacity;
    result.queue_priority_occupancy = queue_priority_occupancy;
    result.queue_priority_capacity = queue_priority_capacity;
    result.queue_pending_occupancy = queue_pending_occupancy;
    result.queue_counted_foreground_occupancy =
        queue_counted_foreground_occupancy;
    result.queue_counted_background_occupancy =
        queue_counted_background_occupancy;
    result.queue_counted_shadow_occupancy = queue_counted_shadow_occupancy;
    result.queue_counted_total_occupancy =
        queue_counted_foreground_occupancy +
        queue_counted_background_occupancy +
        queue_counted_shadow_occupancy;
    result.cmd_issue_budget = std::max<uint32_t>(1, m_cmd_issue_width);
    result.access_budget = std::max<uint32_t>(1, m_shared_access_ports);
    result.open_banks =
        static_cast<uint64_t>(m_shadow_scoreboard.count_open_banks());
    result.inflight_banks =
        static_cast<uint64_t>(m_shadow_scoreboard.count_inflight_banks());
    result.autoprecharge_armed_banks = static_cast<uint64_t>(
        m_shadow_scoreboard.count_autoprecharge_armed_banks());
    result.max_open_row_age_cycles =
        m_shadow_scoreboard.max_open_row_age_cycles(m_clk);
    result.refresh_pending = refresh_state.pending;
    result.refresh_active = refresh_state.active;
    result.refresh_recovery = refresh_state.recovery;
    result.refresh_epoch = refresh_state.epoch;
    result.refresh_horizon_cycles = refresh_state.horizon_cycles;
    result.refresh_window_cycles =
        m_refresh_window_cycles > 0 ? static_cast<uint64_t>(m_refresh_window_cycles)
                                    : 0;
    result.next_refresh_deadline_cycles =
        refresh_hint.valid ? refresh_hint.next_refresh_deadline_cycles : 0;
    result.refresh_slack_cycles =
        controller_refresh_slack_cycles_from_hint(refresh_hint);
    result.refresh_pressure_permille =
        controller_refresh_pressure_permille_from_hint(
            refresh_hint, result.refresh_pending, result.refresh_active,
            result.refresh_recovery);
    result.refresh_pending_banks =
        static_cast<uint64_t>(m_shadow_scoreboard.count_refresh_pending_banks());
    result.refreshing_banks =
        static_cast<uint64_t>(m_shadow_scoreboard.count_refreshing_banks());
    result.refresh_scope =
        controller_refresh_scope_from_scoreboard(refresh_state.owner_scope);
    result.refresh_mode = ControllerRefreshMode::kUnknown;
    result.ready_blocked =
        m_last_ready_block_reason != ControllerReadyBlockReason::kNone;
    result.ready_block_reason = m_last_ready_block_reason;
    result.thermal_valid = false;
    result.temperature_bucket = ControllerTemperatureBucket::kUnknown;
    result.tiered_valid = true;
    result.num_tiers = static_cast<uint32_t>(m_num_tiers);
    result.shared_access_budget = std::max<uint32_t>(1, m_shared_access_ports);
    result.tier_access_budget = std::max<uint32_t>(1, m_tier_access_ports);
    result.vertical_transfer_budget = per_link_vertical_topology_enabled()
                                         ? m_vertical_link_ports_per_hop
                                         : m_vertical_transfer_ports;
    result.vertical_transfer_active =
        static_cast<uint32_t>(active_vertical_transfers_total());
    return true;
  }

  bool query_telemetry_observation_sample_v1(
      ControllerTelemetryObservationSampleV1& result) const override {
    result = ControllerTelemetryObservationSampleV1 {};
    if (!query_telemetry_summary_v2(result.summary) || !result.summary.valid) {
      result.summary = ControllerTelemetrySummaryV2 {};
      return false;
    }

    result.valid = true;
    result.row_hits = static_cast<uint64_t>(s_row_hits);
    result.row_misses = static_cast<uint64_t>(s_row_misses);
    result.row_conflicts = static_cast<uint64_t>(s_row_conflicts);
    result.foreground_row_hits = 0;
    result.foreground_row_misses = 0;
    result.foreground_row_conflicts = 0;
    result.background_row_hits = 0;
    result.background_row_misses = 0;
    result.background_row_conflicts = 0;
    result.shadow_row_hits = 0;
    result.shadow_row_misses = 0;
    result.shadow_row_conflicts = 0;

    result.local_accesses =
        static_cast<uint64_t>(s_tier_local_read_reqs + s_tier_local_write_reqs);
    result.cross_tier_accesses = static_cast<uint64_t>(
        s_cross_tier_read_reqs + s_cross_tier_write_reqs);
    result.vertical_copy_accesses = static_cast<uint64_t>(
        s_vertical_copy_read_reqs + s_vertical_copy_write_reqs);
    return true;
  }

  void tick() override {
    m_clk++;
    m_last_ready_block_reason = ControllerReadyBlockReason::kNone;
    retire_completed_vertical_transfers();

    s_queue_len +=
        m_read_buffer.size() + m_write_buffer.size() + m_priority_buffer.size() +
        pending.size();
    s_read_queue_len += m_read_buffer.size() + pending.size();
    s_write_queue_len += m_write_buffer.size();
    s_priority_queue_len += m_priority_buffer.size();
    for (const auto traffic_class : kTrafficClasses) {
      const size_t idx = traffic_class_index(traffic_class);
      s_queue_len_by_class[idx] += m_counted_queue_occupancy[idx];
    }

    serve_completed_pending();

    m_refresh->tick();

    std::unordered_set<std::string> used_access_banks;
    used_access_banks.reserve(std::max<uint32_t>(1, m_shared_access_ports) * 2);
    std::vector<uint32_t> tier_access_cmds_issued_this_cycle(m_num_tiers, 0);

    const uint32_t issue_budget = std::max<uint32_t>(1, m_cmd_issue_width);
    const uint32_t shared_access_budget =
        std::max<uint32_t>(1, m_shared_access_ports);
    const uint32_t tier_access_budget =
        std::max<uint32_t>(1, m_tier_access_ports);
    uint32_t cmds_issued_this_cycle = 0;
    uint32_t access_cmds_issued_this_cycle = 0;
    uint32_t shared_access_cmds_issued_this_cycle = 0;
    uint32_t vertical_transfer_cmds_issued_this_cycle = 0;
    uint32_t background_cmds_issued_this_cycle = 0;
    uint32_t shadow_cmds_issued_this_cycle = 0;
    for (uint32_t issued = 0; issued < issue_budget; ++issued) {
      ReqBuffer::iterator req_it;
      ReqBuffer* buffer = nullptr;
      const bool allow_access =
          (shared_access_cmds_issued_this_cycle < shared_access_budget);
      bool request_found = schedule_request_filtered(
          req_it, buffer, used_access_banks, allow_access,
          background_cmds_issued_this_cycle, shadow_cmds_issued_this_cycle,
          tier_access_cmds_issued_this_cycle, tier_access_budget);
      if (!request_found) {
        break;
      }

      m_rowpolicy->update(request_found, req_it);

      for (auto plugin : m_plugins) {
        plugin->update(request_found, req_it);
      }

      if (req_it->is_stat_updated == false) {
        update_request_stats(req_it);
      }

      const ExternalTrafficClass issued_traffic_class =
          get_external_traffic_class(*req_it);
      const int command = req_it->command;
      m_dram->issue_command(command, req_it->addr_vec);
      if (controller_scoreboard_enabled()) {
        m_shadow_scoreboard.on_issue_command(m_dram, command, req_it->addr_vec,
                                             m_clk);
        note_post_issue_overlay_diff(*req_it, command);
      }
      cmds_issued_this_cycle++;
      if (issued_traffic_class == ExternalTrafficClass::kBackground) {
        background_cmds_issued_this_cycle++;
      } else if (issued_traffic_class == ExternalTrafficClass::kShadow) {
        shadow_cmds_issued_this_cycle++;
      }

      if (m_dram->m_command_meta(command).is_accessing) {
        used_access_banks.insert(bank_key(req_it->addr_vec));
        access_cmds_issued_this_cycle++;
        shared_access_cmds_issued_this_cycle++;
        const int tier = request_access_tier(*req_it);
        if (tier >= 0 &&
            static_cast<size_t>(tier) < tier_access_cmds_issued_this_cycle.size()) {
          tier_access_cmds_issued_this_cycle[tier]++;
        }
        if (requires_vertical_transfer(*req_it)) {
          vertical_transfer_cmds_issued_this_cycle++;
          const Clk_t occupancy_cycles =
              per_link_vertical_topology_enabled()
                  ? vertical_link_occupancy_cycles(*req_it)
                  : vertical_transfer_occupancy_cycles(*req_it);
          const Clk_t hop_latency_cycles =
              vertical_transfer_hop_latency_cycles(*req_it);
          const Clk_t source_endpoint_cycles =
              source_endpoint_stage_cycles(*req_it);
          const Clk_t destination_endpoint_cycles =
              destination_endpoint_stage_cycles(*req_it);
          if (is_vertical_copy_path(*req_it)) {
            s_vertical_copy_cmds_issued_total++;
            s_vertical_copy_transfer_cycles_total +=
                static_cast<size_t>(std::max<Clk_t>(0, occupancy_cycles));
            s_vertical_copy_hop_latency_cycles_total +=
                static_cast<size_t>(std::max<Clk_t>(0, hop_latency_cycles));
            s_vertical_copy_source_endpoint_cycles_total +=
                static_cast<size_t>(std::max<Clk_t>(0, source_endpoint_cycles));
            s_vertical_copy_destination_endpoint_cycles_total +=
                static_cast<size_t>(std::max<Clk_t>(0, destination_endpoint_cycles));
          } else {
            s_cross_tier_cmds_issued_total++;
            s_cross_tier_transfer_cycles_total +=
                static_cast<size_t>(std::max<Clk_t>(0, occupancy_cycles));
            s_cross_tier_hop_latency_cycles_total +=
                static_cast<size_t>(std::max<Clk_t>(0, hop_latency_cycles));
            s_cross_tier_source_endpoint_cycles_total +=
                static_cast<size_t>(std::max<Clk_t>(0, source_endpoint_cycles));
            s_cross_tier_destination_endpoint_cycles_total += static_cast<size_t>(
                std::max<Clk_t>(0, destination_endpoint_cycles));
          }
          reserve_vertical_transfer(*req_it);
        } else {
          s_local_tier_access_cmds_issued_total++;
        }
      }

      if (command == req_it->final_command) {
        Request completed_req = *req_it;
        completed_req.depart = completion_depart_cycle(completed_req);
        move_request_to_pending(completed_req, buffer);
        buffer->remove(req_it);
      } else if (m_dram->m_command_meta(command).is_opening) {
        const Request active_req = *req_it;
        if (m_active_buffer.enqueue(active_req)) {
          remove_request_from_counted_buffer(active_req, buffer);
          buffer->remove(req_it);
        }
      }
    }

    s_cmds_issued_total += cmds_issued_this_cycle;
    s_access_cmds_issued_total += access_cmds_issued_this_cycle;
    s_shared_access_cmds_issued_total += shared_access_cmds_issued_this_cycle;
    s_vertical_transfer_cmds_issued_total +=
        vertical_transfer_cmds_issued_this_cycle;
    s_max_cmds_issued_per_cycle =
        std::max<size_t>(s_max_cmds_issued_per_cycle, cmds_issued_this_cycle);
    s_max_access_cmds_issued_per_cycle = std::max<size_t>(
        s_max_access_cmds_issued_per_cycle, access_cmds_issued_this_cycle);
    s_max_shared_access_cmds_issued_per_cycle =
        std::max<size_t>(s_max_shared_access_cmds_issued_per_cycle,
                         shared_access_cmds_issued_this_cycle);
    s_max_vertical_transfer_cmds_issued_per_cycle = std::max<size_t>(
        s_max_vertical_transfer_cmds_issued_per_cycle,
        vertical_transfer_cmds_issued_this_cycle);
    for (size_t tier = 0; tier < m_num_tiers; tier++) {
      s_tier_access_cmds_issued_total[tier] +=
          tier_access_cmds_issued_this_cycle[tier];
      s_max_tier_access_cmds_issued_per_cycle[tier] = std::max<size_t>(
          s_max_tier_access_cmds_issued_per_cycle[tier],
          tier_access_cmds_issued_this_cycle[tier]);
    }
    if (has_any_active_vertical_transfer()) {
      s_vertical_port_busy_cycles++;
    }
    for (size_t link = 0; link < m_active_vertical_link_releases.size(); link++) {
      if (!m_active_vertical_link_releases[link].empty()) {
        s_vertical_link_busy_cycles[link]++;
      }
    }
  };

 private:
  Clk_t detect_refresh_window_cycles() const {
    if (!m_dram) return -1;

    constexpr std::array<const char*, 7> kRefreshTimingNames = {
        "nRFC", "nRFCab", "nRFCpb", "nRFC1", "nRFC2", "nRFCSB", "nRFCPB",
    };
    for (const char* name : kRefreshTimingNames) {
      try {
        const int value = m_dram->m_timing_vals(name);
        if (value > 0) {
          return static_cast<Clk_t>(value);
        }
      } catch (const std::out_of_range&) {
      }
    }

    return -1;
  }

  bool is_counted_buffer(const ReqBuffer* buffer) const {
    return buffer == &m_read_buffer || buffer == &m_write_buffer ||
           buffer == &m_priority_buffer;
  }

  void adjust_counted_queue_occupancy(const Request& req, int delta) {
    const size_t idx = traffic_class_index(get_external_traffic_class(req));
    if (delta >= 0) {
      m_counted_queue_occupancy[idx] += static_cast<size_t>(delta);
      return;
    }
    const size_t amount = static_cast<size_t>(-delta);
    if (m_counted_queue_occupancy[idx] >= amount) {
      m_counted_queue_occupancy[idx] -= amount;
    } else {
      m_counted_queue_occupancy[idx] = 0;
    }
  }

  void remove_request_from_counted_buffer(const Request& req, ReqBuffer* buffer) {
    if (!is_counted_buffer(buffer)) return;
    adjust_counted_queue_occupancy(req, -1);
  }

  void move_request_to_pending(const Request& req, ReqBuffer* from_buffer) {
    remove_request_from_counted_buffer(req, from_buffer);
    pending.push_back(req);
    adjust_counted_queue_occupancy(req, +1);
  }

  size_t queue_wait_cycles(const Request& req) const {
    if (req.arrive < 0 || m_clk < req.arrive) return 0;
    return static_cast<size_t>(m_clk - req.arrive);
  }

  bool controller_scoreboard_enabled() const {
    return m_shadow_scoreboard_enable && m_shadow_scoreboard.valid();
  }

  SchedulingState resolve_scheduling_state(int final_command,
                                           const AddrVec_t& addr_vec) const {
    if (controller_scoreboard_enabled()) {
      return m_shadow_scoreboard.resolve_scheduling_state(
          m_dram, final_command, addr_vec, m_clk);
    }

    SchedulingState fallback {};
    if (!m_dram || final_command < 0) {
      return fallback;
    }

    fallback.next_command = m_dram->get_preq_command(final_command, addr_vec);
    if (fallback.next_command >= 0) {
      fallback.next_command_ready =
          m_dram->check_ready(fallback.next_command, addr_vec);
    }
    if (m_dram->m_command_meta(final_command).is_refreshing) {
      fallback.row_state.refreshing = true;
    } else {
      fallback.row_state.valid = true;
      fallback.row_state.row_hit =
          m_dram->check_rowbuffer_hit(final_command, addr_vec);
      fallback.row_state.row_open =
          m_dram->check_node_open(final_command, addr_vec);
    }
    fallback.valid = (fallback.next_command >= 0) || fallback.row_state.valid ||
                     fallback.row_state.refreshing;
    return fallback;
  }

  bool oracle_debug_overlay_enabled() const {
    return controller_scoreboard_enabled() &&
           (m_shadow_scoreboard_debug_overlay_enable ||
            m_shadow_scoreboard_log_mismatch ||
            m_shadow_scoreboard_fail_fast);
  }

  void note_prereq_overlay_mismatch(int final_command,
                                    const AddrVec_t& addr_vec,
                                    int controller_cmd) {
    if (!oracle_debug_overlay_enabled()) return;

    s_shadow_scoreboard_prereq_checks++;
    const int oracle_cmd = m_dram->get_preq_command(final_command, addr_vec);
    if (oracle_cmd == controller_cmd) return;

    s_shadow_scoreboard_prereq_mismatches++;
    if (!(m_shadow_scoreboard_log_mismatch || m_shadow_scoreboard_fail_fast)) {
      return;
    }

    std::ostringstream oss;
    oss << "TieredM3D channel " << m_channel_id
        << ": prereq mismatch final=" << final_command
        << " controller=" << controller_cmd << " oracle=" << oracle_cmd
        << " bank=" << bank_key(addr_vec);
    const std::string msg = oss.str();
    if (m_shadow_scoreboard_log_mismatch) {
      spdlog::warn("{}", msg);
    }
    if (m_shadow_scoreboard_fail_fast) {
      throw std::runtime_error(msg);
    }
  }

  void note_ready_overlay_mismatch(int command, const AddrVec_t& addr_vec,
                                   bool controller_ready) {
    if (!oracle_debug_overlay_enabled()) return;

    const bool oracle_ready = m_dram->check_ready(command, addr_vec);
    s_shadow_scoreboard_ready_checks++;
    if (controller_ready != oracle_ready) {
      s_shadow_scoreboard_ready_mismatches++;
      if (controller_ready && !oracle_ready) {
        s_shadow_scoreboard_ready_oracle_blocked_while_scoreboard_ready++;
      } else if (!controller_ready && oracle_ready) {
        s_shadow_scoreboard_ready_oracle_ready_while_scoreboard_blocked++;
      }
      const auto meta = m_dram->m_command_meta(command);
      if (meta.is_opening) {
        s_shadow_scoreboard_ready_mismatch_open++;
      } else if (meta.is_accessing) {
        s_shadow_scoreboard_ready_mismatch_access++;
      } else if (meta.is_closing) {
        s_shadow_scoreboard_ready_mismatch_close++;
      } else {
        s_shadow_scoreboard_ready_mismatch_other++;
      }

      if (m_shadow_scoreboard_log_mismatch || m_shadow_scoreboard_fail_fast) {
        std::ostringstream oss;
        oss << "TieredM3D channel " << m_channel_id
            << ": ready mismatch cmd=" << command
            << " controller=" << (controller_ready ? 1 : 0)
            << " oracle=" << (oracle_ready ? 1 : 0)
            << " bank=" << bank_key(addr_vec);
        const std::string msg = oss.str();
        if (m_shadow_scoreboard_log_mismatch) {
          spdlog::warn("{}", msg);
        }
        if (m_shadow_scoreboard_fail_fast) {
          throw std::runtime_error(msg);
        }
      }
    }
    if (!controller_ready) {
      s_shadow_scoreboard_ready_blocked_by_scoreboard++;
    }
    if (!oracle_ready) {
      s_shadow_scoreboard_ready_blocked_by_oracle++;
    }
  }

  void note_post_issue_overlay_diff(const Request& req, int command) {
    if (!oracle_debug_overlay_enabled()) return;

    const ShadowDiffResult diff = m_shadow_scoreboard.diff_against_dram(
        m_dram, req.final_command, req.addr_vec);
    if (!diff.valid) return;

    s_shadow_scoreboard_diff_checks++;
    if (!diff.row_hit_match) {
      s_shadow_scoreboard_rowhit_mismatches++;
    }
    if (!diff.row_open_match) {
      s_shadow_scoreboard_rowopen_mismatches++;
    }
    if ((diff.row_hit_match && diff.row_open_match) ||
        !(m_shadow_scoreboard_log_mismatch || m_shadow_scoreboard_fail_fast)) {
      return;
    }

    std::ostringstream oss;
    oss << "TieredM3D scoreboard/oracle overlay mismatch ch=" << m_channel_id
        << " cmd=" << command << " final=" << req.final_command
        << " addr=" << req.addr << " bank=" << bank_key(req.addr_vec)
        << " row_hit(ctrl/oracle)=" << diff.scoreboard_row_hit << "/"
        << diff.dram_row_hit << " row_open(ctrl/oracle)="
        << diff.scoreboard_row_open << "/" << diff.dram_row_open;
    const std::string msg = oss.str();
    if (m_shadow_scoreboard_log_mismatch) {
      spdlog::warn("{}", msg);
    }
    if (m_shadow_scoreboard_fail_fast) {
      throw std::runtime_error(msg);
    }
  }

  ProbeResult resolve_row_state_for_stats(int final_command,
                                          const AddrVec_t& addr_vec) {
    const SchedulingState scheduling_state =
        resolve_scheduling_state(final_command, addr_vec);
    if (controller_scoreboard_enabled() &&
        scheduling_state.rowstate_scoreboard_miss) {
      s_controller_rowstate_scoreboard_misses++;
    }
    return scheduling_state.row_state;
  }

  bool has_pending_foreground_demand() const {
    return m_counted_queue_occupancy[traffic_class_index(
               ExternalTrafficClass::kForeground)] > 0;
  }

  bool is_aged_non_foreground(const Request& req) const {
    if (!m_qos_enable) return false;
    if (m_non_foreground_starvation_threshold_cycles == 0) return false;
    const ExternalTrafficClass traffic_class = get_external_traffic_class(req);
    if (traffic_class == ExternalTrafficClass::kForeground) return false;
    return queue_wait_cycles(req) >=
           static_cast<size_t>(m_non_foreground_starvation_threshold_cycles);
  }

  int effective_traffic_rank(const Request& req) const {
    if (!m_qos_enable) return 0;
    if (is_aged_non_foreground(req)) return 0;
    return static_cast<int>(get_external_traffic_class(req));
  }

  bool prefer_request_qos(const Request& lhs, const Request& rhs) const {
    const int lhs_rank = effective_traffic_rank(lhs);
    const int rhs_rank = effective_traffic_rank(rhs);
    if (lhs_rank != rhs_rank) {
      return lhs_rank < rhs_rank;
    }
    return queue_wait_cycles(lhs) > queue_wait_cycles(rhs);
  }

  bool is_qos_budget_blocked(const Request& req,
                             uint32_t background_cmds_issued_this_cycle,
                             uint32_t shadow_cmds_issued_this_cycle) const {
    if (!m_qos_enable) return false;
    if (!has_pending_foreground_demand()) return false;
    if (is_aged_non_foreground(req)) return false;

    const ExternalTrafficClass traffic_class = get_external_traffic_class(req);
    if (traffic_class == ExternalTrafficClass::kBackground) {
      return m_background_cmd_budget_per_cycle != 0 &&
             background_cmds_issued_this_cycle >=
                 m_background_cmd_budget_per_cycle;
    }
    if (traffic_class == ExternalTrafficClass::kShadow) {
      return m_shadow_cmd_budget_per_cycle != 0 &&
             shadow_cmds_issued_this_cycle >= m_shadow_cmd_budget_per_cycle;
    }
    return false;
  }

  void note_qos_budget_blocked(const Request& req) {
    const size_t idx = traffic_class_index(get_external_traffic_class(req));
    s_qos_budget_blocked_events_by_class[idx]++;
  }

  void note_qos_age_promotion(const Request& req) {
    if (!is_aged_non_foreground(req)) return;
    const size_t idx = traffic_class_index(get_external_traffic_class(req));
    s_qos_age_promotions_by_class[idx]++;
  }

  void update_request_stats(ReqBuffer::iterator& req) {
    req->is_stat_updated = true;

    const ExternalTrafficClass traffic_class = get_external_traffic_class(*req);
    const size_t traffic_idx = traffic_class_index(traffic_class);
    const size_t wait_cycles = queue_wait_cycles(*req);
    s_queue_wait_cycles_by_class[traffic_idx] += wait_cycles;
    s_max_queue_wait_cycles_by_class[traffic_idx] =
        std::max(s_max_queue_wait_cycles_by_class[traffic_idx], wait_cycles);
    note_qos_age_promotion(*req);

    const ProbeResult probe =
        resolve_row_state_for_stats(req->final_command, req->addr_vec);
    const bool row_hit = probe.row_hit;
    const bool row_open = (!row_hit && probe.row_open);

    if (req->type_id == Request::Type::Read) {
      if (row_hit) {
        s_read_row_hits++;
        s_row_hits++;
        if (is_valid_source_id(*req, m_num_cores)) {
          s_read_row_hits_per_core[req->source_id]++;
        }
      } else if (row_open) {
        s_read_row_conflicts++;
        s_row_conflicts++;
        if (is_valid_source_id(*req, m_num_cores)) {
          s_read_row_conflicts_per_core[req->source_id]++;
        }
      } else {
        s_read_row_misses++;
        s_row_misses++;
        if (is_valid_source_id(*req, m_num_cores)) {
          s_read_row_misses_per_core[req->source_id]++;
        }
      }
    } else if (req->type_id == Request::Type::Write) {
      if (row_hit) {
        s_write_row_hits++;
        s_row_hits++;
      } else if (row_open) {
        s_write_row_conflicts++;
        s_row_conflicts++;
      } else {
        s_write_row_misses++;
        s_row_misses++;
      }
    }
  }

  void serve_completed_pending() {
    if (pending.empty()) return;

    // Complete all ready requests in this cycle (avoid HOL blocking).
    std::deque<Request> remain;
    for (auto& req : pending) {
      if (req.depart > m_clk) {
        remain.push_back(req);
        continue;
      }

      adjust_counted_queue_occupancy(req, -1);
      s_num_completed_reqs_by_class[traffic_class_index(
          get_external_traffic_class(req))]++;
      if (controller_scoreboard_enabled()) {
        m_shadow_scoreboard.on_request_completed(req, m_clk);
      }

      if (req.type_id == Request::Type::Read) {
        if (req.depart - req.arrive > 1) {
          s_read_latency += req.depart - req.arrive;
        }
      }

      if (req.callback) {
        req.callback(req);
      }
    }
    pending.swap(remain);
  };

  void set_write_mode() {
    const bool was_write_mode = m_is_write_mode;
    if (!m_is_write_mode) {
      if ((m_write_buffer.size() > m_wr_high_watermark * m_write_buffer.max_size) ||
          m_read_buffer.size() == 0) {
        m_is_write_mode = true;
      }
    } else {
      if ((m_write_buffer.size() < m_wr_low_watermark * m_write_buffer.max_size) &&
          m_read_buffer.size() != 0) {
        m_is_write_mode = false;
      }
    }
    if (!was_write_mode && m_is_write_mode) {
      s_write_mode_enter_count++;
    } else if (was_write_mode && !m_is_write_mode) {
      s_write_mode_exit_count++;
    }
  };

  std::string bank_key(const AddrVec_t& addr_vec) const {
    std::ostringstream oss;
    for (int i = 0; i < m_bank_addr_idx + 1; i++) {
      oss << addr_vec[i] << ",";
    }
    return oss.str();
  }

  int tier_id(const AddrVec_t& addr_vec) const {
    if (m_tier_addr_idx < 0 ||
        m_tier_addr_idx >= static_cast<int>(addr_vec.size())) {
      return 0;
    }
    return std::max(0, addr_vec[m_tier_addr_idx]);
  }

  int explicit_source_tier(const Request& req) const {
    if (req.source_tier_hint < 0) {
      return -1;
    }
    return clamp_tier_index(req.source_tier_hint);
  }

  int explicit_destination_tier(const Request& req) const {
    if (req.destination_tier_hint >= 0) {
      return clamp_tier_index(req.destination_tier_hint);
    }
    if (req.tier_hint >= 0) {
      return clamp_tier_index(req.tier_hint);
    }
    return -1;
  }

  bool is_vertical_copy_path(const Request& req) const {
    return req.path_class == Request::PathClass::VerticalCopy;
  }

  int clamp_tier_index(int tier) const {
    if (m_num_tiers == 0) {
      return 0;
    }
    if (tier < 0) {
      return 0;
    }
    const int max_tier = static_cast<int>(m_num_tiers - 1);
    return std::min(tier, max_tier);
  }

  int request_destination_tier(const Request& req) const {
    const int explicit_dst = explicit_destination_tier(req);
    if (explicit_dst >= 0) {
      return explicit_dst;
    }
    const int explicit_src = explicit_source_tier(req);
    if (explicit_src >= 0 && req.path_class == Request::PathClass::LocalAccess) {
      return explicit_src;
    }
    return clamp_tier_index(tier_id(req.addr_vec));
  }

  bool is_cross_tier_path(const Request& req) const {
    if (req.path_class == Request::PathClass::LocalAccess) {
      return false;
    }
    if (is_vertical_copy_path(req)) {
      return false;
    }
    if (req.path_class == Request::PathClass::CrossTierAccess) {
      return true;
    }
    const int explicit_src = explicit_source_tier(req);
    const int explicit_dst = explicit_destination_tier(req);
    if (explicit_src >= 0 && explicit_dst >= 0) {
      return explicit_src != explicit_dst;
    }
    return request_destination_tier(req) > 0;
  }

  int request_source_tier(const Request& req) const {
    const int explicit_src = explicit_source_tier(req);
    if (explicit_src >= 0) {
      return explicit_src;
    }
    const int dst_tier = request_destination_tier(req);
    if (req.path_class == Request::PathClass::LocalAccess) {
      return dst_tier;
    }
    if (req.path_class == Request::PathClass::CrossTierAccess ||
        req.path_class == Request::PathClass::VerticalCopy) {
      return 0;
    }
    return dst_tier > 0 ? 0 : dst_tier;
  }

  int request_access_tier(const Request& req) const {
    const int explicit_dst = explicit_destination_tier(req);
    if (explicit_dst >= 0) {
      return explicit_dst;
    }
    return clamp_tier_index(tier_id(req.addr_vec));
  }

  uint32_t request_hop_count(const Request& req) const {
    const int src_tier = clamp_tier_index(request_source_tier(req));
    const int dst_tier = clamp_tier_index(request_destination_tier(req));
    return static_cast<uint32_t>(
        src_tier > dst_tier ? (src_tier - dst_tier) : (dst_tier - src_tier));
  }

  bool per_link_vertical_topology_enabled() const {
    return m_vertical_link_ports_per_hop > 0 && m_num_tiers > 1;
  }

  std::vector<size_t> request_vertical_link_indices(const Request& req) const {
    std::vector<size_t> links;
    if (!requires_vertical_transfer(req) || !per_link_vertical_topology_enabled()) {
      return links;
    }
    const int src_tier = clamp_tier_index(request_source_tier(req));
    const int dst_tier = clamp_tier_index(request_destination_tier(req));
    if (src_tier == dst_tier) {
      return links;
    }
    const int first = std::min(src_tier, dst_tier);
    const int last = std::max(src_tier, dst_tier);
    links.reserve(static_cast<size_t>(last - first));
    for (int link = first; link < last; link++) {
      if (link >= 0 &&
          static_cast<size_t>(link) < m_active_vertical_link_releases.size()) {
        links.push_back(static_cast<size_t>(link));
      }
    }
    return links;
  }

  size_t active_vertical_transfers_total() const {
    if (!per_link_vertical_topology_enabled()) {
      return m_active_vertical_transfer_releases.size();
    }
    size_t total = 0;
    for (const auto& link_releases : m_active_vertical_link_releases) {
      total += link_releases.size();
    }
    return total;
  }

  bool has_any_active_vertical_transfer() const {
    if (!per_link_vertical_topology_enabled()) {
      return !m_active_vertical_transfer_releases.empty();
    }
    for (const auto& link_releases : m_active_vertical_link_releases) {
      if (!link_releases.empty()) {
        return true;
      }
    }
    return false;
  }

  void note_request_path_stats(const Request& req) {
    const int src_tier = clamp_tier_index(request_source_tier(req));
    const int dst_tier = clamp_tier_index(request_destination_tier(req));
    const uint32_t hop_count = request_hop_count(req);
    const bool explicit_src = req.source_tier_hint >= 0;
    const bool explicit_dst =
        req.destination_tier_hint >= 0 || req.tier_hint >= 0;

    if (explicit_src) {
      s_explicit_source_tier_hint_reqs++;
    }
    if (explicit_dst) {
      s_explicit_destination_tier_hint_reqs++;
    } else {
      s_fallback_destination_tier_reqs++;
    }

    if (is_vertical_copy_path(req)) {
      if (req.type_id == Request::Type::Read) {
        s_vertical_copy_read_reqs++;
      } else if (req.type_id == Request::Type::Write) {
        s_vertical_copy_write_reqs++;
      }
      if (!explicit_src) {
        s_fallback_source_tier_reqs++;
      }
      s_vertical_copy_hops_total += static_cast<size_t>(hop_count);
      if (static_cast<size_t>(hop_count) < s_vertical_copy_reqs_by_hops.size()) {
        s_vertical_copy_reqs_by_hops[hop_count]++;
      }
      s_vertical_copy_reqs_by_src_dst[src_tier][dst_tier]++;
      return;
    }

    if (is_cross_tier_path(req)) {
      if (req.type_id == Request::Type::Read) {
        s_cross_tier_read_reqs++;
      } else if (req.type_id == Request::Type::Write) {
        s_cross_tier_write_reqs++;
      }
      if (!explicit_src) {
        s_fallback_source_tier_reqs++;
      }
      s_cross_tier_hops_total += static_cast<size_t>(hop_count);
      if (static_cast<size_t>(hop_count) < s_cross_tier_reqs_by_hops.size()) {
        s_cross_tier_reqs_by_hops[hop_count]++;
      }
      s_cross_tier_reqs_by_src_dst[src_tier][dst_tier]++;
      return;
    }

    if (req.type_id == Request::Type::Read) {
      s_tier_local_read_reqs++;
    } else if (req.type_id == Request::Type::Write) {
      s_tier_local_write_reqs++;
    }
  }

  void note_accepted_request_stats(const Request& req) {
    switch (req.type_id) {
      case Request::Type::Read: {
        s_num_read_reqs++;
        switch (get_external_traffic_class(req)) {
          case ExternalTrafficClass::kForeground:
            s_num_foreground_read_reqs++;
            break;
          case ExternalTrafficClass::kBackground:
            s_num_background_read_reqs++;
            break;
          case ExternalTrafficClass::kShadow:
            s_num_shadow_read_reqs++;
            break;
        }
        break;
      }
      case Request::Type::Write: {
        s_num_write_reqs++;
        switch (get_external_traffic_class(req)) {
          case ExternalTrafficClass::kForeground:
            s_num_foreground_write_reqs++;
            break;
          case ExternalTrafficClass::kBackground:
            s_num_background_write_reqs++;
            break;
          case ExternalTrafficClass::kShadow:
            s_num_shadow_write_reqs++;
            break;
        }
        break;
      }
      default: {
        s_num_other_reqs++;
        break;
      }
    }

    note_request_path_stats(req);
  }

  bool requires_vertical_transfer(const Request& req) const {
    if (!m_dram->m_command_meta(req.command).is_accessing) {
      return false;
    }
    return is_vertical_copy_path(req) || is_cross_tier_path(req);
  }

  bool violates_access_bank_parallelism(
      const Request& req,
      const std::unordered_set<std::string>& used_access_banks) const {
    if (!m_dram->m_command_meta(req.command).is_accessing) {
      return false;
    }
    return used_access_banks.find(bank_key(req.addr_vec)) !=
           used_access_banks.end();
  }

  bool violates_tier_access_parallelism(
      const Request& req,
      const std::vector<uint32_t>& tier_access_cmds_issued_this_cycle,
      uint32_t tier_access_budget) const {
    if (!m_dram->m_command_meta(req.command).is_accessing) {
      return false;
    }
    const int tier = request_access_tier(req);
    if (tier < 0 ||
        static_cast<size_t>(tier) >= tier_access_cmds_issued_this_cycle.size()) {
      return false;
    }
    return tier_access_cmds_issued_this_cycle[tier] >= tier_access_budget;
  }

  bool violates_vertical_transfer_budget(
      const Request& req) const {
    if (!requires_vertical_transfer(req)) {
      return false;
    }
    if (per_link_vertical_topology_enabled()) {
      for (const size_t link_idx : request_vertical_link_indices(req)) {
        if (link_idx >= m_active_vertical_link_releases.size()) {
          continue;
        }
        if (m_active_vertical_link_releases[link_idx].size() >=
            static_cast<size_t>(m_vertical_link_ports_per_hop)) {
          return true;
        }
      }
      return false;
    }
    if (m_vertical_transfer_ports == 0) {
      return false;
    }
    return m_active_vertical_transfer_releases.size() >=
           static_cast<size_t>(m_vertical_transfer_ports);
  }

  bool candidate_rhs_better(ReqBuffer::iterator lhs, ReqBuffer::iterator rhs,
                            bool prioritize_traffic_class,
                            bool count_foreground_qos_win) {
    if (prioritize_traffic_class && prefer_request_qos(*rhs, *lhs) &&
        !prefer_request_qos(*lhs, *rhs)) {
      if (count_foreground_qos_win &&
          get_external_traffic_class(*rhs) ==
              ExternalTrafficClass::kForeground &&
          get_external_traffic_class(*lhs) !=
              ExternalTrafficClass::kForeground) {
        s_foreground_qos_wins++;
      }
      return true;
    }
    ReqBuffer::iterator better = m_scheduler->compare(lhs, rhs);
    return &(*better) == &(*rhs);
  }

  void collect_best_buffer_candidates(
      ReqBuffer& buffer,
      const std::unordered_set<std::string>& used_access_banks,
      bool allow_access, bool prioritize_traffic_class,
      uint32_t background_cmds_issued_this_cycle,
      uint32_t shadow_cmds_issued_this_cycle,
      const std::vector<uint32_t>& tier_access_cmds_issued_this_cycle,
      uint32_t tier_access_budget, ReqBuffer::iterator& ready_it,
      ReqBuffer::iterator* blocked_it = nullptr,
      ReadyBlockReason* blocked_reason = nullptr) {
    if (buffer.size() == 0) {
      ready_it = buffer.end();
      if (blocked_it != nullptr) {
        *blocked_it = buffer.end();
      }
      if (blocked_reason != nullptr) {
        *blocked_reason = ReadyBlockReason::kNone;
      }
      return;
    }

    if (blocked_it != nullptr) {
      *blocked_it = buffer.end();
    }
    if (blocked_reason != nullptr) {
      *blocked_reason = ReadyBlockReason::kNone;
    }

    ready_it = buffer.end();
    for (auto it = buffer.begin(); it != buffer.end(); ++it) {
      const SchedulingState scheduling_state =
          resolve_scheduling_state(it->final_command, it->addr_vec);
      if (controller_scoreboard_enabled() &&
          scheduling_state.prereq_scoreboard_miss) {
        s_controller_prereq_scoreboard_misses++;
      }
      it->command = scheduling_state.next_command;
      note_prereq_overlay_mismatch(it->final_command, it->addr_vec,
                                   it->command);

      if (it->command < 0) {
        continue;
      }
      if (m_dram->m_command_meta(it->command).is_accessing && !allow_access) {
        continue;
      }
      if (violates_access_bank_parallelism(*it, used_access_banks)) {
        continue;
      }
      if (violates_tier_access_parallelism(
              *it, tier_access_cmds_issued_this_cycle, tier_access_budget)) {
        continue;
      }
      if (violates_vertical_transfer_budget(*it)) {
        continue;
      }
      if (is_qos_budget_blocked(*it, background_cmds_issued_this_cycle,
                                shadow_cmds_issued_this_cycle)) {
        note_qos_budget_blocked(*it);
        continue;
      }

      if (controller_scoreboard_enabled()) {
        note_ready_overlay_mismatch(it->command, it->addr_vec,
                                    scheduling_state.next_command_ready);
      }

      if (!scheduling_state.next_command_ready) {
        if (blocked_it != nullptr && blocked_reason != nullptr &&
            scheduling_state.ready_block_reason != ReadyBlockReason::kNone) {
          if (*blocked_it == buffer.end() ||
              candidate_rhs_better(*blocked_it, it, prioritize_traffic_class,
                                   false)) {
            *blocked_it = it;
            *blocked_reason = scheduling_state.ready_block_reason;
          }
        }
        continue;
      }

      if (ready_it == buffer.end() ||
          candidate_rhs_better(ready_it, it, prioritize_traffic_class, true)) {
        ready_it = it;
      }
    }
  }

  bool select_best_ready_from_buffer(
      ReqBuffer& buffer,
      const std::unordered_set<std::string>& used_access_banks,
      bool allow_access, bool prioritize_traffic_class,
      uint32_t background_cmds_issued_this_cycle,
      uint32_t shadow_cmds_issued_this_cycle,
      const std::vector<uint32_t>& tier_access_cmds_issued_this_cycle,
      uint32_t tier_access_budget, ReqBuffer::iterator& req_it,
      ControllerReadyBlockReason* blocked_reason_out = nullptr) {
    if (blocked_reason_out != nullptr) {
      *blocked_reason_out = ControllerReadyBlockReason::kNone;
    }
    ReqBuffer::iterator blocked_it = buffer.end();
    ReadyBlockReason blocked_reason = ReadyBlockReason::kNone;
    collect_best_buffer_candidates(buffer, used_access_banks, allow_access,
                                   prioritize_traffic_class,
                                   background_cmds_issued_this_cycle,
                                   shadow_cmds_issued_this_cycle,
                                   tier_access_cmds_issued_this_cycle,
                                   tier_access_budget, req_it,
                                   blocked_reason_out != nullptr
                                       ? &blocked_it
                                       : nullptr,
                                   blocked_reason_out != nullptr
                                       ? &blocked_reason
                                       : nullptr);
    if (req_it != buffer.end()) {
      return true;
    }

    if (blocked_reason_out != nullptr && blocked_it != buffer.end()) {
      *blocked_reason_out =
          controller_ready_block_reason_from_scoreboard(blocked_reason);
    }
    return false;
  }

  bool schedule_request_filtered(
      ReqBuffer::iterator& req_it, ReqBuffer*& req_buffer,
      const std::unordered_set<std::string>& used_access_banks,
      bool allow_access, uint32_t background_cmds_issued_this_cycle,
      uint32_t shadow_cmds_issued_this_cycle,
      const std::vector<uint32_t>& tier_access_cmds_issued_this_cycle,
      uint32_t tier_access_budget) {
    bool request_found = false;
    ControllerReadyBlockReason blocked_reason =
        ControllerReadyBlockReason::kNone;
    ControllerReadyBlockReason current_blocked_reason =
        ControllerReadyBlockReason::kNone;

    if (select_best_ready_from_buffer(
            m_active_buffer, used_access_banks, allow_access, m_qos_enable,
            background_cmds_issued_this_cycle, shadow_cmds_issued_this_cycle,
            tier_access_cmds_issued_this_cycle, tier_access_budget, req_it,
            &current_blocked_reason)) {
      request_found = true;
      req_buffer = &m_active_buffer;
    } else {
      blocked_reason = choose_ready_block_reason(blocked_reason,
                                                 current_blocked_reason);
    }

    if (!request_found) {
      if (m_priority_buffer.size() != 0) {
        current_blocked_reason = ControllerReadyBlockReason::kNone;
        request_found = select_best_ready_from_buffer(
            m_priority_buffer, used_access_banks, allow_access, false,
            background_cmds_issued_this_cycle, shadow_cmds_issued_this_cycle,
            tier_access_cmds_issued_this_cycle, tier_access_budget, req_it,
            &current_blocked_reason);
        if (request_found) {
          req_buffer = &m_priority_buffer;
        }
        blocked_reason = choose_ready_block_reason(blocked_reason,
                                                   current_blocked_reason);
        if (!request_found && m_priority_buffer.size() != 0) {
          m_last_ready_block_reason = blocked_reason;
          return false;
        }
      }

      if (!request_found) {
        set_write_mode();
        ReqBuffer* primary_buffer =
            m_is_write_mode ? &m_write_buffer : &m_read_buffer;
        ReqBuffer* secondary_buffer =
            m_is_write_mode ? &m_read_buffer : &m_write_buffer;
        ReqBuffer::iterator primary_it = primary_buffer->end();
        ReqBuffer::iterator secondary_it = secondary_buffer->end();
        ControllerReadyBlockReason primary_blocked_reason =
            ControllerReadyBlockReason::kNone;
        ControllerReadyBlockReason secondary_blocked_reason =
            ControllerReadyBlockReason::kNone;
        const bool primary_ready = select_best_ready_from_buffer(
            *primary_buffer, used_access_banks, allow_access, true,
            background_cmds_issued_this_cycle, shadow_cmds_issued_this_cycle,
            tier_access_cmds_issued_this_cycle, tier_access_budget, primary_it,
            &primary_blocked_reason);
        bool secondary_ready = false;
        if (!primary_ready ||
            (m_is_write_mode &&
             m_allow_foreground_read_interrupt_write_mode)) {
          secondary_ready = select_best_ready_from_buffer(
              *secondary_buffer, used_access_banks, allow_access, true,
              background_cmds_issued_this_cycle, shadow_cmds_issued_this_cycle,
              tier_access_cmds_issued_this_cycle, tier_access_budget,
              secondary_it, &secondary_blocked_reason);
        }

        if (m_is_write_mode && m_allow_foreground_read_interrupt_write_mode &&
            secondary_ready) {
          const bool secondary_should_win =
              (!primary_ready) ||
              (prefer_request_qos(*secondary_it, *primary_it) &&
               !prefer_request_qos(*primary_it, *secondary_it));
          if (secondary_should_win) {
            if (get_external_traffic_class(*secondary_it) ==
                    ExternalTrafficClass::kForeground &&
                (!primary_ready ||
                 get_external_traffic_class(*primary_it) !=
                     ExternalTrafficClass::kForeground)) {
              s_write_mode_fg_interrupt_count++;
            }
            req_it = secondary_it;
            req_buffer = secondary_buffer;
            request_found = true;
          }
        }

        if (!request_found && primary_ready) {
          req_it = primary_it;
          req_buffer = primary_buffer;
          request_found = true;
        }
        if (!request_found && secondary_ready) {
          req_it = secondary_it;
          req_buffer = secondary_buffer;
          request_found = true;
        }
        if (!request_found) {
          blocked_reason = choose_ready_block_reason(blocked_reason,
                                                     primary_blocked_reason);
          blocked_reason = choose_ready_block_reason(blocked_reason,
                                                     secondary_blocked_reason);
        }
      }
    }

    if (!request_found) {
      m_last_ready_block_reason = blocked_reason;
    }

    if (request_found) {
      if (m_dram->m_command_meta(req_it->command).is_closing) {
        auto& rowgroup = req_it->addr_vec;
        for (auto _it = m_active_buffer.begin(); _it != m_active_buffer.end();
             _it++) {
          auto& _it_rowgroup = _it->addr_vec;
          bool is_matching = true;
          for (int i = 0; i < m_bank_addr_idx + 1; i++) {
            if (_it_rowgroup[i] != rowgroup[i] && _it_rowgroup[i] != -1 &&
                rowgroup[i] != -1) {
              is_matching = false;
              break;
            }
          }
          if (is_matching) {
            request_found = false;
            break;
          }
        }
      }
    }

    return request_found;
  }

  void retire_completed_vertical_transfers() {
    while (!m_active_vertical_transfer_releases.empty()) {
      auto it = m_active_vertical_transfer_releases.begin();
      if (*it > m_clk) {
        break;
      }
      m_active_vertical_transfer_releases.erase(it);
    }
    for (auto& link_releases : m_active_vertical_link_releases) {
      while (!link_releases.empty()) {
        auto it = link_releases.begin();
        if (*it > m_clk) {
          break;
        }
        link_releases.erase(it);
      }
    }
  }

  uint32_t request_transfer_units(const Request& req) const {
    const uint32_t effective_size =
        req.request_size_bytes > 0 ? req.request_size_bytes : m_transfer_unit_bytes;
    const uint32_t unit_bytes = std::max<uint32_t>(1, m_transfer_unit_bytes);
    return std::max<uint32_t>(
        1, (effective_size + unit_bytes - 1) / unit_bytes);
  }

  Clk_t local_data_completion_extra_cycles(const Request& req) const {
    if (!m_size_aware_timing) {
      return 0;
    }
    const uint32_t transfer_units = request_transfer_units(req);
    if (transfer_units <= 1) {
      return 0;
    }
    return static_cast<Clk_t>(transfer_units - 1) *
           static_cast<Clk_t>(m_local_burst_transfer_cycles);
  }

  Clk_t vertical_transfer_data_cycles(const Request& req) const {
    if (!requires_vertical_transfer(req)) {
      return 0;
    }
    if (!m_size_aware_timing) {
      return 0;
    }
    const uint32_t cycles_per_burst =
        is_vertical_copy_path(req) ? m_vertical_copy_cycles_per_burst
                                   : m_vertical_transfer_cycles_per_burst;
    return static_cast<Clk_t>(request_transfer_units(req)) *
           static_cast<Clk_t>(cycles_per_burst);
  }

  Clk_t vertical_transfer_hop_latency_cycles(const Request& req) const {
    if (!requires_vertical_transfer(req)) {
      return 0;
    }
    const uint32_t per_hop_cycles =
        is_vertical_copy_path(req) ? m_vertical_copy_hop_latency_cycles
                                   : m_cross_tier_hop_latency_cycles;
    if (per_hop_cycles == 0) {
      return 0;
    }
    return static_cast<Clk_t>(request_hop_count(req)) *
           static_cast<Clk_t>(per_hop_cycles);
  }

  Clk_t vertical_transfer_extra_cycles(const Request& req) const {
    return vertical_transfer_data_cycles(req) +
           vertical_transfer_hop_latency_cycles(req);
  }

  Clk_t source_endpoint_stage_cycles(const Request& req) const {
    if (!requires_vertical_transfer(req)) {
      return 0;
    }
    if (!m_size_aware_timing) {
      return 0;
    }
    const uint32_t cycles_per_burst =
        is_vertical_copy_path(req) ? m_vertical_copy_source_endpoint_cycles_per_burst
                                   : m_cross_tier_source_endpoint_cycles_per_burst;
    return static_cast<Clk_t>(request_transfer_units(req)) *
           static_cast<Clk_t>(cycles_per_burst);
  }

  Clk_t destination_endpoint_stage_cycles(const Request& req) const {
    if (!requires_vertical_transfer(req)) {
      return 0;
    }
    if (!m_size_aware_timing) {
      return 0;
    }
    const uint32_t cycles_per_burst =
        is_vertical_copy_path(req)
            ? m_vertical_copy_destination_endpoint_cycles_per_burst
            : m_cross_tier_destination_endpoint_cycles_per_burst;
    return static_cast<Clk_t>(request_transfer_units(req)) *
           static_cast<Clk_t>(cycles_per_burst);
  }

  Clk_t remote_endpoint_extra_cycles(const Request& req) const {
    return source_endpoint_stage_cycles(req) +
           destination_endpoint_stage_cycles(req);
  }

  Clk_t vertical_link_occupancy_cycles(const Request& req) const {
    if (!requires_vertical_transfer(req)) {
      return 0;
    }
    if (!m_size_aware_timing) {
      return 1;
    }
    const uint32_t cycles_per_burst =
        is_vertical_copy_path(req) ? m_vertical_copy_cycles_per_burst
                                   : m_vertical_link_cycles_per_burst;
    Clk_t cycles = static_cast<Clk_t>(request_transfer_units(req)) *
                   static_cast<Clk_t>(cycles_per_burst);
    if (cycles < 1) {
      cycles = 1;
    }
    return cycles;
  }

  Clk_t vertical_transfer_occupancy_cycles(const Request& req) const {
    if (!requires_vertical_transfer(req)) {
      return 0;
    }
    if (!m_size_aware_timing) {
      return 1;
    }
    Clk_t cycles = vertical_transfer_data_cycles(req);
    if (cycles < 1) {
      cycles = 1;
    }
    return cycles;
  }

  void reserve_vertical_transfer(const Request& req) {
    if (!requires_vertical_transfer(req)) {
      return;
    }
    if (per_link_vertical_topology_enabled()) {
      const Clk_t occupancy_cycles = vertical_link_occupancy_cycles(req);
      for (const size_t link_idx : request_vertical_link_indices(req)) {
        if (link_idx < m_active_vertical_link_releases.size()) {
          m_active_vertical_link_releases[link_idx].insert(m_clk + occupancy_cycles);
        }
      }
      return;
    }
    if (m_vertical_transfer_ports == 0) {
      return;
    }
    const Clk_t occupancy_cycles = vertical_transfer_occupancy_cycles(req);
    m_active_vertical_transfer_releases.insert(m_clk + occupancy_cycles);
  }

  Clk_t completion_depart_cycle(const Request& req) const {
    if (req.type_id == Request::Type::Read) {
      Clk_t base = m_dram->m_read_latency;
      if (base < 1) {
        base = 1;
      }
      return m_clk + base + local_data_completion_extra_cycles(req) +
             vertical_transfer_extra_cycles(req) +
             remote_endpoint_extra_cycles(req);
    }
    if (req.type_id == Request::Type::Write) {
      if (m_write_completion_mode == WriteCompletionMode::kPosted) {
        return m_clk + 1;
      }
      Clk_t base = m_dram->m_write_latency;
      if (base < 1) {
        base = 1;
      }
      return m_clk + base + local_data_completion_extra_cycles(req) +
             vertical_transfer_extra_cycles(req) +
             remote_endpoint_extra_cycles(req);
    }
    return m_clk + 1;
  }

  void finalize() override {
    s_avg_read_latency =
        s_num_read_reqs ? (float)s_read_latency / (float)s_num_read_reqs : 0.0f;

    if (m_clk > 0) {
      s_queue_len_avg = (float)s_queue_len / (float)m_clk;
      s_read_queue_len_avg = (float)s_read_queue_len / (float)m_clk;
      s_write_queue_len_avg = (float)s_write_queue_len / (float)m_clk;
      s_priority_queue_len_avg = (float)s_priority_queue_len / (float)m_clk;
      for (const auto traffic_class : kTrafficClasses) {
        const size_t idx = traffic_class_index(traffic_class);
        s_queue_len_avg_by_class[idx] =
            (float)s_queue_len_by_class[idx] / (float)m_clk;
      }
      s_avg_cmds_issued_per_cycle = (float)s_cmds_issued_total / (float)m_clk;
      s_avg_access_cmds_issued_per_cycle =
          (float)s_access_cmds_issued_total / (float)m_clk;
      s_avg_shared_access_cmds_issued_per_cycle =
          (float)s_shared_access_cmds_issued_total / (float)m_clk;
      s_avg_vertical_transfer_cmds_issued_per_cycle =
          (float)s_vertical_transfer_cmds_issued_total / (float)m_clk;
      for (size_t tier = 0; tier < m_num_tiers; tier++) {
        s_avg_tier_access_cmds_issued_per_cycle[tier] =
            (float)s_tier_access_cmds_issued_total[tier] / (float)m_clk;
      }
    }

    for (const auto traffic_class : kTrafficClasses) {
      const size_t idx = traffic_class_index(traffic_class);
      s_avg_queue_wait_cycles_by_class[idx] =
          s_num_completed_reqs_by_class[idx]
              ? (float)s_queue_wait_cycles_by_class[idx] /
                    (float)s_num_completed_reqs_by_class[idx]
              : 0.0f;
    }
  }
};

}  // namespace Ramulator
