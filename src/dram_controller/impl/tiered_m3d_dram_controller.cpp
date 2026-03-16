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
  uint32_t m_transfer_unit_bytes = 1;
  enum class WriteCompletionMode { kPosted, kData };
  WriteCompletionMode m_write_completion_mode = WriteCompletionMode::kPosted;
  std::multiset<Clk_t> m_active_vertical_transfer_releases;

  bool m_qos_enable = true;
  bool m_allow_foreground_read_interrupt_write_mode = true;
  uint32_t m_background_cmd_budget_per_cycle = 1;
  uint32_t m_shadow_cmd_budget_per_cycle = 1;
  Clk_t m_non_foreground_starvation_threshold_cycles = 64;
  bool m_shadow_scoreboard_enable = true;
  bool m_shadow_scoreboard_fail_fast = false;
  bool m_shadow_scoreboard_log_mismatch = false;
  BankStateScoreboard m_shadow_scoreboard;

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
  float s_avg_cmds_issued_per_cycle = 0;
  float s_avg_access_cmds_issued_per_cycle = 0;
  float s_avg_shared_access_cmds_issued_per_cycle = 0;
  float s_avg_vertical_transfer_cmds_issued_per_cycle = 0;
  std::vector<size_t> s_tier_access_cmds_issued_total;
  std::vector<size_t> s_max_tier_access_cmds_issued_per_cycle;
  std::vector<float> s_avg_tier_access_cmds_issued_per_cycle;
  std::vector<std::vector<size_t>> s_cross_tier_reqs_by_src_dst;
  std::vector<std::vector<size_t>> s_vertical_copy_reqs_by_src_dst;
  size_t s_shadow_scoreboard_diff_checks = 0;
  size_t s_shadow_scoreboard_rowhit_mismatches = 0;
  size_t s_shadow_scoreboard_rowopen_mismatches = 0;
  size_t s_shadow_scoreboard_prereq_checks = 0;
  size_t s_shadow_scoreboard_prereq_mismatches = 0;
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
            .desc("Enable P0 shadow bank-state scoreboard mirror.")
            .default_val(true);
    m_shadow_scoreboard_fail_fast =
        param<bool>("shadow_scoreboard_fail_fast")
            .desc("Abort on shadow scoreboard and DRAM mismatch (row-state/prereq/ready).")
            .default_val(false);
    m_shadow_scoreboard_log_mismatch =
        param<bool>("shadow_scoreboard_log_mismatch")
            .desc("Log row-state mismatches between shadow scoreboard and DRAM.")
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

    m_num_cores = frontend->get_num_cores();
    s_tier_access_cmds_issued_total.assign(m_num_tiers, 0);
    s_max_tier_access_cmds_issued_per_cycle.assign(m_num_tiers, 0);
    s_avg_tier_access_cmds_issued_per_cycle.assign(m_num_tiers, 0.0f);
    s_cross_tier_reqs_by_src_dst.assign(
        m_num_tiers, std::vector<size_t>(m_num_tiers, 0));
    s_vertical_copy_reqs_by_src_dst.assign(
        m_num_tiers, std::vector<size_t>(m_num_tiers, 0));

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

    if (req.type_id == Request::Type::Read) {
      auto compare_addr = [req](const Request& wreq) {
        return wreq.addr == req.addr;
      };
      if (std::find_if(m_write_buffer.begin(), m_write_buffer.end(),
                       compare_addr) != m_write_buffer.end()) {
        req.arrive = m_clk;
        req.depart = m_clk + 1;
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

    adjust_counted_queue_occupancy(req, +1);
    return true;
  };

  bool priority_send(Request& req) override {
    req.final_command = m_dram->m_request_translations(req.type_id);
    req.arrive = m_clk;
    const bool ok = m_priority_buffer.enqueue(req);
    if (ok) {
      adjust_counted_queue_occupancy(req, +1);
      if (m_shadow_scoreboard_enable && m_shadow_scoreboard.valid() &&
          m_dram->m_command_meta(req.final_command).is_refreshing) {
        m_shadow_scoreboard.on_refresh_scope_pending_from_command(
            m_dram, req.final_command, req.addr_vec, m_clk);
      }
    }
    return ok;
  }

  int get_prereq_command(int final_command,
                         const AddrVec_t& addr_vec) const override {
    if (m_shadow_scoreboard_enable && m_shadow_scoreboard.valid()) {
      const int cmd =
          m_shadow_scoreboard.get_prereq_command(m_dram, final_command, addr_vec);
      if (cmd >= 0) {
        return cmd;
      }
    }

    if (!m_dram) return -1;
    return m_dram->get_preq_command(final_command, addr_vec);
  }

  bool is_command_ready(int command, const AddrVec_t& addr_vec) const override {
    if (m_shadow_scoreboard_enable && m_shadow_scoreboard.valid()) {
      return m_shadow_scoreboard.is_command_ready(m_dram, command, addr_vec,
                                                 m_clk);
    }

    if (!m_dram) return false;
    return m_dram->check_ready(command, addr_vec);
  }

  bool probe_rowbuffer(int final_command, const AddrVec_t& addr_vec,
                       int& result) const override {
    result = 0;
    if (!(m_shadow_scoreboard_enable && m_shadow_scoreboard.valid())) {
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
    if (!(m_shadow_scoreboard_enable && m_shadow_scoreboard.valid())) {
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

  void tick() override {
    m_clk++;
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
      if (m_shadow_scoreboard_enable && m_shadow_scoreboard.valid()) {
        m_shadow_scoreboard.on_issue_command(m_dram, command, req_it->addr_vec,
                                             m_clk);
        const ShadowDiffResult diff = m_shadow_scoreboard.diff_against_dram(
            m_dram, req_it->final_command, req_it->addr_vec);
        if (diff.valid) {
          s_shadow_scoreboard_diff_checks++;
          if (!diff.row_hit_match) {
            s_shadow_scoreboard_rowhit_mismatches++;
          }
          if (!diff.row_open_match) {
            s_shadow_scoreboard_rowopen_mismatches++;
          }
          if ((!diff.row_hit_match || !diff.row_open_match) &&
              (m_shadow_scoreboard_log_mismatch ||
               m_shadow_scoreboard_fail_fast)) {
            std::ostringstream oss;
            oss << "TieredM3D shadow scoreboard mismatch ch=" << m_channel_id
                << " cmd=" << command << " final=" << req_it->final_command
                << " addr=" << req_it->addr
                << " bank=" << bank_key(req_it->addr_vec)
                << " row_hit(sb/dram)=" << diff.scoreboard_row_hit << "/"
                << diff.dram_row_hit
                << " row_open(sb/dram)=" << diff.scoreboard_row_open << "/"
                << diff.dram_row_open;
            const std::string msg = oss.str();
            if (m_shadow_scoreboard_log_mismatch) {
              spdlog::warn("{}", msg);
            }
            if (m_shadow_scoreboard_fail_fast) {
              throw std::runtime_error(msg);
            }
          }
        }
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
              vertical_transfer_occupancy_cycles(*req_it);
          if (is_vertical_copy_path(*req_it)) {
            s_vertical_copy_cmds_issued_total++;
            s_vertical_copy_transfer_cycles_total +=
                static_cast<size_t>(std::max<Clk_t>(0, occupancy_cycles));
          } else {
            s_cross_tier_cmds_issued_total++;
            s_cross_tier_transfer_cycles_total +=
                static_cast<size_t>(std::max<Clk_t>(0, occupancy_cycles));
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
    if (!m_active_vertical_transfer_releases.empty()) {
      s_vertical_port_busy_cycles++;
    }
  };

 private:
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

    bool row_hit = false;
    bool row_open = false;
    if (m_dram) {
      int rowbuffer_state = 0;
      if (probe_rowbuffer(req->final_command, req->addr_vec, rowbuffer_state)) {
        row_hit = (rowbuffer_state == 1);
        row_open = (rowbuffer_state == 2);
      } else {
        row_hit = m_dram->check_rowbuffer_hit(req->final_command, req->addr_vec);
        row_open = m_dram->check_node_open(req->final_command, req->addr_vec);
      }
    }

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
      if (m_shadow_scoreboard_enable && m_shadow_scoreboard.valid()) {
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

  void note_request_path_stats(const Request& req) {
    const int src_tier = clamp_tier_index(request_source_tier(req));
    const int dst_tier = clamp_tier_index(request_destination_tier(req));

    if (is_vertical_copy_path(req)) {
      if (req.type_id == Request::Type::Read) {
        s_vertical_copy_read_reqs++;
      } else if (req.type_id == Request::Type::Write) {
        s_vertical_copy_write_reqs++;
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
      s_cross_tier_reqs_by_src_dst[src_tier][dst_tier]++;
      return;
    }

    if (req.type_id == Request::Type::Read) {
      s_tier_local_read_reqs++;
    } else if (req.type_id == Request::Type::Write) {
      s_tier_local_write_reqs++;
    }
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
    if (m_vertical_transfer_ports == 0) {
      return false;
    }
    return m_active_vertical_transfer_releases.size() >=
           static_cast<size_t>(m_vertical_transfer_ports);
  }

  bool is_command_ready_for_issue(int command, const AddrVec_t& addr_vec) {
    if (!(m_shadow_scoreboard_enable && m_shadow_scoreboard.valid())) {
      return m_dram->check_ready(command, addr_vec);
    }

    const bool scoreboard_ready =
        m_shadow_scoreboard.is_command_ready(m_dram, command, addr_vec, m_clk);
    const bool oracle_ready = m_dram->check_ready(command, addr_vec);
    s_shadow_scoreboard_ready_checks++;
    if (scoreboard_ready != oracle_ready) {
      s_shadow_scoreboard_ready_mismatches++;
      if (scoreboard_ready && !oracle_ready) {
        s_shadow_scoreboard_ready_oracle_blocked_while_scoreboard_ready++;
      } else if (!scoreboard_ready && oracle_ready) {
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
            << " scoreboard=" << (scoreboard_ready ? 1 : 0)
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
    if (!scoreboard_ready) {
      s_shadow_scoreboard_ready_blocked_by_scoreboard++;
    }
    if (!oracle_ready) {
      s_shadow_scoreboard_ready_blocked_by_oracle++;
    }
    return scoreboard_ready;
  }

  ReqBuffer::iterator get_best_request_filtered(
      ReqBuffer& buffer,
      const std::unordered_set<std::string>& used_access_banks,
      bool allow_access, bool prioritize_traffic_class,
      uint32_t background_cmds_issued_this_cycle,
      uint32_t shadow_cmds_issued_this_cycle,
      const std::vector<uint32_t>& tier_access_cmds_issued_this_cycle,
      uint32_t tier_access_budget) {
    if (buffer.size() == 0) {
      return buffer.end();
    }

    for (auto& req : buffer) {
      const int scoreboard_cmd =
          get_prereq_command(req.final_command, req.addr_vec);
      req.command = scoreboard_cmd;

      if (m_shadow_scoreboard_enable && m_shadow_scoreboard.valid()) {
        s_shadow_scoreboard_prereq_checks++;
        const int oracle_cmd =
            m_dram->get_preq_command(req.final_command, req.addr_vec);
        if (oracle_cmd != scoreboard_cmd) {
          s_shadow_scoreboard_prereq_mismatches++;
          if (m_shadow_scoreboard_log_mismatch ||
              m_shadow_scoreboard_fail_fast) {
            std::ostringstream oss;
            oss << "TieredM3D channel " << m_channel_id
                << ": prereq mismatch final=" << req.final_command
                << " scoreboard=" << scoreboard_cmd << " oracle=" << oracle_cmd
                << " bank=" << bank_key(req.addr_vec);
            const std::string msg = oss.str();
            if (m_shadow_scoreboard_log_mismatch) {
              spdlog::warn("{}", msg);
            }
            if (m_shadow_scoreboard_fail_fast) {
              throw std::runtime_error(msg);
            }
          }
        }
      }
    }

    auto candidate = buffer.end();
    for (auto it = buffer.begin(); it != buffer.end(); ++it) {
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
      if (candidate == buffer.end()) {
        candidate = it;
        continue;
      }
      if (prioritize_traffic_class && prefer_request_qos(*it, *candidate) &&
          !prefer_request_qos(*candidate, *it)) {
        if (get_external_traffic_class(*it) == ExternalTrafficClass::kForeground &&
            get_external_traffic_class(*candidate) !=
                ExternalTrafficClass::kForeground) {
          s_foreground_qos_wins++;
        }
        candidate = it;
        continue;
      }
      candidate = m_scheduler->compare(candidate, it);
    }
    return candidate;
  }

  bool select_best_ready_from_buffer(
      ReqBuffer& buffer,
      const std::unordered_set<std::string>& used_access_banks,
      bool allow_access, bool prioritize_traffic_class,
      uint32_t background_cmds_issued_this_cycle,
      uint32_t shadow_cmds_issued_this_cycle,
      const std::vector<uint32_t>& tier_access_cmds_issued_this_cycle,
      uint32_t tier_access_budget, ReqBuffer::iterator& req_it) {
    req_it = get_best_request_filtered(buffer, used_access_banks, allow_access,
                                       prioritize_traffic_class,
                                       background_cmds_issued_this_cycle,
                                       shadow_cmds_issued_this_cycle,
                                       tier_access_cmds_issued_this_cycle,
                                       tier_access_budget);
    if (req_it == buffer.end()) {
      return false;
    }
    return is_command_ready_for_issue(req_it->command, req_it->addr_vec);
  }

  bool schedule_request_filtered(
      ReqBuffer::iterator& req_it, ReqBuffer*& req_buffer,
      const std::unordered_set<std::string>& used_access_banks,
      bool allow_access, uint32_t background_cmds_issued_this_cycle,
      uint32_t shadow_cmds_issued_this_cycle,
      const std::vector<uint32_t>& tier_access_cmds_issued_this_cycle,
      uint32_t tier_access_budget) {
    bool request_found = false;

    if (select_best_ready_from_buffer(
            m_active_buffer, used_access_banks, allow_access, m_qos_enable,
            background_cmds_issued_this_cycle, shadow_cmds_issued_this_cycle,
            tier_access_cmds_issued_this_cycle, tier_access_budget, req_it)) {
      request_found = true;
      req_buffer = &m_active_buffer;
    }

    if (!request_found) {
      if (m_priority_buffer.size() != 0) {
        req_buffer = &m_priority_buffer;
        req_it = m_priority_buffer.begin();
        const int scoreboard_cmd =
            get_prereq_command(req_it->final_command, req_it->addr_vec);
        req_it->command = scoreboard_cmd;
        if (m_shadow_scoreboard_enable && m_shadow_scoreboard.valid()) {
          s_shadow_scoreboard_prereq_checks++;
          const int oracle_cmd =
              m_dram->get_preq_command(req_it->final_command, req_it->addr_vec);
          if (oracle_cmd != scoreboard_cmd) {
            s_shadow_scoreboard_prereq_mismatches++;
            if (m_shadow_scoreboard_log_mismatch ||
                m_shadow_scoreboard_fail_fast) {
              std::ostringstream oss;
              oss << "TieredM3D channel " << m_channel_id
                  << ": prereq mismatch final=" << req_it->final_command
                  << " scoreboard=" << scoreboard_cmd
                  << " oracle=" << oracle_cmd
                  << " bank=" << bank_key(req_it->addr_vec);
              const std::string msg = oss.str();
              if (m_shadow_scoreboard_log_mismatch) {
                spdlog::warn("{}", msg);
              }
              if (m_shadow_scoreboard_fail_fast) {
                throw std::runtime_error(msg);
              }
            }
          }
        }

        if (m_dram->m_command_meta(req_it->command).is_accessing && !allow_access) {
          request_found = false;
        } else if (violates_access_bank_parallelism(*req_it, used_access_banks)) {
          request_found = false;
        } else if (violates_tier_access_parallelism(
                       *req_it, tier_access_cmds_issued_this_cycle,
                       tier_access_budget)) {
          request_found = false;
        } else if (violates_vertical_transfer_budget(*req_it)) {
          request_found = false;
        } else {
          request_found =
              is_command_ready_for_issue(req_it->command, req_it->addr_vec);
        }
        if (!request_found && m_priority_buffer.size() != 0) {
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
        const bool primary_ready = select_best_ready_from_buffer(
            *primary_buffer, used_access_banks, allow_access, true,
            background_cmds_issued_this_cycle, shadow_cmds_issued_this_cycle,
            tier_access_cmds_issued_this_cycle, tier_access_budget, primary_it);
        bool secondary_ready = false;
        if (!primary_ready ||
            (m_is_write_mode &&
             m_allow_foreground_read_interrupt_write_mode)) {
          secondary_ready = select_best_ready_from_buffer(
              *secondary_buffer, used_access_banks, allow_access, true,
              background_cmds_issued_this_cycle, shadow_cmds_issued_this_cycle,
              tier_access_cmds_issued_this_cycle, tier_access_budget,
              secondary_it);
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
      }
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

  Clk_t vertical_transfer_extra_cycles(const Request& req) const {
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

  Clk_t vertical_transfer_occupancy_cycles(const Request& req) const {
    if (!requires_vertical_transfer(req)) {
      return 0;
    }
    if (!m_size_aware_timing) {
      return 1;
    }
    Clk_t cycles = vertical_transfer_extra_cycles(req);
    if (cycles < 1) {
      cycles = 1;
    }
    return cycles;
  }

  void reserve_vertical_transfer(const Request& req) {
    if (!requires_vertical_transfer(req) || m_vertical_transfer_ports == 0) {
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
             vertical_transfer_extra_cycles(req);
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
             vertical_transfer_extra_cycles(req);
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
