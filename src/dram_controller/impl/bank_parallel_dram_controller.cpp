#include "dram_controller/controller.h"
#include "dram_controller/bank_machine.h"
#include "dram_controller/bank_state_scoreboard.h"
#include "memory_system/memory_system.h"

#include <algorithm>
#include <array>
#include <deque>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace Ramulator {

namespace {

constexpr int kShmemTrafficClassScratchpadIdx = 4;
constexpr int kShmemPhaseIdScratchpadIdx = 9;
constexpr size_t kNumExternalTrafficClasses = 3;
constexpr size_t kNumShmemPhases = 3;
constexpr size_t kRowBufferEventRingCapacity = 4096;

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

size_t shmem_phase_index(const Request& req) {
  const int raw = req.scratchpad[kShmemPhaseIdScratchpadIdx];
  if (raw <= 0) return 0;
  if (raw == 1) return 1;
  return kNumShmemPhases - 1;
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

class BankParallelDRAMController final : public IDRAMController,
                                         public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(
      IDRAMController, BankParallelDRAMController, "BankParallel",
      "A DRAM controller that can issue up to N commands per cycle and limits "
      "concurrent RD/WR-like accesses to distinct banks within a channel.")

 private:
  std::deque<Request>
      pending;  // A queue for read/write requests waiting for completion.

  ReqBuffer m_active_buffer;    // Buffer for requests being served
  ReqBuffer m_priority_buffer;  // Buffer for high-priority requests
  ReqBuffer m_read_buffer;      // Read request buffer
  ReqBuffer m_write_buffer;     // Write request buffer

  int m_bank_addr_idx = -1;

  float m_wr_low_watermark;
  float m_wr_high_watermark;
  bool m_is_write_mode = false;

  // Legacy knob: previously used for both "issue width" and "access ports".
  uint32_t m_bank_parallel_ports = 1;
  // New knobs (v2):
  // - m_cmd_issue_width: total #commands issued per controller cycle.
  // - m_access_ports: max #accessing commands (RD/WR/RDA/WRA) per cycle;
  //   additionally constrained to distinct banks.
  uint32_t m_cmd_issue_width = 1;
  uint32_t m_access_ports = 1;
  enum class WriteCompletionMode { kPosted, kData };
  WriteCompletionMode m_write_completion_mode = WriteCompletionMode::kPosted;

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
  struct RefreshLifecycleState {
    bool pending = false;
    bool active = false;
    RefreshScopeKind scope_kind = RefreshScopeKind::kNone;
    AddrVec_t scope_addr_vec {};
    Clk_t pending_since_cycle = -1;
    Clk_t active_since_cycle = -1;
    Clk_t active_until_cycle = -1;
    uint64_t epoch = 0;
  };
  RefreshLifecycleState m_refresh_state;
  Clk_t m_refresh_window_cycles = -1;
  ControllerReadyBlockReason m_last_ready_block_reason =
      ControllerReadyBlockReason::kNone;
  BankMachine m_bank_machine;
  uint32_t m_scoreboard_autoprecharge_cap = 0;
  int m_cmd_rd = -1;
  int m_cmd_wr = -1;
  int m_cmd_rda = -1;
  int m_cmd_wra = -1;
  std::deque<ControllerRowBufferEvent> m_rowbuffer_event_ring;

  size_t s_row_hits = 0;
  size_t s_row_misses = 0;
  size_t s_row_conflicts = 0;
  size_t s_read_row_hits = 0;
  size_t s_read_row_misses = 0;
  size_t s_read_row_conflicts = 0;
  size_t s_write_row_hits = 0;
  size_t s_write_row_misses = 0;
  size_t s_write_row_conflicts = 0;

  std::array<size_t, kNumShmemPhases> s_phase_row_hits = {0, 0, 0};
  std::array<size_t, kNumShmemPhases> s_phase_row_misses = {0, 0, 0};
  std::array<size_t, kNumShmemPhases> s_phase_row_conflicts = {0, 0, 0};

  TrafficClassCounterArray s_row_hits_by_class = {0, 0, 0};
  TrafficClassCounterArray s_row_misses_by_class = {0, 0, 0};
  TrafficClassCounterArray s_row_conflicts_by_class = {0, 0, 0};
  TrafficClassCounterArray s_read_row_hits_by_class = {0, 0, 0};
  TrafficClassCounterArray s_read_row_misses_by_class = {0, 0, 0};
  TrafficClassCounterArray s_read_row_conflicts_by_class = {0, 0, 0};
  TrafficClassCounterArray s_write_row_hits_by_class = {0, 0, 0};
  TrafficClassCounterArray s_write_row_misses_by_class = {0, 0, 0};
  TrafficClassCounterArray s_write_row_conflicts_by_class = {0, 0, 0};

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
  size_t s_cmds_issued_total = 0;
  size_t s_access_cmds_issued_total = 0;
  float s_avg_cmds_issued_per_cycle = 0;
  float s_avg_access_cmds_issued_per_cycle = 0;
  size_t s_shadow_scoreboard_diff_checks = 0;
  size_t s_shadow_scoreboard_rowhit_mismatches = 0;
  size_t s_shadow_scoreboard_rowopen_mismatches = 0;
  size_t s_shadow_scoreboard_ready_checks = 0;
  size_t s_shadow_scoreboard_ready_mismatches = 0;
  size_t s_shadow_scoreboard_ready_mismatch_open = 0;
  size_t s_shadow_scoreboard_ready_mismatch_access = 0;
  size_t s_shadow_scoreboard_ready_mismatch_close = 0;
  size_t s_shadow_scoreboard_ready_mismatch_other = 0;
  size_t s_shadow_scoreboard_ready_blocked_by_scoreboard = 0;
  size_t s_shadow_scoreboard_ready_blocked_by_oracle = 0;
  size_t s_shadow_scoreboard_ready_oracle_blocked_while_scoreboard_ready = 0;
  size_t s_shadow_scoreboard_ready_oracle_ready_while_scoreboard_blocked = 0;
  size_t s_shadow_scoreboard_prereq_checks = 0;
  size_t s_shadow_scoreboard_prereq_mismatches = 0;
  size_t s_controller_prereq_scoreboard_misses = 0;
  size_t s_controller_rowstate_scoreboard_misses = 0;
  size_t s_shadow_scoreboard_refresh_enter_events = 0;
  size_t s_controller_refresh_pending_events = 0;
  size_t s_controller_refresh_enter_events = 0;
  size_t s_controller_refresh_exit_events = 0;
  size_t s_controller_refresh_pending_cycles = 0;
  size_t s_controller_refresh_active_cycles = 0;
  size_t s_controller_refresh_epoch = 0;
  size_t s_controller_refresh_scope_kind_last = 0;
  size_t s_controller_refresh_window_cycles = 0;
  size_t s_shadow_scoreboard_refreshing_banks_peak = 0;
  size_t s_shadow_scoreboard_pending_banks_peak = 0;
  size_t s_scoreboard_forced_autoprecharge = 0;
  size_t s_bankmachine_transition_open = 0;
  size_t s_bankmachine_transition_access = 0;
  size_t s_bankmachine_transition_close = 0;
  size_t s_bankmachine_transition_refresh = 0;
  size_t s_bankmachine_transition_other = 0;

  using IssueCommandPlan = BankMachine::IssuePlan;

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

    m_cmd_issue_width =
        param<uint32_t>("cmd_issue_width_per_layer")
            .desc(
                "Max number of DRAM commands issued per controller cycle (total issue width).")
            .default_val(m_bank_parallel_ports);
    m_access_ports =
        param<uint32_t>("access_ports_per_layer")
            .desc(
                "Max number of accessing DRAM commands (RD/WR/RDA/WRA) issued per controller cycle; "
                "accessing commands are additionally constrained to distinct banks within the channel.")
            .default_val(m_bank_parallel_ports);

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
            .desc("Enable shared-aware foreground/background/shadow QoS in BankParallel.")
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
    m_scoreboard_autoprecharge_cap =
        param<uint32_t>("scoreboard_autoprecharge_cap")
            .desc(
                "When >0, force RDA/WRA in controller once a row reaches this many accesses "
                "according to shadow scoreboard (0 disables controller-side cap policy).")
            .default_val(0);

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
    m_bank_machine.setup(m_dram);
    if (m_shadow_scoreboard_enable) {
      m_shadow_scoreboard.init_from_dram_org(m_dram, m_channel_id);
    }
    m_bank_machine.attach_scoreboard(
        controller_scoreboard_enabled() ? &m_shadow_scoreboard : nullptr);
    m_refresh_window_cycles = detect_refresh_window_cycles();
    s_controller_refresh_window_cycles =
        m_refresh_window_cycles > 0
            ? static_cast<size_t>(m_refresh_window_cycles)
            : 0;
    if (m_scoreboard_autoprecharge_cap > 0) {
      try {
        m_cmd_rd = m_dram->m_commands("RD");
        m_cmd_wr = m_dram->m_commands("WR");
        m_cmd_rda = m_dram->m_commands("RDA");
        m_cmd_wra = m_dram->m_commands("WRA");
      } catch (const std::out_of_range&) {
        spdlog::warn(
            "BankParallel channel {}: disable scoreboard_autoprecharge_cap={} "
            "because DRAM does not expose RD/WR/RDA/WRA.",
            m_channel_id, m_scoreboard_autoprecharge_cap);
        m_scoreboard_autoprecharge_cap = 0;
      }
    }
    m_bank_machine.configure_scoreboard_autoprecharge(
        m_scoreboard_autoprecharge_cap, m_cmd_rd, m_cmd_wr, m_cmd_rda, m_cmd_wra);

    m_num_cores = frontend->get_num_cores();

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

    for (size_t phase = 0; phase < kNumShmemPhases; ++phase) {
      register_stat(s_phase_row_hits[phase])
          .name("phase{}_row_hits_{}", phase, m_channel_id);
      register_stat(s_phase_row_misses[phase])
          .name("phase{}_row_misses_{}", phase, m_channel_id);
      register_stat(s_phase_row_conflicts[phase])
          .name("phase{}_row_conflicts_{}", phase, m_channel_id);
    }

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
      register_stat(s_row_hits_by_class[idx])
          .name("{}_row_hits_{}", prefix, m_channel_id);
      register_stat(s_row_misses_by_class[idx])
          .name("{}_row_misses_{}", prefix, m_channel_id);
      register_stat(s_row_conflicts_by_class[idx])
          .name("{}_row_conflicts_{}", prefix, m_channel_id);
      register_stat(s_read_row_hits_by_class[idx])
          .name("{}_read_row_hits_{}", prefix, m_channel_id);
      register_stat(s_read_row_misses_by_class[idx])
          .name("{}_read_row_misses_{}", prefix, m_channel_id);
      register_stat(s_read_row_conflicts_by_class[idx])
          .name("{}_read_row_conflicts_{}", prefix, m_channel_id);
      register_stat(s_write_row_hits_by_class[idx])
          .name("{}_write_row_hits_{}", prefix, m_channel_id);
      register_stat(s_write_row_misses_by_class[idx])
          .name("{}_write_row_misses_{}", prefix, m_channel_id);
      register_stat(s_write_row_conflicts_by_class[idx])
          .name("{}_write_row_conflicts_{}", prefix, m_channel_id);
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
    register_stat(s_avg_cmds_issued_per_cycle)
        .name("avg_cmds_issued_per_cycle_{}", m_channel_id);
    register_stat(s_avg_access_cmds_issued_per_cycle)
        .name("avg_access_cmds_issued_per_cycle_{}", m_channel_id);
    register_stat(s_shadow_scoreboard_diff_checks)
        .name("shadow_scoreboard_diff_checks_{}", m_channel_id);
    register_stat(s_shadow_scoreboard_rowhit_mismatches)
        .name("shadow_scoreboard_rowhit_mismatches_{}", m_channel_id);
    register_stat(s_shadow_scoreboard_rowopen_mismatches)
        .name("shadow_scoreboard_rowopen_mismatches_{}", m_channel_id);
    register_stat(s_shadow_scoreboard_ready_checks)
        .name("shadow_scoreboard_ready_checks_{}", m_channel_id);
    register_stat(s_shadow_scoreboard_ready_mismatches)
        .name("shadow_scoreboard_ready_mismatches_{}", m_channel_id);
    register_stat(s_shadow_scoreboard_ready_mismatch_open)
        .name("shadow_scoreboard_ready_mismatch_open_{}", m_channel_id);
    register_stat(s_shadow_scoreboard_ready_mismatch_access)
        .name("shadow_scoreboard_ready_mismatch_access_{}", m_channel_id);
    register_stat(s_shadow_scoreboard_ready_mismatch_close)
        .name("shadow_scoreboard_ready_mismatch_close_{}", m_channel_id);
    register_stat(s_shadow_scoreboard_ready_mismatch_other)
        .name("shadow_scoreboard_ready_mismatch_other_{}", m_channel_id);
    register_stat(s_shadow_scoreboard_ready_blocked_by_scoreboard)
        .name("shadow_scoreboard_ready_blocked_by_scoreboard_{}",
              m_channel_id);
    register_stat(s_shadow_scoreboard_ready_blocked_by_oracle)
        .name("shadow_scoreboard_ready_blocked_by_oracle_{}", m_channel_id);
    register_stat(s_shadow_scoreboard_ready_oracle_blocked_while_scoreboard_ready)
        .name("shadow_scoreboard_ready_oracle_blocked_while_scoreboard_ready_{}",
              m_channel_id);
    register_stat(s_shadow_scoreboard_ready_oracle_ready_while_scoreboard_blocked)
        .name("shadow_scoreboard_ready_oracle_ready_while_scoreboard_blocked_{}",
              m_channel_id);
    register_stat(s_shadow_scoreboard_prereq_checks)
        .name("shadow_scoreboard_prereq_checks_{}", m_channel_id);
    register_stat(s_shadow_scoreboard_prereq_mismatches)
        .name("shadow_scoreboard_prereq_mismatches_{}", m_channel_id);
    register_stat(s_controller_prereq_scoreboard_misses)
        .name("controller_prereq_scoreboard_misses_{}", m_channel_id);
    register_stat(s_controller_rowstate_scoreboard_misses)
        .name("controller_rowstate_scoreboard_misses_{}", m_channel_id);
    register_stat(s_shadow_scoreboard_refresh_enter_events)
        .name("shadow_scoreboard_refresh_enter_events_{}", m_channel_id);
    register_stat(s_controller_refresh_pending_events)
        .name("controller_refresh_pending_events_{}", m_channel_id);
    register_stat(s_controller_refresh_enter_events)
        .name("controller_refresh_enter_events_{}", m_channel_id);
    register_stat(s_controller_refresh_exit_events)
        .name("controller_refresh_exit_events_{}", m_channel_id);
    register_stat(s_controller_refresh_pending_cycles)
        .name("controller_refresh_pending_cycles_{}", m_channel_id);
    register_stat(s_controller_refresh_active_cycles)
        .name("controller_refresh_active_cycles_{}", m_channel_id);
    register_stat(s_controller_refresh_epoch)
        .name("controller_refresh_epoch_{}", m_channel_id);
    register_stat(s_controller_refresh_scope_kind_last)
        .name("controller_refresh_scope_kind_last_{}", m_channel_id);
    register_stat(s_controller_refresh_window_cycles)
        .name("controller_refresh_window_cycles_{}", m_channel_id);
    register_stat(s_shadow_scoreboard_refreshing_banks_peak)
        .name("shadow_scoreboard_refreshing_banks_peak_{}", m_channel_id);
    register_stat(s_shadow_scoreboard_pending_banks_peak)
        .name("shadow_scoreboard_pending_banks_peak_{}", m_channel_id);
    register_stat(s_scoreboard_forced_autoprecharge)
        .name("scoreboard_forced_autoprecharge_{}", m_channel_id);
    register_stat(s_bankmachine_transition_open)
        .name("bankmachine_transition_open_{}", m_channel_id);
    register_stat(s_bankmachine_transition_access)
        .name("bankmachine_transition_access_{}", m_channel_id);
    register_stat(s_bankmachine_transition_close)
        .name("bankmachine_transition_close_{}", m_channel_id);
    register_stat(s_bankmachine_transition_refresh)
        .name("bankmachine_transition_refresh_{}", m_channel_id);
    register_stat(s_bankmachine_transition_other)
        .name("bankmachine_transition_other_{}", m_channel_id);
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
      if (m_dram->m_command_meta(req.final_command).is_refreshing) {
        note_refresh_scope_pending(req.final_command, req.addr_vec);
        if (controller_scoreboard_enabled()) {
          m_shadow_scoreboard.on_refresh_scope_pending_from_command(
              m_dram, req.final_command, req.addr_vec, m_clk);
        }
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

  bool is_command_ready(int command,
                        const AddrVec_t& addr_vec) const override {
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

    const TelemetryResult telemetry =
        m_shadow_scoreboard.query_telemetry(m_dram, final_command, addr_vec, m_clk);
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
    result.refresh_mode = m_refresh_window_cycles > 0
                              ? ControllerRefreshMode::kStaticNormal
                              : ControllerRefreshMode::kUnknown;
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
    result.access_budget = std::max<uint32_t>(1, m_access_ports);
    result.open_banks =
        static_cast<uint64_t>(m_shadow_scoreboard.count_open_banks());
    result.inflight_banks =
        static_cast<uint64_t>(m_shadow_scoreboard.count_inflight_banks());
    result.autoprecharge_armed_banks = static_cast<uint64_t>(
        m_shadow_scoreboard.count_autoprecharge_armed_banks());
    result.max_open_row_age_cycles =
        m_shadow_scoreboard.max_open_row_age_cycles(m_clk);
    result.refresh_pending = m_refresh_state.pending || refresh_state.pending;
    result.refresh_active = m_refresh_state.active || refresh_state.active;
    result.refresh_recovery = refresh_state.recovery;
    result.refresh_epoch = std::max<uint64_t>(
        m_refresh_state.epoch, refresh_state.epoch);
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
    result.refresh_scope = controller_refresh_scope_from_scoreboard(
        m_refresh_state.scope_kind != RefreshScopeKind::kNone
            ? m_refresh_state.scope_kind
            : refresh_state.owner_scope);
    result.refresh_mode = m_refresh_window_cycles > 0
                              ? ControllerRefreshMode::kStaticNormal
                              : ControllerRefreshMode::kUnknown;
    result.ready_blocked =
        m_last_ready_block_reason != ControllerReadyBlockReason::kNone;
    result.ready_block_reason = m_last_ready_block_reason;
    result.thermal_valid = false;
    result.temperature_bucket = ControllerTemperatureBucket::kUnknown;
    result.tiered_valid = false;
    result.num_tiers = 0;
    result.shared_access_budget = 0;
    result.tier_access_budget = 0;
    result.vertical_transfer_budget = 0;
    result.vertical_transfer_active = 0;
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

    result.foreground_row_hits = static_cast<uint64_t>(
        s_row_hits_by_class[traffic_class_index(
            ExternalTrafficClass::kForeground)]);
    result.foreground_row_misses = static_cast<uint64_t>(
        s_row_misses_by_class[traffic_class_index(
            ExternalTrafficClass::kForeground)]);
    result.foreground_row_conflicts = static_cast<uint64_t>(
        s_row_conflicts_by_class[traffic_class_index(
            ExternalTrafficClass::kForeground)]);

    result.background_row_hits = static_cast<uint64_t>(
        s_row_hits_by_class[traffic_class_index(
            ExternalTrafficClass::kBackground)]);
    result.background_row_misses = static_cast<uint64_t>(
        s_row_misses_by_class[traffic_class_index(
            ExternalTrafficClass::kBackground)]);
    result.background_row_conflicts = static_cast<uint64_t>(
        s_row_conflicts_by_class[traffic_class_index(
            ExternalTrafficClass::kBackground)]);

    result.shadow_row_hits = static_cast<uint64_t>(
        s_row_hits_by_class[traffic_class_index(ExternalTrafficClass::kShadow)]);
    result.shadow_row_misses = static_cast<uint64_t>(s_row_misses_by_class[
        traffic_class_index(ExternalTrafficClass::kShadow)]);
    result.shadow_row_conflicts = static_cast<uint64_t>(
        s_row_conflicts_by_class[traffic_class_index(
            ExternalTrafficClass::kShadow)]);

    result.local_accesses = 0;
    result.cross_tier_accesses = 0;
    result.vertical_copy_accesses = 0;
    return true;
  }

  bool consume_rowbuffer_event(uint64_t addr, const AddrVec_t& addr_vec,
                               int type_id, int source_id,
                               ControllerRowBufferEvent& result) override {
    result = ControllerRowBufferEvent {};
    for (auto it = m_rowbuffer_event_ring.begin();
         it != m_rowbuffer_event_ring.end(); ++it) {
      if (!it->valid || it->type_id != type_id || it->source_id != source_id) {
        continue;
      }
      if (it->addr != addr && it->addr_vec != addr_vec) {
        continue;
      }
      result = *it;
      m_rowbuffer_event_ring.erase(it);
      return true;
    }
    return false;
  }

  void tick() override {
    m_clk++;
    m_last_ready_block_reason = ControllerReadyBlockReason::kNone;

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
    tick_refresh_lifecycle_state();
    sample_shadow_refresh_state_peaks();

    std::unordered_set<std::string> used_access_banks;
    used_access_banks.reserve(std::max<uint32_t>(1, m_access_ports) * 2);

    const uint32_t issue_budget = std::max<uint32_t>(1, m_cmd_issue_width);
    const uint32_t access_budget = std::max<uint32_t>(1, m_access_ports);
    uint32_t cmds_issued_this_cycle = 0;
    uint32_t access_cmds_issued_this_cycle = 0;
    uint32_t background_cmds_issued_this_cycle = 0;
    uint32_t shadow_cmds_issued_this_cycle = 0;
    for (uint32_t issued = 0; issued < issue_budget; ++issued) {
      ReqBuffer::iterator req_it;
      ReqBuffer* buffer = nullptr;
      const bool allow_access = (access_cmds_issued_this_cycle < access_budget);
      bool request_found = schedule_request_filtered(
          req_it, buffer, used_access_banks, allow_access,
          background_cmds_issued_this_cycle, shadow_cmds_issued_this_cycle);
      if (!request_found) {
        break;
      }

      m_rowpolicy->update(request_found, req_it);

      const IssueCommandPlan issue_plan = build_issue_command_plan(req_it);
      if (issue_plan.issue_command < 0) {
        break;
      }
      if (!is_command_ready_for_issue(issue_plan.issue_command,
                                      req_it->addr_vec)) {
        break;
      }
      if (m_dram->m_command_meta(issue_plan.issue_command).is_closing &&
          has_active_rowgroup_conflict(req_it->addr_vec)) {
        break;
      }

      req_it->command = issue_plan.issue_command;
      for (auto plugin : m_plugins) {
        plugin->update(request_found, req_it);
      }
      req_it->command = issue_plan.issue_command;

      if (req_it->is_stat_updated == false) {
        update_request_stats(req_it);
      }

      const ExternalTrafficClass issued_traffic_class =
          get_external_traffic_class(*req_it);
      const int command = issue_plan.issue_command;
      const bool is_refreshing_command =
          m_dram->m_command_meta(command).is_refreshing;
      m_dram->issue_command(command, req_it->addr_vec);
      if (is_refreshing_command) {
        note_refresh_scope_enter(command, req_it->addr_vec);
      }
      if (issue_plan.forced_autoprecharge) {
        s_scoreboard_forced_autoprecharge++;
      }
      note_bankmachine_transition(issue_plan.transition);
      if (controller_scoreboard_enabled()) {
        if (is_refreshing_command) {
          s_shadow_scoreboard_refresh_enter_events++;
        }
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
      }

      if (issue_plan.completes_request) {
        Request completed_req = *req_it;
        if (req_it->type_id == Request::Type::Read) {
          completed_req.depart = m_clk + m_dram->m_read_latency;
        } else if (req_it->type_id == Request::Type::Write) {
          if (m_write_completion_mode == WriteCompletionMode::kPosted) {
            completed_req.depart = m_clk + 1;
          } else {
            Clk_t lat = m_dram->m_write_latency;
            if (lat < 1) lat = 1;
            completed_req.depart = m_clk + lat;
          }
        }
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
    s_max_cmds_issued_per_cycle =
        std::max<size_t>(s_max_cmds_issued_per_cycle, cmds_issued_this_cycle);
    s_max_access_cmds_issued_per_cycle = std::max<size_t>(
        s_max_access_cmds_issued_per_cycle, access_cmds_issued_this_cycle);
  };

 private:
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

  ProbeResult probe_row_state(ReqBuffer::iterator& req) {
    const SchedulingState scheduling_state =
        resolve_scheduling_state(req->final_command, req->addr_vec);
    if (controller_scoreboard_enabled() &&
        scheduling_state.rowstate_scoreboard_miss) {
      s_controller_rowstate_scoreboard_misses++;
    }
    return scheduling_state.row_state;
  }

  IssueCommandPlan build_issue_command_plan(ReqBuffer::iterator& req_it) {
    const SchedulingState scheduling_state =
        resolve_scheduling_state(req_it->final_command, req_it->addr_vec);
    if (controller_scoreboard_enabled() &&
        scheduling_state.rowstate_scoreboard_miss) {
      s_controller_rowstate_scoreboard_misses++;
    }
    return m_bank_machine.build_issue_plan(*req_it, scheduling_state, m_clk);
  }

  void note_bankmachine_transition(BankMachine::TransitionType transition) {
    switch (transition) {
      case BankMachine::TransitionType::kOpen:
        s_bankmachine_transition_open++;
        break;
      case BankMachine::TransitionType::kAccess:
        s_bankmachine_transition_access++;
        break;
      case BankMachine::TransitionType::kClose:
        s_bankmachine_transition_close++;
        break;
      case BankMachine::TransitionType::kRefresh:
        s_bankmachine_transition_refresh++;
        break;
      case BankMachine::TransitionType::kOther:
      case BankMachine::TransitionType::kInvalid:
      default:
        s_bankmachine_transition_other++;
        break;
    }
  }

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

  RefreshScopeKind refresh_scope_kind_from_command(int command) const {
    if (!m_dram) return RefreshScopeKind::kNone;
    try {
      const int scope_level = m_dram->m_command_scopes(command);
      const std::string_view scope_name = m_dram->m_levels(scope_level);
      if (scope_name == "bank") return RefreshScopeKind::kBank;
      if (scope_name == "bankgroup") return RefreshScopeKind::kBankGroup;
      if (scope_name == "rank") return RefreshScopeKind::kRank;
      if (scope_name == "channel") return RefreshScopeKind::kChannel;
    } catch (const std::out_of_range&) {
      return RefreshScopeKind::kNone;
    }
    return RefreshScopeKind::kNone;
  }

  static size_t refresh_scope_kind_to_stat(RefreshScopeKind scope_kind) {
    return static_cast<size_t>(scope_kind);
  }

  void note_refresh_scope_pending(int command, const AddrVec_t& scope_addr_vec) {
    m_refresh_state.pending = true;
    m_refresh_state.scope_kind = refresh_scope_kind_from_command(command);
    m_refresh_state.scope_addr_vec = scope_addr_vec;
    m_refresh_state.pending_since_cycle = m_clk;
    s_controller_refresh_pending_events++;
    s_controller_refresh_scope_kind_last =
        refresh_scope_kind_to_stat(m_refresh_state.scope_kind);
  }

  void note_refresh_scope_exit(const AddrVec_t& scope_addr_vec,
                               RefreshScopeKind scope_kind) {
    if (!m_refresh_state.active) return;

    m_refresh_state.active = false;
    m_refresh_state.pending = false;
    m_refresh_state.active_since_cycle = -1;
    m_refresh_state.active_until_cycle = -1;
    m_refresh_state.scope_addr_vec = scope_addr_vec;
    m_refresh_state.scope_kind = scope_kind;
    s_controller_refresh_exit_events++;
    s_controller_refresh_scope_kind_last =
        refresh_scope_kind_to_stat(scope_kind);

    if (controller_scoreboard_enabled()) {
      m_shadow_scoreboard.on_refresh_exit(scope_kind, scope_addr_vec, m_clk);
    }
  }

  void note_refresh_scope_enter(int command, const AddrVec_t& scope_addr_vec) {
    if (m_refresh_state.active) {
      note_refresh_scope_exit(m_refresh_state.scope_addr_vec,
                              m_refresh_state.scope_kind);
    }

    m_refresh_state.pending = false;
    m_refresh_state.active = true;
    m_refresh_state.scope_kind = refresh_scope_kind_from_command(command);
    m_refresh_state.scope_addr_vec = scope_addr_vec;
    m_refresh_state.active_since_cycle = m_clk;
    m_refresh_state.active_until_cycle =
        (m_refresh_window_cycles > 0) ? (m_clk + m_refresh_window_cycles) : -1;
    m_refresh_state.epoch++;

    s_controller_refresh_enter_events++;
    s_controller_refresh_epoch = static_cast<size_t>(m_refresh_state.epoch);
    s_controller_refresh_scope_kind_last =
        refresh_scope_kind_to_stat(m_refresh_state.scope_kind);
  }

  void tick_refresh_lifecycle_state() {
    if (m_refresh_state.pending) {
      s_controller_refresh_pending_cycles++;
    }
    if (!m_refresh_state.active) {
      return;
    }

    s_controller_refresh_active_cycles++;
    if (m_refresh_state.active_until_cycle >= 0 &&
        m_clk >= m_refresh_state.active_until_cycle) {
      note_refresh_scope_exit(m_refresh_state.scope_addr_vec,
                              m_refresh_state.scope_kind);
    }
  }

  void sample_shadow_refresh_state_peaks() {
    if (!controller_scoreboard_enabled()) return;
    s_shadow_scoreboard_refreshing_banks_peak = std::max<size_t>(
        s_shadow_scoreboard_refreshing_banks_peak,
        m_shadow_scoreboard.count_refreshing_banks());
    s_shadow_scoreboard_pending_banks_peak = std::max<size_t>(
        s_shadow_scoreboard_pending_banks_peak,
        m_shadow_scoreboard.count_refresh_pending_banks());
  }

  bool has_active_rowgroup_conflict(const AddrVec_t& rowgroup) {
    for (auto it = m_active_buffer.begin(); it != m_active_buffer.end(); ++it) {
      const auto& active_rowgroup = it->addr_vec;
      bool is_matching = true;
      for (int i = 0; i < m_bank_addr_idx + 1; i++) {
        if (active_rowgroup[i] != rowgroup[i] && active_rowgroup[i] != -1 &&
            rowgroup[i] != -1) {
          is_matching = false;
          break;
        }
      }
      if (is_matching) {
        return true;
      }
    }
    return false;
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
    oss << "BankParallel channel " << m_channel_id
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
        oss << "BankParallel channel " << m_channel_id
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

    const std::string msg = fmt::format(
        "BankParallel scoreboard/oracle overlay mismatch ch={} cmd={} final={} "
        "addr={} row_hit(ctrl/oracle)={}/{} row_open(ctrl/oracle)={}/{}",
        m_channel_id, command, req.final_command, req.addr,
        diff.scoreboard_row_hit, diff.dram_row_hit, diff.scoreboard_row_open,
        diff.dram_row_open);
    if (m_shadow_scoreboard_log_mismatch) {
      spdlog::warn("{}", msg);
    }
    if (m_shadow_scoreboard_fail_fast) {
      throw std::runtime_error(msg);
    }
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

    const ProbeResult probe = probe_row_state(req);
    const bool row_hit = probe.row_hit;
    const bool row_open = (!row_hit && probe.row_open);
    const int rowbuffer_state = row_hit ? 1 : (row_open ? 2 : 0);
    const size_t phase_idx = shmem_phase_index(*req);
    req->rowbuffer_event_valid = true;
    req->rowbuffer_event_state = rowbuffer_state;

    ControllerRowBufferEvent event {};
    event.valid = true;
    event.addr = static_cast<uint64_t>(req->addr);
    event.type_id = req->type_id;
    event.source_id = req->source_id;
    event.final_command = req->final_command;
    event.rowbuffer_state = rowbuffer_state;
    event.clk = static_cast<uint64_t>(m_clk);
    event.addr_vec = req->addr_vec;
    event.scratchpad = req->scratchpad;
    m_rowbuffer_event_ring.push_back(event);
    while (m_rowbuffer_event_ring.size() > kRowBufferEventRingCapacity) {
      m_rowbuffer_event_ring.pop_front();
    }

    if (req->type_id == Request::Type::Read) {
      if (row_hit) {
        s_read_row_hits++;
        s_row_hits++;
        s_phase_row_hits[phase_idx]++;
        s_read_row_hits_by_class[traffic_idx]++;
        s_row_hits_by_class[traffic_idx]++;
        if (is_valid_source_id(*req, m_num_cores)) {
          s_read_row_hits_per_core[req->source_id]++;
        }
      } else if (row_open) {
        s_read_row_conflicts++;
        s_row_conflicts++;
        s_phase_row_conflicts[phase_idx]++;
        s_read_row_conflicts_by_class[traffic_idx]++;
        s_row_conflicts_by_class[traffic_idx]++;
        if (is_valid_source_id(*req, m_num_cores)) {
          s_read_row_conflicts_per_core[req->source_id]++;
        }
      } else {
        s_read_row_misses++;
        s_row_misses++;
        s_phase_row_misses[phase_idx]++;
        s_read_row_misses_by_class[traffic_idx]++;
        s_row_misses_by_class[traffic_idx]++;
        if (is_valid_source_id(*req, m_num_cores)) {
          s_read_row_misses_per_core[req->source_id]++;
        }
      }
    } else if (req->type_id == Request::Type::Write) {
      if (row_hit) {
        s_write_row_hits++;
        s_row_hits++;
        s_phase_row_hits[phase_idx]++;
        s_write_row_hits_by_class[traffic_idx]++;
        s_row_hits_by_class[traffic_idx]++;
      } else if (row_open) {
        s_write_row_conflicts++;
        s_row_conflicts++;
        s_phase_row_conflicts[phase_idx]++;
        s_write_row_conflicts_by_class[traffic_idx]++;
        s_row_conflicts_by_class[traffic_idx]++;
      } else {
        s_write_row_misses++;
        s_row_misses++;
        s_phase_row_misses[phase_idx]++;
        s_write_row_misses_by_class[traffic_idx]++;
        s_row_misses_by_class[traffic_idx]++;
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

  bool violates_access_bank_parallelism(
      const Request& req,
      const std::unordered_set<std::string>& used_access_banks) const {
    if (!m_dram->m_command_meta(req.command).is_accessing) {
      return false;
    }
    return used_access_banks.find(bank_key(req.addr_vec)) !=
           used_access_banks.end();
  }

  struct BankCandidate {
    ReqBuffer* buffer = nullptr;
    ReqBuffer::iterator it {};
    std::string bank;
    ReadyBlockReason block_reason = ReadyBlockReason::kNone;
  };

  bool candidate_rhs_better(const BankCandidate& lhs, const BankCandidate& rhs,
                            bool prioritize_traffic_class,
                            bool count_foreground_qos_win) {
    if (prioritize_traffic_class && prefer_request_qos(*rhs.it, *lhs.it) &&
        !prefer_request_qos(*lhs.it, *rhs.it)) {
      if (count_foreground_qos_win &&
          get_external_traffic_class(*rhs.it) ==
              ExternalTrafficClass::kForeground &&
          get_external_traffic_class(*lhs.it) !=
              ExternalTrafficClass::kForeground) {
        s_foreground_qos_wins++;
      }
      return true;
    }
    ReqBuffer::iterator better = m_scheduler->compare(lhs.it, rhs.it);
    return &(*better) == &(*rhs.it);
  }

  void collect_bank_candidates_from_buffer(
      ReqBuffer& buffer, ReqBuffer* buffer_ptr,
      const std::unordered_set<std::string>& used_access_banks,
      bool allow_access, bool prioritize_traffic_class,
      uint32_t background_cmds_issued_this_cycle,
      uint32_t shadow_cmds_issued_this_cycle,
      std::unordered_map<std::string, BankCandidate>& bank_candidates,
      std::unordered_map<std::string, BankCandidate>* blocked_bank_candidates) {
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
        if (blocked_bank_candidates != nullptr &&
            scheduling_state.ready_block_reason != ReadyBlockReason::kNone) {
          const std::string bank = bank_key(it->addr_vec);
          BankCandidate blocked_incoming {
              buffer_ptr, it, bank, scheduling_state.ready_block_reason};
          auto blocked_found = blocked_bank_candidates->find(bank);
          if (blocked_found == blocked_bank_candidates->end()) {
            blocked_bank_candidates->emplace(bank, blocked_incoming);
          } else if (candidate_rhs_better(blocked_found->second,
                                          blocked_incoming,
                                          prioritize_traffic_class, false)) {
            blocked_found->second = blocked_incoming;
          }
        }
        continue;
      }

      const std::string bank = bank_key(it->addr_vec);
      BankCandidate incoming {buffer_ptr, it, bank, ReadyBlockReason::kNone};
      auto found = bank_candidates.find(bank);
      if (found == bank_candidates.end()) {
        bank_candidates.emplace(bank, incoming);
        continue;
      }
      if (candidate_rhs_better(found->second, incoming, prioritize_traffic_class,
                               false)) {
        found->second = incoming;
      }
    }
  }

  bool select_best_ready_from_buffers(
      const std::vector<ReqBuffer*>& buffers,
      const std::unordered_set<std::string>& used_access_banks,
      bool allow_access, bool prioritize_traffic_class,
      uint32_t background_cmds_issued_this_cycle,
      uint32_t shadow_cmds_issued_this_cycle, ReqBuffer::iterator& req_it,
      ReqBuffer*& req_buffer,
      ControllerReadyBlockReason* blocked_reason_out = nullptr) {
    if (blocked_reason_out != nullptr) {
      *blocked_reason_out = ControllerReadyBlockReason::kNone;
    }
    std::unordered_map<std::string, BankCandidate> bank_candidates;
    bank_candidates.reserve(32);
    std::unordered_map<std::string, BankCandidate> blocked_bank_candidates;
    blocked_bank_candidates.reserve(32);
    for (auto* buffer : buffers) {
      if (buffer == nullptr || buffer->size() == 0) continue;
      collect_bank_candidates_from_buffer(
          *buffer, buffer, used_access_banks, allow_access,
          prioritize_traffic_class, background_cmds_issued_this_cycle,
          shadow_cmds_issued_this_cycle, bank_candidates,
          blocked_reason_out != nullptr ? &blocked_bank_candidates : nullptr);
    }

    bool found = false;
    BankCandidate best {};
    for (const auto& kv : bank_candidates) {
      const BankCandidate& cand = kv.second;
      if (!found) {
        best = cand;
        found = true;
        continue;
      }
      if (candidate_rhs_better(best, cand, prioritize_traffic_class, true)) {
        best = cand;
      }
    }

    if (!found) {
      if (blocked_reason_out != nullptr && !blocked_bank_candidates.empty()) {
        bool blocked_found = false;
        BankCandidate blocked_best {};
        for (const auto& kv : blocked_bank_candidates) {
          const BankCandidate& cand = kv.second;
          if (!blocked_found) {
            blocked_best = cand;
            blocked_found = true;
            continue;
          }
          if (candidate_rhs_better(blocked_best, cand, prioritize_traffic_class,
                                   false)) {
            blocked_best = cand;
          }
        }
        if (blocked_found) {
          *blocked_reason_out = controller_ready_block_reason_from_scoreboard(
              blocked_best.block_reason);
        }
      }
      return false;
    }
    req_it = best.it;
    req_buffer = best.buffer;
    return true;
  }

  bool is_command_ready_for_issue(int command, const AddrVec_t& addr_vec) {
    if (command < 0) return false;
    if (!controller_scoreboard_enabled()) {
      return m_dram->check_ready(command, addr_vec);
    }

    const bool controller_ready = is_command_ready(command, addr_vec);
    note_ready_overlay_mismatch(command, addr_vec, controller_ready);
    return controller_ready;
  }

  bool schedule_request_filtered(
      ReqBuffer::iterator& req_it, ReqBuffer*& req_buffer,
      const std::unordered_set<std::string>& used_access_banks,
      bool allow_access, uint32_t background_cmds_issued_this_cycle,
      uint32_t shadow_cmds_issued_this_cycle) {
    bool request_found = false;
    ControllerReadyBlockReason blocked_reason =
        ControllerReadyBlockReason::kNone;
    ControllerReadyBlockReason current_blocked_reason =
        ControllerReadyBlockReason::kNone;

    if (select_best_ready_from_buffers(
            {&m_active_buffer}, used_access_banks, allow_access, m_qos_enable,
            background_cmds_issued_this_cycle, shadow_cmds_issued_this_cycle,
            req_it, req_buffer, &current_blocked_reason)) {
      request_found = true;
    } else {
      blocked_reason = choose_ready_block_reason(blocked_reason,
                                                 current_blocked_reason);
    }

    if (!request_found) {
      if (m_priority_buffer.size() != 0) {
        current_blocked_reason = ControllerReadyBlockReason::kNone;
        request_found = select_best_ready_from_buffers(
            {&m_priority_buffer}, used_access_banks, allow_access, false,
            background_cmds_issued_this_cycle, shadow_cmds_issued_this_cycle,
            req_it, req_buffer, &current_blocked_reason);
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
        ReqBuffer::iterator primary_it;
        ReqBuffer::iterator secondary_it;
        ReqBuffer* primary_selected_buffer = nullptr;
        ReqBuffer* secondary_selected_buffer = nullptr;
        ControllerReadyBlockReason primary_blocked_reason =
            ControllerReadyBlockReason::kNone;
        ControllerReadyBlockReason secondary_blocked_reason =
            ControllerReadyBlockReason::kNone;
        const bool primary_ready = select_best_ready_from_buffers(
            {primary_buffer}, used_access_banks, allow_access, true,
            background_cmds_issued_this_cycle, shadow_cmds_issued_this_cycle,
            primary_it, primary_selected_buffer, &primary_blocked_reason);
        bool secondary_ready = false;
        if (!primary_ready ||
            (m_is_write_mode &&
             m_allow_foreground_read_interrupt_write_mode)) {
          secondary_ready = select_best_ready_from_buffers(
              {secondary_buffer}, used_access_banks, allow_access, true,
              background_cmds_issued_this_cycle, shadow_cmds_issued_this_cycle,
              secondary_it, secondary_selected_buffer,
              &secondary_blocked_reason);
        }

        if (m_is_write_mode && m_allow_foreground_read_interrupt_write_mode &&
            secondary_ready) {
          bool secondary_should_win = !primary_ready;
          if (!secondary_should_win) {
            const BankCandidate primary_cand {primary_selected_buffer, primary_it,
                                              bank_key(primary_it->addr_vec)};
            const BankCandidate secondary_cand {
                secondary_selected_buffer, secondary_it,
                bank_key(secondary_it->addr_vec)};
            secondary_should_win = candidate_rhs_better(
                primary_cand, secondary_cand, true, false);
          }
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
          req_buffer = primary_selected_buffer;
          request_found = true;
        }
        if (!request_found && secondary_ready) {
          req_it = secondary_it;
          req_buffer = secondary_selected_buffer;
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
    return request_found;
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
