#ifndef RAMULATOR_CONTROLLER_CONTROLLER_H
#define RAMULATOR_CONTROLLER_CONTROLLER_H

#include <algorithm>
#include <cstdint>
#include <vector>
#include <deque>

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include "base/base.h"
#include "dram/dram.h"
#include "dram_controller/scheduler.h"
#include "dram_controller/plugin.h"
#include "dram_controller/refresh.h"
#include "dram_controller/rowpolicy.h"


namespace Ramulator {

enum class ControllerRefreshMode : int {
  kUnknown = 0,
  kStaticNormal = 1,
};

enum class ControllerRefreshScope : int {
  kNone = 0,
  kBank = 1,
  kBankGroup = 2,
  kRank = 3,
  kChannel = 4,
  kMixed = 5,
};

enum class ControllerReadyBlockReason : int {
  kNone = 0,
  kScoreboardMiss = 1,
  kInvalidCommand = 2,
  kAddressDecodeMiss = 3,
  kRankRefreshActive = 4,
  kRankRefreshRecovery = 5,
  kRankPrechargeTiming = 6,
  kRankRefreshTiming = 7,
  kRefreshScopeOpenRows = 8,
  kBankRefreshActive = 9,
  kBankRefreshRecovery = 10,
  kBankOpen = 11,
  kBankClosed = 12,
  kRowConflict = 13,
  kActivateWindow = 14,
  kFourActivateWindow = 15,
  kBankTimingAct = 16,
  kBankTimingPre = 17,
  kColumnBusTiming = 18,
  kReadDataTiming = 19,
  kWriteDataTiming = 20,
  kReadTurnaroundTiming = 21,
  kWriteTurnaroundTiming = 22,
  kBankTimingRead = 23,
  kBankTimingWrite = 24,
  kMixed = 25,
};

enum class ControllerTemperatureBucket : int {
  kUnknown = 0,
  kNominal = 1,
  kWarm = 2,
  kHot = 3,
};

struct ControllerTelemetry {
  bool valid = false;
  int rowbuffer_state = 0;
  bool bank_busy = false;
  bool refreshing = false;
  uint64_t refresh_horizon_cycles = 0;
  uint64_t queue_occupancy = 0;
  uint64_t queue_capacity = 0;
  uint32_t queue_pressure_permille = 0;
  uint64_t refresh_epoch = 0;
  ControllerRefreshMode refresh_mode = ControllerRefreshMode::kUnknown;
};

struct ControllerCommandState {
  bool valid = false;
  int next_command = -1;
  bool next_command_ready = false;
  ControllerReadyBlockReason ready_block_reason =
      ControllerReadyBlockReason::kNone;
};

struct ControllerTelemetrySummaryV2 {
  bool valid = false;
  uint32_t controller_count = 0;
  uint64_t clk = 0;

  uint64_t queue_total_occupancy = 0;
  uint64_t queue_total_capacity = 0;
  uint32_t queue_total_pressure_permille = 0;

  uint64_t queue_active_occupancy = 0;
  uint64_t queue_active_capacity = 0;
  uint64_t queue_read_occupancy = 0;
  uint64_t queue_read_capacity = 0;
  uint64_t queue_write_occupancy = 0;
  uint64_t queue_write_capacity = 0;
  uint64_t queue_priority_occupancy = 0;
  uint64_t queue_priority_capacity = 0;
  uint64_t queue_pending_occupancy = 0;

  uint64_t queue_counted_total_occupancy = 0;
  uint64_t queue_counted_foreground_occupancy = 0;
  uint64_t queue_counted_background_occupancy = 0;
  uint64_t queue_counted_shadow_occupancy = 0;

  uint32_t cmd_issue_budget = 0;
  uint32_t access_budget = 0;

  uint64_t open_banks = 0;
  uint64_t inflight_banks = 0;
  uint64_t autoprecharge_armed_banks = 0;
  uint64_t max_open_row_age_cycles = 0;

  bool refresh_pending = false;
  bool refresh_active = false;
  bool refresh_recovery = false;
  uint64_t refresh_epoch = 0;
  uint64_t refresh_horizon_cycles = 0;
  uint64_t refresh_window_cycles = 0;
  uint64_t next_refresh_deadline_cycles = 0;
  uint64_t refresh_slack_cycles = 0;
  uint32_t refresh_pressure_permille = 0;
  uint64_t refresh_pending_banks = 0;
  uint64_t refreshing_banks = 0;
  ControllerRefreshScope refresh_scope = ControllerRefreshScope::kNone;
  ControllerRefreshMode refresh_mode = ControllerRefreshMode::kUnknown;
  bool ready_blocked = false;
  ControllerReadyBlockReason ready_block_reason =
      ControllerReadyBlockReason::kNone;

  bool thermal_valid = false;
  ControllerTemperatureBucket temperature_bucket =
      ControllerTemperatureBucket::kUnknown;

  bool tiered_valid = false;
  uint32_t num_tiers = 0;
  uint32_t shared_access_budget = 0;
  uint32_t tier_access_budget = 0;
  uint32_t vertical_transfer_budget = 0;
  uint32_t vertical_transfer_active = 0;
};

inline uint64_t controller_refresh_slack_cycles_from_hint(
    const RefreshScheduleHint& hint) {
  return hint.valid ? hint.next_refresh_deadline_cycles : 0;
}

inline uint32_t controller_refresh_pressure_permille_from_hint(
    const RefreshScheduleHint& hint, bool refresh_pending, bool refresh_active,
    bool refresh_recovery) {
  if (refresh_pending || refresh_active || refresh_recovery) {
    return 1000;
  }
  if (!hint.valid || hint.refresh_interval_cycles == 0) {
    return 0;
  }

  const uint64_t interval_cycles = hint.refresh_interval_cycles;
  const uint64_t deadline_cycles =
      std::min<uint64_t>(hint.next_refresh_deadline_cycles, interval_cycles);
  const uint64_t elapsed_cycles = interval_cycles - deadline_cycles;
  return static_cast<uint32_t>(
      (elapsed_cycles * 1000ULL + interval_cycles / 2ULL) /
      interval_cycles);
}

struct ControllerTelemetryObservationSampleV1 {
  bool valid = false;
  ControllerTelemetrySummaryV2 summary {};

  uint64_t row_hits = 0;
  uint64_t row_misses = 0;
  uint64_t row_conflicts = 0;

  uint64_t foreground_row_hits = 0;
  uint64_t foreground_row_misses = 0;
  uint64_t foreground_row_conflicts = 0;

  uint64_t background_row_hits = 0;
  uint64_t background_row_misses = 0;
  uint64_t background_row_conflicts = 0;

  uint64_t shadow_row_hits = 0;
  uint64_t shadow_row_misses = 0;
  uint64_t shadow_row_conflicts = 0;

  uint64_t local_accesses = 0;
  uint64_t cross_tier_accesses = 0;
  uint64_t vertical_copy_accesses = 0;
};

struct ControllerRowBufferEvent {
  bool valid = false;
  uint64_t addr = 0;
  int type_id = -1;
  int source_id = -1;
  int final_command = -1;
  int rowbuffer_state = 0;
  uint64_t clk = 0;
  AddrVec_t addr_vec {};
  std::array<int, 12> scratchpad = {0};
};

class IDRAMController : public Clocked<IDRAMController> {
  RAMULATOR_REGISTER_INTERFACE(IDRAMController, "Controller", "Memory Controller Interface");

  public:
    IDRAM*  m_dram = nullptr;          
    IScheduler*   m_scheduler = nullptr;
    IRefreshManager*   m_refresh = nullptr;
    IRowPolicy*   m_rowpolicy = nullptr;
    std::vector<IControllerPlugin*> m_plugins;

    int m_channel_id = -1;
  public:
    /**
     * @brief       Send a request to the memory controller.
     * 
     * @param    req        The request to be enqueued.
     * @return   true       Successful.
     * @return   false      Failed (e.g., buffer full).
     */
    virtual bool send(Request& req) = 0;

    /**
     * @brief       Send a high-priority request to the memory controller.
     * 
     */
    virtual bool priority_send(Request& req) = 0;

    /**
     * @brief       Ticks the memory controller.
     * 
     */
    virtual void tick() = 0;

    /**
     * @brief       Optional controller-owned prerequisite command resolution.
     *
     * @details
     * Default behavior delegates to DRAM device model (legacy Ramulator2 flow).
     * Controllers that maintain explicit bank/row state (e.g., scoreboard) can
     * override this to become the single source of truth for prereq selection.
     */
    virtual int get_prereq_command(int final_command,
                                   const AddrVec_t& addr_vec) const {
      if (!m_dram) return -1;
      return m_dram->get_preq_command(final_command, addr_vec);
    }

    /**
     * @brief       Optional controller-owned command readiness check.
     *
     * @details
     * Default behavior delegates to DRAM device model (legacy Ramulator2 flow).
     * Controllers that maintain explicit timing state can override this to
     * become the single source of truth for readiness decisions.
     */
    virtual bool is_command_ready(int command, const AddrVec_t& addr_vec) const {
      if (!m_dram) return false;
      return m_dram->check_ready(command, addr_vec);
    }

    /**
     * @brief       Optional controller-owned combined prereq/ready query.
     *
     * @details
     * This is the preferred API for schedulers / row policies that want a
     * consistent hot-path answer for:
     *   1) which command should issue next for a request, and
     *   2) whether that command is ready now.
     *
     * Default behavior is derived from `get_prereq_command()` and
     * `is_command_ready()`. Controllers with explicit scoreboard/bank-machine
     * state can override this to answer both from the same source of truth.
     */
    virtual bool query_command_state(int final_command,
                                     const AddrVec_t& addr_vec,
                                     ControllerCommandState& result) const {
      result = ControllerCommandState {};
      if (final_command < 0) return false;

      result.next_command = get_prereq_command(final_command, addr_vec);
      if (result.next_command < 0) return false;

      result.valid = true;
      result.next_command_ready =
          is_command_ready(result.next_command, addr_vec);
      return true;
    }

    /**
     * @brief       Optional row-buffer probe at controller level.
     *
     * @details
     * Returns true if the controller can answer the probe. `result` follows:
     *   0 = closed / unavailable, 1 = row hit, 2 = row open conflict.
     */
    virtual bool probe_rowbuffer(int final_command, const AddrVec_t& addr_vec,
                                 int& result) const {
      (void)final_command;
      (void)addr_vec;
      result = 0;
      return false;
    }

    virtual bool query_telemetry(int final_command, const AddrVec_t& addr_vec,
                                 ControllerTelemetry& result) const {
      (void)final_command;
      (void)addr_vec;
      result = ControllerTelemetry {};
      return false;
    }

    virtual bool query_telemetry_summary_v2(
        ControllerTelemetrySummaryV2& result) const {
      result = ControllerTelemetrySummaryV2 {};
      return false;
    }

    virtual bool query_telemetry_observation_sample_v1(
        ControllerTelemetryObservationSampleV1& result) const {
      result = ControllerTelemetryObservationSampleV1 {};
      return false;
    }

    virtual bool consume_rowbuffer_event(uint64_t addr,
                                         const AddrVec_t& addr_vec,
                                         int type_id, int source_id,
                                         ControllerRowBufferEvent& result) {
      (void)addr;
      (void)addr_vec;
      (void)type_id;
      (void)source_id;
      result = ControllerRowBufferEvent {};
      return false;
    }
   
};

}       // namespace Ramulator

#endif  // RAMULATOR_CONTROLLER_CONTROLLER_H
