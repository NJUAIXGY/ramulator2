#ifndef RAMULATOR_CONTROLLER_CONTROLLER_H
#define RAMULATOR_CONTROLLER_CONTROLLER_H

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
   
};

}       // namespace Ramulator

#endif  // RAMULATOR_CONTROLLER_CONTROLLER_H
