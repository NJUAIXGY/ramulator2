#include <algorithm>
#include <vector>

#include "base/base.h"
#include "dram_controller/controller.h"
#include "dram_controller/scheduler.h"
#include "dram_controller/rowpolicy.h"

namespace Ramulator {

class OpenRowPolicy : public IRowPolicy, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IRowPolicy, OpenRowPolicy, "OpenRowPolicy", "Open Row Policy.")
  private:
    
  public:
    void init() override { };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override { };

    void update(bool request_found, ReqBuffer::iterator& req_it) override { 
      // OpenRowPolicy does not need to take any actions
    };


};

class ClosedRowPolicy : public IRowPolicy, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IRowPolicy, ClosedRowPolicy, "ClosedRowPolicy", "Close Row Policy.")
  private:
    IDRAM* m_dram;
    
    int m_PRE_req_id = -1;
    
    int m_cap = -1;
    
    int m_rank_level = -1;
    int m_bankgroup_level = -1;
    int m_bank_level = -1;
    int m_row_level = -1;
    int m_num_ranks = -1;
    int m_num_bankgroups = -1;
    int m_num_banks = -1;

    int s_num_close_reqs = 0;

    std::vector<uint64_t> m_col_accesses;

  public:
    void init() override { };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      m_ctrl = cast_parent<IDRAMController>();
      m_dram = m_ctrl->m_dram;

      m_cap = param<int>("cap").default_val(10000000); // TODO

      m_rank_level = m_dram->m_levels("rank");
      m_bankgroup_level = m_dram->m_levels("bankgroup");
      m_bank_level = m_dram->m_levels("bank");
      m_row_level = m_dram->m_levels("row");

      m_PRE_req_id = m_dram->m_requests("close-row");

      m_num_ranks = m_dram->get_level_size("rank");
      m_num_bankgroups = m_dram->get_level_size("bankgroup");
      m_num_banks = m_dram->get_level_size("bank");
      
      m_col_accesses.resize(m_num_banks * m_num_bankgroups * m_num_ranks, 0);

      register_stat(s_num_close_reqs).name("num_close_reqs");
    };

    void update(bool request_found, ReqBuffer::iterator& req_it) override {

      if (!request_found)
        return;

      if (m_dram->m_command_meta(req_it->command).is_closing ||
          m_dram->m_command_meta(req_it->command).is_refreshing)  // PRE or REF 
      {  

        if (req_it->addr_vec[m_bankgroup_level] == -1 && req_it->addr_vec[m_bank_level] == -1) {  // all bank closes
          for (int b = 0; b < m_num_banks; b++) {
            for (int bg = 0; bg < m_num_bankgroups; bg++) {
              int rank_id = req_it->addr_vec[m_rank_level];
              int flat_bank_id = b + bg * m_num_banks + rank_id * m_num_banks * m_num_bankgroups;
              m_col_accesses[flat_bank_id] = 0;
            }
          }
        } else if (req_it->addr_vec[m_bankgroup_level] == -1) {  // same bank closes
          for (int bg = 0; bg < m_num_bankgroups; bg++) {
            int bank_id = req_it->addr_vec[m_bank_level];
            int rank_id = req_it->addr_vec[m_rank_level];
            int flat_bank_id = bank_id + bg * m_num_banks + rank_id * m_num_banks * m_num_bankgroups;
            m_col_accesses[flat_bank_id] = 0;
          }
        } else {  // single bank closes  (PRE, VRR, RDA, WRA)
          int flat_bank_id = req_it->addr_vec[m_bank_level] + 
                             req_it->addr_vec[m_bankgroup_level] * m_num_banks + 
                             req_it->addr_vec[m_rank_level] * m_num_banks * m_num_bankgroups;

          m_col_accesses[flat_bank_id] = 0;
        }
      } else if (m_dram->m_command_meta(req_it->command).is_accessing)  // RD or WR
      {
        int flat_bank_id = req_it->addr_vec[m_bank_level] + 
                           req_it->addr_vec[m_bankgroup_level] * m_num_banks + 
                           req_it->addr_vec[m_rank_level] * m_num_banks * m_num_bankgroups;
        
        m_col_accesses[flat_bank_id]++;

        if (m_col_accesses[flat_bank_id] >= m_cap) {
          Request req(req_it->addr_vec, m_PRE_req_id);
          m_ctrl->priority_send(req);
          m_col_accesses[flat_bank_id] = 0;
          s_num_close_reqs++;
        }
      }
    };
};

// Always use auto-precharge (RDA/WRA) as the final command for reads/writes.
// This models closed-page behavior without injecting extra PRE requests.
class AutoPrechargePolicy : public IRowPolicy, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(
      IRowPolicy, AutoPrechargePolicy, "AutoPrechargePolicy",
      "Always use RDA/WRA for reads/writes (closed-page via auto-precharge).");

 private:
  IDRAM* m_dram = nullptr;
  int m_cmd_rda = -1;
  int m_cmd_wra = -1;

 public:
  void init() override { };

  void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
    m_ctrl = cast_parent<IDRAMController>();
    m_dram = m_ctrl->m_dram;

    try {
      m_cmd_rda = m_dram->m_commands("RDA");
      m_cmd_wra = m_dram->m_commands("WRA");
    } catch (const std::out_of_range&) {
      throw std::runtime_error(
          "AutoPrechargePolicy requires DRAM commands RDA/WRA.");
    }
  };

  void update(bool request_found, ReqBuffer::iterator& req_it) override {
    if (!request_found) return;

    int new_final = -1;
    if (req_it->type_id == Request::Type::Read) {
      new_final = m_cmd_rda;
    } else if (req_it->type_id == Request::Type::Write) {
      new_final = m_cmd_wra;
    } else {
      return;
    }

    // Recompute the preq command so the controller issues RDA/WRA when ready.
    req_it->final_command = new_final;
    ControllerCommandState command_state {};
    if (m_ctrl->query_command_state(req_it->final_command, req_it->addr_vec,
                                    command_state)) {
      req_it->command = command_state.next_command;
    } else {
      req_it->command = -1;
    }
  };
};

// Keep a row open for up to N accesses per bank, then close it using RDA/WRA on
// the Nth access. This is a lightweight CacheRAM-style row-buffer model:
// - avoids injecting separate PRE maintenance requests (which can distort
//   bottlenecks)
// - still bounds row "on time" / open-page residency via cap
class CapAutoPrechargePolicy : public IRowPolicy, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(
      IRowPolicy, CapAutoPrechargePolicy, "CapAutoPrechargePolicy",
      "Open rows up to cap accesses per bank, then close using RDA/WRA.");

 private:
  IDRAM* m_dram = nullptr;

  int m_cap = 1;

  int m_rank_level = -1;
  int m_bankgroup_level = -1;
  int m_bank_level = -1;
  int m_num_ranks = -1;
  int m_num_bankgroups = -1;
  int m_num_banks = -1;

  int m_cmd_rda = -1;
  int m_cmd_wra = -1;

  std::vector<uint64_t> m_accesses;
  uint64_t s_num_forced_autoprecharge = 0;

  int flat_bank_id(const AddrVec_t& addr_vec) const {
    const int bank_id = addr_vec[m_bank_level];
    const int bg_id = addr_vec[m_bankgroup_level];
    const int rank_id = addr_vec[m_rank_level];
    if (bank_id < 0 || bg_id < 0 || rank_id < 0) return -1;
    return bank_id + bg_id * m_num_banks +
           rank_id * m_num_banks * m_num_bankgroups;
  }

  void reset_bank_counters_for_close(const AddrVec_t& addr_vec) {
    // Reset counters when the DRAM closes banks (PRE/RDA/WRA/PREab...).
    if (m_accesses.empty()) return;

    const int bank_id = addr_vec[m_bank_level];
    const int bg_id = addr_vec[m_bankgroup_level];
    const int rank_id = addr_vec[m_rank_level];

    if (rank_id < 0) return;

    if (bg_id == -1 && bank_id == -1) {
      // All banks in the rank close.
      for (int b = 0; b < m_num_banks; ++b) {
        for (int bg = 0; bg < m_num_bankgroups; ++bg) {
          const int flat = b + bg * m_num_banks +
                           rank_id * m_num_banks * m_num_bankgroups;
          if (flat >= 0 && flat < (int)m_accesses.size()) m_accesses[flat] = 0;
        }
      }
      return;
    }

    if (bg_id == -1 && bank_id >= 0) {
      // Same bank index across all bankgroups closes.
      for (int bg = 0; bg < m_num_bankgroups; ++bg) {
        const int flat = bank_id + bg * m_num_banks +
                         rank_id * m_num_banks * m_num_bankgroups;
        if (flat >= 0 && flat < (int)m_accesses.size()) m_accesses[flat] = 0;
      }
      return;
    }

    const int flat = flat_bank_id(addr_vec);
    if (flat >= 0 && flat < (int)m_accesses.size()) m_accesses[flat] = 0;
  }

 public:
  void init() override { };

  void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
    m_ctrl = cast_parent<IDRAMController>();
    m_dram = m_ctrl->m_dram;

    m_cap = param<int>("cap").default_val(4);
    if (m_cap < 1) m_cap = 1;

    m_rank_level = m_dram->m_levels("rank");
    m_bankgroup_level = m_dram->m_levels("bankgroup");
    m_bank_level = m_dram->m_levels("bank");

    m_num_ranks = m_dram->get_level_size("rank");
    m_num_bankgroups = m_dram->get_level_size("bankgroup");
    m_num_banks = m_dram->get_level_size("bank");

    try {
      m_cmd_rda = m_dram->m_commands("RDA");
      m_cmd_wra = m_dram->m_commands("WRA");
    } catch (const std::out_of_range&) {
      throw std::runtime_error(
          "CapAutoPrechargePolicy requires DRAM commands RDA/WRA.");
    }

    const int total = m_num_banks * m_num_bankgroups * m_num_ranks;
    m_accesses.assign((size_t)std::max(0, total), 0);

    register_stat(s_num_forced_autoprecharge).name("num_forced_autoprecharge");
  };

  void update(bool request_found, ReqBuffer::iterator& req_it) override {
    if (!request_found) return;

    const int cmd = req_it->command;
    const auto meta = m_dram->m_command_meta(cmd);

    // Count accesses per bank and optionally force auto-precharge on the Nth one.
    if (meta.is_accessing) {
      const int flat = flat_bank_id(req_it->addr_vec);
      if (flat >= 0 && flat < (int)m_accesses.size()) {
        const uint64_t next = m_accesses[flat] + 1;
        m_accesses[flat] = next;

        // Only force auto-precharge when this access is the final command of a
        // read/write request (RD/WR). Otherwise we'd be changing internal
        // maintenance semantics.
        const bool is_final = (req_it->command == req_it->final_command);
        const bool should_close = (next >= (uint64_t)m_cap);
        if (is_final && should_close) {
          int new_final = -1;
          if (req_it->type_id == Request::Type::Read) {
            new_final = m_cmd_rda;
          } else if (req_it->type_id == Request::Type::Write) {
            new_final = m_cmd_wra;
          }
          if (new_final != -1) {
            // Apply only if RDA/WRA is directly ready; otherwise keep RD/WR
            // and try again later (avoid invalidating scheduler readiness).
            ControllerCommandState command_state {};
            if (m_ctrl->query_command_state(new_final, req_it->addr_vec,
                                            command_state) &&
                command_state.next_command == new_final &&
                command_state.next_command_ready) {
              req_it->final_command = new_final;
              req_it->command = new_final;
              m_accesses[flat] = 0;  // auto-precharge closes the row
              s_num_forced_autoprecharge++;
            }
          }
        }
      }
    }

    // If this command closes banks (e.g., PRE or RDA/WRA), reset counters.
    if (meta.is_closing || meta.is_refreshing) {
      reset_bank_counters_for_close(req_it->addr_vec);
    }
  };
};

}       // namespace Ramulator
