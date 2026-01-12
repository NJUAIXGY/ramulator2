#include "dram_controller/controller.h"
#include "memory_system/memory_system.h"

#include <algorithm>
#include <deque>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

namespace Ramulator {

class BankParallelDRAMController final : public IDRAMController,
                                         public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(
      IDRAMController, BankParallelDRAMController, "BankParallel",
      "A DRAM controller that can issue up to N commands per cycle and limits "
      "concurrent RD/WR-like accesses to distinct banks within a channel.")

 private:
  std::deque<Request>
      pending;  // A queue for read requests that are about to finish

  ReqBuffer m_active_buffer;    // Buffer for requests being served
  ReqBuffer m_priority_buffer;  // Buffer for high-priority requests
  ReqBuffer m_read_buffer;      // Read request buffer
  ReqBuffer m_write_buffer;     // Write request buffer

  int m_bank_addr_idx = -1;

  float m_wr_low_watermark;
  float m_wr_high_watermark;
  bool m_is_write_mode = false;

  uint32_t m_bank_parallel_ports = 1;

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
  size_t s_queue_len = 0;
  size_t s_read_queue_len = 0;
  size_t s_write_queue_len = 0;
  size_t s_priority_queue_len = 0;
  float s_queue_len_avg = 0;
  float s_read_queue_len_avg = 0;
  float s_write_queue_len_avg = 0;
  float s_priority_queue_len_avg = 0;

  size_t s_read_latency = 0;
  float s_avg_read_latency = 0;

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
    m_priority_buffer.max_size = 512 * 3 + 32;

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

    register_stat(s_read_latency).name("read_latency_{}", m_channel_id);
    register_stat(s_avg_read_latency).name("avg_read_latency_{}", m_channel_id);
  };

  bool send(Request& req) override {
    req.final_command = m_dram->m_request_translations(req.type_id);

    switch (req.type_id) {
      case Request::Type::Read: {
        s_num_read_reqs++;
        break;
      }
      case Request::Type::Write: {
        s_num_write_reqs++;
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
        req.depart = m_clk + 1;
        pending.push_back(req);
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

    return true;
  };

  bool priority_send(Request& req) override {
    req.final_command = m_dram->m_request_translations(req.type_id);
    return m_priority_buffer.enqueue(req);
  }

  void tick() override {
    m_clk++;

    s_queue_len +=
        m_read_buffer.size() + m_write_buffer.size() + m_priority_buffer.size() +
        pending.size();
    s_read_queue_len += m_read_buffer.size() + pending.size();
    s_write_queue_len += m_write_buffer.size();
    s_priority_queue_len += m_priority_buffer.size();

    serve_completed_pending();

    m_refresh->tick();

    std::unordered_set<std::string> used_access_banks;
    used_access_banks.reserve(std::max<uint32_t>(1, m_bank_parallel_ports) * 2);

    const uint32_t issue_budget = std::max<uint32_t>(1, m_bank_parallel_ports);
    for (uint32_t issued = 0; issued < issue_budget; ++issued) {
      ReqBuffer::iterator req_it;
      ReqBuffer* buffer = nullptr;
      bool request_found = schedule_request_filtered(req_it, buffer, used_access_banks);
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

      const int command = req_it->command;
      m_dram->issue_command(command, req_it->addr_vec);

      if (m_dram->m_command_meta(command).is_accessing) {
        used_access_banks.insert(bank_key(req_it->addr_vec));
      }

      if (command == req_it->final_command) {
        if (req_it->type_id == Request::Type::Read) {
          req_it->depart = m_clk + m_dram->m_read_latency;
          pending.push_back(*req_it);
        } else if (req_it->type_id == Request::Type::Write) {
          req_it->depart = m_clk + 1;
          pending.push_back(*req_it);
        }
        buffer->remove(req_it);
      } else {
        if (m_dram->m_command_meta(command).is_opening) {
          if (m_active_buffer.enqueue(*req_it)) {
            buffer->remove(req_it);
          }
        }
      }
    }
  };

 private:
  static bool is_row_hit(IDRAM* dram, ReqBuffer::iterator& req) {
    return dram->check_rowbuffer_hit(req->final_command, req->addr_vec);
  }

  static bool is_row_open(IDRAM* dram, ReqBuffer::iterator& req) {
    return dram->check_node_open(req->final_command, req->addr_vec);
  }

  void update_request_stats(ReqBuffer::iterator& req) {
    req->is_stat_updated = true;

    if (req->type_id == Request::Type::Read) {
      if (is_row_hit(m_dram, req)) {
        s_read_row_hits++;
        s_row_hits++;
        if (req->source_id != -1) s_read_row_hits_per_core[req->source_id]++;
      } else if (is_row_open(m_dram, req)) {
        s_read_row_conflicts++;
        s_row_conflicts++;
        if (req->source_id != -1)
          s_read_row_conflicts_per_core[req->source_id]++;
      } else {
        s_read_row_misses++;
        s_row_misses++;
        if (req->source_id != -1) s_read_row_misses_per_core[req->source_id]++;
      }
    } else if (req->type_id == Request::Type::Write) {
      if (is_row_hit(m_dram, req)) {
        s_write_row_hits++;
        s_row_hits++;
      } else if (is_row_open(m_dram, req)) {
        s_write_row_conflicts++;
        s_row_conflicts++;
      } else {
        s_write_row_misses++;
        s_row_misses++;
      }
    }
  }

  void serve_completed_pending() {
    if (!pending.size()) return;
    auto& req = pending[0];
    if (req.depart > m_clk) return;

    if (req.type_id == Request::Type::Read) {
      if (req.depart - req.arrive > 1) {
        s_read_latency += req.depart - req.arrive;
      }
    }

    if (req.callback) {
      req.callback(req);
    }
    pending.pop_front();
  };

  void set_write_mode() {
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
    return used_access_banks.find(bank_key(req.addr_vec)) != used_access_banks.end();
  }

  ReqBuffer::iterator get_best_request_filtered(
      ReqBuffer& buffer,
      const std::unordered_set<std::string>& used_access_banks) {
    if (buffer.size() == 0) {
      return buffer.end();
    }

    for (auto& req : buffer) {
      req.command = m_dram->get_preq_command(req.final_command, req.addr_vec);
    }

    auto candidate = buffer.end();
    for (auto it = buffer.begin(); it != buffer.end(); ++it) {
      if (violates_access_bank_parallelism(*it, used_access_banks)) {
        continue;
      }
      if (candidate == buffer.end()) {
        candidate = it;
      } else {
        candidate = m_scheduler->compare(candidate, it);
      }
    }
    return candidate;
  }

  bool schedule_request_filtered(
      ReqBuffer::iterator& req_it, ReqBuffer*& req_buffer,
      const std::unordered_set<std::string>& used_access_banks) {
    bool request_found = false;

    if (req_it = get_best_request_filtered(m_active_buffer, used_access_banks);
        req_it != m_active_buffer.end()) {
      if (m_dram->check_ready(req_it->command, req_it->addr_vec)) {
        request_found = true;
        req_buffer = &m_active_buffer;
      }
    }

    if (!request_found) {
      if (m_priority_buffer.size() != 0) {
        req_buffer = &m_priority_buffer;
        req_it = m_priority_buffer.begin();
        req_it->command =
            m_dram->get_preq_command(req_it->final_command, req_it->addr_vec);

        request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
        if (!request_found & m_priority_buffer.size() != 0) {
          return false;
        }
      }

      if (!request_found) {
        set_write_mode();
        auto& buffer = m_is_write_mode ? m_write_buffer : m_read_buffer;
        if (req_it = get_best_request_filtered(buffer, used_access_banks);
            req_it != buffer.end()) {
          request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
          req_buffer = &buffer;
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

  void finalize() override {
    s_avg_read_latency = (float)s_read_latency / (float)s_num_read_reqs;

    s_queue_len_avg = (float)s_queue_len / (float)m_clk;
    s_read_queue_len_avg = (float)s_read_queue_len / (float)m_clk;
    s_write_queue_len_avg = (float)s_write_queue_len / (float)m_clk;
    s_priority_queue_len_avg = (float)s_priority_queue_len / (float)m_clk;
  }
};

}  // namespace Ramulator

