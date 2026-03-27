#include <cstdint>
#include <string>
#include <vector>

#include "base/exception.h"
#include "frontend/frontend.h"

namespace Ramulator {

namespace {

constexpr int kShmemTrafficClassScratchpadIdx = 4;

enum class ScriptTrafficClass : int {
  kForeground = 0,
  kBackground = 1,
  kShadow = 2,
};

int parse_path_class(const YAML::Node& n) {
  if (!n) return Request::PathClass::Unknown;
  try {
    const int raw = n.as<int>();
    switch (raw) {
      case Request::PathClass::LocalAccess:
      case Request::PathClass::CrossTierAccess:
      case Request::PathClass::VerticalCopy:
        return raw;
      default:
        return Request::PathClass::Unknown;
    }
  } catch (...) {
  }

  const std::string value = n.as<std::string>("");
  if (value == "local" || value == "local_access") {
    return Request::PathClass::LocalAccess;
  }
  if (value == "cross_tier" || value == "cross_tier_access") {
    return Request::PathClass::CrossTierAccess;
  }
  if (value == "vertical_copy") {
    return Request::PathClass::VerticalCopy;
  }
  return Request::PathClass::Unknown;
}

ScriptTrafficClass parse_traffic_class(const YAML::Node& n) {
  if (!n) return ScriptTrafficClass::kForeground;
  try {
    const int raw = n.as<int>();
    if (raw == static_cast<int>(ScriptTrafficClass::kBackground)) {
      return ScriptTrafficClass::kBackground;
    }
    if (raw == static_cast<int>(ScriptTrafficClass::kShadow)) {
      return ScriptTrafficClass::kShadow;
    }
    return ScriptTrafficClass::kForeground;
  } catch (...) {
  }

  const std::string value = n.as<std::string>("");
  if (value == "background") return ScriptTrafficClass::kBackground;
  if (value == "shadow") return ScriptTrafficClass::kShadow;
  return ScriptTrafficClass::kForeground;
}

int parse_op_class(const YAML::Node& n, bool is_write,
                   ScriptTrafficClass traffic_class) {
  if (n) {
    try {
      const int raw = n.as<int>();
      switch (raw) {
        case Request::OpClass::DemandRead:
        case Request::OpClass::DemandWrite:
        case Request::OpClass::BackgroundWriteback:
        case Request::OpClass::ShadowSync:
          return raw;
        default:
          break;
      }
    } catch (...) {
    }

    const std::string value = n.as<std::string>("");
    if (value == "demand_read") return Request::OpClass::DemandRead;
    if (value == "demand_write") return Request::OpClass::DemandWrite;
    if (value == "background_writeback") {
      return Request::OpClass::BackgroundWriteback;
    }
    if (value == "shadow_sync") return Request::OpClass::ShadowSync;
  }

  if (traffic_class == ScriptTrafficClass::kBackground) {
    return Request::OpClass::BackgroundWriteback;
  }
  if (traffic_class == ScriptTrafficClass::kShadow) {
    return Request::OpClass::ShadowSync;
  }
  return is_write ? Request::OpClass::DemandWrite
                  : Request::OpClass::DemandRead;
}

int parse_target_kind(const YAML::Node& n, int op_class) {
  if (n) {
    try {
      const int raw = n.as<int>();
      if (raw == Request::TargetKind::Resident ||
          raw == Request::TargetKind::Shadow) {
        return raw;
      }
    } catch (...) {
    }

    const std::string value = n.as<std::string>("");
    if (value == "resident" || value == "residential") {
      return Request::TargetKind::Resident;
    }
    if (value == "shadow") return Request::TargetKind::Shadow;
  }

  return op_class == Request::OpClass::ShadowSync
             ? Request::TargetKind::Shadow
             : Request::TargetKind::Resident;
}

int parse_ordering_class(const YAML::Node& n,
                         ScriptTrafficClass traffic_class) {
  if (n) {
    try {
      const int raw = n.as<int>();
      if (raw == Request::OrderingClass::ProgramOrder ||
          raw == Request::OrderingClass::Relaxed) {
        return raw;
      }
    } catch (...) {
    }

    const std::string value = n.as<std::string>("");
    if (value == "program_order") {
      return Request::OrderingClass::ProgramOrder;
    }
    if (value == "relaxed") return Request::OrderingClass::Relaxed;
  }

  return traffic_class == ScriptTrafficClass::kForeground
             ? Request::OrderingClass::ProgramOrder
             : Request::OrderingClass::Relaxed;
}

int parse_placement_state(const YAML::Node& n) {
  if (!n) return Request::PlacementState::SingleResident;
  try {
    const int raw = n.as<int>();
    switch (raw) {
      case Request::PlacementState::SingleResident:
      case Request::PlacementState::MultiResident:
      case Request::PlacementState::ShadowResident:
        return raw;
      default:
        return Request::PlacementState::SingleResident;
    }
  } catch (...) {
  }

  const std::string value = n.as<std::string>("");
  if (value == "multi_resident") return Request::PlacementState::MultiResident;
  if (value == "shadow_resident") return Request::PlacementState::ShadowResident;
  return Request::PlacementState::SingleResident;
}

}  // namespace

class Scripted : public IFrontEnd, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IFrontEnd, Scripted, "Scripted",
                                    "A minimal deterministic frontend for functional tests.")

 private:
  struct ScriptReq {
    bool is_write = false;
    Addr_t addr = 0;
    uint32_t size = 0;
    uint64_t at = 0;
    int source_id = -1;
    int route_class = Request::PathClass::Unknown;
    int src_exec_tier_hint = -1;
    int dst_access_tier_hint = -1;
    ScriptTrafficClass traffic_class = ScriptTrafficClass::kForeground;
    int op_class = Request::OpClass::Unknown;
    int target_kind = Request::TargetKind::Unknown;
    int ordering_class = Request::OrderingClass::Unknown;
    int home_tier_hint = -1;
    int resident_tier_hint = -1;
    uint64_t shadow_tiers_mask = 0;
    int placement_state = Request::PlacementState::SingleResident;
    uint64_t placement_epoch = 0;
    uint64_t data_version = 0;
  };

  enum class ExpectKind {
    kOrder,
    kSameDoneCycle,
    kDifferentDoneCycle,
    kMinDoneCycleGap,
    kMinDoneCycle
  };
  struct Expect {
    ExpectKind kind = ExpectKind::kOrder;
    size_t a = 0;
    size_t b = 0;
    uint64_t min_gap = 0;
    uint64_t min_cycle = 0;
  };

  std::vector<ScriptReq> m_script;
  std::vector<bool> m_sent;
  std::vector<uint64_t> m_issue_cycles;
  std::vector<uint64_t> m_done_cycles;
  std::vector<Expect> m_expectations;

  uint64_t m_cycle = 0;
  size_t m_next_idx = 0;
  size_t m_outstanding = 0;
  uint64_t m_max_cycles = 0;

  static uint64_t parse_u64(const YAML::Node& n, const char* what) {
    if (!n) {
      throw ConfigurationError("Missing field {} in Scripted frontend.", what);
    }
    try {
      return n.as<uint64_t>();
    } catch (...) {
    }
    const std::string s = n.as<std::string>("");
    if (s.empty()) {
      throw ConfigurationError("Invalid scalar for {} in Scripted frontend.", what);
    }
    if (s.compare(0, 2, "0x") == 0 || s.compare(0, 2, "0X") == 0) {
      return std::stoull(s.substr(2), nullptr, 16);
    }
    return std::stoull(s, nullptr, 10);
  }

  void on_complete(size_t idx, Request& req) {
    if (idx >= m_done_cycles.size()) return;
    if (m_done_cycles[idx] == UINT64_MAX) {
      m_done_cycles[idx] = (req.depart >= 0) ? (uint64_t)req.depart : m_cycle;
      if (m_outstanding > 0) m_outstanding--;
    }
  }

  void check_expectations_or_die() const {
    for (size_t i = 0; i < m_done_cycles.size(); ++i) {
      if (m_done_cycles[i] == UINT64_MAX) {
        throw ConfigurationError(
            "Scripted frontend: request {} did not complete (max_cycles={}, sent={}, cycle={}).",
            i, m_max_cycles, (i < m_sent.size() && m_sent[i]) ? 1 : 0, m_cycle);
      }
    }

    for (const auto& e : m_expectations) {
      if (e.a >= m_done_cycles.size() || e.b >= m_done_cycles.size()) {
        throw ConfigurationError("Scripted frontend: expectation index out of range.");
      }
      const uint64_t da = m_done_cycles[e.a];
      const uint64_t db = m_done_cycles[e.b];
      switch (e.kind) {
        case ExpectKind::kOrder:
          if (!(da < db)) {
            throw ConfigurationError(
                "Scripted frontend: expected done_cycle[{}] < done_cycle[{}] (got {} vs {}).",
                e.a, e.b, da, db);
          }
          break;
        case ExpectKind::kSameDoneCycle:
          if (!(da == db)) {
            throw ConfigurationError(
                "Scripted frontend: expected done_cycle[{}] == done_cycle[{}] (got {} vs {}).",
                e.a, e.b, da, db);
          }
          break;
        case ExpectKind::kDifferentDoneCycle:
          if (!(da != db)) {
            throw ConfigurationError(
                "Scripted frontend: expected done_cycle[{}] != done_cycle[{}] (got {} vs {}).",
                e.a, e.b, da, db);
          }
          break;
        case ExpectKind::kMinDoneCycleGap:
          if (!(db >= da + e.min_gap)) {
            throw ConfigurationError(
                "Scripted frontend: expected done_cycle[{}] + {} <= done_cycle[{}] (got {} vs {}).",
                e.a, e.min_gap, e.b, da, db);
          }
          break;
        case ExpectKind::kMinDoneCycle:
          if (!(da >= e.min_cycle)) {
            throw ConfigurationError(
                "Scripted frontend: expected done_cycle[{}] >= {} (got {}).",
                e.a, e.min_cycle, da);
          }
          break;
      }
    }
  }

 public:
  void init() override {
    m_clock_ratio = param<uint>("clock_ratio").default_val(1);
    m_max_cycles =
        param<uint64_t>("max_cycles").desc("Max frontend cycles before stop (0=until complete).").default_val(0);

    if (!m_config["script"] || !m_config["script"].IsSequence()) {
      throw ConfigurationError("Scripted frontend requires a YAML sequence at Frontend.script.");
    }
    for (auto it = m_config["script"].begin(); it != m_config["script"].end(); ++it) {
      const YAML::Node n = *it;
      const std::string type = n["type"].as<std::string>("");
      bool is_write = false;
      if (type == "read") {
        is_write = false;
      } else if (type == "write") {
        is_write = true;
      } else {
        throw ConfigurationError("Scripted frontend: invalid script.type \"{}\".", type);
      }
      const uint64_t addr_u64 = parse_u64(n["addr"], "addr");
      const uint32_t size = n["size"].as<uint32_t>(0);
      const uint64_t at = n["at"].as<uint64_t>(0);
      const int source_id = n["source_id"].as<int>(-1);
      const ScriptTrafficClass traffic_class = parse_traffic_class(n["traffic_class"]);
      const int route_class =
          n["route_class"] ? parse_path_class(n["route_class"])
                           : parse_path_class(n["path_class"]);
      const int src_exec_tier_hint =
          n["src_exec_tier_hint"].as<int>(n["source_tier_hint"].as<int>(-1));
      const int dst_access_tier_hint =
          n["dst_access_tier_hint"].as<int>(
              n["destination_tier_hint"].as<int>(n["tier_hint"].as<int>(-1)));
      const int op_class =
          parse_op_class(n["op_class"], is_write, traffic_class);
      const int target_kind = parse_target_kind(n["target_kind"], op_class);
      const int ordering_class =
          parse_ordering_class(n["ordering_class"], traffic_class);
      const int home_tier_hint =
          n["home_tier"].as<int>(n["home_tier_hint"].as<int>(-1));
      const int resident_tier_hint =
          n["resident_tier"].as<int>(n["resident_tier_hint"].as<int>(
              dst_access_tier_hint >= 0 ? dst_access_tier_hint : -1));
      const uint64_t shadow_tiers_mask =
          n["shadow_tiers_mask"]
              ? parse_u64(n["shadow_tiers_mask"], "shadow_tiers_mask")
              : 0;
      const int placement_state =
          parse_placement_state(n["placement_state"]);
      const uint64_t placement_epoch =
          n["placement_epoch"]
              ? parse_u64(n["placement_epoch"], "placement_epoch")
              : 0;
      const uint64_t data_version =
          n["data_version"] ? parse_u64(n["data_version"], "data_version") : 0;
      m_script.push_back(
          {is_write,
           (Addr_t)addr_u64,
           size,
           at,
           source_id,
           route_class,
           src_exec_tier_hint,
           dst_access_tier_hint,
           traffic_class,
           op_class,
           target_kind,
           ordering_class,
           home_tier_hint,
           resident_tier_hint,
           shadow_tiers_mask,
           placement_state,
           placement_epoch,
           data_version});
    }

    m_sent.assign(m_script.size(), false);
    m_issue_cycles.assign(m_script.size(), UINT64_MAX);
    m_done_cycles.assign(m_script.size(), UINT64_MAX);

    if (m_config["expectations"]) {
      const YAML::Node ex = m_config["expectations"];
      if (!ex.IsSequence()) {
        throw ConfigurationError("Scripted frontend: Frontend.expectations must be a sequence.");
      }
      for (auto it = ex.begin(); it != ex.end(); ++it) {
        const YAML::Node e = *it;
        const std::string kind = e["kind"].as<std::string>("");
        if (kind == "order") {
          m_expectations.push_back(
              {ExpectKind::kOrder, (size_t)e["before"].as<uint64_t>(),
               (size_t)e["after"].as<uint64_t>(), 0, 0});
        } else if (kind == "same_done_cycle") {
          m_expectations.push_back(
              {ExpectKind::kSameDoneCycle, (size_t)e["a"].as<uint64_t>(),
               (size_t)e["b"].as<uint64_t>(), 0, 0});
        } else if (kind == "different_done_cycle") {
          m_expectations.push_back(
              {ExpectKind::kDifferentDoneCycle, (size_t)e["a"].as<uint64_t>(),
               (size_t)e["b"].as<uint64_t>(), 0, 0});
        } else if (kind == "min_done_cycle_gap") {
          m_expectations.push_back(
              {ExpectKind::kMinDoneCycleGap, (size_t)e["before"].as<uint64_t>(),
               (size_t)e["after"].as<uint64_t>(),
               e["min_gap"].as<uint64_t>(), 0});
        } else if (kind == "min_done_cycle") {
          m_expectations.push_back(
              {ExpectKind::kMinDoneCycle, (size_t)e["req"].as<uint64_t>(), 0, 0,
               e["cycle"].as<uint64_t>()});
        } else {
          throw ConfigurationError("Scripted frontend: invalid expectations.kind \"{}\".", kind);
        }
      }
    }
  }

  void tick() override {
    while (m_next_idx < m_script.size() && m_cycle >= m_script[m_next_idx].at) {
      const size_t idx = m_next_idx;
      const ScriptReq& r = m_script[idx];
      auto cb = [this, idx](Request& req) { this->on_complete(idx, req); };
      Request req(r.addr,
                  r.is_write ? Request::Type::Write : Request::Type::Read,
                  r.source_id, cb);
      req.request_size_bytes = r.size;
      req.path_class = r.route_class;
      req.route_class = r.route_class;
      req.source_tier_hint = r.src_exec_tier_hint;
      req.src_exec_tier_hint = r.src_exec_tier_hint;
      req.destination_tier_hint = r.dst_access_tier_hint;
      req.dst_access_tier_hint = r.dst_access_tier_hint;
      req.tier_hint = req.destination_tier_hint;
      req.op_class = r.op_class;
      req.target_kind = r.target_kind;
      req.ordering_class = r.ordering_class;
      req.home_tier_hint =
          r.home_tier_hint >= 0 ? r.home_tier_hint : r.resident_tier_hint;
      req.resident_tier_hint = r.resident_tier_hint;
      req.shadow_tiers_mask = r.shadow_tiers_mask;
      req.placement_state = r.placement_state;
      req.placement_epoch = r.placement_epoch;
      req.data_version = r.data_version;
      req.scratchpad[kShmemTrafficClassScratchpadIdx] =
          static_cast<int>(r.traffic_class);
      bool ok = m_memory_system->send(req);
      if (!ok) {
        break;
      }
      m_sent[idx] = true;
      m_issue_cycles[idx] = m_cycle;
      m_outstanding++;
      m_next_idx++;
    }
    m_cycle++;
  }

  bool receive_external_requests(int req_type_id, Addr_t addr, int source_id,
                                 std::function<void(Request&)> callback) override {
    return m_memory_system->send({addr, req_type_id, source_id, callback});
  }

  bool is_finished() override {
    if (m_max_cycles != 0 && m_cycle >= m_max_cycles) {
      return true;
    }
    return (m_next_idx >= m_script.size()) && (m_outstanding == 0);
  }

  void finalize() override {
    check_expectations_or_die();
    IFrontEnd::finalize();
  }
};

// Force odr-use of the registration anchor so "Scripted" is always registered,
// even under link-time garbage collection or other aggressive optimizations.
namespace {
struct ScriptedRegAnchor : public Scripted {
  static bool force() { return Scripted::registered; }
};
volatile bool scripted_registered_anchor = ScriptedRegAnchor::force();
}  // namespace

}  // namespace Ramulator
