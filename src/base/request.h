#ifndef     RAMULATOR_BASE_REQUEST_H
#define     RAMULATOR_BASE_REQUEST_H

#include <array>
#include <cstdint>
#include <list>
#include <string>
#include <vector>

#include "base/base.h"

namespace Ramulator {

struct Request { 
  Addr_t    addr = -1;
  AddrVec_t addr_vec {};

  // Basic request id convention
  // 0 = Read, 1 = Write. The device spec defines all others
  struct Type {
    enum : int {
      Read = 0, 
      Write,
    };
  };

  int type_id = -1;    // An identifier for the type of the request
  int source_id = -1;  // An identifier for where the request is coming from (e.g., which core)

  // Optional semantic contract for tiered/3D backends.
  struct PathClass {
    enum : int {
      Unknown = 0,
      LocalAccess,
      CrossTierAccess,
      VerticalCopy,
    };
  };

  struct OpClass {
    enum : int {
      Unknown = 0,
      DemandRead,
      DemandWrite,
      BackgroundWriteback,
      ShadowSync,
    };
  };

  struct TargetKind {
    enum : int {
      Unknown = 0,
      Resident,
      Shadow,
    };
  };

  struct OrderingClass {
    enum : int {
      Unknown = 0,
      ProgramOrder,
      Relaxed,
    };
  };

  struct PlacementState {
    enum : int {
      Unknown = 0,
      SingleResident,
      MultiResident,
      ShadowResident,
    };
  };

  int path_class = PathClass::Unknown;  // Optional path semantic from the producer.
  int route_class = PathClass::Unknown; // V2 alias for path_class.
  uint32_t request_size_bytes = 0;      // Optional producer-visible payload size.
  int source_tier_hint = -1;            // Optional producer-visible source tier.
  int src_exec_tier_hint = -1;          // V2 alias for source_tier_hint.
  int destination_tier_hint = -1;       // Optional producer-visible destination tier.
  int dst_access_tier_hint = -1;        // V2 alias for destination_tier_hint.
  int tier_hint = -1;                   // Legacy alias for destination_tier_hint.
  int op_class = OpClass::Unknown;      // Producer-visible operation semantic.
  int target_kind = TargetKind::Unknown;
  int ordering_class = OrderingClass::Unknown;
  int home_tier_hint = -1;
  int resident_tier_hint = -1;
  uint64_t shadow_tiers_mask = 0;
  int placement_state = PlacementState::Unknown;
  uint64_t placement_epoch = 0;
  uint64_t data_version = 0;

  int command = -1;          // The command that need to be issued to progress the request
  int final_command = -1;    // The final command that is needed to finish the request
  bool is_stat_updated = false; // Memory controller stats

  Clk_t arrive = -1;   // Clock cycle when the request arrive at the memory controller
  Clk_t depart = -1;   // Clock cycle when the request depart the memory controller

  std::array<int, 12> scratchpad = { 0 };   // Scratchpad for scheduler/controller metadata

  std::function<void(Request&)> callback;

  void* m_payload = nullptr;    // Point to a generic payload

  Request(Addr_t addr, int type);
  Request(AddrVec_t addr_vec, int type);
  Request(Addr_t addr, int type, int source_id, std::function<void(Request&)> callback);
};


struct ReqBuffer {
  std::list<Request> buffer;
  size_t max_size = 32;


  using iterator = std::list<Request>::iterator;
  iterator begin() { return buffer.begin(); };
  iterator end() { return buffer.end(); };


  size_t size() const { return buffer.size(); }

  bool enqueue(const Request& request) {
    if (buffer.size() < max_size) {
      buffer.push_back(request);
      return true;
    } else {
      return false;
    }
  }

  void remove(iterator it) {
    buffer.erase(it);
  }
};

}        // namespace Ramulator


#endif   // RAMULATOR_BASE_REQUEST_H
