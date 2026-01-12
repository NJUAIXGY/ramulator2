#include "base/utils.h"
#include "dram/dram.h"
#include "dram/lambdas.h"

#include <map>
#include <string>
#include <vector>

namespace Ramulator {

// Mono3D: a minimal DRAM device model intended for SM-attached 3D-stacked
// shared-memory (v1). It focuses on row-buffer behavior and bank-level timing,
// while allowing "layer -> channel" mapping via org.channel.
class Mono3D : public IDRAM, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IDRAM, Mono3D, "Mono3D",
                                   "Mono3D (3D-stacked) DRAM Device Model");

 public:
  // Minimal preset: users are expected to override org/timing with Mono3D params.
  inline static const std::map<std::string, Organization> org_presets = {
      //    name              density   dq     Ch  Ra  Bg  Ba   Ro     Co
      {"Mono3D_DefaultOrg", {0,        256, {1, 1, 1, 8, 1 << 10, 1 << 8}}},
  };

  // Minimal timing preset (cycles). Override in YAML with Mono3D-specific values.
  inline static const std::map<std::string, std::vector<int>> timing_presets = {
      // name               rate  nBL nCL nRCD nRP nRAS nRC nWR nRTP nCWL nWTR
      //                    nRTW nCCDS nRRDS nFAW tCK_ps
      {"Mono3D_Default",
       {2000, 1, 10, 10, 10, 24, 34, 10, 5, 8, 5, 5, 1, 4, 16, 1000}},
  };

  /************************************************
   *                Organization
   ***********************************************/
  int m_internal_prefetch_size = 1;

  inline static constexpr ImplDef m_levels = {
      "channel", "rank", "bankgroup", "bank", "row", "column",
  };

  /************************************************
   *             Requests & Commands
   ***********************************************/
  inline static constexpr ImplDef m_commands = {
      "ACT",
      "PRE",
      "RD",
      "WR",
      "RDA",
      "WRA",
  };

  inline static const ImplLUT m_command_scopes =
      LUT(m_commands, m_levels, {
                                  {"ACT", "row"},
                                  {"PRE", "bank"},
                                  {"RD", "column"},
                                  {"WR", "column"},
                                  {"RDA", "column"},
                                  {"WRA", "column"},
                              });

  inline static const ImplLUT m_command_meta = LUT<DRAMCommandMeta>(
      m_commands, {
                      // open? close? access? refresh?
                      {"ACT", {true, false, false, false}},
                      {"PRE", {false, true, false, false}},
                      {"RD", {false, false, true, false}},
                      {"WR", {false, false, true, false}},
                      {"RDA", {false, true, true, false}},
                      {"WRA", {false, true, true, false}},
                  });

  inline static constexpr ImplDef m_requests = {
      "read",
      "write",
      "open-row",
      "close-row",
  };

  inline static const ImplLUT m_request_translations =
      LUT(m_requests, m_commands, {
                                     {"read", "RD"},
                                     {"write", "WR"},
                                     {"open-row", "ACT"},
                                     {"close-row", "PRE"},
                                 });

  /************************************************
   *                   Timing
   ***********************************************/
  inline static constexpr ImplDef m_timings = {
      "rate",   // MT/s (optional)
      "nBL",    // burst length (cycles)
      "nCL",    // read CAS latency (cycles)
      "nRCD",   // ACT -> RD/WR (cycles)
      "nRP",    // PRE -> ACT (cycles)
      "nRAS",   // ACT -> PRE (cycles)
      "nRC",    // ACT -> ACT (same bank) (cycles)
      "nWR",    // write recovery (cycles)
      "nRTP",   // RD -> PRE (cycles)
      "nCWL",   // write CAS latency (cycles)
      "nWTR",   // WR -> RD (cycles)
      "nRTW",   // RD -> WR (cycles)
      "nCCDS",  // CAS -> CAS (same channel) (cycles)
      "nRRDS",  // ACT -> ACT (diff banks) (cycles)
      "nFAW",   // 4-ACT window (cycles)
      "tCK_ps",
  };

  /************************************************
   *                 Node States
   ***********************************************/
  inline static constexpr ImplDef m_states = {
      "Opened",
      "Closed",
      "N/A",
      "Refreshing",
  };

  inline static const ImplLUT m_init_states =
      LUT(m_levels, m_states, {
                                 {"channel", "N/A"},
                                 {"rank", "N/A"},
                                 {"bankgroup", "N/A"},
                                 {"bank", "Closed"},
                                 {"row", "Closed"},
                                 {"column", "N/A"},
                             });

 public:
  struct Node : public DRAMNodeBase<Mono3D> {
    Node(Mono3D* dram, Node* parent, int level, int id)
        : DRAMNodeBase<Mono3D>(dram, parent, level, id) {}
  };

  std::vector<Node*> m_channels;

  FuncMatrix<ActionFunc_t<Node>> m_actions;
  FuncMatrix<PreqFunc_t<Node>> m_preqs;
  FuncMatrix<RowhitFunc_t<Node>> m_rowhits;
  FuncMatrix<RowopenFunc_t<Node>> m_rowopens;

 public:
  void tick() override { m_clk++; }

  void init() override {
    RAMULATOR_DECLARE_SPECS();
    set_organization();
    set_timing_vals();

    set_actions();
    set_preqs();
    set_rowhits();
    set_rowopens();

    create_nodes();
  }

  void issue_command(int command, const AddrVec_t& addr_vec) override {
    int channel_id = addr_vec[m_levels["channel"]];
    m_channels[channel_id]->update_timing(command, addr_vec, m_clk);
    m_channels[channel_id]->update_states(command, addr_vec, m_clk);
  }

  int get_preq_command(int command, const AddrVec_t& addr_vec) override {
    int channel_id = addr_vec[m_levels["channel"]];
    return m_channels[channel_id]->get_preq_command(command, addr_vec, m_clk);
  }

  bool check_ready(int command, const AddrVec_t& addr_vec) override {
    int channel_id = addr_vec[m_levels["channel"]];
    return m_channels[channel_id]->check_ready(command, addr_vec, m_clk);
  }

  bool check_rowbuffer_hit(int command, const AddrVec_t& addr_vec) override {
    int channel_id = addr_vec[m_levels["channel"]];
    return m_channels[channel_id]->check_rowbuffer_hit(command, addr_vec, m_clk);
  }

  bool check_node_open(int command, const AddrVec_t& addr_vec) override {
    int channel_id = addr_vec[m_levels["channel"]];
    return m_channels[channel_id]->check_node_open(command, addr_vec, m_clk);
  }

 private:
  void set_organization() {
    m_channel_width =
        param_group("org").param<int>("channel_width").default_val(256);

    if (auto p =
            param_group("org").param<int>("prefetch_size").optional()) {
      m_internal_prefetch_size = *p;
    } else {
      m_internal_prefetch_size = 1;
    }

    m_organization.count.resize(m_levels.size(), -1);

    if (auto preset_name =
            param_group("org").param<std::string>("preset").optional()) {
      if (org_presets.count(*preset_name) > 0) {
        m_organization = org_presets.at(*preset_name);
      } else {
        throw ConfigurationError("Unrecognized organization preset \"{}\" in {}!",
                                 *preset_name, get_name());
      }
    }

    if (auto dq = param_group("org").param<int>("dq").optional()) {
      m_organization.dq = *dq;
    }

    for (int i = 0; i < m_levels.size(); i++) {
      auto level_name = m_levels(i);
      if (auto sz = param_group("org").param<int>(level_name).optional()) {
        m_organization.count[i] = *sz;
      }
    }

    if (auto density = param_group("org").param<int>("density").optional()) {
      m_organization.density = *density;
    }

    for (int i = 0; i < m_levels.size(); i++) {
      if (m_organization.count[i] == -1) {
        throw ConfigurationError("In \"{}\", organization {} is not specified!",
                                 get_name(), m_levels(i));
      }
    }

    // If density is not specified (0), derive a per-channel density in Mb.
    size_t calc_density_bits = size_t(m_organization.count[m_levels["rank"]]) *
                               size_t(m_organization.count[m_levels["bankgroup"]]) *
                               size_t(m_organization.count[m_levels["bank"]]) *
                               size_t(m_organization.count[m_levels["row"]]) *
                               size_t(m_organization.count[m_levels["column"]]) *
                               size_t(m_organization.dq);
    size_t calc_density_mb = calc_density_bits >> 20;

    if (m_organization.density <= 0) {
      m_organization.density = static_cast<int>(calc_density_mb);
    } else if (static_cast<size_t>(m_organization.density) != calc_density_mb) {
      throw ConfigurationError(
          "Calculated {} channel density {} Mb does not equal the provided density {} Mb!",
          get_name(), calc_density_mb, m_organization.density);
    }
  }

  void set_timing_vals() {
    m_timing_vals.resize(m_timings.size(), -1);

    bool preset_provided = false;
    if (auto preset_name =
            param_group("timing").param<std::string>("preset").optional()) {
      if (timing_presets.count(*preset_name) > 0) {
        m_timing_vals = timing_presets.at(*preset_name);
        preset_provided = true;
      } else {
        throw ConfigurationError("Unrecognized timing preset \"{}\" in {}!",
                                 *preset_name, get_name());
      }
    }

    // Optional: use rate to derive tCK (ps). Users can also directly override tCK_ps.
    if (auto rate = param_group("timing").param<int>("rate").optional()) {
      if (preset_provided) {
        throw ConfigurationError(
            "Cannot change the transfer rate of {} when using a timing preset!",
            get_name());
      }
      m_timing_vals("rate") = *rate;
    }

    if (m_timing_vals("rate") != -1) {
      int tCK_ps = 1E6 / (m_timing_vals("rate") / 2);
      m_timing_vals("tCK_ps") = tCK_ps;
    }

    // Overwrite timing parameters with any user-provided value
    // rate and tCK_ps are handled specially above.
    int tCK_ps = m_timing_vals("tCK_ps");
    for (int i = 1; i < m_timings.size() - 1; i++) {
      auto timing_name = std::string(m_timings(i));

      if (auto provided_cycles =
              param_group("timing").param<int>(timing_name).optional()) {
        m_timing_vals(i) = *provided_cycles;
      } else if (tCK_ps != -1) {
        // e.g., nRCD -> tRCD in ns
        std::string tname = timing_name;
        tname.replace(0, 1, "t");
        if (auto provided_ns =
                param_group("timing").param<float>(tname).optional()) {
          m_timing_vals(i) = static_cast<int>(JEDEC_rounding(*provided_ns, tCK_ps));
        }
      }
    }

    if (auto provided_tck_ps =
            param_group("timing").param<int>("tCK_ps").optional()) {
      m_timing_vals("tCK_ps") = *provided_tck_ps;
    }

    // Sanity: ensure all timing values are set.
    for (int i = 0; i < m_timing_vals.size(); i++) {
      if (m_timing_vals(i) == -1) {
        throw ConfigurationError("In \"{}\", timing {} is not specified!",
                                 get_name(), m_timings(i));
      }
    }

    // Read latency (cycles): RD -> data
    m_read_latency = m_timing_vals("nCL") + m_timing_vals("nBL");

    // Timing constraints
#define V(name) (m_timing_vals(name))
    populate_timingcons(this, {
                                  /*** Channel (layer) ***/
                                  // ACT spacing (different banks)
                                  {.level = "channel",
                                   .preceding = {"ACT"},
                                   .following = {"ACT"},
                                   .latency = V("nRRDS")},
                                  // 4-ACT window
                                  {.level = "channel",
                                   .preceding = {"ACT"},
                                   .following = {"ACT"},
                                   .latency = V("nFAW"),
                                   .window = 4},

                                  // Data bus occupancy
                                  {.level = "channel",
                                   .preceding = {"RD", "RDA"},
                                   .following = {"RD", "RDA"},
                                   .latency = V("nBL")},
                                  {.level = "channel",
                                   .preceding = {"WR", "WRA"},
                                   .following = {"WR", "WRA"},
                                   .latency = V("nBL")},

                                  // CAS-to-CAS minimum gap
                                  {.level = "channel",
                                   .preceding = {"RD", "RDA"},
                                   .following = {"RD", "RDA"},
                                   .latency = V("nCCDS")},
                                  {.level = "channel",
                                   .preceding = {"WR", "WRA"},
                                   .following = {"WR", "WRA"},
                                   .latency = V("nCCDS")},

                                  // Read<->Write turnarounds (simplified, directly parameterized)
                                  {.level = "channel",
                                   .preceding = {"RD", "RDA"},
                                   .following = {"WR", "WRA"},
                                   .latency = V("nRTW")},
                                  {.level = "channel",
                                   .preceding = {"WR", "WRA"},
                                   .following = {"RD", "RDA"},
                                   .latency = V("nWTR")},

                                  /*** Bank ***/
                                  {.level = "bank",
                                   .preceding = {"ACT"},
                                   .following = {"ACT"},
                                   .latency = V("nRC")},
                                  {.level = "bank",
                                   .preceding = {"ACT"},
                                   .following = {"RD", "RDA", "WR", "WRA"},
                                   .latency = V("nRCD")},
                                  {.level = "bank",
                                   .preceding = {"ACT"},
                                   .following = {"PRE"},
                                   .latency = V("nRAS")},
                                  {.level = "bank",
                                   .preceding = {"PRE"},
                                   .following = {"ACT"},
                                   .latency = V("nRP")},
                                  {.level = "bank",
                                   .preceding = {"RD"},
                                   .following = {"PRE"},
                                   .latency = V("nRTP")},
                                  {.level = "bank",
                                   .preceding = {"WR"},
                                   .following = {"PRE"},
                                   .latency = V("nCWL") + V("nBL") + V("nWR")},
                                  {.level = "bank",
                                   .preceding = {"RDA"},
                                   .following = {"ACT"},
                                   .latency = V("nRTP") + V("nRP")},
                                  {.level = "bank",
                                   .preceding = {"WRA"},
                                   .following = {"ACT"},
                                   .latency = V("nCWL") + V("nBL") + V("nWR") +
                                              V("nRP")},
                              });
#undef V
  }

  void set_actions() {
    m_actions.resize(m_levels.size(),
                     std::vector<ActionFunc_t<Node>>(m_commands.size()));

    m_actions[m_levels["bank"]][m_commands["ACT"]] =
        Lambdas::Action::Bank::ACT<Mono3D>;
    m_actions[m_levels["bank"]][m_commands["PRE"]] =
        Lambdas::Action::Bank::PRE<Mono3D>;
    m_actions[m_levels["bank"]][m_commands["RDA"]] =
        Lambdas::Action::Bank::PRE<Mono3D>;
    m_actions[m_levels["bank"]][m_commands["WRA"]] =
        Lambdas::Action::Bank::PRE<Mono3D>;
  }

  void set_preqs() {
    m_preqs.resize(m_levels.size(),
                   std::vector<PreqFunc_t<Node>>(m_commands.size()));

    m_preqs[m_levels["bank"]][m_commands["RD"]] =
        Lambdas::Preq::Bank::RequireRowOpen<Mono3D>;
    m_preqs[m_levels["bank"]][m_commands["WR"]] =
        Lambdas::Preq::Bank::RequireRowOpen<Mono3D>;
    m_preqs[m_levels["bank"]][m_commands["RDA"]] =
        Lambdas::Preq::Bank::RequireRowOpen<Mono3D>;
    m_preqs[m_levels["bank"]][m_commands["WRA"]] =
        Lambdas::Preq::Bank::RequireRowOpen<Mono3D>;
  }

  void set_rowhits() {
    m_rowhits.resize(m_levels.size(),
                     std::vector<RowhitFunc_t<Node>>(m_commands.size()));
    m_rowhits[m_levels["bank"]][m_commands["RD"]] =
        Lambdas::RowHit::Bank::RDWR<Mono3D>;
    m_rowhits[m_levels["bank"]][m_commands["WR"]] =
        Lambdas::RowHit::Bank::RDWR<Mono3D>;
  }

  void set_rowopens() {
    m_rowopens.resize(m_levels.size(),
                      std::vector<RowopenFunc_t<Node>>(m_commands.size()));
    m_rowopens[m_levels["bank"]][m_commands["RD"]] =
        Lambdas::RowOpen::Bank::RDWR<Mono3D>;
    m_rowopens[m_levels["bank"]][m_commands["WR"]] =
        Lambdas::RowOpen::Bank::RDWR<Mono3D>;
  }

  void create_nodes() {
    int num_channels = m_organization.count[m_levels["channel"]];
    for (int i = 0; i < num_channels; i++) {
      Node* channel = new Node(this, nullptr, 0, i);
      m_channels.push_back(channel);
    }
  }
};

}  // namespace Ramulator

