#include <vector>

#include "base/base.h"
#include "dram/dram.h"
#include "addr_mapper/addr_mapper.h"
#include "memory_system/memory_system.h"

namespace Ramulator {

class LinearMapperBase : public IAddrMapper {
  public:
    IDRAM* m_dram = nullptr;

    int m_num_levels = -1;          // How many levels in the hierarchy?
    std::vector<int> m_addr_bits;   // How many address bits for each level in the hierarchy?
    Addr_t m_tx_offset = -1;

    int m_col_bits_idx = -1;
    int m_row_bits_idx = -1;


  protected:
    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) {
      m_dram = memory_system->get_ifce<IDRAM>();

      // Populate m_addr_bits vector with the number of address bits for each level in the hierachy
      const auto& count = m_dram->m_organization.count;
      m_num_levels = count.size();
      m_addr_bits.resize(m_num_levels);
      for (size_t level = 0; level < m_addr_bits.size(); level++) {
        const int c = count[level];
        if (c <= 0) {
          throw std::runtime_error(fmt::format(
              "Invalid DRAM organization: org.{} must be positive (got {}).",
              m_dram->m_levels((int)level), c));
        }
        // Linear mappers bit-slice addresses by log2(count). Non-powers-of-two
        // would silently alias and break bank/row/col semantics.
        const uint64_t cc = (uint64_t)c;
        if ((cc & (cc - 1)) != 0) {
          throw std::runtime_error(fmt::format(
              "Invalid DRAM organization: org.{} must be a power of two for "
              "linear address mapping (got {}).",
              m_dram->m_levels((int)level), c));
        }
        m_addr_bits[level] = calc_log2(count[level]);
      }

      // Last (Column) address have the granularity of the prefetch size
      const int prefetch = m_dram->m_internal_prefetch_size;
      if (prefetch <= 0) {
        throw std::runtime_error("Invalid DRAM organization: prefetch_size must be positive.");
      }
      const uint64_t p = (uint64_t)prefetch;
      if ((p & (p - 1)) != 0) {
        throw std::runtime_error(
            "Invalid DRAM organization: prefetch_size must be a power of two for linear address mapping.");
      }
      m_addr_bits[m_num_levels - 1] -= calc_log2(m_dram->m_internal_prefetch_size);

      if (m_dram->m_channel_width <= 0 || (m_dram->m_channel_width % 8) != 0) {
        throw std::runtime_error(
            "Invalid DRAM organization: channel_width must be a positive multiple of 8 (bits).");
      }
      const uint64_t tx_bytes_u64 =
          (uint64_t)m_dram->m_internal_prefetch_size * (uint64_t)m_dram->m_channel_width / 8ull;
      if (tx_bytes_u64 == 0) {
        throw std::runtime_error("Invalid DRAM organization: tx_bytes must be positive.");
      }
      if ((tx_bytes_u64 & (tx_bytes_u64 - 1)) != 0) {
        throw std::runtime_error(
            "Invalid DRAM organization: tx_bytes (prefetch_size*channel_width/8) must be a power of two for linear address mapping.");
      }
      const int col_count = count.empty() ? 0 : count.back();
      if (col_count > 0 && (col_count % prefetch) != 0) {
        throw std::runtime_error(
            "Invalid DRAM organization: org.column must be a multiple of prefetch_size.");
      }

      int tx_bytes = (int)tx_bytes_u64;
      m_tx_offset = calc_log2(tx_bytes);

      // Determine where are the row and col bits for ChRaBaRoCo and RoBaRaCoCh
      try {
        m_row_bits_idx = m_dram->m_levels("row");
      } catch (const std::out_of_range& r) {
        throw std::runtime_error(fmt::format("Organization \"row\" not found in the spec, cannot use linear mapping!"));
      }

      // Assume column is always the last level
      m_col_bits_idx = m_num_levels - 1;
    }

};


class ChRaBaRoCo final : public LinearMapperBase, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IAddrMapper, ChRaBaRoCo, "ChRaBaRoCo", "Applies a trival mapping to the address.");

  public:
    void init() override { };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      LinearMapperBase::setup(frontend, memory_system);
    }

    void apply(Request& req) override {
      req.addr_vec.resize(m_num_levels, -1);
      Addr_t addr = req.addr >> m_tx_offset;
      for (int i = m_addr_bits.size() - 1; i >= 0; i--) {
        req.addr_vec[i] = slice_lower_bits(addr, m_addr_bits[i]);
      }
    }
};


class RoBaRaCoCh final : public LinearMapperBase, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IAddrMapper, RoBaRaCoCh, "RoBaRaCoCh", "Applies a RoBaRaCoCh mapping to the address.");

  public:
    void init() override { };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      LinearMapperBase::setup(frontend, memory_system);
    }

    void apply(Request& req) override {
      req.addr_vec.resize(m_num_levels, -1);
      Addr_t addr = req.addr >> m_tx_offset;
      req.addr_vec[0] = slice_lower_bits(addr, m_addr_bits[0]);
      req.addr_vec[m_addr_bits.size() - 1] = slice_lower_bits(addr, m_addr_bits[m_addr_bits.size() - 1]);
      for (int i = 1; i <= m_row_bits_idx; i++) {
        req.addr_vec[i] = slice_lower_bits(addr, m_addr_bits[i]);
      }
    }
};


class MOP4CLXOR final : public LinearMapperBase, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IAddrMapper, MOP4CLXOR, "MOP4CLXOR", "Applies a MOP4CLXOR mapping to the address.");

  public:
    void init() override { };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      LinearMapperBase::setup(frontend, memory_system);
    }

    void apply(Request& req) override {
      req.addr_vec.resize(m_num_levels, -1);
      Addr_t addr = req.addr >> m_tx_offset;
      req.addr_vec[m_col_bits_idx] = slice_lower_bits(addr, 2);
      for (int lvl = 0 ; lvl < m_row_bits_idx ; lvl++)
          req.addr_vec[lvl] = slice_lower_bits(addr, m_addr_bits[lvl]);
      req.addr_vec[m_col_bits_idx] += slice_lower_bits(addr, m_addr_bits[m_col_bits_idx]-2) << 2;
      req.addr_vec[m_row_bits_idx] = (int) addr;

      int row_xor_index = 0; 
      for (int lvl = 0 ; lvl < m_col_bits_idx ; lvl++){
        if (m_addr_bits[lvl] > 0){
          int mask = (req.addr_vec[m_col_bits_idx] >> row_xor_index) & ((1<<m_addr_bits[lvl])-1);
          req.addr_vec[lvl] = req.addr_vec[lvl] xor mask;
          row_xor_index += m_addr_bits[lvl];
        }
      }
    }
};

}   // namespace Ramulator
