"""Every bank's actual refresh period must stay within tREFI.

On JEDEC multi-pseudochannel devices, nREFIpb is the per-pseudochannel
refresh interval: each pseudochannel must receive one REFpb every
nREFIpb so that a bank's real refresh period is
(banks per pseudochannel) x nREFIpb <= nREFI. A refresh manager that
walks all channel banks at that interval serves each pseudochannel at
half the required rate (e.g. HBM2: 32 x 243 = 7776 > nREFI 3900).
These tests measure the issued REFpb stream with no demand traffic and
assert every bank stays within tREFI.
"""

import pytest

import ramulator
import tests.controller_scheduling.harness as cs

DRAMS = [
    ("hbm2", ramulator.dram.HBM2,
     dict(org_preset="HBM2_2Gb", timing_preset="HBM2_2000Mbps"),
     cs.ControllerUnderTest.make_hbm12, ramulator.refresh_manager.PerBank),
    ("hbm3", ramulator.dram.HBM3,
     dict(org_preset="HBM3_8Gb_8hi", timing_preset="HBM3_6400Mbps"),
     cs.ControllerUnderTest.make_hbm34, ramulator.refresh_manager.HBM34PerBankRefresh),
    ("hbm4", ramulator.dram.HBM4,
     dict(org_preset="HBM4_32Gb_8Hi", timing_preset="HBM4_8000Mbps"),
     cs.ControllerUnderTest.make_hbm34, ramulator.refresh_manager.HBM34PerBankRefresh),
]

IDX = ("PseudoChannel", "Sid", "BankGroup", "Bank")


@pytest.mark.parametrize("name,cls,kwargs,make,manager", DRAMS)
def test_per_bank_refresh_period_within_trefi(name, cls, kwargs, make, manager):
    dut = make(cls(**kwargs), refresh_manager=manager())
    nrefi = dut.timings["nREFI"]
    # Tick long enough for >= 3 full rounds of every bank.
    horizon = int(nrefi * 3.5) + 2048
    for _ in range(horizon):
        dut.tick()

    idx = [dut.level_names.index(k) for k in IDX if k in dut.level_names]
    per_bank = {}
    for it in dut.history:
        if it.command == "REFpb":
            key = tuple(it.addr_vec[i] for i in idx)
            per_bank.setdefault(key, []).append(it.clk)

    total_banks = dut.org["pseudochannel"] * dut.org.get("sid", 1) \
        * dut.org["bankgroup"] * dut.org["bank"]
    assert len(per_bank) == total_banks, (
        f"{name}: {len(per_bank)}/{total_banks} banks refreshed in {horizon} ticks"
    )
    worst = max(
        c2 - c1 for clks in per_bank.values() for c1, c2 in zip(clks, clks[1:])
    )
    assert worst <= nrefi, (
        f"{name}: slowest per-bank refresh period {worst} ticks > "
        f"nREFI {nrefi} — bank misses its refresh deadline"
    )
