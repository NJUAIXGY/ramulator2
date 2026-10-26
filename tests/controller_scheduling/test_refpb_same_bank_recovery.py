"""Per-bank refresh must respect tRFCpb recovery on the same bank.

A bank that is executing a per-bank refresh is busy for nRFCpb. Issuing a
second REFpb to the same bank before recovery completes is physically
meaningless — the refresh hardware is still occupied. These tests enqueue
two REFpb requests to the same bank and require their issue spacing to be
>= nRFCpb on every HBM generation.
"""

import pytest

import ramulator
import tests.controller_scheduling.harness as cs

DRAMS = [
    ("hbm1", ramulator.dram.HBM1,
     dict(org_preset="HBM1_4Gb", timing_preset="HBM1_1Gbps"),
     cs.ControllerUnderTest.make_hbm12),
    ("hbm2", ramulator.dram.HBM2,
     dict(org_preset="HBM2_2Gb", timing_preset="HBM2_2000Mbps"),
     cs.ControllerUnderTest.make_hbm12),
    ("hbm3", ramulator.dram.HBM3,
     dict(org_preset="HBM3_8Gb_8hi", timing_preset="HBM3_6400Mbps"),
     cs.ControllerUnderTest.make_hbm34),
    ("hbm4", ramulator.dram.HBM4,
     dict(org_preset="HBM4_32Gb_8Hi", timing_preset="HBM4_8000Mbps"),
     cs.ControllerUnderTest.make_hbm34),
]


@pytest.mark.parametrize("name,cls,kwargs,make", DRAMS)
def test_same_bank_refpb_respects_nrfc_pb(name, cls, kwargs, make):
    dut = make(cls(**kwargs), refresh_manager=ramulator.refresh_manager.NoRefresh())
    addr = dut.addr_vec(
        **{k: 0 for k in ("PseudoChannel", "Sid", "BankGroup", "Bank") if k in dut.level_names},
        Row=0,
        Column=0,
    )
    dut.priority_send("REFpb", addr)
    for _ in range(5):
        dut.tick()
    dut.priority_send("REFpb", addr)
    for _ in range(2 * dut.timings["nRFCpb"]):
        dut.tick()

    refs = [it.clk for it in dut.history if it.command == "REFpb"]
    assert len(refs) == 2
    assert refs[1] - refs[0] >= dut.timings["nRFCpb"], (
        f"{name}: same-bank REFpb->REFpb gap {refs[1] - refs[0]} "
        f"< nRFCpb {dut.timings['nRFCpb']} — bank re-refreshed during recovery"
    )
