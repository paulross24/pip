"""Run the conservative Milestone 3 family matrix in PyBullet DIRECT mode."""

from __future__ import annotations

import argparse
from dataclasses import asdict, replace
from pathlib import Path

from sim.diagnostics import summarize_trace, write_trace_atomic
from sim.family_comparison import CandidateComparison, SurfaceComparison, comparison_json_bytes, rank_candidates, rank_families
from sim.pivot_runner import direct_client_factory, load_simulation_settings, load_turn_parameters
from sim.surfaces import required_surfaces
from sim.turn_primitives import (
    DiagonalUnloadPrimitive,
    DifferentialForeAftPrimitive,
    SameSideShearPrimitive,
    StagedPivotPrimitive,
)
from sim.turn_runner import run_primitive


def build_candidates(baseline):
    variants = (
        ("small", replace(baseline, unload_mm=3.0, tangential_mm=2.0, hold_s=0.25)),
        ("baseline", baseline),
        ("strong", replace(baseline, tangential_mm=4.0, hold_s=0.45)),
    )
    primitives = (
        DiagonalUnloadPrimitive(), SameSideShearPrimitive(),
        DifferentialForeAftPrimitive(), StagedPivotPrimitive(),
    )
    return tuple((primitive, identifier, parameters) for primitive in primitives for identifier, parameters in variants)


def comparison_markdown(candidates):
    lines = [
        "# Turn primitive family comparison", "", "## Family ranking", "",
        "| Rank | Primitive family | Promotable | Selected candidate |",
        "|---:|---|:---:|---|",
    ]
    for rank, family in enumerate(rank_families(candidates), 1):
        lines.append(
            f"| {rank} | {family.primitive_family} | {'yes' if family.promotable else 'no'} | "
            f"{family.selected_candidate_id or 'none'} |"
        )
    lines.extend([
        "", "## Candidate ranking", "",
        "| Rank | Primitive family | Candidate | Promotable | Low yaw | Nominal yaw | High yaw | Max translation | Fall |",
        "|---:|---|---|:---:|---:|---:|---:|---:|:---:|",
    ])
    for rank, candidate in enumerate(candidates, 1):
        surfaces = candidate.surfaces
        lines.append(
            f"| {rank} | {candidate.primitive_family} | {candidate.candidate_id} | "
            f"{'yes' if candidate.promotable else 'no'} | {surfaces[0].yaw_delta_deg:.6f} | "
            f"{surfaces[1].yaw_delta_deg:.6f} | {surfaces[2].yaw_delta_deg:.6f} | "
            f"{max(item.translation_m for item in surfaces):.6f} | "
            f"{'yes' if any(item.fell for item in surfaces) else 'no'} |"
        )
    return "\n".join(lines) + "\n"


def run_comparison(output_dir, *, trace_runs=True):
    baseline = load_turn_parameters("config/turn_right_baseline.json")
    settings = load_simulation_settings("config/simulation.json")
    output = Path(output_dir)
    results = []
    mechanism_phase = {
        "diagonal_unload": "DRIVE_TURN",
        "same_side_shear": "DRIVE_SHEAR",
        "differential_fore_aft": "DRIVE_FORCE_COUPLE",
        "staged_pivot": "REPOSITION_FL",
    }
    for primitive, candidate_id, parameters in build_candidates(baseline):
        surfaces = []
        for surface in required_surfaces():
            run_id = f"{primitive.family}-{candidate_id}-{surface.name}"
            run = run_primitive(
                primitive, parameters, surface, settings, direct_client_factory,
                run_id=run_id, candidate_id=candidate_id,
            )
            if run.trace is None:
                raise RuntimeError(f"{run_id} failed: {run.result.invalid_reason}")
            if trace_runs:
                write_trace_atomic(output / "diagnostics" / f"{run_id}.json", run.trace)
            summaries = summarize_trace(run.trace)
            selected = next(item for item in summaries if item.phase == mechanism_phase[primitive.family])
            selected_index = summaries.index(selected)
            cancellation = sum(max(0.0, -item.yaw_delta_deg) for item in summaries[selected_index + 1 :])
            cancellation_fraction = cancellation / selected.yaw_delta_deg if selected.yaw_delta_deg > 0.0 else 1.0
            totals = [sample.total_right_yaw_torque_nm for sample in run.trace.samples if sample.total_right_yaw_torque_nm is not None]
            slip = sum(foot.cumulative_slip_m for foot in run.trace.samples[-1].feet)
            surfaces.append(
                SurfaceComparison(
                    surface.name, run.result.yaw_delta_deg, run.result.translation_m,
                    run.result.max_roll_deviation_deg, run.result.max_pitch_deviation_deg,
                    run.result.contact_instability, max(totals, default=0.0),
                    min(totals, default=0.0), sum(totals), slip, run.result.fell,
                    selected.yaw_delta_deg > 0.0 and (selected.net_right_yaw_torque_nm or 0.0) > 0.0,
                    cancellation_fraction,
                )
            )
        results.append(CandidateComparison(primitive.family, candidate_id, asdict(parameters), tuple(surfaces)))
    ranked = rank_candidates(tuple(results))
    output.mkdir(parents=True, exist_ok=True)
    (output / "comparison.json").write_bytes(comparison_json_bytes(ranked))
    (output / "comparison.md").write_text(comparison_markdown(ranked), encoding="utf-8", newline="\n")
    return ranked


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", default="runs/sim/comparison/latest")
    args = parser.parse_args(argv)
    ranked = run_comparison(args.output_dir)
    print(comparison_markdown(ranked), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
