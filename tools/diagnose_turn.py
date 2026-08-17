"""Write compact all-surface diagnostics for the Milestone 2 control."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path

from sim.diagnostics import summarize_trace, write_trace_atomic
from sim.pivot_runner import direct_client_factory, load_simulation_settings, load_turn_parameters
from sim.surfaces import required_surfaces
from sim.turn_primitives import DiagonalUnloadPrimitive
from sim.turn_runner import run_primitive


def run_diagnostics(output_dir):
    output = Path(output_dir)
    parameters = load_turn_parameters("config/turn_right_baseline.json")
    settings = load_simulation_settings("config/simulation.json")
    summaries = {}
    for surface in required_surfaces():
        run_id = f"diagonal-unload-baseline-{surface.name}"
        run = run_primitive(DiagonalUnloadPrimitive(), parameters, surface, settings, direct_client_factory, run_id=run_id)
        if run.trace is None:
            raise RuntimeError(run.result.invalid_reason)
        write_trace_atomic(output / f"{run_id}.json", run.trace)
        summaries[surface.name] = {
            "result": asdict(run.result),
            "phases": [asdict(item) for item in summarize_trace(run.trace)],
        }
    output.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(summaries, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n"
    (output / "baseline-summary.json").write_text(payload, encoding="utf-8", newline="\n")
    return summaries


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", default="runs/sim/diagnostics")
    args = parser.parse_args(argv)
    summaries = run_diagnostics(args.output_dir)
    print(json.dumps(summaries, sort_keys=True, separators=(",", ":"), allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
