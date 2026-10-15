"""Simulation-only API example; rendering is available through `terrain demo`."""
from pathlib import Path
from cybr_terrain import Config, Simulator, make_terrain

state=make_terrain(Config(preset="badlands",grid=65,duration=20))
report=Simulator(state).run()
assert report["passed"]
state.save(Path("outputs/api-example/state.npz"))
print(report)
