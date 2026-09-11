# Archived material

Historical code and project resources kept for reference and future restoration.
This directory is outside the installed Python package and normal code checks.

Paths generally mirror their former locations at the repository root:

| Location | Contents |
|----------|----------|
| `src/sim/` | Superseded trainers, policies, demos and XML snapshots |
| `src/main.py`, `src/monitor/`, `src/remote/` | Old host-side executables and tools |
| `src/relay/blesense/`, `src/relay/rpi/` | Embedded firmware and deployment scripts |
| `Tests/` | Hardware and networking experiments |
| `Playground/`, `zmq/` | Learning and messaging experiments |
| `scad/` | Mechanical design and CAD resources |
| `notes/` | Historical setup instructions and PDFs |
| `test_observation_pipeline.py`, `scripts/` | Superseded tests and debug scripts |
| `rpdev`, `screenlog.1` | Old development-session configuration and transcript |

Local, previously ignored material is preserved here too: `claude/` contains
earlier development notes, and ignored `local/` holds generated files and old
run output moved out of `src/` and the repository root. Existing root-level
`outputs/`, `plots/` and `video/` remain the current training outputs.

The sim/hardware boundary remains in the active tree: `src/riktigpatric/`,
`src/controller/` and `src/relay/conversions.py`. Archiving the old executables
does not remove the shared robot state, action definitions, backend interfaces,
servo control or wire format.

Archived scripts retain their historical assumptions and may refer to APIs,
relative paths or dependencies that no longer exist. They are reference material,
not supported entry points. Restore and update an individual component when it
is needed rather than adding this entire directory to the training import path.
