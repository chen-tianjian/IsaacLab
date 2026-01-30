# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Isaac Lab is a GPU-accelerated, open-source framework for robotics research workflows (RL, imitation learning, motion planning) built on NVIDIA Isaac Sim. It provides 16+ robot models, 30+ ready-to-train environments, and comprehensive sensor simulation.

**Key Technologies**: Python 3.11, PyTorch, Isaac Sim 5.1, PhysX physics engine

## Essential Development Commands

All development uses the `isaaclab.sh` script (Linux) or `isaaclab.bat` (Windows):

```bash
# Initial setup
uv venv --python 3.11 --seed            # Create uv environment
./isaaclab.sh -i all                    # Install all extensions and RL frameworks

# Daily development
source .venv/bin/activate               # Always activate environment before any other commands
./isaaclab.sh -f                        # Format and lint code (REQUIRED before commits)
./isaaclab.sh -t                        # Run all tests
./isaaclab.sh -p <script>               # Run Python script with Isaac Sim environment
./isaaclab.sh -s                        # Launch Isaac Sim simulator
./isaaclab.sh -d                        # Build documentation (output: docs/_build/current/)
./isaaclab.sh -v                        # Generate VS Code settings

# Run specific tests
./isaaclab.sh -p -m pytest <path>       # Run tests at specific path

# Training examples
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Reach-Franka-v0
./isaaclab.sh -p scripts/imitation_learning/robomimic/train.py --task Isaac-Lift-Cube-Franka-v0
```

## Code Quality Requirements

**CRITICAL**: Run `./isaaclab.sh -f` before every commit. This uses **Ruff** for formatting and linting:
- 120 char line length
- Max cyclomatic complexity: 30
- Google-style docstrings
- Import ordering (via ruff isort) with custom sections:
  1. Standard library
  2. Third-party
  3. Omniverse extensions (isaacsim, omni, pxr, carb, curobo)
  4. isaaclab (core)
  5. isaaclab_contrib, isaaclab_rl, isaaclab_mimic, isaaclab_tasks, isaaclab_assets
  6. Local folder

## Repository Structure

```
IsaacLab/
├── source/                              # Main source packages
│   ├── isaaclab/                       # Core framework
│   │   ├── envs/                       # Environment base classes
│   │   ├── managers/                   # MDP component managers
│   │   ├── sensors/                    # Sensor implementations
│   │   ├── assets/                     # Physical assets (robots, objects)
│   │   ├── actuators/                  # Actuator models
│   │   ├── controllers/                # Control algorithms
│   │   ├── scene/                      # Scene management
│   │   └── sim/                        # Simulation context
│   ├── isaaclab_assets/                # Robot and asset definitions
│   ├── isaaclab_tasks/                 # Task/environment implementations
│   ├── isaaclab_rl/                    # RL framework integrations
│   └── isaaclab_mimic/                 # Imitation learning (Apache 2.0)
├── scripts/                            # Example training/testing scripts
│   ├── reinforcement_learning/         # RL training examples
│   ├── imitation_learning/             # Imitation learning examples
│   └── tutorials/                      # Tutorial scripts
└── tools/                              # Development utilities
```

## Core Architecture

### Manager-Based Environment System

Isaac Lab uses a **manager + term** pattern for composable MDP components:

**Key Managers**:
- `ActionManager` - Processes agent actions → actuator commands
- `ObservationManager` - Collects observations from sensors/state
- `RewardManager` - Computes reward signals (RL environments)
- `TerminationManager` - Determines episode termination
- `EventManager` - Handles reset and randomization events
- `CommandManager` - Generates target commands/goals
- `CurriculumManager` - Manages curriculum learning
- `RecorderManager` - Records trajectory data

Each manager contains **terms** (functions or classes) that define specific behaviors. Terms are registered in configuration classes using `@configclass` decorator.

### Environment Types

1. **ManagerBasedEnv** - Uses managers for all MDP components (recommended for new environments)
2. **DirectRLEnv** - Lower-level interface for direct control without managers

### Configuration System

All components use `@configclass` decorator (custom dataclass wrapper):
- Required parameters: `param: type = MISSING`
- Optional with defaults: `param: type = value`
- Nested configs supported
- Methods: `.to_dict()`, `.from_dict()`, `.copy()`, `.replace()`

### Base Class Hierarchy

**Assets** (physical entities):
- `AssetBase` → `Articulation`, `RigidObject`, `DeformableObject`
- Spawned from configs with `spawn` field (URDF, USD, primitives)

**Sensors** (lazy evaluation):
- `SensorBase` → `Camera`, `RayCaster`, `ContactSensor`, `IMU`, `FrameTransformer`
- Data only computed when accessed via `.data` property
- Configurable update periods to reduce computation

**Actuators** (joint dynamics):
- `ActuatorBase` → `ImplicitActuator` (physics engine), explicit models (DC motor, etc.)
- Per-joint configuration via regex patterns

### Scene Management

`InteractiveScene` is the central registry:
- Parses `InteractiveSceneCfg` to create all entities
- Access entities: `scene["entity_name"]` or `scene.articulations["robot"]`
- Handles environment cloning (vectorized envs)

### Extensibility Pattern

Three ways to extend environments:

1. **Term Functions** (stateless):
```python
def my_observation_term(env: ManagerBasedEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    robot = env.scene[asset_cfg.name]
    return robot.data.joint_pos
```

2. **Term Classes** (stateful):
```python
class MyObservationTerm(ManagerTermBase):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        # Initialize state

    def __call__(self) -> torch.Tensor:
        # Compute observation
        pass
```

3. **Custom Base Classes** (deep integration):
- Inherit from `AssetBase`, `SensorBase`, `ActuatorBase`, or `ManagerBase`

## Testing

Tests mirror source structure:
```
source/isaaclab/test/
├── assets/              # Asset tests
├── sensors/             # Sensor tests
├── managers/            # Manager tests
├── controllers/         # Controller tests
└── performance/         # Performance benchmarks
```

Run single test: `./isaaclab.sh -p -m pytest source/isaaclab/test/sensors/test_camera.py`

## Important Patterns

### Vectorized Environments
- All environments are vectorized (batch operations)
- `num_envs` parameter controls parallel environment count
- GPU-accelerated via PyTorch tensors
- Single `reset()` at start, terminated envs reset during `step()`

### Lazy Sensor Evaluation
```python
# Sensor data only computed when accessed
camera = scene["camera"]
rgb = camera.data.output["rgb"]  # Triggers computation if needed
```

### SceneEntityCfg (Dynamic References)
```python
# Reference scene entities with filtering
ObservationTermCfg(
    func=my_func,
    params={
        "asset_cfg": SceneEntityCfg(
            name="robot",
            joint_names=[".*"],      # Regex: all joints
            body_names=["base"]      # Specific bodies
        )
    }
)
```

### Lifecycle Events
- Components register callbacks on Isaac Sim timeline events
- **PLAY** (order=10): Assets/sensors initialize PhysX handles
- **PLAY** (order=20): Managers resolve scene entity references
- **STOP**: Cleanup and invalidation

## Key Environment Variables

Automatically set by `isaaclab.sh`:
- `ISAACLAB_PATH` - Root path of Isaac Lab
- `PYTHONPATH` - Extended with all source modules
- `RESOURCE_NAME` - "IsaacSim" (for icon display)

## Licensing

- **Core framework**: BSD-3-Clause
- **isaaclab_mimic extension**: Apache 2.0
- **Isaac Sim dependency**: Proprietary (see docs/licenses/)

## RL Framework Integrations

Supported frameworks (via `isaaclab_rl`):
- RSL RL (default, lightweight)
- SKRL (advanced features)
- RL Games (highly optimized)
- Stable Baselines 3 (standard interface)

Install specific framework: `./isaaclab.sh -i rsl_rl`

## Common Pitfalls

1. **Always read before editing**: Use Read tool on files before making changes
2. **Environment cloning**: Understand replicated vs heterogeneous physics
3. **Sensor update periods**: Set appropriately to balance accuracy vs performance
4. **Actuator limits**: Actuator effort limits separate from simulation limits
5. **Reset behavior**: Only call `reset()` once at start, not per environment
6. **GPU memory**: Monitor memory usage with vectorized environments

## Contribution Checklist

Before creating PR:
1. Run `./isaaclab.sh --format` (mandatory)
2. Run tests: `./isaaclab.sh -t`
3. Update documentation if needed
4. Sign commits (Developer Certificate of Origin required)
5. Keep PRs focused and small
6. Update CHANGELOG and version in `config/extension.toml`
7. Add name to `CONTRIBUTORS.md`

## Documentation

- Online docs: https://isaac-sim.github.io/IsaacLab
- Build locally: `./isaaclab.sh -d` → `docs/_build/current/index.html`
- Format: reStructuredText (.rst) and Markdown, built with Sphinx

## Support Resources

- **GitHub Discussions**: Questions, feature requests, show & tell
- **GitHub Issues**: Executable work items (bugs, features, updates)
- **Isaac Sim Forums**: https://forums.developer.nvidia.com/c/agx-autonomous-machines/isaac/67
- **ArXiv Paper**: https://arxiv.org/abs/2511.04831
