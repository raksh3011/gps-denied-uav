# Testing

Four layers, all must pass before a module is DONE (see [DEFINITION_OF_DONE](../README.md#definition-of-done)).

## 1. Unit tests — `tests/unit/`

Pure logic, no ROS runtime required beyond message construction. Fast, run constantly while developing.

```bash
pytest tests/unit -v
```

## 2. Contract tests — `tests/contract/`

Verify a module honors the interface contract in [docs/INTERFACES.md](INTERFACES.md):

- Message-level (`test_message_contracts.py`): fields, frames, enums, default-invalid behavior.
- Node-level (`test_node_contracts.py`): real mock nodes over real topics — e.g. Safety rejects stale/invalid input and holds; Safety forwards a valid `VehicleCommand` once inputs are healthy; Vehicle rejects `valid=False` commands.

```bash
pytest tests/contract -v
```

A module is not considered complete until its contract tests pass — this applies to real implementations too, not just mocks. When you replace `MockPlanner` with a real planner, the same contract tests must still pass unmodified (only the node under test changes).

## 3. Integration tests — `tests/integration/`

Brings up multiple/all nodes together and checks the whole loop, e.g. `test_mock_pipeline.py` proves Mission -> Planner -> Safety -> Vehicle produces a valid `VehicleCommand` end-to-end.

```bash
pytest tests/integration -v
```

## 4. Golden scenario (simulation regression)

One deterministic Gazebo/PX4 SITL scenario (`simulation/scenarios/`) — start, static obstacles, boundary, navigate to target, return, land. This is the baseline regression test added once real Planning/Safety exist; the mocked pipeline test above is its Milestone-1 stand-in.

## Running everything (what CI runs)

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --symlink-install
pytest tests/ -v
```

## Writing new tests

- Contract tests belong with the interface they validate, in `tests/contract/`, not inside a module package — they must be able to fail independent of which side (producer or consumer) is wrong.
- Prefer testing against real mock nodes over topics (as in `test_node_contracts.py`) rather than calling internal functions directly — this catches QoS/serialization issues that a plain function call would miss.
- Every new `.msg` field that has a validity condition (see [CONVENTIONS.md](CONVENTIONS.md#error--status-conventions)) needs a contract test asserting consumers respect it.
