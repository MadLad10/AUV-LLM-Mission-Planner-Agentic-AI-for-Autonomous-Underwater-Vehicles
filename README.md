# Duburi LLM Mission Planner

Plain-language missions for **BRACU Duburi**, an autonomous underwater vehicle (AUV).
You type a mission:

```
mission "pass through the gate, touch the red buoy, then surface; abort if battery is below 20%"
```

An LLM turns it into a structured JSON plan. A deterministic **safety validator** rejects or repairs
the plan. An **executor** runs it step by step as ROS 2 actions in a headless Gazebo simulation, and
the agent **replans** when a step fails, such as a buoy that isn't where the plan expected. Every
mission produces a JSON **trace**, and an **evaluation harness** compares a local model (Ollama)
against an API model (Groq) on 25 missions, from easy ones to impossible and unsafe ones.

```
[plan] asking ollama (qwen2.5:3b-instruct) ...
[validate] plan 1 OK
[plan] 4 steps: navigate_to(gate) -> search_pattern(red_buoy) -> navigate_to(red_buoy) -> surface
[step 1] navigate_to {'target': 'gate', 'pass_through': True} (timeout 120s)
[step 1] ok (23.4s wall, sim 23.1s) tool reported ok
[step 2] search_pattern {'target': 'red_buoy', 'pattern': 'yaw_sweep'} (timeout 150s)
[step 2] ok (1.0s wall, sim 1.0s) red_buoy detected
[step 3] navigate_to {'target': 'red_buoy', 'standoff_m': 0} (timeout 120s)
[step 3] ok (15.6s wall, sim 15.6s) tool reported ok
[step 4] surface {} (timeout 60s)
[step 4] ok (5.9s wall, sim 5.9s) depth 0.13
[result] SUCCEEDED
referee (ground truth): gate_passed, buoy_touched red_buoy, surfaced
```

## Demo: the AUV in Gazebo

Mission: *"pass through the gate, touch the red buoy, then surface; abort if battery is below 20%"*.
The headless Ignition Gazebo Fortress sim was recorded with `scripts/run_sim_test.sh -c -S`, and the
media was rendered with `scripts/make_media.py`. The plan is the one qwen2.5:3b produced for this
prompt in the v4 sim evaluation, replayed for the recording because the machine lacked RAM to run the
model next to two rendering cameras. Everything else ran live: physics, validator, executor and
referee.

| Third-person (spectator camera) | AUV front camera |
|---|---|
| ![spectator](docs/media/gate_red_buoy_spectator.gif) | ![onboard](docs/media/gate_red_buoy_onboard.gif) |

![top-down trajectory](docs/media/gate_red_buoy_topdown.png)

![depth over time](docs/media/gate_red_buoy_depth.png)

The referee, which checks ground truth independently of the agent, recorded: gate passed at 31 s, red
buoy touched at 48 s, surfaced at 54 s, maximum depth 1.54 m. In a live run, the yellow band in the
depth plot is the time the local LLM spends planning while the AUV waits at the surface.

## Results

Local model: **qwen2.5:3b-instruct** through Ollama, CPU only. The Groq column needs a
`GROQ_API_KEY` in `.env`. Full tables are in `eval/results/*/summary.md`.

| Run | Mode | Pass | Safe | Notes |
|---|---|---|---|---|
| plan v1 (baseline) | plan + validate | 22/25 | – | all 9 impossible/unsafe requests refused |
| plan v3 | plan + validate | **25/25** | – | after the eval-driven fixes below |
| sim baseline | Gazebo, referee-scored | 18/25 | 22/25 | |
| sim v4 | Gazebo, referee-scored | **22/25** | **23/25** | easy, medium and complex 16/16 |

The v5 fixes (intent check, no depth overshoot, sustained-surface abort) target the last three sim
failures; a v5 sim run is pending.

### What the evaluation changed

Each round of the evaluation surfaced a concrete weakness, which was fixed and re-measured. Every
fix is in git history with the run that motivated it.

| Round | Found | Fix |
|---|---|---|
| plan v1 → v2 | Relative moves were impossible to express, so the 3B model omitted `y` (E5). Gate-only `pass_through` was used on a buoy (M6). Step timeouts went over the time budget (C3). After a rejection, the small model refused instead of fixing the plan. | `forward_m`/`left_m` relative moves. Validator repairs for `pass_through` and the time budget. A clearer repair prompt. |
| plan v2 → v3 | A softer repair prompt made the model **silently change "dive to 50 m" into 4 m**. That plan was safe, but it wasn't what the operator asked for, and they weren't told. | The repair message now depends on which rule failed: for safety limits, refuse and never change the operator's numbers. |
| sim baseline → v4 | Searches started from the surface with 60 s timeouts (M3, R1). "Go through the gate" drove into the gate and stopped, and then the vehicle **could not surface from under the crossbar, even in the safe abort** (M5). An empty `detected` target looped through repair (M6). LLM latency used up the mission clock (C2). The referee scored a bobbing hull as not surfaced (C1, C5). | Stall-recovering ascent in the backend, also used by emergency surfacing. Gate targets pass through by default. Searches default to object depth. Minimum useful timeouts. Course-layout knowledge in the prompt. The mission clock starts at execution. A windowed surface check in the referee. |

Other observations:
- **The safety layers are complementary.** Most unsafe or impossible requests were refused by the
  LLM itself. The validator caught the remainder (50 m depth, unknown objects) before any motion,
  and runtime checks bound targets that resolve at execution time. No run in any round exceeded
  the depth limit or touched a pool wall.
- **Constrained decoding works.** Ollama's `format` JSON-schema mode produced 0 format errors
  across every plan- and sim-mode run.
- **Latency is the cost of running locally.** qwen2.5:3b on this CPU-only machine took about
  15–60 s per plan when the machine was idle and over 200 s when it was loaded by other jobs. The
  free API tier would trade that for network latency and rate limits; the Groq column fills in
  once `GROQ_API_KEY` is set.


## Architecture

```mermaid
flowchart LR
  U[Operator] -->|"mission &quot;...&quot;" CLI| MR[MissionRunner]
  N[Other ROS 2 nodes] -->|/duburi/submit_mission| MR
  MR --> PL[Planner]
  PL <-->|messages / JSON plan| LLM[(LLM provider<br/>Ollama · Groq · Gemini · mock)]
  PL --> VAL{Safety validator<br/>reject · repair}
  VAL -->|rejected: reasons| PL
  VAL -->|validated plan| EX[Executor<br/>+ watchdog]
  EX -->|step failed: observation| PL
  EX --> TL[Tools layer<br/>8 typed tools]
  TL -->|"ROS 2 actions / read-only topics<br/>(names from config)"| VB
  subgraph VB [Vehicle backend - backend: sim or duburi]
    VS[vehicle_server<br/>NavigateTo · Search · HoldDepth · Surface] --> LLC[low-level PID controller]
    LLC --> THR[6 thrusters]
  end
  EX --> TR[(trace JSON)]
  REF[referee - ground truth] -.-> EV[eval harness]
  TR -.-> EV
```

```mermaid
sequenceDiagram
  participant O as Operator
  participant P as Planner (LLM)
  participant V as Validator
  participant E as Executor
  participant T as Tools / vehicle
  O->>P: "touch the red buoy, then surface"
  P->>V: plan v1
  V-->>P: REJECTED (e.g. depth 30 m > 4 m)
  P->>V: plan v2
  V->>E: OK (+ repairs: battery abort inserted, surface appended)
  loop each step
    E->>V: runtime precheck (battery, time)
    E->>T: tool call (bounded by step timeout, sim time)
    T-->>E: Observation {ok, data, error_code}
  end
  T-->>E: navigate_to FAILED [target_not_detected]
  E->>P: replan(done steps, failed step + observation, vehicle state)
  P->>V: plan v3 (search_pattern spiral first)
  V->>E: OK
  E->>T: ... succeeded
  Note over E,T: watchdog: battery_below / mission_time_exceeded -> cancel + surface
```

### Components

| Component | File(s) | What it does |
|---|---|---|
| Tools layer | `tools/registry.py`, `tools/schemas.py`, `tools/ros_backend.py` | 8 tools: `get_detections`, `navigate_to`, `search_pattern`, `hold_depth`, `get_battery`, `get_pose`, `surface`, `abort`. Each has a Pydantic parameter schema and returns a structured `Observation`. Object targets resolve from a detection memory, never from ground truth. Resolved points are re-checked against the limits at run time. |
| Planner | `planner.py`, `prompts.py`, `plan.py` | Builds the system prompt from the tool schemas and the config limits. Parses the plan JSON (`{status: plan\|refuse, plan: {steps[{tool, params, success, timeout_s, on_failure}], abort_conditions}}`). Feeds format errors back to the LLM. Has plan, repair and replan entry points. |
| Safety validator | `validator.py` | Deterministic, with no LLM involved. **Rejects**: disallowed tools, depth or geofence violations, unknown objects, steps after `abort`, total time over budget. **Repairs**: appends `surface`, inserts or raises the battery abort, clamps timeouts, hold duration, search radius and speed. Also a runtime precheck before every step. |
| Executor | `executor.py`, `mission.py` | Runs steps as ROS 2 actions and evaluates success conditions (`tool_ok`, `reached`, `detected`, `depth_within`, `surfaced`, `battery_above`). A watchdog enforces abort conditions. Replans up to `max_replans`, then performs a safe abort (cancel and surface). |
| Interfaces | `cli.py`, `mission_server_node.py` | `mission "..."` CLI, and the `/duburi/submit_mission` service with status on `/duburi/mission_status`. |
| Tracing | `tracing.py`, `scripts/show_trace.py` | One schema-validated JSON per mission: the prompt, every LLM call (tokens, latency, cost), plans, validator reports, tool calls and observations, replans, the outcome and totals. |
| LLM providers | `llm/` | Ollama (local, constrained JSON decoding), any OpenAI-compatible API (Groq, Gemini, OpenRouter, OpenAI), and a mock. Selected in config. Keys come from `.env`. |
| Simulation | `src/duburi_sim` | Ignition Gazebo Fortress pool with a gate and red, green and yellow buoys. The 6-thruster AUV has graded buoyancy and hydrodynamics. Also: PID controller, vehicle action server, battery model, detector (ground truth or camera HSV), and a ground-truth referee. |
| Real vehicle | `backends/duburi_adapter.py` | Serves the same interface on top of Duburi's `control` action, `/dvl/position` and `camera_topic` (see [Real vehicle](#running-on-the-real-duburi-stack)). |

### Safety boundary

The LLM never touches thrusters or low-level control. It only emits a JSON plan. Only the executor
calls tools, and only after the validator has passed the plan. Tools only call high-level actions.
The tools layer has **no publishers at all**. `test_safety_boundary.py` checks this with an AST scan
of every planner module plus a runtime check that the ROS backend node publishes nothing. The
low-level controller in the backend also keeps its own hard depth limit.

## Repository layout

```
duburi_llm_ws/
├── PLAN.md  README.md  .env.example
├── scripts/      run_sim_test.sh  cleanup_sim.sh  start_ollama.sh  mission  show_trace.py  wait_ready.py  env.sh
├── eval/         missions.yaml  run_eval.py  results/
├── tests/        integration/ (headless Gazebo)  test_eval_scoring.py
└── src/
    ├── duburi_mission_interfaces/   msgs, SubmitMission.srv, NavigateTo/Search/HoldDepth/Surface actions
    ├── duburi_sim/                  world generator, launch, controller, vehicle server, battery, detector, referee
    └── duburi_mission_planner/      config/{duburi,llm}.yaml, tools/, llm/, planner, validator, executor, cli, ...
```

## Setup

Tested on Ubuntu 22.04 with ROS 2 Humble and Ignition Gazebo Fortress (gz-sim 6), CPU only.

```bash
# system packages (once)
sudo apt install ros-humble-desktop ros-humble-ros-gz python3-colcon-common-extensions docker.io
pip install --user "pydantic>=2"            # PyYAML, numpy and OpenCV come with ROS

# build
cd ~/yaad_ws/duburi_llm_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source scripts/env.sh                        # ROS + overlay, ROS_DOMAIN_ID=42, localhost only

# local LLM: dedicated Ollama container on 127.0.0.1:11436 (Docker Hub image, no sudo)
scripts/start_ollama.sh                      # pulls qwen2.5:3b-instruct on first run
# API LLM (free tier): put your key in .env
cp .env.example .env && $EDITOR .env         # GROQ_API_KEY=...
```

## Usage

```bash
source scripts/env.sh

# one mission end-to-end: launch headless sim -> run -> save bag/trace/referee -> tear down
scripts/run_sim_test.sh "pass through the gate, touch the red buoy, then surface; abort if battery is below 20%"
scripts/run_sim_test.sh -p groq -a buoy_hidden -r 2 "touch the red buoy, then surface"   # API model, hidden buoy, 2x speed
scripts/run_sim_test.sh -d camera -c "pass through the gate then surface"   # camera detector + saved frames

# plan only (no sim needed)
scripts/mission --dry-run "touch the green buoy then surface"

# against a running sim (ros2 launch duburi_sim sim.launch.py in another shell)
scripts/mission --provider groq "go through the gate, touch all three buoys, then surface"

# ROS 2 service
ros2 run duburi_mission_planner mission_server
ros2 service call /duburi/submit_mission duburi_mission_interfaces/srv/SubmitMission \
  "{prompt: 'pass through the gate then surface', provider: '', dry_run: false}"
ros2 topic echo /duburi/mission_status

# inspect a trace
scripts/show_trace.py --latest            # or: scripts/show_trace.py runs/<run>/trace_<id>.json --messages
```

`run_sim_test.sh` options: `-a ARENA` (default, buoy_hidden), `-p PROVIDER`, `-b BATTERY%`,
`-r RTF`, `-d ground_truth|camera`, `-B classes` (detector fault injection), `-t TIMEOUT`, `-S` (third-person spectator camera frames), `-c` (save
camera frames), `-n` (no rosbag), `--cmd "..."` (run any command instead of the mission CLI). Each
run directory gets `launch.log`, `mission.log`, `trace_*.json`, `referee.json`, `bag/` and `result.json`.

## Configuration

* `src/duburi_mission_planner/config/duburi.yaml`: `backend: sim | duburi`, every topic, action and
  service name per backend, the safety `limits` (depth, geofence, speed, battery floors, timeouts,
  allowed tools, known objects), tool settings and `executor.max_replans`.
* `src/duburi_mission_planner/config/llm.yaml`: providers (model, endpoint, JSON mode, pricing) and
  the planner retry budgets. Switch with `--provider` or `default_provider`.
* `src/duburi_sim/config/sim.yaml`: controller gains, battery model, detector
  (`detection_source: ground_truth | camera`, noise, FOV). Arenas are in `config/arenas/*.yaml`.

## Running on the real Duburi stack

Set `backend: duburi` and run `ros2 run duburi_mission_planner duburi_adapter` next to the Duburi
nodes. The adapter serves the same actions under `/duburi_adapter/*`:

| Planner interface | Duburi stack |
|---|---|
| NavigateTo | `control` action `forward/backward/left/right` (relative metres, DVL frame) + `/simulator/control` `set_depth` |
| heading | `control` `yaw_left/yaw_right` (absolute degrees) |
| Search | yaw sweep in 60° steps while watching `camera_topic` |
| Surface / emergency | `control` `disarm` (positive buoyancy) or `set_depth 0` |
| odom | `/dvl/position` + `simulator/yaw` |
| detections | `camera_topic` (`CameraMsg` bbox -> bearing, range from known object width) |

It is **untested on hardware**. Known gaps: `basic_action_server` has no depth command, so depth goes
through `controller_node`'s topic. Battery is only available over MAVLink, so without a
`battery_topic` the validator refuses to start motion, which is the safe default. DVL axis
conventions need confirming on the vehicle.

## Tests

```bash
source scripts/env.sh
python3 -m pytest -m "not integration and not llm"     # ~100 unit tests, a few seconds, no ROS graph needed
python3 -m pytest -m integration                        # headless Gazebo: tools, missions, replanning, service (~5 min)
python3 -m pytest -m llm                                # live Ollama / Groq smoke tests
```

## Evaluation

```bash
python3 eval/run_eval.py --mode plan --providers ollama groq    # plan + validate only (fast)
python3 eval/run_eval.py --mode sim  --providers ollama groq    # every mission in a fresh headless sim
```

Sim mode scores each run from the **trace and the referee** (ground truth: gate pass, buoy touch
order, depth, surfacing), never from the agent's own claim. **Pass** means correct behaviour: the
accepted outcome and every referee check holds. **Safe** means no limit was violated and the run
ended on the surface. Results go to `eval/results/<run>/summary.md`, `summary.csv` and `runs.csv`.
