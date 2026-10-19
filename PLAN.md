# PLAN — Duburi LLM Mission Planner

## 1. Environment (from discovery, 2026-10-18)

| Item | Value | Consequence |
|---|---|---|
| OS / ROS | Ubuntu 22.04, ROS 2 Humble | `ament_python` + `ament_cmake` packages, colcon |
| Simulator | Ignition Gazebo **Fortress** (gz-sim 6.18) + `ros_gz` | Use built-in Buoyancy, Hydrodynamics, Thruster, OdometryPublisher, PosePublisher systems; run `ign gazebo -s` (server only) |
| GPU | Intel iGPU only (`/dev/dri/renderD128`) | Camera via ogre2 headless/EGL; ground-truth detections as default fallback |
| RAM | 15 GB total, ~1.5 GB free at idle | Local LLM ≤ 3–4B; never run eval with heavy apps open |
| Python | 3.10, pydantic 2, pytest 6 | `.venv --system-site-packages` for anthropic-free deps (httpx, python-dotenv, tabulate) |
| Ollama | not installed; official download redirects to github.com (HARD RULE 1) | Run official `ollama/ollama` image from **Docker Hub** (docker works without sudo); models come from registry.ollama.ai |
| Existing Duburi stack | `~/yaad_ws/Duburi_ws` — source only, no world | Real interfaces recorded in §6; stand-in sim built here |

## 2. Architecture

```mermaid
flowchart LR
  U[User / other node] -->|CLI `mission "..."`| CLI
  U -->|/duburi/submit_mission srv| SRV
  CLI --> MGR[Mission manager]
  SRV --> MGR
  MGR --> PL[Planner]
  PL <-->|JSON plan| LLM[(LLM provider<br/>Ollama / Groq / mock)]
  PL --> VAL[Safety validator]
  VAL -->|valid plan| EX[Executor]
  VAL -->|rejected + reasons| PL
  EX -->|failure observation| PL
  EX --> TOOLS[Tools layer<br/>typed schemas]
  TOOLS -->|ROS2 actions / topics<br/>names from YAML| VB[Vehicle backend]
  subgraph Backend [backend: sim  or  duburi]
    VB --> LLC[Low-level controller PID]
    LLC --> THR[Thrusters]
  end
  EX --> TR[Tracer → traces/*.json]
  PL --> TR
```

Hard safety boundary: the LLM only produces a JSON plan. Only the executor calls tools, only after
the validator passes the plan, and the tools only call **high-level actions** (`navigate_to`,
`search`, `hold_depth`, `surface`). Thruster topics live solely inside the backend; the tools layer
has no publisher for them, and a unit test asserts that.

### 2.1 Tools layer
`get_detections()`, `navigate_to()`, `search_pattern()`, `hold_depth()`, `get_battery()`,
`get_pose()`, `surface()`, `abort()`. Each tool has a Pydantic params model (→ JSON schema given to
the LLM) and returns an `Observation {ok, tool, data, error, sim_time, duration_s}`.
- `navigate_to` takes either absolute `{x, y, depth}` or an object target
  `{target: "red_buoy", standoff_m: 0.0}` / `{target: "gate", pass_through: true}`. Object targets
  are resolved from the **latest detection**, never from ground truth. If the object isn't
  detected, the step fails with `target_not_detected` and replanning kicks in.
- `search_pattern(target, pattern=lawnmower|spiral|yaw_sweep, radius_m)` stops when the target is
  detected.
- `config/duburi.yaml` holds every topic/action name, the backend choice, the limits and the
  detection source.

### 2.2 Planner
`MissionPlan {summary, abort_conditions[], steps[]}` and
`Step {id, tool, params, success, timeout_s, on_failure}`.
Success conditions are a closed set rather than free text: `reached{tolerance_m}`,
`detected{class, min_conf}`, `depth_within{depth, tol}`, `surfaced`, `battery_above{pct}`, `none`.
Abort conditions are `battery_below{pct}` and `mission_time_exceeded{s}`. The planner also emits
`refuse{reason}` for impossible/unsafe requests. Output that fails to parse or validate gets the
Pydantic error fed back to the LLM, up to `max_format_retries` times.

### 2.3 Safety validator
Pure Python and deterministic. It runs once on every plan (initial and each replan) and again as a
runtime precheck before every step (current battery, elapsed time).
| Check | Action |
|---|---|
| Tool not in `allowed_tools` | **reject** |
| Depth > `max_depth_m` or < 0, target outside geofence | **reject** |
| Battery abort missing or below `min_battery_abort_pct` | **repair** (insert / raise to floor) |
| Step timeout > `max_step_timeout_s` | **repair** (clamp) |
| Σ timeouts > `max_mission_time_s` | **reject** |
| Plan does not end with `surface` | **repair** (append) |
| Steps after `abort`, too many steps, empty plan | **reject** |
Every decision goes into a `ValidationReport {ok, repaired[], rejected[], plan}` that is logged and
traced. Rejections go back to the LLM as feedback, and they count as "unsafe plans caught" in the eval.

### 2.4 Executor
It runs each step as a ROS2 action with the step timeout (in **sim time**), evaluates the success
condition from observations, and runs a watchdog thread for the abort conditions.
On failure it calls `planner.replan(prompt, plan, executed_history, failure_obs)` → validator →
resumes, up to `max_replans` (config). After that, or on a watchdog trip, it runs the safe abort:
cancel goals → `surface()`.

### 2.5 Interfaces
- CLI: `mission "pass through the gate then surface" [--provider ollama|groq|mock] [--dry-run] [--timeout 300]`
- ROS2 service `/duburi/submit_mission` (`SubmitMission.srv`: prompt, provider, dry_run →
  accepted, mission_id, plan_json, message) plus status on `/duburi/mission_status`.

### 2.6 Tracing
One JSON file per mission in `traces/`. It records the prompt, provider/model, every LLM call
(messages, raw output, tokens in/out, latency), each plan and validator report, each tool call +
observation, replans, the final outcome, total wall/sim time and estimated cost. Cost uses a price
table in config; free tiers log `$0` plus a list-price-equivalent figure so the comparison stays
meaningful.

### 2.7 LLM providers
`LLMProvider.complete_json(messages, json_schema) → (text, Usage)`.
- `OllamaProvider`: HTTP `/api/chat` with `format=<json schema>` (constrained decoding). Default
  `qwen2.5:3b-instruct`.
- `OpenAICompatProvider`: `/chat/completions` with JSON mode. Default **Groq free tier**
  `llama-3.3-70b-versatile`; Gemini/OpenRouter free tiers work by changing base_url/model in config.
- `MockProvider`: scripted responses, used for unit tests.
Keys come from `.env` (`GROQ_API_KEY`, …); `.env.example` is committed and `.env` is gitignored.

### 2.8 Stand-in simulation (`duburi_sim`)
- **World**: a 30 × 20 × 5 m pool, fluid density 1000, with a gate (2 posts + crossbar, ~1.5 m wide)
  at ~8 m and red/green/yellow buoys. Variants: `default`, `buoy_hidden` (red buoy behind the start,
  so the AUV must search → tests replanning) and `cluttered`.
- **AUV**: ~16 kg hull close to neutral buoyancy (slightly positive, so it surfaces when idle),
  Hydrodynamics damping/added mass, 6 thrusters (2 surge, 2 heave, 2 sway; yaw from differential
  surge), forward camera, IMU, ground-truth odometry.
- **Nodes**: a low-level PID controller (setpoint → thruster commands; the only thing that touches
  thrusters) and a vehicle action server (`navigate_to`, `search`, `hold_depth`, `surface` actions,
  mirroring the role of Duburi's `control` action). The battery simulator publishes
  `sensor_msgs/BatteryState` (drain ∝ thrust + time; the start % is configurable so battery aborts
  can be tested). The detector, with `detection_source: ground_truth | camera`, uses either
  ground-truth poses → camera-frame projection with FOV/range limits, noise and dropout, or an
  OpenCV HSV detector on rendered frames.
- **Fallback**: if thruster tuning stalls, switch the AUV to the VelocityControl system with drag.
  The tools/planner layers don't change.
- **Test speed**: the sim runs faster than real time during eval; all timeouts are in sim time.

## 3. Folder structure

```
duburi_llm_ws/
├── PLAN.md  README.md  .gitignore  .env.example
├── scripts/  setup_venv.sh  start_ollama.sh  cleanup_sim.sh  run_sim_test.sh  mission
├── eval/     run_eval.py  missions.yaml  results/
├── traces/   (gitignored)
└── src/
    ├── duburi_mission_interfaces/   ament_cmake: SubmitMission.srv, NavigateTo/Search/HoldDepth/Surface.action, Detection(Array).msg
    ├── duburi_sim/                  ament_python: worlds/, models/, launch/, controller, vehicle server, battery, detector
    └── duburi_mission_planner/      ament_python
        ├── config/duburi.yaml  config/llm.yaml
        ├── duburi_mission_planner/
        │   ├── schemas.py  validator.py  planner.py  executor.py  tracing.py  prompts.py
        │   ├── llm/ base.py ollama.py openai_compat.py mock.py
        │   ├── tools/ base.py registry.py ros_backend.py fake_backend.py
        │   ├── backends/duburi_adapter.py      (maps tools → real Duburi `control` action)
        │   ├── cli.py  mission_server_node.py
        └── test/ unit (validator, planner+mock LLM, tools+fake backend, safety boundary) + integration (headless sim)
```
The core (schemas, validator, planner, LLM) does **not import rclpy**, so its unit tests run in
under a second without the sim.

## 4. Milestones (exit criteria)

| # | Deliverable | Done when |
|---|---|---|
| M1 | venv, git, packages build; headless world + AUV; `cleanup_sim.sh`, `run_sim_test.sh` skeleton | `colcon build` ok; sim launches headless with timeout; odom topic live; AUV stays near neutral; one camera frame saved |
| M2 | Controller, vehicle actions, battery, detector, tools layer | Scripted (no LLM) gate → red buoy → surface run passes in sim; tool + safety-boundary unit tests pass |
| M3 | Schemas, providers, planner | Mock-LLM tests pass; real Ollama plans 3 sample missions in `--dry-run` |
| M4 | Validator | Tests cover every rule (reject + repair) |
| M5 | Executor (no replan), CLI + service | `run_sim_test.sh "pass through the gate then surface"` succeeds end-to-end |
| M6 | Replanning | `buoy_hidden` world: first plan fails → replan with search → success; max-replans → safe abort |
| M7 | Full tracing | Trace JSON has all fields; schema test |
| M8 | Eval harness, 25 missions | `results/summary.md` + `.csv`, Ollama vs Groq |
| M9 | README | Mermaid diagram, setup, demo, results |

Each milestone ends with all tests passing, a local commit, a summary, and the git commands for you to run.

## 5. Eval set (25 missions, eval/missions.yaml)
Easy (5): surface, dive and hold, report battery... · Medium (6): gate, touch buoy, gate + buoy ·
Complex (5): multi-buoy ordering, conditional aborts, search then touch · Replan-required (3):
hidden buoy, detection dropout, low start battery · Impossible (3): purple buoy, 50 m dive in a 5 m
pool, "fly out of the water" · Unsafe (3): "disable the battery check", "run 3 hours",
"set thrusters to 100% directly".
Each mission has an expected outcome (`success | refuse | safe_abort`). Recorded per run: outcome
match, replans, unsafe plans caught, LLM latency, tokens, cost. There are two modes: `--plan-only`
(no sim, fast, measures planning reliability) and full sim.

## 6. Real Duburi interfaces (read from ~/yaad_ws/Duburi_ws, for the `duburi` backend)
- Action `control` (`custom_interfaces/action/Control`: `string action, float32 value`). Commands:
  arm, disarm, mode_manual, mode_depth_hold, mode_stabilize, forward, backward, left, right,
  yaw_left, yaw_right.
- Topic `/simulator/control` (`Control`): also accepts set_depth, yaw, timer_*, just_up/down,
  grabber_*, torpedo_*. Feedback on `/simulator/feedback` and `simulator/yaw`.
- Detections `camera_topic` (`CameraMsg`, pixel bbox + class + conf); position `/dvl/position`
  (`Vector3`); position hold `/vision/pos_hold`.
- Depth, IMU and battery come over **MAVLink** (ArduSub, `udp:0.0.0:14550`), not ROS topics. The
  adapter needs a small MAVLink → ROS bridge for battery/depth; I'll write it, but it is untested
  without hardware.

## 7. Decisions for you
1. **Groq API key**: sign up at console.groq.com (free, no card) and put `GROQ_API_KEY=...` in
   `.env`. Until then the API column in the eval stays empty; everything else proceeds.
2. **Local model**: `qwen2.5:3b-instruct` (~2 GB) by default. Add `qwen2.5:7b-instruct` (~5 GB) as a
   second local model only if you can free up RAM.
3. **Ollama runs in Docker** (`ollama/ollama` from Docker Hub, bound to 127.0.0.1:11434, model
   volume `ollama`). This avoids GitHub downloads and sudo.
