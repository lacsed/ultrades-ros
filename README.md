# UltraDES-ROS Interface

This project provides a modular, ROS 1-compatible interface for **UltraDES**, a C#-based Discrete Event Systems (DES) library, allowing deterministic finite automata (DFA) to be modeled, executed, and coordinated via ROS topics in real time.

The system enables:
- Modeling of plants, supervisors, and task specifications as formal automata
- ROS-based deployment and runtime control of UltraDES automata
- Tkinter GUI panel to monitor state transitions and trigger events manually
- Integration with multi-agent systems and real-time supervision scenarios

---

## Project Structure

```
ultrades_ros/
├── build/                # Catkin build outputs (ignored in Git)
├── CMakeLists.txt        # ROS build instructions
├── package.xml           # ROS package metadata and dependencies
├── core/                 # Core Python logic for ROS-automaton interface
│   ├── __init__.py
│   ├── utils.py          # JSON loader for DFA models
│   └── AutomatonNode.py  # Main ROS-compatible automaton wrapper
├── scripts/              # ROS-executable Python scripts and examples
│   ├── simple_plant.py   # Minimal automaton node for testing
│   ├── control_panel.py  # Tkinter GUI for real-time monitoring
│   └── example.json      # JSON example of a DFA model
├── shell/                # Utility shell scripts
│   └── reset_ros_env.sh  # Kill and clean up ROS nodes/processes
```

---

## Requirements

The following tools and libraries must be **pre-installed** on your system:

### Core System Requirements

| Tool             | Version              | Install Guide                                                       |
|------------------|----------------------|---------------------------------------------------------------------|
| **Python**       | 3.8+                 | `sudo apt install python3`                                          |
| **ROS 1**        | Noetic               | http://wiki.ros.org/noetic/Installation                             |
| **Mono**         | ≥ 6.12               | https://www.mono-project.com/download/stable/                       |
| **UltraDES**     | Installed via `pip`  | Must include `UltraDES.dll` inside the Python package               |
| **pythonnet**    | Compatible with Mono | `pip install pythonnet`                                             |
| **Tkinter**      | For GUI support      | `sudo apt install python3-tk`                                       |

---

## What the Code Does

### `AutomatonNode`

Encapsulates any DFA as a **ROS node**, allowing:

- Subscription to `/event`
- Publishing current state to `/<node>/state`
- Publishing enabled events to `/<node>/possible_events`
- Publishing marked state status to `/<node>/is_marked`
- Dynamic callbacks per state

You can create an automaton either from a `.json` file or manually using the UltraDES API.

---

### `core/utils.py`

Contains `load_from_json(path)` to load DFA definitions from `.json` files structured like:

```json
{
  "states": ["q0", "q1"],
  "events": ["a", "b"],
  "initial_state": "q0",
  "marked_states": ["q0"],
  "controllable_events": ["a"],
  "transitions": {
    "q0": {"a": "q1"},
    "q1": {"b": "q0"}
  }
}
```

---

## How to Run a Basic Test

1. Start `roscore` in a new terminal:

```bash
roscore
```

2. Run the simple automaton node:

```bash
rosrun ultrades_ros simple_plant.py
```

3. Publish an event to the global event topic:

```bash
rostopic pub /event std_msgs/String "a" -1
```

4. (Optional) Start the control panel to monitor and interact with automata:

```bash
rosrun ultrades_ros control_panel.py
```

---

## Environment Setup Notes (Mono / UltraDES)

To ensure `UltraDES.dll` is correctly loaded by Mono and Python:

1. Confirm that `UltraDES.dll` is located in your Python package directory:

```
~/.local/lib/python3.8/site-packages/ultrades/UltraDES.dll
```

2. Set the `MONO_PATH` environment variable to point to that folder:

```bash
export MONO_PATH=$HOME/.local/lib/python3.8/site-packages/ultrades
```

To make it permanent, add that line to your `.bashrc` or `.zshrc`.

3. Test DLL loading by running:

```bash
python3 -c "import ultrades.automata"
```

If no error is raised, the integration is working correctly.

