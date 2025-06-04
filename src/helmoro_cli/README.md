# Helmoro CLI

The `helmoro_cli` package provides a command-line interface (CLI) for interacting with the Helmoro robotics system. It allows users to manage simulations, spawn and delete robots, and perform other operations directly from the terminal.

## Features

- **Simulation Management**: Start and stop simulations with ease.
- **Robot Management**: Spawn and delete robots in the simulation environment.
- **Interactive CLI**: A user-friendly interface with commands for various tasks.
- **Multi-Robot Support**: Includes a helper function for spawning multiple robots during development.

## Usage

To launch the CLI, use the following command:

```bash
ros2 run helmoro_cli open_cli
```

Once launched, you will be greeted with a prompt where you can type commands. Use help or ? to list available commands.

## Commands
* start_simulation <world>: Start the simulation with the specified world (default: empty).
* stop_simulation: Stop the currently running simulation.
* spawn <args>: Spawn a robot with the specified arguments.
* delete <robot_name>: Delete a robot by its name.
* multi_robot_spawn <args>: Helper function for spawning multiple robots (for development purposes).
* exit: Exit the CLI.