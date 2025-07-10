# HelMoRo-software-ROS2

Welcome to the microservices branch! This branch provides a modular and extensible software stack for robotics applications using ROS 2. The repository includes tools for simulation, control, visualization, and command-line interaction with robots.

## Roadmap
### Current Features
* **Simulation**: Manage simulation environments with Gazebo and ROS 2 integration.
* **Control**: Implement robot joint control using ros2_control and Gazebo plugins.
* **Visualization**: Visualize robot states and environments using RViz2.
* **Command-Line Interface**: Interact with the system via a user-friendly CLI.
* **Multi-Robot Support**: Built for managing multiple robots in simulation.

### Planned Features
* **State Estimation**: Sensor fusion for accurate odometry.
* **SLAM**: Self-localization and mapping in a dynamic environment.
* **Teleopration**: Steer your robots with joysticks.
* **Navigation**: Autonomous navigation through unknown environments.


## Repository Overview
* **`.devcontainer`**: Development container configuration for VS Code.
* **`.github`**: Configurations for CI pipeline using GitHub Actions.
* **`docker`**: Docker setup for development and deployment.
* **`config`**: Configuration files for middleware and other tools.
* **`src/ helmoro_cli`**: Command-line interface for managing simulations and robots.
* **`src/ helmoro_control`**: ROS 2 package for controlling robot joints and states.
* **`src/ helmoro_description`**: Robot description files (URDF/Xacro)
* **`src/ helmoro_simulation`**: Tools and configurations for simulation environments.
* **`src/ helmoro_utils`**: Utility scripts and tools for the project
* **`src/ helmoro_visualization`**: Visualization tools and RViz configurations.

## Installation

### Prerequisites
- Linux OS
- Docker for [non-root user](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user)
- VSCode with Devcontainer Extension

### Setup
1. Clone the repository and check out the microservices branch:
 ```bash
git clone https://github.com/helbling-technik/HelMoRo-software-ROS2.git
git checkout microservices
 ```

2. Open the repo with VSCode:
 ```bash
code HelMoRo-software-ROS2
 ```

3. Start the Devcontainer through VSCode's command palette:
 ```
 Dev Containers: Reopen in Container
 ```

### Running the Simulation
For running the simulation it's recommended to use the [CLI](/src/helmoro_cli/README.md). Start the CLI with:
```bash
ros2 run helmoro_cli open_cli
```

The simulation can be started with:
```bash
start_simulation depot
```

Robots can be spawned with:
```bash
multi_robot_spawn
```

## Contributing
Contributions are welcome! Create an issue or resolve an existing one. For new contributors, we recommend tackling one of the issues labeled with "good first issue", as they do not require a complete understanding of the repo.

### Testing
For local testing type `test` in your terminal. This will start up the act container. On the first startup, you'll be asked to choose a default image to use with Act, and choose the medium one. Until now that one had all the utilities we needed. During development, the alias `clc_test` is also useful. Instead of creating a containerized environment similar to the tests running in GitHub runners, it runs the tests directly in the Devcontainer. This leads to quicker testing, but may not catch all bugs.

### Pull Request
When starting out create a draft pull request, so others know that you're working on the issue. On completion of a successful local test push to your dev branch and change your draft pull request to a normal one.

### Commit Messages
By following a unified commit message standard, we are able to trace bugs quickly. Please adhere to the [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/) specification.

## License
This project is licensed under the BSD 3-Clause License. See the [LICENSE](./LICENSE) file for details.