import cmd
import rclpy
from helmoro_cli.robot_manager import RobotManager
from helmoro_cli.simulation_manager import SimulationManager


class RobotCLI(cmd.Cmd):
    intro = "Welcome to the Robot CLI. Type help or ? to list commands.\n"
    prompt = "(robot-cli) "

    def __init__(self):
        super().__init__()
        rclpy.init()
        self.robot_manager = RobotManager()
        self.simulation_manager = SimulationManager()
        self.current_robot = None

    def do_start_simulation(self, arg):
        "Start the simulation: start_simulation <world>"
        if self.simulation_manager.get_status() == "running":
            print("Simulation is already running.")
            return
        world = arg or "empty"
        self.simulation_manager.start_simulation(world)

    def do_stop_simulation(self, arg):
        "Stop the simulation: stop_simulation"
        if self.simulation_manager.get_status() == "stopped":
            print("Simulation is already stopped.")
            return
        self.simulation_manager.stop_simulation()

    def do_spawn(self, arg):
        "Spawn a robot: spawn <name> <x> <y> <z> <yaw>"

        if self.simulation_manager.get_status() == "stopped":
            print("Can't spawn robot. Simulation is not running.")
            return

        args = arg.split()
        if len(args) != 5:
            print("Usage: spawn <name> <x> <y> <z> <yaw>")
            return
        self.robot_manager.spawn_robot(*args)
        self.simulation_manager.spawn_robot(*args)

    def do_delete(self, arg):
        "Delete a robot: delete <name>"
        self.robot_manager.delete_robot(arg)

        if self.simulation_manager.get_status() == "running":
            self.simulation_manager.delete_robot(arg)

    def do_exit(self, arg):
        "Exit the CLI"
        if self.simulation_manager.get_status() == "running":
            self.simulation_manager.stop_simulation()

        print("Exiting CLI...")
        return True

    # TODO: This function is meant as a helper during development. Do not use this function for production.
    def do_multi_robot_spawn(self, arg):
        "Run multi-robot test"
        self.do_start_simulation(arg)
        arg1 = "alfred -0.5 3 0 2.2 "
        arg2 = "bob -0.5 -3.5 0 2.6"
        arg3 = "charlie 3 -1.2 0 0"
        arg4 = "echo 5.5 -1.3 0 0"
        arg5 = "foxtrott 7.4 -1.6 0 1.8"
        arg6 = "golf 2.4 2.5 0 1.6"        
        arg7 = "hotel -2.6 2.9 0 -1.6"
        arg8 = "india -1.6 -3.5 0 -0.2"
        arg9 = "juliett 2.6 1 0 -0.1"
        arg10 = "kilo 13 -0.6 0 -1.5"
        self.do_spawn(arg1)
        self.do_spawn(arg2)
        self.do_spawn(arg3)
        self.do_spawn(arg4)
        # self.do_spawn(arg5)
        # self.do_spawn(arg6)
        # self.do_spawn(arg7)
        # self.do_spawn(arg8)
        # self.do_spawn(arg9)
        # self.do_spawn(arg10)

def main():
    try:
        RobotCLI().cmdloop()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
