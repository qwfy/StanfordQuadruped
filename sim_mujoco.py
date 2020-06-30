import mujoco_py
import numpy as np
import logging
logging.basicConfig(level=logging.DEBUG)

import common
import pupper
import sim

logger = logging.getLogger(__name__)

def parallel_to_serial_joint_angles(joint_matrix):
  """Convert from joint angles meant for the parallel linkage in
  Pupper to the joint angles in the serial linkage approximation implemented in the simulation

  Parameters
  ----------
  joint_matrix : Numpy array (3, 4)
      Joint angles for parallel linkage

  Returns
  -------
  Numpy array (3, 4)
      Joint angles for equivalent serial linkage
  """
  temp = joint_matrix
  temp[2, :] -= joint_matrix[1, :]
  return temp


def main(default_velocity=np.zeros(2), default_yaw_rate=0.0):
  sim.gen_mjcf.generate(
    pupper_config=pupper.config.Configuration(),
    simulation_config=pupper.config.SimulationConfig(),
    )

  model = mujoco_py.load_model_from_path(pupper.config.SimulationConfig().XML_OUT)
  simulator = mujoco_py.MjSim(model)
  viewer = mujoco_py.MjViewer(simulator)

  # Create config
  config = pupper.config.Configuration()
  config.z_clearance = 0.02
  # hardware_interface = HardwareInterface(sim.model, sim.joint_indices)

  # Create controller and user input handles
  controller = common.controller.Controller(config, pupper.kinematics.four_legs_inverse_kinematics, )
  state = common.state.State()
  state.behavior_state = common.state.BehaviorState.DEACTIVATED
  state.quat_orientation = np.array([1, 0, 0, 0])

  # Initialize joint angles
  simulator.data.qpos[0:12] = parallel_to_serial_joint_angles(
    state.joint_angles
    ).T.reshape(12)
  # Set the robot to be above the floor to begin with
  simulator.data.qpos[2] = 0.5

  command = common.command.Command()

  # Emulate the joystick inputs required to activate the robot
  command.activate_event = 1
  controller.run(state, command)
  command.activate_event = 0
  command.trot_event = 1
  controller.run(state, command)
  command = common.command.Command()  # zero it out

  # Apply a constant command. # TODO Add support for user input or an external commander
  command.horizontal_velocity = default_velocity
  command.yaw_rate = default_yaw_rate

  # The joystick service is linux-only, so commenting out for mac
  # print("Creating joystick listener...")
  # joystick_interface = JoystickInterface(config)
  # print("Done.")

  print("Summary of gait parameters:")
  print("overlap time: ", config.overlap_time)
  print("swing time: ", config.swing_time)
  print("z clearance: ", config.z_clearance)
  print("x shift: ", config.x_shift)

  # Run the simulation
  timesteps = 240 * 60 * 10  # simulate for a max of 10 minutes

  # Sim seconds per sim step
  sim_steps_per_sim_second = 240
  sim_dt = 1.0 / sim_steps_per_sim_second
  last_control_update = 0

  for steps in range(timesteps):
    sim_time_elapsed = sim_dt * steps
    if sim_time_elapsed - last_control_update > config.dt:
      last_control_update = sim_time_elapsed

      state.quat_orientation = np.array([1, 0, 0, 0])

      # Step the controller forward by dt
      controller.run(state, command)

      # Update the pwm widths going to the servos
      # hardware_interface.set_actuator_postions(state.joint_angles)

      # Convert from joint angles meant for a parallel linkage to the serial linkage implemented in Mujoco
      simulator.data.ctrl[:] = parallel_to_serial_joint_angles(
        state.joint_angles
        ).T.reshape(12)

    # Simulate physics for 1/240 seconds (the default timestep)
    simulator.step()
    viewer.render()


if __name__ == "__main__":
  main(default_velocity=np.array([0.15, 0]))
