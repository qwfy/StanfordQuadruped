def generate(pupper_config, simulation_config):
  """
  Replace the variable names in the placeholder XML robot file
  with actual values given from the configuration object parameters
  """

  # FILE PATHS
  template_file = simulation_config.XML_IN
  result_file = simulation_config.XML_OUT

  # ROBOT PARAMETERS

  # Solver params
  pupper_timestep = simulation_config.DT
  pupper_joint_solref = simulation_config.JOINT_SOLREF
  pupper_joint_solimp = simulation_config.JOINT_SOLIMP
  pupper_geom_solref = simulation_config.GEOM_SOLREF
  pupper_geom_solimp = simulation_config.GEOM_SOLIMP

  # Geometry params
  pupper_leg_radius = pupper_config.FOOT_RADIUS  # radius of leg capsule
  pupper_friction = simulation_config.MU  # friction between legs and ground
  pupper_half_size = "%s %s %s" % (
    pupper_config.L / 2,
    pupper_config.W / 2,
    pupper_config.T / 2,
    )  # half-size of body box
  # to-from leg geometry
  pupper_l1_geom = "0 0 0 0 0 %s" % (-pupper_config.LEG_L1)
  pupper_l2_geom = "0 0 0 0 0 %s" % (-pupper_config.LEG_L2)

  pupper_start_position = "0 0 %s" % (
    simulation_config.START_HEIGHT
  )  # Initial position of the robot torso
  pupper_hip_box = "%s %s %s" % (
    pupper_config.HIP_L / 2,
    pupper_config.HIP_W / 2,
    pupper_config.HIP_T / 2,
    )  # Size of the box representing the hip

  pupper_force_geom = "0 0 -0.34"

  # Mass/Inertia Params
  pupper_armature = simulation_config.ARMATURE  # armature for joints [kgm2]
  pupper_frame_inertia = "%s %s %s" % pupper_config.FRAME_INERTIA
  pupper_module_inertia = "%s %s %s" % pupper_config.MODULE_INERTIA
  pupper_leg_inertia = "%s %s %s" % pupper_config.LEG_INERTIA

  # Joint & servo params
  pupper_rev_kp = simulation_config.SERVO_REV_KP

  pupper_joint_range = "%s %s" % (
    -simulation_config.REVOLUTE_RANGE,
    simulation_config.REVOLUTE_RANGE,
    )  # joint range in rads for angular joints
  pupper_l2_joint_range = "%s %s" % (
    -simulation_config.REVOLUTE_RANGE - simulation_config.REVOLUTE_RANGE,
    simulation_config.REVOLUTE_RANGE - simulation_config.REVOLUTE_RANGE,
    )  # joint range for l2 knee joint
  pupper_rev_torque_range = "%s %s" % (
    -simulation_config.MAX_JOINT_TORQUE,
    simulation_config.MAX_JOINT_TORQUE,
    )  # force range for ab/ad and forward/back angular joints
  pupper_rev_damping = (
    simulation_config.REV_DAMPING
  )  # damping on angular joints [Nm/rad/s]

  # Sensor Noise Parameters #
  pupper_accel_noise = 0.01
  pupper_encoder_noise = 0.001
  pupper_gyro_noise = 0.02
  pupper_encoder_vel_noise = 0.01
  pupper_force_noise = 0

  # Parse the xml
  print("Parsing MuJoCo XML file:")
  print("Input xml: %s" % template_file)
  print("Output xml: %s" % result_file)

  with open(template_file, "r") as file:
    file_data = file.read()

  # Replace variable names with values

  # Solver specs
  file_data = file_data.replace("pupper_timestep", str(pupper_timestep))
  file_data = file_data.replace("pupper_joint_solref", str(pupper_joint_solref))
  file_data = file_data.replace("pupper_geom_solref", str(pupper_geom_solref))
  file_data = file_data.replace("pupper_friction", str(pupper_friction))
  file_data = file_data.replace("pupper_armature", str(pupper_armature))
  file_data = file_data.replace("pupper_joint_solimp", str(pupper_joint_solimp))
  file_data = file_data.replace("pupper_geom_solimp", str(pupper_geom_solimp))

  # Joint specs
  file_data = file_data.replace("pupper_joint_range", str(pupper_joint_range))
  file_data = file_data.replace("pupper_l2_joint_range", str(pupper_l2_joint_range))
  file_data = file_data.replace("pupper_rev_torque_range", str(pupper_rev_torque_range))
  file_data = file_data.replace("pupper_rev_damping", str(pupper_rev_damping))

  # Servo specs
  file_data = file_data.replace("pupper_rev_kp", str(pupper_rev_kp))

  # Geometry specs
  file_data = file_data.replace("pupper_frame_mass", str(pupper_config.FRAME_MASS))
  file_data = file_data.replace("pupper_module_mass", str(pupper_config.MODULE_MASS))
  file_data = file_data.replace("pupper_leg_mass", str(pupper_config.LEG_MASS))
  file_data = file_data.replace("pupper_frame_inertia", str(pupper_frame_inertia))
  file_data = file_data.replace("pupper_module_inertia", str(pupper_module_inertia))
  file_data = file_data.replace("pupper_leg_inertia", str(pupper_leg_inertia))
  file_data = file_data.replace("pupper_leg_radius", str(pupper_leg_radius))
  file_data = file_data.replace("pupper_half_size", str(pupper_half_size))
  file_data = file_data.replace("pupper_leg_fb", str(pupper_config.LEG_FB))
  file_data = file_data.replace("pupper_leg_lr", str(pupper_config.LEG_LR))
  file_data = file_data.replace("pupper_start_position", str(pupper_start_position))
  file_data = file_data.replace("pupper_force_geom", str(pupper_force_geom))
  file_data = file_data.replace("pupper_hip_box", str(pupper_hip_box))
  file_data = file_data.replace("pupper_hip_offset", str(pupper_config.HIP_OFFSET))
  file_data = file_data.replace(
    "pupper_abduction_offset", str(pupper_config.ABDUCTION_OFFSET)
    )
  file_data = file_data.replace("pupper_l1_length", str(pupper_config.LEG_L1))
  file_data = file_data.replace("pupper_l2_length", str(pupper_config.LEG_L2))
  file_data = file_data.replace("pupper_l1_geom", str(pupper_l1_geom))
  file_data = file_data.replace("pupper_l2_geom", str(pupper_l2_geom))

  # Sensor noise
  file_data = file_data.replace("pupper_accel_noise", str(pupper_accel_noise))
  file_data = file_data.replace("pupper_gyro_noise", str(pupper_gyro_noise))
  file_data = file_data.replace("pupper_encoder_noise", str(pupper_encoder_noise))
  file_data = file_data.replace(
    "pupper_encoder_vel_noise", str(pupper_encoder_vel_noise)
    )
  file_data = file_data.replace("pupper_force_noise", str(pupper_force_noise))

  # Write the xml file
  with open(result_file, "w") as file:
    file.write(file_data)
