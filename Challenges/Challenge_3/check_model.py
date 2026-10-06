import mujoco

m = mujoco.MjModel.from_xml_path("assets/panda/panda.xml")
print("nq", m.nq, "nu", m.nu)
print([m.actuator(i).name for i in range(m.nu)])