# import pybullet as p
# import pybullet_data
# import time

# p.connect(p.GUI)
# p.setPhysicsEngineParameter(enableFileCaching=0)
# p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
# p.setAdditionalSearchPath(pybullet_data.getDataPath())

# # Create floor
# plane_shape = p.createCollisionShape(p.GEOM_PLANE)
# floor = p.createMultiBody(plane_shape, plane_shape)
# p.setGravity(0, 0, -10)

# # Load your two-link robot
# rob3 = p.loadURDF("102.urdf")

# # Keep simulation running
# while True:
#     p.stepSimulation()
#     time.sleep(1.0 / 240)

import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# Load plane
p.loadURDF("plane.urdf")

# Load your robot
robot_id = p.loadURDF("two_link_robot.urdf", [0, 0, 0.1])

# Move it around programmatically
for step in range(1000):
    p.stepSimulation()
    if step == 300:
        # Move the whole robot forward and slightly rotated
        pos = [0.5, 0, 0.1]
        orn = p.getQuaternionFromEuler([0, 0, 1.57])
        p.resetBasePositionAndOrientation(robot_id, pos, orn)
    time.sleep(1/240)
