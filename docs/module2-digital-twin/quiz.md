---
sidebar_position: 6
title: Module 2 Quiz
---

# Module 2: Gazebo & Digital Twin Quiz

This quiz will test your understanding of the core concepts covered in Module 2: Gazebo & Digital Twin.

---

1.  What is the primary purpose of a digital twin in robotics development?
    A) To replace physical robots entirely
    B) To provide a virtual representation for testing and validation
    C) To increase the cost of robot development
    D) To eliminate the need for simulation
    **Answer:** B
    **Explanation:** Digital twins serve as virtual counterparts of physical robots, allowing developers to test, validate, and optimize robot behaviors in a safe, controlled environment before deploying to real hardware.

2.  Which physics engines does Gazebo support for simulation?
    A) Only ODE
    B) Only Bullet
    C) ODE, Bullet, and Simbody
    D) Only Simbody
    **Answer:** C
    **Explanation:** Gazebo supports multiple physics engines including ODE (Open Dynamics Engine), Bullet Physics, and Simbody, allowing users to choose based on their specific requirements.

3.  In Gazebo's coordinate system, which axis points upward?
    A) X-axis
    B) Y-axis
    C) Z-axis
    D) None of the above
    **Answer:** C
    **Explanation:** Gazebo uses a right-handed coordinate system where the Z-axis points upward (opposite to gravity direction).

4.  What is the main difference between visual and collision properties in robot modeling?
    A) Visual properties affect physics, collision properties affect appearance
    B) Visual properties affect appearance, collision properties affect physics
    C) There is no difference between them
    D) Both affect both appearance and physics
    **Answer:** B
    **Explanation:** Visual properties define how the robot appears in the simulation environment, while collision properties define the shapes used for collision detection and physics simulation.

5.  What does the `static` property do when set to `true` in a Gazebo model?
    A) Makes the object invisible
    B) Makes the object immovable
    C) Makes the object dynamic
    D) Deletes the object
    **Answer:** B
    **Explanation:** Setting the `<static>` property to `true` makes an object immovable in the simulation, which is useful for environment objects like walls and floors.

6.  Which file format does Gazebo use for describing simulation environments?
    A) URDF
    B) YAML
    C) SDF (Simulation Description Format)
    D) JSON
    **Answer:** C
    **Explanation:** Gazebo uses SDF (Simulation Description Format), an XML-based format that describes the entire simulation environment.

7.  What is the primary purpose of inertial properties in robot modeling for simulation?
    A) To define the visual appearance of the robot
    B) To define how the robot interacts with the physics engine
    C) To define the sensor configuration
    D) To define the joint limits
    **Answer:** B
    **Explanation:** Inertial properties are critical for accurate physics simulation as they define how mass is distributed and how the robot responds to forces and torques.

8.  What does the restitution property control in Gazebo physics?
    A) The friction between surfaces
    B) The bounciness of objects
    C) The density of materials
    D) The color of objects
    **Answer:** B
    **Explanation:** Restitution controls how bouncy objects are, with 0 meaning no bounce and 1 meaning perfectly elastic collision.

9.  Which plugin type would you use to simulate a differential drive robot in Gazebo?
    A) Joint control plugin
    B) Ackermann steering plugin
    C) Differential drive plugin
    D) Sensor plugin
    **Answer:** C
    **Explanation:** The differential drive plugin is specifically designed to simulate robots with differential drive kinematics, controlling left and right wheel joints.

10. What is the primary advantage of using simulation for robotics development?
    A) It always perfectly matches real-world behavior
    B) It provides a safe, controlled environment for testing without risk of hardware damage
    C) It eliminates the need for real hardware
    D) It makes robots faster in real life
    **Answer:** B
    **Explanation:** Simulation provides a safe, controlled environment for testing and validating robot behaviors without the risk of damaging expensive hardware or causing safety issues.