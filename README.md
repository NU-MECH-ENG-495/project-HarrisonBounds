# Project Proposal for C++
For my project, I aim to implement and test code on a physical robotic system, aligning with my major in Robotics. I have chosen to work with the Unitree Robot dogs available in our lab, focusing on enabling them to navigate a space using pathfinding algorithms. While many pathfinding algorithms perform well in static environments, real-world scenarios often involve dynamic obstacles such as moving objects, people, or other robots. My goal is to implement and refine a pathfinding and exploration algorithm that allows the robot dog to successfully navigate a 3D environment with both static and dynamic obstacles.

The first step toward achieving this goal is to interface with the Unitree robot dog's Software Development Kit (SDK) and thoroughly understand the existing codebase. This will involve studying how the robot moves, what sensors it utilizes (e.g., cameras, LiDAR, IMU), and how it processes environmental data. The SDK is publicly available on Unitree's GitHub repository, which I will clone and explore to build a foundation for my project.

### Software Development in the First Step:

- SDK Familiarization: I will begin by setting up the SDK, running basic motion control scripts, and analyzing the sensor data output. This will help me understand the robot's capabilities and limitations.

- Environment Simulation: Using the SDK, I will create a simulated environment to test basic navigation algorithms before deploying them on the physical robot. This will allow me to debug and refine the code in a controlled setting.

- Basic Navigation Implementation: I will implement a simple pathfinding algorithm (e.g., A* or Dijkstra's) to enable the robot to navigate from a starting point to a predefined destination in a static environment. This will serve as a baseline for more complex dynamic obstacle avoidance.

### Defining Success:
Success in the first phase will be measured by the robot's ability to:

- Navigate from a specified starting point to a destination in a static environment without collisions.

- Utilize sensor data (e.g., LiDAR or camera input) to detect and avoid static obstacles in its path.

- Operate autonomously for a specified period of time (e.g., 5-10 minutes) without human intervention.

### Future Steps:
Once the robot can successfully navigate a static environment, I will expand the project to include dynamic obstacles. Success in this phase will involve the robot adapting its path in real-time to avoid moving objects while still reaching its destination. The final goal is to enable the robot to explore and map an unknown 3D environment, dynamically adjusting its path as it encounters new obstacles.

By the end of this project, I aim to have a fully functional implementation of a pathfinding and exploration algorithm that demonstrates the robot's ability to navigate complex, real-world environments. This will not only advance my understanding of robotics and pathfinding but also contribute to the lab's ongoing research in autonomous systems.
