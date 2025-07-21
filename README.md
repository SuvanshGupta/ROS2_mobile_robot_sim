This ROS 2 package simulates a differential-drive mobile robot in Gazebo, integrating core robotic capabilities such as SLAM, PID-based motion control, and realistic sensor data from camera, depth camera, and IMU. The system is designed to serve as a comprehensive testbed for autonomous navigation and control in ROS 2.

In addition to standard control, the package also showcases real-time PID autotuning using two powerful optimization algorithms:

    🦅 Harris Hawks Optimization (HHO) — for rapid convergence and dynamic adaptability

    ❄️🦅 Snow Falcon Optimization Algorithm (SFOA) — An algorithm developed by the author, combining the terrain awareness of snow leopards and the fast targeting of falcons to enable predictive, smooth, and fast control response.

🧩 Key Features

    ✅ Gazebo simulation of a differential-drive mobile robot with realistic physics

    🧭 SLAM integration using slam_toolbox for real-time map generation and localization

    ⚙️ PID-based velocity control with both manual and auto-tuned gains

    📡 Sensor plugins:

        RGB camera (for visual perception)

        Depth camera (for obstacle detection)

        IMU (for orientation feedback)

    📈 Real-time PID tuning using optimization:

        HHO for adaptive exploration-exploitation control

        SFOA (novel) for ultra-fast, predictive PID gain adjustment

    🌐 Modular ROS 2 package structure with launch files and simulation config

    🧪 Ideal for research, education, and prototyping in autonomous robotics

🎯 Applications

    SLAM algorithm testing and benchmarking

    Advanced PID control development and analysis

    Real-time optimization research

    Autonomous navigation simulations

    Educational tool for ROS 2, Gazebo, and control systems
