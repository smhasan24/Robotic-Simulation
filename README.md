

# Autonomous Robot Simulation  
![Autonomous Robot Demo](https://github.com/smhasan24/Robotic-Simulation/blob/main/converted_video.gif)


This project simulates an autonomous robot navigating a terrain while avoiding obstacles and handling slopes. It uses Unity and C# to implement sensor-based decision-making and realistic motion control.  

## Features  
- **Sensor-Based Navigation**: Uses raycasting sensors to detect obstacles, slopes, and road boundaries.  
- **Dynamic Steering**: Adjusts steering angles and motor forces based on sensor inputs.  
- **Realistic Physics**: Utilizes Unity's Rigidbody and WheelCollider components for accurate motion simulation.  
- **Obstacle Avoidance**: Detects and avoids obstacles using multiple sensors.  
- **Slope Handling**: Adjusts motor force dynamically to handle slopes.  

## How It Works  
- **Sensors**: The robot uses multiple raycast sensors to detect obstacles, slopes, and road edges.  
- **Steering**: The robot adjusts its steering angle based on sensor inputs to avoid obstacles and stay on the road.  
- **Motor Force**: The robot dynamically adjusts its motor force to handle slopes and maintain speed.  
- **Simulation**: The simulation pauses when the robot detects an obstacle ahead, marking the end of the run.  

## Code Structure  
- **RobotController.cs**: Main script controlling the robot's behavior, including sensor detection, steering, and motor control.  
- **WheelColliders**: Handles the physics of the robot's wheels.  
- **Transforms**: Manages the visual representation of the wheels and sensors.  

## Usage  
1. Open the project in Unity.  
2. Attach the `RobotController` script to the robot GameObject.  
3. Assign the WheelColliders and Transforms for the wheels and sensors in the Unity Inspector.  
4. Run the simulation and observe the robot navigating the terrain.  

## Parameters  
- **Base Force**: Base motor force for movement.  
- **Max Force**: Maximum motor force.  
- **Max Turn Angle**: Maximum steering angle.  
- **Detection Range**: Range for road and obstacle detection.  
- **Slope Factor**: Force multiplier for slope handling.  

## Credits  
- **Course**: 7COM1032-0901-2024 - Artificial Life with Robotics  
- **Author**: S M Mahamudul Hasan  
- **Registration**: 23106367  

--- 

Feel free to explore, modify, and extend this project! For questions or suggestions, open an issue or submit a pull request.
