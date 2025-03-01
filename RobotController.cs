using System.Collections;
using System.Collections.Specialized;
using System.Security.Cryptography;
using UnityEngine;


public class RobotController : MonoBehaviour
{
    // Wheel colliders for controlling the robot's wheels
    [SerializeField] private WheelCollider FLC; // -Front Left Collider
    [SerializeField] private WheelCollider FRC; // -Front Right Collider
    [SerializeField] private WheelCollider RLC; // -Rear Left Collider
    [SerializeField] private WheelCollider RRC; // -Rear Right Collider


    // Sensors for detecting obstacles and road conditions
    [SerializeField] private Transform FRS; // -Front Sensor
    [SerializeField] private Transform L1S; // -Left Sensor 1
    [SerializeField] private Transform L2S; // -Left Sensor 2
    [SerializeField] private Transform L3S; // -Left Sensor 3
    [SerializeField] private Transform R1S; // -Right Sensor 1
    [SerializeField] private Transform R2S; // -Right Sensor 2
    // Transforms for visual representation of the wheels
    [SerializeField] private Transform FLT; // -Front Left Transform
    [SerializeField] private Transform FRT; // -Front Right Transform
    [SerializeField] private Transform RLT; // -Rear Left Transform
    [SerializeField] private Transform RRT; // -Rear Right Transform

    [SerializeField] private Transform R3S; // -Right Sensor 3
    [SerializeField] private Transform ORS; // -Obstacle Right Sensor

    // Adjustable parameters for sensors and movement
    private float sensorRange = 30f; // -Range of sensors for detecting obstacles
    private float motorTorque = 50f; // -Torque applied to the motors
    private float maxSteeringAngle = 30; // -Maximum steering angle for turning
    private float maxSpeed = 10f; // -Maximum speed of the robot
    private Rigidbody rb; // -Rigidbody for physics calculations
    private int roadLayer; // -Layer mask for detecting roads
    private float downwardAngle = 2f; // -Angle for downward sensor orientation
    private float ORSRange = 20f; // -Range for the obstacle right sensor

// private void AdjustSensorOrientations()
// {
//     float downwardAngle = 4f; // Angle to tilt sensors downward

//     // Set the front sensor to face slightly downward
//     FRS.localRotation = Quaternion.Euler(, 0.0, 0.1);

//     L1S.localRotation = Quaternion.Euler(downwardAngle, -5, 2);
//     L24.localRotation = Quaternion.Euler(downwardAngle, -2, 5);
//     L33.localRotation = Quaternion.Euler(downwardAngle, -15, 10);
    // L40.localRotation = Quaternion.Euler(downwardAngle, -15, 10);
//     // Configure the right sensors to face downward and at various rightward angles
//     R1S.localRotation = Quaternion.Euler(downwardAngle, 2, 0);
//     R2S.localRotation = Quaternion.Euler(downwardAngle, 5, 0);
//     R3S.localRotation = Quaternion.Euler(downwardAngle, 10, 0);

//     // Adjust the obstacle right sensor to face downward and point backwards
//     ORS.rotation = Quaternion.Euler(downwardAngle, 180, 0);
// }

// private void Startrobot()
// {
//     // Identify the layer used for detecting road surfaces
//     roadLayer = NameToLayer("Road");

//     // Align all sensors to their intended forward and downward positions
//     AdjustSensorOrientations();

//     // Retrieve the Rigidbody component for applying physics operations
//     rb = GetComponent<Rigidbody>();
// }





    private void RotateSensorsToFaceForwardAndDown()
    {
        float downAngle = 4f; // Downward angle adjustment

        // Adjust front sensor rotation
        FRS.localRotation = Quaternion.Euler(downAngle, 0, 0);

        // Adjust left sensors' rotation
        L1S.localRotation = Quaternion.Euler(downAngle, -2, 0);
        L2S.localRotation = Quaternion.Euler(downAngle, -5, 0);
        L3S.localRotation = Quaternion.Euler(downAngle, -10, 0);

        // Adjust right sensors' rotation
        R1S.localRotation = Quaternion.Euler(downAngle, 2, 0);
        R2S.localRotation = Quaternion.Euler(downAngle, 5, 0);
        R3S.localRotation = Quaternion.Euler(downAngle, 10, 0);

        // Adjust obstacle right sensor rotation
        ORS.rotation = Quaternion.Euler(downAngle, 180, 0);
    }

    private void Start()
    {
        // Get the road layer for detecting road surfaces
        roadLayer = LayerMask.NameToLayer("Road");

        // Initialize sensors to face forward and downward
        RotateSensorsToFaceForwardAndDown();

        // Get the Rigidbody component for applying forces
        rb = GetComponent<Rigidbody>();
    }


    private bool IsRightClear()
    // if right is clear the go staright with flow of sensors
    {
        return IsRoad(R1S) && IsRoad(R2S) && IsRoad(R3S);
    }

    private bool IsLeftClear()
    // if left is clear the go staright with flow of sensors
    {
        return IsRoad(L1S) && IsRoad(L2S) && IsRoad(L3S);
    }

    private void FixedUpdate()
    {
        // Handle robot steering and obstacle avoidance
        HandleSteeringAndObstacleAvoidance();

        // Update the wheel visuals to match their colliders
        UpdateWheel(FLC, FLT);
        UpdateWheel(FRC, FRT);
        UpdateWheel(RLC, RLT);
        UpdateWheel(RRC, RRT);
    }

    /// <summary>
    /// Moves the robot forward while maintaining speed limits.
    /// </summary>
    private void MoveForward()
    {
        // Check if the speed is below the maximum limit
        if (rb.velocity.magnitude < maxSpeed)
        {
            // Apply torque to all wheels to move forward
            FLC.motorTorque = motorTorque;
            FRC.motorTorque = motorTorque;
            RLC.motorTorque = motorTorque;
            RRC.motorTorque = motorTorque;
        }
        else
        {
            // Reverse if speed exceeds the limit
            Reverse();
        }

        // Apply a boost if moving forward
        if (rb.velocity.z > 0 && FLC.motorTorque > 0)
        {
            float nitro = 800; // Nitro boost torque
            FLC.motorTorque = nitro;
            FRC.motorTorque = nitro;
            RLC.motorTorque = nitro;
            RRC.motorTorque = nitro;
        }
    }

    private void SteerLeft()
    // TO debug to the left angele and move left when it turns
    {
        FLC.steerAngle = -maxSteeringAngle;
        FRC.steerAngle = -maxSteeringAngle;
    }

    private void SteerRight()
    {
        FLC.steerAngle = maxSteeringAngle;
        FRC.steerAngle = maxSteeringAngle;
    }
    /// <summary>
    /// Handles the robot's steering and avoids obstacles using sensors.
    /// </summary>

    // Right Sensors
    // bool right1Clear = Physics.Raycast(R1S.position, R1S.forward, out hit, 10f);
    // if (right1Clear)
    // {
    //     Debug.Log($"Right1 detected: {hit.collider.name}, distance: {hit.distance}");
    // }
    // else
    // {
    //     Debug.Log("Right1 did not detect anything.");
    // }
    private void HandleSteeringAndObstacleAvoidance()
    {
        if (IsFrontClear())
        {
            if (!IsLeftClear())
            {
                SteerRight();
            }
            if (!IsRightClear())
            {
                SteerLeft();
            }
            if (IsLeftClear() && IsRightClear())
            {
                ContinueStraight();
            }
            MoveForward();
        }
        else
        {
            if (IsLeftRoad() && !IsObstacleOnLeft())
            {
                SteerLeft();
            }
            else if (IsRightRoad() && !IsObstacleOnRight())
            {
                SteerRight();
            }
            else        
// {
//     // Evaluate wider road detection on both sides
//     if ((IsRightPathWider(RightSensor1) || IsRightPathWider(RightSensor2) || IsRightPathWider(RightSensor3)) &&
//         !(IsLeftPathWider(LeftSensor1) && IsLeftPathWider(LeftSensor2) && IsLeftPathWider(LeftSensor3)))
//     {
//         // Steer towards the right if the right side is wider
//         SteerRight();
//         motorPower = 1200f;
//     }
//     else if (!(IsRightPathWider(RightSensor1) && IsRightPathWider(RightSensor2) && IsRightPathWider(RightSensor3)) &&
//              (IsLeftPathWider(LeftSensor1) || IsLeftPathWider(LeftSensor2) || IsLeftPathWider(LeftSensor3)))
//     {
//         // Steer towards the left if the left side is wider
//         SteerLeft();
//         motorPower = 1200f;
//     }
//     else
//     {
//         // Reduce motor power if no wider path is detected
//         motorPower = 10f;
//     }

//     // Move forward after evaluating steering
//     MoveForward();

//     // Reset motor power to default for subsequent actions
//     motorPower = 50f;
// }

            {
                // Check for wider road detection
                if ((Widerightcheck(R1S) || Widerightcheck(R2S) || Widerightcheck(R3S)) &&
                    !(Wideleftcheck(L1S) && Wideleftcheck(L2S) && Wideleftcheck(L3S)))
                {
                    SteerRight();
                    motorTorque = 1200f;
                }
                else if (!(Widerightcheck(R1S) && Widerightcheck(R2S) && Widerightcheck(R3S)) &&
                         (Wideleftcheck(L1S) || Wideleftcheck(L2S) || Wideleftcheck(L3S)))
                {
                    SteerLeft();
                    motorTorque = 1200f;
                }
                else
                {
                    motorTorque = 10f;
                }
                MoveForward();
                motorTorque = 50f; // Reset motor torque
            }
        }
    }

    // Additional utility methods (e.g., for checking obstacles, steering, and updating wheels)
    // remain unchanged, with comments added above them to describe their purpose.

    // ...
    bool Widerightcheck(Transform sensor)
    {
        //Vector3 leftDirection = ORS.TransformDirection(Quaternion.Euler(downwardAngle, -70, 0) * Vector3.forward);
        Vector3 rightDirection = sensor.TransformDirection(Quaternion.Euler(downwardAngle, 60, 0) * Vector3.forward);
        RaycastHit hit;
        Physics.Raycast(sensor.position, rightDirection, out hit, ORSRange, -1, QueryTriggerInteraction.Ignore);
        if (hit.collider == null)
        {
            return false;
        }
        bool isRoad = hit.collider.gameObject.layer == roadLayer || hit.collider.gameObject.layer == LayerMask.NameToLayer("Obs");
        //Debug.DrawRay(sensor.position, rightDirection * ORSRange, isRoad ? Color.green : Color.red);
        return isRoad;
    }

/// <summary> procedure
/// This script controls the robot's movement sensors colliders and sensors for obstacle avoidance.
/// </summary>

    bool IsLeftRoad()
    // to the left road 
    {
        return IsRoad(L1S) || IsRoad(L2S) || IsRoad(L3S);
    }

    private bool IsRightRoad()
    // to the right road 
    {
        return IsRoad(R1S) || IsRoad(R2S) || IsRoad(R3S);
    }




    private void ContinueStraight()
    // to check if now obstacle in path and move forward
    {
        FLC.steerAngle = 0;
        FRC.steerAngle = 0;
    }
// for the track that follows and when it comes to turn then lightly it turna back and reverse
    private void Reverse()
    {
        FLC.motorTorque = -motorTorque ;
        FRC.motorTorque = -motorTorque ;
        RLC.motorTorque = -motorTorque ;
        RRC.motorTorque = -motorTorque ;
    }

//  if frnt is clear then the robo moves forward and follow the speed
    private bool IsFrontClear()
    {
        return IsRoad(FRS);
    }


    bool IsRoad(Transform sensor)
    {
        RaycastHit hit;
        Physics.Raycast(sensor.position, sensor.forward, out hit, sensorRange,-1,QueryTriggerInteraction.Ignore);
        if (hit.collider == null)
        {
            return false;
        }
        bool isRoad = hit.collider.gameObject.layer == roadLayer;//|| hit.collider.gameObject.layer == LayerMask.NameToLayer("Default");
        //Debug.DrawRay(sensor.position, sensor.forward * sensorRange, isRoad ? Color.green : Color.red);
        return isRoad;
    }

    private bool IsObstacleOnLeft()
    // obrstacle set up on left then work accordingly 
    {
        return CheckObstacle(L1S) || CheckObstacle(L2S) || CheckObstacle(L3S);
    }

    private bool IsObstacleOnRight()
    // obrstacle set up on right then work accordingly 
    {
        return CheckObstacle(R1S) || CheckObstacle(R2S) || CheckObstacle(R3S) || CheckObstacle(ORS);
    }

    private bool CheckObstacle(Transform sensor)
    {
        RaycastHit hit;
        bool isObstacle = Physics.Raycast(sensor.position, sensor.forward, out hit, sensorRange) &&
                          hit.collider.gameObject.layer == LayerMask.NameToLayer("Obs"); 

        //Debug.DrawRay(sensor.position, sensor.forward * sensorRange, isObstacle ? Color.red : Color.green);
        return isObstacle;
    }

    private void UpdateWheel(WheelCollider collider, Transform transform)
    {
        Vector3 pos;
        Quaternion rot;
        collider.GetWorldPose(out pos, out rot);
        transform.position = pos;
        transform.rotation = rot;
    }



// wide track to detect senors on the way to track and follow conditions
    bool Wideleftcheck(Transform sensor)
    {
        Vector3 leftDirection = sensor.TransformDirection(Quaternion.Euler(downwardAngle, -60, 0) * Vector3.forward);
        //Vector3 rightDirection = ORS.TransformDirection(Quaternion.Euler(downwardAngle, 70, 0) * Vector3.forward);
        RaycastHit hit;
        Physics.Raycast(sensor.position, leftDirection, out hit, ORSRange, -1, QueryTriggerInteraction.Ignore);
        if (hit.collider == null)
        {
            return false;
        }
        bool isRoad = hit.collider.gameObject.layer == roadLayer || hit.collider.gameObject.layer == LayerMask.NameToLayer("Obs");
        //Debug.DrawRay(sensor.position, leftDirection * ORSRange, isRoad ? Color.green : Color.red);
        return isRoad;
    }

}

