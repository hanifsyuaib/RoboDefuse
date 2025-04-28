using UnityEngine;

public class SensorSystem : MonoBehaviour
{
    public float detectionRange = 2f; // Range of the Raycast
    public int rayCount = 10; // Number of rays to cast around the robot
    public Color rayColor = Color.green;  // Color for the Raycast

    public float avoidanceTurnSpeed = 45f;  // How quickly the robot will turn to avoid obstacles

    public LayerMask obstacleLayer;  // Layer mask to filter out non-obstacle objects

    void Update()
    {
        bool obstacleDetected = false;
        Vector3 avoidanceDirection = Vector3.zero; // Default: no avoidance

        // Calculate the angle step between rays to cover a full circle (360 degrees)
        float angleStep = 360f / rayCount;

        // Loop through the range of angles to cast multiple rays around the robot
        for (int i = 0; i < rayCount; i++)
        {
            // Calculate the angle for the current ray (360-degree coverage)
            float currentAngle = angleStep * i;

            // Convert angle to radians and calculate the direction of the ray
            Vector3 direction = Quaternion.Euler(0, currentAngle, 0) * transform.forward;

            // Visualize the Raycast in the Scene view using Debug.DrawRay
            Debug.DrawRay(transform.position, direction * detectionRange, rayColor);

            // Check for obstacles in the direction of the ray
            RaycastHit hit;
            if (Physics.Raycast(transform.position, direction, out hit, detectionRange, obstacleLayer))
            {
                // If an obstacle is detected
                Debug.Log("Obstacle detected!");

                // Mark that an obstacle is detected and calculate the avoidance direction
                obstacleDetected = true;
                avoidanceDirection += hit.normal; // Avoidance vector is the surface normal
            }
        }

        // If an obstacle is detected, adjust the robot's movement to avoid it
        if (obstacleDetected)
        {
            RobotController controller = GetComponent<RobotController>();
            if (controller != null)
            {
                // The robot will turn away from the obstacle (avoidance direction)
                avoidanceDirection.Normalize();  // Make sure the vector has a unit length

                // Determine whether to turn left or right based on avoidance direction
                float turnAngle = Vector3.SignedAngle(transform.forward, avoidanceDirection, Vector3.up);

                // Adjust angular velocity smoothly to avoid the obstacle
                float targetAngularVelocity = Mathf.Sign(turnAngle) * avoidanceTurnSpeed;
                controller.SetTargetMovement(0f, targetAngularVelocity);  // Set the angular movement to avoid the obstacle
            }
        }
        else
        {
            // If no obstacle detected, continue forward movement towards target
            RobotController controllerNoObstacle = GetComponent<RobotController>();
            if (controllerNoObstacle != null)
            {
                controllerNoObstacle.SetTargetMovement(1f, 0f); // Move forward if no obstacle
            }
        }
    }
}
