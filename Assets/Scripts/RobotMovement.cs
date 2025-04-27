using UnityEngine;
using System.Collections;
using System.Collections.Generic;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(AStarNavigation))] // Assuming AStarNavigation handles pathfinding
public class RobotMovement : MonoBehaviour
{
    [Header("Movement Parameters")]
    [SerializeField] private float maxLinearSpeed = 2.0f; // m/s
    [SerializeField] private float maxAngularSpeed = 90.0f; // degrees/s
    [SerializeField] private float wheelRadius = 0.1f; // meters
    [SerializeField] private float wheelBase = 0.5f; // meters (distance between left and right wheels)
    [SerializeField] private float acceleration = 1.0f; // m/s^2
    [SerializeField] private float angularAcceleration = 45.0f; // deg/s^2
    [SerializeField] private float stoppingDistance = 0.5f; // How close to get to waypoint

    [Header("Sensors")]
    [SerializeField] private float lidarRange = 10.0f;
    [SerializeField] private int lidarResolution = 36; // Number of rays (e.g., 36 for 10-degree increments)
    [SerializeField] private Transform visualSensorOrigin; // Empty GameObject representing the visual sensor position/direction
    [SerializeField] private float visualSensorRange = 15.0f;
    [SerializeField] private LayerMask obstacleLayerMask; // Layers considered obstacles by LIDAR and pathfinding
    [SerializeField] private LayerMask bombLayerMask; // Layer containing bombs

    [Header("References")]
    [SerializeField] private Transform[] wheels; // Optional: For visual rotation

    // Internal State
    private Rigidbody rb;
    private AStarNavigation pathfinder;
    private List<Vector3> currentPath = null;
    private int currentWaypointIndex = 0;
    private bool isFollowingPath = false;
    private Dictionary<Vector3, float> lidarReadings = new Dictionary<Vector3, float>(); // Direction vector -> distance
    private GameObject detectedBomb = null;
    private Vector3 targetPosition; // Current movement target

    // DDMR Kinematics related (simplified)
    private float currentLeftWheelSpeed = 0f;
    private float currentRightWheelSpeed = 0f;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        pathfinder = GetComponent<AStarNavigation>();
        if (visualSensorOrigin == null)
        {
            visualSensorOrigin = transform; // Default to robot's main transform if not set
            Debug.LogWarning("Visual Sensor Origin not set, defaulting to robot transform.");
        }
    }

    void Update()
    {
        SimulateLidar();
        DetectBombWithRaycast();
        RotateWheelsVisual(); // Optional visual flair
    }

    void FixedUpdate()
    {
        if (isFollowingPath && currentPath != null && currentWaypointIndex < currentPath.Count)
        {
            ApplyMovementPhysics();
        }
        else
        {
            // Gradually stop if not actively following a path
            ApplyBrakingPhysics();
        }
    }

    // --- Public Methods ---

    /// <summary>
    /// Starts pathfinding to the destination and initiates movement.
    /// </summary>
    public void MoveTo(Vector3 destination)
    {
        targetPosition = destination; // Store the final destination
        Debug.Log($"RobotMovement: Finding path to {destination}");

        // Call FindPath assuming it returns the path directly
        List<Vector3> foundPath = pathfinder.FindPath(transform.position, destination);

        if (foundPath != null && foundPath.Count > 0)
        {
            currentPath = foundPath;
            currentWaypointIndex = 0;
            isFollowingPath = true;
            Debug.Log($"RobotMovement: Path found with {foundPath.Count} waypoints. Starting movement.");
        }
        else
        {
            Debug.LogWarning($"RobotMovement: Pathfinding failed to {destination}.");
            StopMoving(); // Stop if pathfinding fails
        }
    }


    /// <summary>
    /// Stops all movement and clears the current path.
    /// </summary>
    public void StopMoving()
    {
        Debug.Log("RobotMovement: StopMoving called.");
        isFollowingPath = false;
        currentPath = null;
        currentWaypointIndex = 0;
        // Let ApplyBrakingPhysics handle gradual stop
    }

    /// <summary>
    /// Returns the latest simulated LIDAR readings.
    /// </summary>
    public Dictionary<Vector3, float> GetLidarReadings()
    {
        // Return a copy to prevent external modification
        return new Dictionary<Vector3, float>(lidarReadings);
    }

    /// <summary>
    /// Checks if a bomb is currently detected by the visual sensor.
    /// </summary>
    public bool HasDetectedBomb()
    {
        // Ensure detectedBomb is still valid (might have been destroyed/defused elsewhere)
        if (detectedBomb != null && !detectedBomb.activeInHierarchy)
        {
            detectedBomb = null;
        }
        return detectedBomb != null;
    }

    /// <summary>
    /// Gets the currently detected bomb GameObject.
    /// </summary>
    public GameObject GetDetectedBomb()
    {
        // Ensure detectedBomb is still valid
        if (detectedBomb != null && !detectedBomb.activeInHierarchy)
        {
            detectedBomb = null;
        }
        return detectedBomb;
    }

    /// <summary>
    /// Checks if the robot is currently trying to follow a path.
    /// </summary>
    public bool IsFollowingPath()
    {
        return isFollowingPath;
    }

    /// <summary>
    /// Simulates getting a distance reading to a known beacon position.
    /// In a real scenario, this would involve communication/sensor hardware.
    /// </summary>
    public float GetDistanceToBeacon(Vector3 beaconPosition)
    {
        float distance = Vector3.Distance(transform.position, beaconPosition);
        // Add some noise to simulate real sensor inaccuracy
        distance += Random.Range(-0.1f, 0.1f);
        return Mathf.Max(0, distance); // Ensure non-negative distance
    }


    // --- Internal Movement Logic ---

    private void ApplyMovementPhysics()
    {
        if (currentPath == null || currentWaypointIndex >= currentPath.Count)
        {
            StopMoving();
            return;
        }

        Vector3 nextWaypoint = currentPath[currentWaypointIndex];
        Vector3 directionToWaypoint = (nextWaypoint - transform.position).normalized;
        float distanceToWaypoint = Vector3.Distance(transform.position, nextWaypoint);

        // Check if close enough to the current waypoint
        if (distanceToWaypoint < stoppingDistance)
        {
            currentWaypointIndex++;
            if (currentWaypointIndex >= currentPath.Count)
            {
                // Reached the end of the path
                Debug.Log("RobotMovement: Reached final destination.");
                StopMoving();
                return;
            }
            else
            {
                // Move to the next waypoint
                nextWaypoint = currentPath[currentWaypointIndex];
                directionToWaypoint = (nextWaypoint - transform.position).normalized;
                Debug.Log($"RobotMovement: Moving to waypoint {currentWaypointIndex + 1}/{currentPath.Count}");
            }
        }

        // --- Simplified DDMR Control ---
        // Calculate desired linear and angular velocity towards the waypoint
        float targetAngle = Mathf.Atan2(directionToWaypoint.x, directionToWaypoint.z) * Mathf.Rad2Deg;
        float angleDifference = Mathf.DeltaAngle(transform.eulerAngles.y, targetAngle);

        // Target speeds based on angle difference
        float targetAngularVelocity = Mathf.Clamp(angleDifference, -maxAngularSpeed, maxAngularSpeed);
        // Reduce linear speed when turning sharply
        float turnReduction = Mathf.Clamp01(1.0f - Mathf.Abs(angleDifference) / 90.0f); // Reduce speed more for >90 deg turns
        float targetLinearVelocity = maxLinearSpeed * turnReduction;

        // Apply acceleration limits (simplified)
        Vector3 currentVelocity = rb.velocity;
        Vector3 targetVelocityVector = directionToWaypoint * targetLinearVelocity;
        Vector3 newVelocity = Vector3.MoveTowards(currentVelocity, targetVelocityVector, acceleration * Time.fixedDeltaTime);

        float currentAngularVelY = rb.angularVelocity.y * Mathf.Rad2Deg;
        float newAngularVelY = Mathf.MoveTowardsAngle(currentAngularVelY, targetAngularVelocity, angularAcceleration * Time.fixedDeltaTime);

        // Apply to Rigidbody
        rb.velocity = new Vector3(newVelocity.x, rb.velocity.y, newVelocity.z); // Keep existing Y velocity
        rb.angularVelocity = new Vector3(0, newAngularVelY * Mathf.Deg2Rad, 0);

        // --- TODO: Calculate target wheel speeds based on targetLinearVelocity and targetAngularVelocity ---
        // This part involves the differential drive inverse kinematics equations.
        // V_linear = (V_right + V_left) / 2
        // V_angular = (V_right - V_left) / wheelBase
        // Where V_right/left are linear speeds of the wheels (omega * radius)
        // For simplicity here, we are directly controlling Rigidbody velocity/angularVelocity.
        // A more accurate simulation would apply forces/torques to wheel colliders.
    }

    private void ApplyBrakingPhysics()
    {
        // Gradually reduce velocity and angular velocity
        Vector3 brakingVelocity = Vector3.MoveTowards(rb.velocity, Vector3.zero, acceleration * 2f * Time.fixedDeltaTime); // Brake harder
        Vector3 brakingAngularVelocity = Vector3.MoveTowards(rb.angularVelocity, Vector3.zero, angularAcceleration * 2f * Mathf.Deg2Rad * Time.fixedDeltaTime);

        rb.velocity = brakingVelocity;
        rb.angularVelocity = brakingAngularVelocity;

        // Reset wheel speeds (if using direct wheel control)
        currentLeftWheelSpeed = 0f;
        currentRightWheelSpeed = 0f;
    }


    // --- Sensor Simulation ---

    private void SimulateLidar()
    {
        lidarReadings.Clear();
        float angleStep = 360.0f / lidarResolution;

        for (int i = 0; i < lidarResolution; i++)
        {
            float currentAngle = i * angleStep;
            Quaternion rotation = Quaternion.Euler(0, currentAngle, 0);
            Vector3 direction = rotation * transform.forward;
            RaycastHit hit;

            float distance = lidarRange; // Default distance if nothing hit

            if (Physics.Raycast(transform.position, direction, out hit, lidarRange, obstacleLayerMask))
            {
                distance = hit.distance;
                Debug.DrawLine(transform.position, hit.point, Color.red, 0.01f); // Visualize hit
            }
            else
            {
                Debug.DrawRay(transform.position, direction * lidarRange, Color.green, 0.01f); // Visualize miss
            }
            // Store direction vector and distance
            lidarReadings[direction] = distance;
        }
    }

    private void DetectBombWithRaycast()
    {
        if (visualSensorOrigin == null) return;

        Vector3 origin = visualSensorOrigin.position;
        Vector3 direction = visualSensorOrigin.forward;
        RaycastHit hit;

        GameObject previouslyDetected = detectedBomb;
        detectedBomb = null; // Reset detection each frame

        if (Physics.Raycast(origin, direction, out hit, visualSensorRange, bombLayerMask))
        {
            Debug.DrawLine(origin, hit.point, Color.yellow); // Yellow line to hit point

            if (hit.collider.CompareTag("Bomb")) // Ensure bombs have "Bomb" tag
            {
                detectedBomb = hit.collider.gameObject;
                // Debug.Log("Bomb detected: " + detectedBomb.name);

                // --- Simple Defusal Logic ---
                // Check distance from the robot's main transform
                if (Vector3.Distance(transform.position, detectedBomb.transform.position) < 1.5f) // Defusal range
                {
                    DefuseBomb(detectedBomb);
                }
            }
        }
        else
        {
            Debug.DrawRay(origin, direction * visualSensorRange, Color.cyan); // Cyan line for full range if no hit
        }

        // Optional: Log detection changes
        // if (previouslyDetected != null && detectedBomb == null) Debug.Log("Bomb lost from sight.");
        // if (previouslyDetected == null && detectedBomb != null) Debug.Log("Bomb acquired: " + detectedBomb.name);
    }

    private void DefuseBomb(GameObject bombToDefuse)
    {
        if (bombToDefuse == null || !bombToDefuse.activeInHierarchy) return;

        Bomb bombScript = bombToDefuse.GetComponent<Bomb>();
        if (bombScript != null && !bombScript.isDefused)
        {
            Debug.Log($"Attempting to defuse bomb: {bombToDefuse.name} in zone {bombScript.locationArea}");
            bombScript.Defuse(); // Call the Defuse method on the Bomb script

            // Notify BombCounter (assuming Bomb script doesn't do this)
            BombCounter.instance?.DefuseBomb();

            detectedBomb = null; // Clear detection after successful defusal attempt
        }
        else if (bombScript == null)
        {
            Debug.LogError($"Bomb {bombToDefuse.name} is missing Bomb script!", bombToDefuse);
        }
        else if (bombScript.isDefused)
        {
            // Already defused, clear detection
            detectedBomb = null;
        }
    }

    // --- Optional Visuals ---
    private void RotateWheelsVisual()
    {
        if (wheels == null || wheels.Length == 0) return;

        // Simplified wheel rotation based on Rigidbody velocity
        float linearSpeed = new Vector3(rb.velocity.x, 0, rb.velocity.z).magnitude;
        float angularSpeedRad = rb.angularVelocity.y; // Radians per second

        // Calculate rotation speed based on linear and angular velocity
        // This is a simplification. Accurate calculation needs wheel speeds.
        float wheelCircumference = 2 * Mathf.PI * wheelRadius;
        float forwardRotationSpeed = (linearSpeed / wheelCircumference) * 360.0f * Time.deltaTime; // Degrees per second -> degrees per frame
        float turningRotationSpeed = (angularSpeedRad * (wheelBase / 2) / wheelCircumference) * 360.0f * Time.deltaTime;

        foreach (Transform wheel in wheels)
        {
            // Basic assumption: inner wheels turn less during rotation
            // This needs refinement based on actual wheel positions relative to center
            bool isLeftWheel = wheel.localPosition.x < 0; // Simple check
            float turnSpeed = isLeftWheel ? -turningRotationSpeed : turningRotationSpeed;

            wheel.Rotate(Vector3.right, forwardRotationSpeed + turnSpeed, Space.Self);
        }
    }

    // Gizmos for path visualization
    void OnDrawGizmos()
    {
        if (currentPath != null && currentPath.Count > 0)
        {
            Gizmos.color = Color.magenta;
            for (int i = 0; i < currentPath.Count - 1; i++)
            {
                Gizmos.DrawLine(currentPath[i], currentPath[i + 1]);
                Gizmos.DrawSphere(currentPath[i], 0.1f);
            }
            Gizmos.DrawSphere(currentPath[currentPath.Count - 1], 0.1f);

            // Highlight next waypoint
            if (currentWaypointIndex < currentPath.Count)
            {
                Gizmos.color = Color.cyan;
                Gizmos.DrawWireSphere(currentPath[currentWaypointIndex], stoppingDistance);
            }
        }
    }
}