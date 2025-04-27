using UnityEngine;
using System.Collections;
using System.Collections.Generic; // Added for Dictionary

[RequireComponent(typeof(Rigidbody))] // Ensures a Rigidbody is attached
public class RobotMovement : MonoBehaviour
{
    [Header("Movement Settings")]
    [SerializeField] private float moveSpeed = 5f;
    [SerializeField] private float rotationSpeed = 120f; // Degrees per second
    [SerializeField] private float stoppingDistance = 0.2f; // Increased slightly
    [SerializeField] private float pathUpdateRate = 0.5f;
    [SerializeField] private float angleThreshold = 5.0f; // Degrees: How close the angle needs to be to move forward

    [Header("DDMR Settings")]
    [SerializeField] private float wheelRadius = 0.25f; // Ensure this matches your wheel model for visual rotation speed
    [SerializeField] private float trackWidth = 1.0f;
    [SerializeField] [Tooltip("Element 0: Front Left wheel\nElement 1: Rear Left wheel\nElement 2: Front Right wheel\nElement 3: Rear Right wheel")]
    private Transform[] wheels = new Transform[4];
    // [SerializeField] private float wheelRotationSpeed = 200f; // Less relevant now, wheelRadius is used

    [Header("Sensor Settings")]
    [SerializeField] private float lidarRange = 10f;
    [SerializeField] private int lidarRayCount = 36;
    [SerializeField] private LayerMask lidarLayerMask;
    [SerializeField] private float visualSensorRange = 5f;
    [SerializeField] private LayerMask bombLayerMask;
    [SerializeField] private Transform lidarOrigin;
    [SerializeField] private Transform visualSensorOrigin;

    [Header("References")]
    [SerializeField] private AStarNavigation pathfinder;
    [SerializeField] private Transform target; // Optional target transform

    private List<Vector3> path;
    private int currentWaypoint = 0;
    private bool isFollowingPath = false;
    private float lastPathUpdateTime = 0f;
    private float leftWheelSpeed = 0f;
    private float rightWheelSpeed = 0f;
    private GameObject detectedBomb = null;
    private Dictionary<Vector3, float> lidarReadings = new Dictionary<Vector3, float>();

    // Add reference to the Rigidbody
    private Rigidbody rb;

    // State flags for collision handling
    private bool isBlockedByWall = false;
    private float timeLastBlocked = -1f; // Time when blockage was last detected
    private float evasiveRotationTimer = 0f; // Timer for how long to rotate evasively

    // --- NEW: Stuck Detection Variables ---
    private float lastProgressCheckTime;
    private const float PROGRESS_CHECK_INTERVAL = 5f; // Check every 5 seconds
    private const float MIN_PROGRESS_FORWARD_SPEED = 0.05f; // Keep this
    private const float MIN_PROGRESS_DISTANCE_SQR = 0.01f * 0.01f; // Min distance squared (more efficient)

    // --- NEW: State for step assist attempt ---
    private bool isAttemptingStepAssist = false;
    private const float STEP_ASSIST_FORCE_MAGNITUDE = 5.0f; // Force to apply when stuck (tune this)
    private const ForceMode STEP_ASSIST_FORCE_MODE = ForceMode.Force; // Use Impulse for a quick boost

    private const float MIN_ROTATION_SPEED_FOR_STUCK_IGNORE = 10.0f; // Degrees per second

    // Define wheel positions enum for code clarity
    private enum WheelPosition
    {
        FrontLeft = 0,
        RearLeft = 1,
        FrontRight = 2,
        RearRight = 3
    }

    private void Start()
    {
        // Get the Rigidbody component
        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            Debug.LogError("Rigidbody component not found on Robot!", this.gameObject);
        }
        else
        {
            // Configure Rigidbody settings
            rb.isKinematic = false; // IMPORTANT: Must NOT be kinematic for collisions to work this way
            rb.useGravity = true; // Or false if your ground keeps it up reliably
            // Freeze rotation on axes we don't control manually to prevent tipping
            rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
        }

        if (pathfinder == null)
        {
            pathfinder = FindObjectOfType<AStarNavigation>();
            if (pathfinder == null)
            {
                Debug.LogError("No A* pathfinder found in the scene!");
            }
        }

        // Set default positions for sensor origins if not assigned
        if (lidarOrigin == null) lidarOrigin = transform;
        if (visualSensorOrigin == null) visualSensorOrigin = transform;

        // --- Initialize Stuck Detection ---
        lastProgressCheckTime = Time.time;
        isAttemptingStepAssist = false;
    }

    private void FixedUpdate() // Use FixedUpdate for Rigidbody physics
    {
        // Path following logic - Call the movement logic from FixedUpdate
        if (isFollowingPath)
        {
            ApplyMovementPhysics(); // Renamed FollowPath logic to ApplyMovementPhysics
        }
        // Legacy target following (optional) - Needs modification if used
        else if (target != null)
        {
            // Path calculation can still be in Update or less frequent
            // But movement application should be here if using physics
            // If not following path and no target, ensure wheels stop (and physics stops)
             leftWheelSpeed = 0f;
             rightWheelSpeed = 0f;
             // Ensure Rigidbody stops if not kinematic
             if (rb != null && !rb.isKinematic)
             {
                 rb.velocity = Vector3.zero;
                 rb.angularVelocity = Vector3.zero;
             }
        }
        else // If not following a path and no target, ensure wheels stop (and physics stops)
        {
             leftWheelSpeed = 0f;
             rightWheelSpeed = 0f;
             // Ensure Rigidbody stops if not kinematic
             if (rb != null && !rb.isKinematic)
             {
                 rb.velocity = Vector3.zero;
                 rb.angularVelocity = Vector3.zero;
             }
        }
    }

     private void Update() // Keep Update for non-physics logic like input, sensors, path calculation timing, visuals
     {
         // --- Path Calculation Timing (if using target) ---
         if (!isFollowingPath && target != null)
         {
             if (Time.time - lastPathUpdateTime > pathUpdateRate)
             {
                 lastPathUpdateTime = Time.time;
                 FindPath(target.position);
             }
         }

         // --- Sensor Logic ---
         SimulateLidar();
         DetectBombWithRaycast();

         // --- Visual Updates ---
         RotateWheels(); // Keep visual updates in Update
     }


    public void FindPath(Vector3 destination)
    {
        Debug.Log($"FindPath called from {transform.position} to destination {destination}");

        if (pathfinder == null) {
             Debug.LogError("Pathfinder is null in FindPath!");
             isFollowingPath = false;
             return;
        }

        path = pathfinder.FindPath(transform.position, destination);
        if (path != null && path.Count > 0)
        {
            Debug.Log($"Path found with {path.Count} waypoints. First waypoint: {path[0]}");
            currentWaypoint = 0;
            isFollowingPath = true;
            // Ensure Rigidbody doesn't keep sliding from previous movement
            if(rb != null) {
                rb.velocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
            }
        }
        else
        {
            Debug.LogWarning($"No path found to destination {destination}");
            isFollowingPath = false;
            leftWheelSpeed = 0f; // Stop wheels if no path
            rightWheelSpeed = 0f;
        }
    }

    // Renamed from FollowPath and moved logic to FixedUpdate context
    private void ApplyMovementPhysics()
    {
        if (path == null || currentWaypoint >= path.Count)
        {
            // Stop physics movement
            if (isFollowingPath && rb != null) { // Only stop if we were previously following
                 rb.velocity = Vector3.zero;
                 rb.angularVelocity = Vector3.zero;
            }
            isFollowingPath = false;
            leftWheelSpeed = 0f;
            rightWheelSpeed = 0f;
            // Debug.Log("ApplyMovementPhysics stopped - path complete or invalid"); // Less verbose log
            return;
        }

        // --- Target Waypoint ---
        // Use Rigidbody position for calculations
        Vector3 currentPosition = rb.position;
        Vector3 waypointPosition = new Vector3(path[currentWaypoint].x, currentPosition.y, path[currentWaypoint].z); // Keep robot on its current Y plane
        float distToWaypoint = Vector3.Distance(currentPosition, waypointPosition);

        // --- Waypoint Reached Check ---
        if (distToWaypoint < stoppingDistance)
        {
            // Debug.Log($"Reached waypoint {currentWaypoint} ({waypointPosition}). Distance: {distToWaypoint}"); // Less verbose
            currentWaypoint++;
            if (currentWaypoint >= path.Count)
            {
                Debug.Log("Path complete!");
                 // Stop physics movement
                 if (rb != null) {
                     rb.velocity = Vector3.zero;
                     rb.angularVelocity = Vector3.zero;
                 }
                isFollowingPath = false;
                leftWheelSpeed = 0f;
                rightWheelSpeed = 0f;
                return; // Exit
            }
            else
            {
                 // Update waypointPosition for the next target in the same frame if needed
                 waypointPosition = new Vector3(path[currentWaypoint].x, currentPosition.y, path[currentWaypoint].z);
                 // Debug.Log($"Moving to next waypoint {currentWaypoint} ({waypointPosition})"); // Less verbose
            }
            isAttemptingStepAssist = false;
            return;
        }

        // --- Evasion Logic (Check BEFORE rotation/movement) ---
        if (isBlockedByWall && evasiveRotationTimer > 0)
        {
            Debug.Log("Blocked by wall");
            // --- Evasive Action: Blocked by Wall ---
            // Rotate away (e.g., turn right consistently)
            float angularVelocityTarget = rotationSpeed; // Rotate right
            Quaternion deltaRotation = Quaternion.Euler(0f, angularVelocityTarget * Time.fixedDeltaTime, 0f);
            rb.MoveRotation(rb.rotation * deltaRotation); // Use MoveRotation for simplicity here

            // Add a small backward velocity to help dislodge
            Vector3 backwardVelocity = -transform.forward * (moveSpeed * 0.3f); // Move backward slowly
            backwardVelocity.y = rb.velocity.y; // Preserve gravity effect
            rb.velocity = backwardVelocity;

            // Set wheel speeds for visual rotation (backward)
            leftWheelSpeed = -moveSpeed * 0.3f; // Adjust visual speed if needed
            rightWheelSpeed = -moveSpeed * 0.3f;

            // Decrease timer
            evasiveRotationTimer -= Time.fixedDeltaTime;
            isAttemptingStepAssist = false; // Ensure assist is off during evasion

            // --- Check if evasion just finished ---
            if (evasiveRotationTimer <= 0)
            {
                Debug.Log("Evasive rotation finished.");
                isBlockedByWall = false; // Reset blocked state HERE
                evasiveRotationTimer = 0f;
                rb.angularVelocity = Vector3.zero; // Stop rotation explicitly
                // Stop backward movement
                rb.velocity = new Vector3(0, rb.velocity.y, 0);

                // Optional: Force path recalculation
                if (path != null && currentWaypoint < path.Count)
                {
                    Debug.Log("Forcing path recalculation after evasion.");
                    FindPath(path[path.Count - 1]);
                }
            }
            return; // IMPORTANT: Do not proceed to normal rotation/movement during evasion
        }

        // --- Normal Rotation & Movement Logic ---
        Vector3 directionToWaypoint = (waypointPosition - currentPosition).normalized;
        if (directionToWaypoint != Vector3.zero)
        {
            Quaternion targetRotation = Quaternion.LookRotation(directionToWaypoint);
            float angleDifference = Quaternion.Angle(rb.rotation, targetRotation);

            if (angleDifference > angleThreshold)
            {
                // --- Rotate towards waypoint ---
                Vector3 cross = Vector3.Cross(transform.forward, directionToWaypoint);
                float turnDirection = Mathf.Sign(cross.y);
                float angularVelocityTarget = turnDirection * rotationSpeed;
                Quaternion deltaRotation = Quaternion.Euler(0f, angularVelocityTarget * Time.fixedDeltaTime, 0f);
                rb.MoveRotation(rb.rotation * deltaRotation); // Use MoveRotation for path alignment

                // Stop forward/backward velocity while rotating
                rb.velocity = new Vector3(0, rb.velocity.y, 0);

                // Set wheel speeds for visual rotation
                leftWheelSpeed = -angularVelocityTarget * 0.1f;
                rightWheelSpeed = angularVelocityTarget * 0.1f;
                isAttemptingStepAssist = false; // Reset assist state during rotation
            }
            else // Angle is good, move forward
            {
                // --- Move forward using Velocity ---
                // Dampen any residual angular velocity from rotation phase
                rb.angularVelocity *= 0.8f; // Adjust damping factor if needed

                float forwardSpeedTarget = moveSpeed;
                Vector3 targetVelocity = transform.forward * forwardSpeedTarget;
                targetVelocity.y = rb.velocity.y; // Preserve vertical velocity
                rb.velocity = targetVelocity; // Set velocity directly

                // Set wheel speeds
                leftWheelSpeed = forwardSpeedTarget;
                rightWheelSpeed = forwardSpeedTarget;

                // Check if stuck ONLY when trying to move forward
                CheckIfStuck();
            }
        }
        else // Should not happen if waypoint logic is correct, but handle defensively
        {
            rb.velocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
            leftWheelSpeed = 0f;
            rightWheelSpeed = 0f;
            isAttemptingStepAssist = false;
        }


    }

    // --- Sensor Methods ---
    private void SimulateLidar()
    {
        lidarReadings.Clear();
        float angleStep = 360f / lidarRayCount;
        Vector3 origin = lidarOrigin != null ? lidarOrigin.position : transform.position; // Use assigned origin or fallback

        for (int i = 0; i < lidarRayCount; i++)
        {
            float angle = i * angleStep;
            // Rotate around the origin's forward direction
            Quaternion rotation = Quaternion.Euler(0, angle, 0);
            Vector3 direction = rotation * lidarOrigin.forward; // Use origin's forward

            RaycastHit hit;
            float distance = lidarRange;

            if (Physics.Raycast(origin, direction, out hit, lidarRange, lidarLayerMask))
            {
                distance = hit.distance;
                Debug.DrawLine(origin, hit.point, Color.red); // Hit = red
            }
            else
            {
                Debug.DrawLine(origin, origin + direction * lidarRange, Color.green); // No hit = green
            }
            // Store direction relative to the robot's current orientation for consistency
            lidarReadings[transform.InverseTransformDirection(direction)] = distance;
        }
    }

    private void DetectBombWithRaycast()
    {
        if (visualSensorOrigin == null) return; // Safety check

        Vector3 origin = visualSensorOrigin.position;
        Vector3 direction = visualSensorOrigin.forward; // Use sensor's forward direction
        RaycastHit hit;

        // Reset detection each frame before raycasting
        GameObject previouslyDetected = detectedBomb;
        detectedBomb = null;

        if (Physics.Raycast(origin, direction, out hit, visualSensorRange, bombLayerMask))
        {
            // Visualize hit
            Debug.DrawLine(origin, hit.point, Color.yellow); // Yellow line to hit point

            // Check if the hit object has the "Bomb" tag (ensure your bombs have this tag)
            if (hit.collider.CompareTag("Bomb")) // Use CompareTag for efficiency
            {
                detectedBomb = hit.collider.gameObject;
                // Debug.Log("Bomb detected: " + detectedBomb.name); // Less verbose

                // --- Simple Defusal Logic ---
                // Check distance from the robot's main transform (or sensor origin, depending on desired behavior)
                if (Vector3.Distance(transform.position, detectedBomb.transform.position) < 1.5f) // Defusal range
                {
                    DefuseBomb(detectedBomb);
                }
            }
        }
        else
        {
            // Visualize miss
            Debug.DrawRay(origin, direction * visualSensorRange, Color.cyan); // Cyan line for full range if no hit
        }

        // Optional: Log if detection status changed
        // if (previouslyDetected != null && detectedBomb == null) Debug.Log("Bomb lost from sight.");
        // if (previouslyDetected == null && detectedBomb != null) Debug.Log("Bomb acquired: " + detectedBomb.name);
    }


    private void DefuseBomb(GameObject bombToDefuse)
    {
        // Ensure bomb hasn't already been defused/destroyed elsewhere
        if (bombToDefuse == null || !bombToDefuse.activeInHierarchy) return;

        Bomb bombScript = bombToDefuse.GetComponent<Bomb>();
        // Check if the bomb script exists and is not already defused
        if (bombScript != null && !bombScript.isDefused) // Assuming Bomb script has an 'isDefused' flag
        {
             Debug.Log("Attempting to defuse bomb: " + bombToDefuse.name + " in zone " + bombScript.locationArea);
             bombScript.Defuse(); // Call the Defuse method on the Bomb script

             // Optionally, notify BombCounter if the Bomb script doesn't do it
             // BombCounter.instance?.DefuseBomb(); // Make sure BombCounter exists and has this method

             detectedBomb = null; // Clear detection after successful defusal attempt
        } else if (bombScript == null) {
             Debug.LogError($"Bomb {bombToDefuse.name} is missing Bomb script!", bombToDefuse);
        } else if (bombScript.isDefused) {
             // Bomb already defused, maybe clear detection?
             // Debug.Log($"Bomb {bombToDefuse.name} was already defused.");
             detectedBomb = null;
        }
    }

    // --- Utility Methods ---
    public bool HasDetectedBomb()
    {
        // Ensure detectedBomb is still valid (might have been destroyed/defused)
        if (detectedBomb != null && !detectedBomb.activeInHierarchy)
        {
            detectedBomb = null;
        }
        return detectedBomb != null;
    }

    public GameObject GetDetectedBomb()
    {
         // Ensure detectedBomb is still valid
        if (detectedBomb != null && !detectedBomb.activeInHierarchy)
        {
            detectedBomb = null;
        }
        return detectedBomb;
    }

    public Dictionary<Vector3, float> GetLidarReadings()
    {
        // Return a copy to prevent external modification
        return new Dictionary<Vector3, float>(lidarReadings);
    }

     public bool IsFollowingPath()
     {
         return isFollowingPath;
     }

    public void SetTarget(Transform newTarget)
    {
        target = newTarget;
        StopMoving(); // Stop previous movement when setting a new target
        // Optionally start finding path immediately if target is set
        // if (target != null) FindPath(target.position);
    }

    public void StopMoving()
    {
        Debug.Log("StopMoving called.");
        isFollowingPath = false;
        path = null;
        currentWaypoint = 0;
        leftWheelSpeed = 0f;
        rightWheelSpeed = 0f;
        // Stop Rigidbody
        if (rb != null)
        {
            rb.velocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }
        // Stop any running coroutines related to movement
        StopCoroutine("DirectMovementPhysics"); // Stop direct movement if running
    }

    // --- Wheel Rotation Visualization ---
    private void RotateWheels()
    {
        if (wheels == null || wheels.Length != 4 || wheelRadius <= 0) return;

        // Calculate angular speed (degrees per second) based on linear speed and radius
        float leftAngularSpeed = (leftWheelSpeed / wheelRadius) * Mathf.Rad2Deg;
        float rightAngularSpeed = (rightWheelSpeed / wheelRadius) * Mathf.Rad2Deg;

        float rotationAmountLeft = leftAngularSpeed * Time.deltaTime; // Use Time.deltaTime for visual updates in Update()
        float rotationAmountRight = rightAngularSpeed * Time.deltaTime;

        // Rotate around the local X-axis (assuming wheels are oriented correctly)
        if (wheels[(int)WheelPosition.FrontLeft] != null)
             wheels[(int)WheelPosition.FrontLeft].Rotate(rotationAmountLeft, 0, 0, Space.Self);
        if (wheels[(int)WheelPosition.RearLeft] != null)
             wheels[(int)WheelPosition.RearLeft].Rotate(rotationAmountLeft, 0, 0, Space.Self); // Same speed for rear
        if (wheels[(int)WheelPosition.FrontRight] != null)
             wheels[(int)WheelPosition.FrontRight].Rotate(rotationAmountRight, 0, 0, Space.Self);
        if (wheels[(int)WheelPosition.RearRight] != null)
             wheels[(int)WheelPosition.RearRight].Rotate(rotationAmountRight, 0, 0, Space.Self); // Same speed for rear
    }

    // --- Direct Movement (Uses Rigidbody) ---
    public void MoveDirectly(Vector3 destination, float duration = 5.0f)
    {
        if (duration <= 0)
        {
            Debug.LogWarning("MoveDirectly duration must be positive. Teleporting instead.");
            if (rb != null) rb.MovePosition(destination);
            else transform.position = destination;
            StopMoving(); // Ensure state is clean
            return;
        }
        Debug.Log($"Moving directly to {destination} over {duration} seconds using Physics.");
        StopMoving(); // Stop current path following/movement first
        StartCoroutine(DirectMovementPhysics(destination, duration)); // Use physics version
    }

    private IEnumerator DirectMovementPhysics(Vector3 destination, float duration)
    {
        // Ensure Rigidbody is available
        if (rb == null) {
            Debug.LogError("Rigidbody is null, cannot perform DirectMovementPhysics.");
            yield break; // Exit coroutine
        }

        Vector3 startPos = rb.position; // Use rb.position
        Quaternion startRot = rb.rotation; // Store start rotation if needed for Slerp later
        float startTime = Time.time;
        float endTime = startTime + duration;

        isFollowingPath = false; // Ensure path following logic is off

        while (Time.time < endTime)
        {
            float t = (Time.time - startTime) / duration;
            // Smoothstep for nicer interpolation start/end
            t = t * t * (3f - 2f * t);
            Vector3 currentTargetPos = Vector3.Lerp(startPos, destination, t);
            rb.MovePosition(currentTargetPos); // Use MovePosition for kinematic-like control

            // Optional: Add simple rotation towards destination
            Vector3 direction = (destination - rb.position).normalized;
            if (direction != Vector3.zero)
            {
                Quaternion targetRotation = Quaternion.LookRotation(direction);
                // Adjust rotation speed factor as needed, use fixedDeltaTime
                Quaternion newRotation = Quaternion.Slerp(rb.rotation, targetRotation, Time.fixedDeltaTime * rotationSpeed / 45f); // Slower rotation during direct move?
                rb.MoveRotation(newRotation); // Use MoveRotation
            }

            // Optional: Add basic wheel rotation visualization during direct move
            // Calculate approximate speed for visuals
            float currentSpeed = Vector3.Distance(startPos, destination) / duration;
            leftWheelSpeed = currentSpeed; // Simplified visual speed
            rightWheelSpeed = currentSpeed;

            yield return new WaitForFixedUpdate(); // Wait for the next physics update
        }

        // Ensure it ends exactly at the destination and stops
        rb.MovePosition(destination);
        // Optionally force rotation to final direction
        Vector3 finalDir = (destination - startPos).normalized;
        if (finalDir != Vector3.zero) rb.MoveRotation(Quaternion.LookRotation(finalDir));

        rb.velocity = Vector3.zero; // Stop physics movement
        rb.angularVelocity = Vector3.zero;
        leftWheelSpeed = 0f;
        rightWheelSpeed = 0f;
        Debug.Log("Direct movement complete");
    }

    // ... potentially existing OnDrawGizmos method ...
    // Example Gizmo for visual sensor
    private void OnDrawGizmosSelected() {
        if (visualSensorOrigin != null) {
            Gizmos.color = Color.blue;
            Vector3 origin = visualSensorOrigin.position;
            Vector3 direction = visualSensorOrigin.forward;
            Gizmos.DrawRay(origin, direction * visualSensorRange);

            // Draw detected bomb range sphere
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(transform.position, 1.5f); // Defusal range visualization
        }

        // Example Gizmo for Lidar Range
        if(lidarOrigin != null) {
            Gizmos.color = Color.cyan;
            Gizmos.DrawWireSphere(lidarOrigin.position, lidarRange);
        }
    }


    private void CheckIfStuck()
    {
        // Check only if enough time has passed
        if (Time.time - lastProgressCheckTime < PROGRESS_CHECK_INTERVAL)
        {
            return; // Not time to check yet
        }

        // --- Check if intentionally rotating ---
        float minRotationRadPerSecSqr = Mathf.Pow(MIN_ROTATION_SPEED_FOR_STUCK_IGNORE * Mathf.Deg2Rad, 2);
        if (rb.angularVelocity.sqrMagnitude > minRotationRadPerSecSqr)
        {
            Debug.Log($"Stuck Check deferred: Robot is rotating (Angular Vel Sqr: {rb.angularVelocity.sqrMagnitude:F4}).");
            isAttemptingStepAssist = false;
            lastProgressCheckTime = Time.time;
            return; // Don't check for linear stuck state while rotating
        }

        // --- Check Forward Speed ---
        float forwardSpeed = Vector3.Dot(rb.velocity, transform.forward);
        bool makingProgress = forwardSpeed >= MIN_PROGRESS_FORWARD_SPEED;

        Debug.Log($"Stuck Check: Forward Speed: {forwardSpeed:F4}, Progress: {makingProgress}, AttemptingAssist: {isAttemptingStepAssist}");

        if (!makingProgress) // If NOT making progress (and not rotating significantly)
        {
            if (!isAttemptingStepAssist)
            {
                // --- First time stuck: Try Step Assist ---
                Debug.Log("Stuck detected. Attempting step assist boost.");
                isAttemptingStepAssist = true;
                Vector3 forceDirection = (Vector3.up * 0.6f + transform.forward * 0.4f).normalized;
                // Using ForceMode.Force now
                rb.AddForce(forceDirection * STEP_ASSIST_FORCE_MAGNITUDE, STEP_ASSIST_FORCE_MODE);
                lastProgressCheckTime = Time.time;
            }
            else
            {
                // --- Still stuck AFTER attempting step assist: Assume Wall ---
                Debug.LogWarning("Step assist failed. Assuming wall, triggering evasion.");
                isBlockedByWall = true; // Trigger evasion
                evasiveRotationTimer = 1.5f; // Set evasion duration
                isAttemptingStepAssist = false; // Reset assist attempt state
                lastProgressCheckTime = Time.time;
            }
        }
        else // Making progress
        {
            if (isAttemptingStepAssist) Debug.Log("Progress detected after step assist attempt.");
            isAttemptingStepAssist = false; // Reset assist state
            lastProgressCheckTime = Time.time;
        }
    }

}