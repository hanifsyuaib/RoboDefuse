using UnityEngine;

// Require LocalizationSystem again as we need its estimate for the robot's pose
[RequireComponent(typeof(RobotController), typeof(LocalizationSystem), typeof(SensorSystem))]
public class NavigationController : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private RobotController robotController;
    [SerializeField] private LocalizationSystem localizationSystem; // Needed for robot's estimated pose
    [SerializeField] private SensorSystem sensorSystem;

    [Header("Navigation Parameters")]
    public Transform currentTarget; // The target position the robot should move towards
    public float targetReachedThreshold = 0.5f; // How close the robot needs to be to the target (using estimated position)
    public float rotationSpeed = 90.0f; // How fast the robot turns towards the target (deg/s)
    public float avoidanceTurnSpeed = 120.0f; // How fast the robot turns when avoiding obstacles
    public float forwardSpeed = 0.8f; // Speed when moving towards target
    public float alignmentAngleThreshold = 10.0f; // Angle (degrees) within which the robot is considered "facing" the target to move forward
    public float turnOnlyAngleThreshold = 45.0f; // Angle (degrees) beyond which the robot will only turn

    private enum NavigationState
    {
        Idle,
        MovingToTarget,
        AvoidingObstacle
    }
    [SerializeField] private NavigationState currentState = NavigationState.Idle;

    void Awake()
    {
        // Get references if not set in Inspector
        if (robotController == null) robotController = GetComponent<RobotController>();
        if (localizationSystem == null) localizationSystem = GetComponent<LocalizationSystem>(); // Ensure this is fetched
        if (sensorSystem == null) sensorSystem = GetComponent<SensorSystem>();
        Debug.Log($"NavigationController Awake. Initial State: {currentState}, Target: {currentTarget?.name ?? "NULL"}");
    }

     void Start()
    {
        // If a target is assigned in the inspector, immediately try to move
        if (currentTarget != null && currentState == NavigationState.Idle)
        {
            Debug.Log("Target found in Start, switching to MovingToTarget state.");
            currentState = NavigationState.MovingToTarget;
        }
    }

    void Update()
    {
        // Simple state machine for navigation logic
        switch (currentState)
        {
            case NavigationState.Idle:
                HandleIdleState();
                break;
            case NavigationState.MovingToTarget:
                HandleMovingToTargetState();
                break;
            case NavigationState.AvoidingObstacle:
                HandleAvoidingObstacleState();
                break;
        }
    }

    void HandleIdleState()
    {
        robotController.SetTargetMovement(0f, 0f);
        if (currentTarget != null)
        {
            Debug.Log("Target found in Idle, switching to MovingToTarget");
            currentState = NavigationState.MovingToTarget;
        }
    }

    void HandleMovingToTargetState()
    {
        if (currentTarget == null)
        {
            Debug.Log("Target is NULL, switching to Idle");
            currentState = NavigationState.Idle;
            robotController.SetTargetMovement(0f, 0f); // Ensure stop on state change
            return;
        }

        // --- Use Estimated Pose for Robot, Direct Transform for Target ---
        Vector3 estimatedPos = localizationSystem.EstimatedPosition;
        Quaternion estimatedRot = localizationSystem.EstimatedRotation;
        Vector3 estimatedForward = estimatedRot * Vector3.forward; // Robot's estimated forward direction
        Vector3 targetPos = currentTarget.position; // Direct position of the target waypoint
        // --- End Pose Data ---

        // Check for obstacles first
        if (sensorSystem.IsObstacleDetected && sensorSystem.DistanceToObstacle < (robotController.maxLinearSpeed * 0.6f))
        {
             Debug.Log($"Obstacle DETECTED at {sensorSystem.DistanceToObstacle}m, switching to AvoidingObstacle");
            currentState = NavigationState.AvoidingObstacle;
            robotController.SetTargetMovement(0f, 0f); // Ensure stop on state change
            return;
        }

        // Calculate distance and direction to target using estimated robot position
        Vector3 directionToTarget = (targetPos - estimatedPos);
        directionToTarget.y = 0; // Ignore vertical difference for distance and angle checks
        float distanceToTarget = directionToTarget.magnitude;

        // Check if target is reached (based on estimated position)
        if (distanceToTarget < targetReachedThreshold)
        {
            Debug.Log($"Target Reached! (Est. Distance: {distanceToTarget} < Threshold: {targetReachedThreshold}) Switching to Idle.");
            currentTarget = null; // Or move to next waypoint if applicable
            currentState = NavigationState.Idle;
            robotController.SetTargetMovement(0f, 0f); // Stop
            return;
        }

        // --- Calculate desired movement using estimated orientation ---
        // Calculate the angle between the robot's estimated forward direction and the direction to the target
        float angleDifference = Vector3.SignedAngle(estimatedForward, directionToTarget, Vector3.up); // Get signed angle (-180 to 180)

        // Determine turn speed
        float targetAngularVelocity = 0f;
        if (Mathf.Abs(angleDifference) > 2.0f) // Small deadzone
        {
            // Use max speed for large angles, proportional for smaller ones
            if (Mathf.Abs(angleDifference) > turnOnlyAngleThreshold) {
                 targetAngularVelocity = Mathf.Sign(angleDifference) * robotController.maxAngularSpeed; // Use max speed directly
            } else {
                 targetAngularVelocity = Mathf.Sign(angleDifference) * rotationSpeed;
            }
        }
        targetAngularVelocity = Mathf.Clamp(targetAngularVelocity, -robotController.maxAngularSpeed, robotController.maxAngularSpeed);

        // Determine forward speed
        float targetLinearVelocity = 0f;
        // Only move forward if reasonably facing the target (based on estimated orientation)
        if (Mathf.Abs(angleDifference) < alignmentAngleThreshold)
        {
            targetLinearVelocity = forwardSpeed;
        }
        targetLinearVelocity = Mathf.Clamp(targetLinearVelocity, 0, robotController.maxLinearSpeed);

        // Debugging
        // Debug.Log($"EstPos: {estimatedPos:F2}, TgtPos: {targetPos:F2}, Dist: {distanceToTarget:F2}, AngleDiff: {angleDifference:F1}");
        Debug.Log($"Sending Movement Command: Linear={targetLinearVelocity:F2}, Angular={targetAngularVelocity:F1}");

        // Send command to RobotController
        robotController.SetTargetMovement(targetLinearVelocity, targetAngularVelocity);
    }

    void HandleAvoidingObstacleState()
    {
        Debug.Log("Handling AvoidingObstacle State");
        // Simple avoidance: Stop moving forward and turn right
        robotController.SetTargetMovement(0f, -avoidanceTurnSpeed);

        // Check if the path is clear again
        if (!sensorSystem.IsObstacleDetected || sensorSystem.DistanceToObstacle > (robotController.maxLinearSpeed * 0.8f))
        {
            Debug.Log("Path Clear - Resuming Navigation, switching to MovingToTarget");
            currentState = NavigationState.MovingToTarget;
        }
    }

    // Public method to set a new target
    public void SetNavigationTarget(Transform target)
    {
        Debug.Log($"New navigation target set: {target?.name ?? "NULL"}");
        currentTarget = target;
        if (currentTarget != null)
        {
             Debug.Log("SetNavigationTarget: Switching to MovingToTarget state.");
            currentState = NavigationState.MovingToTarget;
        }
        else
        {
            Debug.Log("SetNavigationTarget: Target is null, switching to Idle state.");
            currentState = NavigationState.Idle;
            robotController.SetTargetMovement(0f, 0f); // Stop if target is cleared
        }
    }
}