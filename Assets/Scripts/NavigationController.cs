using UnityEngine;

[RequireComponent(typeof(RobotController), typeof(LocalizationSystem), typeof(SensorSystem))]
public class NavigationController : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private RobotController robotController;
    [SerializeField] private LocalizationSystem localizationSystem;
    [SerializeField] private SensorSystem sensorSystem;

    [Header("Navigation Parameters")]
    public Transform currentTarget; // The target position the robot should move towards
    public float targetReachedThreshold = 0.5f; // How close the robot needs to be to the target
    public float rotationSpeed = 60.0f; // How fast the robot turns towards the target (deg/s)
    public float avoidanceTurnSpeed = 90.0f; // How fast the robot turns when avoiding obstacles
    public float forwardSpeed = 0.8f; // Speed when moving towards target (should be <= RobotController.maxLinearSpeed)

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
        if (localizationSystem == null) localizationSystem = GetComponent<LocalizationSystem>();
        if (sensorSystem == null) sensorSystem = GetComponent<SensorSystem>();
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
        // Stop the robot
        robotController.SetTargetMovement(0f, 0f);

        // If we have a target, start moving towards it
        if (currentTarget != null)
        {
            currentState = NavigationState.MovingToTarget;
        }
    }

    void HandleMovingToTargetState()
    {
        if (currentTarget == null)
        {
            currentState = NavigationState.Idle;
            return;
        }

        // Check for obstacles first
        if (sensorSystem.IsObstacleDetected && sensorSystem.DistanceToObstacle < robotController.maxLinearSpeed * 0.5f) // React if obstacle is close
        {
            currentState = NavigationState.AvoidingObstacle;
            return;
        }

        // Calculate distance and direction to target using estimated position
        Vector3 estimatedPos = localizationSystem.EstimatedPosition;
        Vector3 targetPos = currentTarget.position;
        Vector3 directionToTarget = (targetPos - estimatedPos).normalized;
        float distanceToTarget = Vector3.Distance(new Vector3(estimatedPos.x, 0, estimatedPos.z), new Vector3(targetPos.x, 0, targetPos.z)); // Use XZ distance

        // Check if target is reached
        if (distanceToTarget < targetReachedThreshold)
        {
            Debug.Log("Target Reached!");
            currentTarget = null; // Clear target
            currentState = NavigationState.Idle;
            robotController.SetTargetMovement(0f, 0f); // Stop
            return;
        }

        // --- Calculate desired movement ---
        // 1. Calculate desired rotation
        float targetYawDeg = Mathf.Atan2(directionToTarget.z, directionToTarget.x) * Mathf.Rad2Deg;
        float currentYawDeg = localizationSystem.EstimatedYawDeg;
        float angleDifference = Mathf.DeltaAngle(currentYawDeg, targetYawDeg); // Correctly handles wrapping around 360

        // Determine turn direction and speed
        float targetAngularVelocity = 0f;
        if (Mathf.Abs(angleDifference) > 5.0f) // Only turn if angle difference is significant
        {
            targetAngularVelocity = Mathf.Sign(angleDifference) * rotationSpeed;
        }
        // Clamp angular velocity to robot's max speed
        targetAngularVelocity = Mathf.Clamp(targetAngularVelocity, -robotController.maxAngularSpeed, robotController.maxAngularSpeed);


        // 2. Calculate desired forward speed
        float targetLinearVelocity = 0f;
        // Only move forward if reasonably facing the target
        if (Mathf.Abs(angleDifference) < 45.0f)
        {
            targetLinearVelocity = forwardSpeed;
        }
        // Clamp linear velocity
        targetLinearVelocity = Mathf.Clamp(targetLinearVelocity, 0, robotController.maxLinearSpeed); // Don't move backward towards target

        // Send command to RobotController
        robotController.SetTargetMovement(targetLinearVelocity, targetAngularVelocity);
    }

    void HandleAvoidingObstacleState()
    {
        // Simple avoidance: Stop moving forward and turn (e.g., right)
        // A more sophisticated approach would check side sensors to pick a better direction
        Debug.Log("Avoiding Obstacle - Turning Right");
        robotController.SetTargetMovement(0f, -avoidanceTurnSpeed); // Turn right (negative angular velocity based on previous fix)

        // Check if the path is clear again
        // Use a slightly longer distance check here to ensure we've turned enough
        if (!sensorSystem.IsObstacleDetected || sensorSystem.DistanceToObstacle > robotController.maxLinearSpeed * 0.75f)
        {
            Debug.Log("Path Clear - Resuming Navigation");
            currentState = NavigationState.MovingToTarget;
        }
    }

    // Public method to set a new target (e.g., called by MissionManager)
    public void SetNavigationTarget(Transform target)
    {
        Debug.Log($"New navigation target set: {target?.name ?? "NULL"}");
        currentTarget = target;
        if (currentTarget != null)
        {
            currentState = NavigationState.MovingToTarget;
        }
        else
        {
            currentState = NavigationState.Idle;
        }
    }
}