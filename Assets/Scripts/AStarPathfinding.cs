using UnityEngine;
using System.Linq;

public class AStarPathfinding : MonoBehaviour
{
    public Transform[] bombZones; // Bomb zone locations
    public TrilaterationLocalization localization; // Trilateration localization script
    public float detectionRadius = 2f; // Radius for bomb detection
    public LayerMask bombLayer; // Layer to detect bombs
    public float speed = 5f; // Speed at which the robot moves
    public float turnSpeed = 100f; // Speed at which the robot turns
    public float zoneReachedThreshold = 1.0f; // Distance to consider the zone reached
    public float zoneSearchAreaWidth = 10f; // Example width for zigzag search
    public float zoneSearchStep = 2f; // Example step size for zigzag

    private int currentBombZoneIndex = 0; // Index for the current bomb zone
    private Vector3 currentTarget;
    private Rigidbody rb; // Rigidbody for physics-based movement
    private Transform[] sortedBombZones; // Sorted bomb zones by distance

    private enum RobotState { MovingToZone, SearchingInZone }
    private RobotState currentState = RobotState.MovingToZone;

    // Variables for Zigzag search (example)
    private Vector3 searchZoneCenter;
    private Vector3 currentSearchTarget;
    private bool searchingRight = true;
    private float currentSearchY = 0;
    private float searchMinX, searchMaxX;



    private ObstacleAvoidance obstacleAvoidance;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.useGravity = true;
        rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;

        // disable manual input
        var carCtrl = GetComponent<RobotCarController>();
        if (carCtrl != null) carCtrl.enabled = false;

        // cache avoidance
        obstacleAvoidance = GetComponent<ObstacleAvoidance>();

        sortedBombZones = bombZones
            .OrderBy(z => Vector3.Distance(transform.position, z.position))
            .ToArray();
        SetNextTarget();
    }

    void FixedUpdate()
    {
        // 1) If an obstacle is detected, let ObstacleAvoidance drive this frame:
        if (obstacleAvoidance != null && obstacleAvoidance.IsObstacleInFront())
        {
            obstacleAvoidance.PerformAvoidance();  // you’ll expose this method below
            return;
        }

        // 2) Otherwise resume your A* / Zigzag logic:
        var pos = transform.position;
        switch (currentState)
        {
            case RobotState.MovingToZone:
                MoveToTarget(pos, currentTarget);
                if (Vector3.Distance(pos, currentTarget) < zoneReachedThreshold)
                {
                    currentState = RobotState.SearchingInZone;
                    InitializeSearch(currentTarget);
                }
                break;

            case RobotState.SearchingInZone:
                PerformZigzagSearch(pos);
                if (IsBombDetected(pos))
                {
                    SetNextTarget();
                    currentState = RobotState.MovingToZone;
                }
                break;
        }
    }

    void MoveToTarget(Vector3 currentPosition, Vector3 target)
    {
        // Calculate direction to target, ignoring Y axis
        Vector3 targetXZ = new Vector3(target.x, currentPosition.y, target.z); // Keep current Y
        Vector3 direction = (targetXZ - currentPosition).normalized;

        // Avoid moving if direction is zero (already at target)
        if (direction == Vector3.zero) return;

        // Calculate the angle between the robot's current forward direction and the direction to the target
        // Use direction which is already projected onto the XZ plane
        float angleToTarget = Vector3.SignedAngle(transform.forward, direction, Vector3.up);

        // If the angle is large, rotate towards the target
        if (Mathf.Abs(angleToTarget) > 2f) // Tolerance for when to start turning
        {
            RotateTowardsTarget(angleToTarget); // Rotate slowly in the right direction
        }
        else
        {
            // Move towards the target in the forward direction
            // Ensure rotation is mostly complete before moving forward significantly
            // Move along the calculated XZ direction, maintaining current Y
            Vector3 movement = transform.forward * speed * Time.fixedDeltaTime;
            rb.MovePosition(rb.position + new Vector3(movement.x, 0, movement.z)); // Apply movement only on XZ plane
        }
    }

    void RotateTowardsTarget(float angle)
    {
        // Rotate the robot slowly in the direction of the target
        float turnDirection = Mathf.Sign(angle);
        // Clamp rotation amount to avoid overshooting in one frame
        float maxRotation = Mathf.Abs(angle);
        float rotationAmount = Mathf.Min(turnSpeed * Time.fixedDeltaTime, maxRotation) * turnDirection;
        transform.Rotate(0f, rotationAmount, 0f); // Rotate around Y-axis
    }

    void SetNextTarget()
    {
        // Loop through the sorted bomb zones to find the next one
        if (sortedBombZones.Length > 0)
        {
            currentBombZoneIndex = (currentBombZoneIndex + 1) % sortedBombZones.Length; // Move to next index, loop back
            currentTarget = sortedBombZones[currentBombZoneIndex].position;
            Debug.Log($"Setting next target to zone {currentBombZoneIndex} at {currentTarget}");
        }
        else
        {
            Debug.LogWarning("No bomb zones defined or sorted.");
            // Optional: Handle case with no zones (e.g., stop)
        }
    }

    void InitializeSearch(Vector3 zoneCenter)
    {
        searchZoneCenter = zoneCenter;
        // Define search boundaries based on zoneCenter and zoneSearchAreaWidth
        searchMinX = zoneCenter.x - zoneSearchAreaWidth / 2f;
        searchMaxX = zoneCenter.x + zoneSearchAreaWidth / 2f;
        // Use the robot's current Y for search height, or a fixed ground level Y
        float searchYLevel = transform.position.y;
        currentSearchY = zoneCenter.z - zoneSearchAreaWidth / 2f; // Start at the bottom edge (assuming Z is forward/back)
        searchingRight = true;
        currentSearchTarget = new Vector3(searchMinX, searchYLevel, currentSearchY); // Start at bottom-left corner at robot's height
        Debug.Log($"Initializing search. Start Target: {currentSearchTarget}");
    }

    void PerformZigzagSearch(Vector3 currentPosition)
    {
        // Simple Zigzag: Move horizontally, then step vertically, then move horizontally back
        // Ensure the target Y is consistent (e.g., robot's current Y or a fixed ground level)
        Vector3 targetXZ = new Vector3(currentSearchTarget.x, currentPosition.y, currentSearchTarget.z);
        MoveToTarget(currentPosition, targetXZ); // Use MoveToTarget which handles XZ movement

        // Check if current search target is reached (on the XZ plane)
        Vector2 currentPosXZ = new Vector2(currentPosition.x, currentPosition.z);
        Vector2 targetPosXZ = new Vector2(currentSearchTarget.x, currentSearchTarget.z);
        if (Vector2.Distance(currentPosXZ, targetPosXZ) < 0.5f) // Threshold for reaching search point
        {
            // Use the robot's current Y or a fixed ground level Y for next targets
            float searchYLevel = currentPosition.y;

            if (searchingRight)
            {
                // Was moving right, reached right edge?
                if (Mathf.Abs(currentPosition.x - searchMaxX) < 0.5f) // Check proximity to edge
                {
                    // Move up (increase Z)
                    currentSearchY += zoneSearchStep;
                    // Check if search area height exceeded
                    if (currentSearchY > searchZoneCenter.z + zoneSearchAreaWidth / 2f)
                    {
                        Debug.Log($"Finished searching zone {currentBombZoneIndex}. Moving to next.");
                        SetNextTarget();
                        currentState = RobotState.MovingToZone;
                        return;
                    }
                    // Target is now above current pos, at the right edge X
                    currentSearchTarget = new Vector3(searchMaxX, searchYLevel, currentSearchY);
                    searchingRight = false; // Next horizontal move will be left
                    Debug.Log($"Zigzag: Reached right edge, moving up. New Target: {currentSearchTarget}");
                }
                else // Still moving right
                {
                     // Target is right edge at current Z
                     currentSearchTarget = new Vector3(searchMaxX, searchYLevel, currentSearchY);
                     Debug.Log($"Zigzag: Moving right. New Target: {currentSearchTarget}");
                }
            }
            else // Was moving left
            {
                 // Was moving left, reached left edge?
                 if (Mathf.Abs(currentPosition.x - searchMinX) < 0.5f) // Check proximity to edge
                {
                     // Move up (increase Z)
                    currentSearchY += zoneSearchStep;
                     // Check if search area height exceeded
                    if (currentSearchY > searchZoneCenter.z + zoneSearchAreaWidth / 2f)
                    {
                        Debug.Log($"Finished searching zone {currentBombZoneIndex}. Moving to next.");
                        SetNextTarget();
                        currentState = RobotState.MovingToZone;
                        return;
                    }
                    // Target is now above current pos, at the left edge X
                    currentSearchTarget = new Vector3(searchMinX, searchYLevel, currentSearchY);
                    searchingRight = true; // Next horizontal move will be right
                    Debug.Log($"Zigzag: Reached left edge, moving up. New Target: {currentSearchTarget}");
                }
                 else // Still moving left
                {
                    // Target is left edge at current Z
                    currentSearchTarget = new Vector3(searchMinX, searchYLevel, currentSearchY);
                    Debug.Log($"Zigzag: Moving left. New Target: {currentSearchTarget}");
                }
            }
        }
    }


    bool IsBombDetected(Vector3 position)
    {
        // Use Physics.OverlapSphere or Physics.CheckSphere to detect if the robot is in range of a bomb
        Collider[] hitColliders = Physics.OverlapSphere(position, detectionRadius, bombLayer);
        if (hitColliders.Length > 0)
        {
            // Bomb detected in the area
            Debug.Log("Bomb detected nearby!");
            return true;
        }
        return false;
    }
}