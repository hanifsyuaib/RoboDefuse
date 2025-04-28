using UnityEngine;

public class ObstacleAvoidance : MonoBehaviour
{
    public float detectionDistance = 2f;
    public float reverseTime = 1.5f;
    public float rotateInPlaceTime = 1.0f;
    public float stuckCheckInterval = 2f;
    public float stuckMovementThreshold = 0.5f;
    public float reverseSpeedMultiplier = 0.5f;
    public float enhancedReverseSpeed = 1.0f; // Speed when reversing during stuck
    public LayerMask obstacleLayer;

    private RobotCarController carController;
    private bool isReversing = false;
    private bool isRotating = false;
    private float actionTimer = 0f;
    private int turnDirection = 1; // 1 = right, -1 = left

    // --- For stuck detection ---
    private Vector3 lastPosition;
    private float stuckTimer = 0f;

    // Raycast directions for better obstacle detection
    public float raycastAngleStep = 15f;  // Angle step for surrounding rays
    public float raycastLength = 2f;      // Ray length for obstacle detection
    private RaycastHit hit;

    void Start()
    {
        carController = GetComponent<RobotCarController>();
        lastPosition = transform.position;
    }

    void Update()
    {
        if (isReversing)
        {
            ReverseAndTurn();
            return;
        }
        if (isRotating)
        {
            RotateInPlace();
            return;
        }

        // Cast front rays to check for obstacles
        bool isObstacleDetected = false;
        for (float angle = -30f; angle <= 30f; angle += raycastAngleStep)  // Only cast in front within a cone
        {
            Vector3 direction = Quaternion.Euler(0, angle, 0) * transform.forward;
            bool hitObstacle = Physics.Raycast(transform.position, direction, out hit, raycastLength, obstacleLayer);

            // Visualize the rays in green (turn red if an obstacle is detected)
            Color rayColor = hitObstacle ? Color.red : Color.green;
            Debug.DrawRay(transform.position, direction * raycastLength, rayColor);

            if (hitObstacle)
            {
                isObstacleDetected = true;
            }
        }

        // Cast rays to the left and right of the car to help determine if the space is narrow
        bool obstacleLeft = Physics.Raycast(transform.position, -transform.right, detectionDistance, obstacleLayer);
        bool obstacleRight = Physics.Raycast(transform.position, transform.right, detectionDistance, obstacleLayer);

        // Visualize left and right rays
        Debug.DrawRay(transform.position, -transform.right * detectionDistance, obstacleLeft ? Color.red : Color.green);
        Debug.DrawRay(transform.position, transform.right * detectionDistance, obstacleRight ? Color.red : Color.green);

        // If there's an obstacle in the front, decide what to do
        if (isObstacleDetected)
        {
            AvoidObstacle();
        }
        else
        {
            carController.MoveForward();
        }

        // Adjust rotation based on surrounding space (narrow vs wide)
        if (!obstacleLeft && !obstacleRight)
        {
            // Wider space - make an even smaller rotation to explore
            turnDirection = Random.value > 0.5f ? 1 : -1; // Small random rotation
            carController.Turn(turnDirection * 0.5f); // Much smaller rotation
        }
        else if (obstacleLeft && obstacleRight)
        {
            // Narrow path - make a bigger rotation to escape
            turnDirection = Random.value > 0.5f ? 1 : -1; // Larger rotation to break free
            carController.Turn(turnDirection * 2);  // Increase the rotation speed
        }

        // Stuck detection (same as before)
        stuckTimer += Time.deltaTime;
        if (stuckTimer >= stuckCheckInterval)
        {
            float distanceMoved = Vector3.Distance(transform.position, lastPosition);
            if (distanceMoved < stuckMovementThreshold)
            {
                Debug.Log("Robot stuck! Trying to escape...");
                StartReverseAndRotate();
            }
            lastPosition = transform.position;
            stuckTimer = 0f;
        }
    }

    private void AvoidObstacle()
    {
        bool obstacleLeft = Physics.Raycast(transform.position, -transform.right, detectionDistance, obstacleLayer);
        bool obstacleRight = Physics.Raycast(transform.position, transform.right, detectionDistance, obstacleLayer);

        if (!obstacleLeft && !obstacleRight)
        {
            // No obstacles left or right → randomly pick one
            turnDirection = Random.value > 0.5f ? -1 : 1;
            carController.Turn(turnDirection);
        }
        else if (!obstacleLeft)
        {
            carController.Turn(-1); // Turn left
        }
        else if (!obstacleRight)
        {
            carController.Turn(1); // Turn right
        }
        else
        {
            // Surrounded → reverse with higher speed and then rotate
            StartReverseAndRotate();
        }
    }

    private void StartReverseAndRotate()
    {
        isReversing = true;
        actionTimer = reverseTime;
        turnDirection = Random.value > 0.5f ? 1 : -1; // Random turn direction for variety
    }

    private void ReverseAndTurn()
    {
        if (actionTimer > 0)
        {
            actionTimer -= Time.deltaTime;
            carController.MoveBackward(enhancedReverseSpeed);  // Reverse with enhanced speed to escape stuck situation

            // Turn in place, but with a limit on how far to rotate.
            if (Mathf.Abs(transform.eulerAngles.y) % 360 < 90) // Restrict turning to 90 degrees
            {
                carController.Turn(turnDirection);
            }
        }
        else
        {
            isReversing = false;
            StartRotation();
        }
    }

    private void StartRotation()
    {
        // After reversing, turn in place to find an open path
        isRotating = true;
        actionTimer = rotateInPlaceTime;
    }

    private void RotateInPlace()
    {
        if (actionTimer > 0)
        {
            actionTimer -= Time.deltaTime;
            carController.Turn(turnDirection); // Only rotate in place to avoid returning to the stuck spot
        }
        else
        {
            isRotating = false;
        }
    }

        public bool IsObstacleInFront()
    {
        for (float angle = -30f; angle <= 30f; angle += raycastAngleStep)
        {
            Vector3 dir = Quaternion.Euler(0, angle, 0) * transform.forward;
            if (Physics.Raycast(transform.position, dir, raycastLength, obstacleLayer))
                return true;
        }
        return false;
    }

    /// <summary>
    /// Call this from AStarPathfinding.FixedUpdate to actually do your avoidance move/turn
    /// </summary>
    public void PerformAvoidance()
    {
        if (isReversing)
        {
            ReverseAndTurn();
            return;
        }
        if (isRotating)
        {
            RotateInPlace();
            return;
        }

        // reuse your existing logic:
        bool front = IsObstacleInFront();
        bool left = Physics.Raycast(transform.position,-transform.right, detectionDistance, obstacleLayer);
        bool right= Physics.Raycast(transform.position, transform.right, detectionDistance, obstacleLayer);

        if (front) AvoidObstacle();
        else carController.MoveForward();

        /* … stuck‐check, turn adjustments, etc. … */
    }
    
}
