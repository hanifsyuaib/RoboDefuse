using UnityEngine;

public class ObstacleAvoidance : MonoBehaviour
{
    public float detectionDistance = 2f;
    public float reverseTime = 1.5f;
    public float stuckCheckInterval = 0.4f;
    public float stuckMovementThreshold = 0.5f;
    public float enhancedReverseSpeed = 1.0f;
    public LayerMask obstacleLayer;

    private RobotCarController carController;
    private bool isReversing = false;
    private bool isRotating = false;
    private float actionTimer = 0f;
    private int turnDirection = 1;
    private int lastTurnDirection = 1;

    private Vector3 lastPosition;
    private float stuckTimer = 0f;

    private bool isNarrowPath = false;
    private float rotationTargetAngle = 0f;
    private float rotationStartAngle = 0f;

    private int stuckCount = 0;

    public float raycastAngleStep = 15f;
    public float raycastLength = 2.5f;

    void Start()
    {
        carController = GetComponent<RobotCarController>();
        lastPosition = transform.position;
    }

    void Update()
    {
        if (!IsGrounded())
        {
            Debug.LogWarning("Robot off ground! Stopping.");
            carController.StopMoving();
            return;
        }

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

        bool isObstacleDetected = false;
        for (float angle = -30f; angle <= 30f; angle += raycastAngleStep)
        {
            Vector3 direction = Quaternion.Euler(0, angle, 0) * transform.forward;
            bool hitObstacle = Physics.Raycast(transform.position, direction, raycastLength, obstacleLayer);

            Color rayColor = hitObstacle ? Color.red : Color.green;
            Debug.DrawRay(transform.position, direction * raycastLength, rayColor);

            if (hitObstacle)
            {
                isObstacleDetected = true;
            }
        }

        bool obstacleLeft = Physics.Raycast(transform.position, -transform.right, detectionDistance, obstacleLayer);
        bool obstacleRight = Physics.Raycast(transform.position, transform.right, detectionDistance, obstacleLayer);

        Debug.DrawRay(transform.position, -transform.right * detectionDistance, obstacleLeft ? Color.red : Color.green);
        Debug.DrawRay(transform.position, transform.right * detectionDistance, obstacleRight ? Color.red : Color.green);

        isNarrowPath = obstacleLeft && obstacleRight;

        if (isObstacleDetected)
        {
            AvoidObstacle(obstacleLeft, obstacleRight);
        }
        else
        {
            carController.MoveForward();
            HandleLightAdjustment(obstacleLeft, obstacleRight);
        }

        stuckTimer += Time.deltaTime;
        if (stuckTimer >= stuckCheckInterval)
        {
            float distanceMoved = Vector3.Distance(transform.position, lastPosition);
            if (distanceMoved < stuckMovementThreshold)
            {
                Debug.Log("Robot stuck! Trying to escape...");
                StartReverseAndRotate();
            }
            else
            {
                stuckCount = 0; // 🆕 Reset stuck counter if moved properly
            }
            lastPosition = transform.position;
            stuckTimer = 0f;
        }
    }

    private void AvoidObstacle(bool obstacleLeft, bool obstacleRight)
    {
        if (!obstacleLeft && !obstacleRight)
        {
            turnDirection = Random.value > 0.5f ? -1 : 1;
            carController.Turn(turnDirection);
        }
        else if (!obstacleLeft)
        {
            carController.Turn(-1);
        }
        else if (!obstacleRight)
        {
            carController.Turn(1);
        }
        else
        {
            StartReverseAndRotate();
        }
    }

    private void HandleLightAdjustment(bool obstacleLeft, bool obstacleRight)
    {
        if (!obstacleLeft && !obstacleRight)
        {
            turnDirection = Random.value > 0.5f ? 1 : -1;
            carController.Turn(turnDirection * 0.5f);
        }
        else if (obstacleLeft && obstacleRight)
        {
            turnDirection = Random.value > 0.5f ? 1 : -1;
            carController.Turn(turnDirection * 2f);
        }
    }

    private void StartReverseAndRotate()
    {
        isReversing = true;
        stuckCount++; // 🆕 Increase stuck counter

        if (stuckCount >= 2)
        {
            // 🆕 After multiple stucks, force to change turn direction
            turnDirection = -lastTurnDirection;
        }
        else
        {
            // Normal random
            turnDirection = Random.value > 0.5f ? 1 : -1;
        }

        lastTurnDirection = turnDirection; // 🆕 Remember what we chose

        // Adjust reverse time
        if (isNarrowPath)
        {
            actionTimer = reverseTime;
        }
        else
        {
            actionTimer = reverseTime * 0.25f;
        }
    }

    private void ReverseAndTurn()
    {
        if (actionTimer > 0)
        {
            actionTimer -= Time.deltaTime;
            carController.MoveBackward(enhancedReverseSpeed);
            carController.Turn(turnDirection * 0.5f);
        }
        else
        {
            isReversing = false;
            StartRotation();
        }
    }

    private void StartRotation()
    {
        isRotating = true;
        rotationStartAngle = transform.eulerAngles.y;

        // 🆕 Make rotation bigger if stuck many times
        rotationTargetAngle = isNarrowPath ? 120f : (11f + 20f * (stuckCount - 1)); 
        rotationTargetAngle = Mathf.Clamp(rotationTargetAngle, 15f, 180f); // 🛡 prevent insane value
    }

    private void RotateInPlace()
    {
        float currentAngle = transform.eulerAngles.y;
        float angleRotated = Mathf.Abs(Mathf.DeltaAngle(rotationStartAngle, currentAngle));

        if (angleRotated < rotationTargetAngle)
        {
            carController.Turn(turnDirection);
        }
        else
        {
            isRotating = false;
        }
    }

    private bool IsGrounded()
    {
        return Physics.Raycast(transform.position, Vector3.down, 1.0f);
    }
}
