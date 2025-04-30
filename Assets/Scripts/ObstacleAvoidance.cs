using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class ObstacleAvoidance : MonoBehaviour
{
    public float detectionDistance = 1.2f;
    public float reverseTime = 1.5f;
    public float rotateInPlaceTime = 1.2f;
    public float rotateInPlaceTimeLong = 2.5f;
    public float stuckCheckInterval = 2f;
    public float stuckMovementThreshold = 0.5f;
    public float reverseSpeedMultiplier = 0.5f;
    public float enhancedReverseSpeed = 1.0f;
    public float longStuckThreshold = 5f;
    public LayerMask obstacleLayer;

    public Transform[] bombZones;
    private int currentTargetIndex = 0;

    public float targetReachThreshold = 1.5f;
    public float movementSpeed = 1.5f;
    public float rotationSpeed = 2.0f;

    private RobotCarController carController;
    private bool isReversing = false;
    private bool isRotating = false;
    private float actionTimer = 0f;
    private int turnDirection = 1;

    private Vector3 lastPosition;
    private float stuckTimer = 0f;
    private float longStuckTimer = 0f;

    public float raycastAngleStep = 15f;
    public float raycastLength = 1.2f;
    private RaycastHit hit;

    private List<Vector3> path;
    private int currentPathIndex = 0;

    private HashSet<Vector3> deadEndPositions = new HashSet<Vector3>();
    private float deadEndRadius = 3f;

    void Start()
    {
        carController = GetComponent<RobotCarController>();
        lastPosition = transform.position;

        if (bombZones.Length > 0)
        {
            CalculatePathToTarget();
        }
    }

    void Update()
    {
        if (currentTargetIndex >= bombZones.Length) return;

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

        if (currentPathIndex < path.Count)
        {
            Vector3 target = path[currentPathIndex];
            Vector3 directionToTarget = (target - transform.position).normalized;
            float distanceToTarget = Vector3.Distance(transform.position, target);

            if (distanceToTarget < targetReachThreshold)
            {
                currentPathIndex++;
                if (currentPathIndex < path.Count)
                {
                    ScanForBombs(path[currentPathIndex]);
                }
                else
                {
                    // Reached the bomb zone
                    currentTargetIndex++;
                    if (currentTargetIndex < bombZones.Length)
                    {
                        CalculatePathToTarget();
                        currentPathIndex = 0;
                    }
                    else
                    {
                        Debug.Log("All bomb zones visited.");
                    }
                }
                return;
            }

            bool isObstacleDetected = false;
            for (float angle = -30f; angle <= 30f; angle += raycastAngleStep)
            {
                Vector3 rayDirection = Quaternion.Euler(0, angle, 0) * transform.forward;
                bool hitObstacle = Physics.Raycast(transform.position, rayDirection, out hit, raycastLength, obstacleLayer);
                Debug.DrawRay(transform.position, rayDirection * raycastLength, hitObstacle ? Color.red : Color.green);
                if (hitObstacle) isObstacleDetected = true;
            }

            if (isObstacleDetected)
            {
                AvoidObstacle();
            }
            else
            {
                MoveForward(directionToTarget);
            }

            stuckTimer += Time.deltaTime;
            longStuckTimer += Time.deltaTime;

            if (stuckTimer >= stuckCheckInterval)
            {
                float distanceMoved = Vector3.Distance(transform.position, lastPosition);
                if (distanceMoved < stuckMovementThreshold)
                {
                    if (!IsNearKnownDeadEnd(transform.position))
                    {
                        deadEndPositions.Add(transform.position);
                        Debug.Log("Marking dead end at: " + transform.position);
                    }

                    Debug.Log("Robot stuck!");

                    if (longStuckTimer >= longStuckThreshold)
                    {
                        Debug.Log("Severely stuck! Performing extended reverse and BIG rotate.");
                        StartExtendedReverseAndRotate();
                        longStuckTimer = 0f;
                    }
                    else
                    {
                        Debug.Log("Trying normal reverse and SMALL turn...");
                        StartReverseAndRotateInPlace();
                    }
                }
                else
                {
                    longStuckTimer = 0f;
                }

                lastPosition = transform.position;
                stuckTimer = 0f;
            }
        }
    }

    private void ScanForBombs(Vector3 targetZone)
    {
        Collider[] bombs = Physics.OverlapSphere(targetZone, 10f, LayerMask.GetMask("Bomb"));
        if (bombs.Length > 0)
        {
            foreach (var bomb in bombs)
            {
                Debug.Log("Bomb found at: " + bomb.transform.position);
                // Immediately go to the next bomb zone
                currentTargetIndex++;
                if (currentTargetIndex < bombZones.Length)
                {
                    CalculatePathToTarget();
                    currentPathIndex = 0;
                }
                else
                {
                    Debug.Log("All bomb zones visited.");
                }
                return;
            }
        }
        else
        {
            Debug.Log("No bombs found around " + targetZone);
        }
    }

    private void CalculatePathToTarget()
    {
        path = new List<Vector3>();
        if (currentTargetIndex < bombZones.Length)
        {
            path.Add(bombZones[currentTargetIndex].position);
        }
    }

    private void AvoidObstacle()
    {
        bool obstacleLeft = Physics.Raycast(transform.position, -transform.right, detectionDistance, obstacleLayer);
        bool obstacleRight = Physics.Raycast(transform.position, transform.right, detectionDistance, obstacleLayer);

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
            StartReverseAndRotateInPlace();
        }
    }

    private void StartReverseAndRotateInPlace()
    {
        isReversing = true;
        actionTimer = reverseTime;
        turnDirection = Random.value > 0.5f ? 1 : -1;
    }

    private void StartExtendedReverseAndRotate()
    {
        isReversing = true;
        actionTimer = reverseTime * 2f;
        turnDirection = Random.value > 0.5f ? 1 : -1;
        isRotating = true;
    }

    private void ReverseAndTurn()
    {
        if (actionTimer > 0)
        {
            actionTimer -= Time.deltaTime;
            carController.MoveBackward(enhancedReverseSpeed);
            carController.Turn(turnDirection);
        }
        else
        {
            isReversing = false;

            if (isRotating)
            {
                actionTimer = rotateInPlaceTimeLong;
            }
            else
            {
                isRotating = true;
                actionTimer = rotateInPlaceTime;
            }
        }
    }

    private void RotateInPlace()
    {
        if (actionTimer > 0)
        {
            actionTimer -= Time.deltaTime;
            carController.Turn(turnDirection);
        }
        else
        {
            isRotating = false;
        }
    }

    private bool IsNearKnownDeadEnd(Vector3 position)
    {
        foreach (var deadEnd in deadEndPositions)
        {
            if (Vector3.Distance(position, deadEnd) < deadEndRadius)
                return true;
        }
        return false;
    }

    private void MoveForward(Vector3 directionToTarget)
    {
        Quaternion targetRotation = Quaternion.LookRotation(directionToTarget);
        transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, Time.deltaTime * rotationSpeed);
        carController.MoveForward();
    }
}
