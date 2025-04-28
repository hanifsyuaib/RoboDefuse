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
    private int currentBombZoneIndex = 0; // Index for the current bomb zone
    private Vector3 currentTarget;
    private Rigidbody rb; // Rigidbody for physics-based movement
    private Transform[] sortedBombZones; // Sorted bomb zones by distance

    void Start()
    {
        // Get the Rigidbody component
        rb = GetComponent<Rigidbody>();

        // Ensure the Rigidbody is not affected by gravity or rotation on certain axes
        rb.useGravity = false;
        rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;

        // Sort bomb zones based on the robot's current position
        sortedBombZones = bombZones.OrderBy(zone => Vector3.Distance(transform.position, zone.position)).ToArray();

        // Set the first bomb zone as the target
        SetNextTarget();
    }

    void Update()
    {
        // Get the robot's current position using trilateration
        Vector3 robotPosition = localization.estimatedPosition;

        // Move towards the current target bomb zone
        MoveToTarget(robotPosition);

        // Check if the robot has reached the target bomb zone
        if (IsBombDetected(robotPosition))
        {
            Debug.Log("Bomb zone reached! Moving to next zone...");
            SetNextTarget(); // Move to the next bomb zone
        }
    }

    void MoveToTarget(Vector3 currentPosition)
    {
        // Calculate direction to target
        Vector3 direction = (currentTarget - currentPosition).normalized;

        // Calculate the angle between the robot's current forward direction and the direction to the target
        float angleToTarget = Vector3.SignedAngle(transform.forward, direction, Vector3.up);

        // If the angle is large, rotate slowly towards the target
        if (Mathf.Abs(angleToTarget) > 2f) // Tolerance for when to start turning
        {
            RotateTowardsTarget(angleToTarget); // Rotate slowly in the right direction
        }
        else
        {
            // Move towards the target in the forward direction
            rb.MovePosition(transform.position + direction * speed * Time.deltaTime);
        }
    }

    void RotateTowardsTarget(float angle)
    {
        // Rotate the robot slowly in the direction of the target
        float turnDirection = Mathf.Sign(angle);
        float rotationAmount = turnSpeed * turnDirection * Time.deltaTime;
        transform.Rotate(0f, rotationAmount, 0f); // Rotate around Y-axis
    }

    void SetNextTarget()
    {
        // Loop through the sorted bomb zones to find the next one
        if (sortedBombZones.Length > 0)
        {
            currentTarget = sortedBombZones[currentBombZoneIndex].position;
            currentBombZoneIndex = (currentBombZoneIndex + 1) % sortedBombZones.Length; // Loop back to first zone
        }
    }

    bool IsBombDetected(Vector3 position)
    {
        // Use Physics.OverlapSphere or Physics.CheckSphere to detect if the robot is in range of a bomb
        Collider[] hitColliders = Physics.OverlapSphere(position, detectionRadius, bombLayer);
        if (hitColliders.Length > 0)
        {
            // Bomb detected in the area
            return true;
        }
        return false;
    }
}
