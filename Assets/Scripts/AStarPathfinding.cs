using UnityEngine;
using System.Linq;

[RequireComponent(typeof(RobotCarController))]
public class AStarPathfinding : MonoBehaviour
{
    public Transform[] bombZones; // Input bomb zone locations
    public TrilaterationLocalization localization; // Reference to localization
    public float detectionRadius = 2f; // Bomb detection range
    public LayerMask bombLayer;

    private RobotCarController carController;
    private Transform[] sortedBombZones;
    private int currentIndex = 0;

    void Start()
    {
        carController = GetComponent<RobotCarController>();

        // Sort bomb zones by distance from the robot
        sortedBombZones = bombZones
            .OrderBy(z => Vector3.Distance(transform.position, z.position))
            .ToArray();
    }

    void Update()
    {
        if (currentIndex >= sortedBombZones.Length)
        {
            Debug.Log("✅ All bomb zones visited.");
            carController.StopMoving();
            return;
        }

        Vector3 currentPos = localization.estimatedPosition;
        Vector3 targetPos = sortedBombZones[currentIndex].position;

        // Check for arrival
        if (Vector3.Distance(currentPos, targetPos) <= detectionRadius)
        {
            Debug.Log($"🎯 Bomb zone {currentIndex + 1} reached!");
            currentIndex++;
            return;
        }

        // Direction and rotation
        Vector3 direction = (targetPos - currentPos).normalized;
        float angleToTarget = Vector3.SignedAngle(transform.forward, direction, Vector3.up);

        if (Mathf.Abs(angleToTarget) > 5f)
        {
            carController.Rotate(Mathf.Sign(angleToTarget));
        }
        else
        {
            carController.MoveForward();
        }
    }
}
