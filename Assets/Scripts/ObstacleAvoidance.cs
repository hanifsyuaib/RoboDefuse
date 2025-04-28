using UnityEngine;

public class ObstacleAvoidance : MonoBehaviour
{
    public float detectionDistance = 2f;
    public LayerMask obstacleLayer;

    private RobotCarController carController;

    private void Start()
    {
        carController = GetComponent<RobotCarController>();
    }

    private void Update()
    {
        // Raycasts
        bool obstacleFront = Physics.Raycast(transform.position, transform.forward, detectionDistance, obstacleLayer);
        bool obstacleLeft = Physics.Raycast(transform.position, -transform.right, detectionDistance * 0.8f, obstacleLayer);
        bool obstacleRight = Physics.Raycast(transform.position, transform.right, detectionDistance * 0.8f, obstacleLayer);

        // Visualization for debugging
        Debug.DrawRay(transform.position, transform.forward * detectionDistance, Color.red);
        Debug.DrawRay(transform.position, -transform.right * detectionDistance * 0.8f, Color.yellow);
        Debug.DrawRay(transform.position, transform.right * detectionDistance * 0.8f, Color.yellow);

        // Decision making
        if (obstacleFront)
        {
            if (!obstacleLeft)
            {
                carController.Turn(-1); // Turn left
            }
            else if (!obstacleRight)
            {
                carController.Turn(1); // Turn right
            }
            else
            {
                carController.MoveBackward(); // Nowhere to go, backup
            }
        }
        else
        {
            carController.MoveForward(); // No obstacle, move forward
        }
    }
}
