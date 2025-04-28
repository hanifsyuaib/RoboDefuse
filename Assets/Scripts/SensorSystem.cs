using UnityEngine;
using System.Collections.Generic;

public class SensorSystem : MonoBehaviour
{
    [Header("Sensor Settings")]
    public float obstacleDetectionDistance = 2.0f;
    public float bombDetectionDistance = 5.0f;
    public LayerMask obstacleLayers;
    public LayerMask bombLayer;
    public float sensorHeightOffset = 0.1f;
    [Range(0f, 90f)] public float obstacleRayAngle = 30f; // Angle for side rays
    public int numberOfObstacleRays = 3; // Total rays (center + sides)

    [Header("Sensor Data (Read Only)")]
    [SerializeField] private bool obstacleDetected = false;
    [SerializeField] private float distanceToObstacle = float.MaxValue; // Distance to closest detected obstacle
    [SerializeField] private GameObject detectedBomb = null;

    // Public properties
    public bool IsObstacleDetected => obstacleDetected;
    public float DistanceToObstacle => distanceToObstacle;
    public GameObject DetectedBomb => detectedBomb;

    void Update()
    {
        PerformRaycasts();
    }

    void PerformRaycasts()
    {
        Vector3 rayStartPoint = transform.position + Vector3.up * sensorHeightOffset;
        obstacleDetected = false; // Reset detection status each frame
        distanceToObstacle = float.MaxValue; // Reset distance

        // --- Obstacle Detection (Multiple Rays) ---
        float angleStep = 0;
        if (numberOfObstacleRays > 1)
        {
            // Calculate angle step based on total angle spread and number of rays
            // Example: 3 rays spread over 60 degrees (-30, 0, +30) -> step = 60 / (3-1) = 30
            angleStep = (obstacleRayAngle * 2) / (numberOfObstacleRays - 1);
        }

        for (int i = 0; i < numberOfObstacleRays; i++)
        {
            // Calculate the angle for the current ray
            // Starts from -obstacleRayAngle and goes up to +obstacleRayAngle
            float currentAngle = -obstacleRayAngle + (i * angleStep);
            Quaternion rotation = Quaternion.Euler(0, currentAngle, 0);
            Vector3 direction = rotation * transform.forward;

            RaycastHit obstacleHit;
            bool hit = Physics.Raycast(rayStartPoint, direction, out obstacleHit, obstacleDetectionDistance, obstacleLayers);

            if (hit)
            {
                obstacleDetected = true; // Mark obstacle detected if any ray hits
                // Keep track of the *closest* obstacle detected
                if (obstacleHit.distance < distanceToObstacle)
                {
                    distanceToObstacle = obstacleHit.distance;
                }
                Debug.DrawRay(rayStartPoint, direction * obstacleHit.distance, Color.red);
            }
            else
            {
                Debug.DrawRay(rayStartPoint, direction * obstacleDetectionDistance, Color.green);
            }
        }


        // --- Bomb Detection (Simple Forward Ray - can be expanded similarly) ---
        RaycastHit bombHit;
        if (Physics.Raycast(rayStartPoint, transform.forward, out bombHit, bombDetectionDistance, bombLayer))
        {
            if (bombHit.collider.CompareTag("Bombs"))
            {
                detectedBomb = bombHit.collider.gameObject;
                Debug.DrawRay(rayStartPoint, transform.forward * bombHit.distance, Color.yellow);
            }
            else
            {
                detectedBomb = null;
                Debug.Log($"Hit non-bomb object on bomb layer: {bombHit.collider.name}");
            }
        }
        else
        {
            detectedBomb = null;
        }
    }
}