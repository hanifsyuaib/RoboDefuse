using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ExplorationController : MonoBehaviour
{
    [Header("Zone Settings")]
    [SerializeField] private Vector2[] zoneA; // Array of 4 points forming zone A (X,Z)
    [SerializeField] private Vector2[] zoneB; // Array of 4 points forming zone B (X,Z)
    [SerializeField] private Vector2[] zoneC; // Array of 4 points forming zone C (X,Z)
    
    [Header("Exploration Settings")]
    [SerializeField] private float zigzagSpacing = 2f; // Distance between zigzag lines
    [SerializeField] private float explorationSpeed = 1f; // How fast to move between waypoints
    [SerializeField] private float entryPointDistance = 2f; // Distance from zone boundary to entry point
    
    [Header("References")]
    [SerializeField] private RobotMovement robotMovement;
    [SerializeField] private AStarNavigation pathfinder;
    [SerializeField] private BombGenerator bombGenerator;
    
    private List<Vector3> explorationWaypoints = new List<Vector3>();
    private int currentZone = 0; // 0 = zone A, 1 = zone B, 2 = zone C
    private int currentWaypoint = 0;
    private bool isExploring = false;
    private bool isNavigatingToZone = false;
    private float[] distanceReadings = new float[3]; // For trilateration

    // Beacon positions (for trilateration)
    private Vector3[] beaconPositions = new Vector3[3];
    
    private void Start()
    {
        // Initialize the beacons (you would place these at known positions in your scene)
        beaconPositions[0] = new Vector3(0, 0, 0);     // Beacon 1 position
        beaconPositions[1] = new Vector3(10, 0, 0);    // Beacon 2 position
        beaconPositions[2] = new Vector3(0, 0, 10);    // Beacon 3 position
        
        // Auto-check references
        if (robotMovement == null)
        {
            robotMovement = FindObjectOfType<RobotMovement>();
            if (robotMovement == null)
                Debug.LogError("No RobotMovement component found in the scene!");
        }
        
        if (pathfinder == null)
        {
            pathfinder = FindObjectOfType<AStarNavigation>();
            if (pathfinder == null)
                Debug.LogError("No AStarNavigation component found in the scene!");
        }

        // Auto-find BombGenerator if not assigned
        if (bombGenerator == null)
        {
            bombGenerator = FindObjectOfType<BombGenerator>();
            if (bombGenerator == null)
            {
                Debug.LogError("ExplorationController: BombGenerator reference not found in the scene!");
            }
        }
    }
    
    public void StartExploration()
    {
        Debug.Log("Starting exploration process...");
        currentZone = 0;
        isNavigatingToZone = true;
        StartCoroutine(NavigateToNextZone());
    }
    
    private IEnumerator NavigateToNextZone()
    {
        Debug.Log("Navigating to Zone " + (currentZone + 1));

        // Get the current zone
        Vector2[] zone = null;
        switch (currentZone)
        {
            case 0: zone = zoneA; break;
            case 1: zone = zoneB; break;
            case 2: zone = zoneC; break;
        }

        if (zone == null || zone.Length < 4)
        {
            Debug.LogError("Zone " + (currentZone + 1) + " is not properly defined!");
            yield break;
        }

        // Calculate zone center
        Vector2 zoneCenter = Vector2.zero;
        foreach (Vector2 point in zone)
        {
            zoneCenter += point;
        }
        zoneCenter /= zone.Length;

        // Use the center as entry point
        Vector3 entryPoint = new Vector3(zoneCenter.x, 0, zoneCenter.y);
        Debug.Log($"Moving to entry point: {entryPoint}. Robot start pos: {robotMovement.transform.position}");

        // Navigate to entry point
        robotMovement.FindPath(entryPoint);

        // --- MODIFIED WAIT LOOP ---
        float targetDistance = 0.5f; // Use a threshold slightly larger than robot's stoppingDistance
        float checkTimer = 0f; // Timer for periodic logging
        float navigationTimeout = 60.0f; // Max time to wait for navigation
        float navigationStartTime = Time.time;

        while (Vector3.Distance(robotMovement.transform.position, entryPoint) > targetDistance)
        {
            // // Check for timeout
            // if (Time.time - navigationStartTime > navigationTimeout)
            // {
            //     Debug.LogError($"NavigateToNextZone: Timeout waiting to reach entry point {entryPoint}. Aborting.");
            //     yield break;
            // }

            // Log distance periodically
            checkTimer += Time.deltaTime;
            if (checkTimer >= 1.0f) // Log every 1 second
            {
                Debug.Log($"Waiting to reach entry point... Current Distance: {Vector3.Distance(robotMovement.transform.position, entryPoint):F2} (Target: <= {targetDistance})");
                checkTimer = 0f;
            }

            // If a bomb is detected during navigation, handle it
            if (robotMovement.HasDetectedBomb())
            {
                Debug.Log("Bomb detected while navigating to zone!");
                // isNavigatingToZone = false; // Flag seems unused
                yield break; // Stop navigating to zone if bomb found
            }

            // Check if robot stopped following path unexpectedly
            if (!robotMovement.IsFollowingPath())
            {
                // Check distance one last time in case it stopped exactly at the target in the previous frame
                if (Vector3.Distance(robotMovement.transform.position, entryPoint) <= targetDistance) {
                    Debug.Log("Robot stopped following path but is close enough to entry point.");
                    break; // Exit loop, proceed to exploration
                } else {
                    Debug.LogError($"Robot stopped following path before reaching entry point! Distance: {Vector3.Distance(robotMovement.transform.position, entryPoint):F2}. Aborting zone navigation.");
                    // Decide how to handle this - maybe try pathfinding again or skip zone?
                    MoveToNextZone(); // Try moving to the *next* zone as a fallback
                    yield break; // Stop this coroutine
                }
            }

            yield return null; // Wait for the next frame
        }
        // --- END OF MODIFIED WAIT LOOP ---

        Debug.Log($"Reached Zone {currentZone + 1} (Distance: {Vector3.Distance(robotMovement.transform.position, entryPoint):F2}), starting exploration within zone");
        // isNavigatingToZone = false; // Flag seems unused

        // Now that we're at the zone, start exploration within it
        GenerateZigZagPattern(currentZone);
        if (explorationWaypoints.Count > 0) // Only start if waypoints were generated
        {
            StartCoroutine(ExploreCurrentZone());
        }
        else
        {
            Debug.LogError("No exploration waypoints generated, cannot start ExploreCurrentZone. Moving to next zone.");
            MoveToNextZone(); // Skip exploration if no waypoints
        }
    }
    
    private IEnumerator ExploreCurrentZone()
    {
        Debug.Log($"ExploreCurrentZone started for Zone {currentZone + 1}. Waypoint count: {explorationWaypoints.Count}"); // Log start

        isExploring = true;
        currentWaypoint = 0; // Ensure waypoint index is reset

        // Check if there are any waypoints to explore
        if (explorationWaypoints == null || explorationWaypoints.Count == 0)
        {
            Debug.LogError($"ExploreCurrentZone: No waypoints available for Zone {currentZone + 1}. Cannot explore.");
            isExploring = false;
            // Decide what to do - move to next zone or stop?
            MoveToNextZone(); // Try moving to the next zone
            yield break; // Exit this coroutine
        }

        while (isExploring && currentWaypoint < explorationWaypoints.Count)
        {
            // Set the next waypoint as the target for the robot
            Vector3 targetPosition = explorationWaypoints[currentWaypoint];
            Debug.Log($"Exploring zone {currentZone + 1}, attempting to find path to waypoint {currentWaypoint + 1}/{explorationWaypoints.Count}: {targetPosition}");

            // Ensure robotMovement reference is valid
            if (robotMovement == null) {
                Debug.LogError("ExploreCurrentZone: robotMovement reference is null!");
                isExploring = false;
                yield break;
            }

            robotMovement.FindPath(targetPosition);

            // Give pathfinding a moment to start and set isFollowingPath
            yield return new WaitForSeconds(0.1f);

            // Wait until robot reaches the waypoint OR pathfinding fails/stops
            // Check isFollowingPath flag from RobotMovement
            float waitStartTime = Time.time;
            while (robotMovement.IsFollowingPath() && Vector3.Distance(robotMovement.transform.position, targetPosition) > 0.5f)
            {
                // Update localization using trilateration (optional during exploration)
                // UpdateTrilateration();

                // If a bomb is detected during exploration, handle it
                if (robotMovement.HasDetectedBomb())
                {
                    Debug.Log("Bomb detected during zone exploration!");
                    // Wait for a moment to let the robot get to the bomb and potentially defuse it
                    yield return new WaitForSeconds(1.0f); // Reduced wait time

                    // Check if the bomb is now defused (assuming Bomb script has an IsDefused property/method)
                    Bomb detectedBombScript = robotMovement.GetDetectedBomb()?.GetComponent<Bomb>();
                    if (detectedBombScript != null && detectedBombScript.isDefused)
                    {
                        Debug.Log("Bomb appears defused. Checking if zone is complete.");
                        if (CheckIfZoneComplete()) // Check if this was the last bomb etc.
                        {
                            MoveToNextZone();
                            yield break; // Exit exploration coroutine
                        }
                        else
                        {
                            // Continue exploration in this zone after defusal
                            Debug.Log("Continuing exploration in zone after defusal.");
                            // No 'continue' needed, loop will proceed naturally
                        }
                    }
                    else {
                        Debug.Log("Bomb still detected or not defused, continuing approach/wait.");
                        // Robot should continue moving towards waypoint (which might be near the bomb)
                    }
                }

                // Timeout check in case robot gets stuck
                if (Time.time - waitStartTime > 30.0f) { // 30 second timeout per waypoint
                    Debug.LogError($"ExploreCurrentZone: Timeout waiting to reach waypoint {currentWaypoint}. Robot might be stuck. Aborting zone.");
                    isExploring = false;
                    robotMovement.StopMoving();
                    MoveToNextZone(); // Try next zone
                    yield break;
                }

                yield return null; // Wait for the next frame
            }

            // Check *why* the inner loop exited
            if (!robotMovement.IsFollowingPath() && currentWaypoint < explorationWaypoints.Count && Vector3.Distance(robotMovement.transform.position, targetPosition) > 0.5f)
            {
                Debug.LogWarning($"ExploreCurrentZone: Robot stopped following path before reaching waypoint {currentWaypoint}. Distance: {Vector3.Distance(robotMovement.transform.position, targetPosition):F2}. Trying next waypoint or greedy.");
                // Maybe pathfinding failed mid-way, or something else stopped it.
                // Let's try moving to the next waypoint anyway, or switch to greedy if it keeps happening.
            } else {
                Debug.Log($"Reached waypoint {currentWaypoint} or very close.");
            }


            currentWaypoint++;

            // If finished with the waypoints in current zone
            if (currentWaypoint >= explorationWaypoints.Count)
            {
                Debug.Log($"Finished all generated waypoints for Zone {currentZone + 1}.");
                if (CheckIfZoneComplete()) // Check if bomb was found/defused
                {
                    MoveToNextZone();
                    yield break; // Exit coroutine
                }
                else
                {
                    Debug.Log("Zone exploration complete but objective not met (bomb not found/defused?), switching to Greedy Exploration.");
                    SwitchToGreedyExploration();
                    yield break; // Exit this coroutine, Greedy takes over
                }
            }

            yield return new WaitForSeconds(0.1f); // Small delay before starting next waypoint
        }

        Debug.Log($"ExploreCurrentZone finished loop for Zone {currentZone + 1}. isExploring={isExploring}");
        isExploring = false; // Ensure flag is reset if loop finishes unexpectedly
    }
    
    private bool CheckIfZoneComplete()
    {
        if (bombGenerator == null)
        {
            Debug.LogError("CheckIfZoneComplete: BombGenerator reference is missing!");
            return false; // Cannot determine status without generator
        }

        // Determine which list of bombs corresponds to the current zone
        List<GameObject> zoneBombs = null;
        switch (currentZone) // currentZone is 0, 1, 2
        {
            case 0:
                zoneBombs = bombGenerator.bombA;
                break;
            case 1:
                zoneBombs = bombGenerator.bombB;
                break;
            case 2:
                zoneBombs = bombGenerator.bombC;
                break;
            default:
                Debug.LogError($"CheckIfZoneComplete: Invalid currentZone index {currentZone}");
                return false;
        }

        if (zoneBombs == null || zoneBombs.Count == 0)
        {
            Debug.LogWarning($"CheckIfZoneComplete: No bombs found in the list for Zone {currentZone + 1}. Assuming complete.");
            return true; // Or false, depending on desired behavior if list is empty
        }

        // Find the *active* bomb within that list
        foreach (GameObject bombObj in zoneBombs)
        {
            // Check if this is the one bomb activated by BombGenerator
            if (bombObj != null && bombObj.activeInHierarchy)
            {
                Bomb bombScript = bombObj.GetComponent<Bomb>();
                if (bombScript != null)
                {
                    // Check if the active bomb for this zone is defused
                    if (bombScript.isDefused)
                    {
                        Debug.Log($"CheckIfZoneComplete: Active bomb '{bombObj.name}' for Zone {currentZone + 1} IS defused. Zone complete.");
                        return true; // The active bomb for this zone is defused
                    }
                    else
                    {
                        Debug.Log($"CheckIfZoneComplete: Active bomb '{bombObj.name}' for Zone {currentZone + 1} is NOT defused. Zone incomplete.");
                        return false; // The active bomb for this zone is not yet defused
                    }
                }
                else
                {
                    Debug.LogError($"CheckIfZoneComplete: Active bomb '{bombObj.name}' for Zone {currentZone + 1} is missing Bomb script!");
                    return false; // Cannot determine status
                }
            }
        }

        // If we looped through the zone's list and didn't find any active bomb
        // (This shouldn't happen if BombGenerator worked correctly)
        Debug.LogWarning($"CheckIfZoneComplete: Could not find an *active* bomb in the list for Zone {currentZone + 1}. Assuming incomplete.");
        return false;
    }
    
    private void MoveToNextZone()
    {
        currentZone++;
        if (currentZone <= 2)
        {
            Debug.Log("Moving to next zone: " + (currentZone + 1));
            isNavigatingToZone = true;
            StartCoroutine(NavigateToNextZone());
        }
        else
        {
            Debug.Log("All zones explored!");
            StopExploration();
        }
    }
    
    private void GenerateZigZagPattern(int zoneIndex)
    {
        explorationWaypoints.Clear();
        currentWaypoint = 0; // Reset waypoint index

        Vector2[] zone;
        switch (zoneIndex)
        {
            case 0: zone = zoneA; break;
            case 1: zone = zoneB; break;
            case 2: zone = zoneC; break;
            default:
                Debug.LogError($"GenerateZigZagPattern: Invalid zone index {zoneIndex}");
                return;
        }

        if (zone == null || zone.Length < 4)
        {
            Debug.LogError($"GenerateZigZagPattern: Zone {zoneIndex + 1} is not properly defined (needs 4 points).");
            return;
        }
        if (zigzagSpacing <= 0)
        {
            Debug.LogError("GenerateZigZagPattern: zigzagSpacing must be positive.");
            return;
        }

        // Calculate the bounds of the zone
        float minX = Mathf.Infinity, maxX = -Mathf.Infinity;
        float minZ = Mathf.Infinity, maxZ = -Mathf.Infinity;

        foreach (Vector2 point in zone)
        {
            minX = Mathf.Min(minX, point.x);
            maxX = Mathf.Max(maxX, point.x);
            minZ = Mathf.Min(minZ, point.y); // y component represents Z in world space
            maxZ = Mathf.Max(maxZ, point.y); // y component represents Z in world space
        }

        Debug.Log($"Zone {zoneIndex + 1} Bounds: X({minX:F1} to {maxX:F1}), Z({minZ:F1} to {maxZ:F1})");

        // Check if zone has valid dimensions
        if (maxX <= minX || maxZ <= minZ)
        {
            Debug.LogError($"GenerateZigZagPattern: Zone {zoneIndex + 1} has invalid dimensions (zero or negative area). Check zone points.");
            return;
        }

        // Add a small margin inside the zone bounds for the pattern
        float margin = 0.5f; // Adjust as needed
        minX += margin;
        maxX -= margin;
        minZ += margin;
        maxZ -= margin;

        // Ensure bounds are still valid after adding margin
        if (maxX <= minX || maxZ <= minZ)
        {
            Debug.LogWarning($"GenerateZigZagPattern: Zone {zoneIndex + 1} is too small for the margin. Waypoints might be outside or very close.");
            // Reset bounds if margin made them invalid, but proceed cautiously
            minX = (minX + maxX) / 2f; maxX = minX;
            minZ = (minZ + maxZ) / 2f; maxZ = minZ;
            // Or simply return if too small:
            // Debug.LogError($"GenerateZigZagPattern: Zone {zoneIndex + 1} too small for margin."); return;
        }


        // Generate zigzag pattern
        bool goingRight = true;
        for (float z = minZ; z <= maxZ; z += zigzagSpacing)
        {
            if (goingRight)
            {
                explorationWaypoints.Add(new Vector3(minX, 0f, z));
                explorationWaypoints.Add(new Vector3(maxX, 0f, z));
            }
            else
            {
                explorationWaypoints.Add(new Vector3(maxX, 0f, z));
                explorationWaypoints.Add(new Vector3(minX, 0f, z));
            }
            goingRight = !goingRight;

            // Add a small vertical step if the loop might only run once
            if (maxZ - minZ < zigzagSpacing && z == minZ) {
                if (z + zigzagSpacing > maxZ) break; // Prevent going over maxZ if spacing is large
            }
        }

        // If only one row was generated and maxZ > minZ, add the last row explicitly if needed
        if (explorationWaypoints.Count == 2 && maxZ > minZ + float.Epsilon) {
            float z = maxZ; // Use the max Z boundary
            if (goingRight) { // Should be false after the first row
                explorationWaypoints.Add(new Vector3(maxX, 0f, z));
                explorationWaypoints.Add(new Vector3(minX, 0f, z));
            } else { // Should be true if only one row was added
                explorationWaypoints.Add(new Vector3(minX, 0f, z));
                explorationWaypoints.Add(new Vector3(maxX, 0f, z));
            }
            Debug.Log("Added explicit last row for zigzag pattern.");
        }


        if (explorationWaypoints.Count == 0)
        {
            Debug.LogError($"GenerateZigZagPattern: No waypoints generated for Zone {zoneIndex + 1}. Check zone size and zigzagSpacing.");
        }
        else
        {
            Debug.Log($"Generated {explorationWaypoints.Count} waypoints for Zone {zoneIndex + 1}. First waypoint: {explorationWaypoints[0]}");
        }
    }
    
    // Trilateration algorithm to estimate robot position
    private void UpdateTrilateration()
    {
        if (robotMovement == null) return;
        
        // Simulate distance readings from beacons
        for (int i = 0; i < 3; i++)
        {
            distanceReadings[i] = Vector3.Distance(robotMovement.transform.position, beaconPositions[i]);
            
            // Add some noise to simulate real-world sensors
            distanceReadings[i] += Random.Range(-0.1f, 0.1f);
        }
        
        // Perform trilateration calculation
        Vector3 estimatedPosition = CalculatePosition(
            beaconPositions[0], beaconPositions[1], beaconPositions[2],
            distanceReadings[0], distanceReadings[1], distanceReadings[2]);
        
        // For debugging only
        Debug.LogFormat("Trilateration estimate: {0}, Actual position: {1}", 
            estimatedPosition, robotMovement.transform.position);
    }
    
    // Trilateration calculation
    private Vector3 CalculatePosition(Vector3 p1, Vector3 p2, Vector3 p3, float r1, float r2, float r3)
    {
        // Corrected: Using x and z coordinates from the positions
        float x1 = p1.x; float z1 = p1.z;
        float x2 = p2.x; float z2 = p2.z;
        float x3 = p3.x; float z3 = p3.z;
        
        float A = 2 * (x2 - x1);
        float B = 2 * (z2 - z1);
        float C = r1 * r1 - r2 * r2 - x1 * x1 + x2 * x2 - z1 * z1 + z2 * z2;
        float D = 2 * (x3 - x2);
        float E = 2 * (z3 - z2);
        float F = r2 * r2 - r3 * r3 - x2 * x2 + x3 * x3 - z2 * z2 + z3 * z3;
        
        float denominator = (E * A - B * D);
        
        if (Mathf.Abs(denominator) < 0.001f)
        {
            // Singular case, beacons might be collinear
            return robotMovement.transform.position; // Fall back to current position
        }
        
        float x = (C * E - F * B) / denominator;
        float z = (C * D - A * F) / (B * D - A * E);
        
        return new Vector3(x, robotMovement.transform.position.y, z);
    }
    
    // Convert from zigzag to greedy exploration mid-exploration
    public void SwitchToGreedyExploration()
    {
        StopAllCoroutines();
        StartCoroutine(GreedyExploration());
    }
    
    private IEnumerator GreedyExploration()
    {
        Debug.Log("Starting greedy exploration in Zone " + (currentZone + 1));
        while (true)
        {
            // Get LIDAR readings from the robot
            Dictionary<Vector3, float> lidarReadings = robotMovement.GetLidarReadings();
            
            // Find the direction with the maximum open space
            Vector3 bestDirection = Vector3.zero;
            float maxDistance = 0f;
            
            foreach (var reading in lidarReadings)
            {
                if (reading.Value > maxDistance)
                {
                    maxDistance = reading.Value;
                    bestDirection = reading.Key;
                }
            }
            
            // Normalize the best direction
            bestDirection.Normalize();
            
            // Set a target point in that direction
            Vector3 targetPoint = robotMovement.transform.position + bestDirection * 5.0f;
            
            Debug.Log("Greedy exploration - moving to " + targetPoint);
            
            // Make sure the target is within the current zone
            targetPoint = ConstrainToCurrentZone(targetPoint);
            
            // Move toward that point
            robotMovement.FindPath(targetPoint);
            
            // Wait a bit before recalculating
            yield return new WaitForSeconds(2.0f);
            
            // If a bomb is detected, stop exploration
            if (robotMovement.HasDetectedBomb())
            {
                Debug.Log("Bomb detected during greedy exploration!");
                yield return new WaitForSeconds(2.0f); // Wait for potential defusal
                
                // If finished with this zone, move to the next
                if (CheckIfZoneComplete())
                {
                    MoveToNextZone();
                    yield break;
                }
            }
        }
    }
    
    private Vector3 ConstrainToCurrentZone(Vector3 targetPoint)
    {
        Vector2[] zone;
        switch (currentZone)
        {
            case 0: zone = zoneA; break;
            case 1: zone = zoneB; break;
            case 2: zone = zoneC; break;
            default: return targetPoint;
        }
        
        if (zone == null || zone.Length < 3) return targetPoint;
        
        // Simple check - if the point is outside the zone, move it toward zone center
        if (!IsPointInZone(new Vector2(targetPoint.x, targetPoint.z), zone))
        {
            // Calculate zone center
            Vector2 zoneCenter = Vector2.zero;
            foreach (Vector2 point in zone)
            {
                zoneCenter += point;
            }
            zoneCenter /= zone.Length;
            
            // Move the target point toward center
            Vector2 direction = (zoneCenter - new Vector2(targetPoint.x, targetPoint.z)).normalized;
            Vector2 constrained = new Vector2(targetPoint.x, targetPoint.z) + direction * 2.0f;
            
            Debug.Log("Constraining target point from " + targetPoint + " to " + 
                      new Vector3(constrained.x, targetPoint.y, constrained.y));
                      
            return new Vector3(constrained.x, targetPoint.y, constrained.y);
        }
        
        return targetPoint;
    }
    
    private bool IsPointInZone(Vector2 point, Vector2[] zone)
    {
        // Implementation of the point-in-polygon algorithm
        bool isInside = false;
        for (int i = 0, j = zone.Length - 1; i < zone.Length; j = i++)
        {
            if (((zone[i].y > point.y) != (zone[j].y > point.y)) &&
                (point.x < (zone[j].x - zone[i].x) * (point.y - zone[i].y) / (zone[j].y - zone[i].y) + zone[i].x))
            {
                isInside = !isInside;
            }
        }
        return isInside;
    }
    
    private void OnDrawGizmos()
    {
        // Draw the exploration zones
        DrawZone(zoneA, Color.red);
        DrawZone(zoneB, Color.green);
        DrawZone(zoneC, Color.blue);
        
        // Draw the exploration path
        Gizmos.color = Color.yellow;
        for (int i = 0; i < explorationWaypoints.Count - 1; i++)
        {
            Gizmos.DrawLine(explorationWaypoints[i], explorationWaypoints[i + 1]);
            Gizmos.DrawSphere(explorationWaypoints[i], 0.2f);
        }
        
        if (explorationWaypoints.Count > 0)
        {
            Gizmos.DrawSphere(explorationWaypoints[explorationWaypoints.Count - 1], 0.2f);
        }
        
        // Draw beacon positions
        Gizmos.color = Color.cyan;
        for (int i = 0; i < beaconPositions.Length; i++)
        {
            Gizmos.DrawSphere(beaconPositions[i], 0.5f);
        }
    }
    
    private void DrawZone(Vector2[] zone, Color color)
    {
        if (zone == null || zone.Length < 4) return;
        
        Gizmos.color = color;
        for (int i = 0; i < zone.Length; i++)
        {
            // Corrected: Vector2.x maps to world X, Vector2.y maps to world Z
            Vector3 p1 = new Vector3(zone[i].x, 0, zone[i].y);
            Vector3 p2 = new Vector3(zone[(i + 1) % zone.Length].x, 0, zone[(i + 1) % zone.Length].y);
            Gizmos.DrawLine(p1, p2);
        }
    }

    public void StopExploration()
    {
        isExploring = false;
        isNavigatingToZone = false;
        StopAllCoroutines();
        
        // Stop the robot from moving
        if (robotMovement != null)
        {
            // Clear any current path
            robotMovement.StopMoving();
        }
    }

}