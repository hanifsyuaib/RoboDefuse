using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Linq; // Required for LINQ operations like OrderByDescending

public class ExplorationController : MonoBehaviour
{
    public enum ExplorationMode { ZigZag, Greedy }

    [Header("Zone Definitions")]
    [Tooltip("Define 4 corner points (X,Z) for each zone.")]
    [SerializeField] private Vector2[] zoneACorners; // Expecting 4 Vector2 points
    [SerializeField] private Vector2[] zoneBCorners;
    [SerializeField] private Vector2[] zoneCCorners;

    [Header("Exploration Settings")]
    [SerializeField] private ExplorationMode initialMode = ExplorationMode.ZigZag;
    [SerializeField] private float zigzagSpacing = 2.0f; // Distance between zigzag lines
    [SerializeField] private float zoneMargin = 1.0f; // Keep waypoints inside zone boundaries by this margin
    [SerializeField] private float greedyStepDistance = 5.0f; // How far to move in the chosen greedy direction
    [SerializeField] private float greedyCheckInterval = 1.0f; // How often to re-evaluate greedy direction

    [Header("Localization")]
    [Tooltip("Assign known beacon GameObjects.")]
    [SerializeField] private Transform[] beacons; // Should have 3 beacons for trilateration
    [SerializeField] private float localizationUpdateInterval = 0.5f;

    [Header("References")]
    [SerializeField] private RobotMovement robotMovement;
    [SerializeField] private BombGenerator bombGenerator; // To know which bombs belong to which zone

    // State Variables
    private int currentZoneIndex = -1; // 0=A, 1=B, 2=C
    private List<Vector3> currentWaypoints = new List<Vector3>();
    private int currentWaypointTargetIndex = 0;
    private bool isExploring = false;
    private bool isNavigatingToZone = false;
    private ExplorationMode currentMode;
    private Vector3 estimatedPosition; // Position estimated via localization
    private Coroutine explorationCoroutine = null;
    private Coroutine navigationCoroutine = null;
    private Coroutine localizationCoroutine = null;

    void Start()
    {
        // Auto-find references if not assigned
        if (robotMovement == null) robotMovement = FindObjectOfType<RobotMovement>();
        if (bombGenerator == null) bombGenerator = FindObjectOfType<BombGenerator>();

        if (robotMovement == null) Debug.LogError("ExplorationController: RobotMovement reference missing!");
        if (bombGenerator == null) Debug.LogError("ExplorationController: BombGenerator reference missing!");
        if (beacons == null || beacons.Length < 3) Debug.LogError("ExplorationController: Requires at least 3 beacons for trilateration!");

        currentMode = initialMode;
        estimatedPosition = robotMovement != null ? robotMovement.transform.position : Vector3.zero; // Initial estimate

        // Start localization updates
        if (beacons != null && beacons.Length >= 3 && robotMovement != null)
        {
            localizationCoroutine = StartCoroutine(UpdateLocalizationRoutine());
        }
    }

    // --- Public Control Methods ---

    /// <summary>
    /// Starts the entire exploration process, beginning with Zone A.
    /// </summary>
    public void StartExploration()
    {
        if (isExploring || isNavigatingToZone)
        {
            Debug.LogWarning("Exploration already in progress.");
            return;
        }
        if (robotMovement == null)
        {
            Debug.LogError("Cannot start exploration: RobotMovement reference missing.");
            return;
        }

        Debug.Log("Starting exploration sequence...");
        currentZoneIndex = -1; // Reset zone index
        MoveToNextZone();
    }

    /// <summary>
    /// Stops the current exploration or navigation task immediately.
    /// </summary>
    public void StopExploration()
    {
        Debug.Log("Stopping exploration...");
        isExploring = false;
        isNavigatingToZone = false;

        if (explorationCoroutine != null) StopCoroutine(explorationCoroutine);
        if (navigationCoroutine != null) StopCoroutine(navigationCoroutine);
        // Don't stop localization coroutine usually

        explorationCoroutine = null;
        navigationCoroutine = null;

        if (robotMovement != null)
        {
            robotMovement.StopMoving();
        }
        currentWaypoints.Clear();
    }

    /// <summary>
    /// Switches the exploration mode between ZigZag and Greedy.
    /// If exploring, restarts exploration in the current zone with the new mode.
    /// </summary>
    public void SwitchExplorationMode(ExplorationMode newMode)
    {
        if (currentMode == newMode) return;

        Debug.Log($"Switching exploration mode to {newMode}");
        currentMode = newMode;

        // If currently exploring a zone, stop the old coroutine and start a new one
        if (isExploring && explorationCoroutine != null)
        {
            StopCoroutine(explorationCoroutine);
            explorationCoroutine = null; // Clear the reference
            robotMovement.StopMoving(); // Stop current movement
            StartExplorationInCurrentZone(); // Restart with new mode
        }
    }

    // --- Core Logic ---

    private void MoveToNextZone()
    {
        if (isNavigatingToZone) return; // Already navigating

        currentZoneIndex++;
        if (currentZoneIndex > 2)
        {
            Debug.Log("All zones explored!");
            StopExploration(); // Finished
            return;
        }

        Debug.Log($"Preparing to navigate to Zone {(char)('A' + currentZoneIndex)}");
        navigationCoroutine = StartCoroutine(NavigateToZoneRoutine(currentZoneIndex));
    }

    private IEnumerator NavigateToZoneRoutine(int zoneIndex)
    {
        isNavigatingToZone = true;
        isExploring = false; // Ensure not exploring while navigating

        Vector2[] zoneCorners = GetZoneCorners(zoneIndex);
        if (zoneCorners == null || zoneCorners.Length < 3)
        {
            Debug.LogError($"Zone {(char)('A' + zoneIndex)} definition invalid. Stopping exploration.");
            StopExploration();
            yield break;
        }

        // Calculate a safe entry point (e.g., center or closest point on boundary)
        Vector3 entryPoint = CalculateZoneEntryPoint(zoneCorners);
        Debug.Log($"Navigating to Zone {(char)('A' + zoneIndex)} entry point: {entryPoint}");

        robotMovement.MoveTo(entryPoint);

        // Wait until the robot reaches the vicinity of the entry point
        float navigationTimeout = 60.0f; // Max time to wait
        float navigationStartTime = Time.time;
        while (Vector3.Distance(estimatedPosition, entryPoint) > 1.0f) // Use estimated position
        {
            if (Time.time - navigationStartTime > navigationTimeout)
            {
                Debug.LogError($"Timeout navigating to Zone {(char)('A' + zoneIndex)}. Stopping.");
                StopExploration();
                yield break;
            }

            // Check if robot stopped unexpectedly (e.g., pathfinding failed)
            if (!robotMovement.IsFollowingPath() && Vector3.Distance(estimatedPosition, entryPoint) > 1.5f)
            {
                 Debug.LogError($"Robot stopped unexpectedly while navigating to Zone {(char)('A' + zoneIndex)}. Stopping.");
                 StopExploration();
                 yield break;
            }

            // Check for bombs during navigation
            if (robotMovement.HasDetectedBomb())
            {
                Debug.Log($"Bomb detected while navigating to Zone {(char)('A' + zoneIndex)}! Handling bomb...");
                yield return HandleDetectedBomb(); // Wait for defusal attempt
                // Decide whether to continue navigation or start exploring if zone complete
                if (CheckIfZoneComplete(zoneIndex))
                {
                    Debug.Log($"Zone {(char)('A' + zoneIndex)} completed after detecting bomb during navigation.");
                    isNavigatingToZone = false;
                    MoveToNextZone(); // Move to the *next* zone
                    yield break;
                }
                // else: Bomb detected wasn't the target or wasn't defused, continue navigation? Or maybe start exploring?
                // For now, let's assume we continue navigation if zone isn't complete.
                Debug.Log("Zone not complete, continuing navigation...");
                // Re-issue move command in case bomb handling stopped it
                 robotMovement.MoveTo(entryPoint);
            }

            yield return null; // Wait for the next frame
        }

        Debug.Log($"Reached Zone {(char)('A' + zoneIndex)} entry point.");
        isNavigatingToZone = false;
        navigationCoroutine = null;

        // Start exploring the zone we just arrived at
        StartExplorationInCurrentZone();
    }

    private void StartExplorationInCurrentZone()
    {
        if (isExploring) return; // Already exploring

        Debug.Log($"Starting exploration in Zone {(char)('A' + currentZoneIndex)} using {currentMode} mode.");
        isExploring = true;
        isNavigatingToZone = false; // Ensure navigation flag is off

        if (currentMode == ExplorationMode.ZigZag)
        {
            GenerateZigZagPattern(currentZoneIndex);
            if (currentWaypoints.Count > 0)
            {
                explorationCoroutine = StartCoroutine(FollowWaypointsRoutine());
            }
            else
            {
                Debug.LogWarning($"No ZigZag waypoints generated for Zone {(char)('A' + currentZoneIndex)}. Switching to Greedy.");
                SwitchExplorationMode(ExplorationMode.Greedy); // Fallback to Greedy
            }
        }
        else // Greedy Mode
        {
            explorationCoroutine = StartCoroutine(GreedyExplorationRoutine());
        }
    }


    private IEnumerator FollowWaypointsRoutine()
    {
        Debug.Log($"Starting waypoint following. {currentWaypoints.Count} waypoints.");
        currentWaypointTargetIndex = 0;

        while (currentWaypointTargetIndex < currentWaypoints.Count && isExploring)
        {
            Vector3 targetWaypoint = currentWaypoints[currentWaypointTargetIndex];
            Debug.Log($"Moving to waypoint {currentWaypointTargetIndex + 1}/{currentWaypoints.Count}: {targetWaypoint}");
            robotMovement.MoveTo(targetWaypoint);

            float waypointStartTime = Time.time;
            float waypointTimeout = 45.0f; // Max time per waypoint

            // Wait until robot reaches the waypoint or detects a bomb
            while (Vector3.Distance(estimatedPosition, targetWaypoint) > 1.0f && isExploring) // Use estimated position
            {
                 if (Time.time - waypointStartTime > waypointTimeout)
                 {
                     Debug.LogError($"Timeout reaching waypoint {currentWaypointTargetIndex + 1}. Stopping exploration.");
                     StopExploration();
                     yield break;
                 }

                 // Check if robot stopped unexpectedly
                 if (!robotMovement.IsFollowingPath() && Vector3.Distance(estimatedPosition, targetWaypoint) > 1.5f)
                 {
                      Debug.LogError($"Robot stopped unexpectedly while moving to waypoint {currentWaypointTargetIndex + 1}. Stopping exploration.");
                      StopExploration();
                      yield break;
                 }

                // Check for bombs during waypoint following
                if (robotMovement.HasDetectedBomb())
                {
                    Debug.Log("Bomb detected while following waypoints! Handling bomb...");
                    yield return HandleDetectedBomb();
                    if (CheckIfZoneComplete(currentZoneIndex))
                    {
                        Debug.Log($"Zone {(char)('A' + currentZoneIndex)} completed after detecting bomb.");
                        isExploring = false; // Stop current exploration loop
                        MoveToNextZone();
                        yield break; // Exit this coroutine
                    }
                    else
                    {
                        // Bomb detected wasn't the target or wasn't defused. Continue exploration.
                        Debug.Log("Zone not complete, continuing waypoint following...");
                        // Re-issue move command in case bomb handling stopped it
                        robotMovement.MoveTo(targetWaypoint);
                    }
                }
                yield return null; // Wait for next frame
            }

             if (!isExploring) yield break; // Check if StopExploration was called

            Debug.Log($"Reached waypoint {currentWaypointTargetIndex + 1}.");
            currentWaypointTargetIndex++;

            yield return new WaitForSeconds(0.2f); // Small pause before next waypoint
        }

        if (isExploring) // Only if the loop completed naturally
        {
            Debug.Log($"Finished all waypoints for Zone {(char)('A' + currentZoneIndex)}.");
            // If waypoints finished but zone not complete, maybe switch to greedy?
            if (!CheckIfZoneComplete(currentZoneIndex))
            {
                Debug.LogWarning($"ZigZag finished but zone {(char)('A' + currentZoneIndex)} not complete. Switching to Greedy.");
                SwitchExplorationMode(ExplorationMode.Greedy);
            }
            else
            {
                 MoveToNextZone(); // Zone is complete, move on
            }
        }
        isExploring = false; // Mark exploration as finished for this zone/mode
        explorationCoroutine = null;
    }

    private IEnumerator GreedyExplorationRoutine()
    {
        Debug.Log($"Starting Greedy exploration in Zone {(char)('A' + currentZoneIndex)}.");

        while (isExploring)
        {
            // 1. Get LIDAR data
            Dictionary<Vector3, float> readings = robotMovement.GetLidarReadings();
            if (readings.Count == 0)
            {
                Debug.LogWarning("Greedy: No LIDAR readings available.");
                yield return new WaitForSeconds(greedyCheckInterval);
                continue;
            }

            // 2. Find direction with max distance (simplistic approach)
            Vector3 bestDirection = Vector3.zero;
            float maxDistance = 0f;
            foreach (var reading in readings)
            {
                if (reading.Value > maxDistance)
                {
                    maxDistance = reading.Value;
                    bestDirection = reading.Key; // Key is the direction vector
                }
            }

            if (bestDirection == Vector3.zero || maxDistance < 1.0f) // Avoid moving if completely blocked
            {
                Debug.Log("Greedy: No clear path found or blocked. Waiting.");
                 // Optional: Implement more complex logic like turning around or random movement
                 robotMovement.StopMoving(); // Stop if blocked
                 yield return new WaitForSeconds(greedyCheckInterval * 2); // Wait longer if blocked
                 continue;
            }

            // 3. Calculate target point
            Vector3 targetPoint = estimatedPosition + bestDirection.normalized * greedyStepDistance; // Use estimated position

            // 4. Constrain target point to the current zone
            targetPoint = ConstrainToZone(targetPoint, currentZoneIndex);

            Debug.Log($"Greedy: Moving towards {targetPoint} (Max distance: {maxDistance:F2} in direction {bestDirection.normalized})");
            robotMovement.MoveTo(targetPoint);

            // 5. Wait and check for bombs while moving
            float moveStartTime = Time.time;
            float moveTimeout = 20.0f; // Timeout for this greedy step
            while (robotMovement.IsFollowingPath() && Time.time - moveStartTime < moveTimeout && isExploring)
            {
                 // Check for bombs frequently during greedy movement
                 if (robotMovement.HasDetectedBomb())
                 {
                     Debug.Log("Bomb detected during Greedy exploration! Handling bomb...");
                     yield return HandleDetectedBomb();
                     if (CheckIfZoneComplete(currentZoneIndex))
                     {
                         Debug.Log($"Zone {(char)('A' + currentZoneIndex)} completed after detecting bomb during Greedy.");
                         isExploring = false; // Stop greedy loop
                         MoveToNextZone();
                         yield break; // Exit this coroutine
                     }
                     else
                     {
                         Debug.Log("Zone not complete, continuing Greedy exploration...");
                         // Break inner loop to re-evaluate direction after handling bomb
                         break;
                     }
                 }
                 yield return new WaitForSeconds(0.1f); // Check frequently
            }

             if (!isExploring) yield break; // Check if StopExploration was called

            // Wait for the check interval before the next greedy step
            yield return new WaitForSeconds(greedyCheckInterval);
        }
        isExploring = false; // Mark exploration as finished
        explorationCoroutine = null;
    }

    /// <summary>
    /// Handles the process when a bomb is detected (e.g., wait for defusal).
    /// </summary>
    private IEnumerator HandleDetectedBomb()
    {
        robotMovement.StopMoving(); // Stop movement to focus on the bomb
        Debug.Log("Handling detected bomb. Waiting for potential defusal...");
        // RobotMovement script handles the actual defusal attempt when close enough.
        // We just wait here for a short period to see if it gets defused.
        yield return new WaitForSeconds(2.0f); // Give time for RobotMovement's defusal logic

        GameObject detected = robotMovement.GetDetectedBomb(); // Re-check if still detected
        if (detected != null)
        {
             Bomb bombScript = detected.GetComponent<Bomb>();
             if (bombScript != null && !bombScript.isDefused)
             {
                 Debug.Log("Bomb still detected and not defused. Robot might need to get closer or defusal failed.");
                 // Optional: Add logic to explicitly move closer if needed
             } else {
                 Debug.Log("Bomb no longer detected or already defused.");
             }
        } else {
             Debug.Log("Bomb no longer detected (likely defused or out of range).");
        }
    }


    // --- Localization ---

    private IEnumerator UpdateLocalizationRoutine()
    {
        while (true)
        {
            if (robotMovement != null && beacons != null && beacons.Length >= 3)
            {
                // Get simulated distance readings from RobotMovement
                float r1 = robotMovement.GetDistanceToBeacon(beacons[0].position);
                float r2 = robotMovement.GetDistanceToBeacon(beacons[1].position);
                float r3 = robotMovement.GetDistanceToBeacon(beacons[2].position);

                // Perform trilateration
                estimatedPosition = CalculateTrilateration(beacons[0].position, beacons[1].position, beacons[2].position, r1, r2, r3);

                // Optional: Add smoothing or Kalman filter here
                // Debug.Log($"Trilateration Estimate: {estimatedPosition:F2}, Actual: {robotMovement.transform.position:F2}");
            }
            yield return new WaitForSeconds(localizationUpdateInterval);
        }
    }

    // Basic 2D Trilateration (assumes beacons and robot are roughly on the same Y plane)
    private Vector3 CalculateTrilateration(Vector3 p1, Vector3 p2, Vector3 p3, float r1, float r2, float r3)
    {
        // Using formula from https://en.wikipedia.org/wiki/Trilateration#Mathematical_basis_2D
        // Simplified for XZ plane (Y is ignored for calculation, uses robot's current Y)

        float x1 = p1.x, z1 = p1.z;
        float x2 = p2.x, z2 = p2.z;
        float x3 = p3.x, z3 = p3.z;

        // Check for valid inputs (non-negative radii)
        r1 = Mathf.Max(0, r1);
        r2 = Mathf.Max(0, r2);
        r3 = Mathf.Max(0, r3);

        float A = 2 * x2 - 2 * x1;
        float B = 2 * z2 - 2 * z1;
        float C = r1 * r1 - r2 * r2 - x1 * x1 + x2 * x2 - z1 * z1 + z2 * z2;
        float D = 2 * x3 - 2 * x2;
        float E = 2 * z3 - 2 * z2;
        float F = r2 * r2 - r3 * r3 - x2 * x2 + x3 * x3 - z2 * z2 + z3 * z3;

        float denominator = (E * A - B * D);

        if (Mathf.Abs(denominator) < 0.001f)
        {
            // Beacons might be collinear, cannot solve accurately
            // Debug.LogWarning("Trilateration: Denominator close to zero, beacons might be collinear. Returning last known estimate.");
            return estimatedPosition; // Return last estimate or robot's transform
        }

        float x = (C * E - F * B) / denominator;
        float z = (C * D - A * F) / (B * D - A * E); // Note the denominator swap for z

        return new Vector3(x, robotMovement.transform.position.y, z); // Use robot's current Y
    }


    // --- Zone & Waypoint Generation ---

    private Vector2[] GetZoneCorners(int zoneIndex)
    {
        switch (zoneIndex)
        {
            case 0: return zoneACorners;
            case 1: return zoneBCorners;
            case 2: return zoneCCorners;
            default: return null;
        }
    }

    private Vector3 CalculateZoneEntryPoint(Vector2[] zoneCorners)
    {
        // Simple approach: Calculate the center of the zone
        Vector2 center = Vector2.zero;
        if (zoneCorners == null || zoneCorners.Length == 0) return Vector3.zero;
        foreach (Vector2 corner in zoneCorners)
        {
            center += corner;
        }
        center /= zoneCorners.Length;
        return new Vector3(center.x, 0, center.y); // Assume ground level (Y=0)
    }

    private void GenerateZigZagPattern(int zoneIndex)
    {
        currentWaypoints.Clear();
        Vector2[] corners = GetZoneCorners(zoneIndex);
        if (corners == null || corners.Length < 4)
        {
            Debug.LogError($"Cannot generate ZigZag for Zone {(char)('A' + zoneIndex)}: Invalid corners.");
            return;
        }

        // Find min/max X and Z, applying margin
        float minX = float.MaxValue, maxX = float.MinValue;
        float minZ = float.MaxValue, maxZ = float.MinValue;
        foreach (Vector2 corner in corners)
        {
            minX = Mathf.Min(minX, corner.x);
            maxX = Mathf.Max(maxX, corner.x);
            minZ = Mathf.Min(minZ, corner.y); // Vector2.y maps to World Z
            maxZ = Mathf.Max(maxZ, corner.y);
        }

        // Apply margin
        minX += zoneMargin;
        maxX -= zoneMargin;
        minZ += zoneMargin;
        maxZ -= zoneMargin;

        if (minX >= maxX || minZ >= maxZ)
        {
            Debug.LogWarning($"Zone {(char)('A' + zoneIndex)} too small for margin {zoneMargin}. Cannot generate ZigZag.");
            return;
        }

        // Generate waypoints
        bool goingRight = true;
        for (float z = minZ; z <= maxZ; z += zigzagSpacing)
        {
            float startX = goingRight ? minX : maxX;
            float endX = goingRight ? maxX : minX;

            currentWaypoints.Add(new Vector3(startX, 0, z));
            currentWaypoints.Add(new Vector3(endX, 0, z));

            goingRight = !goingRight;

             // Ensure the last Z level is included if spacing doesn't align perfectly
             if (z + zigzagSpacing > maxZ && z < maxZ)
             {
                 z = maxZ - zigzagSpacing; // Will cause the next loop iteration to use maxZ
             }
        }
         Debug.Log($"Generated {currentWaypoints.Count} ZigZag waypoints for Zone {(char)('A' + zoneIndex)}.");
    }

    /// <summary>
    /// Clamps a target point to be within the approximate bounding box of the zone.
    /// A more accurate method would check against polygon boundaries.
    /// </summary>
    private Vector3 ConstrainToZone(Vector3 targetPoint, int zoneIndex)
    {
        Vector2[] corners = GetZoneCorners(zoneIndex);
        if (corners == null || corners.Length < 3) return targetPoint; // Cannot constrain

        // Find min/max X and Z (without margin for constraining)
        float minX = float.MaxValue, maxX = float.MinValue;
        float minZ = float.MaxValue, maxZ = float.MinValue;
        foreach (Vector2 corner in corners)
        {
            minX = Mathf.Min(minX, corner.x);
            maxX = Mathf.Max(maxX, corner.x);
            minZ = Mathf.Min(minZ, corner.y);
            maxZ = Mathf.Max(maxZ, corner.y);
        }

        // Clamp the target point's X and Z
        float clampedX = Mathf.Clamp(targetPoint.x, minX, maxX);
        float clampedZ = Mathf.Clamp(targetPoint.z, minZ, maxZ);

        // Optional: Check if clamping occurred and log it
        // if (clampedX != targetPoint.x || clampedZ != targetPoint.z)
        // {
        //     Debug.Log($"Constrained target {targetPoint} to ({clampedX}, {targetPoint.y}, {clampedZ}) within Zone {(char)('A' + zoneIndex)} bounds.");
        // }

        return new Vector3(clampedX, targetPoint.y, clampedZ);
    }


    // --- Zone Completion Check ---

    /// <summary>
    /// Checks if the objective (defusing the active bomb) for the specified zone is complete.
    /// </summary>
    private bool CheckIfZoneComplete(int zoneIndex)
    {
        if (bombGenerator == null)
        {
            Debug.LogError("CheckIfZoneComplete: BombGenerator reference missing!");
            return false;
        }

        // Determine which list of bombs corresponds to the zone
        List<GameObject> zoneBombs = null;
        int locationArea = zoneIndex + 1; // Assuming locationArea 1, 2, 3 corresponds to zone index 0, 1, 2

        switch (locationArea)
        {
            case 1: zoneBombs = bombGenerator.bombA; break;
            case 2: zoneBombs = bombGenerator.bombB; break;
            case 3: zoneBombs = bombGenerator.bombC; break;
            default:
                Debug.LogError($"CheckIfZoneComplete: Invalid zone index {zoneIndex}");
                return false;
        }

        if (zoneBombs == null || zoneBombs.Count == 0)
        {
            Debug.LogWarning($"CheckIfZoneComplete: No bombs configured for Zone {(char)('A' + zoneIndex)}. Assuming complete.");
            return true; // Or false, depending on desired behavior
        }

        // Find the *active* bomb within that list and check its status
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
                        Debug.Log($"CheckIfZoneComplete: Active bomb '{bombObj.name}' for Zone {(char)('A' + zoneIndex)} IS defused. Zone complete.");
                        return true;
                    }
                    else
                    {
                        // Found the active bomb, but it's not defused yet
                        return false;
                    }
                }
                else
                {
                    Debug.LogError($"CheckIfZoneComplete: Active bomb '{bombObj.name}' is missing Bomb script!");
                    return false; // Cannot determine status
                }
            }
        }

        // If we looped through and didn't find an *active* bomb (shouldn't happen if BombGenerator worked)
        Debug.LogWarning($"CheckIfZoneComplete: Could not find an *active* bomb in the list for Zone {(char)('A' + zoneIndex)}. Assuming incomplete.");
        return false;
    }

    // --- Gizmos ---
    void OnDrawGizmos()
    {
        // Draw Zones
        DrawZoneGizmo(zoneACorners, Color.red);
        DrawZoneGizmo(zoneBCorners, Color.green);
        DrawZoneGizmo(zoneCCorners, Color.blue);

        // Draw Waypoints
        if (currentWaypoints != null && currentWaypoints.Count > 0)
        {
            Gizmos.color = Color.yellow;
            for (int i = 0; i < currentWaypoints.Count - 1; i++)
            {
                Gizmos.DrawLine(currentWaypoints[i], currentWaypoints[i + 1]);
                Gizmos.DrawSphere(currentWaypoints[i], 0.2f);
            }
            Gizmos.DrawSphere(currentWaypoints[currentWaypoints.Count - 1], 0.2f);

            // Highlight next waypoint target
            if (isExploring && currentWaypointTargetIndex < currentWaypoints.Count)
            {
                Gizmos.color = Color.cyan;
                Gizmos.DrawWireSphere(currentWaypoints[currentWaypointTargetIndex], 0.5f);
            }
        }

        // Draw Estimated Position
        Gizmos.color = Color.white;
        Gizmos.DrawWireSphere(estimatedPosition, 0.3f);
        #if UNITY_EDITOR
        UnityEditor.Handles.Label(estimatedPosition + Vector3.up * 0.5f, "Est. Pos");
        #endif

        // Draw Beacons
        if (beacons != null)
        {
            Gizmos.color = Color.cyan;
            foreach (Transform beacon in beacons)
            {
                if (beacon != null) Gizmos.DrawSphere(beacon.position, 0.4f);
            }
        }
    }

    private void DrawZoneGizmo(Vector2[] corners, Color color)
    {
        if (corners == null || corners.Length < 3) return;
        Gizmos.color = color;
        for (int i = 0; i < corners.Length; i++)
        {
            Vector3 p1 = new Vector3(corners[i].x, 0, corners[i].y);
            Vector3 p2 = new Vector3(corners[(i + 1) % corners.Length].x, 0, corners[(i + 1) % corners.Length].y);
            Gizmos.DrawLine(p1, p2);
        }
    }
}