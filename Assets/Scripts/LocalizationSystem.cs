using UnityEngine;

[RequireComponent(typeof(RobotController))] // Needs RobotController to get wheel data
public class LocalizationSystem : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private RobotController robotController;

    [Header("Estimated Pose (Read Only)")]
    [SerializeField] private Vector3 estimatedPosition;
    [SerializeField] private Quaternion estimatedRotation;
    [SerializeField] private float estimatedYawRad; // Store yaw in radians for easier math

    // Public properties for other scripts to read the estimated pose
    public Vector3 EstimatedPosition => estimatedPosition;
    public Quaternion EstimatedRotation => estimatedRotation;
    public float EstimatedYawRad => estimatedYawRad; // Provide yaw in radians
    public float EstimatedYawDeg => estimatedYawRad * Mathf.Rad2Deg; // Provide yaw in degrees

    // Keep track of previous wheel rotations/RPM if needed for delta calculations
    // private float prevLeftRpm = 0f;
    // private float prevRightRpm = 0f;

    void Awake()
    {
        // Get the RobotController component attached to the same GameObject
        if (robotController == null)
        {
            robotController = GetComponent<RobotController>();
        }
    }

    void Start()
    {
        // Initialize estimated pose based on the actual starting pose
        // This is the ONLY time we read the transform directly.
        estimatedPosition = transform.position;
        estimatedRotation = transform.rotation;
        estimatedYawRad = transform.eulerAngles.y * Mathf.Deg2Rad; // Convert initial Y rotation to radians

        Debug.Log($"Localization Initialized: Pos={estimatedPosition}, Rot={estimatedRotation.eulerAngles}");
    }

    void FixedUpdate() // Use FixedUpdate for physics-related calculations
    {
        UpdateOdometry();
    }

    void UpdateOdometry()
    {
        if (robotController == null) return;

        // 1. Get Wheel Speeds (RPM) from RobotController
        robotController.GetWheelSpeedsRpm(out float currentLeftRpm, out float currentRightRpm);

        // 2. Convert RPM to Wheel Surface Velocities (m/s)
        // Velocity = RPM * (2 * PI * Radius) / 60
        // Assuming RobotController has access to wheel radius or provides it
        // For now, let's assume RobotController can give us the radius
        // If not, we might need to add a radius field here or get it from WheelColliders directly
        float wheelRadius = robotController.wheelFL.radius; // Get radius from one of the wheels
        float vLeft = currentLeftRpm * (2.0f * Mathf.PI * wheelRadius) / 60.0f;
        float vRight = currentRightRpm * (2.0f * Mathf.PI * wheelRadius) / 60.0f;

        // 3. Calculate Robot's Linear and Angular Velocity from Wheel Velocities
        // Linear Velocity (V) = (vRight + vLeft) / 2
        // Angular Velocity (Omega) = (vRight - vLeft) / wheelBase
        float linearVelocity = (vRight + vLeft) / 2.0f;
        float angularVelocityRad = (vRight - vLeft) / robotController.wheelBase; // Result is in radians/sec

        // 4. Integrate Velocities over Time to Update Pose Estimate
        float dt = Time.fixedDeltaTime;

        // Calculate change in position based on current estimated orientation
        // dx = V * cos(yaw) * dt
        // dz = V * sin(yaw) * dt
        float deltaX = linearVelocity * Mathf.Cos(estimatedYawRad) * dt;
        float deltaZ = linearVelocity * Mathf.Sin(estimatedYawRad) * dt;

        // Update estimated position (assuming movement primarily on XZ plane)
        estimatedPosition += new Vector3(deltaX, 0, deltaZ);

        // Calculate change in orientation (yaw)
        // dYaw = Omega * dt
        float deltaYaw = angularVelocityRad * dt;

        // Update estimated yaw and rotation
        estimatedYawRad += deltaYaw;
        // Normalize yaw angle (optional but good practice)
        estimatedYawRad = NormalizeAngleRad(estimatedYawRad);
        estimatedRotation = Quaternion.Euler(0, estimatedYawRad * Mathf.Rad2Deg, 0);

        // Optional: Store current RPMs for next frame if using delta RPM method
        // prevLeftRpm = currentLeftRpm;
        // prevRightRpm = currentRightRpm;

        // Optional: Add debug visualization
        // Debug.DrawRay(estimatedPosition, estimatedRotation * Vector3.forward, Color.blue);
    }

    // Helper function to keep angle within -PI to PI range
    private float NormalizeAngleRad(float angleRad)
    {
        while (angleRad > Mathf.PI) angleRad -= 2.0f * Mathf.PI;
        while (angleRad <= -Mathf.PI) angleRad += 2.0f * Mathf.PI;
        return angleRad;
    }

    // TODO: Incorporate sensor data for correction (e.g., landmark detection, SLAM - more advanced)
    // TODO: Add noise simulation for more realistic odometry drift
}