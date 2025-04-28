using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class RobotController : MonoBehaviour
{
    [Header("Robot Specs")]
    public float wheelBase = 0.5f; // meters (distance between left and right wheels)
    public float maxLinearSpeed = 1.0f; // m/s
    public float maxAngularSpeed = 90.0f; // degrees/s

    [Header("Movement Control")]
    [SerializeField] private float targetLinearVelocity = 0f; // m/s
    [SerializeField] private float targetAngularVelocity = 0f; // degrees/s
    public float motorTorque = 100f; // Max torque applied to wheels (needs tuning)
    public float brakeTorque = 200f; // Torque applied when stopping (needs tuning)

    [Header("Wheel Colliders")]
    public WheelCollider wheelFL;
    public WheelCollider wheelFR;
    public WheelCollider wheelRL;
    public WheelCollider wheelRR;

    public float speedControlGain = 5f;  // Control gain for speed adjustments

    private Rigidbody rb;
    private bool isBraking = false;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        // Ensure Rigidbody settings are appropriate (e.g., constraints)
        rb.centerOfMass = transform.InverseTransformPoint(rb.worldCenterOfMass) + new Vector3(0, -0.2f, 0);
    }

    void Update()
    {
        HandleKeyboardInput();
        if (Input.GetKeyDown(KeyCode.Space))
        {
            isBraking = true;
        }
        else if (Input.GetKeyUp(KeyCode.Space))
        {
            isBraking = false;
        }
    }

    void FixedUpdate()
    {
        ApplyDifferentialDriveMovementWithWheels();
    }

    // Public method for NavigationController to set desired movement
    public void SetTargetMovement(float linear, float angular)
    {
        targetLinearVelocity = Mathf.Clamp(linear, -maxLinearSpeed, maxLinearSpeed);
        targetAngularVelocity = Mathf.Clamp(angular, -maxAngularSpeed, maxAngularSpeed);
    }

    private void HandleKeyboardInput()
    {
        if (isBraking)
        {
            SetTargetMovement(0f, 0f);
            return; // Exit early
        }

        float targetLinear = 0f;
        float targetAngular = 0f;

        // Forward/Backward (W/S)
        if (Input.GetKey(KeyCode.W))
        {
            targetLinear = maxLinearSpeed;
        }
        else if (Input.GetKey(KeyCode.S))
        {
            targetLinear = -maxLinearSpeed; // Move backward
        }

        // Left/Right Rotation (A/D)
        if (Input.GetKey(KeyCode.A))
        {
            targetAngular = -maxAngularSpeed; // Rotate left
        }
        else if (Input.GetKey(KeyCode.D))
        {
            targetAngular = maxAngularSpeed; // Rotate right
        }

        // Apply the calculated movement targets
        SetTargetMovement(targetLinear, targetAngular);
    }

    private void ApplyDifferentialDriveMovementWithWheels()
    {
        // --- Manual Braking Override ---
        if (isBraking)
        {
            // Apply full brakes and zero motor torque
            wheelFL.motorTorque = 0f;
            wheelRL.motorTorque = 0f;
            wheelFR.motorTorque = 0f;
            wheelRR.motorTorque = 0f;

            wheelFL.brakeTorque = brakeTorque;
            wheelRL.brakeTorque = brakeTorque;
            wheelFR.brakeTorque = brakeTorque;
            wheelRR.brakeTorque = brakeTorque;
            return; // Don't process regular movement/braking
        }

        float angularVelRad = targetAngularVelocity * Mathf.Deg2Rad;

        // Calculate target surface speeds for left and right sides
        float targetVLeft = targetLinearVelocity - (angularVelRad * wheelBase / 2.0f);
        float targetVRight = targetLinearVelocity + (angularVelRad * wheelBase / 2.0f);

        // --- Convert target speeds to target RPM ---
        float wheelRadius = wheelFL.radius; // Assuming all wheels have the same radius
        float targetRpmLeft = (targetVLeft * 60.0f) / (2.0f * Mathf.PI * wheelRadius);
        float targetRpmRight = (targetVRight * 60.0f) / (2.0f * Mathf.PI * wheelRadius);

        // --- Proportional Speed Control ---
        // Calculate RPM error for each side (average of front/rear)
        float currentRpmLeft = (wheelFL.rpm + wheelRL.rpm) / 2.0f;
        float currentRpmRight = (wheelFR.rpm + wheelRR.rpm) / 2.0f;

        float errorLeft = targetRpmLeft - currentRpmLeft;
        float errorRight = targetRpmRight - currentRpmRight;

        // Apply torque proportional to the error, clamped by motorTorque
        float leftMotorTorque = Mathf.Clamp(errorLeft * speedControlGain, -motorTorque, motorTorque);
        float rightMotorTorque = Mathf.Clamp(errorRight * speedControlGain, -motorTorque, motorTorque);

        // Apply torque to wheels
        wheelFL.motorTorque = leftMotorTorque;
        wheelRL.motorTorque = leftMotorTorque; // Apply same torque to both left wheels
        wheelFR.motorTorque = rightMotorTorque;
        wheelRR.motorTorque = rightMotorTorque; // Apply same torque to both right wheels

        // Apply brakes if target speed is near zero AND current speed is low
        float currentLinearSpeed = rb.velocity.magnitude; // Approximation of linear speed

        bool shouldBrakeLeft = Mathf.Abs(targetVLeft) < 0.01f && Mathf.Abs(currentRpmLeft) < 10f; // Brake if stopping and slow
        bool shouldBrakeRight = Mathf.Abs(targetVRight) < 0.01f && Mathf.Abs(currentRpmRight) < 10f; // Brake if stopping and slow

        wheelFL.brakeTorque = shouldBrakeLeft ? brakeTorque : 0f;
        wheelRL.brakeTorque = shouldBrakeLeft ? brakeTorque : 0f;
        wheelFR.brakeTorque = shouldBrakeRight ? brakeTorque : 0f;
        wheelRR.brakeTorque = shouldBrakeRight ? brakeTorque : 0f;

        // --- Steering (not used in differential drive but can be used for other purposes) ---
        wheelFL.steerAngle = 0;
        wheelFR.steerAngle = 0;
        wheelRL.steerAngle = 0;
        wheelRR.steerAngle = 0;
    }
}