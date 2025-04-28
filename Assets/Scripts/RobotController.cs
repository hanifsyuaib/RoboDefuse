using UnityEngine;

[RequireComponent(typeof(Rigidbody))] // Or CharacterController if you prefer
public class RobotController : MonoBehaviour
{
    [Header("Robot Specs")]
    // public float wheelRadius = 0.1f; // meters
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

    public float speedControlGain = 5f;

    private Rigidbody rb;
    private bool isBraking = false;
    // Add variables for wheel joints/colliders if controlling physics more directly

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        // Ensure Rigidbody settings are appropriate (e.g., constraints)
        rb.centerOfMass = transform.InverseTransformPoint(rb.worldCenterOfMass) + new Vector3(0, -0.2f, 0);
    }

    // Add this Update method for keyboard input
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

    void FixedUpdate() // Use FixedUpdate for physics-based movement
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

        // If manually braking, ignore movement input
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
            targetAngular = maxAngularSpeed; // Rotate left
        }
        else if (Input.GetKey(KeyCode.D))
        {
            targetAngular = -maxAngularSpeed; // Rotate right
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
        // RPM = (Velocity * 60) / (2 * PI * Radius)
        // We need the wheel radius from the collider
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
        // Use speedControlGain to tune how aggressively it tries to reach the target
        float leftMotorTorque = Mathf.Clamp(errorLeft * speedControlGain, -motorTorque, motorTorque);
        float rightMotorTorque = Mathf.Clamp(errorRight * speedControlGain, -motorTorque, motorTorque);


        // Apply torque to wheels
        wheelFL.motorTorque = leftMotorTorque;
        wheelRL.motorTorque = leftMotorTorque; // Apply same torque to both left wheels
        wheelFR.motorTorque = rightMotorTorque;
        wheelRR.motorTorque = rightMotorTorque; // Apply same torque to both right wheels

        // Apply brakes if target speed is near zero AND current speed is low
        float currentLinearSpeed = rb.velocity.magnitude; // Approximation of linear speed
        // float currentAngularSpeed = rb.angularVelocity.y * Mathf.Rad2Deg; // Approximation of angular speed

        // Adjust braking condition slightly: Brake if target is zero OR if trying to move opposite to current high speed
        bool shouldBrakeLeft = Mathf.Abs(targetVLeft) < 0.01f && Mathf.Abs(currentRpmLeft) < 10f; // Brake if stopping and slow
        bool shouldBrakeRight = Mathf.Abs(targetVRight) < 0.01f && Mathf.Abs(currentRpmRight) < 10f; // Brake if stopping and slow

        // OR consider adding braking if error sign opposes current RPM sign (counter-torque braking)
        // if (Mathf.Sign(errorLeft) != Mathf.Sign(currentRpmLeft) && Mathf.Abs(currentRpmLeft) > 10f) shouldBrakeLeft = true;
        // if (Mathf.Sign(errorRight) != Mathf.Sign(currentRpmRight) && Mathf.Abs(currentRpmRight) > 10f) shouldBrakeRight = true;


        wheelFL.brakeTorque = shouldBrakeLeft ? brakeTorque : 0f;
        wheelRL.brakeTorque = shouldBrakeLeft ? brakeTorque : 0f;
        wheelFR.brakeTorque = shouldBrakeRight ? brakeTorque : 0f;
        wheelRR.brakeTorque = shouldBrakeRight ? brakeTorque : 0f;


        // --- Steering ---
        wheelFL.steerAngle = 0;
        wheelFR.steerAngle = 0;
        wheelRL.steerAngle = 0;
        wheelRR.steerAngle = 0;
    }

    // Optional method to update visual wheel meshes
    /*
    void UpdateVisualWheels()
    {
        UpdateSingleWheel(wheelFL, visualWheelFL); // Assuming visualWheelFL is a Transform reference
        UpdateSingleWheel(wheelFR, visualWheelFR);
        UpdateSingleWheel(wheelRL, visualWheelRL);
        UpdateSingleWheel(wheelRR, visualWheelRR);
    }

    void UpdateSingleWheel(WheelCollider collider, Transform wheelTransform)
    {
        if (wheelTransform == null) return;

        Vector3 pos;
        Quaternion rot;
        collider.GetWorldPose(out pos, out rot);
        wheelTransform.position = pos;
        wheelTransform.rotation = rot;
    }
    */


    // GetWheelSpeeds might need adjustment if Localization needs RPM instead of surface speed
    public void GetWheelSpeedsRpm(out float leftRpm, out float rightRpm)
    {
        // Average RPM of left/right pairs
        leftRpm = (wheelFL.rpm + wheelRL.rpm) / 2.0f;
        rightRpm = (wheelFR.rpm + wheelRR.rpm) / 2.0f;
    }
}