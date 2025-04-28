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

    private Rigidbody rb;
    private bool isBraking = false;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.centerOfMass = new Vector3(0, -0.5f, 0); // Adjusted for proper balance
    }

    void Update()
    {
        HandleKeyboardInput();
    }

    void FixedUpdate()
    {
        ApplyMotorAndSteering();
    }

    void HandleKeyboardInput()
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

    // Method to set target linear and angular velocities
    public void SetTargetMovement(float linear, float angular)
    {
        targetLinearVelocity = Mathf.Clamp(linear, -maxLinearSpeed, maxLinearSpeed);
        targetAngularVelocity = Mathf.Clamp(angular, -maxAngularSpeed, maxAngularSpeed);
    }

    // Apply motor and steering force to the wheels
    void ApplyMotorAndSteering()
    {
        if (isBraking)
        {
            // Apply braking torque if braking is activated
            ApplyBrakes();
            return;
        }

        float angularVelRad = targetAngularVelocity * Mathf.Deg2Rad;

        // Calculate target wheel speeds (left and right side)
        float targetVLeft = targetLinearVelocity - (angularVelRad * wheelBase / 2.0f);
        float targetVRight = targetLinearVelocity + (angularVelRad * wheelBase / 2.0f);

        // Apply torque to wheels based on target speeds
        wheelFL.motorTorque = Mathf.Clamp(targetVLeft, -motorTorque, motorTorque);
        wheelFR.motorTorque = Mathf.Clamp(targetVRight, -motorTorque, motorTorque);
        wheelRL.motorTorque = Mathf.Clamp(targetVLeft, -motorTorque, motorTorque);
        wheelRR.motorTorque = Mathf.Clamp(targetVRight, -motorTorque, motorTorque);

        // Steering - simple turning, assuming front wheels steer
        wheelFL.steerAngle = targetAngularVelocity;
        wheelFR.steerAngle = targetAngularVelocity;
    }

    void ApplyBrakes()
    {
        wheelFL.brakeTorque = brakeTorque;
        wheelFR.brakeTorque = brakeTorque;
        wheelRL.brakeTorque = brakeTorque;
        wheelRR.brakeTorque = brakeTorque;
    }
}
