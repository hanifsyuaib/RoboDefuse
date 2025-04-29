using UnityEngine;

public class RobotCarController : MonoBehaviour
{
    [Header("Movement Settings")]
    public float moveSpeed = 5f;
    public float turnSpeed = 120f;

    private Rigidbody rb;

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rb.centerOfMass = new Vector3(0, -0.5f, 0); // Lower center of mass
    }

    private void Update()
    {
        if (Vector3.Dot(transform.up, Vector3.down) > 0.7f)
        {
            FlipBack();
        }

        HandleManualInput();
    }

    private void HandleManualInput()
    {
        float moveInput = Input.GetAxis("Vertical");
        float turnInput = Input.GetAxis("Horizontal");

        if (Mathf.Abs(moveInput) > 0.01f || Mathf.Abs(turnInput) > 0.01f)
        {
            Vector3 move = transform.forward * moveInput * moveSpeed * Time.deltaTime;
            rb.MovePosition(rb.position + move);

            float turn = turnInput * turnSpeed * Time.deltaTime;
            Quaternion turnRotation = Quaternion.Euler(0f, turn, 0f);
            rb.MoveRotation(rb.rotation * turnRotation);
        }
    }

    private void FlipBack()
    {
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        transform.rotation = Quaternion.Euler(0f, transform.eulerAngles.y, 0f);
    }

    // --- Automatic Controls ---

    public void MoveForward()
    {
        Move(transform.forward);
    }

    public void MoveBackward(float speedMultiplier = 1f)
    {
        Move(-transform.forward * speedMultiplier);
    }

    public void Turn(float direction)
    {
        Rotate(direction);
    }

    public void Move(Vector3 direction)
    {
        Vector3 move = direction.normalized * moveSpeed * Time.deltaTime;
        rb.MovePosition(rb.position + move);
    }

    public void Rotate(float direction)
    {
        Quaternion turnRotation = Quaternion.Euler(0f, direction * turnSpeed * Time.deltaTime, 0f);
        rb.MoveRotation(rb.rotation * turnRotation);
    }

    public void StopMoving()
    {
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
    }
}
