using UnityEngine;

public class WheelDirectionTest : MonoBehaviour
{
    public WheelCollider wheelFL;
    public WheelCollider wheelFR;
    public WheelCollider wheelRL;
    public WheelCollider wheelRR;
    
    public float motorTorque = 100f;
    
    private bool isTestActive = false;
    
    void Update()
    {
        // Press 1 to test all wheels forward
        if (Input.GetKeyDown(KeyCode.Alpha1))
        {
            isTestActive = true;
            Debug.Log("TEST: All wheels forward");
            wheelFL.motorTorque = motorTorque;
            wheelFR.motorTorque = motorTorque;
            wheelRL.motorTorque = motorTorque;
            wheelRR.motorTorque = motorTorque;
        }
        
        // Press 2 to test right turn (left wheels forward, right wheels backward)
        if (Input.GetKeyDown(KeyCode.Alpha2))
        {
            isTestActive = true;
            Debug.Log("TEST: Right turn (left forward, right backward)");
            wheelFL.motorTorque = motorTorque;
            wheelFR.motorTorque = -motorTorque;
            wheelRL.motorTorque = motorTorque;
            wheelRR.motorTorque = -motorTorque;
        }
        
        // Press 3 to test left turn (left wheels backward, right wheels forward)
        if (Input.GetKeyDown(KeyCode.Alpha3))
        {
            isTestActive = true;
            Debug.Log("TEST: Left turn (left backward, right forward)");
            wheelFL.motorTorque = -motorTorque;
            wheelFR.motorTorque = motorTorque;
            wheelRL.motorTorque = -motorTorque;
            wheelRR.motorTorque = motorTorque;
        }
        
        // Press 0 to stop all tests
        if (Input.GetKeyDown(KeyCode.Alpha0))
        {
            isTestActive = false;
            Debug.Log("TEST: Stopping all wheels");
            wheelFL.motorTorque = 0f;
            wheelFR.motorTorque = 0f;
            wheelRL.motorTorque = 0f;
            wheelRR.motorTorque = 0f;
            
            wheelFL.brakeTorque = 1000f;
            wheelFR.brakeTorque = 1000f;
            wheelRL.brakeTorque = 1000f;
            wheelRR.brakeTorque = 1000f;
        }
        
        // If no test is active, ensure brakes are off
        if (!isTestActive) 
        {
            wheelFL.brakeTorque = 0f;
            wheelFR.brakeTorque = 0f;
            wheelRL.brakeTorque = 0f;
            wheelRR.brakeTorque = 0f;
        }
    }
}