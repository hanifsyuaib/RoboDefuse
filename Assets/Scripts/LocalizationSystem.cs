using UnityEngine;

public class LocalizationSystem : MonoBehaviour
{
    public Transform robotTransform;

    void Update()
    {
        Vector3 position = robotTransform.position;
        Quaternion rotation = robotTransform.rotation;

        // Log or use these values for localization
        Debug.Log("Robot Position: " + position + " Rotation: " + rotation);
    }
}
