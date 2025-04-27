using UnityEngine;

public class TrilaterationVisualizer : MonoBehaviour
{
    [Header("Beacon Settings")]
    [SerializeField] private Transform beacon1;
    [SerializeField] private Transform beacon2;
    [SerializeField] private Transform beacon3;
    [SerializeField] private float updateInterval = 0.5f;
    [SerializeField] private float noiseFactor = 0.05f; // Reduced noise for better accuracy
    
    [Header("Visual Settings")]
    [SerializeField] private Color beacon1Color = Color.red;
    [SerializeField] private Color beacon2Color = Color.green;
    [SerializeField] private Color beacon3Color = Color.blue;
    [SerializeField] private Color estimateColor = Color.yellow;
    
    private float lastUpdateTime;
    private Vector3 estimatedPosition;
    private float[] distances = new float[3];
    
    private void Update()
    {
        if (Time.time - lastUpdateTime >= updateInterval)
        {
            lastUpdateTime = Time.time;
            UpdateTrilateration();
        }
    }
    
    private void UpdateTrilateration()
    {
        if (beacon1 == null || beacon2 == null || beacon3 == null)
            return;
        
        // Calculate distances (in a real application, these would come from sensors)
        distances[0] = Vector3.Distance(transform.position, beacon1.position);
        distances[1] = Vector3.Distance(transform.position, beacon2.position);
        distances[2] = Vector3.Distance(transform.position, beacon3.position);
        
        // Add smaller noise to simulate real-world conditions
        distances[0] += Random.Range(-noiseFactor, noiseFactor);
        distances[1] += Random.Range(-noiseFactor, noiseFactor);
        distances[2] += Random.Range(-noiseFactor, noiseFactor);
        
        // Calculate estimated position
        estimatedPosition = Trilaterate(
            beacon1.position, beacon2.position, beacon3.position,
            distances[0], distances[1], distances[2]);
        
        // Debug output
        float error = Vector3.Distance(transform.position, estimatedPosition);
        string errorMessage = error > 5.0f ? " <color=red>ERROR TOO HIGH!</color>" : "";
        
        Debug.Log($"Trilateration: Actual={transform.position}, Estimated={estimatedPosition}, Error={error:F2}{errorMessage}");
    }
    
    private Vector3 Trilaterate(Vector3 p1, Vector3 p2, Vector3 p3, float r1, float r2, float r3)
    {
        // Extract 2D coordinates (X and Z, since Y is height)
        float x1 = p1.x; float z1 = p1.z;
        float x2 = p2.x; float z2 = p2.z;
        float x3 = p3.x; float z3 = p3.z;
        
        // Calculate intermediate values
        float A = 2 * (x2 - x1);
        float B = 2 * (z2 - z1);
        float C = r1 * r1 - r2 * r2 - x1 * x1 + x2 * x2 - z1 * z1 + z2 * z2;
        float D = 2 * (x3 - x2);
        float E = 2 * (z3 - z2);
        float F = r2 * r2 - r3 * r3 - x2 * x2 + x3 * x3 - z2 * z2 + z3 * z3;
        
        // Fixed formula for calculating position
        float denominator = (A * E - B * D);
        
        if (Mathf.Abs(denominator) < 0.001f)
        {
            // Singular case, beacons might be collinear
            Debug.LogWarning("Trilateration: Singular case detected. Beacons might be collinear.");
            return transform.position; // Fall back to current position
        }
        
        float x = (C * E - F * B) / denominator;
        float z = (A * F - C * D) / denominator;
        
        return new Vector3(x, transform.position.y, z);
    }
    
    private void OnDrawGizmos()
    {
        if (beacon1 == null || beacon2 == null || beacon3 == null)
            return;
        
        // Draw beacons
        Gizmos.color = beacon1Color;
        Gizmos.DrawSphere(beacon1.position, 0.3f);
        Gizmos.color = beacon2Color;
        Gizmos.DrawSphere(beacon2.position, 0.3f);
        Gizmos.color = beacon3Color;
        Gizmos.DrawSphere(beacon3.position, 0.3f);
        
        // Draw distance circles
        DrawCircle(beacon1.position, distances[0], beacon1Color);
        DrawCircle(beacon2.position, distances[1], beacon2Color);
        DrawCircle(beacon3.position, distances[2], beacon3Color);
        
        // Draw estimated position
        Gizmos.color = estimateColor;
        Gizmos.DrawSphere(estimatedPosition, 0.3f);
        
        // Draw line to actual position
        Gizmos.DrawLine(estimatedPosition, transform.position);
    }
    
    private void DrawCircle(Vector3 center, float radius, Color color)
    {
        Gizmos.color = new Color(color.r, color.g, color.b, 0.2f);
        
        int segments = 36;
        Vector3 prevPoint = center + new Vector3(radius, 0, 0);
        
        for (int i = 1; i <= segments; i++)
        {
            float angle = i * 2 * Mathf.PI / segments;
            Vector3 point = center + new Vector3(radius * Mathf.Cos(angle), 0, radius * Mathf.Sin(angle));
            Gizmos.DrawLine(prevPoint, point);
            prevPoint = point;
        }
    }
}