using UnityEngine;

public class TrilaterationLocalization : MonoBehaviour
{
    [Header("Landmarks")]
    public Transform landmarkA;
    public Transform landmarkB;
    public Transform landmarkC;

    [Header("Measured Distances (simulate)")]
    public float measuredDistanceA;
    public float measuredDistanceB;
    public float measuredDistanceC;

    [Header("Localization Result")]
    public Vector3 estimatedPosition;

    void Update()
    {
        PerformTrilateration();
    }

    private void PerformTrilateration()
    {
        Vector3 p1 = landmarkA.position;
        Vector3 p2 = landmarkB.position;
        Vector3 p3 = landmarkC.position;

        // Project into 2D (ignore Y for simplicity)
        Vector2 P1 = new Vector2(p1.x, p1.z);
        Vector2 P2 = new Vector2(p2.x, p2.z);
        Vector2 P3 = new Vector2(p3.x, p3.z);

        float dA = measuredDistanceA;
        float dB = measuredDistanceB;
        float dC = measuredDistanceC;

        // Based on trilateration formula
        float W = Mathf.Pow(P1.x, 2) + Mathf.Pow(P1.y, 2) - Mathf.Pow(dA, 2);
        float X = Mathf.Pow(P2.x, 2) + Mathf.Pow(P2.y, 2) - Mathf.Pow(dB, 2);
        float Y = Mathf.Pow(P3.x, 2) + Mathf.Pow(P3.y, 2) - Mathf.Pow(dC, 2);

        float x = (W * (P3.y - P2.y) + X * (P1.y - P3.y) + Y * (P2.y - P1.y)) / (2 * (P1.x * (P3.y - P2.y) + P2.x * (P1.y - P3.y) + P3.x * (P2.y - P1.y)));
        float y = (W * (P3.x - P2.x) + X * (P1.x - P3.x) + Y * (P2.x - P1.x)) / (2 * (P1.y * (P3.x - P2.x) + P2.y * (P1.x - P3.x) + P3.y * (P2.x - P1.x)));

        estimatedPosition = new Vector3(x, transform.position.y, y);
    }
}
