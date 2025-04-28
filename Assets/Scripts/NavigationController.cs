using UnityEngine;

public class NavigationController : MonoBehaviour
{
    public Transform[] waypoints;
    private int currentWaypoint = 0;

    void Update()
    {
        if (currentWaypoint < waypoints.Length)
        {
            MoveToWaypoint(waypoints[currentWaypoint]);
        }
    }

    void MoveToWaypoint(Transform target)
    {
        float step = 5f * Time.deltaTime; // Move speed
        transform.position = Vector3.MoveTowards(transform.position, target.position, step);

        if (Vector3.Distance(transform.position, target.position) < 0.1f)
        {
            currentWaypoint++;
        }
    }
}
