using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class AStarNavigation : MonoBehaviour
{
    [Header("Pathfinding Settings")]
    [SerializeField] private LayerMask unwalkableMask; // IMPORTANT: Ensure this MASK does NOT include your ground/floor layer! Only obstacles.
    [SerializeField] private Vector2 gridWorldSize = new Vector2(50, 50); // IMPORTANT: Increase this (e.g., 200x200) if your target points are far from the grid center (this object's position). X=WorldX, Y=WorldZ.
    [SerializeField] private float nodeRadius = 0.5f; // Radius for collision checks. Reduce slightly if nodes near walls are incorrectly marked unwalkable.
    [SerializeField] private bool displayGridGizmos = false; // Set to TRUE to visualize the grid in the Scene view (pause game).

    private Node[,] grid;
    private float nodeDiameter;
    private int gridSizeX, gridSizeY;

    void Awake()
    {
        nodeDiameter = nodeRadius * 2;
        gridSizeX = Mathf.RoundToInt(gridWorldSize.x / nodeDiameter);
        gridSizeY = Mathf.RoundToInt(gridWorldSize.y / nodeDiameter);
        CreateGrid();
    }

    void CreateGrid()
    {
        grid = new Node[gridSizeX, gridSizeY];
        // Grid is centered around this GameObject's position. Ensure this object is placed appropriately (e.g., at world 0,0,0 or center of playable area).
        Vector3 worldBottomLeft = transform.position - Vector3.right * gridWorldSize.x / 2 - Vector3.forward * gridWorldSize.y / 2;

        for (int x = 0; x < gridSizeX; x++)
        {
            for (int y = 0; y < gridSizeY; y++)
            {
                Vector3 worldPoint = worldBottomLeft + Vector3.right * (x * nodeDiameter + nodeRadius) + Vector3.forward * (y * nodeDiameter + nodeRadius);
                // CheckSphere determines walkability based on unwalkableMask and nodeRadius.
                bool walkable = !(Physics.CheckSphere(worldPoint, nodeRadius, unwalkableMask));
                grid[x, y] = new Node(walkable, worldPoint, x, y);
            }
        }
        Debug.Log($"A* Grid Created: Size {gridSizeX}x{gridSizeY}, World Size {gridWorldSize}, Centered at {transform.position}");
    }

    public List<Node> GetNeighbors(Node node)
    {
        List<Node> neighbors = new List<Node>();

        for (int x = -1; x <= 1; x++)
        {
            for (int y = -1; y <= 1; y++)
            {
                if (x == 0 && y == 0)
                    continue;

                int checkX = node.gridX + x;
                int checkY = node.gridY + y;

                if (checkX >= 0 && checkX < gridSizeX && checkY >= 0 && checkY < gridSizeY)
                {
                    neighbors.Add(grid[checkX, checkY]);
                }
            }
        }

        return neighbors;
    }

    public Node NodeFromWorldPoint(Vector3 worldPosition)
    {
        // Adjusted calculation to account for the AStarNavigation object's position
        float percentX = (worldPosition.x - transform.position.x + gridWorldSize.x / 2) / gridWorldSize.x;
        float percentY = (worldPosition.z - transform.position.z + gridWorldSize.y / 2) / gridWorldSize.y; // gridWorldSize.y maps to World Z

        percentX = Mathf.Clamp01(percentX);
        percentY = Mathf.Clamp01(percentY);

        int x = Mathf.RoundToInt((gridSizeX - 1) * percentX);
        int y = Mathf.RoundToInt((gridSizeY - 1) * percentY);

        // Safety check for grid bounds
        if (x < 0 || x >= gridSizeX || y < 0 || y >= gridSizeY) {
            // This error means the worldPosition is outside the defined gridWorldSize relative to the grid center. Increase gridWorldSize or check grid center.
            Debug.LogError($"Calculated grid indices ({x},{y}) are out of bounds for world position {worldPosition} relative to grid center {transform.position}. Grid Size: {gridWorldSize}");
            return null;
        }

        if (grid == null) {
             Debug.LogError("Grid is not initialized!");
             return null;
        }

        return grid[x, y];
    }

    public List<Vector3> FindPath(Vector3 startPos, Vector3 targetPos)
    {
        Debug.Log($"A* FindPath requested from {startPos} to {targetPos}"); // Log request
        Node startNode = NodeFromWorldPoint(startPos);
        Node targetNode = NodeFromWorldPoint(targetPos);

        // --- Add these logs ---
        if (startNode == null) {
            Debug.LogError($"A* Error: Could not map start position {startPos} to a grid node. Is it outside the grid bounds?");
            return null;
        }
        if (targetNode == null) {
            Debug.LogError($"A* Error: Could not map target position {targetPos} to a grid node. Is it outside the grid bounds?");
            return null;
        }
        // Log walkability and world position of the nodes found
        Debug.Log($"A* Start Node ({startNode.gridX},{startNode.gridY}) Walkable: {startNode.walkable} at World Pos: {startNode.worldPosition}");
        Debug.Log($"A* Target Node ({targetNode.gridX},{targetNode.gridY}) Walkable: {targetNode.walkable} at World Pos: {targetNode.worldPosition}");

        if (!startNode.walkable) {
            Debug.LogError("A* Error: Start node is not walkable! Check unwalkableMask, nodeRadius, or if the start position is inside an obstacle.");
        }
        if (!targetNode.walkable) {
            Debug.LogError("A* Error: Target node is not walkable! Check unwalkableMask, nodeRadius, or if the target position is inside an obstacle.");
        }
        // --- End of added logs ---


        // Ensure nodes are walkable before starting
        if (!startNode.walkable || !targetNode.walkable) {
             Debug.LogWarning("A* Warning: Start or Target node is not walkable. Cannot find path.");
             return null; // Exit early if start/target unwalkable
        }

        List<Node> openSet = new List<Node>();
        HashSet<Node> closedSet = new HashSet<Node>();
        openSet.Add(startNode);

        // Reset node costs from previous runs if necessary
        // (Consider adding a grid reset function if pathfinding is called frequently)

        while (openSet.Count > 0)
        {
            Node currentNode = openSet[0];
            for (int i = 1; i < openSet.Count; i++)
            {
                if (openSet[i].fCost < currentNode.fCost || (openSet[i].fCost == currentNode.fCost && openSet[i].hCost < currentNode.hCost))
                {
                    currentNode = openSet[i];
                }
            }

            openSet.Remove(currentNode);
            closedSet.Add(currentNode);

            if (currentNode == targetNode)
            {
                return RetracePath(startNode, targetNode);
            }

            foreach (Node neighbor in GetNeighbors(currentNode))
            {
                if (!neighbor.walkable || closedSet.Contains(neighbor))
                {
                    continue;
                }

                int newMovementCostToNeighbor = currentNode.gCost + GetDistance(currentNode, neighbor);
                if (newMovementCostToNeighbor < neighbor.gCost || !openSet.Contains(neighbor))
                {
                    neighbor.gCost = newMovementCostToNeighbor;
                    neighbor.hCost = GetDistance(neighbor, targetNode);
                    neighbor.parent = currentNode;

                    if (!openSet.Contains(neighbor))
                        openSet.Add(neighbor);
                }
            }
        }

        // No path found
        Debug.LogWarning($"A* Algorithm completed without finding a path from {startPos} to {targetPos}. Check grid connectivity, obstacles, and walkability."); // Added log
        return null;
    }

    List<Vector3> RetracePath(Node startNode, Node endNode)
    {
        List<Vector3> path = new List<Vector3>();
        Node currentNode = endNode;

        while (currentNode != startNode)
        {
            path.Add(currentNode.worldPosition);
            currentNode = currentNode.parent;
        }
        
        path.Reverse();
        return path;
    }

    int GetDistance(Node nodeA, Node nodeB)
    {
        int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
        int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);

        if (dstX > dstY)
            return 14 * dstY + 10 * (dstX - dstY);
        return 14 * dstX + 10 * (dstY - dstX);
    }

    void OnDrawGizmos()
    {
        if (grid != null && displayGridGizmos)
        {
            foreach (Node n in grid)
            {
                Gizmos.color = n.walkable ? Color.white : Color.red;
                Gizmos.DrawCube(n.worldPosition, Vector3.one * (nodeDiameter - 0.1f));
            }
        }
    }
}