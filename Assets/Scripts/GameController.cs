using UnityEngine;
using UnityEngine.UI;

public class GameController : MonoBehaviour
{
    [SerializeField] private ExplorationController explorationController;
    [SerializeField] private float startDelay = 2.0f;
    
    [Header("Optional UI")]
    [SerializeField] private Button startButton;
    [SerializeField] private Button stopButton;
    
    private void Start()
    {
        // Set up UI buttons if they exist
        if (startButton != null)
            startButton.onClick.AddListener(StartExploration);
            
        if (stopButton != null)
            stopButton.onClick.AddListener(StopExploration);
        
        // Auto-start exploration after delay
        Invoke("StartExploration", startDelay);

        // Test direct movement to a specific position
        // Invoke("TestDirectMovement", 1.0f);
    }
    
    public void StartExploration()
    {
        if (explorationController != null)
        {
            Debug.Log("Starting exploration sequence...");
            explorationController.StartExploration();
        }
        else
        {
            Debug.LogError("ExplorationController reference is missing!");
        }
    }
    
    public void StopExploration()
    {
        if (explorationController != null)
        {
            Debug.Log("Stopping exploration...");
            explorationController.StopExploration();
        }
    }

    public void TestDirectMovement()
    {
        RobotMovement robot = FindObjectOfType<RobotMovement>();
        if (robot != null)
        {
            Vector3 testPosition = robot.transform.position + Vector3.forward * 5;
            Debug.Log($"Testing direct movement to {testPosition}");
            robot.MoveDirectly(testPosition);
        }
    }
}