using UnityEngine;
using UnityEngine.UI;

public class GameController : MonoBehaviour
{
    [SerializeField] private ExplorationController explorationController;
    [SerializeField] private float startDelay = 2.0f;

    [Header("Optional UI")]
    [SerializeField] private Button startButton;
    [SerializeField] private Button stopButton;
    // Optional: Add a button to switch exploration modes
    // [SerializeField] private Button switchModeButton;

    private void Start()
    {
        // Auto-find exploration controller if not assigned
        if (explorationController == null)
        {
            explorationController = FindObjectOfType<ExplorationController>();
            if (explorationController == null)
            {
                 Debug.LogError("GameController could not find ExplorationController in the scene!");
                 return; // Stop if controller is missing
            }
        }

        // Set up UI buttons if they exist
        if (startButton != null)
            startButton.onClick.AddListener(StartExploration);

        if (stopButton != null)
            stopButton.onClick.AddListener(StopExploration);

        // Example: Set up mode switch button
        // if (switchModeButton != null)
        //     switchModeButton.onClick.AddListener(SwitchExplorationMode);

        // Auto-start exploration after delay
        Invoke(nameof(StartExploration), startDelay);

        // Test direct movement to a specific position (ensure RobotMovement.cs exists)
        // Invoke(nameof(TestDirectMovement), 1.0f);
    }

    public void StartExploration()
    {
        if (explorationController != null)
        {
            Debug.Log("GameController: Requesting exploration start...");
            explorationController.StartExploration();
        }
        else
        {
            Debug.LogError("GameController: ExplorationController reference is missing!");
        }
    }

    public void StopExploration()
    {
        if (explorationController != null)
        {
            Debug.Log("GameController: Requesting exploration stop...");
            explorationController.StopExploration();
        }
         else
        {
            Debug.LogError("GameController: ExplorationController reference is missing!");
        }
    }

    // Optional: Method to switch exploration mode via UI or other input
    // public void SwitchExplorationMode()
    // {
    //     if (explorationController != null)
    //     {
    //         // Example: Toggle between ZigZag and Greedy
    //         var currentMode = explorationController.currentMode; // Assuming ExplorationController exposes currentMode
    //         var nextMode = currentMode == ExplorationController.ExplorationMode.ZigZag ?
    //                        ExplorationController.ExplorationMode.Greedy :
    //                        ExplorationController.ExplorationMode.ZigZag;
    //         Debug.Log($"GameController: Requesting switch to {nextMode} mode...");
    //         explorationController.SwitchExplorationMode(nextMode);
    //     }
    //      else
    //     {
    //         Debug.LogError("GameController: ExplorationController reference is missing!");
    //     }
    // }


    /// <summary>
    /// Test function to move the robot to a relative position.
    /// Requires a RobotMovement component in the scene.
    /// Uses MoveTo instead of the previous MoveDirectly.
    /// </summary>
    public void TestDirectMovement()
    {
        RobotMovement robot = FindObjectOfType<RobotMovement>();
        if (robot != null)
        {
            Vector3 testPosition = robot.transform.position + robot.transform.forward * 5; // Move 5 units forward from current pos
            Debug.Log($"GameController: Testing direct movement command to {testPosition}");
            robot.MoveTo(testPosition); // Use MoveTo as defined in RobotMovement.cs
        }
        else
        {
            Debug.LogError("GameController: TestDirectMovement failed, RobotMovement not found in scene.");
        }
    }
}