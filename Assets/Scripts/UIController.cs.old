using UnityEngine;
using UnityEngine.UI;

public class UIController : MonoBehaviour
{
    [SerializeField] private ExplorationController explorationController;
    [SerializeField] private Button startZigZagButton;
    [SerializeField] private Button startGreedyButton;
    
    private void Start()
    {
        if (startZigZagButton != null)
            startZigZagButton.onClick.AddListener(StartZigZagExploration);
            
        if (startGreedyButton != null)
            startGreedyButton.onClick.AddListener(StartGreedyExploration);
    }
    
    public void StartZigZagExploration()
    {
        if (explorationController != null)
            explorationController.StartExploration();
    }
    
    public void StartGreedyExploration()
    {
        if (explorationController != null)
            explorationController.SwitchToGreedyExploration();
    }
}