using UnityEngine;

public class MissionManager : MonoBehaviour
{
    public bool missionActive = true;

    void StartMission()
    {
        missionActive = true;
        Debug.Log("Mission started!");
    }

    void CompleteMission()
    {
        missionActive = false;
        Debug.Log("Mission complete!");
    }
}
