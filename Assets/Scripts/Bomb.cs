using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Bomb : MonoBehaviour
{
    public int locationArea;
    public bool isActive = true;
    public bool isDefused = false;
    
    [SerializeField] private GameObject activeVisual;
    [SerializeField] private GameObject defusedVisual;
    
    private void OnEnable()
    {
        isActive = true;
        isDefused = false;
        
        if (activeVisual != null) activeVisual.SetActive(true);
        if (defusedVisual != null) defusedVisual.SetActive(false);
    }
    
    public void Defuse()
    {
        isDefused = true;
        BombCounter.instance.DefuseBomb();
        
        // Visual feedback
        if (activeVisual != null) activeVisual.SetActive(false);
        if (defusedVisual != null) defusedVisual.SetActive(true);
        
        // Backward compatibility with your existing code
        // Don't disable the whole GameObject to keep the visual feedback
        // Instead, disable the collider if it exists
        Collider bombCollider = GetComponent<Collider>();
        if (bombCollider != null) bombCollider.enabled = false;
    }

    void OnTriggerEnter(Collider other) {
        if(other.CompareTag("Player") && !isDefused){
            Defuse();
            // We don't set gameObject.SetActive(false) anymore
            // Instead, we call Defuse() which handles visuals and disabling the collider
        }
    }
}
