using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Bomb : MonoBehaviour
{
    public int locationArea;

    void OnTriggerEnter(Collider other) {
        if(other.CompareTag("Player")){
            Debug.Log($"Bomb OnTriggerEnter: Checking BombCounter.instance. Is it null? {BombCounter.instance == null}");
            BombCounter.instance.DefuseBomb();
            gameObject.SetActive(false);
        }
    }
}
