using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
using DG.Tweening;

public class BombCounter : MonoBehaviour
{
    public static BombCounter instance;
    public TMP_Text bombCounterText;
    public TMP_Text txtDefused, txtFinished;
    public Transform centerPos, upPos;

    [SerializeField] int bombCount = 0;

    void Awake(){
        if(instance == null){
            instance = this;
            Debug.Log($"BombCounter instance assigned to: {this.gameObject.name}");
        } else {
            Debug.LogWarning($"Duplicate BombCounter detected on {this.gameObject.name}. Destroying this duplicate.", this.gameObject);
            Destroy(gameObject);
        }
    }

    public void AddBomb(){
        bombCount++;
        UpdateBombCounter();
    }

    public void DefuseBomb(){
        bombCount--;
        UpdateBombCounter();

        if(bombCount == 0){
            Debug.Log("All bombs defused!");
            AudioManager.instance.PlayFinishedSFX();
            txtFinished.transform.DOMove(centerPos.position, 1f).OnComplete(() => {
                // txtFinished.transform.DOMove(upPos.position, 1f);
            });

            Timer.instance.StopTimer();
        }else{
            AudioManager.instance.PlayDefusedSFX();
            txtDefused.transform.DOMove(centerPos.position, 1f).OnComplete(() => {
                txtDefused.transform.DOMove(upPos.position, 1f);
            });
        }
    }

    void UpdateBombCounter(){
        bombCounterText.text = "Bombs remaining: " + bombCount;
    }
}
