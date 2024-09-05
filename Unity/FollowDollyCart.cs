using UnityEngine;
using Cinemachine;
using System.Diagnostics;

public class FollowDollyCart : MonoBehaviour
{
    public CinemachineDollyCart dollyCart;

    void Update()
    {
        if (dollyCart != null)
        {
            UnityEngine.Debug.Log("Cart Position: " + dollyCart.transform.position);
            UnityEngine.Debug.Log("Camera Position: " + transform.position);

            transform.position = dollyCart.transform.position;
            transform.rotation = dollyCart.transform.rotation;
        }
    }
}

