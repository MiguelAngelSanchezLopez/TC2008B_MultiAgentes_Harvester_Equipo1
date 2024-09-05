using UnityEngine;
using Cinemachine;

public class CustomPathMover : MonoBehaviour
{
    public CinemachineVirtualCamera virtualCamera;
    public CinemachineSmoothPath path;
    public float speed = 5.0f;
    private float pathPosition = 0.0f;

    void Update()
    {
        if (virtualCamera != null && path != null)
        {
            // Aumentar la posición del trayecto
            pathPosition += speed * Time.deltaTime;

            // Si llega al final, vuelve al inicio (loop)
            if (pathPosition > path.MaxPos)
                pathPosition = 0;  // Vuelve al inicio del trayecto cuando llegue al final

            // Asignar la nueva posición en el trayecto al Dolly
            CinemachineTrackedDolly dolly = virtualCamera.GetCinemachineComponent<CinemachineTrackedDolly>();
            dolly.m_PathPosition = pathPosition;
        }
    }
}

