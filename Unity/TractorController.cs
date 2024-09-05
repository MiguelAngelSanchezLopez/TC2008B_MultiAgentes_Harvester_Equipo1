using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class TractorController : MonoBehaviour
{
    public int capacity = 0; // Capacidad actual del tractor
    public int maxCapacity = 45; // Capacidad máxima del tractor
    private bool isTransferring = false; // Indica si está transfiriendo crops al collector

    private NavMeshAgent agent;
    private List<GameObject> allCrops; // Lista que almacena todos los cultivos en la escena
    private GameObject targetCrop = null;
    private NavMeshObstacle obstacle;
    private static HashSet<GameObject> occupiedCrops = new HashSet<GameObject>(); // Para evitar que dos tractores tomen el mismo cultivo
    private static Dictionary<GameObject, TractorController> cropAssignments = new Dictionary<GameObject, TractorController>(); // Para manejar asignaciones de cultivos
    private float cropRecheckTimer = 0f;
    private const float cropRecheckInterval = 2f; // Revisa cada 2 segundos si hay cultivos no recolectados
    private float rotationSpeed = 5f; // Velocidad de rotación hacia el objetivo

    void Start()
    {
        agent = GetComponent<NavMeshAgent>();
        obstacle = GetComponent<NavMeshObstacle>();

        // Configura el NavMeshAgent para evitar obstáculos y otros tractores con alta precisión
        agent.obstacleAvoidanceType = ObstacleAvoidanceType.HighQualityObstacleAvoidance;
        agent.radius = 1.5f;
        agent.autoBraking = true;
        agent.avoidancePriority = UnityEngine.Random.Range(50, 100);

        agent.stoppingDistance = 0.2f;

        DetectAllCrops();

        if (allCrops.Count == 0)
        {
            UnityEngine.Debug.LogError("No se encontraron objetos con el tag 'Crop'. Asegúrate de que los objetos estén correctamente etiquetados y tengan colliders.");
            return;
        }

        FindNextCrop();
    }

    void Update()
    {
        // Si el tractor está lleno o transfiriendo, se detiene
        if (capacity >= maxCapacity || isTransferring)
        {
            agent.isStopped = true;
            return;
        }

        if (!agent.pathPending && agent.remainingDistance <= agent.stoppingDistance)
        {
            if (!agent.hasPath || agent.velocity.sqrMagnitude == 0f)
            {
                UnityEngine.Debug.Log("El tractor se ha detenido, recalculando la ruta...");
                RecalculateRoute();
            }
        }

        RotateTowardsTarget();

        RecheckNearbyCrops();

        cropRecheckTimer += Time.deltaTime;
        if (cropRecheckTimer >= cropRecheckInterval)
        {
            cropRecheckTimer = 0f;
            if (allCrops.Count > 0 && targetCrop == null)
            {
                UnityEngine.Debug.LogWarning("Forzando la detección de cultivos no recolectados.");
                RecalculateRoute();
            }
        }
    }

    void RotateTowardsTarget()
    {
        if (targetCrop != null)
        {
            UnityEngine.Vector3 direction = targetCrop.transform.position - transform.position;
            direction.y = 0;
            if (direction.magnitude > agent.stoppingDistance)
            {
                UnityEngine.Quaternion targetRotation = UnityEngine.Quaternion.LookRotation(direction);
                transform.rotation = UnityEngine.Quaternion.Slerp(transform.rotation, targetRotation, Time.deltaTime * rotationSpeed);
            }
        }
    }

    void DetectAllCrops()
    {
        var allObjects = UnityEngine.GameObject.FindObjectsByType<UnityEngine.Transform>(UnityEngine.FindObjectsSortMode.None);
        allCrops = allObjects.Where(obj => obj.CompareTag("Crop") && obj.gameObject.activeInHierarchy).Select(obj => obj.gameObject).ToList();

        UnityEngine.Debug.Log($"Número de cultivos activos detectados: {allCrops.Count}");
    }

    void FindNextCrop()
    {
        UnityEngine.Debug.Log("Buscando el siguiente cultivo más óptimo...");

        DetectAllCrops();

        targetCrop = allCrops
            .Where(crop => crop != null && !occupiedCrops.Contains(crop))
            .OrderBy(crop => CalculateNavMeshPathDistance(transform.position, crop.transform.position))
            .FirstOrDefault();

        if (targetCrop != null)
        {
            UnityEngine.Debug.Log($"Asignado el cultivo {targetCrop.name} en la posición {targetCrop.transform.position}.");

            if (cropAssignments.ContainsKey(targetCrop))
            {
                var otherTractor = cropAssignments[targetCrop];
                if (UnityEngine.Vector3.Distance(transform.position, targetCrop.transform.position) < UnityEngine.Vector3.Distance(otherTractor.transform.position, targetCrop.transform.position))
                {
                    UnityEngine.Debug.Log($"El tractor {name} tiene prioridad sobre el cultivo {targetCrop.name}.");
                    cropAssignments[targetCrop] = this;
                    occupiedCrops.Add(targetCrop);
                    agent.SetDestination(targetCrop.transform.position);
                }
                else
                {
                    UnityEngine.Debug.Log($"El tractor {name} cede el cultivo {targetCrop.name} al tractor más cercano.");
                    targetCrop = null;
                    FindNextCrop();
                }
            }
            else
            {
                cropAssignments[targetCrop] = this;
                occupiedCrops.Add(targetCrop);
                agent.SetDestination(targetCrop.transform.position);
            }
        }
        else
        {
            UnityEngine.Debug.Log("No hay más cultivos disponibles para cosechar.");
        }
    }

    void OnTriggerEnter(UnityEngine.Collider other)
    {
        if (other.CompareTag("Crop") && capacity < maxCapacity && !isTransferring)
        {
            UnityEngine.Debug.Log($"Llegué al cultivo: {other.name} en la posición {other.transform.position}");

            if (other.gameObject != null)
            {
                UnityEngine.Debug.Log($"Cosechando y destruyendo el cultivo: {other.name}");
                Destroy(other.gameObject);
                allCrops.Remove(other.gameObject);
                occupiedCrops.Remove(other.gameObject);
                cropAssignments.Remove(other.gameObject);

                capacity++;

                FindNextCrop();
            }
        }
    }

    public void StartTransfer()
    {
        isTransferring = true;
        agent.isStopped = true; // Asegurarse de que el tractor se detenga completamente durante la transferencia
    }

    public void FinishTransfer()
    {
        isTransferring = false;
        capacity = 0; // Resetear la capacidad después de la transferencia
        agent.isStopped = false; // Reanudar la recolección
        FindNextCrop(); // Continuar con la cosecha
    }

    void RecheckNearbyCrops()
    {
        // Aumenta el radio de detección para cultivos cercanos
        float detectionRadius = agent.radius * 2.5f; // Incrementa el radio de detección para detectar más cultivos cercanos

        var nearbyCrops = allCrops.Where(crop => crop != null && UnityEngine.Vector3.Distance(transform.position, crop.transform.position) < detectionRadius).ToList();

        if (nearbyCrops.Any())
        {
            // Ordena los cultivos cercanos por distancia, para que el tractor elija el más cercano primero
            nearbyCrops = nearbyCrops.OrderBy(crop => UnityEngine.Vector3.Distance(transform.position, crop.transform.position)).ToList();

            foreach (var nearbyCrop in nearbyCrops)
            {
                UnityEngine.Debug.Log($"El tractor detectó un cultivo cercano: {nearbyCrop.name} en la posición {nearbyCrop.transform.position}");

                // Si el objetivo actual no es el cultivo más cercano, cámbialo
                if (targetCrop != nearbyCrop)
                {
                    targetCrop = nearbyCrop;
                    UnityEngine.Debug.Log($"Nuevo objetivo: {targetCrop.name}, recalculando ruta.");

                    // Añade el cultivo a la lista de cultivos ocupados
                    occupiedCrops.Add(targetCrop);

                    // Mueve el tractor hacia el nuevo cultivo cercano
                    agent.SetDestination(targetCrop.transform.position);
                }
                break; // Sal del bucle una vez que hayas seleccionado el cultivo más cercano
            }
        }
    }


    void RecalculateRoute()
    {
        UnityEngine.Debug.Log("Recalculando la ruta hacia un nuevo cultivo...");
        FindNextCrop();
    }

    float CalculateNavMeshPathDistance(UnityEngine.Vector3 start, UnityEngine.Vector3 end)
    {
        return UnityEngine.Vector3.Distance(start, end);
    }
}



























