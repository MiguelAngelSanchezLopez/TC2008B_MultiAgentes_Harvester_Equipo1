using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class CollectorController : MonoBehaviour
{
    public int capacity = 0; // Capacidad actual del colector
    public int maxCapacity = 45; // Capacidad máxima del colector
    private NavMeshAgent agent;
    private GameObject targetTractor = null;
    private GameObject warehouse;
    private float transferInterval = 0.25f;
    private bool isTransferring = false;
    private bool isMovingToWarehouse = false; // Indicador para evitar recalculaciones de ruta innecesarias
    private static Dictionary<TractorController, CollectorController> tractorCollectorAssignments = new Dictionary<TractorController, CollectorController>();
    private bool isAtWarehouse = false; // Verifica si el collector ha vaciado su capacidad

    public float stoppingDistanceFromTractor = 7.0f; // Distancia a la que el colector se detendrá cerca del tractor
    public float warehouseArrivalThreshold = 2.0f; // Ajustar esta distancia para evitar colisiones con el NavMeshObstacle

    void Start()
    {
        agent = GetComponent<NavMeshAgent>();
        agent.speed = 5f; // Asegurarse de que el agent tenga una velocidad adecuada
        agent.acceleration = 8f; // Asegurar una aceleración adecuada
        agent.stoppingDistance = 1.5f; // Aumentar el stoppingDistance para que se detenga antes de llegar al NavMeshObstacle
        agent.autoBraking = true; // Asegurarse de que pueda frenar adecuadamente

        FindClosestWarehouse(); // Buscar el warehouse más cercano al iniciar
    }

    void Update()
    {
        // Si el collector está lleno y no se está moviendo al warehouse, se dirige al warehouse
        if (capacity >= maxCapacity && !isMovingToWarehouse)
        {
            GoToWarehouse();
        }

        // Si no está transfiriendo y no está lleno, buscar un tractor lleno
        if (!isTransferring && targetTractor == null && capacity == 0 && !isMovingToWarehouse)
        {
            FindFullTractor();
        }

        // Si hay un tractor lleno y no está transfiriendo, moverse hacia él
        if (targetTractor != null && !isTransferring)
        {
            if (UnityEngine.Vector3.Distance(transform.position, targetTractor.transform.position) > stoppingDistanceFromTractor)
            {
                agent.SetDestination(targetTractor.transform.position);
            }
            else
            {
                agent.isStopped = true;
                StartCoroutine(TransferCrops(targetTractor.GetComponent<TractorController>()));
            }
        }

        // Si no hay más tractores llenos y la capacidad del colector no está llena, moverse al warehouse si no está ya moviéndose
        if (targetTractor == null && !isTransferring && capacity == 0 && !isAtWarehouse && !isMovingToWarehouse)
        {
            agent.SetDestination(warehouse.transform.position);
        }
    }

    void FindFullTractor()
    {
        TractorController[] allTractors = FindObjectsByType<TractorController>(FindObjectsSortMode.None);
        targetTractor = null;
        float minDistance = Mathf.Infinity;

        foreach (var tractor in allTractors)
        {
            // Verificar que el tractor esté lleno y que no esté ya asignado a otro collector
            if (tractor.capacity == tractor.maxCapacity && (!tractorCollectorAssignments.ContainsKey(tractor) || tractorCollectorAssignments[tractor] == this))
            {
                float distance = UnityEngine.Vector3.Distance(transform.position, tractor.transform.position);
                if (distance < minDistance)
                {
                    minDistance = distance;
                    targetTractor = tractor.gameObject;
                }
            }
        }

        if (targetTractor != null)
        {
            // Asignar el tractor al collector
            tractorCollectorAssignments[targetTractor.GetComponent<TractorController>()] = this;
            UnityEngine.Debug.Log($"Collector asignado al tractor: {targetTractor.name}");
            agent.SetDestination(targetTractor.transform.position); // Ir al tractor más cercano
        }
        else
        {
            UnityEngine.Debug.Log("No se encontraron tractores llenos disponibles.");
        }
    }

    void FindClosestWarehouse()
    {
        GameObject[] warehouses = GameObject.FindGameObjectsWithTag("Warehouse");
        warehouse = null;
        float minDistance = Mathf.Infinity;

        foreach (var wh in warehouses)
        {
            float distance = UnityEngine.Vector3.Distance(transform.position, wh.transform.position);
            if (distance < minDistance)
            {
                minDistance = distance;
                warehouse = wh;
            }
        }

        if (warehouse == null)
        {
            UnityEngine.Debug.LogError("No se encontró un objeto con el tag 'Warehouse'. Asegúrate de que haya un warehouse en la escena.");
        }
    }

    IEnumerator TransferCrops(TractorController tractor)
    {
        if (isTransferring)
        {
            yield break; // Evitar que se llame múltiples veces
        }

        isTransferring = true;
        UnityEngine.Debug.Log($"Iniciando transferencia de cultivos del tractor {tractor.name} al collector.");

        tractor.StartTransfer(); // Notificar al tractor que comience la transferencia

        while (tractor.capacity > 0 && this.capacity < this.maxCapacity)
        {
            tractor.capacity--;
            this.capacity++;
            UnityEngine.Debug.Log($"Transferido un crop. Capacidad del tractor: {tractor.capacity}, Capacidad del collector: {this.capacity}");
            yield return new WaitForSeconds(transferInterval);
        }

        UnityEngine.Debug.Log("Transferencia completada.");
        tractor.FinishTransfer(); // Notificar al tractor que la transferencia ha finalizado

        // Liberar el tractor de la asignación después de la transferencia
        if (tractorCollectorAssignments.ContainsKey(tractor))
        {
            tractorCollectorAssignments.Remove(tractor);
        }

        isTransferring = false;
        targetTractor = null;
        agent.isStopped = false;

        // Dirigir al collector al warehouse más cercano para depositar la carga después de la transferencia
        FindClosestWarehouse(); // Actualizar el warehouse más cercano antes de ir
        GoToWarehouse();
    }

    void GoToWarehouse()
    {
        UnityEngine.Debug.Log("El collector está lleno, dirigiéndose al warehouse para vaciar.");
        isAtWarehouse = false;  // Indicamos que aún no ha vaciado su capacidad
        isMovingToWarehouse = true; // Indicamos que ya está en proceso de moverse al warehouse
        agent.SetDestination(warehouse.transform.position);
    }

    void OnTriggerEnter(Collider other)
    {
        // Si el collector entra en contacto con el trigger del warehouse
        if (other.CompareTag("Warehouse") && capacity > 0)
        {
            UnityEngine.Debug.Log("Collector ha llegado al warehouse, vaciando capacidad.");
            StartCoroutine(EmptyCollector());
        }
    }

    IEnumerator EmptyCollector()
    {
        UnityEngine.Debug.Log("Vaciando el collector en el warehouse...");
        yield return new WaitForSeconds(2.0f); // Simula el tiempo de vaciado
        capacity = 0; // Vaciar la capacidad del collector
        isAtWarehouse = true;
        isMovingToWarehouse = false;
        UnityEngine.Debug.Log("El collector ha vaciado su capacidad y está listo para recolectar de nuevo.");

        // Después de vaciar, asegurar que el agent esté activado y buscar el tractor más cercano
        agent.isStopped = false; // Asegurarse de que el agente se pueda mover
        FindFullTractor(); // Reiniciar la búsqueda de tractores llenos
    }
}










