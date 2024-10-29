using UnityEngine;
using RVO;
using Vector2 = RVO.Vector2;
using System.Collections.Generic;

public class VehicleAvoidance_dynamic : MonoBehaviour
{
    public GameObject egoVehicle; // Reference to the ego vehicle
    public GameObject obstacleVehicle; // Reference to the static obstacle vehicle
    private Simulator rvoSimulator;

    private Vector3 initialEgoPosition;
    private Vector3 initialObstaclePosition;
    public float detectionDistance = 2f; // Threshold distance to start avoidance
    public float speed = 2f; // Ego vehicle's speed
    public GameObject additionalObstacleVehicle; // �µľ�̬�ϰ���

    public float obstacle1Speed= 1f; // ��̬�ϰ�����ٶ�
    public float obstacle2Speed = 1f; //�ڶ�����̬�ϰ�����ٶ�
    void Start()
    {
        // ȷ��ֻ��ʼ��һ��
        if (Simulator.Instance != null)
        {
            rvoSimulator = Simulator.Instance;
            Debug.Log("RVO Simulator initialized.");
        }
        else
        {
            Debug.LogError("Failed to get RVO Simulator instance.");
            return; // �˳� Start()�������������ִ��
        }

        // ��ӡ�ϰ����״̬
        if (obstacleVehicle == null)
        {
            Debug.LogError("Obstacle vehicle is not assigned in the Inspector.");
        }
        else
        {
            Debug.Log("Obstacle vehicle is assigned: " + obstacleVehicle.name);
        }

        // ��������
        CreateRVOAgents();

        // ����������
        Debug.Log("Number of agents: " + rvoSimulator.getNumAgents());
    }
    private bool isAvoiding = false; // ���ڸ��ٱ���״̬
    private Vector3 originalForwardDirection = Vector3.forward; // ���ڻָ���ǰ������

    void Update()
    {
        // �������ϰ��������ǰ�˶�
        float movementSpeed = 1.0f; // �ɵ����ƶ��ٶ�
        if (obstacleVehicle != null)
        {
            obstacleVehicle.transform.Translate(Vector3.forward * movementSpeed * Time.deltaTime);
        }
        if (additionalObstacleVehicle != null)
        {
            additionalObstacleVehicle.transform.Translate(Vector3.forward * movementSpeed * Time.deltaTime);
        }

        // ������ԭ�еı��ϼ����ƶ��߼�
        Vector3 egoPosition = egoVehicle.transform.position;

        // �洢�����ϰ���ľ���
        List<float> distancesToObstacles = new List<float>();
        List<Vector3> avoidanceDirections = new List<Vector3>();

        // ����һ���ϰ���
        if (obstacleVehicle != null)
        {
            float distanceToObstacle = Vector3.Distance(egoPosition, obstacleVehicle.transform.position);
            distancesToObstacles.Add(distanceToObstacle);
            avoidanceDirections.Add(CalculateAvoidanceDirection(egoPosition, obstacleVehicle.transform.position));
        }

        // ���ڶ����ϰ���
        if (additionalObstacleVehicle != null)
        {
            float distanceToAdditionalObstacle = Vector3.Distance(egoPosition, additionalObstacleVehicle.transform.position);
            distancesToObstacles.Add(distanceToAdditionalObstacle);
            avoidanceDirections.Add(CalculateAvoidanceDirection(egoPosition, additionalObstacleVehicle.transform.position));
        }

        // ����������ϰ���
        float minDistance = Mathf.Min(distancesToObstacles.ToArray());
        int closestObstacleIndex = distancesToObstacles.IndexOf(minDistance);

        if (minDistance <= detectionDistance)
        {
            isAvoiding = true; // ��ʼ����
            MoveEgoVehicle(avoidanceDirections[closestObstacleIndex]);
        }
        else if (isAvoiding) // ������ɵ��߼�
        {
            RecoverToForwardDirection();
            if (Vector3.Angle(egoVehicle.transform.forward, originalForwardDirection) < 0.5f)
            {
                isAvoiding = false; // ���ñ���״̬
            }
        }
        else
        {
            egoVehicle.transform.Translate(Vector3.forward * speed * Time.deltaTime);
        }

        // ���� RVO ģ��
        rvoSimulator.doStep();
    }



    // �����ָ�������ʻ������ǰ���������������ٶ�Ϊ��ǰ��
    void RecoverToForwardDirection()
    {
        // Ŀ�귽����ǰ����
        Vector3 forwardDirection = Vector3.forward;

        // ��ȡ��ǰ�����ĳ���
        Vector3 currentForward = egoVehicle.transform.forward;

        // ����ָ��ĽǶ�
        Quaternion targetRotation = Quaternion.LookRotation(forwardDirection);
        Quaternion currentRotation = egoVehicle.transform.rotation;

        // ƽ�����ɵ�Ŀ�곯��
        float rotationSpeed = 30.0f; // ���Ե�������ת�ٶ�
        egoVehicle.transform.rotation = Quaternion.Slerp(currentRotation, targetRotation, Time.deltaTime * rotationSpeed);

        // ͬʱ�ó����𽥻ص�ֱ����ʻ
        egoVehicle.transform.Translate(Vector3.forward * speed * Time.deltaTime);
    }
    void CreateRVOAgents()
    {
        if (egoVehicle != null)
        {
            // ���ô���ĳ�ʼ����
            Vector2 egoPosition = new Vector2(egoVehicle.transform.position.x, egoVehicle.transform.position.z);
            float radius = 0.6f;
            int maxSpeed = 2;
            float neighborDist = 1.0f;
            float timeHorizon = 0.4f;

            float timeHorizonObst = 0.6f;
            int maxNeighbors = 10;

            // ��Ӵ���ȷ���������б�Ҫ����
            int egoAgent = rvoSimulator.addAgent(
             egoPosition,         // ����ĳ�ʼ��άλ��
             neighborDist,        // �ھӾ��룬������Ը�֪��������
             maxNeighbors,        // ������Կ��ǵ�����ھ�����
             timeHorizon,         // �������������ʱ�䴰��
             timeHorizonObst,     // ���þ�̬�ϰ����ʱ�䴰��
             radius,              // ����İ뾶
             maxSpeed,            // ���������ٶ�
             new RVO.Vector2(0, 0) // ����ĳ�ʼ�ٶȣ�(0, 0)��ʾ��ֹ
         );
            Debug.Log("Ego vehicle agent added at index: " + egoAgent);
        }
        else
        {
            Debug.LogError("Ego vehicle is null, cannot add RVO agent.");
        }

        if (obstacleVehicle != null)
        {
            // ��ȡ�ϰ��ﳵ���Ŀ�Ⱥͳ���
            float vehicleWidth = 0.89f; // ������
            float vehicleLength = 0.42f; // ���賤��
            // ��ȡ�ϰ��ﳵ��������λ��
            Vector3 obstaclePos = obstacleVehicle.transform.position;
            Debug.Log("Obstacle position: " + obstaclePos); // ��ӡ�ϰ���λ��

            // ���㳵���ĸ��ǵĶ�ά���� (x, z)
            Vector2[] obstacleVertices = new Vector2[]
            {
                new Vector2(obstaclePos.x - vehicleWidth / 4, obstaclePos.z - vehicleLength / 4), // ���½�
                new Vector2(obstaclePos.x - vehicleWidth / 4, obstaclePos.z + vehicleLength / 4), // ���Ͻ�
                new Vector2(obstaclePos.x + vehicleWidth / 4, obstaclePos.z + vehicleLength / 4), // ���Ͻ�
                new Vector2(obstaclePos.x + vehicleWidth / 4, obstaclePos.z - vehicleLength / 4)  // ���½�
            };

            // ��ӡ�ϰ��ﶥ��
            for (int i = 0; i < obstacleVertices.Length; i++)
            {
                Debug.Log("Obstacle vertex " + i + ": " + obstacleVertices[i]);
            }

            // ����ϰ��ﵽRVOģ����
            rvoSimulator.addObstacle(obstacleVertices);
            Debug.Log("Obstacle added with " + obstacleVertices.Length + " vertices.");
        }
        else
        {
            Debug.LogError("Obstacle vehicle is null, cannot add obstacle.");
        }
        // �����µľ�̬�ϰ���
        if (additionalObstacleVehicle != null)
        {
            // ��ȡ�µ��ϰ��ﳵ���Ŀ�Ⱥͳ���
            float vehicleWidth = 0.89f; // ������
            float vehicleLength = 0.42f; // ���賤��
                                         // ��ȡ�µ��ϰ��ﳵ��������λ��
            Vector3 obstaclePos = additionalObstacleVehicle.transform.position;

            // ���㳵���ĸ��ǵĶ�ά���� (x, z)
            Vector2[] obstacleVertices = new Vector2[]
            {
            new Vector2(obstaclePos.x - vehicleWidth / 4, obstaclePos.z - vehicleLength / 4), // ���½�
            new Vector2(obstaclePos.x - vehicleWidth / 4, obstaclePos.z + vehicleLength / 4), // ���Ͻ�
            new Vector2(obstaclePos.x + vehicleWidth / 4, obstaclePos.z + vehicleLength / 4), // ���Ͻ�
            new Vector2(obstaclePos.x + vehicleWidth / 4, obstaclePos.z - vehicleLength / 4)  // ���½�
            };

            // ����ϰ��ﵽRVOģ����
            rvoSimulator.addObstacle(obstacleVertices);
            Debug.Log("Additional obstacle added with " + obstacleVertices.Length + " vertices.");
        }
        else
        {
            Debug.LogError("Additional obstacle vehicle is null, cannot add obstacle.");
        }
        // �����ϰ���
        rvoSimulator.processObstacles();

        // ����������
        Debug.Log("Number of agents after adding ego and obstacle: " + rvoSimulator.getNumAgents());
    }

    Vector3 CalculateAvoidanceDirection(Vector3 egoPosition, Vector3 obstaclePosition)
    {
        Vector3 avoidanceDirection = Vector3.zero;

        // Calculate the relative position of the obstacle
        Vector3 directionToObstacle = obstaclePosition - egoPosition;

        // Determine the angle of lane change
        float angle = 20f; // Set the angle for a smoother lane change

        // If the obstacle is on the right side
        if (directionToObstacle.x > 0)
        {
            // Move to the left with an angle
            avoidanceDirection = Quaternion.Euler(0, -angle, 0) * Vector3.forward; // Turn left at an angle
        }
        else
        {
            // Move to the right with an angle
            avoidanceDirection = Quaternion.Euler(0, angle, 0) * Vector3.forward; // Turn right at an angle
        }

        return avoidanceDirection.normalized;
    }

    void MoveEgoVehicle(Vector3 direction)
    {
        int egoAgent = 0;

        if (egoAgent < rvoSimulator.getNumAgents())
        {
            // ���ô���������ٶ�Ϊ���Ϸ���
            rvoSimulator.setAgentPrefVelocity(egoAgent, new Vector2(direction.x, direction.z));

            // ��ȡ��������ٶ�
            Vector2 newVelocity = rvoSimulator.getAgentVelocity(egoAgent);
            Vector3 velocity3D = new Vector3(newVelocity.x_, 0, newVelocity.y_);

            // ƽ����ת���������µķ���
            if (velocity3D != Vector3.zero)
            {
                Quaternion targetRotation = Quaternion.LookRotation(velocity3D);
                egoVehicle.transform.rotation = Quaternion.Slerp(egoVehicle.transform.rotation, targetRotation, Time.deltaTime * 3f);
            }

            // �ƶ�����
            egoVehicle.transform.position += velocity3D * Time.deltaTime;
        }
        else
        {
            Debug.LogError("Invalid agent index: " + egoAgent);
        }
    }
    // ��һ����̬�ϰ�����˶��߼�
    void MoveFirstDynamicObstacle()
    {
        float moveDirection = Mathf.Sin(Time.time) * obstacle1Speed * Time.deltaTime;
        obstacleVehicle.transform.position += new Vector3(0, 0, moveDirection);
    }

    // �ڶ�����̬�ϰ�����˶��߼�
    void MoveSecondDynamicObstacle()
    {
        float moveDirection = Mathf.Cos(Time.time) * obstacle2Speed * Time.deltaTime;
        additionalObstacleVehicle.transform.position += new Vector3(0, 0, moveDirection);
    }

}
