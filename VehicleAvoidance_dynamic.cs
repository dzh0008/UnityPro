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
    public GameObject additionalObstacleVehicle; // 新的静态障碍物

    public float obstacle1Speed= 1f; // 动态障碍物的速度
    public float obstacle2Speed = 1f; //第二个静态障碍物的速度
    void Start()
    {
        // 确保只初始化一次
        if (Simulator.Instance != null)
        {
            rvoSimulator = Simulator.Instance;
            Debug.Log("RVO Simulator initialized.");
        }
        else
        {
            Debug.LogError("Failed to get RVO Simulator instance.");
            return; // 退出 Start()，避免后续代码执行
        }

        // 打印障碍物的状态
        if (obstacleVehicle == null)
        {
            Debug.LogError("Obstacle vehicle is not assigned in the Inspector.");
        }
        else
        {
            Debug.Log("Obstacle vehicle is assigned: " + obstacleVehicle.name);
        }

        // 创建代理
        CreateRVOAgents();

        // 检查代理数量
        Debug.Log("Number of agents: " + rvoSimulator.getNumAgents());
    }
    private bool isAvoiding = false; // 用于跟踪避障状态
    private Vector3 originalForwardDirection = Vector3.forward; // 用于恢复正前方方向

    void Update()
    {
        // 让两个障碍物持续向前运动
        float movementSpeed = 1.0f; // 可调的移动速度
        if (obstacleVehicle != null)
        {
            obstacleVehicle.transform.Translate(Vector3.forward * movementSpeed * Time.deltaTime);
        }
        if (additionalObstacleVehicle != null)
        {
            additionalObstacleVehicle.transform.Translate(Vector3.forward * movementSpeed * Time.deltaTime);
        }

        // 以下是原有的避障检测和移动逻辑
        Vector3 egoPosition = egoVehicle.transform.position;

        // 存储所有障碍物的距离
        List<float> distancesToObstacles = new List<float>();
        List<Vector3> avoidanceDirections = new List<Vector3>();

        // 检查第一个障碍物
        if (obstacleVehicle != null)
        {
            float distanceToObstacle = Vector3.Distance(egoPosition, obstacleVehicle.transform.position);
            distancesToObstacles.Add(distanceToObstacle);
            avoidanceDirections.Add(CalculateAvoidanceDirection(egoPosition, obstacleVehicle.transform.position));
        }

        // 检查第二个障碍物
        if (additionalObstacleVehicle != null)
        {
            float distanceToAdditionalObstacle = Vector3.Distance(egoPosition, additionalObstacleVehicle.transform.position);
            distancesToObstacles.Add(distanceToAdditionalObstacle);
            avoidanceDirections.Add(CalculateAvoidanceDirection(egoPosition, additionalObstacleVehicle.transform.position));
        }

        // 查找最近的障碍物
        float minDistance = Mathf.Min(distancesToObstacles.ToArray());
        int closestObstacleIndex = distancesToObstacles.IndexOf(minDistance);

        if (minDistance <= detectionDistance)
        {
            isAvoiding = true; // 开始避障
            MoveEgoVehicle(avoidanceDirections[closestObstacleIndex]);
        }
        else if (isAvoiding) // 避障完成的逻辑
        {
            RecoverToForwardDirection();
            if (Vector3.Angle(egoVehicle.transform.forward, originalForwardDirection) < 0.5f)
            {
                isAvoiding = false; // 重置避障状态
            }
        }
        else
        {
            egoVehicle.transform.Translate(Vector3.forward * speed * Time.deltaTime);
        }

        // 更新 RVO 模拟
        rvoSimulator.doStep();
    }



    // 立即恢复车辆行驶方向到正前方，并设置期望速度为正前方
    void RecoverToForwardDirection()
    {
        // 目标方向（正前方）
        Vector3 forwardDirection = Vector3.forward;

        // 获取当前车辆的朝向
        Vector3 currentForward = egoVehicle.transform.forward;

        // 计算恢复的角度
        Quaternion targetRotation = Quaternion.LookRotation(forwardDirection);
        Quaternion currentRotation = egoVehicle.transform.rotation;

        // 平滑过渡到目标朝向
        float rotationSpeed = 30.0f; // 可以调整的旋转速度
        egoVehicle.transform.rotation = Quaternion.Slerp(currentRotation, targetRotation, Time.deltaTime * rotationSpeed);

        // 同时让车辆逐渐回到直线行驶
        egoVehicle.transform.Translate(Vector3.forward * speed * Time.deltaTime);
    }
    void CreateRVOAgents()
    {
        if (egoVehicle != null)
        {
            // 设置代理的初始参数
            Vector2 egoPosition = new Vector2(egoVehicle.transform.position.x, egoVehicle.transform.position.z);
            float radius = 0.6f;
            int maxSpeed = 2;
            float neighborDist = 1.0f;
            float timeHorizon = 0.4f;

            float timeHorizonObst = 0.6f;
            int maxNeighbors = 10;

            // 添加代理，确保包含所有必要参数
            int egoAgent = rvoSimulator.addAgent(
             egoPosition,         // 代理的初始二维位置
             neighborDist,        // 邻居距离，代理可以感知的最大距离
             maxNeighbors,        // 代理可以考虑的最大邻居数量
             timeHorizon,         // 避让其他代理的时间窗口
             timeHorizonObst,     // 避让静态障碍物的时间窗口
             radius,              // 代理的半径
             maxSpeed,            // 代理的最大速度
             new RVO.Vector2(0, 0) // 代理的初始速度，(0, 0)表示静止
         );
            Debug.Log("Ego vehicle agent added at index: " + egoAgent);
        }
        else
        {
            Debug.LogError("Ego vehicle is null, cannot add RVO agent.");
        }

        if (obstacleVehicle != null)
        {
            // 获取障碍物车辆的宽度和长度
            float vehicleWidth = 0.89f; // 假设宽度
            float vehicleLength = 0.42f; // 假设长度
            // 获取障碍物车辆的中心位置
            Vector3 obstaclePos = obstacleVehicle.transform.position;
            Debug.Log("Obstacle position: " + obstaclePos); // 打印障碍物位置

            // 计算车辆四个角的二维坐标 (x, z)
            Vector2[] obstacleVertices = new Vector2[]
            {
                new Vector2(obstaclePos.x - vehicleWidth / 4, obstaclePos.z - vehicleLength / 4), // 左下角
                new Vector2(obstaclePos.x - vehicleWidth / 4, obstaclePos.z + vehicleLength / 4), // 左上角
                new Vector2(obstaclePos.x + vehicleWidth / 4, obstaclePos.z + vehicleLength / 4), // 右上角
                new Vector2(obstaclePos.x + vehicleWidth / 4, obstaclePos.z - vehicleLength / 4)  // 右下角
            };

            // 打印障碍物顶点
            for (int i = 0; i < obstacleVertices.Length; i++)
            {
                Debug.Log("Obstacle vertex " + i + ": " + obstacleVertices[i]);
            }

            // 添加障碍物到RVO模拟器
            rvoSimulator.addObstacle(obstacleVertices);
            Debug.Log("Obstacle added with " + obstacleVertices.Length + " vertices.");
        }
        else
        {
            Debug.LogError("Obstacle vehicle is null, cannot add obstacle.");
        }
        // 处理新的静态障碍物
        if (additionalObstacleVehicle != null)
        {
            // 获取新的障碍物车辆的宽度和长度
            float vehicleWidth = 0.89f; // 假设宽度
            float vehicleLength = 0.42f; // 假设长度
                                         // 获取新的障碍物车辆的中心位置
            Vector3 obstaclePos = additionalObstacleVehicle.transform.position;

            // 计算车辆四个角的二维坐标 (x, z)
            Vector2[] obstacleVertices = new Vector2[]
            {
            new Vector2(obstaclePos.x - vehicleWidth / 4, obstaclePos.z - vehicleLength / 4), // 左下角
            new Vector2(obstaclePos.x - vehicleWidth / 4, obstaclePos.z + vehicleLength / 4), // 左上角
            new Vector2(obstaclePos.x + vehicleWidth / 4, obstaclePos.z + vehicleLength / 4), // 右上角
            new Vector2(obstaclePos.x + vehicleWidth / 4, obstaclePos.z - vehicleLength / 4)  // 右下角
            };

            // 添加障碍物到RVO模拟器
            rvoSimulator.addObstacle(obstacleVertices);
            Debug.Log("Additional obstacle added with " + obstacleVertices.Length + " vertices.");
        }
        else
        {
            Debug.LogError("Additional obstacle vehicle is null, cannot add obstacle.");
        }
        // 处理障碍物
        rvoSimulator.processObstacles();

        // 检查代理数量
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
            // 设置代理的期望速度为避障方向
            rvoSimulator.setAgentPrefVelocity(egoAgent, new Vector2(direction.x, direction.z));

            // 获取代理的新速度
            Vector2 newVelocity = rvoSimulator.getAgentVelocity(egoAgent);
            Vector3 velocity3D = new Vector3(newVelocity.x_, 0, newVelocity.y_);

            // 平滑旋转车辆朝向新的方向
            if (velocity3D != Vector3.zero)
            {
                Quaternion targetRotation = Quaternion.LookRotation(velocity3D);
                egoVehicle.transform.rotation = Quaternion.Slerp(egoVehicle.transform.rotation, targetRotation, Time.deltaTime * 3f);
            }

            // 移动车辆
            egoVehicle.transform.position += velocity3D * Time.deltaTime;
        }
        else
        {
            Debug.LogError("Invalid agent index: " + egoAgent);
        }
    }
    // 第一个动态障碍物的运动逻辑
    void MoveFirstDynamicObstacle()
    {
        float moveDirection = Mathf.Sin(Time.time) * obstacle1Speed * Time.deltaTime;
        obstacleVehicle.transform.position += new Vector3(0, 0, moveDirection);
    }

    // 第二个动态障碍物的运动逻辑
    void MoveSecondDynamicObstacle()
    {
        float moveDirection = Mathf.Cos(Time.time) * obstacle2Speed * Time.deltaTime;
        additionalObstacleVehicle.transform.position += new Vector3(0, 0, moveDirection);
    }

}
