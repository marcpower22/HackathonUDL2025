using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Globalization;
using System.Net.Sockets;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System;

public class SkeletonVisualizer : MonoBehaviour
{
    [Header("Configuration")]
    public string serverIP = "127.0.0.1"; // IP of the Python Receiver
    public int serverPort = 5000;
    public int webSocketPort = 5001; // Port for WebGL WebSocket
    public bool useWebSocket = false; // Force WebSocket (Auto-enabled on WebGL)
    public string webSocketUrlOverride = ""; // e.g. wss://your-ngrok-url.ngrok-free.app

    public GameObject jointPrefab; // Assign a small Sphere prefab here
    public Material boneMaterial;  // Assign a material for lines

    [Header("Scale & Offset")]
    public float scale = 0.01f; // Adjust based on your camera calibration units (cm vs mm vs m)
    public Vector3 offset = Vector3.zero;

    // Networking
    private TcpClient tcpClient;
    private NetworkStream tcpStream;
    private ClientWebSocket wsClient;
    
    private Thread receiveThread;
    private bool isRunning = false;
    private Queue<string> dataQueue = new Queue<string>();
    private object queueLock = new object();

    // Visuals
    private GameObject[] joints = new GameObject[17];
    private LineRenderer[] bones = new LineRenderer[12]; // Approximate number of connections

    // YOLOv8 Keypoint Mapping
    // 0: Nose, 1: L-Eye, 2: R-Eye, 3: L-Ear, 4: R-Ear
    // 5: L-Shoulder, 6: R-Shoulder, 7: L-Elbow, 8: R-Elbow, 9: L-Wrist, 10: R-Wrist
    // 11: L-Hip, 12: R-Hip, 13: L-Knee, 14: R-Knee, 15: L-Ankle, 16: R-Ankle
    private readonly int[,] boneConnections = new int[,] {
        {5, 7}, {7, 9},       // Left Arm
        {6, 8}, {8, 10},      // Right Arm
        {5, 6},               // Shoulders
        {5, 11}, {6, 12},     // Torso
        {11, 13}, {13, 15},   // Left Leg
        {12, 14}, {14, 16},   // Right Leg
        {11, 12}              // Hips
    };

    void Start()
    {
        InitializeSkeleton();
        
        #if UNITY_WEBGL && !UNITY_EDITOR
            useWebSocket = true;
        #endif

        if (useWebSocket)
        {
            StartCoroutine(ConnectWebSocket());
        }
        else
        {
            StartCoroutine(AttemptConnectionTCP());
        }
    }

    IEnumerator ConnectWebSocket()
    {
        string url = string.IsNullOrEmpty(webSocketUrlOverride) 
            ? $"ws://{serverIP}:{webSocketPort}" 
            : webSocketUrlOverride;

        Debug.Log($"[WebSocket] Connecting to {url}...");

        wsClient = new ClientWebSocket();
        
        // ConnectAsync must be awaited, but we are in a Coroutine.
        // We can use a Task to wrap it.
        Task connectTask = wsClient.ConnectAsync(new Uri(url), CancellationToken.None);
        
        while (!connectTask.IsCompleted) yield return null;

        if (wsClient.State == WebSocketState.Open)
        {
            Debug.Log("[WebSocket] Connected!");
            isRunning = true;
            ReceiveWebSocketData(); // Start receiving loop (async void)
        }
        else
        {
            Debug.LogError($"[WebSocket] Connection failed: {wsClient.State}");
        }
    }

    async void ReceiveWebSocketData()
    {
        byte[] buffer = new byte[4096];
        while (wsClient.State == WebSocketState.Open)
        {
            try
            {
                var result = await wsClient.ReceiveAsync(new ArraySegment<byte>(buffer), CancellationToken.None);
                if (result.MessageType == WebSocketMessageType.Close)
                {
                    await wsClient.CloseAsync(WebSocketCloseStatus.NormalClosure, string.Empty, CancellationToken.None);
                }
                else
                {
                    string message = Encoding.UTF8.GetString(buffer, 0, result.Count);
                    // Handle multiple packets
                    string[] packets = message.Split('\n');
                    lock (queueLock)
                    {
                        foreach (string packet in packets)
                        {
                            if (!string.IsNullOrWhiteSpace(packet))
                            {
                                dataQueue.Enqueue(packet);
                            }
                        }
                    }
                }
            }
            catch (Exception e)
            {
                Debug.LogError($"[WebSocket] Receive Error: {e.Message}");
                break;
            }
        }
    }

    IEnumerator AttemptConnectionTCP()
    {
        while (!isRunning)
        {
            try
            {
                tcpClient = new TcpClient();
                var result = tcpClient.BeginConnect(serverIP, serverPort, null, null);
                var success = result.AsyncWaitHandle.WaitOne(System.TimeSpan.FromSeconds(1));

                if (success)
                {
                    tcpClient.EndConnect(result);
                    tcpStream = tcpClient.GetStream();
                    isRunning = true;
                    receiveThread = new Thread(ReceiveDataTCP);
                    receiveThread.IsBackground = true;
                    receiveThread.Start();
                    Debug.Log("Connected to Python Server (TCP)!");
                }
                else
                {
                    Debug.LogWarning($"Could not connect to {serverIP}:{serverPort}. Retrying...");
                    tcpClient.Close();
                }
            }
            catch (System.Exception e)
            {
                Debug.LogError($"Connection error: {e.Message}");
            }
            
            if (!isRunning) yield return new WaitForSeconds(2);
        }
    }

    void ReceiveDataTCP()
    {
        byte[] buffer = new byte[4096];
        while (isRunning)
        {
            try
            {
                if (tcpStream.DataAvailable)
                {
                    int bytesRead = tcpStream.Read(buffer, 0, buffer.Length);
                    if (bytesRead > 0)
                    {
                        string message = Encoding.UTF8.GetString(buffer, 0, bytesRead);
                        string[] packets = message.Split('\n');
                        
                        lock (queueLock)
                        {
                            foreach (string packet in packets)
                            {
                                if (!string.IsNullOrWhiteSpace(packet))
                                {
                                    dataQueue.Enqueue(packet);
                                }
                            }
                        }
                    }
                }
            }
            catch (System.Exception e)
            {
                Debug.LogWarning($"Receive error: {e.Message}");
                isRunning = false;
            }
            Thread.Sleep(1);
        }
    }

    void InitializeSkeleton()
    {
        // Create Joints
        for (int i = 0; i < 17; i++)
        {
            if (jointPrefab != null)
            {
                joints[i] = Instantiate(jointPrefab, transform);
                joints[i].name = $"Joint_{i}";
            }
            else
            {
                // Fallback: Create a sphere primitive
                // In WebGL, if Physics is stripped, CreatePrimitive(Sphere) might fail because it tries to add a SphereCollider.
                try
                {
                    GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    
                    // We don't need physics for visualization, so remove the collider to avoid issues and save performance
                    Collider col = sphere.GetComponent<Collider>();
                    if (col != null) Destroy(col);

                    sphere.transform.parent = transform;
                    sphere.transform.localScale = Vector3.one * 0.1f;
                    sphere.name = $"Joint_{i}";
                    joints[i] = sphere;
                }
                catch
                {
                    // If CreatePrimitive fails, create an empty object and warn the user
                    Debug.LogWarning($"[SkeletonVisualizer] Could not create Sphere primitive (likely due to stripped Physics in WebGL). Please assign a 'Joint Prefab' in the Inspector.");
                    GameObject empty = new GameObject($"Joint_{i}");
                    empty.transform.parent = transform;
                    joints[i] = empty;
                }
            }
            joints[i].SetActive(false);
        }

        // Create Bones
        for (int i = 0; i < boneConnections.GetLength(0); i++)
        {
            GameObject boneObj = new GameObject($"Bone_{i}");
            boneObj.transform.parent = transform;
            LineRenderer lr = boneObj.AddComponent<LineRenderer>();
            lr.startWidth = 0.05f;
            lr.endWidth = 0.05f;
            lr.material = boneMaterial != null ? boneMaterial : new Material(Shader.Find("Sprites/Default"));
            bones[i] = lr;
            bones[i].enabled = false;
        }
    }

    void Update()
    {
        string dataToProcess = null;
        
        lock (queueLock)
        {
            if (dataQueue.Count > 0)
            {
                // Get the latest packet, discard older ones to reduce lag
                while (dataQueue.Count > 0)
                {
                    dataToProcess = dataQueue.Dequeue();
                }
            }
        }

        if (dataToProcess != null)
        {
            UpdateFrame(dataToProcess);
        }
    }

    void UpdateFrame(string csvLine)
    {
        string[] row = csvLine.Split(',');
        
        // Format: Timestamp, BodyID, KP0_X, KP0_Y, KP0_Z, KP1_X...
        
        for (int i = 0; i < 17; i++)
        {
            int baseIdx = 2 + (i * 3);
            
            if (baseIdx + 2 >= row.Length) break;

            string sX = row[baseIdx];
            string sY = row[baseIdx + 1];
            string sZ = row[baseIdx + 2];

            if (string.IsNullOrEmpty(sX) || string.IsNullOrEmpty(sY) || string.IsNullOrEmpty(sZ))
            {
                joints[i].SetActive(false);
                continue;
            }

            if (float.TryParse(sX, NumberStyles.Any, CultureInfo.InvariantCulture, out float x) &&
                float.TryParse(sY, NumberStyles.Any, CultureInfo.InvariantCulture, out float y) &&
                float.TryParse(sZ, NumberStyles.Any, CultureInfo.InvariantCulture, out float z))
            {
                Vector3 pos = new Vector3(x, -y, z) * scale + offset; 
                joints[i].transform.localPosition = pos;
                joints[i].SetActive(true);
            }
            else
            {
                joints[i].SetActive(false);
            }
        }

        // Update Bones
        for (int i = 0; i < boneConnections.GetLength(0); i++)
        {
            int idxA = boneConnections[i, 0];
            int idxB = boneConnections[i, 1];

            if (joints[idxA].activeSelf && joints[idxB].activeSelf)
            {
                bones[i].enabled = true;
                bones[i].SetPosition(0, joints[idxA].transform.position);
                bones[i].SetPosition(1, joints[idxB].transform.position);
            }
            else
            {
                bones[i].enabled = false;
            }
        }
    }

    void OnApplicationQuit()
    {
        isRunning = false;
        if (receiveThread != null && receiveThread.IsAlive) receiveThread.Join();
        if (tcpStream != null) tcpStream.Close();
        if (tcpClient != null) tcpClient.Close();
        if (wsClient != null && wsClient.State == WebSocketState.Open) wsClient.CloseAsync(WebSocketCloseStatus.NormalClosure, "Quit", CancellationToken.None);
    }
}

