using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class RPLidarTest : MonoBehaviour
{
    public InputField portInputField;
    public Text stateText;
    public MeshFilter target;
    
    private readonly RPLidarC1 lidar = new();
    private Mesh mesh;

    private void Start()
    {
        mesh = new Mesh();
        target.sharedMesh = mesh;
    }

    private void Update()
    {
        if (lidar.isConnected)
        {
            if (lidar.isScanning) stateText.text = "Scanning";
            else stateText.text = "Connected";
        }
        else stateText.text = "Disconnected";

        if (lidar.isConnected)
        {
            List<PolarCoordinate> points = lidar.GetAllPoints();
            
            Vector3[] vertices = new Vector3[points.Count + 1];
            for (int i = 0; i < points.Count; i++)
                vertices[i] = points[i].ToVector();
            vertices[points.Count] = Vector3.zero;

            int[] indices = new int[points.Count * 3];
            indices[0] = points.Count;
            indices[1] = 0;
            indices[2] = points.Count - 1;
            for (int i = 1; i < points.Count; i++)
            {
                indices[i * 3 + 0] = points.Count;
                indices[i * 3 + 1] = i;
                indices[i * 3 + 2] = i - 1;
            }

            mesh.Clear();
            mesh.vertices = vertices;
            mesh.triangles = indices;
        }
    }

    public void Connect()
    {
        lidar.Connect(portInputField.text);
    }

    public void Disconnect()
    {
        lidar.Disconnect();
    }

    public void StartScanning()
    {
        lidar.StartScanning();
    }

    public void StopScanning()
    {
        lidar.StopScanning();
    }
}
