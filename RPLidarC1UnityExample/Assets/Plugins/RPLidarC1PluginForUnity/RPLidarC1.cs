using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;

public struct PolarCoordinate
{
    public float angle;
    public float distance;

    public PolarCoordinate(float angle, float distance)
    {
        this.angle = angle;
        this.distance = distance;
    }
    
    public Vector2 ToVector() =>
        distance * new Vector2(Mathf.Cos(angle * Mathf.Deg2Rad), Mathf.Sin(angle * Mathf.Deg2Rad));
}

public class RPLidarC1
{
    public bool isConnected { get; private set; } = false;
    public bool isScanning { get; private set; } = false;
    
    private readonly IntPtr nativeLidar;

    private const int LidarBufferLength = 1024;

    public RPLidarC1()
    {
        nativeLidar = _Create();
    }

    ~RPLidarC1()
    {
        if (isConnected) Disconnect();
        _Destroy(nativeLidar);
    }

    public int head => _Head(nativeLidar);

    public bool Connect(string port)
    {
        if (isConnected) Disconnect();
        if (!_Connect(nativeLidar, port)) return false;
        isConnected = true;
        return true;
    }

    public void Disconnect()
    {
        if (!isConnected) return;
        if (isScanning) StopScanning();
        _Disconnect(nativeLidar);
        isConnected = false;
    }

    public bool StartScanning()
    {
        if (isScanning) return true;
        if (!_Start(nativeLidar)) return false;
        isScanning = true;
        return true;
    }

    public bool StopScanning()
    {
        if (!isScanning) return true;
        if (!_Stop(nativeLidar)) return false;
        isScanning = false;
        return true;
    }

    public PolarCoordinate GetPoint(int index)
    {
        if (!isConnected) return new PolarCoordinate(0, 0);
        return new PolarCoordinate(-_GetAngle(nativeLidar, index), _GetDistance(nativeLidar, index));
    }

    public List<PolarCoordinate> GetAllPoints()
    {
        List<PolarCoordinate> points = new List<PolarCoordinate>();
        int index = head;
        PolarCoordinate prev = GetPoint(index), next;
        float headAngle = prev.angle;
        do
        {
            points.Add(prev);
            next = prev;
            index = (index + LidarBufferLength - 1) % LidarBufferLength;
            prev = GetPoint(index);
        } while (prev.angle < next.angle);
        while (prev.angle > headAngle)
        {
            points.Add(prev);
            index = (index + LidarBufferLength - 1) % LidarBufferLength;
            prev = GetPoint(index);
        }
        return points;
    }

    [DllImport("RPLidarC1NativePlugin", EntryPoint = "create")]
    private static extern IntPtr _Create();
    
    [DllImport("RPLidarC1NativePlugin", EntryPoint = "destroy")]
    private static extern void _Destroy(IntPtr lidar);
    
    [DllImport("RPLidarC1NativePlugin", EntryPoint = "connect")]
    private static extern bool _Connect(IntPtr lidar, string port);
    
    [DllImport("RPLidarC1NativePlugin", EntryPoint = "disconnect")]
    private static extern void _Disconnect(IntPtr lidar);
    
    [DllImport("RPLidarC1NativePlugin", EntryPoint = "start")]
    private static extern bool _Start(IntPtr lidar);
    
    [DllImport("RPLidarC1NativePlugin", EntryPoint = "stop")]
    private static extern bool _Stop(IntPtr lidar);
    
    [DllImport("RPLidarC1NativePlugin", EntryPoint = "head")]
    private static extern int _Head(IntPtr lidar);
    
    [DllImport("RPLidarC1NativePlugin", EntryPoint = "getAngle")]
    private static extern float _GetAngle(IntPtr lidar, int index);
    
    [DllImport("RPLidarC1NativePlugin", EntryPoint = "getDistance")]
    private static extern float _GetDistance(IntPtr lidar, int index);
}
