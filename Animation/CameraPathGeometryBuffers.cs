namespace BerylliumCamera.Animation;

public sealed class CameraPathGeometryBuffers
{
    public readonly List<Vector3> Polyline = [];
    public readonly List<Vector3> Waypoints = [];
    public readonly List<CameraPathGizmo> Gizmos = [];
    public readonly List<CameraPathGizmo> WaypointGizmos = [];

    public void Clear()
    {
        Polyline.Clear();
        Waypoints.Clear();
        Gizmos.Clear();
        WaypointGizmos.Clear();
    }
}
