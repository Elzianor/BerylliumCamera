namespace BerylliumCamera.Animation;

// a sample taken from camera path spline at a given path distance
public readonly struct CameraPathPoint
{
    // world-space point on the spline
    public readonly Vector3 Position;

    // unit direction of travel at that point (where the path is heading)
    public readonly Vector3 Tangent;

    // the camera's orientation there, slerped between the two bracketing waypoints' resolved quaternions
    public readonly Quaternion Orientation;

    // length measured along the curve itself from the first waypoint to the sampled point
    // (arc-length from the start, world units)
    public readonly float DistanceUnits;

    public CameraPathPoint(Vector3 position, Vector3 tangent, Quaternion orientation, float distanceUnits)
    {
        Position = position;
        Tangent = tangent;
        Orientation = orientation;
        DistanceUnits = distanceUnits;
    }
}
