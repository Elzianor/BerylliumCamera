namespace BerylliumCamera.Animation;

public enum CameraWaypointOrientationMode
{
    // user-defined quaternion
    Fixed,
    // derive orientation by looking at a world-space target
    LookAtTarget,
    // derive orientation from the spline tangent (direction of travel) - looks "down the track"
    TangentFollow
}

public struct CameraWaypoint
{
    public Vector3 Position;

    // for Fixed mode
    public Quaternion Orientation;

    // world-space target for LookAtTarget mode
    public Vector3 LookAtTarget;

    public CameraWaypointOrientationMode OrientationMode;

    #region Factories
    public static CameraWaypoint CreateFixed(Vector3 position, Quaternion orientation)
    {
        return new CameraWaypoint
        {
            Position = position,
            Orientation = orientation,
            LookAtTarget = Vector3.Zero,
            OrientationMode = CameraWaypointOrientationMode.Fixed
        };
    }

    public static CameraWaypoint CreateLookAtTarget(Vector3 position, Vector3 lookAtTarget)
    {
        return new CameraWaypoint
        {
            Position = position,
            Orientation = Quaternion.Identity,
            LookAtTarget = lookAtTarget,
            OrientationMode = CameraWaypointOrientationMode.LookAtTarget
        };
    }

    public static CameraWaypoint CreateTangentFollow(Vector3 position)
    {
        return new CameraWaypoint
        {
            Position = position,
            Orientation = Quaternion.Identity,
            LookAtTarget = Vector3.Zero,
            OrientationMode = CameraWaypointOrientationMode.TangentFollow
        };
    }

    public static CameraWaypoint CreateFixedFromCamera(Camera camera)
    {
        return CreateFixed(camera.Position, camera.Orientation);
    }
    #endregion
}
