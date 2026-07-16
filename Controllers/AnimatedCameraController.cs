namespace BerylliumCamera.Controllers;

public class AnimatedCameraController : BaseCameraController
{
    private int _direction = 1; // +1 forward, -1 on the ping-pong return
    private bool _blending;
    private float _blendElapsed;
    private Vector3 _blendFromPosition;
    private Quaternion _blendFromOrientation;

    #region Properties
    public CameraPath Path { get; set; }

    public float DurationSec { get; set; }

    public CameraAnimationMode OpenPathAnimationMode { get; set; }

    public float EntryBlendDurationSec { get; set; }

    public bool SnapToNearestOnEntry { get; set; }

    public bool IsPlaying { get; private set; }

    public float PathDistance { get; private set; }
    #endregion

    public AnimatedCameraController()
    {
        Path = new CameraPath
        {
            Closed = false
        };
        DurationSec = 8.0f;
        OpenPathAnimationMode = CameraAnimationMode.Stop;
        EntryBlendDurationSec = 2.0f;
        SnapToNearestOnEntry = true;
    }

    #region Control methods
    public void Play()
    {
        IsPlaying = true;
    }

    public void Pause()
    {
        IsPlaying = false;
    }

    public void Seek(float pathDistance)
    {
        PathDistance = MathHelper.Clamp(pathDistance, 0.0f, Path?.Length ?? 0.0f);
    }

    public void SeekNormalized(float value)
    {
        Seek(MathHelper.Clamp(value, 0.0f, 1.0f) * (Path?.Length ?? 0.0f));
    }

    public void AddWaypoint(Camera camera,
        CameraWaypointOrientationMode mode = CameraWaypointOrientationMode.Fixed,
        Vector3? lookAt = null)
    {
        if (camera == null) return;

        var waypoint = mode switch
        {
            CameraWaypointOrientationMode.Fixed => CameraWaypoint.CreateFixed(camera.Position, camera.Orientation),
            CameraWaypointOrientationMode.LookAtTarget => CameraWaypoint.CreateLookAtTarget(camera.Position, lookAt ?? Vector3.Zero),
            CameraWaypointOrientationMode.TangentFollow => CameraWaypoint.CreateTangentFollow(camera.Position),
            _ => throw new ArgumentOutOfRangeException(nameof(mode), mode, null)
        };

        Path.Waypoints.Add(waypoint);
        Path.Rebuild();
    }
    #endregion

    #region Sync/update
    protected override void SyncFrom()
    {
        if (Camera == null) return;

        if (SnapToNearestOnEntry && Path is { SegmentCount: >= 1 }) PathDistance = Path.ClosestDistance(Camera.Position);

        _blendFromPosition = Camera.Position;
        _blendFromOrientation = Camera.Orientation;
        _blendElapsed = 0.0f;
        _blending = EntryBlendDurationSec > 0.0f;
    }

    public override void Update(float elapsedSeconds)
    {
        if (Camera == null) return;

        if (Path == null ||
            Path.SegmentCount < 1)
            return;

        if (IsPlaying) Advance(elapsedSeconds);

        var sample = Path.SampleByPathDistance(PathDistance);

        if (_blending)
        {
            _blendElapsed += elapsedSeconds;

            var u = MathHelper.Clamp(_blendElapsed / EntryBlendDurationSec, 0.0f, 1.0f);
            var s = u * u * (3.0f - 2.0f * u); // smoothstep

            Camera.Position = Vector3.Lerp(_blendFromPosition, sample.Position, s);
            Camera.Orientation = Quaternion.Slerp(_blendFromOrientation, sample.Orientation, s);

            if (u >= 1.0f) _blending = false;
        }
        else
        {
            Camera.Position = sample.Position;
            Camera.Orientation = sample.Orientation;
        }
    }
    #endregion

    #region Helpers
    private void Advance(float elapsedSeconds)
    {
        var length = Path.Length;

        if (length <= 0.0f ||
            DurationSec <= 0.0f)
            return;

        PathDistance += length / DurationSec * _direction * elapsedSeconds;

        // Closed paths have no ends — always wrap.
        if (Path.Closed)
        {
            PathDistance = Wrap(PathDistance, length);

            return;
        }

        switch (OpenPathAnimationMode)
        {
            case CameraAnimationMode.Loop:
                PathDistance = Wrap(PathDistance, length);

                break;

            case CameraAnimationMode.PingPong:
                if (PathDistance > length)
                {
                    PathDistance = 2.0f * length - PathDistance;
                    _direction = -1;
                }
                else if (PathDistance < 0.0f)
                {
                    PathDistance = -PathDistance;
                    _direction = 1;
                }

                PathDistance = MathHelper.Clamp(PathDistance, 0.0f, length); // guard huge "elapsedSeconds" overshoot

                break;

            default: // Stop
                if (PathDistance >= length)
                {
                    PathDistance = length;
                    IsPlaying = false;
                }
                else if (PathDistance <= 0.0f)
                {
                    PathDistance = 0.0f;
                    IsPlaying = false;
                }

                break;
        }
    }

    // folds a distance value back into [0, len) for looping traversal
    private static float Wrap(float v, float len)
    {
        v %= len;

        return v < 0.0f ? v + len : v;
    }
    #endregion
}
