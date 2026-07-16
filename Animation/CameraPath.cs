namespace BerylliumCamera.Animation;

// Solves one problem: given a distance traveled, where is the camera and which way does it face.
// A smooth camera path built from waypoints using a centripetal Catmull-Rom spline,
// reparameterized by arc-length so it can be traversed at constant speed
public sealed class CameraPath
{
    #region Constants
    private const float Alpha = 0.5f; // centripetal
    private const float Epsilon = 1e-6f;
    #endregion

    private readonly int _samplesPerSegment;

    #region Look-up tables
    private float[] _lutDistance;
    private float[] _lutParam;
    #endregion

    #region Waypoints
    public List<CameraWaypoint> Waypoints { get; }
    private Quaternion[] _waypointOrientations;
    #endregion

    #region Visualization
    public bool IsVisualized
    {
        get;
        set
        {
            if (field == value) return;

            field = value;
            if (field) CameraPathVisualizer.Build(this, VisualGeometryBuffers, in VisualOptions);
        }
    }
    public CameraPathGeometryBuffers VisualGeometryBuffers { get; }
    public CameraPathVisualOptions VisualOptions;
    #endregion

    #region Properties
    /// <summary>Loop the path back to the first waypoint. Requires at least 3 waypoints.</summary>
    public bool Closed { get; set; }

    /// <summary>Reference up axis used when deriving look-at / tangent orientations.</summary>
    public Vector3 WorldUp { get; set; }

    /// <summary>Total arc-length in world units. Valid after <see cref="Rebuild" />.</summary>
    public float Length { get; private set; }

    private bool IsClosed => Closed && Waypoints.Count >= 3;

    /// <summary>Number of Catmull-Rom segments.</summary>
    public int SegmentCount => IsClosed ? Waypoints.Count : Math.Max(Waypoints.Count - 1, 0);
    #endregion

    public CameraPath(int samplesPerSegment = 32)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(samplesPerSegment, 1);

        _samplesPerSegment = samplesPerSegment;

        _lutDistance = [];
        _lutParam = [];
        _waypointOrientations = [];
        Waypoints = [];

        VisualGeometryBuffers = new CameraPathGeometryBuffers();
        VisualOptions = CameraPathVisualOptions.Default;

        WorldUp = Vector3.Up;
    }

    #region Rebuild
    public void Rebuild()
    {
        Length = 0.0f;

        var segments = SegmentCount;

        if (segments < 1)
        {
            _lutDistance = [];
            _lutParam = [];
            _waypointOrientations = [];

            // lone waypoint still has a position and an orientation worth showing
            if (IsVisualized) CameraPathVisualizer.Build(this, VisualGeometryBuffers, in VisualOptions);

            return;
        }

        var steps = segments * _samplesPerSegment;
        _lutDistance = new float[steps + 1];
        _lutParam = new float[steps + 1];

        var prevPosition = EvaluatePosition(0.0f);

        // "u" - the spline parameter. Ranges [0, SegmentCount].
        // The integer part picks the segment, the fractional part is position within that segment.
        // This is the "raw" curve coordinate. Equal steps in u do not move equal distances —
        // the camera would crawl on long segments and rush on short ones.

        // _lutDistance[i] — cumulative arc-length up to sample "i" (summed straight chords between samples).
        // _lutParam[i] — the "u" value at that same sample.
        for (var i = 1; i <= steps; i++)
        {
            var u = (float)i / steps * segments;
            var currentPosition = EvaluatePosition(u);

            Length += Vector3.Distance(prevPosition, currentPosition);

            _lutDistance[i] = Length;
            _lutParam[i] = u;

            prevPosition = currentPosition;
        }

        ResolveWaypointOrientations();

        if (!IsVisualized) return;

        CameraPathVisualizer.Build(this, VisualGeometryBuffers, in VisualOptions);
    }
    #endregion

    #region Sampling
    public CameraPathPoint SampleByPathDistance(float pathDistance)
    {
        if (SegmentCount < 1)
        {
            var p0 = Waypoints.Count > 0 ? Waypoints[0].Position : Vector3.Zero;
            var o0 = Waypoints.Count > 0 ? SafeNormalize(Waypoints[0].Orientation) : Quaternion.Identity;

            return new CameraPathPoint(p0, Vector3.Forward, o0, 0.0f);
        }

        pathDistance = MathHelper.Clamp(pathDistance, 0.0f, Length);
        var u = DistanceToParam(pathDistance);

        return new CameraPathPoint(EvaluatePosition(u),
            EvaluateTangent(u),
            SampleOrientation(u, pathDistance),
            pathDistance);
    }

    public CameraPathPoint SampleByPathDistanceNormalized(float value)
    {
        return SampleByPathDistance(MathHelper.Clamp(value, 0.0f, 1.0f) * Length);
    }

    // slerp between the two bracketing waypoints' quaternions
    private Quaternion SampleOrientation(float u, float distance)
    {
        var segments = SegmentCount;

        var currentSegment = Math.Clamp((int)MathF.Floor(WrapParam(u)), 0, segments - 1);
        var nextSegment = IsClosed ? (currentSegment + 1) % Waypoints.Count : currentSegment + 1;

        // interpolate by arc-length within the segment (waypoint boundaries sit on LUT indices).
        var d0 = _lutDistance[currentSegment * _samplesPerSegment];
        var d1 = _lutDistance[(currentSegment + 1) * _samplesPerSegment];

        var span = d1 - d0;

        var f = span > Epsilon ? MathHelper.Clamp((distance - d0) / span, 0.0f, 1.0f) : 0.0f;

        return Quaternion.Slerp(_waypointOrientations[currentSegment], _waypointOrientations[nextSegment], f);
    }

    public Vector3 PositionByPathDistance(float pathDistance)
    {
        if (SegmentCount < 1) return Waypoints.Count > 0 ? Waypoints[0].Position : Vector3.Zero;

        return EvaluatePosition(DistanceToParam(MathHelper.Clamp(pathDistance, 0.0f, Length)));
    }

    public Vector3 PositionByPathDistanceNormalized(float value)
    {
        return PositionByPathDistance(MathHelper.Clamp(value, 0.0f, 1.0f) * Length);
    }
    #endregion

    #region Curve math
    // Position on the curve for the given parameter "u".
    // Decodes "u" into a segment index and a local [0,1],
    // then grabs four control points — P0 P1 P2 P3 — and hands them to CentripetalCatmullRom.
    // The curve only passes through the middle two (P1→P2); the outer two shape the tangents at the ends of the segment.
    private Vector3 EvaluatePosition(float u)
    {
        var segments = SegmentCount;
        u = WrapParam(u);
        var segment = Math.Clamp((int)MathF.Floor(u), 0, segments - 1);
        var local = u - segment;

        return CentripetalCatmullRom(PositionAtWaypoint(segment - 1),
            PositionAtWaypoint(segment),
            PositionAtWaypoint(segment + 1),
            PositionAtWaypoint(segment + 2),
            local);
    }

    // Direction of curve for given the parameter "u".
    private Vector3 EvaluateTangent(float u)
    {
        const float h = 1e-3f;
        var delta = EvaluatePosition(u + h) - EvaluatePosition(u - h);

        return delta.LengthSquared() > Epsilon ? Vector3.Normalize(delta) : Vector3.Forward;
    }

    // Path distance of the point on the path nearest "worldPos". Used to
    // drop the camera onto the path from an arbitrary position. Coarse scan over LUT
    // samples, then local golden-section refine — the coarse pass avoids settling into
    // the wrong local minimum on a curve that folds back on itself.
    public float ClosestDistance(Vector3 worldPos)
    {
        if (SegmentCount < 1) return 0.0f;

        var bestIndex = 0;
        var bestSq = float.MaxValue;

        for (var i = 0; i < _lutParam.Length; i++)
        {
            var sq = Vector3.DistanceSquared(EvaluatePosition(_lutParam[i]), worldPos);

            if (sq >= bestSq) continue;

            bestSq = sq;
            bestIndex = i;
        }

        var step = Length / MathF.Max(1, _lutDistance.Length - 1);
        var low = MathF.Max(0f, _lutDistance[bestIndex] - step);
        var high = MathF.Min(Length, _lutDistance[bestIndex] + step);

        return RefineClosest(worldPos, low, high);
    }
    #endregion

    #region Orientation
    // Turns each waypoint into one quaternion.
    // Fixed reads it directly, LookAtTarget builds it from "target − position",
    // TangentFollow builds it from the tangent (the last two via LookRotation).
    private void ResolveWaypointOrientations()
    {
        var n = Waypoints.Count;
        _waypointOrientations = new Quaternion[n];

        for (var k = 0; k < n; k++) _waypointOrientations[k] = ResolveWaypointOrientation(k);
    }

    /// <summary>
    ///     World orientation of the waypoint at <paramref name="index" />, resolved the same way the spline faces
    ///     there: read directly for Fixed, built from the target for LookAtTarget, from the tangent for TangentFollow.
    ///     Computed on demand, so a per-waypoint orientation frame can be drawn even for a lone waypoint (no Rebuild).
    /// </summary>
    public Quaternion GetWaypointOrientation(int index)
    {
        ArgumentOutOfRangeException.ThrowIfNegative(index);
        ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(index, Waypoints.Count);

        return ResolveWaypointOrientation(index);
    }

    private Quaternion ResolveWaypointOrientation(int k)
    {
        var w = Waypoints[k];
        var q = w.OrientationMode switch
        {
            CameraWaypointOrientationMode.LookAtTarget => LookRotation(w.LookAtTarget - w.Position, WorldUp),
            CameraWaypointOrientationMode.TangentFollow => LookRotation(EvaluateTangent(k), WorldUp),
            _ => w.Orientation
        };

        return SafeNormalize(q);
    }
    #endregion

    #region Helpers
    private float RefineClosest(Vector3 target, float a, float b)
    {
        const float inverseGoldenRatio = 0.6180339887f; // 1/phi, where phi is a golden ratio = 1.61803...
        var c = b - inverseGoldenRatio * (b - a);
        var d = a + inverseGoldenRatio * (b - a);
        var fc = Vector3.DistanceSquared(PositionByPathDistance(c), target);
        var fd = Vector3.DistanceSquared(PositionByPathDistance(d), target);

        for (var k = 0; k < 24 && b - a > 1e-4f; k++)
            if (fc < fd)
            {
                b = d;
                d = c;
                fd = fc;
                c = b - inverseGoldenRatio * (b - a);
                fc = Vector3.DistanceSquared(PositionByPathDistance(c), target);
            }
            else
            {
                a = c;
                c = d;
                fc = fd;
                d = a + inverseGoldenRatio * (b - a);
                fd = Vector3.DistanceSquared(PositionByPathDistance(d), target);
            }

        return 0.5f * (a + b);
    }

    // Binary-searches _lutDistance for the bracket around your distance,
    // then linearly interpolates the stored u. This is the distance → u inversion.
    // Result: the spline parameter that sits at that traveled length.
    private float DistanceToParam(float distance)
    {
        var low = 0;
        var high = _lutDistance.Length - 1;

        while (low < high)
        {
            var mid = (low + high) >> 1;

            if (_lutDistance[mid] < distance)
                low = mid + 1;
            else
                high = mid;
        }

        if (low == 0) return _lutParam[0];

        var i0 = low - 1;
        var i1 = low;

        var span = _lutDistance[i1] - _lutDistance[i0];

        var f = span > Epsilon ? (distance - _lutDistance[i0]) / span : 0.0f;

        return MathHelper.Lerp(_lutParam[i0], _lutParam[i1], f);
    }

    // Normalizes a raw spline parameter "u" into the path's valid range, handling open and closed paths differently.
    // Empty path (SegmentCount < 1): returns 0.
    // Open path: clamps "u" to [0, SegmentCount]. Anything past either end sticks at the endpoint — no extrapolation off the curve.
    // Closed path: wraps "u" modulo SegmentCount (with the + segments fixup so negatives come back positive),
    // so the parameter loops seamlessly — going past the end re-enters at the start.
    private float WrapParam(float u)
    {
        var segments = SegmentCount;

        if (segments < 1) return 0.0f;
        if (!IsClosed) return MathHelper.Clamp(u, 0.0f, segments);

        u %= segments;

        return u < 0.0f ? u + segments : u;
    }

    // Neighbor access: wraps when closed, reflects phantom points at open ends.
    // Closed → indices wrap around (% n), so the curve loops seamlessly.
    // Open → out-of-range indices are reflected (2·P0 − P1), giving the end segments
    // a sensible phantom neighbor instead of running off a cliff.
    private Vector3 PositionAtWaypoint(int i)
    {
        var n = Waypoints.Count;

        if (IsClosed) return Waypoints[(i % n + n) % n].Position;
        if (i < 0) return 2.0f * Waypoints[0].Position - Waypoints[1].Position;
        if (i > n - 1) return 2.0f * Waypoints[n - 1].Position - Waypoints[n - 2].Position;

        return Waypoints[i].Position;
    }

    // Barry-Goldman evaluation of a centripetal Catmull-Rom segment; t in [0,1] over P1->P2.
    private static Vector3 CentripetalCatmullRom(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
    {
        const float t0 = 0.0f;
        var t1 = t0 + KnotDelta(p0, p1);
        var t2 = t1 + KnotDelta(p1, p2);
        var t3 = t2 + KnotDelta(p2, p3);

        var tt = MathHelper.Lerp(t1, t2, t);

        var a1 = Interp(p0, p1, t0, t1, tt);
        var a2 = Interp(p1, p2, t1, t2, tt);
        var a3 = Interp(p2, p3, t2, t3, tt);
        var b1 = Interp(a1, a2, t0, t2, tt);
        var b2 = Interp(a2, a3, t1, t3, tt);

        return Interp(b1, b2, t1, t2, tt);
    }

    // Waypoint is a world position on a curve. Knot is a corresponding parameter "u" value.
    // A rough mental model: waypoints are where the curve goes; knots are when (in parameter terms) the curve arrives at each one.
    // KnotDelta is the function that decides the "when" from the "where."
    // Computes the spacing between two consecutive knots in the centripetal Catmull-Rom parameterization.
    // Given two control points, it returns the distance between them raised to the power Alpha (0.5).
    private static float KnotDelta(Vector3 a, Vector3 b)
    {
        return MathF.Max(MathF.Pow(Vector3.Distance(a, b), Alpha), Epsilon);
    }

    // Linear-interpolation helper — the single building block that the Barry-Goldman evaluation is made of.
    private static Vector3 Interp(Vector3 a, Vector3 b, float ta, float tb, float t)
    {
        var span = tb - ta;
        var f = span > Epsilon ? (t - ta) / span : 0f;

        return Vector3.Lerp(a, b, f);
    }

    // Orientation whose forward faces "forward" (MonoGame -Z convention).
    private static Quaternion LookRotation(Vector3 forward, Vector3 worldUp)
    {
        if (forward.LengthSquared() < Epsilon) return Quaternion.Identity;

        forward = Vector3.Normalize(forward);

        // Pick a non-parallel up if forward is (nearly) vertical.
        if (MathF.Abs(Vector3.Dot(forward, worldUp)) > 0.9999f) worldUp = MathF.Abs(forward.Y) > 0.9999f ? Vector3.Forward : Vector3.Up;

        var m = Matrix.CreateWorld(Vector3.Zero, forward, worldUp);

        return Quaternion.CreateFromRotationMatrix(m);
    }

    // Returns a unit-length version of a quaternion, but guards against the degenerate zero-quaternion case.
    // Only unit-length quaternions represent rotations.
    // The set of all quaternions with length 1 forms the "unit hypersphere" in 4D space,
    // and rotations live precisely on that surface.
    private static Quaternion SafeNormalize(Quaternion q)
    {
        return q.LengthSquared() > Epsilon ? Quaternion.Normalize(q) : Quaternion.Identity;
    }
    #endregion
}
