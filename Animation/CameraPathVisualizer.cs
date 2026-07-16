namespace BerylliumCamera.Animation;

/// <summary>
///     Emits renderable geometry for a <see cref="CameraPath" /> — curve polyline, waypoint marker
///     positions, and orientation gizmo frames — for your own line/gizmo renderer. Pure data.
/// </summary>
public static class CameraPathVisualizer
{
    #region Build
    public static void Build(CameraPath path, CameraPathGeometryBuffers buffers, in CameraPathVisualOptions options)
    {
        ArgumentNullException.ThrowIfNull(path);
        ArgumentNullException.ThrowIfNull(buffers);

        buffers.Clear();

        if (options.IsAdaptive)
            AppendPolylineAdaptive(path, options.ChordTolerance, options.MaxSubdivisionDepth, buffers.Polyline);
        else
            AppendPolylineUniform(path, options.PolylineSamples, buffers.Polyline);

        AppendWaypoints(path, buffers.Waypoints);
        AppendGizmos(path, options.GizmoCount, buffers.Gizmos);
        AppendWaypointGizmos(path, buffers.WaypointGizmos);
    }
    #endregion

    #region Polyline
    // Even spacing by arc-length gives uniform screen density regardless of waypoint spacing.
    private static void AppendPolylineUniform(CameraPath path, int samples, List<Vector3> outPts)
    {
        if (path.SegmentCount < 1) return;

        samples = Math.Max(1, samples);
        for (var i = 0; i <= samples; i++) outPts.Add(path.PositionByPathDistanceNormalized((float)i / samples));
    }

    // Recursive midpoint subdivision: split only where the curve strays from its chord by
    // more than ChordTolerance, so straight runs stay cheap and bends get detail.
    private static void AppendPolylineAdaptive(CameraPath path, float tolerance, int maxDepth, List<Vector3> outPts)
    {
        var segments = path.SegmentCount;

        if (segments < 1) return;

        tolerance = MathF.Max(tolerance, 1e-5f);

        // Seed one interval per segment (x2) so symmetric bends can't hide from the midpoint test.
        var initial = Math.Max(1, segments * 2);
        outPts.Add(path.PositionByPathDistanceNormalized(0.0f));

        for (var i = 0; i < initial; i++)
        {
            var ta = (float)i / initial;
            var tb = (float)(i + 1) / initial;
            Subdivide(path,
                ta,
                tb,
                path.PositionByPathDistanceNormalized(ta),
                path.PositionByPathDistanceNormalized(tb),
                tolerance,
                maxDepth,
                outPts);
        }
    }

    private static void Subdivide(CameraPath path,
        float ta,
        float tb,
        Vector3 pa,
        Vector3 pb,
        float tolerance,
        int depth,
        List<Vector3> outPts)
    {
        var tm = 0.5f * (ta + tb);
        var pm = path.PositionByPathDistanceNormalized(tm);

        if (depth > 0 &&
            Deviation(pa, pb, pm) > tolerance)
        {
            Subdivide(path, ta, tm, pa, pm, tolerance, depth - 1, outPts);
            Subdivide(path, tm, tb, pm, pb, tolerance, depth - 1, outPts);
        }
        else
        {
            outPts.Add(pb); // segment flat enough; emit the far endpoint only (keeps order, no dupes)
        }
    }

    // Perpendicular distance from midpoint "m" to the chord "a->b".
    private static float Deviation(Vector3 a, Vector3 b, Vector3 m)
    {
        var ab = b - a;
        var abLen = ab.Length();

        if (abLen < 1e-6f) return Vector3.Distance(m, a);

        // The magnitude of a cross-product |u × v| equals the area of the parallelogram spanned by the two vectors.
        // Parallelogram's area: base * height, where base is "ab".
        return Vector3.Cross(m - a, ab).Length() / abLen;
    }
    #endregion

    #region Waypoints
    private static void AppendWaypoints(CameraPath path, List<Vector3> outPts)
    {
        for (var i = 0; i < path.Waypoints.Count; i++) outPts.Add(path.Waypoints[i].Position);
    }
    #endregion

    #region Gizmos
    private static void AppendGizmos(CameraPath path, int count, List<CameraPathGizmo> outFrames)
    {
        if (count < 1) return;

        // A single waypoint forms no segment, but its orientation is exactly what the user
        // wants to see while placing the first one. SampleByFraction handles the degenerate
        // case by returning that waypoint's own position and orientation.
        if (path.SegmentCount < 1)
        {
            if (path.Waypoints.Count > 0) outFrames.Add(ToFrame(path.SampleByPathDistanceNormalized(0.0f)));

            return;
        }

        // Open: include both ends. Closed: last would coincide with first at the seam, so stop short.
        var divisor = path.Closed ? count : Math.Max(1, count - 1);

        for (var i = 0; i < count; i++) outFrames.Add(ToFrame(path.SampleByPathDistanceNormalized((float)i / divisor)));
    }

    private static CameraPathGizmo ToFrame(in CameraPathPoint point)
    {
        return new CameraPathGizmo(point.Position,
            Vector3.Transform(Vector3.Right, point.Orientation),
            Vector3.Transform(Vector3.Up, point.Orientation),
            Vector3.Transform(Vector3.Forward, point.Orientation));
    }
    #endregion

    #region Waypoint gizmos
    private static void AppendWaypointGizmos(CameraPath path, List<CameraPathGizmo> outFrames)
    {
        for (var i = 0; i < path.Waypoints.Count; i++)
        {
            var orientation = path.GetWaypointOrientation(i);

            outFrames.Add(new CameraPathGizmo(path.Waypoints[i].Position,
                Vector3.Transform(Vector3.Right, orientation),
                Vector3.Transform(Vector3.Up, orientation),
                Vector3.Transform(Vector3.Forward, orientation)));
        }
    }
    #endregion
}
