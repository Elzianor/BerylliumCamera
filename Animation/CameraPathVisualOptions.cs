namespace BerylliumCamera.Animation;

public struct CameraPathVisualOptions
{
    // adaptive (chord-error) sampling vs. fixed sample count
    public bool IsAdaptive;

    // orientation gizmos spaced evenly on whole path
    public int GizmoCount;

    #region For fixed mode
    public int PolylineSamples;
    #endregion

    #region For adaptive mode
    // max allowed deviation of the curve from a chord, in world units
    public float ChordTolerance;

    // recursion depth cap (guards against tiny tolerances)
    public int MaxSubdivisionDepth;
    #endregion

    public static CameraPathVisualOptions Default =>
        new()
        {
            IsAdaptive = false,
            PolylineSamples = 128,
            ChordTolerance = 0.05f,
            MaxSubdivisionDepth = 12,
            GizmoCount = 12
        };
}
