namespace BerylliumCamera.Animation;

public readonly struct CameraPathGizmo
{
    public readonly Vector3 Position;
    public readonly Vector3 Right;    // local +X
    public readonly Vector3 Up;       // local +Y
    public readonly Vector3 Forward;  // local -Z (look direction)

    public CameraPathGizmo(Vector3 position, Vector3 right, Vector3 up, Vector3 forward)
    {
        Position = position;
        Right = right;
        Up = up;
        Forward = forward;
    }
}
