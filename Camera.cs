namespace BerylliumCamera;

public class Camera
{
    #region Offset cell size
    private const float CellSize = 1000.0f;
    #endregion

    #region Update flags
    private bool _updateOffset;
    private bool _updateView;
    private bool _updateProjection;
    #endregion

    #region Parameters
    private Vector3 _offset;
    public Vector3 Offset => _offset;

    public Vector3 RebasedPosition => Position - _offset;

    public Vector3 Position
    {
        get;
        set
        {
            if (field == value) return;

            field = value;
            _updateOffset = true;
            _updateView = true;
        }
    }

    public Quaternion Orientation
    {
        get;
        set
        {
            if (field == value) return;

            field = value;
            _updateView = true;
        }
    }

    private float _fieldOfView;
    public float FieldOfViewDegrees
    {
        get;
        set
        {
            if (Mathematics.AreEqual(field, value)) return;

            field = value;
            TrySetUpdateProjectionFlag(ref _fieldOfView, MathHelper.ToRadians(field));
        }
    }

    public float AspectRatio
    {
        get;
        set => TrySetUpdateProjectionFlag(ref field, value);
    }

    public float NearPlane
    {
        get;
        set => TrySetUpdateProjectionFlag(ref field, value);
    }

    public float FarPlane
    {
        get;
        set => TrySetUpdateProjectionFlag(ref field, value);
    }
    #endregion

    #region Basis
    public Vector3 Forward => Vector3.Transform(Vector3.Forward, Orientation);
    public Vector3 Right => Vector3.Transform(Vector3.Right, Orientation);
    public Vector3 Up => Vector3.Transform(Vector3.Up, Orientation);
    #endregion

    #region Matrices
    public Matrix View { get; private set; }
    public Matrix Projection { get; private set; }
    #endregion

    #region Controller
    public BaseCameraController Controller
    {
        get;
        set
        {
            if (field == value) return;

            field = value;
            field.ApplyTo(this);
        }
    }
    #endregion

    public Camera()
        : this(Vector3.Zero, Vector3.Forward, Vector3.Up)
    {
    }

    public Camera(Vector3 position, Vector3 target, Vector3 up)
    {
        Position = position;
        Orientation = Quaternion.Identity;
        LookAt(target, up);

        FieldOfViewDegrees = 45.0f;
        AspectRatio = 4.0f / 3.0f;
        NearPlane = 0.1f;
        FarPlane = 1000.0f;

        _updateOffset = _updateView = _updateProjection = true;
        Update(0.0f);
    }

    #region LookAt
    public void LookAt(Vector3 worldTarget, Vector3 worldUp)
    {
        var dir = worldTarget - Position; // offset-invariant, so world-space is fine

        if (dir.LengthSquared() < 1e-12f) return; // degenerate: target == position

        var basis = Matrix.CreateWorld(Vector3.Zero, Vector3.Normalize(dir), worldUp);
        Orientation = Quaternion.CreateFromRotationMatrix(basis);
    }
    #endregion

    #region Picking
    public Ray ScreenPointToRay(Vector2 pixel, Viewport viewport)
    {
        var near = viewport.Unproject(new Vector3(pixel.X, pixel.Y, 0.0f), Projection, View, Matrix.Identity) + _offset;
        var far = viewport.Unproject(new Vector3(pixel.X, pixel.Y, 1.0f), Projection, View, Matrix.Identity) + _offset;

        return new Ray(near, Vector3.Normalize(far - near));
    }
    #endregion

    #region Update
    public void Update(float elapsedSeconds)
    {
        Controller?.Update(elapsedSeconds);

        if (_updateOffset) UpdateOffset();
        if (_updateView) UpdateView();
        if (_updateProjection) UpdateProjection();

        _updateOffset = _updateView = _updateProjection = false;
    }
    #endregion

    #region Helpers
    private void TrySetUpdateProjectionFlag(ref float field, float value)
    {
        if (Mathematics.AreEqual(field, value)) return;

        field = value;
        _updateProjection = true;
    }

    private void UpdateOffset()
    {
        var delta = Position - Offset;
        var before = _offset;

        if (MathF.Abs(delta.X) > CellSize) _offset.X += MathF.Truncate(delta.X / CellSize) * CellSize;
        if (MathF.Abs(delta.Y) > CellSize) _offset.Y += MathF.Truncate(delta.Y / CellSize) * CellSize;
        if (MathF.Abs(delta.Z) > CellSize) _offset.Z += MathF.Truncate(delta.Z / CellSize) * CellSize;

        if (_offset != before) _updateView = true;
    }

    private void UpdateView()
    {
        var pos = Position - _offset;
        View = Matrix.CreateLookAt(pos, pos + Forward, Up); // built from the quaternion basis
    }

    private void UpdateProjection()
    {
        Projection = Matrix.CreatePerspectiveFieldOfView(_fieldOfView, AspectRatio, NearPlane, FarPlane);
    }
    #endregion
}
