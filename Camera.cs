using Microsoft.Xna.Framework;

namespace Beryllium.Camera;

public enum CameraType
{
    Free,
    Fixed,
    LookAtLocked
}

public class Camera
{
    private float _frameTime;

    #region Type
    public CameraType CameraType { get; set; }
    #endregion

    #region Main matrices
    public Matrix OffsetWorldMatrix { get; private set; }
    public Matrix ViewMatrix { get; private set; }
    public Matrix ProjectionMatrix { get; private set; }
    #endregion

    #region Position(offset) / look-at
    private Vector3 _oldOffset;

    private Vector3 _offset;
    public Vector3 Offset
    {
        get => _offset;
        set
        {
            if (value == _offset) return;
            _offset = value;
            OffsetWorldMatrix = Matrix.CreateTranslation(-_offset);
        }
    }

    private Vector3 _lookAt;
    public Vector3 LookAt
    {
        get => _lookAt;
        set
        {
            if (value == _lookAt) return;
            _lookAt = value;
            CreateViewMatrixFromLookAtUp();
        }
    }
    #endregion

    #region Main axes
    public Vector3 Forward { get; private set; }

    private Vector3 _up;
    public Vector3 Up
    {
        get => _up;
        set
        {
            if (value == _up) return;
            _up = value;
            CreateViewMatrixFromLookAtUp();
        }
    }
    public Vector3 Right { get; private set; }
    #endregion

    #region Rotation quaternion
    private Quaternion _oldRotation;

    private Quaternion _rotation;
    public Quaternion Rotation
    {
        get => _rotation;
        set
        {
            if (value == _rotation) return;
            _rotation = value;
            CreateViewMatrixFromRotation();
        }
    }
    #endregion

    #region Perspective params
    private int _viewPortWidth;
    public int ViewPortWidth
    {
        get => _viewPortWidth;
        set
        {
            if (value == _viewPortWidth) return;
            _viewPortWidth = value;
            UpdatePerspectiveFov();
        }
    }

    private int _viewPortHeight;
    public int ViewPortHeight
    {
        get => _viewPortHeight;
        set
        {
            if (value == _viewPortHeight) return;
            _viewPortHeight = value;
            UpdatePerspectiveFov();
        }
    }

    private float _fovDegrees;
    public float FovDegrees
    {
        get => _fovDegrees;
        set
        {
            if (Mathematics.Mathematics.IsEqual(value, _fovDegrees)) return;
            _fovDegrees = value;
            UpdatePerspectiveFov();
        }
    }

    private float _zNear;
    public float ZNear
    {
        get => _zNear;
        set
        {
            if (Mathematics.Mathematics.IsEqual(value, _zNear)) return;
            _zNear = value;
            UpdatePerspectiveFov();
        }
    }

    private float _zFar;
    public float ZFar
    {
        get => _zFar;
        set
        {
            if (Mathematics.Mathematics.IsEqual(value, _zFar)) return;
            _zFar = value;
            UpdatePerspectiveFov();
        }
    }
    #endregion

    #region Velocity
    public float MovementVelocity { get; set; }
    public float RotationVelocity { get; set; }
    #endregion

    #region States
    public bool IsMoving { get; private set; }
    public bool IsRotating { get; private set; }
    #endregion

    public Camera(Vector3 position,
        Vector3 lookAt,
        Vector3 up,
        int viewPortWidth,
        int viewPortHeight,
        CameraType cameraType = CameraType.Free,
        float movementVelocity = 200.0f,
        float rotationVelocity = 50.0f,
        float fovDegrees = 45.0f,
        float zNear = 0.1f,
        float zFar = 10000000.0f)
    {
        CameraType = cameraType;
        MovementVelocity = movementVelocity;
        RotationVelocity = rotationVelocity;

        Offset = position;
        _lookAt = lookAt;
        _up = up;

        CreateViewMatrixFromLookAtUp();

        _viewPortWidth = viewPortWidth;
        _viewPortHeight = viewPortHeight;
        _fovDegrees = fovDegrees;
        _zNear = zNear;
        _zFar = zFar;

        UpdatePerspectiveFov();
    }

    #region Creators
    private void CreateViewMatrixFromLookAtUp()
    {
        ViewMatrix = Matrix.CreateLookAt(Vector3.Zero, LookAt - Offset, Up);
        ViewMatrix.Decompose(out _, out _rotation, out _);
        GetMainAxesFromViewMatrix();
    }

    private void CreateViewMatrixFromRotation()
    {
        ViewMatrix = Matrix.CreateFromQuaternion(_rotation);
        GetMainAxesFromViewMatrix();
    }
    #endregion

    #region Updaters
    private void UpdatePerspectiveFov()
    {
        ProjectionMatrix = Matrix.CreatePerspectiveFieldOfView(MathHelper.ToRadians(FovDegrees),
            (float)ViewPortWidth / ViewPortHeight,
            ZNear,
            ZFar);
    }

    public void Update(GameTime gameTime)
    {
        _frameTime = (float)gameTime.ElapsedGameTime.TotalSeconds;
        CheckIsMovingRotating();
    }
    #endregion

    #region Rotation basic functions
    public void SetAbsoluteRotation(float angleXInDegrees, float angleYInDegrees, float angleZInDegrees)
    {
        Rotation = Quaternion.CreateFromYawPitchRoll(MathHelper.ToRadians(angleYInDegrees),
            MathHelper.ToRadians(angleXInDegrees),
            MathHelper.ToRadians(angleZInDegrees));
    }

    public void RotateRelativeX(float angleInDegrees)
    {
        Rotation = Quaternion.Multiply(Quaternion.CreateFromAxisAngle(Vector3.UnitX, MathHelper.ToRadians(angleInDegrees)),
            Rotation);
    }

    public void RotateRelativeY(float angleInDegrees)
    {
        Rotation = Quaternion.Multiply(Quaternion.CreateFromAxisAngle(Vector3.UnitY, MathHelper.ToRadians(angleInDegrees)),
            Rotation);
    }

    public void RotateRelativeZ(float angleInDegrees)
    {
        Rotation = Quaternion.Multiply(Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathHelper.ToRadians(angleInDegrees)),
            Rotation);
    }
    #endregion

    #region Movement basic functions
    public void MoveRelativeX(float relativeX)
    {
        Offset += Vector3.Multiply(Right, relativeX);
    }

    public void MoveRelativeY(float relativeY)
    {
        Offset += Vector3.Multiply(Up, relativeY);
    }

    public void MoveRelativeZ(float relativeZ)
    {
        Offset += Vector3.Multiply(-Forward, relativeZ);
    }
    #endregion

    #region Rotation utility functions
    public void RotateUp()
    {
        RotateRelativeX(-RotationVelocity * _frameTime);
    }

    public void RotateDown()
    {
        RotateRelativeX(RotationVelocity * _frameTime);
    }

    public void RotateLeft()
    {
        RotateRelativeY(-RotationVelocity * _frameTime);
    }

    public void RotateRight()
    {
        RotateRelativeY(RotationVelocity * _frameTime);
    }

    public void TiltLeft()
    {
        RotateRelativeZ(-RotationVelocity * _frameTime);
    }

    public void TiltRight()
    {
        RotateRelativeZ(RotationVelocity * _frameTime);
    }
    #endregion

    #region Movement utility functions
    public void MoveForward()
    {
        MoveRelativeZ(-MovementVelocity * _frameTime);
    }

    public void MoveForward(float multiplier)
    {
        MoveRelativeZ(-MovementVelocity * multiplier * _frameTime);
    }

    public void MoveBackwards()
    {
        MoveRelativeZ(MovementVelocity * _frameTime);
    }

    public void MoveBackwards(float multiplier)
    {
        MoveRelativeZ(MovementVelocity * multiplier * _frameTime);
    }

    public void MoveLeft()
    {
        MoveRelativeX(-MovementVelocity * _frameTime);
    }

    public void MoveRight()
    {
        MoveRelativeX(MovementVelocity * _frameTime);
    }

    public void MoveUp()
    {
        MoveRelativeY(MovementVelocity * _frameTime);
    }

    public void MoveDown()
    {
        MoveRelativeY(-MovementVelocity * _frameTime);
    }
    #endregion

    #region Misc
    private void GetMainAxesFromViewMatrix()
    {
        Right = new Vector3(ViewMatrix.M11, ViewMatrix.M21, ViewMatrix.M31);
        _up = new Vector3(ViewMatrix.M12, ViewMatrix.M22, ViewMatrix.M32);
        Forward = new Vector3(-ViewMatrix.M13, -ViewMatrix.M23, -ViewMatrix.M33);
    }

    private void CheckIsMovingRotating()
    {
        IsMoving = _oldOffset != Offset;
        IsRotating = _oldRotation != _rotation;

        _oldOffset = Offset;
        _oldRotation = _rotation;
    }
    #endregion
}