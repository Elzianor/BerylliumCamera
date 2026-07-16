namespace BerylliumCamera.Controllers;

public class FreeCameraController : BaseCameraController
{
    #region Constants
    private const float SnapCompleteRadians = 0.0015f;
    #endregion

    #region Orientation
    // Held as a quaternion rather than yaw/pitch/roll angles, and turned about the camera's OWN axes. Euler angles
    // cannot express that: their yaw is about world Y by construction, so a banked camera swung around the world's
    // up instead of its own and the controls stopped matching the view. There is no pitch limit any more either —
    // "pitch" is only meaningful against a fixed horizon, and this camera no longer has one.
    private Quaternion _orientation = Quaternion.Identity;

    private float _yawInput;
    private float _pitchInput;
    private float _rollInput;
    #endregion

    #region Move input
    private Vector3 _moveInput; // local: X = right, Y = up, Z = forward
    #endregion

    #region Orientation gizmo snapping
    private bool _isSnapping;
    private Quaternion _targetOrientation = Quaternion.Identity;
    private float _snapDamping;
    #endregion

    #region Properties
    public float MoveSpeedUnitSec { get; set; }

    public float RollSpeedRadSec { get; set; }

    public float LookSensitivity { get; set; }

    public float AxisSnapConvergenceSec
    {
        get;
        set
        {
            if (Mathematics.AreEqual(field, value)) return;

            field = value;
            _snapDamping = Mathematics.SecondsToDampingCoefficient(ConvergenceAmount, field);
        }
    }
    #endregion

    public FreeCameraController()
    {
        MoveSpeedUnitSec = 20.0f;
        RollSpeedRadSec = 2.5f;
        LookSensitivity = 0.0025f;
        AxisSnapConvergenceSec = 0.18f;
    }

    #region Control methods
    public void Look(float deltaYaw, float deltaPitch)
    {
        _yawInput += deltaYaw;
        _pitchInput += deltaPitch;
    }

    public void Move(Vector3 delta)
    {
        _moveInput += delta;
    }

    // direction: negative = counter-clockwise, positive = clockwise
    public void Roll(float direction)
    {
        _rollInput += direction;
    }

    // smoothly reorient to look along "forward" (roll levels to 0)
    // used by the orientation gizmo's click-to-snap
    public void SnapTo(Vector3 forward)
    {
        if (forward.LengthSquared() < 1e-12f) return;

        _targetOrientation = LookRotation(Vector3.Normalize(forward));
        _isSnapping = true;
    }
    #endregion

    #region Sync/update
    protected override void SyncFrom()
    {
        if (Camera == null) return;

        // Adopt the camera's orientation whole. The old yaw/pitch state had nowhere to keep roll, so entering free
        // mode quietly levelled the view; nothing is dropped now, so it is picked up exactly as it was left.
        _orientation = Camera.Orientation;
        _orientation.Normalize();

        _yawInput = _pitchInput = _rollInput = 0.0f;
        _moveInput = Vector3.Zero;
        _isSnapping = false;
    }

    public override void Update(float elapsedSeconds)
    {
        if (Camera == null) return;

        // any orientational user input cancels ongoing snap
        if (_yawInput != 0.0f ||
            _pitchInput != 0.0f ||
            _rollInput != 0.0f)
            _isSnapping = false;

        if (_isSnapping)
        {
            var t = Mathematics.InverseExpLerpAmount(_snapDamping, elapsedSeconds);

            _orientation = Quaternion.Slerp(_orientation, _targetOrientation, t);
            _orientation.Normalize();

            if (IsSnapComplete())
            {
                _orientation = _targetOrientation;
                _isSnapping = false;
            }
        }
        else
        {
            // Same axes and signs the yaw/pitch/roll angles used, but applied about the camera's own frame. From a
            // level start these agree exactly with the old Euler build; once rolled or pitched they part company,
            // which is the point — look-left now means left as seen, not left as the world sees it.
            ApplyLocalTurn(Vector3.Up, -_yawInput * LookSensitivity);
            ApplyLocalTurn(Vector3.Right, -_pitchInput * LookSensitivity);
            ApplyLocalTurn(Vector3.Backward, -_rollInput * RollSpeedRadSec * elapsedSeconds);
        }

        _yawInput = 0.0f;
        _pitchInput = 0.0f;
        _rollInput = 0.0f;

        Camera.Orientation = _orientation;

        if (_moveInput == Vector3.Zero) return;

        var world = Vector3.Normalize(Camera.Right * _moveInput.X + // left/right
                                      Camera.Up * _moveInput.Y + // up/down
                                      Camera.Forward * -_moveInput.Z // forward/backward
        );

        Camera.Position += world * MoveSpeedUnitSec * elapsedSeconds;

        _moveInput = Vector3.Zero;
    }
    #endregion

    #region Helpers
    // Composed on the RIGHT: "a * b" applies b first, in the frame a maps out of — so the turn happens in the
    // camera's own axes. On the left it would happen in world axes, which is the behaviour this replaces.
    private void ApplyLocalTurn(Vector3 localAxis, float angle)
    {
        if (angle == 0.0f) return;

        _orientation *= Quaternion.CreateFromAxisAngle(localAxis, angle);
        _orientation.Normalize();
    }

    // Level orientation looking along "forward" — the camera looks down its own -Z, so that axis is negated to get
    // the frame's +Z, and the remaining two are squared off against the world's up to leave no roll.
    private static Quaternion LookRotation(Vector3 forward)
    {
        var z = -forward; // the frame's +Z (backward)

        // Straight up or down: the world's up is parallel to the view and gives no heading, so take the hint from
        // the world's forward instead of letting the cross product collapse.
        var upHint = MathF.Abs(z.Y) > 0.9999f ? Vector3.Forward : Vector3.Up;

        var x = Vector3.Normalize(Vector3.Cross(upHint, z)); // the frame's +X (right)
        var y = Vector3.Cross(z, x);                         // the frame's +Y (up); already unit

        return Quaternion.CreateFromRotationMatrix(new Matrix(x.X,
            x.Y,
            x.Z,
            0.0f,
            y.X,
            y.Y,
            y.Z,
            0.0f,
            z.X,
            z.Y,
            z.Z,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            1.0f));
    }

    private bool IsSnapComplete()
    {
        // q and -q are the same rotation, so compare the angle between them rather than their components.
        var dot = MathF.Min(MathF.Abs(Quaternion.Dot(_orientation, _targetOrientation)), 1.0f);

        return 2.0f * MathF.Acos(dot) < SnapCompleteRadians;
    }
    #endregion
}
