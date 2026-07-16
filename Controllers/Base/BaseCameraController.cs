namespace BerylliumCamera.Controllers.Base;

public abstract class BaseCameraController
{
    #region Constants
    protected const float ConvergenceAmount = 0.95f;
    #endregion

    protected Camera Camera { get; private set; }

    public virtual void ApplyTo(Camera camera)
    {
        Camera = camera;
        SyncFrom();
        camera.Controller = this;
    }

    protected virtual void SyncFrom()
    {
    }

    public abstract void Update(float elapsedSeconds);
}
