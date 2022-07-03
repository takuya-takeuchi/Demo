using UnityEngine;

public class LaserPointerBehaviour : MonoBehaviour
{

    #region SerializeFields

    [SerializeField]
    private Transform _HandAnchor;

    [SerializeField]
    private OVRInput.Controller _TargetController;
    
    [SerializeField]
    private OVRInput.Button _GrabTriggerButton;

    [SerializeField]
    private float _MaxDistance = 100.0f;

    [SerializeField]
    private uint _Responsiveness = 1;

    [SerializeField]
    private LineRenderer _LaserPointerRenderer;
    
    [SerializeField]
    private Transform _TrackingSpace;

    #endregion

    #region Fields

    private bool _IsGrabbedObject = false;

    private GameObject _GrabbedObject = null;
    
    private Vector3 _GrabbedPrevFramePos;
    
    private Vector3? _PreviousControllerPosition;

    #endregion

    #region Methods

    #region Magic Methods

    void Update()
    {
        var connected = OVRInput.IsControllerConnected(this._TargetController);
        if (!connected)
            return;

        // Cast ray from controller
        var pointerRay = new Ray(this._HandAnchor.position, this._HandAnchor.forward);

        // 0 is start point
        this._LaserPointerRenderer.SetPosition(0, pointerRay.origin);

        var hit = Physics.Raycast(pointerRay, out var hitInfo, this._MaxDistance);
        if (hit)
        {
            // Set end point to hit position if ray intersects with a collider
            this._LaserPointerRenderer.SetPosition(1, hitInfo.point);

            // Grab object
            var grabTriggerOn = OVRInput.Get(this._GrabTriggerButton);
            if (grabTriggerOn)
            {
                var hitObj = hitInfo.collider.gameObject;
            
                // Check whether that grabbed object is ground or not
                if (hitObj.name != "Plane")
                {
                    this._GrabbedObject = hitObj;
                    this._GrabbedPrevFramePos = hitObj.transform.position;
                    this._IsGrabbedObject = true;
                }

                // Avoid to check _GrabbedObject is null or not
                // https://github.com/JetBrains/resharper-unity/wiki/Avoid-null-comparisons-against-UnityEngine.Object-subclasses
                if (this._IsGrabbedObject)
                {
                    // Move object
                    var controllerPosition = OVRInput.GetLocalControllerPosition(this._TargetController);
                    controllerPosition = this._TrackingSpace.TransformPoint(controllerPosition);
                    if (this._PreviousControllerPosition.HasValue)
                    {
                        // Get movement from previous controller position
                        var diff = controllerPosition - this._PreviousControllerPosition.Value;
                        this._GrabbedObject.transform.position += (diff * this._Responsiveness);
                    }
                    
                    this._PreviousControllerPosition = controllerPosition;
                    
                    // Remember that GetComponent is expensive!!
                    // https://github.com/JetBrains/resharper-unity/wiki/Avoid-usage-of-GetComponent-methods-in-performance-critical-context
                    // Set 0 to velocity because grabbed object will drop suddenly with high speed.
                    // Gravitational acceleration is growing when grabbed object is lifted in the air.
                    var rigidBody = this._GrabbedObject.GetComponent<Rigidbody>();
                    rigidBody.velocity = Vector3.zero;
                }
            }
        }
        else
        {
            // Extend MaxDistance in the direction if ray does not intersect with a collider
            this._LaserPointerRenderer.SetPosition(1, pointerRay.origin + pointerRay.direction * this._MaxDistance);
        }
        
        // Check GetUp rather than grabTriggerOn is false!!
        var grabTriggerOff = OVRInput.GetUp(this._GrabTriggerButton);
        if (grabTriggerOff)
        {
            this._GrabbedObject = null;
            this._PreviousControllerPosition = null;
        }
    }

    #endregion

    #endregion

}