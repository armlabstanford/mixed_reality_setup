using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class ScalePublisher : Publisher<Messages.Geometry.Vector3>
    {
        public Transform PublishedTransform;

        private Messages.Geometry.Vector3 message;
        private float previousRealTime;
        //private Vector3 previousPosition = Vector3.zero;
        //private Quaternion previousRotation = Quaternion.identity;

        protected override void Start()
        {
            base.Start();
            InitializeMessage();
        }

        private void FixedUpdate()
        {
            UpdateMessage();
        }

        private void InitializeMessage()
        {

            message = new Messages.Geometry.Vector3();
            //message.linear = new MessageTypes.Geometry.Vector3();
            //message.angular = new MessageTypes.Geometry.Vector3();
        }
        private void UpdateMessage()
        {
            //float deltaTime = Time.realtimeSinceStartup - previousRealTime;

            //Vector3 linearVelocity = (PublishedTransform.position - previousPosition)/deltaTime;
            //Vector3 angularVelocity = (PublishedTransform.rotation.eulerAngles - previousRotation.eulerAngles)/deltaTime;

            // message = GetGeometryPoint(PublishedTransform.position.Unity2Ros());
            message = GetGeometryScale(PublishedTransform.localScale.Unity2RosScale());
            //message.orientation = GetGeometryQuaternion(PublishedTransform.rotation.Unity2Ros());
            //message.linear = GetGeometryVector3(linearVelocity.Unity2Ros());
            //message.angular = GetGeometryVector3(- angularVelocity.Unity2Ros());

            //previousRealTime = Time.realtimeSinceStartup;
            //previousPosition = PublishedTransform.position;
            //previousRotation = PublishedTransform.rotation;

            Publish(message);
        }

        private Messages.Geometry.Vector3 GetGeometryScale(Vector3 scale)
        {
            Messages.Geometry.Vector3 geometryScale = new Messages.Geometry.Vector3();
            geometryScale.x = scale.x;
            geometryScale.y = scale.y;
            geometryScale.z = scale.z;
            return geometryScale;
        }
    }
}
