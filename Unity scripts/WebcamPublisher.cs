/*
Original code from Siemens AG modified by Shivani Guptasarma, 2021.
WebCamTexture code from Peter Koch, 2017 https://www.youtube.com/watch?v=q96sVKLhjdg
*/

/*
© CentraleSupelec, 2017
Author: Dr. Jeremy Fix (jeremy.fix@centralesupelec.fr)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

// Adjustments to new Publication Timing and Execution Framework 
// © Siemens AG, 2018, Dr. Martin Bischoff (martin.bischoff@siemens.com)

using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class WebcamPublisher : UnityPublisher<MessageTypes.Sensor.CompressedImage>
    {
        //public Camera ImageCamera;
        public string FrameId = "Camera";
        public int resolutionWidth = 640;
        public int resolutionHeight = 480;
        [Range(0, 100)]
        public int qualityLevel = 50;

        private MessageTypes.Sensor.CompressedImage message;
        //private Texture2D texture2D;
        //private Rect rect;

        WebCamTexture webcam;
        Texture2D webcamImage;

        protected override void Start()
        {
            base.Start();

            //the following code accesses webcam and configures capture
            WebCamDevice[] devices = WebCamTexture.devices;
            for (int i = 0; i < devices.Length; i++)
                Debug.Log(devices[i].name);
            //start webcam feed
            webcam = new WebCamTexture(devices[0].name);
            webcam.Play();
            Debug.LogFormat("webcam: {0} {1} x {2}", webcam.deviceName, webcam.width, webcam.height);
            webcamImage = new Texture2D(webcam.width, webcam.height);

            //modified from original
            //InitializeGameObject();
            InitializeMessage();
            //Camera.onPostRender += UpdateImage; //we don't want Unity camera
            UpdateMessage();
        }

        //run in every frame (to replace camera callback)
        private void Update()
        {
            UpdateMessage();
        }

        //modified from original:
        /*private void UpdateImage(Camera _camera)
        {
            if (texture2D != null && _camera == this.ImageCamera)
                UpdateMessage();
        }

        private void InitializeGameObject()
        {
            texture2D = new Texture2D(resolutionWidth, resolutionHeight, TextureFormat.RGB24, false);
            rect = new Rect(0, 0, resolutionWidth, resolutionHeight);
            ImageCamera.targetTexture = new RenderTexture(resolutionWidth, resolutionHeight, 24);
        }*/

        public Texture2D TakePhoto()
        {
            //Debug.Log("take photo");
            webcamImage.SetPixels(webcam.GetPixels());
            webcamImage.Apply();
            Debug.Log(webcamImage.dimension);
            return webcamImage;
        }

        private void InitializeMessage()
        {
            message = new MessageTypes.Sensor.CompressedImage();
            message.header.frame_id = FrameId;
            message.format = "jpeg";
        }

        private void UpdateMessage()
        {
            message.header.Update();
            webcamImage = TakePhoto();
            message.data = webcamImage.EncodeToJPG(qualityLevel);
            Publish(message);
        }

    }
}
