using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using Microsoft.Azure.Kinect;
using Microsoft.Azure.Kinect.Sensor;
using Microsoft.Azure.Kinect.BodyTracking;
using Image = Microsoft.Azure.Kinect.Sensor.Image;
using PixelFormat = System.Drawing.Imaging.PixelFormat;
using System.Buffers;
using System.Numerics;

namespace SkeletonTrackK4A
{
    public partial class Form1 : Form
    {      
        Device kinect; 
        Tracker bodyTracker;
        Calibration calib;
        Skeleton skl;

        Bitmap colorBitmap;    
        const int JOINT_NUM = 32;
        bool detected = false;
        bool loop = true;

        public Form1()
        {
            InitializeComponent();
            InitKinect();          
            Task t = KinectLoop();
        }

        private void InitKinect()
        {
            //Connect to 0th Kinect
            kinect = Device.Open(0);
            //Setting device configuration
            kinect.StartCameras(new DeviceConfiguration
            {
                ColorFormat = ImageFormat.ColorBGRA32,
                ColorResolution = ColorResolution.R720p,
                DepthMode = DepthMode.NFOV_2x2Binned,
                SynchronizedImagesOnly = true,
                CameraFPS = FPS.FPS30
            });
            //Getting calibration information to be used for 2D<->3D conversion.
            calib = kinect.GetCalibration();
            
            //Creating information of body tracking mode.
            bodyTracker = Tracker.Create(calib, new TrackerConfiguration 
            {
                ProcessingMode = TrackerProcessingMode.Gpu,
                SensorOrientation = SensorOrientation.Default
            });

           
            
        }
       
        private void CreateColorImage(Capture capture)
        {
            unsafe
            {
                //Getting color image of kinect.
                Image colorImage = capture.Color;
                //Geting the pointer of color image
                using (MemoryHandle pin = colorImage.Memory.Pin())
                {
                    //creating bitmap image
                    colorBitmap = new Bitmap(
                         colorImage.WidthPixels, //width of color image
                         colorImage.HeightPixels,//height of color image
                         colorImage.StrideBytes, //data size of a stride (width*4)
                         PixelFormat.Format32bppArgb,//format (RGBA)
                         (IntPtr)pin.Pointer); //pointer of each pixel
                }        
            }
        }

        private async Task KinectLoop()
        {
            //works while loop is true
            while (loop)
            {
                //Getting latest frame of kinect
                using (Capture capture = await Task.Run(() => kinect.GetCapture()).ConfigureAwait(true))
                {
                    //Creating Bitmap image to be shown as background scene.
                    CreateColorImage(capture);
                    //Input captured frame into bodytracker to recognize human body.
                    bodyTracker.EnqueueCapture(capture);

                    //Acquire the result of human recognition
                    Frame frame = bodyTracker.PopResult();
                    if (frame != null)
                    {
                        uint bodyNum = frame.NumberOfBodies;
                        if (bodyNum > 0)
                        {
                            detected = true;
                            //Using only 0th person even if multiple bodies are recognized.
                            skl = frame.GetBodySkeleton(0);                         
                        }
                        else
                        {
                            detected = false;
                        }
                    }
                    frame.Dispose();
                    //attatching color image of kinect on the picturebox
                    pictureBox1.Image = colorBitmap;
                }             
                this.Update();
            }
            //Stop Kinect 
            kinect.StopCameras();
        }
        

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            loop = false;
        }

        private void pictureBox1_Paint(object sender, PaintEventArgs e)
        {
            if (!detected) return;

            Graphics g = e.Graphics;
            for (int i = 0; i < JOINT_NUM; i++)
            {
                //Convert 3D point to 2D point of color image
                var point = calib.TransformTo2D(skl.GetJoint(i).Position, CalibrationDeviceType.Depth, CalibrationDeviceType.Color);
                if (point != null && skl.GetJoint(i).ConfidenceLevel != JointConfidenceLevel.Low)
                {
                    g.FillEllipse(Brushes.White, point.Value.X - 8, point.Value.Y - 8, 16, 16);                 
                }              
            }
        }
    }
}
