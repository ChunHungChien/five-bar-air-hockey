using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using Emgu.CV;
using Emgu.CV.Structure;
using Emgu.Util;
using Emgu.CV.CvEnum;
using Emgu.CV.Util;


namespace Ptp2AxesControl_CS
{
    
    public partial class Form1 : Form
    {
        private Capture cap = null;

        private const int TOTAL_AXIS = 2;
        private const ushort MASTER_ID = 1;

        private string mEniPath = "";
        private uint mSlaveCnt;

        private uint[] mAxisId = new uint[TOTAL_AXIS];
        private uint[] mVelocity = new uint[TOTAL_AXIS];
        private uint[] mAccDec = new uint[TOTAL_AXIS];
        private int[] mTargetPos = new int[TOTAL_AXIS];
        private float posX;
        private float posY;
        private float posXX;
        private float posYY;
        private float GSWei_X;
        private float GSWei_Y;
        int time = 100;
        //int cam_center_x = 320;
        //int cam_center_y = 240;
        //float robot_center_x = 288f;
        //int robot_center_y = 100;
        int cam_rotation = 0;
        float cam_pix_to_mm = 1.2f;
        float puckCoordX = 0;
        float puckCoordY = 0;
        float puckSpeedX = 0;
        float puckSpeedY = 0;
        float puckOldCoordX = 0;
        float puckOldCoordY = 0;
        float defense_position = 125.0f;//要假設
        float predict_x = 0;    // X position at impact (mm)
        float predict_y = 0;
        float predict_x_old = 0;
        float predict_y_old = 0;
        float predict_time = 0;
        char tempStr;
        char tempStr2;
        int trigger = 0;
        float coordX = 0;
        float coordY = 0;
        float vectorX = 0;
        float vectorY = 0;
        float slope = 0;
        float bounce_x = 0;
        float bounce_y = 0;
        float predict_pixX = 0;
        float predict_pixY = 0;
        float bounce_pixX = 0;
        float bounce_pixY = 0;
        ThreadStart sample;
        Thread mysample;

        double theta1 = 90.0;
        double theta2 = 90.0;

        int b;
        int a;
        int oldb;
        int olda;

        public static int countt = 1;    //計時器

        public Form1()
        {
            InitializeComponent();
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            cap = new Capture(0);
            Application.Idle += new EventHandler(Application_Idle);
            int ret = NexECM.NEC_LoadRtxApp("C:\\Program Files\\NEXCOM\\NexECMRtx\\Lib\\NexECMRtx\\x32\\NexECMRtx.rtss");
            if (ret != 0) { MessageBox.Show("NEC_LoadRtxApp failed" + ret.ToString()); return; }
        }
        void Application_Idle(object sender, EventArgs e)
        {
            //計時檢測
         //   System.Diagnostics.Stopwatch sw = new System.Diagnostics.Stopwatch();//引用stopwatch物件
         //   sw.Reset();//碼表歸零
           // sw.Start();//碼表開始計時
            
            Image<Bgr, Byte> frame = cap.QueryFrame().ToImage<Bgr, Byte>(); // 去query該畫面
            Image<Hsv, byte> framehsv = frame.Convert<Hsv, byte>();   //fram to hsv                       //MessageBox.Show(frame.Size.ToString());
            Image<Gray, byte> grayImage = framehsv.Convert<Gray, byte>();//framhsv to gray
            //MessageBox.Show(image.Data[131, 104, 0].ToString() + "," + image.Data[131, 104, 1].ToString() + "," + image.Data[131, 104, 2].ToString());
            Gray thresholdValue = new Gray(150);
            //取得二值化影像
            //var thresholdImage = grayImage.ThresholdBinary(thresholdValue, new Gray(255));
            CvInvoke.InRange(framehsv, new ScalarArray(new MCvScalar(0, 120, 120)),
            new ScalarArray(new MCvScalar(8, 255, 255)), grayImage);
            VectorOfVectorOfPoint contours = new VectorOfVectorOfPoint();
            CvInvoke.FindContours(grayImage, contours, null, RetrType.External, ChainApproxMethod.ChainApproxSimple);

            int count = contours.Size;
            for (int i = 0; i < count; i++)
            {
                using (VectorOfPoint contour = contours[i])
                {
                    // 使用 BoundingRectangle 取得框選矩形
                    Rectangle BoundingBox = CvInvoke.BoundingRectangle(contour);
                    CvInvoke.Rectangle(grayImage, BoundingBox, new MCvScalar(255), 3);
                    //CvInvoke.Rectangle(frame, BoundingBox, new MCvScalar(255), 3);
                    posXX = (BoundingBox.Left + BoundingBox.Right) / 2;
                    posYY = (BoundingBox.Bottom + BoundingBox.Top) / 2;
                    if ((BoundingBox.Right - BoundingBox.Left) * (BoundingBox.Bottom - BoundingBox.Top) >= 300)
                    {
                        posX = posXX;
                        posY = posYY;
                        //GSWei_Y = (640 - posX) / cam_pix_to_mm;
                       // GSWei_X = (480 - 80 - posY) / (cam_pix_to_mm * (1 - cam_rotation));
                        // CvInvoke.Circle(grayImage, new Point((int)posX,(int)posY), 3, new MCvScalar(255),3);
                        CvInvoke.Circle(frame, new Point((int)posX, (int)posY), 3, new MCvScalar(255), 3);
                        //CvInvoke.Circle(frame, new Point(cam_center_x, cam_center_y), 3, new MCvScalar(255), 3);//畫正中心
                        label6.Text = string.Format("X: {0}, Y: {1}", (int)posX, (int)posY);
                        //label7.Text = string.Format("GSWei_X: {0}, GSWei_Y: {1}", (int)GSWei_X, (int)GSWei_Y);                     
                        
                    }
                }
            }

            

            //cameraprocess(posX, posY);
            // Convert from Camera reference system to Robot reference system
            // We suppose very small angle rotatios (less than 5 degrees) so we use the 
            // aproximation that sin cam_rotation = cam_rotation (in radians)
            // Camera X axis correspond to robot Y axis

            coordY = ((640 - posX) * cam_pix_to_mm) - 0;   // First we convert image coordinates to center of image
            coordX = ((480 - posY) * cam_pix_to_mm) - 0;

            //coordY = robot_center_y - coordY * cam_pix_to_mm;
            //coordX = robot_center_x - coordX * cam_pix_to_mm * (1 - cam_rotation);

            // Calculate speed and angle
            vectorX = (coordX - puckCoordX);
            vectorY = (coordY - puckCoordY);

            puckSpeedX = vectorX * 100 / time;  // speed in dm/ms (
            puckSpeedY = vectorY * 100 / time;
            //puckSpeed = sqrt(vectorX*vectorX + vectorY*vectorY)*1000.0/time;
            //puckDirection = atan2(vectorY,vectorX);
            puckOldCoordX = puckCoordX;
            puckOldCoordY = puckCoordY;
            puckCoordX = coordX;
            puckCoordY = coordY;

            // Noise detection, big vector...
            if ((vectorY < -100) || (vectorY > 100) || (vectorX > 100) || (vectorX < -100))
            {
                Console.WriteLine("NOISE");
                //return ;
            }
            // Its time to predict...
            // Based on actual position, speed and angle we need to know the future...
            // Posible impact?
            if (puckSpeedY < -10)
            {
                // Puck is comming...
                // We need to predict the puck position when it reaches our goal Y=0
                // slope formula: m = (y2-y1)/(x2-x1)
                if (vectorX == 0)  // To avoid division by 0
                    slope = 9999999;
                else
                    slope = (float)vectorY / (float)vectorX;
                // x = (y2-y1)/m + x1
                predict_y = defense_position;
                predict_x = (predict_y - coordY) / slope + coordX;//改了-284.71

                // puck has a bounce with side wall?
                if ((predict_x < 115) || (predict_x > 461))//將會碰撞
                {
                    // We start a new prediction
                    // Wich side?
                    if (predict_x < 115)//碰撞上方
                    {
                        //Left side. We calculare the impact point
                        bounce_x = 115.0f;
                    }
                    else//碰撞下方
                    {
                        //Right side. We calculare the impact point
                        bounce_x = 461.0f;
                    }
                    //bounce_y = (bounce_x - coordX)*slope + coordY;//原式
                    bounce_y = ((bounce_x - predict_x) * slope + 100);//改-284.71
                    bounce_pixX = 640 - (bounce_y / cam_pix_to_mm);
                    bounce_pixY = 480 - (bounce_x / (cam_pix_to_mm * (1 - cam_rotation)));
                    predict_time = (bounce_y - puckCoordY) * 100 / puckSpeedY;  // time until bouce
                                                                                // bounce prediction
                                                                                // Slope change
                    slope = -slope;
                    predict_y = defense_position;
                    predict_x = (predict_y - bounce_y) / slope + bounce_x;

                    if ((predict_x < 115) || (predict_x > 461))
                    {
                        // New bounce?? 
                        // We do nothing then...
                        //sprintf(tempStr2, "2B %d %d", bounce_x, bounce_y);
                        predict_x_old = -1;
                    }
                    else
                    {
                        // draw line
                        //~~~~~~~~cvLine(frameGrabbed, cvPoint(posX / 2, posY / 2), cvPoint(bounce_pixX / 2, bounce_pixY / 2), cvScalar(255, 0, 0), 2);
                        CvInvoke.Line(frame, new Point((int)posX, (int)posY), new Point((int)bounce_pixX, (int)bounce_pixY), new MCvScalar(255, 0, 0), 2);

                        // result average
                        if (predict_x_old != -1)
                            predict_x = (int)(predict_x_old + predict_x) >> 1;
                        predict_x_old = predict_x;
                        predict_time = predict_time + (predict_y - puckCoordY) * 100 / puckSpeedY;  // in ms
                                                                                                    //sprintf(tempStr2, "%d;%d %d %d t%d", coordX, coordY, predict_x, puckSpeedY, predict_time);
                                                                                                    //predict_pixX = cam_center_x - (predict_y - robot_center_y) / cam_pix_to_mm;
                                                                                                    //predict_pixY = cam_center_y - (predict_x - robot_center_x) / (cam_pix_to_mm * (1 - cam_rotation));
                        predict_pixX = 640 - (predict_y / cam_pix_to_mm);
                        predict_pixY = 480 - (predict_x / (cam_pix_to_mm * (1 - cam_rotation)));
                        // draw line
                        //~~~~~~~~~~cvLine(frameGrabbed, cvPoint(bounce_pixX / 2, bounce_pixY / 2), cvPoint(predict_pixX / 2, predict_pixY / 2), cvScalar(0, 255, 0), 2);
                        CvInvoke.Line(frame, new Point((int)bounce_pixX, (int)bounce_pixY), new Point((int)predict_pixX, (int)predict_pixY), new MCvScalar(0, 255, 0), 2);

                    }
                }
                else  // No bounce, direct impact
                {
                    // result average
                    if (predict_x_old != -1)
                        predict_x = (int)(predict_x_old + predict_x) >> 1;
                    predict_x_old = predict_x;

                    predict_time = (predict_y - puckCoordY) * 100 / puckSpeedY;  // in ms
                                                                                 //~~~~~~~~~sprintf(tempStr2, "%d;%d %d %d t%d", coordX, coordY, predict_x, puckSpeedY, predict_time);
                                                                                 // Convert impact prediction position to pixels (to show on image)
                                                                                 //predict_pixX = cam_center_x - (predict_y - robot_center_y) / cam_pix_to_mm;
                    predict_pixX = 640 - (predict_y / cam_pix_to_mm);
                    //predict_pixY = cam_center_y - (predict_x - robot_center_x) / (cam_pix_to_mm * (1 - cam_rotation));
                    predict_pixY = 480 - (predict_x / (cam_pix_to_mm * (1 - cam_rotation)));
                    // draw line
                    //~~~~~~~cvLine(frameGrabbed, cvPoint(posX / 2, posY / 2), cvPoint(predict_pixX / 2, predict_pixY / 2), cvScalar(0, 255, 0), 2);
                    CvInvoke.Line(frame, new Point((int)posX, (int)posY), new Point((int)predict_pixX, (int)predict_pixY), new MCvScalar(0, 255, 0), 2);
                }
            }
            else // Puck is moving slowly or to the other side
            {
                //~~~~~~~~~sprintf(tempStr2, "NO %d,%d %d", coordX, coordY, puckSpeedY);
                predict_x_old = -1;
            }

            GSWei_Y = predict_y;
            GSWei_X = predict_x;
            label7.Text = string.Format("GSWei_X: {0}, GSWei_Y: {1}", (int)GSWei_X, (int)GSWei_Y);

            CvInvoke.Line(frame, new Point((int)0, (int)(95 / cam_pix_to_mm)), new Point((int)640, (int)(95 / cam_pix_to_mm)), new MCvScalar(0, 0, 0), 2);
            CvInvoke.Line(frame, new Point((int)0, (int)(481 / cam_pix_to_mm)), new Point((int)640, (int)(481 / cam_pix_to_mm)), new MCvScalar(0, 0, 0), 2);

            CvInvoke.Line(frame, new Point((int)0, (int)(115 / cam_pix_to_mm)), new Point((int)640, (int)(115 / cam_pix_to_mm)), new MCvScalar(255, 255, 255), 2);
            CvInvoke.Line(frame, new Point((int)0, (int)(461 / cam_pix_to_mm)), new Point((int)640, (int)(461 / cam_pix_to_mm)), new MCvScalar(255, 255, 255), 2);
            CvInvoke.PutText(frame, "posX", new Point(10, 20), 0, 0.7, new MCvScalar(255, 255, 0), 2);

            pictureBox1.Image = frame.ToBitmap();
            CvInvoke.Line(grayImage, new Point(210, 75), new Point((int)(210+180*(Math.Cos(theta1 * Math.PI / 180))), (int)(75 + 180 * (Math.Sin(theta1*Math.PI/180)))), new MCvScalar(255, 255, 255), 2);
            CvInvoke.Line(grayImage, new Point(430, 75), new Point((int)(430 + 180 * (Math.Cos(theta2 * Math.PI / 180))), (int)(75 + 180 * (Math.Sin(theta2 * Math.PI / 180)))), new MCvScalar(255, 255, 255), 2);
            //CvInvoke.Line(grayImage, new Point((int)(210 + 180 * (Math.Cos(theta1 * Math.PI / 180))), (int)(75 + 180 * (Math.Sin(theta1 * Math.PI / 180)))), new Point((int)(210 + 180 * (Math.Cos(theta1 * Math.PI / 180))), (int)(75 + 180 * (Math.Sin(theta1 * Math.PI / 180)))), new MCvScalar(255, 255, 255), 2);
            //CvInvoke.Line(grayImage, new Point((int)(430 + 180 * (Math.Cos(theta2 * Math.PI / 180))), (int)(75 + 180 * (Math.Sin(theta2 * Math.PI / 180)))), new Point((int)(210 + 180 * (Math.Cos(theta1 * Math.PI / 180))), (int)(75 + 180 * (Math.Sin(theta1 * Math.PI / 180)))), new MCvScalar(255, 255, 255), 2);

            pictureBox2.Image = grayImage.ToBitmap();

            frame.Dispose();
            framehsv.Dispose();
            grayImage.Dispose();

            //int x_asix= Convert.ToInt32(GSWei_X);
            //int y_asix = Convert.ToInt32(GSWei_Y);
            //togetherFun1(x_asix, y_asix);
        //    sw.Stop();//碼錶停止                                    
         //   string result1 = sw.Elapsed.TotalMilliseconds.ToString();//印出所花費的總豪秒數                  
         //   richTextBox1.Text += (countt++) + " : " + result1;
          //  richTextBox1.Text += "\n";
        }
  
        private void btn_startNetwork_Click(object sender, EventArgs e)
        {
            int ret = 0;

            if ((tb_EniPath.Text == null) || (tb_EniPath.Text == ""))
            {
                MessageBox.Show("Please load ENI first");
                return;
            }

            if (btn_StartNetwork.BackColor != Color.YellowGreen)
            {
                ret = NexECM.NEC_StartDriver();//初始化函式庫
                if (ret != 0) { MessageBox.Show("NEC_StartDriver failed" + ret.ToString()); return; }

                ret = NexECM.NEC_ResetEcMaster(MASTER_ID);//重置EC-Master
                if (ret != 0) { MessageBox.Show("NEC_ResetEcMaster failed" + ret.ToString()); return; }

                ret = NexECM.NEC_SetParameter(MASTER_ID, 0, 1000);//設定EC-Master參數
                if (ret != 0) { MessageBox.Show("NEC_SetParameter failed" + ret.ToString()); return; }

                ret = NexCoeMotion.NEC_CoE402Reset();//重置函式庫
                if (ret != 0) { MessageBox.Show("NexCoeMotion failed" + ret.ToString()); return; }

                ret = NexECM.NEC_StartNetworkEx(MASTER_ID, mEniPath, 1, 5000);//啟動EC-Master
                if (ret != 0) { MessageBox.Show("NEC_StartNetworkEx failed" + ret.ToString()); return; }

                ret = NexECM.NEC_GetSlaveCount(MASTER_ID, ref mSlaveCnt);//讀取線上EC-Slaves數量(2)
                if (ret != 0) { MessageBox.Show("NEC_GetSlaveCount failed" + ret.ToString()); return; }


                if (TOTAL_AXIS != mSlaveCnt)
                {
                    MessageBox.Show("Please check ENI file and TOTAL_AXIS value!!");
                    return;
                }

                for (ushort i = 0; i < mSlaveCnt; i++)
                {
                    ret = NexCoeMotion.NEC_CoE402GetAxisId(MASTER_ID, i, ref mAxisId[i]);//讀取驅動器ID
                    if (ret != 0) { MessageBox.Show("NEC_CoE402GetAxisId failed" + ret.ToString()); return; }

                    ret = NexCoeMotion.NEC_CoE402UpdatePdoMapping(mAxisId[i]);//更新PDO設定
                    if (ret != 0) { MessageBox.Show("NEC_CoE402UpdatePdoMapping failed" + ret.ToString()); return; }

                    ret = NexCoeMotion.NEC_CoE402FaultReset(mAxisId[i], 5000);//清除重製驅動器錯誤
                    if (ret != 0) { MessageBox.Show("NEC_CoE402FaultReset failed" + ret.ToString()); return; }
                }

                btn_StartNetwork.BackColor = Color.YellowGreen;
                Timer1.Enabled = true;//?
                btn_StartNetwork.Text = "Stop Network";
                btn_Browse.Enabled = false;//?
            }
            else
            {
                Timer1.Enabled = false;//?

                for (int i = 0; i < mSlaveCnt; i++)
                {
                    ret = NexCoeMotion.NEC_CoE402ServoOn(mAxisId[i], 0);//設定驅動器消磁(disable)
                    if (ret != 0) { MessageBox.Show("NEC_CoE402ServoOn diaable failed" + ret.ToString()); return; }
                }

                btn_ServoOn_0.Text = "Servo on";
                btn_Move_0.Text = "Move";

                btn_ServoOn_1.Text = "Servo on";
                btn_Move_1.Text = "Move";

                ret = NexECM.NEC_StopNetwork(MASTER_ID, 5000);
                if (ret != 0) { MessageBox.Show("NEC_StopNetwork failed" + ret.ToString()); return; }

                ret = NexECM.NEC_StopDriver();
                if (ret != 0) { MessageBox.Show("NEC_StopDriver failed" + ret.ToString()); return; }

                btn_StartNetwork.Text = "Start Network";
                btn_StartNetwork.BackColor = Color.WhiteSmoke;
                btn_Browse.Enabled = true;
            }
        }

        private void Form1_FormClosed(object sender, FormClosedEventArgs e)
        {
            Timer1.Enabled = false;

            if (btn_StartNetwork.Text == "Stop Network")
            {
                NexECM.NEC_StopNetwork(MASTER_ID, 5000);
            }
            NexECM.NEC_StopDriver();

           /*if (sample.IsAlive)
            {
                sample.Abort();
            }*/
        }

        private void Timer1_Tick(object sender, EventArgs e)
        {
            int ret = 0;
            int[] actualPos = new int[TOTAL_AXIS];
            ushort[] actualState = new ushort[TOTAL_AXIS];
            ushort[] actualStatusWord = new ushort[TOTAL_AXIS];

            for (int i = 0; i < mSlaveCnt; i++)
            {
                ret = NexCoeMotion.NEC_CoE402GetActualPosition(mAxisId[i], ref actualPos[i]);//讀取馬達實際位置
                if (ret != 0)
                {
                    Timer1.Enabled = false;
                    return;
                }

                ret = NexCoeMotion.NEC_CoE402GetState(mAxisId[i], ref actualState[i]);//讀取馬達狀態
                if (ret != 0)
                {
                    Timer1.Enabled = false;
                    return;
                }

                ret = NexCoeMotion.NEC_CoE402GetStatusWord(mAxisId[i], ref actualStatusWord[i]);//讀取CIA402狀態
                if (ret != 0)
                {
                    Timer1.Enabled = false;
                    return;
                }
            }
            for (int i = 0; i < mSlaveCnt; i++)
            {
                if (actualPos[i] >= 0)
                {
                    actualPos[i] = ((actualPos[i] % 129600) / 360);
                }
                else {
                    actualPos[i] = ((actualPos[i] % 129600) / 360) + 360;
                }
            }

            theta1 = (double)actualPos[0];
            theta2 = (double)actualPos[1];

            tb_ActualPos_0.Text = Convert.ToString(actualPos[0]);
            tb_SlaveState_0.Text = Convert.ToString(actualState[0]);

            tb_ActualPos_1.Text = Convert.ToString(actualPos[1]);
            tb_SlaveState_1.Text = Convert.ToString(actualState[1]);

            if ((actualStatusWord[0] & 1024) == 1024)
            {
                btn_Move_0.Text = "Move";

            }
            if ((actualStatusWord[1] & 1024) == 1024)
            {
                btn_Move_1.Text = "Move";
            }   
        }

        private void btn_Exit_Click(object sender, EventArgs e)
        {
            mysample.Abort();

            Timer1.Enabled = false;

            //sample

            if (btn_StartNetwork.Text == "Stop Network")
            {
                NexECM.NEC_StopNetwork(MASTER_ID, 5000);
            }

            NexECM.NEC_StopDriver();
            this.Close();
        }

        private void btn_browse_Click(object sender, EventArgs e)
        {
            OpenFileDialog dialog = new OpenFileDialog();
            dialog.Filter = "ENI Files|*.xml";
            dialog.Title = "Select a ENI Files";

            if (dialog.ShowDialog() == DialogResult.OK)
            {
                tb_EniPath.Text = dialog.FileName;
                mEniPath = dialog.FileName;
            }
        }

        private void btn_startMove_Click(object sender, EventArgs e)
        {
            const int ID = 0;
            ServoOn(ID);
        }

        private void btn_startMove1_Click(object sender, EventArgs e)
        {
            const int ID = 1;
            ServoOn(ID);
        }

        private void btn_Move_Click(object sender, EventArgs e)
        {
            const int ID = 0;
            MoveFunc(ID);
        }

        private void btn_Move1_Click(object sender, EventArgs e)
        {
            const int ID = 1;
            MoveFunc(ID);
        }

        private void ServoOn(int ID)
        {
            if (btn_StartNetwork.BackColor != Color.YellowGreen)
            {
                MessageBox.Show("Please start network first!!");
                return;
            }

            int ret = 0;

            Button btn_startMove;
            Button btn_move;

            if (ID == 0)
            {
                btn_startMove = btn_ServoOn_0;
                btn_move = btn_Move_0;
            }
            else
            {
                btn_startMove = btn_ServoOn_1;
                btn_move = btn_Move_1;
            }

            if (btn_startMove.Text == "Servo on")
            {
                ret = NexCoeMotion.NEC_CoE402SetOperationMode(mAxisId[ID], 1, 5000);
                if (ret != 0) { MessageBox.Show("NEC_CoE402SetOperationMode failed" + ret.ToString()); return; }

                ret = NexCoeMotion.NEC_CoE402ServoOn(mAxisId[ID], 1);
                if (ret != 0) { MessageBox.Show("NEC_CoE402ServoOn failed" + ret.ToString()); return; }

                ret = NexCoeMotion.NEC_CoE402Halt(mAxisId[ID], 0);
                if (ret != 0) { MessageBox.Show("NEC_CoE402Halt failed" + ret.ToString()); return; }

                btn_startMove.Text = "Servo off";

            }
            else
            {
                ret = NexCoeMotion.NEC_CoE402ServoOn(mAxisId[ID], 0);
                if (ret != 0) { MessageBox.Show("NEC_CoE402ServoOn failed" + ret.ToString()); return; }

                btn_startMove.Text = "Servo on";
                btn_move.Text = "Move";
            }
        }

        private void MoveFunc(int ID)
        {
            int ret = 0;

            TextBox tb_targetPos;
            TextBox tb_velocity;
            TextBox tb_acc;
            Button btn_move;
            Button btn_startMove;

            if (ID == 0)
            {
                tb_targetPos = tb_TargetPos_0;
                tb_velocity = tb_Vel_0;
                tb_acc = tb_AccDec_0;
                btn_move = btn_Move_0;
                btn_startMove = btn_ServoOn_0;
            }
            else
            {
                tb_targetPos = tb_TargetPos_1;
                tb_velocity = tb_Vel_1;
                tb_acc = tb_AccDec_1;
                btn_move = btn_Move_1;
                btn_startMove = btn_ServoOn_1;
            }

            if (btn_startMove.Text == "Servo on")
            {
                MessageBox.Show("Please Servo on first!!");
                return;
            }

            mTargetPos[ID] = Convert.ToInt32(tb_targetPos.Text);
            mVelocity[ID] = Convert.ToUInt32(tb_velocity.Text);
            mAccDec[ID] = Convert.ToUInt32(tb_acc.Text);

            int[] actualPos = new int[TOTAL_AXIS];
            ret = NexCoeMotion.NEC_CoE402GetActualPosition(mAxisId[ID], ref actualPos[ID]);//讀取馬達實際位置
            if (ret != 0)
            {
                MessageBox.Show("transform failed");
            }

            int realactualPos = actualPos[ID];
            if (actualPos[ID] >= 0)
            {
                actualPos[ID] = ((actualPos[ID] % 129600) / 360);
            }
            else
            {
                actualPos[ID] = ((actualPos[ID] % 129600) / 360) + 360;
            }
            mTargetPos[ID] = (mTargetPos[ID] - actualPos[ID]) * 360 + realactualPos;

            if (btn_move.Text == "Move")
            {
                ret = NexCoeMotion.NEC_CoE402Halt(mAxisId[ID], 0);
                if (ret != 0) { MessageBox.Show("NEC_CoE402Halt failed" + ret.ToString()); return; }

                ret = NexCoeMotion.NEC_CoE402PtpA(mAxisId[ID], 0, mTargetPos[ID], mVelocity[ID], mAccDec[ID], mAccDec[ID]);
                if (ret != 0) { MessageBox.Show("NEC_CoE402PtpA failed" + ret.ToString()); return; }

                ret = NexCoeMotion.NEC_CoE402WaitTargetReached(mAxisId[ID]);

                btn_move.Text = "Stop";
            }
            else
            {
                ret = NexCoeMotion.NEC_CoE402Halt(mAxisId[ID], 1);//移動終止
                if (ret != 0) { MessageBox.Show("NEC_CoE402Halt failed" + ret.ToString()); return; }
                btn_move.Text = "Move";
            }
        }

        private void button3_Click(object sender, EventArgs e)   //Problem
        {
            sample = new ThreadStart(move);
            mysample = new Thread(move);
            mysample.Start();
        }

        private void move()
        {
            while (true)
            {
                togetherFun1();
            }
        }
        private void togetherFun1()
        {
            

            Image<Bgr, Byte> frame = cap.QueryFrame().ToImage<Bgr, Byte>(); // 去query該畫面
            Image<Hsv, byte> framehsv = frame.Convert<Hsv, byte>();   //fram to hsv                       //MessageBox.Show(frame.Size.ToString());
            Image<Gray, byte> grayImage = framehsv.Convert<Gray, byte>();//framhsv to gray
            //MessageBox.Show(image.Data[131, 104, 0].ToString() + "," + image.Data[131, 104, 1].ToString() + "," + image.Data[131, 104, 2].ToString());
            Gray thresholdValue = new Gray(150);
            //取得二值化影像
            //var thresholdImage = grayImage.ThresholdBinary(thresholdValue, new Gray(255));
            CvInvoke.InRange(framehsv, new ScalarArray(new MCvScalar(0, 120, 120)),
                       new ScalarArray(new MCvScalar(8, 255, 255)), grayImage);
            VectorOfVectorOfPoint contours = new VectorOfVectorOfPoint();
            CvInvoke.FindContours(grayImage, contours, null, RetrType.External, ChainApproxMethod.ChainApproxSimple);

            int count = contours.Size;
            for (int i = 0; i < count; i++)
            {
                using (VectorOfPoint contour = contours[i])
                {
                    // 使用 BoundingRectangle 取得框選矩形
                    Rectangle BoundingBox = CvInvoke.BoundingRectangle(contour);
                    CvInvoke.Rectangle(grayImage, BoundingBox, new MCvScalar(255), 3);
                    //CvInvoke.Rectangle(frame, BoundingBox, new MCvScalar(255), 3);
                    posXX = (BoundingBox.Left + BoundingBox.Right) / 2;
                    posYY = (BoundingBox.Bottom + BoundingBox.Top) / 2;
                    if ((BoundingBox.Right - BoundingBox.Left) * (BoundingBox.Bottom - BoundingBox.Top) >= 300)
                    {
                        posX = posXX;
                        posY = posYY;
                        //GSWei_Y = (640 - posX) / cam_pix_to_mm;
                        // GSWei_X = (480 - 80 - posY) / (cam_pix_to_mm * (1 - cam_rotation));
                        // CvInvoke.Circle(grayImage, new Point((int)posX,(int)posY), 3, new MCvScalar(255),3);
                        CvInvoke.Circle(frame, new Point((int)posX, (int)posY), 3, new MCvScalar(255), 3);
                        //CvInvoke.Circle(frame, new Point(cam_center_x, cam_center_y), 3, new MCvScalar(255), 3);//畫正中心
                        label6.Text = string.Format("X: {0}, Y: {1}", (int)posX, (int)posY);
                        //label7.Text = string.Format("GSWei_X: {0}, GSWei_Y: {1}", (int)GSWei_X, (int)GSWei_Y);                     
                        if ((640 - posX) * cam_pix_to_mm <= 150)
                        {
                            trigger = 1;
                        }
                        else
                        {
                            trigger = 0;
                        }
                    }
                }
            }

            //cameraprocess(posX, posY);
            // Convert from Camera reference system to Robot reference system
            // We suppose very small angle rotatios (less than 5 degrees) so we use the 
            // aproximation that sin cam_rotation = cam_rotation (in radians)
            // Camera X axis correspond to robot Y axis

            coordY = ((640 - posX) * cam_pix_to_mm) - 0;   // First we convert image coordinates to center of image
            coordX = ((480 - posY) * cam_pix_to_mm) - 0;

            //coordY = robot_center_y - coordY * cam_pix_to_mm;
            //coordX = robot_center_x - coordX * cam_pix_to_mm * (1 - cam_rotation);

            // Calculate speed and angle
            vectorX = (coordX - puckCoordX);
            vectorY = (coordY - puckCoordY);

            puckSpeedX = vectorX * 100 / time;  // speed in dm/ms (
            puckSpeedY = vectorY * 100 / time;
            //puckSpeed = sqrt(vectorX*vectorX + vectorY*vectorY)*1000.0/time;
            //puckDirection = atan2(vectorY,vectorX);
            puckOldCoordX = puckCoordX;
            puckOldCoordY = puckCoordY;
            puckCoordX = coordX;
            puckCoordY = coordY;

            // Noise detection, big vector...
            if ((vectorY < -100) || (vectorY > 100) || (vectorX > 100) || (vectorX < -100))
            {
                Console.WriteLine("NOISE");
                //return ;
            }
            // Its time to predict...
            // Based on actual position, speed and angle we need to know the future...
            // Posible impact?
            if (puckSpeedY < -10)
            {
                // Puck is comming...
                // We need to predict the puck position when it reaches our goal Y=0
                // slope formula: m = (y2-y1)/(x2-x1)
                if (vectorX == 0)  // To avoid division by 0
                    slope = 9999999;
                else
                    slope = (float)vectorY / (float)vectorX;
                // x = (y2-y1)/m + x1
                predict_y = defense_position;
                predict_x = (predict_y - coordY) / slope + coordX;//改了-284.71

                // puck has a bounce with side wall?
                if ((predict_x < 115) || (predict_x > 461))//將會碰撞
                {
                    // We start a new prediction
                    // Wich side?
                    if (predict_x < 115)//碰撞上方
                    {
                        //Left side. We calculare the impact point
                        bounce_x = 115.0f;
                    }
                    else//碰撞下方
                    {
                        //Right side. We calculare the impact point
                        bounce_x = 461.0f;
                    }
                    //bounce_y = (bounce_x - coordX)*slope + coordY;//原式
                    bounce_y = ((bounce_x - predict_x) * slope + 100);//改-284.71
                    bounce_pixX = 640 - (bounce_y / cam_pix_to_mm);
                    bounce_pixY = 480 - (bounce_x / (cam_pix_to_mm * (1 - cam_rotation)));
                    predict_time = (bounce_y - puckCoordY) * 100 / puckSpeedY;  // time until bouce
                                                                                // bounce prediction
                                                                                // Slope change
                    slope = -slope;
                    predict_y = defense_position;
                    predict_x = (predict_y - bounce_y) / slope + bounce_x;

                    if ((predict_x < 115) || (predict_x > 461))
                    {
                        // New bounce?? 
                        // We do nothing then...
                        //sprintf(tempStr2, "2B %d %d", bounce_x, bounce_y);
                        predict_x_old = -1;
                    }
                    else
                    {
                        // draw line
                        //~~~~~~~~cvLine(frameGrabbed, cvPoint(posX / 2, posY / 2), cvPoint(bounce_pixX / 2, bounce_pixY / 2), cvScalar(255, 0, 0), 2);
                        CvInvoke.Line(frame, new Point((int)posX, (int)posY), new Point((int)bounce_pixX, (int)bounce_pixY), new MCvScalar(255, 0, 0), 2);

                        // result average
                        if (predict_x_old != -1)
                            predict_x = (int)(predict_x_old + predict_x) >> 1;
                        predict_x_old = predict_x;
                        predict_time = predict_time + (predict_y - puckCoordY) * 100 / puckSpeedY;  // in ms
                                                                                                    //sprintf(tempStr2, "%d;%d %d %d t%d", coordX, coordY, predict_x, puckSpeedY, predict_time);
                                                                                                    //predict_pixX = cam_center_x - (predict_y - robot_center_y) / cam_pix_to_mm;
                                                                                                    //predict_pixY = cam_center_y - (predict_x - robot_center_x) / (cam_pix_to_mm * (1 - cam_rotation));
                        predict_pixX = 640 - (predict_y / cam_pix_to_mm);
                        predict_pixY = 480 - (predict_x / (cam_pix_to_mm * (1 - cam_rotation)));
                        // draw line
                        //~~~~~~~~~~cvLine(frameGrabbed, cvPoint(bounce_pixX / 2, bounce_pixY / 2), cvPoint(predict_pixX / 2, predict_pixY / 2), cvScalar(0, 255, 0), 2);
                        CvInvoke.Line(frame, new Point((int)bounce_pixX, (int)bounce_pixY), new Point((int)predict_pixX, (int)predict_pixY), new MCvScalar(0, 255, 0), 2);

                    }
                }
                else  // No bounce, direct impact
                {
                    // result average
                    if (predict_x_old != -1)
                        predict_x = (int)(predict_x_old + predict_x) >> 1;
                    predict_x_old = predict_x;

                    predict_time = (predict_y - puckCoordY) * 100 / puckSpeedY;  // in ms
                                                                                 //~~~~~~~~~sprintf(tempStr2, "%d;%d %d %d t%d", coordX, coordY, predict_x, puckSpeedY, predict_time);
                                                                                 // Convert impact prediction position to pixels (to show on image)
                                                                                 //predict_pixX = cam_center_x - (predict_y - robot_center_y) / cam_pix_to_mm;
                    predict_pixX = 640 - (predict_y / cam_pix_to_mm);
                    //predict_pixY = cam_center_y - (predict_x - robot_center_x) / (cam_pix_to_mm * (1 - cam_rotation));
                    predict_pixY = 480 - (predict_x / (cam_pix_to_mm * (1 - cam_rotation)));
                    // draw line
                    //~~~~~~~cvLine(frameGrabbed, cvPoint(posX / 2, posY / 2), cvPoint(predict_pixX / 2, predict_pixY / 2), cvScalar(0, 255, 0), 2);
                    CvInvoke.Line(frame, new Point((int)posX, (int)posY), new Point((int)predict_pixX, (int)predict_pixY), new MCvScalar(0, 255, 0), 2);
                }
            }
            else // Puck is moving slowly or to the other side
            {
                //~~~~~~~~~sprintf(tempStr2, "NO %d,%d %d", coordX, coordY, puckSpeedY);
                predict_x_old = -1;
            }

            GSWei_Y = predict_y;
            GSWei_X = predict_x;

           

            label7.Text = string.Format("GSWei_X: {0}, GSWei_Y: {1}", (int)GSWei_X, (int)GSWei_Y);

            CvInvoke.Line(frame, new Point((int)0, (int)(95 / cam_pix_to_mm)), new Point((int)640, (int)(95 / cam_pix_to_mm)), new MCvScalar(0, 0, 0), 2);
            CvInvoke.Line(frame, new Point((int)0, (int)(481 / cam_pix_to_mm)), new Point((int)640, (int)(481 / cam_pix_to_mm)), new MCvScalar(0, 0, 0), 2);

            CvInvoke.Line(frame, new Point((int)0, (int)(115 / cam_pix_to_mm)), new Point((int)640, (int)(115 / cam_pix_to_mm)), new MCvScalar(255, 255, 255), 2);
            CvInvoke.Line(frame, new Point((int)0, (int)(461 / cam_pix_to_mm)), new Point((int)640, (int)(461 / cam_pix_to_mm)), new MCvScalar(255, 255, 255), 2);
            CvInvoke.PutText(frame, "posX", new Point(10, 20), 0, 0.7, new MCvScalar(255, 255, 0), 2);

            CvInvoke.Line(grayImage, new Point((int)0, (int)(115 / cam_pix_to_mm)), new Point((int)640, (int)(115 / cam_pix_to_mm)), new MCvScalar(255, 255, 255), 2);
            pictureBox2.Image = grayImage.ToBitmap();

            frame.Dispose();
            framehsv.Dispose();
            grayImage.Dispose();

            //int x_asix= Convert.ToInt32(GSWei_X);
            //int y_asix = Convert.ToInt32(GSWei_Y);
            //togetherFun1(x_asix, y_asix);

            if (btn_ServoOn_0.Text == "Servo on" || btn_ServoOn_1.Text == "Servo on")//控制部分
            {
                MessageBox.Show("Please Servo on first!!");
                return;
            }

            if (button3.Text == "Move")
            {
                olda = a;
                oldb = b;
                

                if (GSWei_X > 90 && GSWei_X <= 120)
                    {


                    b = 122; 
                    a = 154;

                      //  MessageBox.Show("1");

                    }
                    else if (GSWei_X > 120 && GSWei_X <= 146)
                    {

                    b = 122;
                    a = 154;
                        //   MessageBox.Show("2");
                    }
                    else if (GSWei_X > 146 && GSWei_X <= 172)
                    {

                    b = 122;
                    a = 154;
                     //   MessageBox.Show("2");            
                    }
                    else if (GSWei_X > 172 && GSWei_X <= 198)
                    {

                    b = 111;
                    a = 136;
                        //   MessageBox.Show("2");
                    }
                    else if (GSWei_X > 198 && GSWei_X <= 226)
                    {

                    b = 105;
                    a = 126;
                      //  MessageBox.Show("3");
                    }
                    else if (GSWei_X > 226 && GSWei_X <= 252)
                    {

                    b = 95;
                    a = 112;
                        //   MessageBox.Show("2");
                    }
                    else if (GSWei_X > 252 && GSWei_X <= 278)
                    {

                    b = 87;
                    a = 107;
                      //  MessageBox.Show("4");
                    }
                    else if (GSWei_X > 278 && GSWei_X <= 304)
                    {

                    b = 82;
                    a = 97;
                        //   MessageBox.Show("2");
                    }
                    else if (GSWei_X > 304 && GSWei_X <= 330)
                    {

                    b = 70;
                    a = 87;
                       // MessageBox.Show("4");
                    }
                    else if (GSWei_X > 330 && GSWei_X <= 356)
                    {

                    b = 63;
                    a = 80;
                        //   MessageBox.Show("2");
                    }
                    else if (GSWei_X > 356 && GSWei_X <= 382)
                    {

                    b = 50;
                        a = 75;
                     //   MessageBox.Show("4");
                    }
                    else if (GSWei_X > 382 && GSWei_X <= 408)
                    {

                    b = 50;
                    a = 60;
                        //   MessageBox.Show("2");
                    }
                    else if (GSWei_X > 408 && GSWei_X <= 434)
                    {

                     b = 30;
                    a = 60;
                        
                       // MessageBox.Show("4");
                    }
                    else if (GSWei_X > 434 && GSWei_X <= 460)
                    {

                    b = 30;
                    a = 60;
                       
                        //   MessageBox.Show("2");
                    }
                    else if (GSWei_X > 460 && GSWei_X <= 530)
                    {

                    b = 30;
                    a = 60;
                        
                    //   MessageBox.Show("2");
                    
                }
                if (olda != a && oldb != b)
                {
                    RunLoop(a, b);
                }
                Thread.Sleep(100);                    
                
            }
            
        }

        private void RunLoop(int a,int b)
        {
            //計時檢測
            System.Diagnostics.Stopwatch sw = new System.Diagnostics.Stopwatch();//引用stopwatch物件
            sw.Reset();//碼表歸零
            sw.Start();//碼表開始計時
            int ret = 0;

            
            
            

            
            mTargetPos[0] = a;
            mVelocity[0] = Convert.ToUInt32(tb_Vel_0.Text);
            mAccDec[0] = Convert.ToUInt32(tb_AccDec_0.Text);
            mTargetPos[1] = b;
            mVelocity[1] = Convert.ToUInt32(tb_Vel_1.Text);
            mAccDec[1] = Convert.ToUInt32(tb_AccDec_1.Text);

            int[] actualPos = new int[TOTAL_AXIS];
            ret = NexCoeMotion.NEC_CoE402GetActualPosition(mAxisId[0], ref actualPos[0]);//讀取馬達實際位置
            if (ret != 0)
            {
                MessageBox.Show("transform failed");
            }
            ret = NexCoeMotion.NEC_CoE402GetActualPosition(mAxisId[1], ref actualPos[1]);//讀取馬達實際位置
            if (ret != 0)
            {
                MessageBox.Show("transform failed");
            }
            int realactualPos0 = actualPos[0];
            int realactualPos1 = actualPos[1];

            if (actualPos[0] >= 0)
            {
                actualPos[0] = ((actualPos[0] % 129600) / 360);
            }
            else
            {
                actualPos[0] = ((actualPos[0] % 129600) / 360) + 360;
            }

            mTargetPos[0] = (mTargetPos[0] - actualPos[0]) * 360 + realactualPos0;

            if (actualPos[1] >= 0)
            {
                actualPos[1] = ((actualPos[1] % 129600) / 360);
            }
            else
            {
                actualPos[1] = ((actualPos[1] % 129600) / 360) + 360;
            }

            mTargetPos[1] = (mTargetPos[1] - actualPos[1]) * 360 + realactualPos1;

            if (button3.Text == "Move")
            {
                ret = NexCoeMotion.NEC_CoE402Halt(mAxisId[0], 0);
                if (ret != 0) { MessageBox.Show("NEC_CoE402Halt failed" + ret.ToString()); return; }

                ret = NexCoeMotion.NEC_CoE402PtpA(mAxisId[0], 0, mTargetPos[0], mVelocity[0], mAccDec[0], mAccDec[0]);
                if (ret != 0) { MessageBox.Show("NEC_CoE402PtpA failed" + ret.ToString()); return; }

                ret = NexCoeMotion.NEC_CoE402Halt(mAxisId[1], 0);
                if (ret != 0) { MessageBox.Show("NEC_CoE402Halt failed" + ret.ToString()); return; }

                ret = NexCoeMotion.NEC_CoE402PtpA(mAxisId[1], 0, mTargetPos[1], mVelocity[1], mAccDec[1], mAccDec[1]);
                if (ret != 0) { MessageBox.Show("NEC_CoE402PtpA failed" + ret.ToString()); return; }
            }
            else
            {
                ret = NexCoeMotion.NEC_CoE402Halt(mAxisId[0], 1);//移動終止
                if (ret != 0) { MessageBox.Show("NEC_CoE402Halt failed" + ret.ToString()); return; }

                ret = NexCoeMotion.NEC_CoE402Halt(mAxisId[1], 1);//移動終止
                if (ret != 0) { MessageBox.Show("NEC_CoE402Halt failed" + ret.ToString()); return; }
                button3.Text = "Move";
            }
            sw.Stop();//碼錶停止                                    
            string result1 = sw.Elapsed.TotalMilliseconds.ToString();//印出所花費的總豪秒數                  
            richTextBox1.Text += (countt++) + " : " + result1;
            richTextBox1.Text += "\n";
        }

        private void button1_Click(object sender, EventArgs e)
        {
            for (int i = 0; i < mSlaveCnt; i++)
            {
                NexCoeMotion.NEC_CoE402Home(mAxisId[i], 0);
            }
        }

        private void GroupBox1_Enter(object sender, EventArgs e)
        {

        }

        private void pictureBox1_Click(object sender, EventArgs e)
        {

        }

        private void pictureBox1_MouseMove(object sender, MouseEventArgs e)
        {
            //label6.Text = string.Format("X: {0}, Y: {1}", e.X, e.Y);
        }

        private void label6_Click(object sender, EventArgs e)
        {

        }

        private void tb_Vel_1_TextChanged(object sender, EventArgs e)
        {

        }

        private void tb_AccDec_1_TextChanged(object sender, EventArgs e)
        {

        }

        private void tb_AccDec_0_TextChanged(object sender, EventArgs e)
        {

        }

        private void pictureBox3_Click(object sender, EventArgs e)
        {

        }

        private void tb_ActualPos_0_TextChanged(object sender, EventArgs e)
        {

        }

        private void tb_SlaveState_0_TextChanged(object sender, EventArgs e)
        {

        }
    }   
}
