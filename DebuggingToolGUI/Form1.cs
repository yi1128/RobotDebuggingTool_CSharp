using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace DebuggingToolGUI
{
    public partial class DebuggingToolSW : Form
    {
        /* Status Check Variable */
        public bool StatusMainTimerStart = false;
        public bool StatusCommunicationConnect = false;
        public bool StatusServoOn = false;
        public bool StatusHoming = false;
        public bool StatusReady = false;
        public bool StatusGravityCompensator = false;
        public bool StatusJointMode = false;
        public bool StatusCartesianMode = false;
        public bool StatusJointPositionPIDgainSet = false;
        public bool StatusJointTorquePIDgainSet = false;
        public bool StatusJointConstantParamSet = false;
        public bool StatusJointGravityNFrictionParamSet = false;
        public bool StatusJointTrajectoryParamSet = false;
        public bool StatusCartesianPositionPIDgainSet = false;
        public bool StatusCartesianTrajectoryParamSet = false;
        /* End Status Check Variable */


        /* Initailize Image Box */
        Image StatusOnImage = Image.FromFile(Application.StartupPath + @"\\Status_On.bmp");
        Image StatusOffImage = Image.FromFile(Application.StartupPath + @"\\Status_Off.bmp");
        /* End Initailize Image Box */


        public DebuggingToolSW()
        {
            InitializeComponent();
            /* Nonactivation GroupBox */
            LoginGroupBox.Enabled = true;
            StateGroupBox.Enabled = false;
            LoggingSystemgroupBox.Enabled = false;
            ParameterSetgroupBox.Enabled = false;
            AbsoluteEnocdergroupBox.Enabled = false;
            IncrementalEncodergroupBox.Enabled = false;
            MainTapControl.Enabled = false;
            GCbutton.Enabled = false;
            JointModebutton.Enabled = false;
            CartesianModebutton.Enabled = false;
            ServoOnButton.Enabled = false;
            Homingbutton.Enabled = false;
            Readybutton.Enabled = false;
            JointPositionSetgroupBox.Enabled = false;
            CartesianPositionSetgroupBox.Enabled = false;
            /* End Nonactivation GroupBox */

            /* Logging Status Initialize */
            LoggingpictureBox.Image = StatusOffImage;
            /* End Logging Status Initialize */

            /* Joint Enable Picture Box */
            Joint1pictureBox.Image = StatusOffImage;
            Joint2pictureBox.Image = StatusOffImage;
            Joint3pictureBox.Image = StatusOffImage;
            Joint4pictureBox.Image = StatusOffImage;
            Joint5pictureBox.Image = StatusOffImage;
            Joint6pictureBox.Image = StatusOffImage;
            Joint7pictureBox.Image = StatusOffImage;
            Joint8pictureBox.Image = StatusOffImage;
            /* End Joint Enable Picture Box */

            /* Status Image Initialize */
            AuxPowerpictureBox.Image = StatusOffImage;
            CartesianMovingpictureBox.Image = StatusOffImage;
            CartesianReachedpictureBox.Image = StatusOffImage;
            ConnectCompletepictureBox.Image = StatusOffImage;
            ConnectpictureBox.Image = StatusOffImage;
            GCpictureBox.Image = StatusOffImage;
            JointMovingpictureBox.Image = StatusOffImage;
            JointReachedpictureBox.Image = StatusOffImage;
            MainPowerpictureBox.Image = StatusOffImage;
            ParametersReadpictureBox.Image = StatusOffImage;
            ParametersSavepictureBox.Image = StatusOffImage;
            PositionModepictureBox.Image = StatusOffImage;
            TorqueModepictureBox.Image = StatusOffImage;
            /* End Status Image Initialize */

            /* Communication Select */
            CommunicationComboBox.Items.Add("ADS");
            CommunicationComboBox.Items.Add("TCP/IP");
            CommunicationComboBox.Items.Add("UDP/IP");
            CommunicationComboBox.SelectedIndex = 1;
            /* End Communication Select */

            /* Number of Joint Set */
            NumJointcomboBox.Items.Add("1");
            NumJointcomboBox.Items.Add("2");
            NumJointcomboBox.Items.Add("3");
            NumJointcomboBox.Items.Add("4");
            NumJointcomboBox.Items.Add("5");
            NumJointcomboBox.Items.Add("6");
            NumJointcomboBox.Items.Add("7");
            NumJointcomboBox.Items.Add("8");
            NumJointcomboBox.Items.Add("9");
            NumJointcomboBox.Items.Add("10");
            NumJointcomboBox.SelectedIndex = 0;
            /* End Number of Joint Set */

            /* Timer Start */
            MainTimer.Start();
            /* End Timer Start */

        }

        public void CheckHomingReady()
        {
            /* Status Setting Check */
            if (StatusJointPositionPIDgainSet == true &&
                StatusJointTorquePIDgainSet == true &&
                StatusJointTrajectoryParamSet == true &&
                StatusJointConstantParamSet == true &&
                StatusJointGravityNFrictionParamSet == true)
            {
                /* Activation Homing Button */
                Homingbutton.Enabled = true;
                /* End Activation Homing Button */
            }
            /* End Status Setting Check */
        }

        public void CheckCartesianModeReady()
        {

            /* Status Setting Check */
            if (StatusHoming == true && StatusReady == true)
            {
                if (StatusCartesianPositionPIDgainSet == true &&
                StatusCartesianTrajectoryParamSet == true)
                {
                    /* Activation Cartesian Control Button */
                    CartesianModebutton.Enabled = true;
                    CartesianPositionSetgroupBox.Enabled = true;
                    /* End Activation Cartesian Control Button */
                }
            }
            /* End Status Setting Check */
        }

        private void groupBox1_Enter(object sender, EventArgs e)
        {

        }

        private void label6_Click(object sender, EventArgs e)
        {

        }

        private void Default_Click(object sender, EventArgs e)
        {

        }

        private void label38_Click(object sender, EventArgs e)
        {

        }

        private void label37_Click(object sender, EventArgs e)
        {

        }

        private void label138_Click(object sender, EventArgs e)
        {

        }
        const int ADS = 0;
        const int TCP = 1;
        const int UDP = 2;
        private void Connectbutton_Click(object sender, EventArgs e)
        {
            switch(CommunicationComboBox.SelectedIndex)
            {
                case ADS:
                    {
                        /* Main Timer Start */
                        StatusMainTimerStart = true;
                        StatusCommunicationConnect = true;
                        
                        break;
                    }
                case TCP:
                    {
                        /* Main Timer Start */
                        StatusMainTimerStart = true;
                        StatusCommunicationConnect = true;
                        
                        break;
                    }
                case UDP:
                    {
                        /* Main Timer Start */
                        StatusMainTimerStart = true;
                        StatusCommunicationConnect = true;
                        break;
                    }   
                default:
                    MessageBox.Show("Please Select Communication Method");
                    StatusCommunicationConnect = false;
                    break;
            }

            
            if(StatusCommunicationConnect == true)
            {
                /* Activation GroupBox */
                StateGroupBox.Enabled = true;
                ServoOnButton.Enabled = true;
                /* End Activation GroupBox */
            }
        }

        private void MainTimer_Tick(object sender, EventArgs e)
        {

            ErrorCodetextBox.Text = "Checking....";

           if (StatusMainTimerStart == true)
            {
                /* Status Setting Check */
                CheckCartesianModeReady();
                /* End Status Setting Check */      
            }
        }
        public static bool ServoOnCheck = false;
        private void ServoOnButton_Click(object sender, EventArgs e)
        {

            /* Servo On Behavior */
            
            StatusServoOn = true;
            /* End Servo On Behavior */
            
            if(StatusServoOn == true)
            {
                /* ServoOn Button Color Change */
                ServoOnButton.ForeColor = System.Drawing.Color.Black;
                ServoOnButton.BackColor = System.Drawing.Color.YellowGreen;
                /* End ServoOn Button Color Change */

                IncrementalEncodergroupBox.Enabled = true;
                AbsoluteEnocdergroupBox.Enabled = true;
                MainTapControl.Enabled = true;
            }
        }

        private void SetJointPosPIDGainbutton_Click(object sender, EventArgs e)
        {
            /* Joint Position PID Gain Set */

            /* End Joint Position PID Gain Set */
            StatusJointPositionPIDgainSet = true;

            /* Status Setting Check */
            CheckHomingReady();
            /* End Status Setting Check */
        }

        private void SetJointToqPIDGainbutton_Click(object sender, EventArgs e)
        {

            /* Joint Torque PID Gain Set */

            /* End Joint Torque PID Gain Set */
            StatusJointTorquePIDgainSet = true;

            /* Status Setting Check */
            CheckHomingReady();
            /* End Status Setting Check */
        }

        private void JointConstantParamSetbutton_Click(object sender, EventArgs e)
        {
            /* Joint Constant Param Gain Set */

            /* End Joint Constant Param Gain Set */
            StatusJointConstantParamSet = true;

            /* Status Setting Check */
            CheckHomingReady();
            /* End Status Setting Check */
        }

        private void GCnFricParamSetbutton_Click(object sender, EventArgs e)
        {
            /* Gravity Compensator and Friction Compensator Parameters Set */

            /* End Gravity Compensator and Friction Compensator Parameters Set */
            StatusJointGravityNFrictionParamSet = true;

            /* Status Setting Check */
            CheckHomingReady();
            /* End Status Setting Check */
        }

        private void JointTrajetorySetbutton_Click(object sender, EventArgs e)
        {
            /* Joint Trajectory Parameters Set */

            /* End Joint Trajectory Parameters Set */
            StatusJointTrajectoryParamSet = true;

            /* Status Setting Check */
            CheckHomingReady();
            /* End Status Setting Check */
        }

        private void CartesianPIDSetbutton_Click(object sender, EventArgs e)
        {
            /* Cartesian Position PID Gain Set */

            /* End Cartesian Position PID Gain Set */
            StatusCartesianPositionPIDgainSet = true;

            /* Status Setting Check */
            CheckHomingReady();
            /* End Status Setting Check */
        }

        private void CartesianTrajSetbutton_Click(object sender, EventArgs e)
        {
            /* Cartesian Trajectory Parameters Set */

            /* End Cartesian Trajectory Parameters Set */
            StatusCartesianTrajectoryParamSet = true;

            /* Status Setting Check */
            CheckHomingReady();
            /* End Status Setting Check */
        }

        private void Homingbutton_Click(object sender, EventArgs e)
        {
            /* Homing Start */
            PositionModepictureBox.Image = StatusOnImage;
            TorqueModepictureBox.Image = StatusOffImage;

            /* End Homing Start */
            StatusHoming = true;

            if(StatusHoming == true)
            {
                /* ServoOn Button Color Change */
                Homingbutton.ForeColor = System.Drawing.Color.Black;
                Homingbutton.BackColor = System.Drawing.Color.YellowGreen;
                /* End ServoOn Button Color Change */
            }

            /* Activation Ready Button */
            Readybutton.Enabled = true;
            /* End Activation Ready Button */
        }

        private void Readybutton_Click(object sender, EventArgs e)
        {
            /* Ready */

            /* End Ready */
            StatusReady = true;

            if (StatusReady == true)
            {
                /* ServoOn Button Color Change */
                Readybutton.ForeColor = System.Drawing.Color.Black;
                Readybutton.BackColor = System.Drawing.Color.YellowGreen;
                /* End ServoOn Button Color Change */
            }

            /* Main Timer Start */
            StatusMainTimerStart = true;
            /* Actiavtion Joint Set & Joint Mode & Gravity Compensation Button */
            JointPositionSetgroupBox.Enabled = true;
            JointModebutton.Enabled = true;
            GCbutton.Enabled = true;
            GCbutton_Click(null, null);
            /* End Activation Joint Set & Joint Mode & Gravity Compensation Button */
        }
        public static bool GCbuttonCheck = false;
        private void GCbutton_Click(object sender, EventArgs e)
        {


            /* GCbutton Color Change */
            if(GCbuttonCheck == false)
            {
                GCbutton.ForeColor = System.Drawing.Color.Black;
                GCbutton.BackColor = System.Drawing.Color.YellowGreen;

                /* Status Light */
                TorqueModepictureBox.Image = StatusOnImage;
                PositionModepictureBox.Image = StatusOffImage;
                /* End Status Light */
                GCbuttonCheck = true;
            }
            /* End GCbutton Color Change */
        }
    }
}
