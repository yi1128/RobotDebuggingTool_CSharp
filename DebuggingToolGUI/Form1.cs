using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO;
using System.Runtime.InteropServices;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;
using System.Diagnostics;


using log4net;
using log4net.Config;

namespace DebuggingToolGUI
{
    

    public partial class DebuggingToolSW : Form
    {
        /* Dof Select */
        public int LegDofSelect = 4;
        public int CartesianDofSelect = 3;

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
        public bool StatusLogging = false;
        public bool StatusThreadStop = true;
        public bool StatusCommunicationStart = false;
        /* End Status Check Variable */

        /* Parameters Set State */
        public const int JOINT_PARAMETER_SET = 0;
        public const int JOINT_TRAJECTORY_SET = 1;
        public const int CARTESIAN_PARAMETER_SET = 2;
        public const int CARTESIAN_TRAJECTORY_SET = 3;
        public const int JOINT_TARGET_SET = 4;
        public const int CARTESIAN_TARGET_SET = 5;
        public const int SERVER_SYSTEM_DATA = 6;
        public const int NONE = 7;
        /* End Parameters Set State */

        /* Communication State */
        public const int REGISTRATION = 0;
        public const int REGISTRATION_COMPLETE = 1;
        public const int SERVO_ON = 2;
        public const int SERVO_ON_COMPLETE = 3;
        public const int TUNNING_STATE = 4;
        public const int TUNNING_STATE_COMPLETE = 5;
        public const int HOMING = 6;
        public const int HOMING_COMPLETE = 7;
        public const int FREE_STATE = 8;
        public const int READY_STATE = 128;
        /* End Communication State */

        /* Control Mode Select */
        public const int NONE_MODE = 0;
        public const int GRAVITY_MODE = 1;
        public const int JOINT_MODE = 2;
        public const int CARTESIAN_MODE = 3;
        public const int GRAVITY_WITH_JOINT_MODE = 4;
        public const int GRAVITY_WITH_CARTESIAN_MODE = 5;
        /* End Control Mode Select */

        /* Initailize Image Box */
        Image StatusOnImage = Image.FromFile(Application.StartupPath + @"\\Status_On.bmp");
        Image StatusOffImage = Image.FromFile(Application.StartupPath + @"\\Status_Off.bmp");
        /* End Initailize Image Box */

        /* Mutex declaration */
        private static Mutex tcpIpMutex;
        /* End Mutex declaration */

        /* Basic Communication Structure */
        [StructLayout(LayoutKind.Sequential, Pack = 1), Serializable]
        public struct MsgState
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
            public byte[] packetType;
            public UInt16 commState;
            public UInt16 payloadSize;
            public byte controlMode;
        };
        public static MsgState MsgStateSend;
        public static MsgState MsgStateRecv;

        public void MsgStateSettingInit()
        {
            MsgStateSend.packetType = new byte[2];
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1), Serializable]
        public struct AxisServerData
        {
            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = Axis_Size)]
            public float[] actualMotorPosition;
            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = Axis_Size)]
            public float[] actualLinkPosition;
            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = Axis_Size)]
            public float[] actualMotorVelocity;
            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = Axis_Size)]
            public float[] actualLinkVelocity;
            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = Axis_Size)]
            public float[] actualCurrent;
            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = Axis_Size)]
            public float[] targetPosition;
            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = Axis_Size)]
            public float[] targetVelocity;
            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = Axis_Size)]
            public float[] targetCurrent;
            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = Axis_Size)]
            public Int32[] modeOfOperation;
            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = Axis_Size)]
            public Int32[] statusword;
        };

        [StructLayout(LayoutKind.Sequential, Pack = 1), Serializable]
        public struct ServerSystemData
        {
            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = 1)]
            public Int32[] cnt;
            public Int32 logCnt;
            public Int32 gravityMode;
            public Int32 targetReached;
            public Int32 controlMode;// 0: Gravity mode, 1:Joint Mode, 2:Cartesian Mode

            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
            public float[] cartesianTargetPose;
            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
            public float[] cartesianCurrentPose;
            
            // Position Mode
            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
            public float[] targetTrajectoryTime;
            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
            public float[] targetTrajectoryAcc;

            // Module Data
            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = 10)]
            public AxisServerData[] moduleData;
        };
        public static ServerSystemData ServerDataSave;
        public static ServerSystemData ServerData;
        /* End Basic Communication Structure */

        public void ServerSystemDataInit(int Axis_Size)
        {
            /* ServerData Init */
            ServerData.cnt = new Int32[1];

            ServerData.cartesianTargetPose = new float[6];
            ServerData.cartesianCurrentPose = new float[6];

            ServerData.targetTrajectoryTime = new float[6];
            ServerData.targetTrajectoryAcc = new float[6];

            ServerData.moduleData = new AxisServerData[1];
            ServerData.moduleData[0].actualMotorPosition = new float[Axis_Size];
            ServerData.moduleData[0].actualLinkPosition = new float[Axis_Size];
            ServerData.moduleData[0].actualMotorVelocity = new float[Axis_Size];
            ServerData.moduleData[0].actualLinkVelocity = new float[Axis_Size];
            ServerData.moduleData[0].actualCurrent = new float[Axis_Size];
            ServerData.moduleData[0].targetPosition = new float[Axis_Size];
            ServerData.moduleData[0].targetVelocity = new float[Axis_Size];
            ServerData.moduleData[0].targetCurrent = new float[Axis_Size];
            ServerData.moduleData[0].modeOfOperation = new Int32[Axis_Size];
            ServerData.moduleData[0].statusword = new Int32[Axis_Size];

            /* ServerDataSave Init */
            ServerDataSave.cnt = new Int32[1];

            ServerDataSave.cartesianTargetPose = new float[3];
            ServerDataSave.cartesianCurrentPose = new float[3];

            ServerDataSave.targetTrajectoryTime = new float[3];
            ServerDataSave.targetTrajectoryAcc = new float[3];

            ServerDataSave.moduleData = new AxisServerData[1];
            ServerDataSave.moduleData[0].actualMotorPosition = new float[Axis_Size];
            ServerDataSave.moduleData[0].actualLinkPosition = new float[Axis_Size];
            ServerDataSave.moduleData[0].actualMotorVelocity = new float[Axis_Size];
            ServerDataSave.moduleData[0].actualLinkVelocity = new float[Axis_Size];
            ServerDataSave.moduleData[0].actualCurrent = new float[Axis_Size];
            ServerDataSave.moduleData[0].targetPosition = new float[Axis_Size];
            ServerDataSave.moduleData[0].targetVelocity = new float[Axis_Size];
            ServerDataSave.moduleData[0].targetCurrent = new float[Axis_Size];
            ServerDataSave.moduleData[0].modeOfOperation = new Int32[Axis_Size];
            ServerDataSave.moduleData[0].statusword = new Int32[Axis_Size];
        }


        /* Parameters Tunning & Target Structure */
        [StructLayout(LayoutKind.Sequential, Pack = 1), Serializable]
        public struct JointParameterSettingStruct
        {
            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = Axis_Size)]
            public float[] jointPositionPgain;
            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = Axis_Size)]
            public float[] jointPositionIgain;
            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = Axis_Size)]
            public float[] jointPositionDgain;

            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = Axis_Size)]
            public float[] jointTorquePgain;
            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = Axis_Size)]
            public float[] jointTorqueIgain;
            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = Axis_Size)]
            public float[] jointTorqueDgain;

            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = Axis_Size)]
            public float[] jointConstantEfficiency;
            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = Axis_Size)]
            public float[] jointConstantTorque;
            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = Axis_Size)]
            public float[] jointConstantSpring;

            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = Axis_Size)]
            public float[] jointGravityGain;
            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = Axis_Size)]
            public float[] jointCurrentGain;
            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = Axis_Size)]
            public float[] jointFrictionGain;
        };
        public static JointParameterSettingStruct jParam;
        public static JointParameterSettingStruct jParamSet;
        public static JointParameterSettingStruct jParamGet;
        /* End Parameters Tunning & Target Structure */

        public void JointParameterSettingInit(int Axis_Size)
        {
            /* Joint Parameters Init */
            /* Save Data  */
            jParam.jointPositionPgain = new float[8];
            jParam.jointPositionIgain = new float[8];
            jParam.jointPositionDgain = new float[8];

            jParam.jointTorquePgain = new float[8];
            jParam.jointTorqueIgain = new float[8];
            jParam.jointTorqueDgain = new float[8];

            jParam.jointConstantEfficiency = new float[8];
            jParam.jointConstantTorque = new float[8];
            jParam.jointConstantSpring = new float[8];

            jParam.jointGravityGain = new float[8];
            jParam.jointCurrentGain = new float[8];
            jParam.jointFrictionGain = new float[8];

            jTraj.jointTrajectoryAcc = new float[8];
            jTraj.jointTrajectoryTime = new float[8];

            /* Send Data  */
            jParamSet.jointPositionPgain = new float[Axis_Size];
            jParamSet.jointPositionIgain = new float[Axis_Size];
            jParamSet.jointPositionDgain = new float[Axis_Size];

            jParamSet.jointTorquePgain = new float[Axis_Size];
            jParamSet.jointTorqueIgain = new float[Axis_Size];
            jParamSet.jointTorqueDgain = new float[Axis_Size];

            jParamSet.jointConstantEfficiency = new float[Axis_Size];
            jParamSet.jointConstantTorque = new float[Axis_Size];
            jParamSet.jointConstantSpring = new float[Axis_Size];

            jParamSet.jointGravityGain = new float[Axis_Size];
            jParamSet.jointCurrentGain = new float[Axis_Size];
            jParamSet.jointFrictionGain = new float[Axis_Size];

            jTrajSet.jointTrajectoryAcc = new float[Axis_Size];
            jTrajSet.jointTrajectoryTime = new float[Axis_Size];

            /* Recv Data */
            jParamGet.jointPositionPgain = new float[Axis_Size];
            jParamGet.jointPositionIgain = new float[Axis_Size];
            jParamGet.jointPositionDgain = new float[Axis_Size];

            jParamGet.jointTorquePgain = new float[Axis_Size];
            jParamGet.jointTorqueIgain = new float[Axis_Size];
            jParamGet.jointTorqueDgain = new float[Axis_Size];

            jParamGet.jointConstantEfficiency = new float[Axis_Size];
            jParamGet.jointConstantTorque = new float[Axis_Size];
            jParamGet.jointConstantSpring = new float[Axis_Size];

            jParamGet.jointGravityGain = new float[Axis_Size];
            jParamGet.jointCurrentGain = new float[Axis_Size];
            jParamGet.jointFrictionGain = new float[Axis_Size];

            jTrajGet.jointTrajectoryAcc = new float[Axis_Size];
            jTrajGet.jointTrajectoryTime = new float[Axis_Size];
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1), Serializable]
        public struct JointTrajectorySetStruct
        {
            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = Axis_Size)]
            public float[] jointTrajectoryTime;
            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = Axis_Size)]
            public float[] jointTrajectoryAcc;
        };
        public static JointTrajectorySetStruct jTraj;
        public static JointTrajectorySetStruct jTrajSet;
        public static JointTrajectorySetStruct jTrajGet;

        [StructLayout(LayoutKind.Sequential, Pack = 1), Serializable]
        public struct CartesianParameterSettingStruct
        {
            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
            public float[] cartesianPositionPgain;
            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
            public float[] cartesianPositionIgain;
            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
            public float[] cartesianPositionDgain;

            
        };
        public static CartesianParameterSettingStruct cParam;
        public static CartesianParameterSettingStruct cParamSet;
        public static CartesianParameterSettingStruct cParamGet;

        public void CartesianParameterSettingInit()
        {
            /* Data Save */
            cParam.cartesianPositionPgain = new float[6];
            cParam.cartesianPositionIgain = new float[6];
            cParam.cartesianPositionDgain = new float[6];

            cTraj.cartesianTrajectoryTime = new float[6];
            cTraj.cartesianTrajectoryAcc = new float[6];


            /* Send Data */
            cParamSet.cartesianPositionPgain = new float[CartesianDofSelect];
            cParamSet.cartesianPositionIgain = new float[CartesianDofSelect];
            cParamSet.cartesianPositionDgain = new float[CartesianDofSelect];

            cTrajSet.cartesianTrajectoryTime = new float[CartesianDofSelect];
            cTrajSet.cartesianTrajectoryAcc = new float[CartesianDofSelect];
            
            /* Recv Data */
            cParamGet.cartesianPositionPgain = new float[CartesianDofSelect];
            cParamGet.cartesianPositionIgain = new float[CartesianDofSelect];
            cParamGet.cartesianPositionDgain = new float[CartesianDofSelect];

            cTrajGet.cartesianTrajectoryTime = new float[CartesianDofSelect];
            cTrajGet.cartesianTrajectoryAcc = new float[CartesianDofSelect];
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1), Serializable]
        public struct CartesianTrajectorySetStruct
        {
            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
            public float[] cartesianTrajectoryTime;
            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
            public float[] cartesianTrajectoryAcc;
        };
        public static CartesianTrajectorySetStruct cTraj;
        public static CartesianTrajectorySetStruct cTrajSet;
        public static CartesianTrajectorySetStruct cTrajGet;

        [StructLayout(LayoutKind.Sequential, Pack = 1), Serializable]
        public struct JointTargetStruct
        {
            //[MarshalAs(UnmanagedType.ByValArray, SizeConst = Axis_Size)]
            public float[] jointTarget;
        };
        public static JointTargetStruct jTarget;
        public static JointTargetStruct jTargetSet;
        public static JointTargetStruct jTargetGet;
        public void JointTargetInit(int Axis_Size)
        {
            /* Data Save */
            jTarget.jointTarget = new float[8];
            /* Send Data */
            jTargetSet.jointTarget = new float[Axis_Size];
            /* Recv Data */
            jTargetGet.jointTarget = new float[Axis_Size];
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1), Serializable]
        public struct CartesianTargetStruct
        {
            public float pX;
            public float pY;
            public float pZ;
            public float rX;
            public float rY;
            public float rZ;
        };
        public static CartesianTargetStruct cTarget;
        public static CartesianTargetStruct cTargetSet;
        public static CartesianTargetStruct cTargetGet;
        /* End Parameters Tunning & Target Structure */

        /* Text Box Update Invoke */
        private void UpdateTextBox(TextBox textBox, string data)
        {
            if (textBox.InvokeRequired)
            {
                // 작업쓰레드인 경우
                textBox.BeginInvoke(new Action(() =>
                {
                    textBox.Text = data;
                }));
            }
            else
            {
                // UI 쓰레드인 경우
                textBox.Text = data;
            }
        }

        void AddListBox(ListBox listbox, String text)
        {
            if (listbox.InvokeRequired)
            {
                listbox.BeginInvoke(new Action(() => {
                    if (listbox.Items.Count > 1000)
                    {
                        listbox.Items.Clear();
                    }
                    listbox.Items.Add(text);
                    listbox.SelectedIndex = listbox.Items.Count - 1;
                }));
            }
            else
            {
                if (listbox.Items.Count > 1000)
                {
                    listbox.Items.Clear();
                }
                listbox.Items.Add(text);
                listbox.SelectedIndex = listbox.Items.Count - 1;
            }
        }

        /* Structure Information To Byte Array Convert Function */
        public static byte[] StructToByte(object obj)
        {
            int nSize = Marshal.SizeOf(obj);
            byte[] arr = new byte[nSize];
            IntPtr ptr = Marshal.AllocHGlobal(nSize);
            Marshal.StructureToPtr(obj, ptr, true);
            Marshal.Copy(ptr, arr, 0, nSize);
            Marshal.FreeHGlobal(ptr);
            return arr;
        }

        public static T ByteToStruct<T>(byte[] buffer) where T : struct
        {
            int size = Marshal.SizeOf(typeof(T));
            if (size > buffer.Length)
            {
                throw new Exception();
            }

            IntPtr ptr = Marshal.AllocHGlobal(size);
            Marshal.Copy(buffer, 0, ptr, size);
            T obj = (T)Marshal.PtrToStructure(ptr, typeof(T));
            Marshal.FreeHGlobal(ptr);
            return obj;
        }


        /* String to byte Array Convert Function */
        public static byte[] StringToByte(string obj, int dstSize)
        {
            int nSize = obj.Length;
            byte[] arr = new byte[dstSize];
            IntPtr ptr = Marshal.AllocHGlobal(nSize);
            Buffer.BlockCopy(Encoding.ASCII.GetBytes(obj), 0, arr, 0, nSize);

            Marshal.FreeHGlobal(ptr);
            return arr;
        }

        /* Log4net Initialize */
        //private static readonly log4net.ILog log = log4net.LogManager.GetLogger(typeof(DebuggingToolSW));
        private static readonly log4net.ILog log = log4net.LogManager.GetLogger(System.Reflection.MethodBase.GetCurrentMethod().DeclaringType);
        public DebuggingToolSW()
        {
            InitializeComponent();
            /* Log4net XML Configurator Initialize */
            //XmlConfigurator.Configure(new FileInfo("log4net.xml"));
            
            /* Nonactivation GroupBox */
            LoginGroupBox.Enabled = true;
            StateGroupBox.Enabled = false;
            LoggingSystemgroupBox.Enabled = false;
            ParameterSetgroupBox.Enabled = false;
            AbsoluteEnocdergroupBox.Enabled = false;
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
            NumJointcomboBox.Items.Add("0");
            NumJointcomboBox.Items.Add("1");
            NumJointcomboBox.Items.Add("2");
            NumJointcomboBox.Items.Add("3");
            NumJointcomboBox.Items.Add("4");
            NumJointcomboBox.Items.Add("5");
            NumJointcomboBox.Items.Add("6");
            NumJointcomboBox.Items.Add("7");
            NumJointcomboBox.Items.Add("8");
            NumJointcomboBox.SelectedIndex = 0;
            /* End Number of Joint Set */

            /* Timer Start */
            MainTimer.Start();
            /* End Timer Start */

            /* Msg State Init */
            MsgStateSettingInit();

            /* MsgStateSend */
            BitArray SendPacketType1 = new BitArray(8);
            BitArray SendPacketType2 = new BitArray(new byte[] { 0 });
            SendPacketType1.Set(0, false);
            SendPacketType1.Set(1, false);
            SendPacketType1.Set(2, false);
            SendPacketType1.Set(3, false);
            SendPacketType1.Set(4, false);
            SendPacketType1.Set(5, false);
            SendPacketType1.Set(6, false);
            SendPacketType1.Set(7, false);
            MsgStateSend.packetType[0] = GetByteFromBitArray(SendPacketType1);
            MsgStateSend.packetType[1] = GetByteFromBitArray(SendPacketType2);
            MsgStateSend.commState = Convert.ToUInt16(REGISTRATION);
            MsgStateSend.controlMode = Convert.ToByte(NONE_MODE);
            MsgStateSend.payloadSize = 0;

            /* MsgStateRecv */
            BitArray RecvPacketType1 = new BitArray(8);
            BitArray RecvPacketType2 = new BitArray(new byte[] { 0 });
            RecvPacketType1.Set(0, false);
            RecvPacketType1.Set(1, false);
            RecvPacketType1.Set(2, false);
            RecvPacketType1.Set(3, false);
            RecvPacketType1.Set(4, false);
            RecvPacketType1.Set(5, false);
            RecvPacketType1.Set(6, false);
            RecvPacketType1.Set(7, false);
            MsgStateRecv.packetType[0] = GetByteFromBitArray(RecvPacketType1);
            MsgStateRecv.packetType[1] = GetByteFromBitArray(RecvPacketType2);
            MsgStateRecv.commState = Convert.ToUInt16(REGISTRATION);
            MsgStateRecv.controlMode = Convert.ToByte(NONE_MODE);
            MsgStateRecv.payloadSize = 0;
            /* End Msg State Init */

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

        private byte GetByteFromBitArray(BitArray bitArray)
        {

            if (bitArray.Length > 8)
                throw new ArgumentException("Argument length shall be at most 32 bits.");

            byte[] array = new byte[1];
            bitArray.CopyTo(array, 0);
            return array[0];
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

        IPAddress ipAddr;
        TcpClient tcpClient;

        private void TcpClientRun(String ipStr, int port)
        {
            AddListBox(LoglistBox, "TCP Thread Running");

            int errCnt = 0;

            if (IPAddress.TryParse(ipStr, out ipAddr))
            {
                while (true)
                {
                    /* Mutex Start */
                    tcpIpMutex.WaitOne();

                    for (int i=0;i<LegDofSelect;i++)
                    {
                        /* Joint Parameters Set from Save Data */
                        jParamSet.jointPositionPgain[i] = jParam.jointPositionPgain[i];
                        jParamSet.jointPositionIgain[i] = jParam.jointPositionPgain[i];
                        jParamSet.jointPositionDgain[i] = jParam.jointPositionDgain[i];

                        jParamSet.jointTorquePgain[i] = jParam.jointTorquePgain[i];
                        jParamSet.jointTorqueIgain[i] = jParam.jointTorqueIgain[i];
                        jParamSet.jointTorqueDgain[i] = jParam.jointTorqueDgain[i];

                        jParamSet.jointConstantEfficiency[i] = jParam.jointConstantEfficiency[i];
                        jParamSet.jointConstantTorque[i] = jParam.jointConstantTorque[i];
                        jParamSet.jointConstantSpring[i] = jParam.jointConstantSpring[i];

                        jParamSet.jointGravityGain[i] = jParam.jointGravityGain[i];
                        jParamSet.jointCurrentGain[i] = jParam.jointCurrentGain[i];
                        jParamSet.jointFrictionGain[i] = jParam.jointFrictionGain[i];

                        /* Joint Trajectory Set from Save Data */
                        jTrajSet.jointTrajectoryTime[i] = jTraj.jointTrajectoryTime[i];
                        jTrajSet.jointTrajectoryAcc[i] = jTraj.jointTrajectoryAcc[i];

                        /* Joint Target Set from Save Data */
                        jTargetSet.jointTarget[i] = jTarget.jointTarget[i];
                    }

                    for(int i = 0; i < CartesianDofSelect; i++)
                    {
                        /* Cartesian Parameters Set from Save Data */
                        cParamSet.cartesianPositionPgain[i] = cParam.cartesianPositionPgain[i];
                        cParamSet.cartesianPositionIgain[i] = cParam.cartesianPositionIgain[i];
                        cParamSet.cartesianPositionDgain[i] = cParam.cartesianPositionDgain[i];

                        /* Cartesian Trajectory Set from Save Data */
                        cTrajSet.cartesianTrajectoryTime[i] = cTraj.cartesianTrajectoryTime[i];
                        cTrajSet.cartesianTrajectoryAcc[i] = cTraj.cartesianTrajectoryAcc[i];
                        
                    }

                    /* Cartesian Parameters Set from Save Data */
                    cParamSet = cParam;
                    /* Cartesian Trajectory Set from Save Data */
                    cTrajSet = cTraj;
                    /* Cartesian Target Set from Save Data */
                    cTargetSet = cTarget;

                    tcpClient = new TcpClient();
                    tcpClient.Connect(ipAddr, port);
                    /* TCP/IP Communication - Send Part */
                    NetworkStream tcpClientNS = tcpClient.GetStream();
                    tcpClientNS.WriteTimeout = 100;
                    //tcpClientNS.ReadTimeout = 100;
                    
                    BitArray SendPacketType1 = new BitArray(new byte[] { MsgStateSend.packetType[0]});
                    BitArray SendPacketType2 = new BitArray(new byte[] { MsgStateSend.packetType[1]});
                    
                    UInt16 bufferSize = Convert.ToUInt16(Marshal.SizeOf(MsgStateSend));
                    
                    byte[] VariableDataInput = new byte[0];
                    for (int i = 0;i < 8;i++)
                    {
                        if(SendPacketType1[i] == true)
                        {
                            switch (i)
                            {
                                case JOINT_PARAMETER_SET:
                                    {
                                        bufferSize += Convert.ToUInt16(Marshal.SizeOf(jParamSet));
                                        Array.Resize<byte>(ref VariableDataInput, bufferSize - Convert.ToUInt16(Marshal.SizeOf(MsgStateSend)));
                                        Array.Copy(StructToByte(jParamSet), 0, VariableDataInput, 0, StructToByte(jParamSet).Length);
                                        SendPacketType1[i] = false;
                                        break;
                                    }
                                case JOINT_TRAJECTORY_SET:
                                    {
                                        bufferSize += Convert.ToUInt16(Marshal.SizeOf(jTrajSet));
                                        Array.Resize<byte>(ref VariableDataInput, bufferSize - Convert.ToUInt16(Marshal.SizeOf(MsgStateSend)));
                                        Array.Copy(StructToByte(jTrajSet), 0, VariableDataInput, 0, StructToByte(jTrajSet).Length);
                                        SendPacketType1[i] = false;
                                        break;
                                    }
                                case CARTESIAN_PARAMETER_SET:
                                    {
                                        bufferSize += Convert.ToUInt16(Marshal.SizeOf(cParamSet));
                                        Array.Resize<byte>(ref VariableDataInput, bufferSize - Convert.ToUInt16(Marshal.SizeOf(MsgStateSend)));
                                        Array.Copy(StructToByte(cParamSet), 0, VariableDataInput, 0, StructToByte(cParamSet).Length);
                                        SendPacketType1[i] = false;
                                        break;
                                    }
                                case CARTESIAN_TRAJECTORY_SET:
                                    {
                                        bufferSize += Convert.ToUInt16(Marshal.SizeOf(cTrajSet));
                                        Array.Resize<byte>(ref VariableDataInput, bufferSize - Convert.ToUInt16(Marshal.SizeOf(MsgStateSend)));
                                        Array.Copy(StructToByte(cTrajSet), 0, VariableDataInput, 0, StructToByte(cTrajSet).Length);
                                        SendPacketType1[i] = false;
                                        break;
                                    }
                                case JOINT_TARGET_SET:
                                    {
                                        bufferSize += Convert.ToUInt16(Marshal.SizeOf(jTrajSet));
                                        Array.Resize<byte>(ref VariableDataInput, bufferSize - Convert.ToUInt16(Marshal.SizeOf(MsgStateSend)));
                                        Array.Copy(StructToByte(jTarget), 0, VariableDataInput, 0, StructToByte(jTarget).Length);
                                        SendPacketType1[i] = false;
                                        break;
                                    }
                                case CARTESIAN_TARGET_SET:
                                    {
                                        bufferSize += Convert.ToUInt16(Marshal.SizeOf(cTrajSet));
                                        Array.Resize<byte>(ref VariableDataInput, bufferSize - Convert.ToUInt16(Marshal.SizeOf(MsgStateSend)));
                                        Array.Copy(StructToByte(cTarget), 0, VariableDataInput, 0, StructToByte(cTarget).Length);
                                        SendPacketType1[i] = false;
                                        break;
                                    }
                                case SERVER_SYSTEM_DATA:
                                    {
                                        bufferSize += Convert.ToUInt16(Marshal.SizeOf(ServerData));
                                        Array.Resize<byte>(ref VariableDataInput, bufferSize - Convert.ToUInt16(Marshal.SizeOf(MsgStateSend)));
                                        Array.Copy(StructToByte(ServerData), 0, VariableDataInput, 0, StructToByte(ServerData).Length);
                                        SendPacketType1[i] = false;
                                        break;
                                    }
                            }
                        }
                    }

                    byte[] LoadBuffer;
                    if (bufferSize > Convert.ToUInt16(Marshal.SizeOf(MsgStateSend)))
                    {
                        //AddListBox(LoglistBox, "Send Load Buffer");
                        LoadBuffer = new byte[bufferSize];
                        Array.Copy(StructToByte(MsgStateSend), 0, LoadBuffer, 0, StructToByte(MsgStateSend).Length);
                        Array.Copy(VariableDataInput, 0, LoadBuffer, StructToByte(MsgStateSend).Length, VariableDataInput.Length);
                        AddListBox(LoglistBox, "Send Load Buffer");
                        tcpClientNS.Write(LoadBuffer, 0, LoadBuffer.Length);
                        tcpClientNS.Flush();
                        LoadBuffer = null;
                    }
                    else
                    {
                        //AddListBox(LoglistBox, "Send MsgState");
                        tcpClientNS.Write(StructToByte(MsgStateSend), 0, StructToByte(MsgStateSend).Length);
                        AddListBox(LoglistBox, "MsgStateSend : " + Convert.ToString(MsgStateSend.packetType[0]) + ", " + Convert.ToString(MsgStateSend.packetType[1]) +
                            ", " + Convert.ToString(MsgStateSend.commState) + ", " + Convert.ToString(MsgStateSend.controlMode) + ", " + Convert.ToString(MsgStateSend.payloadSize));
                        tcpClientNS.Flush();
                    }
                    /* End TCP/IP Communication - Send Part */


                    /* TCP/IP Communication - Recv Part */
                    /* Msg State Read */
                    byte[] msgStatebuffer = new byte[Marshal.SizeOf(MsgStateRecv)];
                    int _msgStateRead = tcpClientNS.Read(msgStatebuffer, 0, msgStatebuffer.Length);
                    if (_msgStateRead != msgStatebuffer.Length)
                    {
                        AddListBox(LoglistBox, "TCP/IP - MsgState Read Error!!!");
                    }
                    else
                    {
                        /* Convert MsgState */
                        MsgStateRecv = ByteToStruct<MsgState>(msgStatebuffer);

                        BitArray RecvPacketType1 = new BitArray(new byte[] { MsgStateRecv.packetType[0] });
                        BitArray RecvPacketType2 = new BitArray(new byte[] { MsgStateRecv.packetType[1] });
                        
                        /* Communication State Machine */
                        int recvState = Convert.ToInt32(MsgStateRecv.commState);
                        switch (recvState)
                        {
                            case REGISTRATION:
                                {
                                    MsgStateSend.packetType[0] = 0;
                                    MsgStateSend.packetType[1] = 0;
                                    MsgStateSend.commState = REGISTRATION;
                                    MsgStateSend.payloadSize = Convert.ToUInt16(Marshal.SizeOf(MsgStateSend));
                                    break;
                                }
                            case REGISTRATION_COMPLETE:
                                {
                                    if(MsgStateSend.commState == SERVO_ON)
                                    {
                                        MsgStateSend.packetType[0] = 0;
                                        MsgStateSend.packetType[1] = 0;
                                        MsgStateSend.commState = SERVO_ON;
                                        MsgStateSend.payloadSize = Convert.ToUInt16(Marshal.SizeOf(MsgStateSend));
                                    }
                                    else
                                    {
                                        MsgStateSend.packetType[0] = 0;
                                        MsgStateSend.packetType[1] = 0;
                                        MsgStateSend.commState = REGISTRATION;
                                        MsgStateSend.payloadSize = Convert.ToUInt16(Marshal.SizeOf(MsgStateSend));
                                    }
                                    break;
                                }
                            case SERVO_ON:
                                {
                                    MsgStateSend.packetType[0] = 0;
                                    MsgStateSend.packetType[1] = 0;
                                    MsgStateSend.commState = SERVO_ON;
                                    MsgStateSend.payloadSize = Convert.ToUInt16(Marshal.SizeOf(MsgStateSend));
                                    break;
                                }
                            case SERVO_ON_COMPLETE:
                                {
                                    MsgStateSend.commState = TUNNING_STATE;
                                    break;
                                }
                            case TUNNING_STATE:
                                {
                                    MsgStateSend.commState = TUNNING_STATE;
                                    break;
                                }
                            case TUNNING_STATE_COMPLETE:
                                {
                                    if(MsgStateSend.commState == HOMING)
                                    {
                                        MsgStateSend.commState = HOMING;
                                    }
                                    else
                                    {
                                        MsgStateSend.packetType[0] = 0;
                                        MsgStateSend.packetType[1] = 0;
                                        MsgStateSend.commState = TUNNING_STATE;
                                        MsgStateSend.payloadSize = Convert.ToUInt16(Marshal.SizeOf(MsgStateSend));
                                    }
                                    break;
                                }
                            case HOMING:
                                {
                                    break;
                                }
                            case HOMING_COMPLETE:
                                {
                                    if(MsgStateSend.commState == FREE_STATE)
                                    {
                                        MsgStateSend.commState = FREE_STATE;
                                    }
                                    else
                                    {
                                        MsgStateSend.commState = HOMING;
                                    }
                                    break;
                                }
                            case FREE_STATE:
                                {
                                    MsgStateSend.commState = FREE_STATE;
                                    break;
                                }
                        }
                        /* End Communication State Machine */

                        for (int i = 0; i < 8; i++)
                        {
                            if (RecvPacketType1[i] == true)
                            {
                                switch (i)
                                {
                                    case JOINT_PARAMETER_SET:
                                        {
                                            /* Recv Joint Param Msg */
                                            byte[] JointParamBuffer = new byte[Marshal.SizeOf(jParamGet)];
                                            int _JointParamRead = tcpClientNS.Read(JointParamBuffer, 0, JointParamBuffer.Length);
                                            if (_JointParamRead != JointParamBuffer.Length)
                                            {
                                                AddListBox(LoglistBox, "TCP/IP - Joint Parameters Get Error!!!");
                                            }

                                            /* Convert Joint Parameters */
                                            jParamGet = ByteToStruct<JointParameterSettingStruct>(JointParamBuffer);
                                            break;
                                        }
                                    case JOINT_TRAJECTORY_SET:
                                        {
                                            /* Recv Joint Trajectory Msg */
                                            byte[] JointTrajBuffer = new byte[Marshal.SizeOf(jTrajGet)];
                                            int _JointTrajRead = tcpClientNS.Read(JointTrajBuffer, 0, JointTrajBuffer.Length);
                                            if (_JointTrajRead != JointTrajBuffer.Length)
                                            {
                                                AddListBox(LoglistBox, "TCP/IP - Joint Trajectory Parameters Get Error!!!");
                                            }

                                            /* Convert Joint Trajectory */
                                            jTrajGet = ByteToStruct<JointTrajectorySetStruct>(JointTrajBuffer);
                                            break;
                                        }
                                    case CARTESIAN_PARAMETER_SET:
                                        {
                                            /* Recv Cartesian Param Msg */
                                            byte[] CartesianParamBuffer = new byte[Marshal.SizeOf(cParamGet)];
                                            int _CartesianParamRead = tcpClientNS.Read(CartesianParamBuffer, 0, CartesianParamBuffer.Length);
                                            if (_CartesianParamRead != CartesianParamBuffer.Length)
                                            {
                                                AddListBox(LoglistBox, "TCP/IP - Cartesian Parameters Get Error!!!");
                                            }

                                            /* Convert Cartesian Parameters */
                                            cParamGet = ByteToStruct<CartesianParameterSettingStruct>(CartesianParamBuffer);
                                            break;
                                        }
                                    case CARTESIAN_TRAJECTORY_SET:
                                        {
                                            /* Recv Cartesian Trajectory Msg */
                                            byte[] CartesianTrajBuffer = new byte[Marshal.SizeOf(cTrajGet)];
                                            int _CartesianTrajRead = tcpClientNS.Read(CartesianTrajBuffer, 0, CartesianTrajBuffer.Length);
                                            if (_CartesianTrajRead != CartesianTrajBuffer.Length)
                                            {
                                                AddListBox(LoglistBox, "TCP/IP - Cartesian Trajectory Parameters Get Error!!!");
                                            }

                                            /* Convert Cartesian Trajectory */
                                            cTrajGet = ByteToStruct<CartesianTrajectorySetStruct>(CartesianTrajBuffer);
                                            break;
                                        }
                                    case JOINT_TARGET_SET:
                                        {
                                            /* Recv Joint Target Msg */
                                            byte[] JointTargetBuffer = new byte[Marshal.SizeOf(cTargetGet)];
                                            int _JointTargetRead = tcpClientNS.Read(JointTargetBuffer, 0, JointTargetBuffer.Length);
                                            if (_JointTargetRead != JointTargetBuffer.Length)
                                            {
                                                AddListBox(LoglistBox, "TCP/IP - Joint Target Get Error!!!");
                                            }

                                            /* Convert Joint Target */
                                            jTargetGet = ByteToStruct<JointTargetStruct>(JointTargetBuffer);
                                            break;
                                        }
                                    case CARTESIAN_TARGET_SET:
                                        {
                                            /* Recv Cartesian Target Msg */
                                            byte[] CartesianTargetBuffer = new byte[Marshal.SizeOf(cTargetGet)];
                                            int _CartesianTargetRead = tcpClientNS.Read(CartesianTargetBuffer, 0, CartesianTargetBuffer.Length);
                                            if (_CartesianTargetRead != CartesianTargetBuffer.Length)
                                            {
                                                AddListBox(LoglistBox, "TCP/IP - Joint Target Get Error!!!");
                                            }

                                            /* Convert Cartesian Target */
                                            jTargetGet = ByteToStruct<JointTargetStruct>(CartesianTargetBuffer);
                                            break;
                                        }
                                    case SERVER_SYSTEM_DATA:
                                       {
                                            /* Recv Control Module(CM) Data Msg (a.k.a. ServerData) */
                                            byte[] ServerDataBuffer = new byte[Marshal.SizeOf(ServerData)];
                                            int _ServerDataRead = tcpClientNS.Read(ServerDataBuffer, 0, ServerDataBuffer.Length);
                                            if (_ServerDataRead != ServerDataBuffer.Length)
                                            {
                                                AddListBox(LoglistBox, "TCP/IP - Sever System Data Get Error!!!");
                                            }

                                            /* Convert ServerData */
                                            ServerData = ByteToStruct<ServerSystemData>(ServerDataBuffer);
                                            break;
                                       }
                                }
                            }
                        }

                    }
                    /* End TCP/IP Communication - Recv Part */

                    

                    /* Communication State Machine */
                    if (tcpClient.Connected)
                    {
                        tcpClientNS.Close();
                        tcpClient.Close();
                        //log.Error("TCP Client Communication Close");
                        //AddListBox(LoglistBox, "TCP Client Communication Close");
                    }

                    /* End Mutex */
                    tcpIpMutex.ReleaseMutex();
                    Thread.Sleep(1);
                }
            }
        }

        const int ADS = 0;
        const int TCP = 1;
        const int UDP = 2;
        public Thread tcpRunThread;
        private void Connectbutton_Click(object sender, EventArgs e)
        {
            switch(CommunicationComboBox.SelectedIndex)
            {
                case ADS:
                    {
                        /* Registration */
                        MsgStateSend.packetType[0] = 0;
                        MsgStateSend.packetType[1] = 0;
                        MsgStateSend.commState = REGISTRATION;
                        MsgStateSend.payloadSize = Convert.ToUInt16(Marshal.SizeOf(MsgStateSend));
                        /* End Registration */

                        /* Main Timer Start */
                        StatusMainTimerStart = true;
                        StatusCommunicationConnect = true;
                        /* Status Light */
                        ConnectpictureBox.Image = StatusOnImage;
                        /* End Status Light */

                        /* Logger */
                        log.Info("Communication Ready : ADS Communication");
                        break;
                    }
                case TCP:
                    {
                        /* Tcp Thread Init */
                        try
                        {
                            /* Registration */
                            MsgStateSend.packetType[0] = 0;
                            MsgStateSend.packetType[1] = 0;
                            MsgStateSend.commState = REGISTRATION;
                            MsgStateSend.payloadSize = Convert.ToUInt16(Marshal.SizeOf(MsgStateSend));
                            /* End Registration */

                            tcpRunThread = new Thread(() => TcpClientRun(IpAddressTextBox.Text, int.Parse(PortTextBox.Text)));
                            AddListBox(LoglistBox, "TCP Client Thread Run IP : " + IpAddressTextBox.Text + " Port : " + PortTextBox.Text);
                            tcpRunThread.IsBackground = true;
                            StatusThreadStop = false;
                            tcpRunThread.Start();
                        }
                        catch(Exception ex)
                        {
                            Debug.WriteLine(ex.Message);
                            log.Error(ex.Message);
                        }

                        /* Main Timer Start */
                        StatusMainTimerStart = true;
                        StatusCommunicationConnect = true;
                        ConnectpictureBox.Image = StatusOnImage;
                        /* Logger */
                        log.Info("Communication Ready : TCP Communication");
                        break;
                    }
                case UDP:
                    {
                        /* Registration */
                        MsgStateSend.packetType[0] = 0;
                        MsgStateSend.packetType[1] = 0;
                        MsgStateSend.commState = REGISTRATION;
                        MsgStateSend.payloadSize = Convert.ToUInt16(Marshal.SizeOf(MsgStateSend));
                        /* End Registration */

                        /* Main Timer Start */
                        StatusMainTimerStart = true;
                        StatusCommunicationConnect = true;
                        /* Status Light */
                        ConnectpictureBox.Image = StatusOnImage;
                        /* End Status Light */

                        /* Logger */
                        log.Info("CommunicationComboBox Ready : UDP Communication");
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
                LoggingSystemgroupBox.Enabled = true;
                ParameterSetgroupBox.Enabled = true;
                /* End Activation GroupBox */

                ConnectCompletepictureBox.Image = StatusOnImage;
            }
        }
        
        private void MainTimer_Tick(object sender, EventArgs e)
        {

            //ErrorCodetextBox.Text = "Checking....";

           if (StatusMainTimerStart == true)
            {
                /* Status Setting Check */
                CheckCartesianModeReady();
                /* End Status Setting Check */      
            }

            if(!StatusThreadStop)
            {
                /* Status Light */
                /*
                if (tcpClient.Connected)
                {
                    ConnectpictureBox.Image = StatusOnImage;
                }
                else
                {
                    ConnectpictureBox.Image = StatusOffImage;
                }
                */
                /* End Status Light */
                UpdateTextBox(AbsPosJoint1textBox, Convert.ToString(ServerData.moduleData[0].actualLinkPosition[0]));
                UpdateTextBox(AbsPosJoint2textBox, Convert.ToString(ServerData.moduleData[0].actualLinkPosition[1]));
                UpdateTextBox(AbsPosJoint3textBox, Convert.ToString(ServerData.moduleData[0].actualLinkPosition[2]));
                UpdateTextBox(AbsPosJoint4textBox, Convert.ToString(ServerData.moduleData[0].actualLinkPosition[3]));
                UpdateTextBox(AbsPosJoint5textBox, Convert.ToString(ServerData.moduleData[0].actualLinkPosition[4]));
                UpdateTextBox(AbsPosJoint6textBox, Convert.ToString(ServerData.moduleData[0].actualLinkPosition[5]));
                UpdateTextBox(AbsPosJoint7textBox, Convert.ToString(ServerData.moduleData[0].actualLinkPosition[6]));
                UpdateTextBox(AbsPosJoint8textBox, Convert.ToString(0));

                UpdateTextBox(IncPositionJoint1textBox, Convert.ToString(ServerData.moduleData[0].actualLinkPosition[0]));
                UpdateTextBox(IncPositionJoint2textBox, Convert.ToString(ServerData.moduleData[0].actualLinkPosition[1]));
                UpdateTextBox(IncPositionJoint3textBox, Convert.ToString(ServerData.moduleData[0].actualLinkPosition[2]));
                UpdateTextBox(IncPositionJoint4textBox, Convert.ToString(ServerData.moduleData[0].actualLinkPosition[3]));
                UpdateTextBox(IncPositionJoint5textBox, Convert.ToString(ServerData.moduleData[0].actualLinkPosition[4]));
                UpdateTextBox(IncPositionJoint6textBox, Convert.ToString(ServerData.moduleData[0].actualLinkPosition[5]));
                UpdateTextBox(IncPositionJoint7textBox, Convert.ToString(ServerData.moduleData[0].actualLinkPosition[6]));
                UpdateTextBox(IncPositionJoint8textBox, Convert.ToString(0));

                UpdateTextBox(IncVelocityJoint1textBox, Convert.ToString(ServerData.moduleData[0].actualLinkVelocity[0]));
                UpdateTextBox(IncVelocityJoint2textBox, Convert.ToString(ServerData.moduleData[0].actualLinkVelocity[1]));
                UpdateTextBox(IncVelocityJoint3textBox, Convert.ToString(ServerData.moduleData[0].actualLinkVelocity[2]));
                UpdateTextBox(IncVelocityJoint4textBox, Convert.ToString(ServerData.moduleData[0].actualLinkVelocity[3]));
                UpdateTextBox(IncVelocityJoint5textBox, Convert.ToString(ServerData.moduleData[0].actualLinkVelocity[4]));
                UpdateTextBox(IncVelocityJoint6textBox, Convert.ToString(ServerData.moduleData[0].actualLinkVelocity[5]));
                UpdateTextBox(IncVelocityJoint7textBox, Convert.ToString(ServerData.moduleData[0].actualLinkVelocity[6]));
                UpdateTextBox(IncVelocityJoint8textBox, Convert.ToString(0));

                UpdateTextBox(CurrentJoint1textBox, Convert.ToString(ServerData.moduleData[0].actualCurrent[0]));
                UpdateTextBox(CurrentJoint2textBox, Convert.ToString(ServerData.moduleData[0].actualCurrent[1]));
                UpdateTextBox(CurrentJoint3textBox, Convert.ToString(ServerData.moduleData[0].actualCurrent[2]));
                UpdateTextBox(CurrentJoint4textBox, Convert.ToString(ServerData.moduleData[0].actualCurrent[3]));
                UpdateTextBox(CurrentJoint5textBox, Convert.ToString(ServerData.moduleData[0].actualCurrent[4]));
                UpdateTextBox(CurrentJoint6textBox, Convert.ToString(ServerData.moduleData[0].actualCurrent[5]));
                UpdateTextBox(CurrentJoint7textBox, Convert.ToString(ServerData.moduleData[0].actualCurrent[6]));
                UpdateTextBox(CurrentJoint8textBox, Convert.ToString(0));

            }
        }
        public static bool ServoOnCheck = false;
        private void ServoOnButton_Click(object sender, EventArgs e)
        {
            /* Variable Initialize */
            int _dof = Convert.ToInt32(NumJointcomboBox.SelectedItem.ToString());
            UpdateTextBox(ErrorCodetextBox, _dof.ToString() + " Setting");
            /* Parameters and Command Initialize */
            JointParameterSettingInit(_dof);
            CartesianParameterSettingInit();
            JointTargetInit(_dof);
            /* End Parameters and Command Initialize */
            /* End Variable Initialize */

            /* Servo On Behavior */
            StatusServoOn = true;
            /* End Servo On Behavior */
           
            if(StatusServoOn == true)
            {
                /* ServoOn Button Color Change */
                ServoOnButton.ForeColor = System.Drawing.Color.Black;
                ServoOnButton.BackColor = System.Drawing.Color.YellowGreen;
                /* End ServoOn Button Color Change */

                /* Status Light */
                AuxPowerpictureBox.Image = StatusOnImage;
                MainPowerpictureBox.Image = StatusOnImage;
                /* End Status Light */


                AbsoluteEnocdergroupBox.Enabled = true;
                MainTapControl.Enabled = true;
            }
            /* Servo ON Send Command */
            BitArray PacketType1 = new BitArray(8);
            BitArray PacketType2 = new BitArray(new byte[] { 0 });
            PacketType1.Set(0, false);
            PacketType1.Set(1, false);
            PacketType1.Set(2, false);
            PacketType1.Set(3, false);
            PacketType1.Set(4, false);
            PacketType1.Set(5, false);
            PacketType1.Set(6, false);
            PacketType1.Set(7, false);
            MsgStateSend.packetType[0] = GetByteFromBitArray(PacketType1);
            MsgStateSend.packetType[1] = GetByteFromBitArray(PacketType2);
            MsgStateSend.commState = Convert.ToUInt16(REGISTRATION);
            MsgStateSend.controlMode = Convert.ToByte(GRAVITY_MODE);
            MsgStateSend.payloadSize = 0;
            /* End Servo On Send Command */
        }

        private void SetJointPosPIDGainbutton_Click(object sender, EventArgs e)
        {
            /* Joint Position PID Gain Set */
            jParam.jointPositionPgain[0] = float.Parse(jpPgainJoint1textBox.Text);
            jParam.jointPositionPgain[1] = float.Parse(jpPgainJoint2textBox.Text);
            jParam.jointPositionPgain[2] = float.Parse(jpPgainJoint3textBox.Text);
            jParam.jointPositionPgain[3] = float.Parse(jpPgainJoint4textBox.Text);
            jParam.jointPositionPgain[4] = float.Parse(jpPgainJoint5textBox.Text);
            jParam.jointPositionPgain[5] = float.Parse(jpPgainJoint6textBox.Text);
            jParam.jointPositionPgain[6] = float.Parse(jpPgainJoint7textBox.Text);
            jParam.jointPositionPgain[7] = float.Parse(jpPgainJoint8textBox.Text);

            jParam.jointPositionIgain[0] = float.Parse(jpIgainJoint1textBox.Text);
            jParam.jointPositionIgain[1] = float.Parse(jpIgainJoint2textBox.Text);
            jParam.jointPositionIgain[2] = float.Parse(jpIgainJoint3textBox.Text);
            jParam.jointPositionIgain[3] = float.Parse(jpIgainJoint4textBox.Text);
            jParam.jointPositionIgain[4] = float.Parse(jpIgainJoint5textBox.Text);
            jParam.jointPositionIgain[5] = float.Parse(jpIgainJoint6textBox.Text);
            jParam.jointPositionIgain[6] = float.Parse(jpIgainJoint7textBox.Text);
            jParam.jointPositionIgain[7] = float.Parse(jpIgainJoint8textBox.Text);

            jParam.jointPositionDgain[0] = float.Parse(jpDgainJoint1textBox.Text);
            jParam.jointPositionDgain[1] = float.Parse(jpDgainJoint2textBox.Text);
            jParam.jointPositionDgain[2] = float.Parse(jpDgainJoint3textBox.Text);
            jParam.jointPositionDgain[3] = float.Parse(jpDgainJoint4textBox.Text);
            jParam.jointPositionDgain[4] = float.Parse(jpDgainJoint5textBox.Text);
            jParam.jointPositionDgain[5] = float.Parse(jpDgainJoint6textBox.Text);
            jParam.jointPositionDgain[6] = float.Parse(jpDgainJoint7textBox.Text);
            jParam.jointPositionDgain[7] = float.Parse(jpDgainJoint8textBox.Text);
            /* End Joint Position PID Gain Set */
            StatusJointPositionPIDgainSet = true;
            /* packetType[0] Set */
            MsgStateSend.packetType[0] += 1;
            /* Status Setting Check */
            CheckHomingReady();
            /* End Status Setting Check */
        }

        private void SetJointToqPIDGainbutton_Click(object sender, EventArgs e)
        {
            /* Joint Torque PID Gain Set */
            jParam.jointTorquePgain[0] = float.Parse(jtPgainJoint1textBox.Text);
            jParam.jointTorquePgain[1] = float.Parse(jtPgainJoint2textBox.Text);
            jParam.jointTorquePgain[2] = float.Parse(jtPgainJoint3textBox.Text);
            jParam.jointTorquePgain[3] = float.Parse(jtPgainJoint4textBox.Text);
            jParam.jointTorquePgain[4] = float.Parse(jtPgainJoint5textBox.Text);
            jParam.jointTorquePgain[5] = float.Parse(jtPgainJoint6textBox.Text);
            jParam.jointTorquePgain[6] = float.Parse(jtPgainJoint7textBox.Text);
            jParam.jointTorquePgain[7] = float.Parse(jtPgainJoint8textBox.Text);

            jParam.jointTorqueIgain[0] = float.Parse(jtIgainJoint1textBox.Text);
            jParam.jointTorqueIgain[1] = float.Parse(jtIgainJoint2textBox.Text);
            jParam.jointTorqueIgain[2] = float.Parse(jtIgainJoint3textBox.Text);
            jParam.jointTorqueIgain[3] = float.Parse(jtIgainJoint4textBox.Text);
            jParam.jointTorqueIgain[4] = float.Parse(jtIgainJoint5textBox.Text);
            jParam.jointTorqueIgain[5] = float.Parse(jtIgainJoint6textBox.Text);
            jParam.jointTorqueIgain[6] = float.Parse(jtIgainJoint7textBox.Text);
            jParam.jointTorqueIgain[7] = float.Parse(jtIgainJoint8textBox.Text);

            jParam.jointTorqueDgain[0] = float.Parse(jtDgainJoint1textBox.Text);
            jParam.jointTorqueDgain[1] = float.Parse(jtDgainJoint2textBox.Text);
            jParam.jointTorqueDgain[2] = float.Parse(jtDgainJoint3textBox.Text);
            jParam.jointTorqueDgain[3] = float.Parse(jtDgainJoint4textBox.Text);
            jParam.jointTorqueDgain[4] = float.Parse(jtDgainJoint5textBox.Text);
            jParam.jointTorqueDgain[5] = float.Parse(jtDgainJoint6textBox.Text);
            jParam.jointTorqueDgain[6] = float.Parse(jtDgainJoint7textBox.Text);
            jParam.jointTorqueDgain[7] = float.Parse(jtDgainJoint8textBox.Text);
            /* End Joint Torque PID Gain Set */
            StatusJointTorquePIDgainSet = true;
            /* packetType[0] Set */
            MsgStateSend.packetType[0] += 1;
            /* Status Setting Check */
            CheckHomingReady();
            /* End Status Setting Check */
        }

        private void JointConstantParamSetbutton_Click(object sender, EventArgs e)
        {
            /* Joint Constant Param Gain Set */
            jParam.jointConstantTorque[0] = float.Parse(jcTorqueConstJoint1textBox.Text);
            jParam.jointConstantTorque[1] = float.Parse(jcTorqueConstJoint2textBox.Text);
            jParam.jointConstantTorque[2] = float.Parse(jcTorqueConstJoint3textBox.Text);
            jParam.jointConstantTorque[3] = float.Parse(jcTorqueConstJoint4textBox.Text);
            jParam.jointConstantTorque[4] = float.Parse(jcTorqueConstJoint5textBox.Text);
            jParam.jointConstantTorque[5] = float.Parse(jcTorqueConstJoint6textBox.Text);
            jParam.jointConstantTorque[6] = float.Parse(jcTorqueConstJoint7textBox.Text);
            jParam.jointConstantTorque[7] = float.Parse(jcTorqueConstJoint8textBox.Text);

            jParam.jointConstantEfficiency[0] = float.Parse(jcEfficiencyJoint1textBox.Text);
            jParam.jointConstantEfficiency[1] = float.Parse(jcEfficiencyJoint2textBox.Text);
            jParam.jointConstantEfficiency[2] = float.Parse(jcEfficiencyJoint3textBox.Text);
            jParam.jointConstantEfficiency[3] = float.Parse(jcEfficiencyJoint4textBox.Text);
            jParam.jointConstantEfficiency[4] = float.Parse(jcEfficiencyJoint5textBox.Text);
            jParam.jointConstantEfficiency[5] = float.Parse(jcEfficiencyJoint6textBox.Text);
            jParam.jointConstantEfficiency[6] = float.Parse(jcEfficiencyJoint7textBox.Text);
            jParam.jointConstantEfficiency[7] = float.Parse(jcEfficiencyJoint8textBox.Text);

            jParam.jointConstantSpring[0] = float.Parse(jcSpringJoint1textBox.Text);
            jParam.jointConstantSpring[1] = float.Parse(jcSpringJoint2textBox.Text);
            jParam.jointConstantSpring[2] = float.Parse(jcSpringJoint3textBox.Text);
            jParam.jointConstantSpring[3] = float.Parse(jcSpringJoint4textBox.Text);
            jParam.jointConstantSpring[4] = float.Parse(jcSpringJoint5textBox.Text);
            jParam.jointConstantSpring[5] = float.Parse(jcSpringJoint6textBox.Text);
            jParam.jointConstantSpring[6] = float.Parse(jcSpringJoint7textBox.Text);
            jParam.jointConstantSpring[7] = float.Parse(jcSpringJoint8textBox.Text);
            /* End Joint Constant Param Gain Set */
            StatusJointConstantParamSet = true;
            /* packetType[0] Set */
            MsgStateSend.packetType[0] += 1;
            /* Status Setting Check */
            CheckHomingReady();
            /* End Status Setting Check */
        }

        private void GCnFricParamSetbutton_Click(object sender, EventArgs e)
        {
            /* Gravity Compensator and Friction Compensator Parameters Set */
            jParam.jointGravityGain[0] = float.Parse(GCGainJoint1textBox.Text);
            jParam.jointGravityGain[1] = float.Parse(GCGainJoint2textBox.Text);
            jParam.jointGravityGain[2] = float.Parse(GCGainJoint3textBox.Text);
            jParam.jointGravityGain[3] = float.Parse(GCGainJoint4textBox.Text);
            jParam.jointGravityGain[4] = float.Parse(GCGainJoint5textBox.Text);
            jParam.jointGravityGain[5] = float.Parse(GCGainJoint6textBox.Text);
            jParam.jointGravityGain[6] = float.Parse(GCGainJoint7textBox.Text);
            jParam.jointGravityGain[7] = float.Parse(GCGainJoint8textBox.Text);

            jParam.jointCurrentGain[0] = float.Parse(CurGainJoint1textBox.Text);
            jParam.jointCurrentGain[1] = float.Parse(CurGainJoint2textBox.Text);
            jParam.jointCurrentGain[2] = float.Parse(CurGainJoint3textBox.Text);
            jParam.jointCurrentGain[3] = float.Parse(CurGainJoint4textBox.Text);
            jParam.jointCurrentGain[4] = float.Parse(CurGainJoint5textBox.Text);
            jParam.jointCurrentGain[5] = float.Parse(CurGainJoint6textBox.Text);
            jParam.jointCurrentGain[6] = float.Parse(CurGainJoint7textBox.Text);
            jParam.jointCurrentGain[7] = float.Parse(CurGainJoint8textBox.Text);

            jParam.jointFrictionGain[0] = float.Parse(FricGainJoint1textBox.Text);
            jParam.jointFrictionGain[1] = float.Parse(FricGainJoint2textBox.Text);
            jParam.jointFrictionGain[2] = float.Parse(FricGainJoint3textBox.Text);
            jParam.jointFrictionGain[3] = float.Parse(FricGainJoint4textBox.Text);
            jParam.jointFrictionGain[4] = float.Parse(FricGainJoint5textBox.Text);
            jParam.jointFrictionGain[5] = float.Parse(FricGainJoint6textBox.Text);
            jParam.jointFrictionGain[6] = float.Parse(FricGainJoint7textBox.Text);
            jParam.jointFrictionGain[7] = float.Parse(FricGainJoint8textBox.Text);
            /* End Gravity Compensator and Friction Compensator Parameters Set */
            StatusJointGravityNFrictionParamSet = true;
            /* packetType[0] Set */
            MsgStateSend.packetType[0] += 1;
            /* Status Setting Check */
            CheckHomingReady();
            /* End Status Setting Check */
        }

        private void JointTrajetorySetbutton_Click(object sender, EventArgs e)
        {
            /* Joint Trajectory Parameters Set */
            jTraj.jointTrajectoryTime[0] = float.Parse(TrajTimeJoint1textBox.Text);
            jTraj.jointTrajectoryTime[1] = float.Parse(TrajTimeJoint2textBox.Text);
            jTraj.jointTrajectoryTime[2] = float.Parse(TrajTimeJoint3textBox.Text);
            jTraj.jointTrajectoryTime[3] = float.Parse(TrajTimeJoint4textBox.Text);
            jTraj.jointTrajectoryTime[4] = float.Parse(TrajTimeJoint5textBox.Text);
            jTraj.jointTrajectoryTime[5] = float.Parse(TrajTimeJoint6textBox.Text);
            jTraj.jointTrajectoryTime[6] = float.Parse(TrajTimeJoint7textBox.Text);
            jTraj.jointTrajectoryTime[7] = float.Parse(TrajTimeJoint8textBox.Text);

            jTraj.jointTrajectoryAcc[0] = float.Parse(TrajAccJoint1textBox.Text);
            jTraj.jointTrajectoryAcc[1] = float.Parse(TrajAccJoint2textBox.Text);
            jTraj.jointTrajectoryAcc[2] = float.Parse(TrajAccJoint3textBox.Text);
            jTraj.jointTrajectoryAcc[3] = float.Parse(TrajAccJoint4textBox.Text);
            jTraj.jointTrajectoryAcc[4] = float.Parse(TrajAccJoint5textBox.Text);
            jTraj.jointTrajectoryAcc[5] = float.Parse(TrajAccJoint6textBox.Text);
            jTraj.jointTrajectoryAcc[6] = float.Parse(TrajAccJoint7textBox.Text);
            jTraj.jointTrajectoryAcc[7] = float.Parse(TrajAccJoint8textBox.Text);
            /* End Joint Trajectory Parameters Set */
            StatusJointTrajectoryParamSet = true;
            /* packetType[0] Set */
            MsgStateSend.packetType[0] += 2;
            /* Status Setting Check */
            CheckHomingReady();
            /* End Status Setting Check */
        }

        private void CartesianPIDSetbutton_Click(object sender, EventArgs e)
        {
            /* Cartesian Position PID Gain Set */
            cParam.cartesianPositionPgain[0] = float.Parse(cpPgainCartesianXtextBox.Text);
            cParam.cartesianPositionPgain[1] = float.Parse(cpPgainCartesianYtextBox.Text);
            cParam.cartesianPositionPgain[2] = float.Parse(cpPgainCartesianZtextBox.Text);
            cParam.cartesianPositionPgain[3] = float.Parse(cpPgainCartesianRolltextBox.Text);
            cParam.cartesianPositionPgain[4] = float.Parse(cpPgainCartesianPitchtextBox.Text);
            cParam.cartesianPositionPgain[5] = float.Parse(cpPgainCartesianYawtextBox.Text);

            cParam.cartesianPositionIgain[0] = float.Parse(cpIgainCartesianXtextBox.Text);
            cParam.cartesianPositionIgain[1] = float.Parse(cpIgainCartesianYtextBox.Text);
            cParam.cartesianPositionIgain[2] = float.Parse(cpIgainCartesianZtextBox.Text);
            cParam.cartesianPositionIgain[3] = float.Parse(cpIgainCartesianRolltextBox.Text);
            cParam.cartesianPositionIgain[4] = float.Parse(cpIgainCartesianPitchtextBox.Text);
            cParam.cartesianPositionIgain[5] = float.Parse(cpIgainCartesianYawtextBox.Text);

            cParam.cartesianPositionDgain[0] = float.Parse(cpDgainCartesianXtextBox.Text);
            cParam.cartesianPositionDgain[1] = float.Parse(cpDgainCartesianYtextBox.Text);
            cParam.cartesianPositionDgain[2] = float.Parse(cpDgainCartesianZtextBox.Text);
            cParam.cartesianPositionDgain[3] = float.Parse(cpDgainCartesianRolltextBox.Text);
            cParam.cartesianPositionDgain[4] = float.Parse(cpDgainCartesianPitchtextBox.Text);
            cParam.cartesianPositionDgain[5] = float.Parse(cpDgainCartesianYawtextBox.Text);
            /* End Cartesian Position PID Gain Set */
            StatusCartesianPositionPIDgainSet = true;
            /* packetType[0] Set */
            MsgStateSend.packetType[0] += 4;
            /* Status Setting Check */
            CheckHomingReady();
            /* End Status Setting Check */
        }

        private void CartesianTrajSetbutton_Click(object sender, EventArgs e)
        {
            
            /* Cartesian Trajectory Parameters Set */
            cTraj.cartesianTrajectoryTime[0] = float.Parse(TrajTimeCartesianXtextBox.Text);
            cTraj.cartesianTrajectoryTime[1] = float.Parse(TrajTimeCartesianYtextBox.Text);
            cTraj.cartesianTrajectoryTime[2] = float.Parse(TrajTimeCartesianZtextBox.Text);
            cTraj.cartesianTrajectoryTime[3] = float.Parse(TrajTimeCartesianRolltextBox.Text);
            cTraj.cartesianTrajectoryTime[4] = float.Parse(TrajTimeCartesianPitchtextBox.Text);
            cTraj.cartesianTrajectoryTime[5] = float.Parse(TrajTimeCartesianYawtextBox.Text);

            cTraj.cartesianTrajectoryAcc[0] = float.Parse(TrajAccCartesianXtextBox.Text);
            cTraj.cartesianTrajectoryAcc[1] = float.Parse(TrajAccCartesianYtextBox.Text);
            cTraj.cartesianTrajectoryAcc[2] = float.Parse(TrajAccCartesianZtextBox.Text);
            cTraj.cartesianTrajectoryAcc[3] = float.Parse(TrajAccCartesianRolltextBox.Text);
            cTraj.cartesianTrajectoryAcc[4] = float.Parse(TrajAccCartesianPitchtextBox.Text);
            cTraj.cartesianTrajectoryAcc[5] = float.Parse(TrajAccCartesianYawtextBox.Text);
            /* End Cartesian Trajectory Parameters Set */
            StatusCartesianTrajectoryParamSet = true;
            /* packetType[0] Set */
            MsgStateSend.packetType[0] += 8;
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
                /* Gravtiy Compensator Button Color Change */
                GCbutton.ForeColor = System.Drawing.Color.Black;
                GCbutton.BackColor = System.Drawing.Color.YellowGreen;
                /* End Gravtiy Compensator Button Color Change */

                /* Status Light */
                GCpictureBox.Image = StatusOnImage;
                TorqueModepictureBox.Image = StatusOnImage;
                PositionModepictureBox.Image = StatusOffImage;
                /* End Status Light */
                GCbuttonCheck = true;
            }
            else
            {
                /* Gravtiy Compensator Button Color Change */
                GCbutton.ForeColor = System.Drawing.Color.Black;
                GCbutton.BackColor = System.Drawing.Color.LightGray;
                /* End Gravtiy Compensator Button Color Change */

                /* Status Light */
                GCpictureBox.Image = StatusOffImage;
                /* End Status Light */
                GCbuttonCheck = false;
            }
            /* End GCbutton Color Change */
        }
        public static bool JointModeButtonCheck = false;
        private void JointModebutton_Click(object sender, EventArgs e)
        {
            /* Activate Joint Mode */
            if(JointModeButtonCheck == false)
            {
                /* Joint Mode Button Color Change */
                JointModebutton.ForeColor = System.Drawing.Color.Black;
                JointModebutton.BackColor = System.Drawing.Color.YellowGreen;
                /* End Joint Mode Button Color Change */

                /* Cartesian Mode Deactivate */
                CartesianModeButtonCheck = true;
                CartesianModebutton_Click(null, null);

                JointModeButtonCheck = true;
            }
            else /* Deactivate Joint Mode */
            {
                /* Joint Mode Button Color Change */
                JointModebutton.ForeColor = System.Drawing.Color.Black;
                JointModebutton.BackColor = System.Drawing.Color.LightGray;
                /* End Joint Mode Button Color Change */
                JointModeButtonCheck = false;
            }
        }
        public static bool CartesianModeButtonCheck = false;
        private void CartesianModebutton_Click(object sender, EventArgs e)
        {
            /*  Activate Cartesian Mode */
            if (CartesianModeButtonCheck == false)
            {
                /* Cartesian Mode Button Color Change */
                CartesianModebutton.ForeColor = System.Drawing.Color.Black;
                CartesianModebutton.BackColor = System.Drawing.Color.YellowGreen;
                /* End Joint Mode Button Color Change */

                /* Joint Deactiavte */
                JointModeButtonCheck = true;
                JointModebutton_Click(null, null);

                CartesianModeButtonCheck = true;
            }
            else /* Deactivate Cartesian Mode */
            {
                /* Cartesian Mode Button Color Change */
                CartesianModebutton.ForeColor = System.Drawing.Color.Black;
                CartesianModebutton.BackColor = System.Drawing.Color.LightGray;
                /* End Joint Mode Button Color Change */
                CartesianModeButtonCheck = false;
            }
        }

        private void NumJointcomboBox_SelectedIndexChanged(object sender, EventArgs e)
        {
           
        }

        private void ParametersReadbutton_Click(object sender, EventArgs e)
        {
            string path = @"Parameters.txt";
            FileInfo file = new FileInfo(path);
            if(file.Exists)
            {
                FileStream paramRead = new FileStream(path, FileMode.Open, FileAccess.Read);
                StreamReader strRead = new StreamReader(paramRead);
                string readBuffer = strRead.ReadToEnd();

                string[] data = readBuffer.Split(';');
                for (int i = 0; i < data.Length; i++)
                {
                    string[] divName = data[i].Split(Environment.NewLine.ToCharArray(), StringSplitOptions.RemoveEmptyEntries); 
                    
                    if(divName.Length < 1)
                    {
                        continue;
                    }

                    /* Joint Position PID Gain Parameters Read */
                    if (divName[0] == "Joint Position PID Gain Param")
                    {
                        for (int j = 1; j < divName.Length; j++)
                        {
                            if (String.Equals(divName[j], "P Gain"))
                            {
                                string[] divGain = divName[j+1].Split(' ');

                                jpPgainJoint1textBox.Text = divGain[0];
                                jpPgainJoint2textBox.Text = divGain[1];
                                jpPgainJoint3textBox.Text = divGain[2];
                                jpPgainJoint4textBox.Text = divGain[3];
                                jpPgainJoint5textBox.Text = divGain[4];
                                jpPgainJoint6textBox.Text = divGain[5];
                                jpPgainJoint7textBox.Text = divGain[6];
                                jpPgainJoint8textBox.Text = divGain[7];
                            }
                            else if (String.Equals(divName[j], "I Gain"))
                            {
                                string[] divGain = divName[j+1].Split(' ');
                                jpIgainJoint1textBox.Text = divGain[0];
                                jpIgainJoint2textBox.Text = divGain[1];
                                jpIgainJoint3textBox.Text = divGain[2];
                                jpIgainJoint4textBox.Text = divGain[3];
                                jpIgainJoint5textBox.Text = divGain[4];
                                jpIgainJoint6textBox.Text = divGain[5];
                                jpIgainJoint7textBox.Text = divGain[6];
                                jpIgainJoint8textBox.Text = divGain[7];
                            }
                            else if (String.Equals(divName[j], "D Gain"))
                            {
                                string[] divGain = divName[j+1].Split(' ');
                                jpDgainJoint1textBox.Text = divGain[0];
                                jpDgainJoint2textBox.Text = divGain[1];
                                jpDgainJoint3textBox.Text = divGain[2];
                                jpDgainJoint4textBox.Text = divGain[3];
                                jpDgainJoint5textBox.Text = divGain[4];
                                jpDgainJoint6textBox.Text = divGain[5];
                                jpDgainJoint7textBox.Text = divGain[6];
                                jpDgainJoint8textBox.Text = divGain[7];
                            }
                        }    
                    }
                    /* End Joint Position PID Gain Parameters Read */

                    /* Torque PID Gain Parameters Read */
                    if (divName[0] == "Torque PID Gain Param")
                    {
                        for (int j = 1; j < divName.Length; j++)
                        {
                            if (String.Equals(divName[j], "P Gain"))
                            {
                                string[] divGain = divName[j + 1].Split(' ');

                                jtPgainJoint1textBox.Text = divGain[0];
                                jtPgainJoint2textBox.Text = divGain[1];
                                jtPgainJoint3textBox.Text = divGain[2];
                                jtPgainJoint4textBox.Text = divGain[3];
                                jtPgainJoint5textBox.Text = divGain[4];
                                jtPgainJoint6textBox.Text = divGain[5];
                                jtPgainJoint7textBox.Text = divGain[6];
                                jtPgainJoint8textBox.Text = divGain[7];
                            }
                            else if (String.Equals(divName[j], "I Gain"))
                            {
                                string[] divGain = divName[j + 1].Split(' ');
                                jtIgainJoint1textBox.Text = divGain[0];
                                jtIgainJoint2textBox.Text = divGain[1];
                                jtIgainJoint3textBox.Text = divGain[2];
                                jtIgainJoint4textBox.Text = divGain[3];
                                jtIgainJoint5textBox.Text = divGain[4];
                                jtIgainJoint6textBox.Text = divGain[5];
                                jtIgainJoint7textBox.Text = divGain[6];
                                jtIgainJoint8textBox.Text = divGain[7];
                            }
                            else if (String.Equals(divName[j], "D Gain"))
                            {
                                string[] divGain = divName[j + 1].Split(' ');
                                jtDgainJoint1textBox.Text = divGain[0];
                                jtDgainJoint2textBox.Text = divGain[1];
                                jtDgainJoint3textBox.Text = divGain[2];
                                jtDgainJoint4textBox.Text = divGain[3];
                                jtDgainJoint5textBox.Text = divGain[4];
                                jtDgainJoint6textBox.Text = divGain[5];
                                jtDgainJoint7textBox.Text = divGain[6];
                                jtDgainJoint8textBox.Text = divGain[7];
                            }
                        }

                    }
                    /* End Torque PID Gain Parameters Read */

                    /* Joint Constant Setting Parameters Read */
                    if (divName[0] == "Joint Constant Setting")
                    {
                        for (int j = 1; j < divName.Length; j++)
                        {
                            if (String.Equals(divName[j], "Torque"))
                            {
                                string[] divGain = divName[j + 1].Split(' ');

                                jcTorqueConstJoint1textBox.Text = divGain[0];
                                jcTorqueConstJoint2textBox.Text = divGain[1];
                                jcTorqueConstJoint3textBox.Text = divGain[2];
                                jcTorqueConstJoint4textBox.Text = divGain[3];
                                jcTorqueConstJoint5textBox.Text = divGain[4];
                                jcTorqueConstJoint6textBox.Text = divGain[5];
                                jcTorqueConstJoint7textBox.Text = divGain[6];
                                jcTorqueConstJoint8textBox.Text = divGain[7];
                            }
                            else if (String.Equals(divName[j], "Efficiency"))
                            {
                                string[] divGain = divName[j + 1].Split(' ');
                                jcEfficiencyJoint1textBox.Text = divGain[0];
                                jcEfficiencyJoint2textBox.Text = divGain[1];
                                jcEfficiencyJoint3textBox.Text = divGain[2];
                                jcEfficiencyJoint4textBox.Text = divGain[3];
                                jcEfficiencyJoint5textBox.Text = divGain[4];
                                jcEfficiencyJoint6textBox.Text = divGain[5];
                                jcEfficiencyJoint7textBox.Text = divGain[6];
                                jcEfficiencyJoint8textBox.Text = divGain[7];
                            }
                            else if (String.Equals(divName[j], "Spring"))
                            {
                                string[] divGain = divName[j + 1].Split(' ');
                                jcSpringJoint1textBox.Text = divGain[0];
                                jcSpringJoint2textBox.Text = divGain[1];
                                jcSpringJoint3textBox.Text = divGain[2];
                                jcSpringJoint4textBox.Text = divGain[3];
                                jcSpringJoint5textBox.Text = divGain[4];
                                jcSpringJoint6textBox.Text = divGain[5];
                                jcSpringJoint7textBox.Text = divGain[6];
                                jcSpringJoint8textBox.Text = divGain[7];
                            }
                        }
                    }
                    /* End Joint Constant Setting Parameters Read */

                    /* Joint Gravity n Friction Param Parameters Read */
                    if (divName[0] == "Joint Gravity n Friction Param")
                    {
                        for (int j = 1; j < divName.Length; j++)
                        {
                            if (String.Equals(divName[j], "G.C. Gain"))
                            {
                                string[] divGain = divName[j + 1].Split(' ');
                                GCGainJoint1textBox.Text = divGain[0];
                                GCGainJoint2textBox.Text = divGain[1];
                                GCGainJoint3textBox.Text = divGain[2];
                                GCGainJoint4textBox.Text = divGain[3];
                                GCGainJoint5textBox.Text = divGain[4];
                                GCGainJoint6textBox.Text = divGain[5];
                                GCGainJoint7textBox.Text = divGain[6];
                                GCGainJoint8textBox.Text = divGain[7];
                            }
                            else if (String.Equals(divName[j], "Cur. Gain"))
                            {
                                string[] divGain = divName[j + 1].Split(' ');
                                CurGainJoint1textBox.Text = divGain[0];
                                CurGainJoint2textBox.Text = divGain[1];
                                CurGainJoint3textBox.Text = divGain[2];
                                CurGainJoint4textBox.Text = divGain[3];
                                CurGainJoint5textBox.Text = divGain[4];
                                CurGainJoint6textBox.Text = divGain[5];
                                CurGainJoint7textBox.Text = divGain[6];
                                CurGainJoint8textBox.Text = divGain[7];
                            }
                            else if (String.Equals(divName[j], "Fric. Gain"))
                            {
                                string[] divGain = divName[j + 1].Split(' ');
                                FricGainJoint1textBox.Text = divGain[0];
                                FricGainJoint2textBox.Text = divGain[1];
                                FricGainJoint3textBox.Text = divGain[2];
                                FricGainJoint4textBox.Text = divGain[3];
                                FricGainJoint5textBox.Text = divGain[4];
                                FricGainJoint6textBox.Text = divGain[5];
                                FricGainJoint7textBox.Text = divGain[6];
                                FricGainJoint8textBox.Text = divGain[7];
                            }
                        }
                    }
                    /* End Joint Gravity n Friction Param Parameters Read */

                    /* Joint Trajectory Param Parameters Read */
                    if (divName[0] == "Joint Trajectory Param")
                    {
                        for (int j = 1; j < divName.Length; j++)
                        {
                            if (String.Equals(divName[j], "Time"))
                            {
                                string[] divGain = divName[j + 1].Split(' ');
                                TrajTimeJoint1textBox.Text = divGain[0];
                                TrajTimeJoint2textBox.Text = divGain[1];
                                TrajTimeJoint3textBox.Text = divGain[2];
                                TrajTimeJoint4textBox.Text = divGain[3];
                                TrajTimeJoint5textBox.Text = divGain[4];
                                TrajTimeJoint6textBox.Text = divGain[5];
                                TrajTimeJoint7textBox.Text = divGain[6];
                                TrajTimeJoint8textBox.Text = divGain[7];
                            }
                            else if (String.Equals(divName[j], "Acc/Dec"))
                            {
                                string[] divGain = divName[j + 1].Split(' ');
                                TrajAccJoint1textBox.Text = divGain[0];
                                TrajAccJoint2textBox.Text = divGain[1];
                                TrajAccJoint3textBox.Text = divGain[2];
                                TrajAccJoint4textBox.Text = divGain[3];
                                TrajAccJoint5textBox.Text = divGain[4];
                                TrajAccJoint6textBox.Text = divGain[5];
                                TrajAccJoint7textBox.Text = divGain[6];
                                TrajAccJoint8textBox.Text = divGain[7];
                            }
                        }
                    }
                    /* End Joint Trajectory Param Parameters Read */

                    /* Cartesian Position PID gain Parameters Read */
                    if (divName[0] == "Cartesian Position PID gain")
                    {
                        for (int j = 1; j < divName.Length; j++)
                        {
                            if (String.Equals(divName[j], "P Gain"))
                            {
                                string[] divGain = divName[j + 1].Split(' ');

                                cpPgainCartesianXtextBox.Text = divGain[0];
                                cpPgainCartesianYtextBox.Text = divGain[1];

                                cpPgainCartesianZtextBox.Text = divGain[2];
                                cpPgainCartesianRolltextBox.Text = divGain[3];
                                cpPgainCartesianPitchtextBox.Text = divGain[4];
                                cpPgainCartesianYawtextBox.Text = divGain[5];
                            }
                            else if (String.Equals(divName[j], "I Gain"))
                            {
                                string[] divGain = divName[j + 1].Split(' ');
                                cpIgainCartesianXtextBox.Text = divGain[0];
                                cpIgainCartesianYtextBox.Text = divGain[1];
                                cpIgainCartesianZtextBox.Text = divGain[2];
                                cpIgainCartesianRolltextBox.Text = divGain[3];
                                cpIgainCartesianPitchtextBox.Text = divGain[4];
                                cpIgainCartesianYawtextBox.Text = divGain[5];
                            }
                            else if (String.Equals(divName[j], "D Gain"))
                            {
                                string[] divGain = divName[j + 1].Split(' ');
                                cpDgainCartesianXtextBox.Text = divGain[0];
                                cpDgainCartesianYtextBox.Text = divGain[1];
                                cpDgainCartesianZtextBox.Text = divGain[2];
                                cpDgainCartesianRolltextBox.Text = divGain[3];
                                cpDgainCartesianPitchtextBox.Text = divGain[4];
                                cpDgainCartesianYawtextBox.Text = divGain[5];
                            }
                        }
                    }
                    /* End Cartesian Position PID gain Parameters Read */

                    /* Cartesian Trajectory Parameters Read */
                    if (divName[0] == "Cartesian Trajectory Param")
                    {
                        for (int j = 1; j < divName.Length; j++)
                        {
                            if (String.Equals(divName[j], "Time"))
                            {
                                string[] divGain = divName[j + 1].Split(' ');
                                TrajTimeCartesianXtextBox.Text = divGain[0];
                                TrajTimeCartesianYtextBox.Text = divGain[1];
                                TrajTimeCartesianZtextBox.Text = divGain[2];
                                TrajTimeCartesianRolltextBox.Text = divGain[3];
                                TrajTimeCartesianPitchtextBox.Text = divGain[4];
                                TrajTimeCartesianYawtextBox.Text = divGain[5];
                            }
                            else if (String.Equals(divName[j], "Acc/Dec"))
                            {
                                string[] divGain = divName[j + 1].Split(' ');
                                TrajAccCartesianXtextBox.Text = divGain[0];
                                TrajAccCartesianYtextBox.Text = divGain[1];
                                TrajAccCartesianZtextBox.Text = divGain[2];
                                TrajAccCartesianRolltextBox.Text = divGain[3];
                                TrajAccCartesianPitchtextBox.Text = divGain[4];
                                TrajAccCartesianYawtextBox.Text = divGain[5];
                            }
                        }
                    }
                    /* End Cartesian Trajectory Parameters Read */
                }
               
                /* Close StreamReader */
                strRead.Close();
                ParametersReadpictureBox.Image = StatusOnImage;
                log.Info("Parameters File Read Complete");
            }
            else
            {
                UpdateTextBox(ErrorCodetextBox, "Parameters Read Error!!");
                ParametersReadpictureBox.Image = StatusOffImage;

                log.Error("Parameters File Read Error");
            }
        }

        private void ParametersSavebutton_Click(object sender, EventArgs e)
        {
            try
            {
                string path = "Parameters";
                string curFileName = path + "_" + DateTime.Now.ToString("yyyymmdd-HHmmdd") + ".txt";
                UpdateTextBox(ErrorCodetextBox, curFileName);

                FileStream curParamSave = new FileStream(curFileName, FileMode.Create, FileAccess.ReadWrite);
                StreamWriter curStrWrite = new StreamWriter(curParamSave);

                curStrWrite.WriteLine("Joint Position PID Gain Param");
                curStrWrite.WriteLine("P Gain");
                curStrWrite.WriteLine(jParamSet.jointPositionPgain[0] + " " + jParamSet.jointPositionPgain[1]
                    + " " + jParamSet.jointPositionPgain[2] + " " + jParamSet.jointPositionPgain[3] + " " + jParamSet.jointPositionPgain[4]
                    + " " + jParamSet.jointPositionPgain[5] + " " + jParamSet.jointPositionPgain[6] + " " + jParamSet.jointPositionPgain[7]);
                curStrWrite.WriteLine("I Gain");
                curStrWrite.WriteLine(jParamSet.jointPositionIgain[0] + " " + jParamSet.jointPositionIgain[1]
                    + " " + jParamSet.jointPositionIgain[2] + " " + jParamSet.jointPositionIgain[3] + " " + jParamSet.jointPositionIgain[4]
                    + " " + jParamSet.jointPositionIgain[5] + " " + jParamSet.jointPositionIgain[6] + " " + jParamSet.jointPositionIgain[7]);
                curStrWrite.WriteLine("D Gain");
                curStrWrite.WriteLine(jParamSet.jointPositionDgain[0] + " " + jParamSet.jointPositionDgain[1]
                    + " " + jParamSet.jointPositionDgain[2] + " " + jParamSet.jointPositionDgain[3] + " " + jParamSet.jointPositionDgain[4]
                    + " " + jParamSet.jointPositionDgain[5] + " " + jParamSet.jointPositionDgain[6] + " " + jParamSet.jointPositionDgain[7] + ";");
                curStrWrite.WriteLine("");

                curStrWrite.WriteLine("Torque PID Gain Param");
                curStrWrite.WriteLine("P Gain");
                curStrWrite.WriteLine(jParamSet.jointTorquePgain[0] + " " + jParamSet.jointTorquePgain[1]
                    + " " + jParamSet.jointTorquePgain[2] + " " + jParamSet.jointTorquePgain[3] + " " + jParamSet.jointTorquePgain[4]
                    + " " + jParamSet.jointTorquePgain[5] + " " + jParamSet.jointTorquePgain[6] + " " + jParamSet.jointTorquePgain[7]);
                curStrWrite.WriteLine("I Gain");
                curStrWrite.WriteLine(jParamSet.jointTorqueIgain[0] + " " + jParamSet.jointTorqueIgain[1]
                    + " " + jParamSet.jointTorqueIgain[2] + " " + jParamSet.jointTorqueIgain[3] + " " + jParamSet.jointTorqueIgain[4]
                    + " " + jParamSet.jointTorqueIgain[5] + " " + jParamSet.jointTorqueIgain[6] + " " + jParamSet.jointTorqueIgain[7]);
                curStrWrite.WriteLine("D Gain");
                curStrWrite.WriteLine(jParamSet.jointTorqueDgain[0] + " " + jParamSet.jointTorqueDgain[1]
                    + " " + jParamSet.jointTorqueDgain[2] + " " + jParamSet.jointTorqueDgain[3] + " " + jParamSet.jointTorqueDgain[4]
                    + " " + jParamSet.jointTorqueDgain[5] + " " + jParamSet.jointTorqueDgain[6] + " " + jParamSet.jointTorqueDgain[7] + ";");
                curStrWrite.WriteLine("");

                curStrWrite.WriteLine("Joint Constant Setting");
                curStrWrite.WriteLine("Efficiency");
                curStrWrite.WriteLine(jParamSet.jointConstantEfficiency[0] + " " + jParamSet.jointConstantEfficiency[1]
                    + " " + jParamSet.jointConstantEfficiency[2] + " " + jParamSet.jointConstantEfficiency[3] + " " + jParamSet.jointConstantEfficiency[4]
                    + " " + jParamSet.jointConstantEfficiency[5] + " " + jParamSet.jointConstantEfficiency[6] + " " + jParamSet.jointConstantEfficiency[7]);
                curStrWrite.WriteLine("Torque");
                curStrWrite.WriteLine(jParamSet.jointConstantTorque[0] + " " + jParamSet.jointConstantTorque[1]
                    + " " + jParamSet.jointConstantTorque[2] + " " + jParamSet.jointConstantTorque[3] + " " + jParamSet.jointConstantTorque[4]
                    + " " + jParamSet.jointConstantTorque[5] + " " + jParamSet.jointConstantTorque[6] + " " + jParamSet.jointConstantTorque[7]);
                curStrWrite.WriteLine("Spring");
                curStrWrite.WriteLine(jParamSet.jointConstantSpring[0] + " " + jParamSet.jointConstantSpring[1]
                    + " " + jParamSet.jointConstantSpring[2] + " " + jParamSet.jointConstantSpring[3] + " " + jParamSet.jointConstantSpring[4]
                    + " " + jParamSet.jointConstantSpring[5] + " " + jParamSet.jointConstantSpring[6] + " " + jParamSet.jointConstantSpring[7] + ";");
                curStrWrite.WriteLine("");

                curStrWrite.WriteLine("Joint Gravity n Friction Param");
                curStrWrite.WriteLine("G.C. Gain");
                curStrWrite.WriteLine(jParamSet.jointGravityGain[0] + " " + jParamSet.jointGravityGain[1]
                    + " " + jParamSet.jointGravityGain[2] + " " + jParamSet.jointGravityGain[3] + " " + jParamSet.jointGravityGain[4]
                    + " " + jParamSet.jointGravityGain[5] + " " + jParamSet.jointGravityGain[6] + " " + jParamSet.jointGravityGain[7]);
                curStrWrite.WriteLine("Cur. Gain");
                curStrWrite.WriteLine(jParamSet.jointCurrentGain[0] + " " + jParamSet.jointCurrentGain[1]
                    + " " + jParamSet.jointCurrentGain[2] + " " + jParamSet.jointCurrentGain[3] + " " + jParamSet.jointCurrentGain[4]
                    + " " + jParamSet.jointCurrentGain[5] + " " + jParamSet.jointCurrentGain[6] + " " + jParamSet.jointCurrentGain[7]);
                curStrWrite.WriteLine("Fric. Gain");
                curStrWrite.WriteLine(jParamSet.jointFrictionGain[0] + " " + jParamSet.jointFrictionGain[1]
                    + " " + jParamSet.jointFrictionGain[2] + " " + jParamSet.jointFrictionGain[3] + " " + jParamSet.jointFrictionGain[4]
                    + " " + jParamSet.jointFrictionGain[5] + " " + jParamSet.jointFrictionGain[6] + " " + jParamSet.jointFrictionGain[7] + ";");
                curStrWrite.WriteLine("");

                curStrWrite.WriteLine("Joint Trajectory Param");
                curStrWrite.WriteLine("Time");
                curStrWrite.WriteLine(jTrajSet.jointTrajectoryTime[0] + " " + jTrajSet.jointTrajectoryTime[1]
                    + " " + jTrajSet.jointTrajectoryTime[2] + " " + jTrajSet.jointTrajectoryTime[3] + " " + jTrajSet.jointTrajectoryTime[4]
                    + " " + jTrajSet.jointTrajectoryTime[5] + " " + jTrajSet.jointTrajectoryTime[6] + " " + jTrajSet.jointTrajectoryTime[7]);
                curStrWrite.WriteLine("Acc/Dec");
                curStrWrite.WriteLine(jTrajSet.jointTrajectoryAcc[0] + " " + jTrajSet.jointTrajectoryAcc[1]
                    + " " + jTrajSet.jointTrajectoryAcc[2] + " " + jTrajSet.jointTrajectoryAcc[3] + " " + jTrajSet.jointTrajectoryAcc[4]
                    + " " + jTrajSet.jointTrajectoryAcc[5] + " " + jTrajSet.jointTrajectoryAcc[6] + " " + jTrajSet.jointTrajectoryAcc[7] + ";");
                curStrWrite.WriteLine("");

                curStrWrite.WriteLine("Cartesian Position PID gain");
                curStrWrite.WriteLine("P Gain");
                curStrWrite.WriteLine(cParamSet.cartesianPositionPgain[0] + " " + cParamSet.cartesianPositionPgain[1]
                    + " " + cParamSet.cartesianPositionPgain[2] + " " + cParamSet.cartesianPositionPgain[3] + " " + cParamSet.cartesianPositionPgain[4]
                    + " " + cParamSet.cartesianPositionPgain[5]);
                curStrWrite.WriteLine("I Gain");
                curStrWrite.WriteLine(cParamSet.cartesianPositionIgain[0] + " " + cParamSet.cartesianPositionIgain[1]
                    + " " + cParamSet.cartesianPositionIgain[2] + " " + cParamSet.cartesianPositionIgain[3] + " " + cParamSet.cartesianPositionIgain[4]
                    + " " + cParamSet.cartesianPositionIgain[5]);
                curStrWrite.WriteLine("D Gain");
                curStrWrite.WriteLine(cParamSet.cartesianPositionDgain[0] + " " + cParamSet.cartesianPositionDgain[1]
                    + " " + cParamSet.cartesianPositionDgain[2] + " " + cParamSet.cartesianPositionDgain[3] + " " + cParamSet.cartesianPositionDgain[4]
                    + " " + cParamSet.cartesianPositionDgain[5] + ";");
                curStrWrite.WriteLine("");

                curStrWrite.WriteLine("Cartesian Trajectory Param");
                curStrWrite.WriteLine("Time");
                curStrWrite.WriteLine(cTrajSet.cartesianTrajectoryTime[0] + " " + cTrajSet.cartesianTrajectoryTime[1]
                    + " " + cTrajSet.cartesianTrajectoryTime[2] + " " + cTrajSet.cartesianTrajectoryTime[3] + " " + cTrajSet.cartesianTrajectoryTime[4]
                    + " " + cTrajSet.cartesianTrajectoryTime[5]);
                curStrWrite.WriteLine("Acc/Dec");
                curStrWrite.WriteLine(cTrajSet.cartesianTrajectoryAcc[0] + " " + cTrajSet.cartesianTrajectoryAcc[1]
                    + " " + cTrajSet.cartesianTrajectoryAcc[2] + " " + cTrajSet.cartesianTrajectoryAcc[3] + " " + cTrajSet.cartesianTrajectoryAcc[4]
                    + " " + cTrajSet.cartesianTrajectoryAcc[5] + ";");

                /* Close curStrWrite StreamWriter */
                curStrWrite.Close();

                FileStream ParamSave = new FileStream(path + ".txt", FileMode.Create, FileAccess.ReadWrite);
                StreamWriter StrWrite = new StreamWriter(ParamSave);

                StrWrite.WriteLine("Joint Position PID Gain Param");
                StrWrite.WriteLine("P Gain");
                StrWrite.WriteLine(jParamSet.jointPositionPgain[0] + " " + jParamSet.jointPositionPgain[1]
                    + " " + jParamSet.jointPositionPgain[2] + " " + jParamSet.jointPositionPgain[3] + " " + jParamSet.jointPositionPgain[4]
                    + " " + jParamSet.jointPositionPgain[5] + " " + jParamSet.jointPositionPgain[6] + " " + jParamSet.jointPositionPgain[7]);
                StrWrite.WriteLine("I Gain");
                StrWrite.WriteLine(jParamSet.jointPositionIgain[0] + " " + jParamSet.jointPositionIgain[1]
                    + " " + jParamSet.jointPositionIgain[2] + " " + jParamSet.jointPositionIgain[3] + " " + jParamSet.jointPositionIgain[4]
                    + " " + jParamSet.jointPositionIgain[5] + " " + jParamSet.jointPositionIgain[6] + " " + jParamSet.jointPositionIgain[7]);
                StrWrite.WriteLine("D Gain");
                StrWrite.WriteLine(jParamSet.jointPositionDgain[0] + " " + jParamSet.jointPositionDgain[1]
                    + " " + jParamSet.jointPositionDgain[2] + " " + jParamSet.jointPositionDgain[3] + " " + jParamSet.jointPositionDgain[4]
                    + " " + jParamSet.jointPositionDgain[5] + " " + jParamSet.jointPositionDgain[6] + " " + jParamSet.jointPositionDgain[7] + ";");
                StrWrite.WriteLine("");

                StrWrite.WriteLine("Torque PID Gain Param");
                StrWrite.WriteLine("P Gain");
                StrWrite.WriteLine(jParamSet.jointTorquePgain[0] + " " + jParamSet.jointTorquePgain[1]
                    + " " + jParamSet.jointTorquePgain[2] + " " + jParamSet.jointTorquePgain[3] + " " + jParamSet.jointTorquePgain[4]
                    + " " + jParamSet.jointTorquePgain[5] + " " + jParamSet.jointTorquePgain[6] + " " + jParamSet.jointTorquePgain[7]);
                StrWrite.WriteLine("I Gain");
                StrWrite.WriteLine(jParamSet.jointTorqueIgain[0] + " " + jParamSet.jointTorqueIgain[1]
                    + " " + jParamSet.jointTorqueIgain[2] + " " + jParamSet.jointTorqueIgain[3] + " " + jParamSet.jointTorqueIgain[4]
                    + " " + jParamSet.jointTorqueIgain[5] + " " + jParamSet.jointTorqueIgain[6] + " " + jParamSet.jointTorqueIgain[7]);
                StrWrite.WriteLine("D Gain");
                StrWrite.WriteLine(jParamSet.jointTorqueDgain[0] + " " + jParamSet.jointTorqueDgain[1]
                    + " " + jParamSet.jointTorqueDgain[2] + " " + jParamSet.jointTorqueDgain[3] + " " + jParamSet.jointTorqueDgain[4]
                    + " " + jParamSet.jointTorqueDgain[5] + " " + jParamSet.jointTorqueDgain[6] + " " + jParamSet.jointTorqueDgain[7] + ";");
                StrWrite.WriteLine("");

                StrWrite.WriteLine("Joint Constant Setting");
                StrWrite.WriteLine("Efficiency");
                StrWrite.WriteLine(jParamSet.jointConstantEfficiency[0] + " " + jParamSet.jointConstantEfficiency[1]
                    + " " + jParamSet.jointConstantEfficiency[2] + " " + jParamSet.jointConstantEfficiency[3] + " " + jParamSet.jointConstantEfficiency[4]
                    + " " + jParamSet.jointConstantEfficiency[5] + " " + jParamSet.jointConstantEfficiency[6] + " " + jParamSet.jointConstantEfficiency[7]);
                StrWrite.WriteLine("Torque");
                StrWrite.WriteLine(jParamSet.jointConstantTorque[0] + " " + jParamSet.jointConstantTorque[1]
                    + " " + jParamSet.jointConstantTorque[2] + " " + jParamSet.jointConstantTorque[3] + " " + jParamSet.jointConstantTorque[4]
                    + " " + jParamSet.jointConstantTorque[5] + " " + jParamSet.jointConstantTorque[6] + " " + jParamSet.jointConstantTorque[7]);
                StrWrite.WriteLine("Spring");
                StrWrite.WriteLine(jParamSet.jointConstantSpring[0] + " " + jParamSet.jointConstantSpring[1]
                    + " " + jParamSet.jointConstantSpring[2] + " " + jParamSet.jointConstantSpring[3] + " " + jParamSet.jointConstantSpring[4]
                    + " " + jParamSet.jointConstantSpring[5] + " " + jParamSet.jointConstantSpring[6] + " " + jParamSet.jointConstantSpring[7] + ";");
                StrWrite.WriteLine("");

                StrWrite.WriteLine("Joint Gravity n Friction Param");
                StrWrite.WriteLine("G.C. Gain");
                StrWrite.WriteLine(jParamSet.jointGravityGain[0] + " " + jParamSet.jointGravityGain[1]
                    + " " + jParamSet.jointGravityGain[2] + " " + jParamSet.jointGravityGain[3] + " " + jParamSet.jointGravityGain[4]
                    + " " + jParamSet.jointGravityGain[5] + " " + jParamSet.jointGravityGain[6] + " " + jParamSet.jointGravityGain[7]);
                StrWrite.WriteLine("Cur. Gain");
                StrWrite.WriteLine(jParamSet.jointCurrentGain[0] + " " + jParamSet.jointCurrentGain[1]
                    + " " + jParamSet.jointCurrentGain[2] + " " + jParamSet.jointCurrentGain[3] + " " + jParamSet.jointCurrentGain[4]
                    + " " + jParamSet.jointCurrentGain[5] + " " + jParamSet.jointCurrentGain[6] + " " + jParamSet.jointCurrentGain[7]);
                StrWrite.WriteLine("Fric. Gain");
                StrWrite.WriteLine(jParamSet.jointFrictionGain[0] + " " + jParamSet.jointFrictionGain[1]
                    + " " + jParamSet.jointFrictionGain[2] + " " + jParamSet.jointFrictionGain[3] + " " + jParamSet.jointFrictionGain[4]
                    + " " + jParamSet.jointFrictionGain[5] + " " + jParamSet.jointFrictionGain[6] + " " + jParamSet.jointFrictionGain[7] + ";");
                StrWrite.WriteLine("");

                StrWrite.WriteLine("Joint Trajectory Param");
                StrWrite.WriteLine("Time");
                StrWrite.WriteLine(jTrajSet.jointTrajectoryTime[0] + " " + jTrajSet.jointTrajectoryTime[1]
                    + " " + jTrajSet.jointTrajectoryTime[2] + " " + jTrajSet.jointTrajectoryTime[3] + " " + jTrajSet.jointTrajectoryTime[4]
                    + " " + jTrajSet.jointTrajectoryTime[5] + " " + jTrajSet.jointTrajectoryTime[6] + " " + jTrajSet.jointTrajectoryTime[7]);
                StrWrite.WriteLine("Acc/Dec");
                StrWrite.WriteLine(jTrajSet.jointTrajectoryAcc[0] + " " + jTrajSet.jointTrajectoryAcc[1]
                    + " " + jTrajSet.jointTrajectoryAcc[2] + " " + jTrajSet.jointTrajectoryAcc[3] + " " + jTrajSet.jointTrajectoryAcc[4]
                    + " " + jTrajSet.jointTrajectoryAcc[5] + " " + jTrajSet.jointTrajectoryAcc[6] + " " + jTrajSet.jointTrajectoryAcc[7] + ";");
                StrWrite.WriteLine("");

                StrWrite.WriteLine("Cartesian Position PID gain");
                StrWrite.WriteLine("P Gain");
                StrWrite.WriteLine(cParamSet.cartesianPositionPgain[0] + " " + cParamSet.cartesianPositionPgain[1]
                    + " " + cParamSet.cartesianPositionPgain[2] + " " + cParamSet.cartesianPositionPgain[3] + " " + cParamSet.cartesianPositionPgain[4]
                    + " " + cParamSet.cartesianPositionPgain[5]);
                StrWrite.WriteLine("I Gain");
                StrWrite.WriteLine(cParamSet.cartesianPositionIgain[0] + " " + cParamSet.cartesianPositionIgain[1]
                    + " " + cParamSet.cartesianPositionIgain[2] + " " + cParamSet.cartesianPositionIgain[3] + " " + cParamSet.cartesianPositionIgain[4]
                    + " " + cParamSet.cartesianPositionIgain[5]);
                StrWrite.WriteLine("D Gain");
                StrWrite.WriteLine(cParamSet.cartesianPositionDgain[0] + " " + cParamSet.cartesianPositionDgain[1]
                    + " " + cParamSet.cartesianPositionDgain[2] + " " + cParamSet.cartesianPositionDgain[3] + " " + cParamSet.cartesianPositionDgain[4]
                    + " " + cParamSet.cartesianPositionDgain[5] + ";");
                StrWrite.WriteLine("");

                StrWrite.WriteLine("Cartesian Trajectory Param");
                StrWrite.WriteLine("Time");
                StrWrite.WriteLine(cTrajSet.cartesianTrajectoryTime[0] + " " + cTrajSet.cartesianTrajectoryTime[1]
                    + " " + cTrajSet.cartesianTrajectoryTime[2] + " " + cTrajSet.cartesianTrajectoryTime[3] + " " + cTrajSet.cartesianTrajectoryTime[4]
                    + " " + cTrajSet.cartesianTrajectoryTime[5]);
                StrWrite.WriteLine("Acc/Dec");
                StrWrite.WriteLine(cTrajSet.cartesianTrajectoryAcc[0] + " " + cTrajSet.cartesianTrajectoryAcc[1]
                    + " " + cTrajSet.cartesianTrajectoryAcc[2] + " " + cTrajSet.cartesianTrajectoryAcc[3] + " " + cTrajSet.cartesianTrajectoryAcc[4]
                    + " " + cTrajSet.cartesianTrajectoryAcc[5] + ";");

                /* Close StrWrite StreamWriter */
                StrWrite.Close();
                ParametersSavepictureBox.Image = StatusOnImage;
                log.Info("Parameters File Save Complete , File Name : " + curFileName);
            }
            catch(FileNotFoundException err)
            {
                ErrorCodetextBox.Text = "File Save Error!!";
                log.Error("Parameters File Save Error");
                ParametersSavepictureBox.Image = StatusOffImage;
            }
        }

        private void DebuggingToolSW_Load(object sender, EventArgs e)
        {
            log.Info("DebuggingToolSW Loaded Complete");
            UpdateTextBox(ErrorCodetextBox,"DebuggingToolSW Loaded Complete");
        }


        private void LoggingStartbutton_Click(object sender, EventArgs e)
        {
            FileInfo file = new FileInfo("logging_.csv");

            StreamWriter logging_SW = file.CreateText();
            logging_SW.Close();

            /* Logging Status On */
            LoggingpictureBox.Image = StatusOnImage;
            StatusLogging = true;
        }

        private void LoggingStopbutton_Click(object sender, EventArgs e)
        {
            /* Logging Status Off */
            LoggingpictureBox.Image = StatusOffImage;
            StatusLogging = false;
        }

        private void JointPositionSetbutton_Click(object sender, EventArgs e)
        {
            
            jTarget.jointTarget[0] = float.Parse(PositionJoint1textBox.Text);
            jTarget.jointTarget[1] = float.Parse(PositionJoint2textBox.Text);
            jTarget.jointTarget[2] = float.Parse(PositionJoint3textBox.Text);
            jTarget.jointTarget[3] = float.Parse(PositionJoint4textBox.Text);
            jTarget.jointTarget[4] = float.Parse(PositionJoint5textBox.Text);
            jTarget.jointTarget[5] = float.Parse(PositionJoint6textBox.Text);
            jTarget.jointTarget[6] = float.Parse(PositionJoint7textBox.Text);
            jTarget.jointTarget[7] = float.Parse(PositionJoint8textBox.Text);

            /* packetType[0] Set */
            MsgStateSend.packetType[0] += 16;
        }

        private void CartesianPositionSetbutton_Click(object sender, EventArgs e)
        {  
            cTarget.pX = float.Parse(PositionCartesianPxtextBox.Text);
            cTarget.pY = float.Parse(PositionCartesianPytextBox.Text);
            cTarget.pZ = float.Parse(PositionCartesianPztextBox.Text);
            cTarget.rX = float.Parse(PositionCartesianRxtextBox.Text);
            cTarget.rX = float.Parse(PositionCartesianRxtextBox.Text);
            cTarget.rX = float.Parse(PositionCartesianRxtextBox.Text);

            /* packetType[0] Set */
            MsgStateSend.packetType[0] += 32;
        }

        private void AbPositionJoint3textBox_TextChanged(object sender, EventArgs e)
        {

        }

        private void AbPositionJoint6textBox_TextChanged(object sender, EventArgs e)
        {

        }

        private void AbPositionJoint8textBox_TextChanged(object sender, EventArgs e)
        {

        }

        private void AbVelocityJoint3textBox_TextChanged(object sender, EventArgs e)
        {

        }

        private void NumJointcomboBox_SelectedIndexChanged_1(object sender, EventArgs e)
        {
            LegDofSelect = Convert.ToInt32(NumJointcomboBox.SelectedItem.ToString());
        }
    }
}
