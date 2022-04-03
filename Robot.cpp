/*
 * FRC Team 4918 2022 Competition Robot C/C++ code.
 * (C) 2022 Team 4918, the Roboctopi, original creation on 20feb2020.
 * This code was derived from the MentorBot code.
 */

// #define SAFETY_LIMITS 1 /* Implement power/speed limits, for safety. This */
                        /* should be removed (not defined) for competition. */
// #define DISP_SMARTDASHBOARD 1  // add smartdashboard tuning displays

#define VISION_PROCESSING 1

#ifdef SAFETY_LIMITS
   const double yawRateMax      =   50.0;  // degrees/sec
   const double yawRateAccelMax =  500.0;  // degrees/sec/sec
   const double DriveMaxRPM     =  600.0;  // RPM (max for drivetrain motors)
#else
   const double yawRateMax      =  500.0;  // degrees/sec
   const double yawRateAccelMax = 3000.0;  // degrees/sec/sec
   const double DriveMaxRPM     = 5700.0;  // RPM (max for drivetrain motors)
#endif

// #include <frc/WPILib.h>  // uncomment to include everything
#include "ctre/Phoenix.h"
#include "rev/CANSparkMax.h"
#include "frc/AnalogInput.h"
#include "frc/BuiltInAccelerometer.h"
#include "frc/PneumaticsModuleType.h"
#include "frc/Compressor.h"
#include "frc/DigitalInput.h"
#include "frc/DigitalOutput.h"
#include "frc/DigitalSource.h"
#include "frc/DoubleSolenoid.h"
#include "frc/DriverStation.h"
#include "frc/Joystick.h"
#include "frc/Servo.h"
#include "frc/Solenoid.h"
#include "frc/TimedRobot.h"
#include "frc/Timer.h"
// #include "frc/RobotDrive.h"
#include "frc/drive/DifferentialDrive.h"
#include "frc/AddressableLED.h"
#include <frc/ADIS16470_IMU.h>
#ifdef DISP_SMARTDASHBOARD
   #include <frc/smartdashboard/SmartDashboard.h>
#endif
#include "cameraserver/CameraServer.h"
#include "vision/VisionRunner.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core.hpp>
// #include "rev/ColorSensorV3.h"
#include <unistd.h>
#include <sstream>
// #include <wpi/PortForwarder.h>    // Max tried this to make the limelight
                                     // more reliable, but said it didn't help

using std::cout;
using std::endl;
using std::setw;
using std::setfill;         // so we can use "setfill('0') in cout streams
using std::abs;
using namespace cv;

class Robot : public frc::TimedRobot {
 private:
         // jag; 10feb2022 -- From example SparkMax code on this webpage:
         // https://docs.revrobotics.com/sparkmax/software-resources/
         //                                       spark-max-code-examples
         // https://github.com/REVrobotics/SPARK-MAX-Examples/tree/master/
         //                                C%2B%2B/Arcade%20Drive%20With%20CAN
         // https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/
         //         C%2B%2B/Arcade%20Drive%20With%20CAN/src/main/cpp/Robot.cpp

   static const int LSMasterDeviceID = 15;
   static const int LSFollowDeviceID = 14;
   static const int RSMasterDeviceID = 10;
   static const int RSFollowDeviceID =  1;
   static const int LSClimberDeviceID = 13;
   static const int RSClimberDeviceID =  2;
   rev::CANSparkMax m_motorLSMaster{ LSMasterDeviceID,
                                     rev::CANSparkMax::MotorType::kBrushless};
   rev::CANSparkMax m_motorLSFollow{ LSFollowDeviceID,
                                     rev::CANSparkMax::MotorType::kBrushless};
   rev::CANSparkMax m_motorRSMaster{ RSMasterDeviceID,
                                     rev::CANSparkMax::MotorType::kBrushless};
   rev::CANSparkMax m_motorRSFollow{ RSFollowDeviceID,
                                     rev::CANSparkMax::MotorType::kBrushless};
   rev::CANSparkMax m_motorLSClimber{ LSClimberDeviceID,
                                     rev::CANSparkMax::MotorType::kBrushed};
   rev::CANSparkMax m_motorRSClimber{ RSClimberDeviceID,
                                     rev::CANSparkMax::MotorType::kBrushed};

   WPI_TalonSRX m_motorTopShooter{     3 };   // top motor on shooter
   WPI_TalonSRX m_motorBotShooter{    12 };   // bottom motor on shooter
   WPI_VictorSPX m_motorIntake{        4 };   // intake motor
   WPI_VictorSPX m_motorConveyMaster{ 11 };   // conveyor motor

                                                            // CTRE compressor
   frc::Compressor m_compressor{ 0, frc::PneumaticsModuleType::CTREPCM };

   frc::DoubleSolenoid m_flipperSolenoid{
                                  0, frc::PneumaticsModuleType::CTREPCM, 4, 6};

// PigeonIMU    pigeonIMU{ 1 };
   frc::ADIS16470_IMU gyro;        // ADIS16470 plugged into the MXP (SPI) port

   frc::DigitalInput conveyorDIO0{0};
   frc::DigitalInput conveyorDIO1{1};
   
   int iAutoCount;
   float drivePowerFactor = 0.8;
   frc::Joystick m_stick{0};
   frc::Joystick m_console{1};
   //frc::PowerDistributionPanel pdp{0};
   frc::AnalogInput distSensor0{0};
   frc::AnalogInput distSensor1{1};
   frc::BuiltInAccelerometer RoborioAccel{};

   std::shared_ptr<nt::NetworkTable> limenttable =
               nt::NetworkTableInstance::GetDefault().GetTable( "limelight" );

                                            // create a list of maneuver types
   enum MANEUVER_TYPE {
      M_STOP           = 0,
      M_DRIVE_STRAIGHT = 1,  // drive straight on a specified yaw direction
      M_TURN_LEFT      = 2,  // turn  left with a 12" radius to desired yaw
      M_TURN_RIGHT     = 3,  // turn right with a 12" radius to desired yaw
      M_ROTATE         = 4,  // rotate (in place) left or right to desired yaw
      M_WAIT           = 5,  // wait for a period of time (a number of ticks)
      M_LIMELOCK       = 6,  // rotate robot to point to limelight target
      M_SHOOT          = 7,  // shoot cargo into high goal
      M_TERMINATE_SEQ  = 8 
   };

                 // create a struct which can contain a full maneuver
                 // (type, distance, yaw (heading), etc.)
   struct maneuver {
      int                index;      // index of this element in an array of
                                     // maneuvers
      enum MANEUVER_TYPE type;       // type of maneuver (stop, turn, etc.)
      double             distance;   // distance in feet
      double             yaw;        // yaw angle in degrees
      bool               bDivertToCargo;  // if true, divert to a cargo ball
                                          // if one is seen by the videocamera
   };

   int mSeqIndex = 0;

   bool autoConveyor;
               // Create a sequence of full maneuvers
   struct maneuver mSeq[256] =
   {
     // Perform a maneuver until the specified distance
     // or heading has been achieved.
     // Distances are relative to the end of the previous maneuver;
     // headings are absolute, from the initial yaw when AutonomousInit()
     // was called and sCurrState.initialYaw was set.
     //
     //                                    yaw (heading)    divert
     //                          distance  (degrees,          to
     // index command             (feet)    positive left)  cargo ball?
     // ----- ----------------     ----     --------        -----
      // index 00: simple drive autonomous; no cargo collection or shooting
      {   0,  M_DRIVE_STRAIGHT,     4.0,       0.0,         false },
      {   1,  M_STOP,               0.0,       0.0,         false },
      {   2,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {   3,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {   4,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {   5,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {   6,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {   7,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {   8,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {   9,  M_TERMINATE_SEQ,      0.0,       0.0,         false },

      // index 10: 2 ball autonomous, no vision needed
      {  10,  M_DRIVE_STRAIGHT,     0.0,       0.0,         false },
      {  11,  M_DRIVE_STRAIGHT,     3.3,       0.0,         false }, // was true, removed to isolate from vision bugs
      // {  12,  M_DRIVE_STRAIGHT,    -0.3,       0.0,         false },
      {  12,  M_SHOOT,              0.0,       0.0,         true  }, // shoots with whatever is on conX (Medium)
      {  13,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  14,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  15,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  16,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  17,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  18,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  19,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  20,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  21,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  22,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  23,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  24,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  25,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  26,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  27,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  28,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  29,  M_TERMINATE_SEQ,      0.0,       0.0,         false },

      // index 30: 3 ball autonomous
      {  30,  M_DRIVE_STRAIGHT,     1.3,       0.0,         false },
      {  31,  M_DRIVE_STRAIGHT,     2.0,       0.0,         true  },
      {  32,  M_WAIT,              50.0,       0.0,         true  },
      {  33,  M_DRIVE_STRAIGHT,    -0.3,       0.0,         false }, 
      {  34,  M_LIMELOCK,           0.0,       0.0,         false },
      {  35,  M_SHOOT,              0.0,       0.0,         true  },
      {  36,  M_ROTATE,             0.0,    - 90.0,         false },
      {  37,  M_DRIVE_STRAIGHT,     4.0,    - 90.0,         false },
      {  38,  M_DRIVE_STRAIGHT,     4.0,    - 90.0,         true  },
      {  39,  M_WAIT,              50.0,       0.0,         true  },

      {  40,  M_ROTATE,             0.0,     -45.0,         false },
      {  41,  M_DRIVE_STRAIGHT,     1.0,     -45.0,         false }, 
      {  42,  M_LIMELOCK,           0.0,       0.0,         false },
      {  43,  M_SHOOT,              0.0,       0.0,         true  },
//      {  44,  M_ROTATE,             0.0,     -80.0,         false },
      {  44,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  45,  M_DRIVE_STRAIGHT,    10.0,     -80.0,         false },
      {  46,  M_DRIVE_STRAIGHT,     3.5,     -80.0,         true },
      {  47,  M_WAIT,             150.0,       0.0,         true  },
      {  48,  M_ROTATE,             0.0,     -60.0,         false },
      {  49,  M_DRIVE_STRAIGHT,    -6.0,     -60.0,         false }, 

      {  50,  M_LIMELOCK,           0.0,       0.0,         false },
      {  51,  M_SHOOT,              0.0,       0.0,         true  },
      {  52,  M_TERMINATE_SEQ,      0.0,       0.0,         false },

      {  53,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  54,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  55,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  56,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  57,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  58,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  59,  M_TERMINATE_SEQ,      0.0,       0.0,         false },

      // index 60: 5 ball autonomous
      {  60,  M_DRIVE_STRAIGHT,     1.3,       0.0,         false },
      {  61,  M_DRIVE_STRAIGHT,     2.0,       0.0,         true  },
      {  62,  M_WAIT,              50.0,       0.0,         true  },
      {  63,  M_DRIVE_STRAIGHT,    -0.3,       0.0,         false }, 
      {  64,  M_LIMELOCK,           0.0,       0.0,         false },
      {  65,  M_SHOOT,              0.0,       0.0,         false },
      {  66,  M_ROTATE,             0.0,    - 90.0,         false },
      {  67,  M_DRIVE_STRAIGHT,     4.0,    - 90.0,         false },
      {  68,  M_DRIVE_STRAIGHT,     4.0,    - 90.0,         true  },
      {  69,  M_WAIT,              50.0,       0.0,         true  },

      {  70,  M_ROTATE,             0.0,     -45.0,         false },
      {  71,  M_DRIVE_STRAIGHT,     1.0,     -45.0,         false }, 
      {  72,  M_LIMELOCK,           0.0,       0.0,         false },
      {  73,  M_SHOOT,              0.0,       0.0,         false },
      {  74,  M_ROTATE,             0.0,     -80.0,         false },
      {  75,  M_DRIVE_STRAIGHT,    10.0,     -80.0,         false },
      {  76,  M_DRIVE_STRAIGHT,     3.5,     -80.0,         true },
      {  77,  M_WAIT,             150.0,       0.0,         true  },
      {  78,  M_ROTATE,             0.0,     -60.0,         false },
      {  79,  M_DRIVE_STRAIGHT,    -6.0,     -60.0,         false }, 

      {  80,  M_LIMELOCK,           0.0,       0.0,         false },
      {  81,  M_SHOOT,              0.0,       0.0,         false },
      {  82,  M_TERMINATE_SEQ,      0.0,       0.0,         false },

      {  83,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  84,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  85,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  86,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  87,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  88,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  89,  M_TERMINATE_SEQ,      0.0,       0.0,         false },

   };


   struct sState {
      double joyX;
      double joyY;
      double joyZ;
      double conX;
      double conY;
      bool   joyButton[12];
      bool   conButton[13];
      double dLSMasterPosition;
      double dRSMasterPosition;
      int    iTSMasterPosition; // Top Shooter
      int    iBSMasterPosition; // Bottom Shooter
      double dLSMasterVelocity;
      double dRSMasterVelocity;
      int    iTSMasterVelocity; // Top Shooter
      int    iBSMasterVelocity; // Bottom Shooter
      int    iIntakePercent;
      int    iConveyPercent;
      double yawPitchRoll[3];  // position data from Pigeon or ADIS16470 IMU
      double rateXYZ[3];       // rate data from Pigeon IMU or ADIS16470 IMU;
                               // index [2] (Z axis) is yaw rate in deg/sec.
      double yawPosnEstimate;  // Current estimate of yaw position in degrees
                               // (left-positive).  This is necessary because
                               // the gyro only updates the yaw position once
                               // every 0.1 second (5 20-millisecond ticks).
      double yawRateEstimate;  // Current estimate of yaw turn rate in deg/sec
                               // (left-positive).  Necessary because the gyro
                               // only updates every 0.1 second (5 ticks).
      double initialYaw;
      bool   cargoInIntake;
      bool   cargoInPosition5;
      bool   teleop;
      double dLimelightDistanceToGoal;
      double dLimelightDesiredShooterSpeed;
   } sCurrState, sPrevState;

      // Joystick button 1 (the "trigger" switch on the front of the joystick)
      // tells the robot to align with a target (with the cargo ball if in
      // forward drive mode; and with the limelight goal retro-reflective
      // target if in reverse drive mode).
#define BUTTON_TARGET               ( sCurrState.joyButton[1] )
#define BUTTON_TARGET_PREV          ( sPrevState.joyButton[1] )
      // Joystick button 2 (the bottom-center button on back of the joystick)
      // reverses the drive direction (pushing the joystick forward drives
      // toward the cargo intake when in normal mode, and toward the limelight
      // when in reverse mode).
#define BUTTON_REVERSE              ( sCurrState.joyButton[2] )
#define BUTTON_REVERSE_PREV         ( sPrevState.joyButton[2] )
      // joystick button 3 (the topmost center button on back of joystick)
#define BUTTON_JOYTHREE             ( sCurrState.joyButton[3] )
#define BUTTON_JOYTHREE_PREV        ( sPrevState.joyButton[3] )
      // joystick button 4 (the leftmost button on back of joystick)
#define BUTTON_JOYFOUR              ( sCurrState.joyButton[4] )
#define BUTTON_JOYFOUR_PREV         ( sPrevState.joyButton[4] )
      // joystick button 5 (the rightmost button on back of joystick)
#define BUTTON_JOYFIVE              ( sCurrState.joyButton[5] )
#define BUTTON_JOYFIVE_PREV         ( sPrevState.joyButton[5] )

      // Console button  1 (the left-top pushbutton switch on the console)
      // runs the climber up.
#define BUTTON_BLUECLIMBERUP            ( sCurrState.conButton[3] )
#define BUTTON_BLUECLIMBERUP_PREV       ( sPrevState.conButton[1] )
#define BUTTON_REDCLIMBERUP            ( sCurrState.conButton[4] )
#define BUTTON_REDCLIMBERUP_PREV       ( sPrevState.conButton[2] )
      // Console button  5 (the center-top pushbutton switch on the console)
      // runs the conveyor forward.
#define BUTTON_CONVEYORFORWARD      ( sCurrState.conButton[5] )
#define BUTTON_CONVEYORFORWARD_PREV ( sPrevState.conButton[5] )
      // Console button  3 (the right-top pushbutton switch on the console)
      // runs the climber down.
#define BUTTON_BLUECLIMBERDOWN          ( sCurrState.conButton[1] )
#define BUTTON_BLUECLIMBERDOWN_PREV     ( sPrevState.conButton[3] )
#define BUTTON_REDCLIMBERDOWN           ( sCurrState.conButton[2] )
#define BUTTON_REDCLIMBERDOWN_PREV      ( sPrevState.conButton[4] )
      // Console button  4 (the centermost pushbutton switch on the console)
      // runs the conveyor backward.
#define BUTTON_CONVEYORBACKWARD        ( sCurrState.conButton[8] )
#define BUTTON_CONVEYORBACKWARD_PREV   ( sPrevState.conButton[8] )
      // No button currently set for camera switching
      // 13 does not exist
      // defines kept for future debugging use
#define BUTTON_SWITCHCAMERA           ( sCurrState.conButton[13] )
#define BUTTON_SWITCHCAMERA_PREV      ( sPrevState.conButton[13] )
      // Console button  6 (leftmost bottom pushbutton switch on the console)
      // flips up the color wheel
#define BUTTON_UPPYDOWNEY             ( sCurrState.conButton[6] )
#define BUTTON_UPPYDOWNEY_PREV        ( sPrevState.conButton[6] )
      // Console button  8 (rightmost bottom pushbutton switch on the console)
      // turns on the intake motor
#define BUTTON_RUNINTAKE              ( sCurrState.conButton[7] )
#define BUTTON_RUNINTAKE_PREV         ( sPrevState.conButton[7] )
      // Console button 12 (the leftmost missile switch)
      // turns on cartesian (field-oriented) drive.
//#define BUTTON_CARTESIANDRIVE       ( sCurrState.conButton[12] )
//#define BUTTON_CARTESIANDRIVE_PREV  ( sPrevState.conButton[12] )
#define BUTTON_SWITCH1 ( sCurrState.conButton[12] )
#define BUTTON_SWITCH2 ( sCurrState.conButton[9]  )
#define BUTTON_SWITCH3 ( sCurrState.conButton[10] )
#define BUTTON_SWITCH4 ( sCurrState.conButton[11] )

   struct sMotorState {
      double targetVelocity_UnitsPer100ms;
      double targetVelocityRPM;
      double sensorVmax, sensorVmin;
   };

   struct sMotorState LSMotorState, RSMotorState;
   struct sMotorState TSMotorState, BSMotorState;

   bool aBooleanVariable = false;    // just an example of a boolean variable
   int    iCallCount = 0;
   int    state = 0; 
   // double dTimeOfLastCall = 0.0;
   // units::time::second_t dTimeOfLastCall = (units::second_t) 0;

       // Length is 100 here, though there are 300 LEDs on the 5-meter strip,
       // because they are arranged in 3-LED sets in the WS2811 strip we have.
   static constexpr int kLEDStripLength = 100;
   static constexpr int kNumLEDBufs = 4;
   frc::AddressableLED m_led{0};  // PWM port 0 (do not connect to MXP or DIO)
   std::array<frc::AddressableLED::LEDData, kLEDStripLength>
           m_ledBuffer;  // Reuse the buffer
   std::array<frc::AddressableLED::LEDData, kLEDStripLength>
           m_ledBufferArr[kNumLEDBufs];  // Array of LED buffers
                             // Store what the last hue of the first pixel is:
   int firstLEDStripPixelHue = 0;

      // NOTE: Because of reports of the Roborio locking up sometimes when I2C
      //       is used, we have decided not to use the I2C color sensor.
      // From example code at
      // https://github.com/REVrobotics/Color-Sensor-v3-Examples/blob/master/
      //                    C%2B%2B/Read%20RGB%20Values/src/main/cpp/Robot.cpp
                              // I2C port for the Rev Robotics V3 color sensor
  // static constexpr auto i2cPort = frc::I2C::Port::kOnboard;

  /*
   * A Rev Color Sensor V3 object is constructed with an I2C port as a 
   * parameter. The device will be automatically initialized with default 
   * parameters.
   */
  // rev::ColorSensorV3 m_colorSensor{i2cPort};

   rev::SparkMaxPIDController m_LSMasterPID =
                                           m_motorLSMaster.GetPIDController();
   rev::SparkMaxPIDController m_RSMasterPID =
                                          m_motorRSMaster.GetPIDController();
   rev::SparkMaxRelativeEncoder m_LSMasterEncoder =
                                           m_motorLSMaster.GetEncoder();
   rev::SparkMaxRelativeEncoder m_RSMasterEncoder =
                                          m_motorRSMaster.GetEncoder();
   rev::SparkMaxLimitSwitch m_LSMasterForwardLimitSwitch =
                   m_motorLSMaster.GetForwardLimitSwitch(
                            rev::SparkMaxLimitSwitch::Type::kNormallyClosed );
   rev::SparkMaxLimitSwitch m_RSMasterForwardLimitSwitch =
                   m_motorRSMaster.GetForwardLimitSwitch(
                            rev::SparkMaxLimitSwitch::Type::kNormallyClosed );
   rev::SparkMaxLimitSwitch m_LSMasterReverseLimitSwitch =
                   m_motorLSMaster.GetReverseLimitSwitch(
                            rev::SparkMaxLimitSwitch::Type::kNormallyClosed );
   rev::SparkMaxLimitSwitch m_RSMasterReverseLimitSwitch =
                   m_motorRSMaster.GetReverseLimitSwitch(
                            rev::SparkMaxLimitSwitch::Type::kNormallyClosed );
   double DrivePIDkP = 0.0000;    // REV example had: 6e-5;
   double DrivePIDkI = 0.0;       // REV example had: 1e-6;
   double DrivePIDkD = 0.0;
   double DrivePIDkIz = 0.0;
   double DrivePIDkFF = 0.0017;  // REV example had: 0.000015;
   double DrivePIDkMaxOutput =  1.0;
   double DrivePIDkMinOutput = -1.0;
   rev::SparkMaxLimitSwitch m_LSClimberForwardLimitSwitch =
                   m_motorLSClimber.GetForwardLimitSwitch(
                            rev::SparkMaxLimitSwitch::Type::kNormallyClosed );
   rev::SparkMaxLimitSwitch m_LSClimberReverseLimitSwitch =
                   m_motorLSClimber.GetReverseLimitSwitch(
                            rev::SparkMaxLimitSwitch::Type::kNormallyClosed );
   rev::SparkMaxLimitSwitch m_RSClimberForwardLimitSwitch =
                   m_motorRSClimber.GetForwardLimitSwitch(
                            rev::SparkMaxLimitSwitch::Type::kNormallyClosed );
   rev::SparkMaxLimitSwitch m_RSClimberReverseLimitSwitch =
                   m_motorRSClimber.GetReverseLimitSwitch(
                            rev::SparkMaxLimitSwitch::Type::kNormallyClosed );

 public:

   static bool WeAreOnRedAlliance;

   static struct sCargoOnVideo {
      bool SeenByCamera;
      int  X;
      int  Y;
      int  Radius;
      bool SwitchToColorWheelCam;
      bool TestMode;
   } cargoOnVideo;

 private:
              /* limelight variables: x: offset from vertical centerline,   */
              /*                      y: offset from horizontal centerline, */
              /*                      a: area of target, % (0-100),         */
              /*                      v: whether the data is valid,         */
              /*                      s: skew or rotation, deg (-90-0).     */
   double limex, limey, limea, limev, limes;

#ifdef VISION_PROCESSING
      /*---------------------------------------------------------------------*/
      /* VisionThread()                                                      */
      /* This function executes as a separate thread, to take 320x240-pixel  */
      /* video frames from the USB video camera, change to grayscale,        */
      /* and send to the DriverStation. It is documented here:               */
      /* https://docs.wpilib.org/en/latest/docs/software/vision-processing/  */
      /*         introduction/using-the-cameraserver-on-the-roborio.html     */
      /* http://opencv-cpp.blogspot.com/2016/10/                             */
      /*        object-detection-and-tracking-color-separation.html          */
      /*---------------------------------------------------------------------*/
   static void VisionThread() {
      // static double dTimeOfLastCall;
      // units::time::second_t dTimeOfLastCall = (units::second_t) 0;
      static bool   prevSwitchToColorWheelCam = false;

      int minCircleRadius = 12;       // these are the limits of radius that
      int maxCircleRadius = 50;       // we look for

      int iBiggestCircleIndex = -1;   // And these are the largest radiuses
      int iBiggestCircleRadius = -1;  // that we've found

      bool bDiagnosticMode = false;

      if ( frc::DriverStation::kRed == frc::DriverStation::GetAlliance() ) {
         WeAreOnRedAlliance = true;    // we are on red alliance
      } else {
         WeAreOnRedAlliance = false;   // we are on blue alliance
      }
                  // "camera" is the camera connected to the outboard USB port
      cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture(0);
         //camera.SetResolution( 640, 480 );   // too detailed and slow
         //camera.SetResolution( 160, 120 );   // too coarse
      camera.SetResolution( 320, 240 );        // just right
                  // "camera2" is the camera connected to the inboard USB port
//    cs::UsbCamera camera2 = frc::CameraServer::StartAutomaticCapture(1);
//    camera2.SetResolution( 320, 240 );

      cargoOnVideo.SeenByCamera = false;  // Make sure no other code thinks
      cargoOnVideo.X = 0;                 // we see a cargo ball until we
      cargoOnVideo.Y = 0;                 // actually see one!
      cargoOnVideo.Radius = -1;
      cargoOnVideo.SwitchToColorWheelCam = false;
      cargoOnVideo.TestMode = false;

      cs::CvSink cvSink = frc::CameraServer::GetVideo();
      cs::CvSource outputStreamStd =
              frc::CameraServer::PutVideo( "Processed", 320, 240 );
      cv::Mat source;
      cv::Mat output;
      cv::Mat hsvImg;
      cv::Mat threshImg;
      cv::Mat *pOutput;
      cv::Mat lower_red_hue_range;
      cv::Mat upper_red_hue_range;

      std::vector<Vec3f> v3fCircles;      // 3-element vector of floats;
                                          // this will be the pass-by-reference
                                          // output of HoughCircles()
         /* HUE for YELLOW is 21-30.                                         */
         /* Adjust Hue depending on the lighting condition                   */
         /* of the environment as well as the surface of the object.         */
      // int     lowH = 19;       // Set Hue
      // int     highH = 37;      // (orig: 30)  
      // int     lowH = 14;       // Set Hue
      // int     lowH = 20;       // Set Hue
      // int     highH = 34;      // (orig: 30)  

          // jag; 08jan2022: 30/65 are good values for a yellow ball like
          // the 7" powercell in 2020 and 2021 -- but for 2022 we must
          // determine good values for a red (170-40) or a blue (80-120) ball,
          // then add initialization code to switch to the red or blue values
          // depending on which alliance we are on for each match.
          // And don't forget to check under different lighting conditions!

      int     lowHRed   = 170;   // Set Hue range for red (full range is 0-180)
      int     highHRed  =  40;   //   (these values select 200-255 OR 0-40)
                                 // Yellow would be: 30-65
      int     lowHBlue  =  80;   // Set Hue range for blue (full range is 0-255)
      int     highHBlue = 120;

      int     lowSRed   = 160;   // Set Saturation (prev: 0)
      int     highSRed  = 255;
      int     lowSBlue  = 100;   // Set Saturation (prev: 0)
      int     highSBlue = 256;

      int     lowVRed   = 160;   // Set Value (orig: 102)
      int     highVRed  = 255;   // (orig: 225)
      int     lowVBlue  = 160;   // Set Value (orig: 102)
      int     highVBlue = 255;   // (orig: 225)

      cvSink.SetSource( camera );
      while( true ) {
         static int iFrameCount = 0;
         static int iLoopCount  = 0;

         if ( cargoOnVideo.SwitchToColorWheelCam &&
              !prevSwitchToColorWheelCam ) {
                                   // This camera has a wide field of view, so
                                   // the circles will be small
            minCircleRadius = 10;  // was 20         // minimum circle radius
            maxCircleRadius = 50;  // was 56         // maximum circle radius
//       } else if ( !cargoOnVideo.SwitchToColorWheelCam &&
//                   prevSwitchToColorWheelCam ) {
//          prevSwitchToColorWheelCam = cargoOnVideo.SwitchToColorWheelCam;
                                  // This camera has a narrow field of view, so
                                  // the circles will be larger
//          minCircleRadius = 16;  // was 20         // minimum circle radius
//          maxCircleRadius = 46;  // was 56         // maximum circle radius
            bDiagnosticMode = !bDiagnosticMode;       // toggle diagnostic mode
            cout << "liv says diagnostic mode was changed." <<endl;
            cout << bDiagnosticMode <<endl; 
            cargoOnVideo.SwitchToColorWheelCam = false;
         }
         prevSwitchToColorWheelCam = cargoOnVideo.SwitchToColorWheelCam;

                   // set the pointer to the frame to be sent to DriverStation
         if ( bDiagnosticMode ) {
            pOutput = &threshImg;   // diagnostic image (shows where yellow is,
                                    //                         or red or blue))
         } else if ( cargoOnVideo.SwitchToColorWheelCam ) {
            // pOutput = &output;         // gray-scale image, for low latency
            pOutput = &source;          // full-color, direct from videocamera
         } else {
            // pOutput = &output;         // gray-scale image, for low latency
            pOutput = &source;          // full-color, direct from videocamera
         }

         usleep( 1000 );                               // wait one millisecond
         if ( cvSink.GrabFrame(source) )
         {
            // dTimeOfLastCall = frc::GetTime();
            iFrameCount++;
            // cvtColor( source, output, cv::COLOR_BGR2GRAY );
            cvtColor( source, hsvImg, cv::COLOR_BGR2HSV );
            if ( WeAreOnRedAlliance ) {
               // jag; code adapted from:
               //  https://cppsecrets.com/users/189897115119971161041034955641
               //    03495564103109971051084699111109/C00-OpenCV-cvinRange.php
               //  This solves the problem that there are two ranges of red
               //  pixels: from about 200-255 and also from 0-40
               //  Threshold the HSV image; keep only the red pixels
               cv::inRange( hsvImg, cv::Scalar(0, lowSRed, lowVRed),
                            cv::Scalar(highHRed, highSRed, highVRed),
                            lower_red_hue_range );
               cv::inRange( hsvImg, cv::Scalar( lowHRed, lowSRed, lowVRed),
                            cv::Scalar(255, highSRed, highVRed),
                            upper_red_hue_range );
                                               // Combine the above two images
               cv::addWeighted( lower_red_hue_range, 1.0,
                                upper_red_hue_range, 1.0, 0.0, threshImg );
            } else {      // else blue alliance, no image-combination required
               cv::inRange( hsvImg, cv::Scalar(lowHBlue, lowSBlue, lowVBlue),
                            cv::Scalar(highHBlue, highSBlue, highVBlue),
                            threshImg );
            }
                                                                  //Blur Effect
            cv::GaussianBlur( threshImg, threshImg, cv::Size(9, 9), 0);
            cv::dilate( threshImg, threshImg, 0 );      // Dilate Filter Effect
            cv::erode( threshImg, threshImg, 0 );       // Erode  Filter Effect
                     // fill circles vector with all circles in processed image
                     // HoughCircles() is an algorithm for detecting circles  
            cv::HoughCircles( threshImg, v3fCircles, HOUGH_GRADIENT,
                              2,          // dp: inverse accumulator resolution
                              threshImg.rows / 2,   // min dist between centers
                                                    // of the detected circles
                              100,          // param1 (edge detector threshold)
                              56,  // p2: increase this to reduce false circles
                              minCircleRadius,         // minimum circle radius
                              maxCircleRadius );       // maximum circle radius
                              // was: threshImg.rows / 4, 100, 50, 10, 800 );

            iBiggestCircleIndex = -1;     // init to an impossible index
            iBiggestCircleRadius = -1;    // init to an impossibly-small radius

                                                             // for each circle
            for ( unsigned int i = 0; i < v3fCircles.size(); i++ ) {

               // if ( 0 == iFrameCount%6007 ) {     // every 200 seconds or so
               if ( 0 == iFrameCount%1000 ) {         // every 4 seconds or so
                                      // Log the x and y position of the center
                                      // point of circle, and the radius.
                  std::cout << "Ball position X = " << v3fCircles[i][0] <<
                               ",\tY = " << v3fCircles[i][1] <<
                               ",\tRadius = " << v3fCircles[i][2] <<
                               " (" << v3fCircles[i][0] - threshImg.cols / 2 <<
                               ", " << threshImg.rows / 2 - v3fCircles[i][1] <<
                               ")" << endl;

//                  std::cout << "Video Frame Cols/Rows: ";
//                  std::cout << hsvImg.cols << "/" << hsvImg.rows << endl;
                  std::cout << "Center pixel H/S/V: ";
                                     // make a copy of the center pixel vector
                  cv::Vec3b pixel = hsvImg.at<cv::Vec3b>( hsvImg.rows/2,
                                                          hsvImg.cols/2);
                  std::cout << "pixel: " << (int)(pixel[0]) << "/" <<
                                            (int)(pixel[1]) << "/" <<
                                            (int)(pixel[2]) << endl;

                  // std::cout << "Vision Processing duration: ";
                  // std::cout << frc::GetTime() - dTimeOfLastCall << endl;
                                 // use frpc:Timer::GetFPGATimestamp() instead?
               }      // if on a 100-second boundary

                                          // if in bottom-left corner (bumper)
               if ( ( (int)v3fCircles[i][0] < 40  ) &&
                    ( 180 < (int)v3fCircles[i][1] )   ) {
                  continue;                                          // skip it
                              // else if in bottom-right corner (bumper)
               } else if ( ( 280 < (int)v3fCircles[i][0] ) &&
                           ( 180 < (int)v3fCircles[i][1] )   ) {
                  continue;                                          // skip it
               }

                     // if a bigger circle has been found than any found before
               if ( iBiggestCircleRadius < (int)v3fCircles[i][2] ) {
                  iBiggestCircleIndex = i;
                  iBiggestCircleRadius = (int)v3fCircles[i][2];
               }

                        // draw small green circle at center of object detected
               cv::circle( *pOutput,             // draw on DriverStation image
                           cv::Point( (int)v3fCircles[i][0],       // center of
                                      (int)v3fCircles[i][1] ),     // circle
                           3,                     // radius of circle in pixels
                           cv::Scalar(0, 255, 0),                 // draw green
                           FILLED );                            // thickness

                                      // draw red circle around object detected 
               cv::circle( *pOutput,          // draw on DriverStation image
                           cv::Point( (int)v3fCircles[i][0],       // center of
                                      (int)v3fCircles[i][1] ),     // circle
                           (int)v3fCircles[i][2], // radius of circle in pixels
                           cv::Scalar(0, 0, 255),                   // draw red
                           3 );                                    // thickness
            }      // for every circle found

            if ( -1 < iBiggestCircleIndex ) {  // if at least one ball was seen
                   // Then save all the info so other code can drive toward it.
                            // Convert X and Y positions so 0,0 is at center of
                            // camera image, with X increasing to the right and
                            // Y increasing up.
               cargoOnVideo.X = (int)v3fCircles[iBiggestCircleIndex][0] -
                                                         (int)threshImg.cols/2;
               cargoOnVideo.Y = (int)threshImg.rows/2 -
                                       (int)v3fCircles[iBiggestCircleIndex][1];
               cargoOnVideo.Radius = iBiggestCircleRadius;
               cargoOnVideo.SeenByCamera = true;
            } else {
               cargoOnVideo.SeenByCamera = false;
            }

            if ( cargoOnVideo.TestMode ) {                  // if in Test Mode
                           // display the H/S/V values every couple of seconds
               if ( 0 == iFrameCount%30 ) {
                  std::cout << "Center pixel H/S/V: ";
                                     // make a copy of the center pixel vector
                  cv::Vec3b pixel = hsvImg.at<cv::Vec3b>( hsvImg.rows/2,
                                                          hsvImg.cols/2 );
                  std::cout << "pixel: " << (int)(pixel[0]) << "/" <<
                                            (int)(pixel[1]) << "/" <<
                                            (int)(pixel[2]) << endl;
               }

                           // draw small white circle at center of video frame
               cv::circle( *pOutput,                 // draw on original image
                           cv::Point( hsvImg.cols/2,        // center of frame
                                      hsvImg.rows/2 ),
                           6,                    // radius of circle in pixels
                           cv::Scalar( 255, 255, 255 ),          // draw white
                           2 );                                   // thickness
                 // draw slightly-larger black circle at center of video frame
               cv::circle( *pOutput,                 // draw on original image
                           cv::Point( hsvImg.cols/2,        // center of frame
                                      hsvImg.rows/2 ),
                           8,                    // radius of circle in pixels
                           cv::Scalar( 0, 0, 0 ),                // draw black
                           2 );                                   // thickness
            }      // if in Test Mode

            outputStreamStd.PutFrame( *pOutput );

            v3fCircles.clear();

         } // if we got an image
         if ( 0 == iLoopCount%100 ) {  // every few seconds, check if we
                                       // are still on the same alliance
            if ( frc::DriverStation::kRed ==
                                          frc::DriverStation::GetAlliance() ) {
               if ( !WeAreOnRedAlliance ) {
                  WeAreOnRedAlliance = true;    // we are now on red alliance
                  cout << "VisionThread(): We are now on the Red Alliance."
                       << endl;
               }
            } else {
               if ( WeAreOnRedAlliance ) {
                  WeAreOnRedAlliance = false;   // we are now on blue alliance
                  cout << "VisionThread(): We are now on the Blue Alliance."
                       << endl;
               }
            }
         }
         iLoopCount++;
      }   // do while 
   }      // VisionThread() 
#endif  // if VISION_PROCESSING

 public:

      /*---------------------------------------------------------------------*/
      /* SwitchCameraIfNecessary()                                           */
      /* This function switches USB cameras, in case they were initialized   */
      /* in a different order than usual.                                    */
      /*---------------------------------------------------------------------*/
   void SwitchCameraIfNecessary( void ) {
             // Switch cameras if the "switch camera" button is newly-pressed.
      // cargoOnVideo.SwitchToColorWheelCam = !BUTTON_SWITCHCAMERA_PREV &&
      //                                       BUTTON_SWITCHCAMERA;
      if ( !BUTTON_SWITCHCAMERA_PREV &&
            BUTTON_SWITCHCAMERA         ) {
         cargoOnVideo.SwitchToColorWheelCam = true;

//       cargoOnVideo.SwitchToColorWheelCam =
//                 !cargoOnVideo.SwitchToColorWheelCam;

         cout << "Switching camera." << endl;
//       if ( cargoOnVideo.SwitchToColorWheelCam ) {
//          cout << "GoalCam" << endl;
//       } else {
//          cout << "CargoCam" << endl;
//       }
      }
   }


      /*---------------------------------------------------------------------*/
      /* GetAllVariables()                                                   */
      /* Retrieves all variable values from sensors, encoders,               */
      /* the limelight, etc.  It should be called at the beginning of        */
      /* every 20-millisecond tick.  Doing it this way, rather than          */
      /* having each function retrieve the values it needs when it needs     */
      /* them, should minimize CANbus traffic and keep the robot CPU fast.   */
      /*---------------------------------------------------------------------*/
   void GetAllVariables()  {

      static int iCallCount = 0;
      iCallCount++;
                                // use frc::Timer::GetFPGATimestamp() instead?
      // dTimeOfLastCall = frc::GetTime();
      //      cout << std::setw( 20 ) << std::setprecision( 16 ) <<
      //              dTimeOfLastCall << " ";


      sPrevState = sCurrState;                  // save all previous variables

      sCurrState.joyX = m_stick.GetX();
      sCurrState.joyY = m_stick.GetY();
      sCurrState.joyZ = m_stick.GetZ();
      sCurrState.conX = m_console.GetX();
      sCurrState.conY = m_console.GetY();
      for ( int iLoopCount=1; iLoopCount<=11; iLoopCount++ ) {
         sCurrState.joyButton[iLoopCount] = m_stick.GetRawButton(iLoopCount);
         sCurrState.conButton[iLoopCount] = m_console.GetRawButton(iLoopCount);
      }
      sCurrState.conButton[12] = m_console.GetRawButton(12);

                // Get encoder position values.  Units are in motor rotations.
      sCurrState.dLSMasterPosition = m_LSMasterEncoder.GetPosition();
      sCurrState.dRSMasterPosition = m_RSMasterEncoder.GetPosition();

      sCurrState.iTSMasterPosition =
                                m_motorTopShooter.GetSelectedSensorPosition();
      sCurrState.iBSMasterPosition =
                                m_motorBotShooter.GetSelectedSensorPosition();
        // Get encoder velocity values.  Units are motor rotations per minute.
        // These values may be incorrect; they are delayed ~110 millisecs.
      sCurrState.dLSMasterVelocity = m_LSMasterEncoder.GetVelocity();
      sCurrState.dRSMasterVelocity = m_RSMasterEncoder.GetVelocity();

      sCurrState.iTSMasterVelocity =
                                m_motorTopShooter.GetSelectedSensorVelocity();
      sCurrState.iBSMasterVelocity =
                                m_motorBotShooter.GetSelectedSensorVelocity();

// jag; if ( 0 == iCallCount%100 ) {
// jag;   cout << "Shoot speeds: " << sCurrState.iBSMasterVelocity * 600 / 4096;
// jag;   cout << "/" << sCurrState.iTSMasterVelocity * 600 / 4096 << endl;
// jag; }

//    pigeonIMU.GetYawPitchRoll( sCurrState.yawPitchRoll );
      sCurrState.yawPitchRoll[0] = (double)gyro.GetAngle();
      sCurrState.yawPitchRoll[1] = (double)gyro.GetRate();
      sCurrState.rateXYZ[2]      = (double)sCurrState.yawPitchRoll[1];

               // Has the gyro rate updated? (the Pigeon and the ADIS gyro
               // only update every 0.1 second).
      if ( sPrevState.rateXYZ[2] != sCurrState.rateXYZ[2] ) {
                            // yes; record actual value (correct our estimate)
         sCurrState.yawRateEstimate = sCurrState.rateXYZ[2];
         if ( 30.0 < abs( sCurrState.yawRateEstimate -
                          sPrevState.yawRateEstimate   ) ) {
         // MM; cout << "yawRate correction: " << sPrevState.yawRateEstimate <<
                     // "/" << sCurrState.yawRateEstimate << endl;
         }
      } else { // Else the gyro hasn't updated the yaw rate (it only seems
               // to update every 0.1 second), so we estimate the current
               // value ourselves here.  We calculate the rate using the
               // diff between left/right wheel encoder position changes,
               // divided by the time between ticks (20 milliseconds).
               // Remember that the left side motors of the robot drive
               // positive backward, and right side motors positive forward
               // (and forward is toward the cargo ball intake).
         double tempYawRateEstimate = 0.0;
             // get total encoder rotations since last tick (20 millisecs ago)
         double dTotalEncoderRotsLS = (double)sCurrState.dLSMasterPosition -
                                              sPrevState.dLSMasterPosition;
         double dTotalEncoderRotsRS = (double)sCurrState.dRSMasterPosition -
                                              sPrevState.dRSMasterPosition;
                    // Calculate inches of travel on each side
                    // The SparkMax encoder reports position in units of motor
                    // rotations, so we have to divide by the gear ratio
                    // (which is 10.71:1 for the drivetrain in this robot)
                    // to get wheel rotations, then multiply by (pi * 6")
                    // to get inches along the ground.
                 // (number of Rotations) * (pi) * (wheel diameter in inches) /
                 //  gear ratio
         double dDistanceDrivenLS = dTotalEncoderRotsLS * ( 3.14159 * 6.25 /
                                                                      10.71 );
         double dDistanceDrivenRS = dTotalEncoderRotsRS * ( 3.14159 * 6.25 /
                                                                      10.71 );
                  // Calculate the yaw rate.
                  // 24.0 is the width of the robot, in inches.
                  // 180.0 / 3.14159 is for the conversion to degrees.
                  // 0.02 is the number of seconds between ticks
                  // (the 20-millisecond period between calls).
                  // The "dDistanceDriven" variables are added to each other,
                  // because the left side drives positive backward, and the
                  // right side positive forward, so positive numbers in each
                  // both mean the robot is turning left (which is a positive
                  // yaw rate, according to the gyro).
         tempYawRateEstimate = ( 180.0 / 3.14159 / 24.0 / 0.02 ) *
                                    ( dDistanceDrivenLS + dDistanceDrivenRS );
                          // If the newly-calculated value is more than
                          // 30 degrees/sec different from the previous value
         if ( 50.0 < abs( tempYawRateEstimate -
                          sCurrState.yawRateEstimate ) ) {
            // cout << "Yaw rate: " << sCurrState.yawRateEstimate << ":" <<
            //                         tempYawRateEstimate << endl;
                                       // They're too far apart; average them.
            sCurrState.yawRateEstimate =
                         ( sCurrState.yawRateEstimate + tempYawRateEstimate )/2;
         } else {
            sCurrState.yawRateEstimate = tempYawRateEstimate;
         }
      }  // else the gyro didn't update the yaw rate

          // Has the gyro position updated? Like the yaw rate (handled above),
          // the gyro also only updates yaw position every 0.1 second.
      if ( sPrevState.yawPitchRoll[0] != sCurrState.yawPitchRoll[0] ) {
                            // yes; record actual value (correct our estimate)
         sCurrState.yawPosnEstimate = sCurrState.yawPitchRoll[0];
         if ( 10.0 < abs( sCurrState.yawPosnEstimate -
                          sPrevState.yawPosnEstimate   ) ) {
            cout << "yawPosn correction: " << sPrevState.yawPosnEstimate <<
                    "/" << sCurrState.yawPosnEstimate << endl;
         }
      } else { // Else the gyro hasn't updated the yaw position, so
               // we estimate the current value ourselves here
         sCurrState.yawPosnEstimate += sCurrState.yawRateEstimate * 0.02;
      }

                 // Record the positions of cargo balls in the conveyor system.
                 // The Digital I/O (DIO) connections are made to IR sensors,
                 // which return false if the IR beam is blocked (which means
                 // there is a cargo ball there) -- so we invert them here. 
      sCurrState.cargoInPosition5 = !conveyorDIO0.Get();
      sCurrState.cargoInIntake    = !conveyorDIO1.Get();

      limev = limenttable->GetNumber("tv",0.0);  // valid
      limex = limenttable->GetNumber("tx",0.0);  // x position
      limea = limenttable->GetNumber("ta",0.0);  // area
      limey = limenttable->GetNumber("ty",0.0);  // y position
      limes = limenttable->GetNumber("ts",0.0);  // skew
   }      // GetAllVariables()


      /*---------------------------------------------------------------------*/
      /* AdjustJoystickValues()                                              */
      /* Adjust the X and Y values coming from the joystick, for the zero    */
      /* deadband, joystick sensitivity, etc.                                */
      /* This should only be called or used in teleop mode; in all other     */
      /* modes the joystick values used are generated by the program itself  */
      /* and should not be adjusted.                                         */
      /*---------------------------------------------------------------------*/
   void AdjustJoystickValues( void ) {
      double adjustedJoyY;
      double adjustedJoyX;

              // our joystick increases Y when pulled BACKWARDS, and increases
              // X when pushed to the right.

      adjustedJoyY = sCurrState.joyY;
      adjustedJoyX = sCurrState.joyX;

      if ( ( -0.045 < adjustedJoyY ) && ( adjustedJoyY < 0.025 ) ) {
         adjustedJoyY = 0.0;
      } else {
         if ( false ) {     // formerly BUTTON_CARTESIANDRIVE
            adjustedJoyY = -sCurrState.joyY;               // just invert joyY
         } else {                  // else in normal robot-oriented drive mode
            if ( BUTTON_REVERSE ) {          // if "reverse" button is pressed
                    // do not invert joyY, so forward is towards the limelight
               adjustedJoyY = sCurrState.joyY*abs(sCurrState.joyY);
            } else {                      // else "reverse" button not pressed
                     // invert joyY so forward is towards the intake
               adjustedJoyY = -sCurrState.joyY*abs(sCurrState.joyY);
            }
         }
      }
      if ( ( -0.105 < adjustedJoyX ) && ( adjustedJoyX < 0.025 ) ) {
         adjustedJoyX = 0.0;
      } else {
         if ( true ) { // formerly !BUTTON_CARTESIANDRIVE
            adjustedJoyX = sCurrState.joyX*abs(sCurrState.joyX);
         }
      }
#ifdef SAFETY_LIMITS
                        // multiply both settings by 0.75 for extra safety
      adjustedJoyY *= .75;
      adjustedJoyX *= .75;
#endif

          // Squaring the joystick inputs (while preserving the sign)
          // decreases the input sensitivity at low speeds, allowing
          // more accurate driver control.  It increases fine control
          // at low speeds, but still permits full power when the
          // joystick is thrown all the way to its limit.
      if ( true ) { // formerly !BUTTON_CARTESIANDRIVE
         adjustedJoyY = std::copysign( adjustedJoyY * adjustedJoyY,
                                       adjustedJoyY );
         adjustedJoyX = std::copysign( adjustedJoyX * adjustedJoyX,
                                       adjustedJoyX );
      }

      sCurrState.joyY = adjustedJoyY;
      sCurrState.joyX = adjustedJoyX;

   }      // AdjustJoystickValues()


#ifdef DISP_SMARTDASHBOARD
   void AdjustPIDValues( void ) {
                              // read PID coefficients from the SmartDashboard
      double p   = frc::SmartDashboard::GetNumber( "P Gain", 0 );
      double i   = frc::SmartDashboard::GetNumber( "I Gain", 0 );
      double d   = frc::SmartDashboard::GetNumber( "D Gain", 0 );
      double iz  = frc::SmartDashboard::GetNumber( "I Zone", 0 );
      double ff  = frc::SmartDashboard::GetNumber( "Feed Forward", 0 );
      double min = frc::SmartDashboard::GetNumber( "Min Output", 0 );
      double max = frc::SmartDashboard::GetNumber( "Max Output", 0 );

                      // If PID coefficients from SmartDashboard have changed,
                      // write new values to controllers.
      if ( p != DrivePIDkP ) {
         DrivePIDkP = p;
         m_LSMasterPID.SetP( DrivePIDkP );
         m_RSMasterPID.SetP( DrivePIDkP );
      }
      if ( i != DrivePIDkI ) {
         DrivePIDkI = i;
         m_LSMasterPID.SetI( DrivePIDkI );
         m_RSMasterPID.SetI( DrivePIDkI );
      }
      if ( d != DrivePIDkD ) {
         DrivePIDkD = d;
         m_LSMasterPID.SetD( DrivePIDkD );
         m_RSMasterPID.SetD( DrivePIDkD );
      }
      if ( iz != DrivePIDkIz ) {
         DrivePIDkIz = iz;
         m_LSMasterPID.SetIZone( DrivePIDkIz );
         m_RSMasterPID.SetIZone( DrivePIDkIz );
      }
      if ( ff != DrivePIDkFF ) {
         DrivePIDkFF = ff;
         m_LSMasterPID.SetFF( DrivePIDkFF );
         m_RSMasterPID.SetFF( DrivePIDkFF );
      }
      if ( ( min != DrivePIDkMinOutput ) || ( max != DrivePIDkMaxOutput ) ) {
         DrivePIDkMinOutput = min;
         DrivePIDkMaxOutput = max;
         m_LSMasterPID.SetOutputRange( min, max );
         m_RSMasterPID.SetOutputRange( min, max );
      }
   }   // AdjustPIDValues()
#endif

      /*---------------------------------------------------------------------*/
      /* JoystickDisplay()                                                   */
      /* Display all the joystick values on the console log.                 */
      /*---------------------------------------------------------------------*/
   void JoystickDisplay( void ) {
         cout << "joy (y/x): " << setw(8) << sCurrState.joyY << "/" <<
                 setw(8) << sCurrState.joyX << endl;
   }


      /*---------------------------------------------------------------------*/
      /* IMUOrientationDisplay()                                             */
      /* Display all the yaw pitch & roll values on the console log.         */
      /*---------------------------------------------------------------------*/
   void IMUOrientationDisplay( void ) {
//       cout << "pigeonIMU (yaw/pitch/roll): " <<
         cout << "ADIS_IMU (yaw/pitch/roll+yrate): " <<
               sCurrState.yawPitchRoll[0] << "/" <<
               sCurrState.yawPitchRoll[1] << "/" <<
               sCurrState.yawPitchRoll[2] << "+" <<
               sCurrState.rateXYZ[2] << endl;
//         cout << "pigeontemp: " << pigeonIMU.GetTemp() << endl; 
//         cout << "gyro_temp: " << gyro.GetTemp() << endl; 
   }      // IMUOrientationDisplay()


      /*---------------------------------------------------------------------*/
      /* MotorDisplaySpark()                                                 */
      /* Display all the current motor values for a specified                */
      /* SparkMax-controlled motor on the console log.                       */
      /*---------------------------------------------------------------------*/
   void MotorDisplaySpark( const char * cTitle,
                           rev::CANSparkMax & m_motor,
                           rev::SparkMaxRelativeEncoder & m_motorEncoder,
                           struct sMotorState & sMState ) {
      double motorVelocity = m_motorEncoder.GetVelocity();
      cout << cTitle << " vel(min:max)tgt/% A (pos): ";
      cout << setw(5) << motorVelocity;
      cout << "(" << setw(5) << sMState.sensorVmin << ":";
      cout <<        setw(5) << sMState.sensorVmax << ")";
      cout << setw(5) << sMState.targetVelocityRPM << "/ ";
      cout << setw(3) << m_motor.GetAppliedOutput() << "% ";
      cout << setw(5) << m_motor.GetOutputCurrent() << "A ";
      cout << "(" << setw(10) << m_motorEncoder.GetPosition() << ")";
      cout << endl;
      sMState.sensorVmin = sMState.sensorVmax = motorVelocity;
   }      // MotorDisplaySpark()


      /*---------------------------------------------------------------------*/
      /* MotorDisplay()                                                      */
      /* Display all the current motor values for a specified Talon- or      */
      /* Victor-driven motor on the console log.                             */
      /*---------------------------------------------------------------------*/
   void MotorDisplay( const char * cTitle,
                      WPI_TalonSRX & m_motor,
                      struct sMotorState & sMState ) {
      double motorVelocity = m_motor.GetSelectedSensorVelocity();
      cout << cTitle << " vel(min:max)tgt/% A (pos): ";
      cout << setw(5) << motorVelocity*600/4096;
      cout << "(" << setw(5) << sMState.sensorVmin*600/4096 << ":";
      cout <<        setw(5) << sMState.sensorVmax*600/4096 << ")";
      cout << setw(5) << sMState.targetVelocity_UnitsPer100ms*600/4096 << "/ ";
      cout << setw(3) << m_motor.GetMotorOutputPercent() << "% ";
      cout << setw(5) << m_motor.GetStatorCurrent() << "A ";
      cout << "(" << setw(10) << m_motor.GetSelectedSensorPosition() << ")";
      cout << endl;
      sMState.sensorVmin = sMState.sensorVmax = motorVelocity;
   }      // MotorDisplay()


      /*---------------------------------------------------------------------*/
      /* motorFindMinMaxVelocitySpark()                                      */
      /* Keep a motor's Vmin and Vmax updated, for display on the            */
      /* console log (this version is for SparkMax-driven motors).           */
      /*---------------------------------------------------------------------*/
   void motorFindMinMaxVelocitySpark(
                                rev::SparkMaxRelativeEncoder & m_motorEncoder,
                                struct sMotorState & sMState ) {
      double motorVelocity = m_motorEncoder.GetVelocity();
      if ( motorVelocity < sMState.sensorVmin ) {
         sMState.sensorVmin = motorVelocity;
      } else if ( sMState.sensorVmax < motorVelocity ) {
         sMState.sensorVmax = motorVelocity;
      }
   }      // motorFindMinMaxVelocitySpark()


      /*---------------------------------------------------------------------*/
      /* motorFindMinMaxVelocity()                                           */
      /* Keep a motor's Vmin and Vmax updated, for display on the            */
      /* console log (this version is for Talon- or Victor-driven motors).   */
      /*---------------------------------------------------------------------*/
   void motorFindMinMaxVelocity( WPI_TalonSRX & m_motor,
                                 struct sMotorState & sMState ) {
      double motorVelocity = m_motor.GetSelectedSensorVelocity();
      if ( motorVelocity < sMState.sensorVmin ) {
         sMState.sensorVmin = motorVelocity;
      } else if ( sMState.sensorVmax < motorVelocity ) {
         sMState.sensorVmax = motorVelocity;
      }
   }      // motorFindMinMaxVelocity()


      /*---------------------------------------------------------------------*/
      /* Team4918Drive()                                                     */
      /* This function implements something similar to                       */
      /* frc::DifferentialDrive::ArcadeDrive() or CurvatureDrive, but        */
      /* customized for Team 4918's Robot.  In particular, it uses           */
      /* ControlMode::Velocity rather than ControlMode::PercentOutput        */
      /* to drive the motors.  So this should be much more precise for the   */
      /* drivers.                                                            */
      /* It uses desiredForward (-1.0 to +1.0) as the desired forward speed, */
      /* and desiredTurn (-1.0 to +1.0, positive to the right) for the       */
      /* desired turn rate.                                                  */
      /* "Forward" (positive desiredForward) in this function is always      */
      /* towards the cargo ball intake (regardless of BUTTON_REVERSE), and   */
      /* positive desiredTurn is always towards the right.                   */
      /* Added use of joyZ axis (the throttle paddle on the joystick) to     */
      /* scale the forward and turn speed; pushing that throttle forward     */
      /* increases the sensitivity of the main joystick.                     */
      /*---------------------------------------------------------------------*/
   void Team4918Drive( double desiredForward, double desiredTurn ) {
                  /* To drive the robot using the drive motor encoders,     */
                  /* specifying each motor separately, we must              */
                  /* convert the desired speed to units / 100ms,            */
                  /* because the velocity setpoint is in units/100ms.       */
                  /* For example, to convert 500 RPM to units / 100ms:      */
                  /* 4096 Units/Rev * 500 RPM / 600 100ms/min               */
                  /* So code to drive the robot at up to 500 rpm in either  */
                  /* direction could look like this:                        */
      double LSMasterOutput = 0.0;
      double RSMasterOutput = 0.0;
      double dThrottle;
      // m_drive.StopMotor();
                          // Use throttle (Z-axis at base of joystick)
                          // to give the driver very precise control of speed.
                          // The joyZ value runs from -1.0 to +1.0, and is
                          // negative when pushed forward, and positive when
                          // pushed backward -- so change range to 0.1-1.0,
                          // with 0.1 when pulled all the way backward and
                          // 1.0 when pushed all the way forward -- then
                          // multiply that number by both axes to scale them.
      dThrottle = ( 1.0-sCurrState.joyZ ) / 2.0;
      
                    // square it so its more sensitive at low throttle settings
      dThrottle = dThrottle * dThrottle;
      dThrottle = std::max( 0.01, dThrottle );
      dThrottle = std::min( 1.0,  dThrottle );
      desiredForward = desiredForward * dThrottle;
      desiredTurn    = ( desiredTurn    * dThrottle ) * 0.9;
      // if ( 19 == iCallCount%200 ) {
         // cout << "Throttle/forward/turn " << dThrottle << "/"
         //      << desiredForward << "/" << desiredTurn << endl;
      // }
#ifdef SAFETY_LIMITS
                           // for safety: allow limited range only -0.5 to 0.5
      desiredForward = std::min(  0.5, desiredForward );
      desiredTurn    = std::min(  0.5, desiredTurn  );
      desiredForward = std::max( -0.5, desiredForward );
      desiredTurn    = std::max( -0.5, desiredTurn  );
#else
                           // safety mode off: allow full range of -1.0 to 1.0
      desiredForward = std::min(  1.0, desiredForward );
      desiredTurn    = std::min(  0.9, desiredTurn  );
      desiredForward = std::max( -1.0, desiredForward );
      desiredTurn    = std::max( -0.9, desiredTurn  );
#endif
      LSMasterOutput = -desiredForward - desiredTurn;
      RSMasterOutput = +desiredForward - desiredTurn;
      LSMasterOutput = std::min(  1.0, LSMasterOutput );
      RSMasterOutput = std::min(  1.0, RSMasterOutput );
      LSMasterOutput = std::max( -1.0, LSMasterOutput );
      RSMasterOutput = std::max( -1.0, RSMasterOutput );
// jag; 10feb2022 -- for testing the new NEO motor, controlled by a SparkMax:
// These are simple "PercentOutput" outputs, rather than from a PID.
//    m_motorLSMaster.Set( LSMasterOutput  );
//    m_motorRSMaster.Set( RSMasterOutput );
        // jag; from https://github.com/REVRobotics/SPARK-MAX-Examples/ :
           // PIDController objects are commanded to a set point using the
           // SetReference() method.
           // The first parameter is the value of the set point, whose units
           // vary depending on the control type set in the second parameter.
           // The second parameter is the control type, which can be set
           // to one of these values:
           //    rev::CANSparkMax::ControlType::kDutyCycle
           //    rev::CANSparkMax::ControlType::kPosition
           //    rev::CANSparkMax::ControlType::kVelocity
           //    rev::CANSparkMax::ControlType::kVoltage
           //    rev::CANSparkMax::ControlType::kCurrent
           //        (and kSmartMotion and kSmartVelocity)
           // Note that the setpoint is specified in RPM, not encoder ticks.
      // This seems to chatter the motors:
      m_LSMasterPID.SetReference( LSMasterOutput * DriveMaxRPM,
                                  rev::CANSparkMax::ControlType::kVelocity );
      m_RSMasterPID.SetReference( RSMasterOutput * DriveMaxRPM,
                                  rev::CANSparkMax::ControlType::kVelocity );
      // These other drive methods all still chattered the drive motors.
      // m_LSMasterPID.SetReference( LSMasterOutput * 12.0,
      //                             rev::CANSparkMax::ControlType::kVoltage );
      // m_RSMasterPID.SetReference( RSMasterOutput * 12.0,
      //                             rev::CANSparkMax::ControlType::kVoltage );
      // This was maybe a little better (?)
      // m_motorLSMaster.Set( LSMasterOutput );
      // m_motorRSMaster.Set( RSMasterOutput );
      // m_LSMasterPID.SetReference( LSMasterOutput * 30.0,
      //                             rev::CANSparkMax::ControlType::kCurrent );
      // m_RSMasterPID.SetReference( RSMasterOutput * 30.0,
      //                             rev::CANSparkMax::ControlType::kCurrent );
      LSMotorState.targetVelocityRPM = LSMasterOutput * DriveMaxRPM; 
      RSMotorState.targetVelocityRPM = RSMasterOutput * DriveMaxRPM;
#ifdef DISP_SMARTDASHBOARD
                         // Display Drive Motor Speeds on the SmartDashboard.
                         // NOTE: There are comments on ChiefDelphi that
                         //       the REV encoders have an unavoidable
                         //       measurement delay of ~110 milliseconds, and
                         //       that delay affects internal velocity loops.
                         //       We may have to adjust for that.
      frc::SmartDashboard::PutNumber( "LeftDrive Desired Speed",
                                      LSMasterOutput * DriveMaxRPM );
      frc::SmartDashboard::PutNumber( "LeftDrive Current Speed",
                                      m_LSMasterEncoder.GetVelocity() );
      frc::SmartDashboard::PutNumber( "LeftDrive Current Position",
                                      m_LSMasterEncoder.GetPosition() );
      frc::SmartDashboard::PutNumber( "RightDrive Desired Speed",
                                      RSMasterOutput * DriveMaxRPM );
      frc::SmartDashboard::PutNumber( "RightDrive Current Speed",
                                      m_RSMasterEncoder.GetVelocity() );
      frc::SmartDashboard::PutNumber( "RightDrive Current Position",
                                      m_RSMasterEncoder.GetPosition() );
#endif
   }      // Team4918Drive()


      /*---------------------------------------------------------------------*/
      /* DriveCartesianByJoystick()                                          */
      /* Drive robot according to the commanded Y and X joystick position,   */
      /* where the position of the joystick indicates which direction of the */
      /* field to drive toward, regardless of which way the robot is         */
      /* currently pointed.                                                  */
      /* The baseline (zero degree) direction is the direction the robot was */
      /* pointed when first powered up, which is expected to be in the       */
      /* direction the driver is facing.                                     */
      /*---------------------------------------------------------------------*/
   void DriveCartesianByJoystick( void ) {
      double desiredHeading;
      double desiredSpeed;
      double currentHeading;
      double headingDifference;

      desiredSpeed = sqrt( sCurrState.joyY * sCurrState.joyY +
                           sCurrState.joyX * sCurrState.joyX   );
                       // Protect against dividing by zero (giving the atan2()
                       // function below arguments which are too small).
      if ( 0.1 < desiredSpeed ) {
              // Our joystick increases Y when pulled BACKWARDS, and increases
              // X when pushed to the right.  This should have already been
              // adjusted by the "AdjustJoystickValues() function if we are
              // in Teleop mode, so this function assumes positive joyY is
              // forward, and positive joyX is to the right.
              // In this case:
              //    positive joyY points toward the 0-degree heading,
              //    negative joyY points toward 180 degrees,
              //    positive joyX points toward the 270-degree heading, and
              //    negative joyX points toward  90 degrees,
         desiredHeading = atan2( -sCurrState.joyX, sCurrState.joyY ) *
                                                          ( 180.0 / 3.14159 );
                                // desiredHeading ranges from -180.0 to +180.0
                                // convert it to range from 0.0 to 360.0
                                // NOT NECESSARY.
         // desiredHeading = std::fmod( desiredHeading + 360.0, 360.0 );

                     // currentHeading is the current heading of the robot
                     // within the range 0.0 to 360.0
         // currentHeading =  std::fmod( sCurrState.yawPosnEstimate, 360.0 );
            // Don't bother correcting the range to 0-360; the arithmetic
            // below works even if currentHeading ranges from -36000 to +36000
         if ( BUTTON_REVERSE ) {           // If we're driving in reverse mode
                        // then adjust so "currentHeading" is in the direction
                        // of the limelight, rather than the intake,
            currentHeading =  sCurrState.yawPosnEstimate + 180.0;
                        // and speed is negative, so robot drives backward.
            desiredSpeed = desiredSpeed * -1.0;
         } else {
                        // else currentheading is towards the intake
            currentHeading =  sCurrState.yawPosnEstimate;
         }

         headingDifference =
                std::fmod( desiredHeading - currentHeading + 36000.0, 360.0 );

         // cout << "des/cur/diff Hdg: " << desiredHeading <<
         //                          "/" << currentHeading <<
         //                          "/" << headingDifference << endl;

         if ( headingDifference < 180.0 ) {
            desiredHeading = sCurrState.yawPosnEstimate + headingDifference;
         } else {
            desiredHeading = sCurrState.yawPosnEstimate + headingDifference -
                                                                        360.0;
         }

                       // If the robot is currently pointed within 20 degrees
                       // of the desired heading
         if ( abs( desiredHeading - sCurrState.yawPosnEstimate ) < 20.0 ) {
            DriveToDistance( desiredHeading,       // heading in degrees
                             desiredSpeed * 4.0,   // distance in feet,
                             false,         // whether to divert to cargo ball
                             true  );       // whether to initialize distance
         } else {
                                              // (handle initialization later)
            TurnToHeading( desiredHeading, false );
         }
      } else {
         Team4918Drive( 0.0, 0.0 );  // Stop the robot
      }

   }      // DriveCartesianByJoystick()


      /*---------------------------------------------------------------------*/
      /* DriveByJoystick()                                                   */
      /* Drive robot according to the commanded Y and X joystick position.   */
      /*---------------------------------------------------------------------*/
   void DriveByJoystick( void ) {
      double desiredForward;
      double desiredTurn;

              // Our joystick increases Y when pulled BACKWARDS, and increases
              // X when pushed to the right.  This should have already been
              // adjusted by the "AdjustJoystickValues() function if we are
              // in Teleop mode, so this function assumes positive joyY is
              // forward, and positive joyX is to the right.
              // (Or in Cartesian, field-oriented drive mode:
              //    positive joyY is toward the 0-degree direction, and
              //    positive joyX is toward the right
              //                                (the 270-degree direction) ).

      if ( false ) { //formerly BUTTON_CARTESIANDRIVE
         DriveCartesianByJoystick();
      } else {
         desiredForward = sCurrState.joyY;
         desiredTurn    = sCurrState.joyX;

         Team4918Drive( desiredForward, desiredTurn );
      }
   }      // DriveByJoystick()


      // --------------------------------------------------------------------
      // DriveToLimelightTarget()
      // DriveToLimelightTarget() drives autonomously towards a limelight
      // vision target.                                                    
      // It returns true if the limelight data is valid, false otherwise.
      // Maggie and Jeff found these limelight values work pretty well
      // for the 2022 goal retro-reflective tape shapes:
      // Then Maggie and Max made some changes during the Auburn competition,
      // and those values are in the second column, where they differ from
      // the original values.  The third column has settings we went back to
      // in the Robot lab (again, just the values that are different from
      // the 1st or 2nd column).  We noticed that the Exposure was set to 95
      // at Auburn, which we thought may have been a mistake (it's easy to
      // bump the values accidentally), and that was why the robot was
      // continually hunting in yaw when trying to lock using the limelight,
      // both at Auburn and in our lab, until we changed it back to 5 and
      // it behaved well again.
      // The third column is our best guess right now, but we will check all
      // values and maybe tweak them at the Bonney Lake competition.
      //                           Orig       Auburn      31mar2022
      //                           values     values (where different)
      //    Pipeline type: Limelight standard
      //       Source Image: Camera
      //       Resolutions 320x240 90fps
      //       LEDs: On
      //       Orientation: Normal
      //       Exposure: 2                      95           5
      //       Black Level Offset: 24                       15
      //       Red Balance: 1428
      //       Blue Balance: 1428
      //       Thresholding page:
      //       Hue: 40-87                                   55-93
      //       Saturation: 99-255
      //       Value: 106-255
      //       Erosion Steps: 0
      //       Dilation Steps: 1
      //       X-Crop: -1 to 1
      //       Y-Crop: -1 to 1
      //       Invert Hue Selection (for red-colored objects): No
      //    Countour filtering page:
      //       Sort Mode: Highest
      //       Area Percentage of Image: 0-.4712
      //       Fullness (percentage of blue rectangle): 10-100
      //       Width to Height Ratio (yellow rectangle): .5652 - 4.0302
      //       Direction Filter: None
      //       Smart Speckle Rejection: 0
      //       Target Grouping: dual-target      Smart target group:
      //       Dual-target filters:
      //       Intersection Filter: None
      //                                Smart-target filters:
      //                                  group size: 1-7                1-5
      //                                  group outlier rejector: none
      //                                  horiz. outlier filter: 1.5
      //                                  vert.  outlier filter: 1.5
      //    Output page:
      //       Targeting Region: Center
      //       Send Raw Corners? No
      //       Send Raw Contours? No
      //       Crosshair Mode: Single Crosshair
      //       Crosshair A:
      //          (all settings: X, Y: 0.00
      //       Hardware Panning:
      //          (all settings: Pan X, Pan Y: 0.00
      //    3D Experimental page:
      //       Compute 3D? No
      //       Force Convex? Yes
      //       Contour Simplification Percentage? 5.0
      //       Acceptable Error (px)? 8.0
      //       Goal Z-offset? 0.0
      //       Bind Target? Yes
      // ---------------------------------------------------------------------
   bool DriveToLimelightTarget()  {
      bool          returnVal  = true;
      static int    iCallCount = 0;
      static double dEventualYawPosition = 0.0;
      static double dDesiredYaw          = 0.0;
      static double dDesiredTurn         = 0.0;

      iCallCount++;

      limenttable->PutNumber( "ledMode", 3 );                   // turn LEDs on

      if ( 1 == limev )  {                       // if limelight data is valid
         double dDesiredYawInstant = 0.0;
         double autoDriveSpeed;
             // limea is the area of the target seen by the limelight camera
             // and is in percent (between 0 and 100) of the whole screen area.
             // limey is the height above the center of the field of view
             // Could change the if/else statements below to calculate
             // autoDriveSpeed by using a math expression based on limey.
         // if        ( 15 < limey ) {
         //    autoDriveSpeed = -0.1;
         // } else if ( 12 < limey )  { // if we're really close...
         //    autoDriveSpeed = 0.0;    //   stop (or 0.08 to go slow)
         // } else if (  8 < limey ) {  // if we're a little farther...
         //    autoDriveSpeed = 0.1;    //   go a little faster
         // } else if (  2 < limey ) {  // if we're farther still...
         //    autoDriveSpeed = 0.15;   //   go a little faster still
         // } else {                    // else we must be really far...
         //    autoDriveSpeed = 0.20;   //   go as fast as we dare
         // }
                     // Drive forward/back as commanded by Y joystick
         autoDriveSpeed = -sCurrState.joyY;

                          // LATER: May want to modify autoDriveSpeed depending
                          // on the distance from the target determined
                          // by sonar transducers.

                 // Calculate the Yaw Position we'd arrive at (when we came to
                 // a stop) if we simply decelerated our turn rate at
                 // 600.0 degrees/sec/sec, starting right now.
                 // This is conservative; I have measured EZ-PZ angular
                 // acceleration rate at about 2500 deg/sec/sec
                 // (max yaw rate is about 500 deg/sec).
         dEventualYawPosition = sCurrState.yawPosnEstimate +
                                ( 0.5 / 600.0 ) *
                                          sCurrState.yawRateEstimate *
                                          abs(sCurrState.yawRateEstimate);
                 // yawPosnEstimate is left-turn-positive, but
                 // limex is left-turn-negative -- so we subtract limex to get
                 // a left-turn-positive result.
         dDesiredYawInstant = sCurrState.yawPosnEstimate - limex;
         dDesiredYaw = ( 19 * dDesiredYaw + dDesiredYawInstant ) / 20.0;
             // Now calculate how hard we want to turn (right-turn-positive).
             // Since we now account for yaw rate in deciding when to end the
             // turn, we can be a little more aggressive in starting the turn:
         dDesiredTurn = ( dEventualYawPosition - dDesiredYaw ) * 1.0/60.0;
         dDesiredTurn = std::max( -1.0, dDesiredTurn );
         dDesiredTurn = std::min(  1.0, dDesiredTurn );
      if ( 0 == iCallCount%100 )  {
         cout << "dDesiredTurn: " << dEventualYawPosition << ":" << dDesiredYaw << ":" << dDesiredTurn << endl;
      }

         // May have to add/subtract a constant from limex here, to account
         // for the offset of the camera away from the centerline of the robot.
         if ( aBooleanVariable ) {
            Team4918Drive( -autoDriveSpeed, 0.0 );
         } else {
                                // drive forward or backward, as commanded
                                // by the operator, turning towards the target
            Team4918Drive( -autoDriveSpeed, dDesiredTurn );
         }

      } else {                    // else limelight data is not valid any more
         if ( !sCurrState.teleop ) {  // If not in teleop mode
            sCurrState.joyY = 0.3;    // then simulate some joystick settings
            sCurrState.joyX = 0.0;
         }
                      // No limelight target seen; drive according to joystick
         DriveByJoystick();
         returnVal = false;
      }

      if ( 0 == iCallCount%100 )  {
         cout << "lime: " << limev << ":" << limex << "/" << limey;
         cout << ", " << limea << ":" << limes << "." << endl;
      }

      return returnVal;

   }  /* DriveToLimelightTarget() */


      /*---------------------------------------------------------------------*/
      /* DriveToCargo()                                                      */
      /* DriveToCargo() drives autonomously towards a vision target.         */
      /* It returns true if the usb vision data has detected a cargo ball,   */
      /* false otherwise.                                                    */
      /*---------------------------------------------------------------------*/
   bool DriveToCargo()  {

      bool returnVal = true;
      static int  iCallCount = 0;

      static double dDesiredYaw;   // degrees absolute yaw, positive to left
      static double dDesiredTurn;  // -1.0 to +1.0, positive is to the right
      static double dEventualYawPosition = 0;   // degrees, positive to left

      iCallCount++;
//    if (  sCurrState.cargoInIntake ) {          // if cargo ball in intake
//       m_motorIntake.Set( ControlMode::PercentOutput,  0.6 ); // be gentle
//    } else {
//       m_motorIntake.Set( ControlMode::PercentOutput,  0.6 ); // be strong
//    }
      RunIntake();
      RunConveyor();

      if ( cargoOnVideo.SeenByCamera ) {          // if USB video data is valid
         double autoDriveSpeed;
             // If cargoOnVideo.SeenByCamera is true, that means that the
             // vision-processing code in VisionThread() has found a red/blue
             // circle in the latest video frame from the USB videocamera (and
             // we hope that red/blue circle is a cargo ball).
             // cargoOnVideo.Y is the Y-position in the video frame of the
             //    cargo ball; the range is from -120 to +120 (pixels).
             // cargoOnVideo.X is the X-position in the video frame of the
             //    cargo ball; the range is from -160 to +160 (pixels).
             // cargoOnVideo.Radius is the radius of the cargo ball;
             //    the range is from 20 to 70 (pixels).
             // In the code below, we use those cargoOnVideo values to
             // determine how fast and in which direction to drive, to go
             // towards the cargo ball.
             // jag; 22mar2021: all these values have been changed; it may be
             // useful to compare with the original working code in
             // ~/Desktop/2020-Robot/Robot.cpp
             // Latest change: now we let the driver maintain joystick control
             // of the forward/backward speeds; all we do is control the turn.
         autoDriveSpeed = sCurrState.joyY;

                         // LATER: May want to modify autoDriveSpeed depending
                         // on the distance from the target or a wall (or
                         // other obstruction) determined by sonar transducers.

                     // See the dDesiredTurn calculations in DriveToDistance()
                     // for an explanation of the following code.
         cargoOnVideo.X = std::max( -160, cargoOnVideo.X );
         cargoOnVideo.X = std::min(  160, cargoOnVideo.X );
         cargoOnVideo.Y = std::max( -120, cargoOnVideo.Y );
         cargoOnVideo.Y = std::min(  120, cargoOnVideo.Y );
         dDesiredYaw = sCurrState.yawPosnEstimate -
                180.0 * 3.14156 * cargoOnVideo.X / ( 300.0 + cargoOnVideo.Y );
         dEventualYawPosition = sCurrState.yawPosnEstimate + ( 0.5 / 600.0 ) *
                                              sCurrState.yawRateEstimate *
                                              abs(sCurrState.yawRateEstimate);
         dDesiredTurn = ( dEventualYawPosition - dDesiredYaw ) * 1.0/50.0;
         dDesiredTurn = std::max( -1.0, dDesiredTurn );
         dDesiredTurn = std::min(  1.0, dDesiredTurn );

#ifdef JAG_OLD_WAY
            // Delete this code once the new way is verified to work correctly.
         if        ( 50 <= cargoOnVideo.X ) {
                             // if target to the right, turn towards the right
            Team4918Drive( autoDriveSpeed/1.5,
                           cargoOnVideo.X/500.0 );
                           // sqrt(cargoOnVideo.X/400.0)/2 );
         } else if ( cargoOnVideo.X < -50 ) {
                               // if target to the left, turn towards the left
            Team4918Drive( autoDriveSpeed/1.5,
                           cargoOnVideo.X/500.0 );
                           // -sqrt(-cargoOnVideo.X/400.0)/2 );
         } else {
            Team4918Drive( autoDriveSpeed/1.5, 0.0 ); // drive straight forward
         }
#else
         Team4918Drive( autoDriveSpeed, dDesiredTurn );
#endif

      } else {               // else USB videocamera data is not valid any more
         if ( !sCurrState.teleop ) {  // If not in teleop mode
            sCurrState.joyY = 0.3;    // then simulate some joystick settings
            sCurrState.joyX = 0.0;
         }
         DriveByJoystick(); // No cargo balls seen; drive according to joystick
         returnVal = false;
      }

      if ( 0 == iCallCount%100 )  {
         cout << "Cargo Seen flag " << cargoOnVideo.SeenByCamera <<
                 ": " << cargoOnVideo.X << "/" << cargoOnVideo.Y;
         cout << ", " << cargoOnVideo.Radius  << "." << endl;
      }

      return returnVal;

   }  /* DriveToCargo() */


      /*---------------------------------------------------------------------*/
      /* RunDriveMotors()                                                    */
      /* RunDriveMotors() drives the robot.  It uses joystick and console    */
      /* inputs to determine what the robot should do, and then runs the     */
      /* m_motorLSMaster and m_motorRSMaster motors to make the robot do it. */
      /*---------------------------------------------------------------------*/
   bool RunDriveMotors( void ) {
      static int iCallCount = 0;
      iCallCount++;

      motorFindMinMaxVelocitySpark( m_LSMasterEncoder, LSMotorState );
      motorFindMinMaxVelocitySpark( m_RSMasterEncoder, RSMotorState );

      if ( 0 == iCallCount%1000 )  {   // every 20 seconds
         // JoystickDisplay();
      } else if ( 1 == iCallCount%1000 )  {   // every 20 seconds
           MotorDisplaySpark( "LS:", m_motorLSMaster, 
                              m_LSMasterEncoder, LSMotorState );
      } else if ( 2 == iCallCount%1000 )  {   // every 20 seconds
           MotorDisplaySpark( "RS:", m_motorRSMaster,
                              m_RSMasterEncoder, RSMotorState );
      } else if ( 3 == iCallCount%1000 )  {   // every 20 seconds
         //IMUOrientationDisplay();

         // max free speed for MiniCims is about 6200
         //cout << "Accel: x/y/z: " << RoborioAccel.GetX() << "/";
         //cout << RoborioAccel.GetY() << "/";
         //cout << RoborioAccel.GetZ() << endl;
      } else {
         // cout << "yawrate: " << sCurrState.rateXYZ[2] << endl;
      }

                                      /* Button 1 is the trigger button on  */
                                      /* the front of the joystick.         */
                                      /* (drive to target; either cargo     */
                                      /* or limelight target)               */
                                      /* Button 2 is the bottom button on   */
                                      /* the rear of the joystick.          */
                                      /* (drive in reverse direction).      */

                                       // If driver is pressing button one
                                       // ("trigger: drive-to-a-target")
                                       // but not button 2
                                       // ("drive in reverse direction),
                                       // and a cargo ball is visible
      if ( ( BUTTON_TARGET ) && ( !BUTTON_REVERSE ) &&
           ( cargoOnVideo.SeenByCamera ) ) { 
         DriveToCargo();     // then autonomously drive towards the cargo ball

                                       // If driver is pressing button one
                                       // ("trigger: drive-to-a-target")
                                       // AND button 2
                                       // ("drive in reverse direction),
                                       // and the limelight sees a target
      } else if ( BUTTON_TARGET && BUTTON_REVERSE &&
                  ( 1  == limev )                ) {
                                // Then autonomously drive towards the target.
         sCurrState.joyZ = 0.0; // Set throttle to 1/2 power; do we want this?
                                // Can't we use normal joyZ paddle setting?
         DriveToLimelightTarget();

               // If the console button 12 (the leftmost missile switch) is on
      } else if ( false ) { //formerly BUTTON_CARTESIANDRIVE
                       // Then drive by cartesian coordinates (field oriented)
         DriveCartesianByJoystick();

      } else {  // else no buttons pressed
                             /* Drive the robot according to the            */
         DriveByJoystick();  /* commanded Y and X joystick position.        */
                             /* This function is only called in             */
                             /* teleop mode, so no need to insert data into */
                             /* the joyY and joyX variables.                */
      }
      return true;
   }      // RunDriveMotors()


      /*---------------------------------------------------------------------*/
      /* TurnToHeading()                                                     */
      /* This function turns the robot to a specified heading.               */
      /* heading is a parameter, in the same degree units as the Pigeon IMU  */
      /* (or ADIS16470) IMU                                                  */
      /* produces; it is positive for left turns (the same as trigonometry,  */
      /* but the opposite of ordinary 0-360 degree compass directions).      */
      /* bInit is a boolean that tells the function to initialize (to record */
      /* the current yaw as the starting yaw).                               */
      /* This function returns false if the yaw value has not been reached   */
      /* yet, and returns true when the yaw has been reached.                */
      /*---------------------------------------------------------------------*/
   bool TurnToHeading ( double desiredYaw, bool bInit ) {
      static bool   bReturnValue = true;
      static double startYaw = 0;
      static double dEventualYawPosition = 0;

      // double      currentYaw = sCurrState.yawPitchRoll[0];
      // double      prevYaw    = sPrevState.yawPitchRoll[0];
      double      currentYaw = sCurrState.yawPosnEstimate;
#ifdef JAG_NOTDEFINED
      double      prevYaw    = sPrevState.yawPosnEstimate;
#endif

      static double dDesiredSpeed = 0.01; // -1.0 to +1.0, positive is forward
      static double dDesiredTurn = 0.0;  // -1.0 to +1.0, positive to the right

                 // If we've been told to initialize, or the last call returned
                 // true (indicating that we finished the previous turn).
      if ( bInit || bReturnValue ) {
         startYaw = currentYaw;
         dDesiredSpeed = 0.01;
         cout << "TurnToHeading(): startYaw = " << startYaw;
         cout << " desiredYaw = " << desiredYaw << endl;
      }
                 // Calculate the Yaw Position we'd arrive at (when we came to
                 // a stop) if we simply decelerated our turn rate at
                 // 600.0 degrees/sec/sec, starting right now.
                 // This is conservative; I have measured EZ-PZ angular
                 // acceleration rate at about 2500 deg/sec/sec
                 // (max yaw rate is about 500 deg/sec).
      dEventualYawPosition = currentYaw + ( 0.5 / 600.0 ) *
                                          sCurrState.yawRateEstimate *
                                          abs(sCurrState.yawRateEstimate);
             // Since we now account for yaw rate in deciding when to end the
             // turn, we can be more aggressive in starting the turn:
      dDesiredTurn = ( dEventualYawPosition - desiredYaw ) * 1.0/25.0;
      dDesiredTurn = std::max( -1.0, dDesiredTurn );
      dDesiredTurn = std::min(  1.0, dDesiredTurn );

#ifdef JAG_NOTDEFINED
                       // If we're turning at less than 50 degrees per second,
      if ( std::abs( currentYaw - prevYaw ) < 1.0 ) {
           // Increase the forward speed, to reduce friction and ease the turn.
         dDesiredSpeed = std::min( 1.0,  dDesiredSpeed * 1.1 );
      } else {                // Else we're turning at a good speed, so
                              // reduce the forward speed, to tighten the turn.
         dDesiredSpeed = std::max( 0.01, dDesiredSpeed * 0.9 );
      }
#else
      dDesiredSpeed = 0.0;
#endif

      if ( startYaw < desiredYaw ) {                // Do we need to turn left?
                                          // dDesiredTurn is left negative, but
                                          // *Yaw variables are left positive.
         if ( currentYaw < desiredYaw-5.0 ) {        // Should we keep turning?
            bReturnValue = false;
         } else {                              // else we're done; stop turning
            dDesiredSpeed = 0.0;
            bReturnValue = true; 
         }
      } else {                                   // Else we need to turn right.
                                                // If we have a long way to go,
         if ( desiredYaw+5.0 < currentYaw ) {        // Should we keep turning?
            bReturnValue = false;
         } else {                              // else we're done; stop turning
            dDesiredSpeed = 0.0;
            bReturnValue = true;
         }
      }

//    cout << "Turn2Hdg(): " << dDesiredSpeed << ", " << dDesiredTurn << endl;
      Team4918Drive( dDesiredSpeed, dDesiredTurn );

      if ( bReturnValue ) {
         cout << "TurnToHeading() returning TRUE!!!!!!!!!" << endl;
         cout << "Final yaw: " <<  currentYaw << endl;
      }

      return bReturnValue;
   }      // TurnToHeading()

      /*---------------------------------------------------------------------*/
      /* DriveToDistance()                                                   */
      /* This function drives the robot on a specified heading,              */
      /* to a specified distance.                                            */
      /* desiredYaw is a parameter, in the same degree units as the Pigeon   */
      /*    (or ADIS16470)                                                   */
      /*    IMU produces; it is positive for left turns (the same as         */
      /*    trigonometry, but the opposite of ordinary 0-360 degree compass  */
      /*    directions).  It is also an absolute angle, relative to the      */
      /*    direction the robot was facing when first powered up; it is not  */
      /*    limited to the normal 0-360 degree range.                        */
      /* desiredDistance is a parameter, in feet; positive is in the         */
      /*    "forward" direction (toward the cargo-sucking side of            */
      /*    the robot).                                                      */
      /* bDivertToCargo is a boolean that tells the function to ignore       */
      /*    the specified heading and drive toward a cargo ball instead,     */
      /*    if one is seen by the vision-processing thread.                  */
      /* bInit is a boolean that tells the function to initialize (to record */
      /*    the current position as the starting position).                  */
      /* This function returns false if the distance has not been reached    */
      /* yet, and returns true when the distance has been reached.           */
      /* It also returns true if it has been told to drive toward a          */
      /* cargo ball (bDivertToCargo is true) and a cargo ball is in the      */
      /* intake ( sCurrState.cargoInIntake is true ).                        */
      /*---------------------------------------------------------------------*/
   bool DriveToDistance( double desiredYaw,
                         double desiredDistance,
                         bool   bDivertToCargo,
                         bool   bInit            ) {
      static bool bReturnValue = true;
      static double dTotalRotations = 0.0;
      static double dLSPrevPosition = 0.0;
      static double dRSPrevPosition = 0.0;
      double      dDistanceDrivenRaw; // distance driven in motor rotations
      double      dDistanceDriven;    // distance driven in feet (floating pt.)
      double      dDesiredSpeed;  // -1.0 to +1.0, positive is forward
      double      dDesiredTurn;   // -1.0 to +1.0, positive is to the right
      static double dEventualYawPosition = 0;
      static int  iCallCount = 0;
      iCallCount++;

          // If we've been told to initialize, or the last call returned
          // true (indicating that we finished the previous drive to distance).
      if ( bInit || bReturnValue ) {
         dLSPrevPosition = sCurrState.dLSMasterPosition;
         dRSPrevPosition = sCurrState.dRSMasterPosition;
         dTotalRotations = 0.0;
      }
      dDistanceDrivenRaw =
                   ( - ( sCurrState.dLSMasterPosition - dLSPrevPosition ) +
                     ( sCurrState.dRSMasterPosition - dRSPrevPosition ) ) / 2;
      dTotalRotations += dDistanceDrivenRaw;

            // Convert motor rotations to feet, using diameter of the wheels
            // and the number of inches/foot.
            // Also have to adjust for gear ratio, since the encoders are on 
            // the motors, before the gearbox, and we need the rotation of the
            // wheels, after the gearbox.

            // The drivetrain in this robot has 2 Rev Robotics Neo motors
            // on each side, each with a 10.71:1 gear ratio.  So estimates
            // of the robot wheel performance at full motor power;
            //            529 RPM output at max motor speed,
            //             41.08 foot-pounds output at motor stall with 2 Neos
            //             13.88 feet/second with 6" wheels
            //            328.   pounds of driving force (all 4 Neos)

      dDistanceDriven = dTotalRotations * ( 3.14159 * 6.25 / 10.71 / 12.0 );

                                         // if we haven't driven far enough yet
      if ( std::abs( dDistanceDriven ) < std::abs( desiredDistance ) ) {
         if ( 0.0 < desiredDistance ) {             // If we're driving forward
                                      // and still have more than 10 feet to go
            if ( dDistanceDriven < desiredDistance -  5.0 ) {
               dDesiredSpeed = 1.0;                            // go full speed
            } else {
                   // Otherwise speed is proportional to distance still needed.
               dDesiredSpeed = 0.2 +
                                  ( desiredDistance - dDistanceDriven ) / 10.0;
            }
         } else {                               // else we're driving backwards
                                      // and still have more than 10 feet to go
            if ( desiredDistance +  5.0 < dDistanceDriven ) {
               dDesiredSpeed = -1.0;               // go full speed (backwards)
            } else {
                   // Otherwise speed is proportional to distance still needed.
               dDesiredSpeed = -0.2 +
                                  ( desiredDistance - dDistanceDriven ) / 10.0;
            }
         }
                             // if we have been told to divert to cargo balls
                             // if we see one, and we do see one,
         if ( bDivertToCargo && cargoOnVideo.SeenByCamera ) {
                 // then drive toward the cargo ball, by changing desiredYaw
                 // to point to the cargo ball, based on the X,Y coordinates
                 // of the cargo ball in the camera view.
                 // cargoOnVideo.Y is the Y-position in the video frame of
                 // the cargo ball; the range is from -120 to +120 (pixels).
                 // cargoOnVideo.X is the X-position in the video frame of
                 // the cargo ball; the range is from -160 to +160 (pixels).
                 // (0,0) is right in the center of the field of view, with
                 // X increasing to the right, and Y increasing up away from
                 // the robot.
                 // For this calculation, we find the angle between the center
                 // of the robot and the cargo ball, which is very
                 // approximately the same as the ratio between the X value
                 // in pixels and Y+300 pixels (in radians, assuming the
                 // center (turn-axis) of the robot is at a y-value of -300),
                 // multiplied by 180.0/pi to convert to degrees.
                 // (We do that rather than call atan2() because it is
                 //  accurate enough, and much faster than atan2() ).
                 // That angle is then added to the current yaw angle to get
                 // the desiredYaw angle.
                 // Since yawPosnEstimate is positive to the left but
                 // X is positive to the right, we have to subtract
                 // the ball offset angle from yawPosnEstimate.
                 // NOTE: May have to add/subtract a constant from x-values
                 // here, to account for the offset of the camera away from
                 // the centerline of the robot.
            cargoOnVideo.X = std::max( -160, cargoOnVideo.X );
            cargoOnVideo.X = std::min(  160, cargoOnVideo.X );
            cargoOnVideo.Y = std::max( -120, cargoOnVideo.Y );
            cargoOnVideo.Y = std::min(  120, cargoOnVideo.Y );
            desiredYaw = sCurrState.yawPosnEstimate -
                180.0 * 3.14156 * cargoOnVideo.X / ( 300.0 + cargoOnVideo.Y );
         }

                 // Calculate the Yaw Position we'd arrive at (when we came to
                 // a stop) if we simply decelerated our turn rate at
                 // 600.0 degrees/sec/sec, starting right now.
                 // This is conservative; I have measured EZ-PZ angular
                 // acceleration rate at about 2500 deg/sec/sec.
                 // (max yaw rate is about 500 deg/sec).
         dEventualYawPosition = sCurrState.yawPosnEstimate + ( 0.5 / 600.0 ) *
                                              sCurrState.yawRateEstimate *
                                              abs(sCurrState.yawRateEstimate);
             // Since we now account for yaw rate in deciding when to end the
             // turn, we can be more aggressive in starting the turn:
         dDesiredTurn = ( dEventualYawPosition - desiredYaw ) * 1.0/50.0;
         dDesiredTurn = std::max( -1.0, dDesiredTurn );
         dDesiredTurn = std::min(  1.0, dDesiredTurn );
         
            // If we've been told to drive to a cargo ball, and we now have one
         if ( bDivertToCargo && sCurrState.cargoInIntake ) {
            dDesiredSpeed = 0.0;                              // stop the robot
            dDesiredTurn  = 0.0;
//            m_motorLSMaster.SetIntegralAccumulator( 0.0 );
//            m_motorRSMaster.SetIntegralAccumulator( 0.0 );
            cout << "D2D() has a cargo ball!" << endl;
            bReturnValue = true;                    // tell caller we are done
         } else{
            bReturnValue = false;          // tell caller we are still driving
         }

         Team4918Drive( dDesiredSpeed, dDesiredTurn );      // then just drive

         if ( 0 == iCallCount%100 ) {                        // Every 2 seconds
            cout << "D2D(): curYaw/desYaw desTurn: " <<
                    sCurrState.yawPitchRoll[0] << 
                    "/" << desiredYaw << " " <<
                    " " << dDesiredTurn << endl;
            cout << "dDistanceDriven: " << dDistanceDriven <<
                    sCurrState.dLSMasterPosition <<
                    sCurrState.dRSMasterPosition << endl;
         }
      } else {   // else we *have* driven far enough now
         Team4918Drive( 0.0, 0.0 );                          // stop the robot
//       m_motorLSMaster.SetIntegralAccumulator( 0.0 );
//       m_motorRSMaster.SetIntegralAccumulator( 0.0 );
         bReturnValue = true;    // tell caller we've reached desired distance
      }

      if ( bReturnValue ) {
         cout << "DriveToDistance() returning TRUE!!!!!!!!" << endl;
         cout << "Final Distance: " <<  dDistanceDriven << endl;
         cout << " Final Yaw: " <<  sCurrState.yawPitchRoll[0] << endl;
      }

      dLSPrevPosition = sCurrState.dLSMasterPosition;
      dRSPrevPosition = sCurrState.dRSMasterPosition;

      return bReturnValue;
   }  // DriveToDistance()


         /*------------------------------------------------------------------*/
         /* RunShooter()                                                     */
         /* RunShooter() drives the 2 shooter motors.                        */
         /*------------------------------------------------------------------*/
   bool RunShooter(void) {
           //02/04/2022 max speed 3600
      if (   ( 0.5 < sCurrState.conY ) &&           // if console "joystick" is
            !( 0.5 < sPrevState.conY ) ) {          // newly-pressed downward
         TSMotorState.targetVelocity_UnitsPer100ms =  800 * 4096 / 600;
         BSMotorState.targetVelocity_UnitsPer100ms =  800 * 4096 / 600;
      } else if ( !( 0.5 < sCurrState.conY ) &&
                   ( 0.5 < sPrevState.conY ) ) {     // newly-released downward
         TSMotorState.targetVelocity_UnitsPer100ms = 0 * 4096 / 600;
         BSMotorState.targetVelocity_UnitsPer100ms = 0 * 4096 / 600;
         
      } else if (( sCurrState.conX < -0.5 ) &&  //newly pressed leftward
               !( sPrevState.conX < -0.5 )) {
         TSMotorState.targetVelocity_UnitsPer100ms =  2700 * 4096 / 600;
         BSMotorState.targetVelocity_UnitsPer100ms =  2600 * 4096 / 600;
      } else if (!( sCurrState.conX < -0.5 ) &&  //newly released leftward
               ( sPrevState.conX < -0.5 ) ) {
         TSMotorState.targetVelocity_UnitsPer100ms = 0 * 4096 / 600;
         BSMotorState.targetVelocity_UnitsPer100ms = 0 * 4096 / 600;
      } else if (  ( sCurrState.conY < -0.5 ) &&  // else if console "joystick"
                  !( sPrevState.conY < -0.5 ) ) { // is newly-pressed upward
         TSMotorState.targetVelocity_UnitsPer100ms =  3000 * 4096 / 600;
         BSMotorState.targetVelocity_UnitsPer100ms =  2950 * 4096 / 600;
      } else if ( !( sCurrState.conY < -0.5 ) &&
                   ( sPrevState.conY < -0.5 ) ) {     // newly-released upward
         TSMotorState.targetVelocity_UnitsPer100ms = 0 * 4096 / 600;
         BSMotorState.targetVelocity_UnitsPer100ms = 0 * 4096 / 600;
      }       /*** Following code for testing only - slowly spit out balls
               when console "joystick" is pushed in the positive direction ***/
      else if ( ( sCurrState.conX > 0.5 ) &&  //newly pressed rightward
               !( sPrevState.conX > 0.5 ) ) {
         TSMotorState.targetVelocity_UnitsPer100ms =  2250 * 4096 / 600;
         BSMotorState.targetVelocity_UnitsPer100ms =  2050 * 4096 / 600;
      } else if ( !( sCurrState.conX > 0.5 ) && //newly released rightward
                   ( sPrevState.conX > 0.5 ) ) {
         TSMotorState.targetVelocity_UnitsPer100ms = 0 * 4096 / 600;
         BSMotorState.targetVelocity_UnitsPer100ms = 0 * 4096 / 600;
      /***End of testing code***/
                     // If commanded to shoot based on the limelight data
                     // (this is the same if statement used elsewhere to
                     //  decide if DriveToLimelightTarget() should be called).
      } else if ( BUTTON_TARGET && BUTTON_REVERSE && ( 1  == limev ) ) {
                   // The goal is 104 inches from the floor,
                   // the limelight is 22 inches from the floor, and
                   // the limelight is angled 27.7 degrees above horizontal;
                   // so this equation gives us the distance, in feet, for any
                   // limelight Y value (Y is degrees above the centerline, or
                   // below the centerline if negative).
         if ( limey < -20.0 ) {
            limey = -20.0;
         } else if ( 50.0 < limey ) {
            limey = 50.0;
         }
         sCurrState.dLimelightDistanceToGoal = (104.0 - 22.0)/12.0 /
                                        tan( (27.7 + limey) * 3.14159 / 180 );
	 sCurrState.dLimelightDistanceToGoal -= 2.0;      // correction amount
         if ( sCurrState.dLimelightDistanceToGoal < 2.0 ) {
            sCurrState.dLimelightDistanceToGoal = 2.0;
         } else if ( 25.0 < sCurrState.dLimelightDistanceToGoal ) {
            sCurrState.dLimelightDistanceToGoal = 25.0;
         }
       if ( 18 == iCallCount%100 ) {                      // Every 2 seconds
            cout << "LimeY/LimeDist2Goal: " << limex << "/" << limey << "/" <<
                    sCurrState.dLimelightDistanceToGoal << endl;
       }
                   // By the time the cargo ball lands, the distance will be
                   // a little different, depending on the current speed of
                   // the robot.  So allow for that here.
                   //
                   // The sCurrState.dxSMasterVelocity variables give the RPM
                   // of each drive motor.  Convert that to feet/second
                   // forward for the robot, by dividing by the gear ratio,
                   // dividing by 60 seconds/minute to get revs/second,
                   // multiplying by pi * Diameter to get inches/second, and
                   // dividing by 12 inches/foot to get feet/second.
         double dLSSpeed = sCurrState.dLSMasterVelocity * ( 3.14159 * 6.25 /
                                                        10.71 / 60.0 / 12.0 );
         double dRSSpeed = sCurrState.dRSMasterVelocity * ( 3.14159 * 6.25 /
                                                        10.71 / 60.0 / 12.0 );
                   // The left side motor positive direction is in the
                   // direction of the limelight target (since we are driving
                   // in reverse), but the right side *negative* direction is
                   // toward the limelight target, so subtract and take
                   // the average:
         double dRobotSpeed = ( dLSSpeed - dRSSpeed ) / 2.0;
                   // The time-of-flight of the cargo ball will be proportional
                   // (approximately) to the square root of the distance:
         double dTimeOfFlight =
                 sqrt( sCurrState.dLimelightDistanceToGoal ) / 4.0;
                   // So the correction amount is the distance the robot would
                   // travel while the cargo ball is in the air:
         sCurrState.dLimelightDistanceToGoal -= dTimeOfFlight * dRobotSpeed;
                                       // Protect against out-of-range values.
         if ( sCurrState.dLimelightDistanceToGoal < 6.0 ) {
            sCurrState.dLimelightDistanceToGoal = 6.0;
         } else if ( 18.0 < sCurrState.dLimelightDistanceToGoal ) {
            sCurrState.dLimelightDistanceToGoal = 18.0;
         }
       if ( 19 == iCallCount%100 ) {                      // Every 2 seconds
            cout << "LimeDist2Goal2: " <<
                                  sCurrState.dLimelightDistanceToGoal << endl;
       } else if ( 20 == iCallCount%100 ) {
            cout << "RobotSpeed/FlightTime: " << dRobotSpeed << "/" <<
                                                 dTimeOfFlight << endl;
       }

                   // The shooter needs to be at about 2100 RPM (top shooter)
                   // just to get the cargo ball up to the height of the goal,
                   // and a goal can be scored at that RPM at about 6' from
                   // the goal.  After that initial speed requirement, the
                   // distance covered by the shooter in a vacuum would be
                   // proportional to the square of the exit speed of the
                   // cargo ball:
                   //     distance beyond 6' = (some constant)*rpm*rpm
                   // where "rpm" is the RPM above 2000.  But the effects of
                   // drag make higher speeds less effective than RPM squared.
                   // By measuring several different speeds and distances on
                   // the actual robot, we found these values for the shooter
                   // motors' RPMs would shoot well into the upper goal
                   //     6' --> 2100 RPM top, 2050 RPM bottom
                   //     9' --> 2300(?) RPM top, 2250(?) RPM bottom
                   //    12' --> 2600 RPM top, 2400 RPM bottom
                   //    15' --> 3000 RPM top, 2950 RPM bottom
                   // To simplify a little bit, we will use an equation
                   // to find the RPM of the top shooter, and set the
                   // bottom shooter to 50 RPM below that.
                   // Our limited measurements show that a simple linear
                   // equation does a good a job of fitting the data.
                   // The equation below works better (it produces results
                   // which fit all the points above very well):
         if ( sCurrState.dLimelightDistanceToGoal < 6.0 ) {
            sCurrState.dLimelightDesiredShooterSpeed = 2100.0;
         } else {
                // This equation gives 2100 RPM at  6', 2400 RPM at 9',
                //                     2700 RPM at 12', 3000 RPM at 15'.
                //   sCurrState.dLimelightDesiredShooterSpeed = 1500.0 +
                //                100.0 * sCurrState.dLimelightDistanceToGoal;
                // But this equation does better; it gives
                //                     2111 RPM at  6', 2255 RPM at 9',
                //                     2536 RPM at 12', 3000 RPM at 15'.
           sCurrState.dLimelightDesiredShooterSpeed = 2050.0 +
                     0.2815 * sCurrState.dLimelightDistanceToGoal *
                              sCurrState.dLimelightDistanceToGoal *
                              sCurrState.dLimelightDistanceToGoal;
         }
         if ( 3100.0 < sCurrState.dLimelightDesiredShooterSpeed ) {
            sCurrState.dLimelightDesiredShooterSpeed = 3100.0;
         }
                 // Command the motors to turn 100 RPM faster than we need,
                 // to ensure they can actually reach the speed we need.
                 // The Shoot() function will run the conveyor to transport
                 // the cargo balls into the shooter when
                 // dLimelightDesiredShooterSpeed is reached
                 // (dLimelightDesiredShooterSpeed-50 for the bottom motor).
         TSMotorState.targetVelocity_UnitsPer100ms =
              (sCurrState.dLimelightDesiredShooterSpeed        ) * 4096 / 600;
         BSMotorState.targetVelocity_UnitsPer100ms =
              (sCurrState.dLimelightDesiredShooterSpeed -  50.0) * 4096 / 600;
       if ( 21 == iCallCount%100 ) {                      // Every 2 seconds
            cout << "Shooters: " <<
                    sCurrState.dLimelightDesiredShooterSpeed << endl;
       }
      } else if ( ( -0.5 < sCurrState.conX       ) && 
                  (        sCurrState.conX < 0.5 ) &&
                  ( -0.5 < sCurrState.conY       ) && 
                  (        sCurrState.conY < 0.5 ) ) {
                                   // else spin the shooter motors down slowly
         sCurrState.dLimelightDesiredShooterSpeed = 0.0;
         TSMotorState.targetVelocity_UnitsPer100ms = 0.0;
//                0.95 * (double)m_motorTopShooter.GetSelectedSensorVelocity();
         BSMotorState.targetVelocity_UnitsPer100ms = 0.0;
//                0.95 * (double)m_motorBotShooter.GetSelectedSensorVelocity();
//      } else {
//         sCurrState.dLimelightDesiredShooterSpeed = 0.0;
//         TSMotorState.targetVelocity_UnitsPer100ms = 0.0;
//         BSMotorState.targetVelocity_UnitsPer100ms = 0.0;
      }
                   // Top shooter:
                   // if current speed is more than 100 RPM lower than desired
      if ( sCurrState.iTSMasterVelocity <
                  TSMotorState.targetVelocity_UnitsPer100ms - 100*4096/600 ) {
                 // set the motor velocity 100 RPM above what we want it to be
         m_motorTopShooter.Set( ControlMode::Velocity,
                 TSMotorState.targetVelocity_UnitsPer100ms + 50 * 4096 / 600 );
             // else if current speed is more than 500 RPM higher than desired
      } else if ( TSMotorState.targetVelocity_UnitsPer100ms + 500*4096/600 <
                                 sCurrState.iTSMasterVelocity ) {
              // Set motor speed to 0.  This is OK even if the desired speed
              // is 0, because we have configured the shooter motors to never
              // be driven in reverse, so they will slow down smoothly even
              // when commanded to stop immediately.
         m_motorTopShooter.Set( ControlMode::Velocity, 0 );
      }

                   // Bottom shooter:
                   // if current speed is more than 100 RPM lower than desired
      if ( sCurrState.iBSMasterVelocity <
                  BSMotorState.targetVelocity_UnitsPer100ms - 100*4096/600 ) {
                 // set the motor velocity 100 RPM above what we want it to be
         m_motorBotShooter.Set( ControlMode::Velocity,
               BSMotorState.targetVelocity_UnitsPer100ms + 50 * 4096 / 600 );
             // else if current speed is more than 500 RPM higher than desired
      } else if ( BSMotorState.targetVelocity_UnitsPer100ms + 500*4096/600 <
                                 sCurrState.iBSMasterVelocity ) {
              // Set motor speed to 0.  This is OK even if the desired speed
              // is 0, because we have configured the shooter motors to never
              // be driven in reverse, so they will slow down smoothly even
              // when commanded to stop immediately.
         m_motorBotShooter.Set( ControlMode::Velocity, 0 );
      }

      if ( 0 == iCallCount%100 )  {   // every 2 seconds (at 2.00)
         if ( 100.0 < abs(m_motorTopShooter.GetSelectedSensorVelocity()) ) {
            MotorDisplay( "TS:", m_motorTopShooter, TSMotorState );
         }
      } else if ( 1 == iCallCount%100 )  {   // every 2 seconds (at 2.02)
         if ( 100.0 < abs(m_motorBotShooter.GetSelectedSensorVelocity()) ) {
            MotorDisplay( "BS:", m_motorBotShooter, BSMotorState );
         }
      }
 
      return true;
   }     // RunShooter()


         /*------------------------------------------------------------------*/
         /* Shoot()                                                          */
         /* Shoot() waits until the shooter rollers are moving at a desired  */
         /* speed, then moves the conveyor to shoot cargo balls.             */
         /*------------------------------------------------------------------*/
   void Shoot( void ) {
           // if the shooter motors have been commanded to spin
           // at more than 500 RPM
      if ( ( 500*4096/600 < TSMotorState.targetVelocity_UnitsPer100ms ) &&
           ( 500*4096/600 < BSMotorState.targetVelocity_UnitsPer100ms ) ) {
                                                   // then we must be shooting
                  // if both motors are within 100 RPM of their desired speeds
         if ( ( abs( sCurrState.iTSMasterVelocity -
              TSMotorState.targetVelocity_UnitsPer100ms ) < 100*4096/600 ) &&
              ( abs( sCurrState.iBSMasterVelocity -
              BSMotorState.targetVelocity_UnitsPer100ms ) < 100*4096/600 ) ) {

                                         // run the conveyor to shoot the balls
            sCurrState.iConveyPercent = 100;
            m_motorConveyMaster.Set( ControlMode::PercentOutput, 1.0 );
            m_motorIntake.Set( ControlMode::PercentOutput, 0.5 ); // be strong
         }
      } else if ( ( 0.5 < sPrevState.conY  ) ||    // else if X- or Y-stick WAS
                  ( sPrevState.conY < -0.5 ) ||    // pushed in any "shoot"
                  ( 0.5 < sPrevState.conX  ) ||
                  ( sPrevState.conX < -0.5 )  ) { // direction, and now is not
                                           // Then turn off conveyor and intake
         sPrevState.iConveyPercent = 0;
         sCurrState.iConveyPercent = 0;
         m_motorConveyMaster.Set( ControlMode::PercentOutput, 0.0 );
         m_motorIntake.Set( ControlMode::PercentOutput, 0.0 );

                     // If commanded to shoot based on the limelight data
                     // (this is the same if statement used elsewhere to
                     //  decide if DriveToLimelightTarget() should be called,
                     //  and also makes sure the Limelight is locked onto
                     //  the target).
      } else if ( BUTTON_TARGET && BUTTON_REVERSE && ( 1 == limev ) 
                  && ( -2.0 <= limex) && (limex <= 2.0) ) {
                 // If speed of both motors is close to what is requested
                 // (top motor from DesiredSpeed to DesiredSpeed+100 RPM;
                 //  bottom motor from DesiredSpeed-50 to DesiredSpeed+50 RPM)
         if ( ( sCurrState.dLimelightDesiredShooterSpeed * 4096 / 600 <
                   abs( m_motorTopShooter.GetSelectedSensorVelocity() ) ) &&
              ( abs( m_motorTopShooter.GetSelectedSensorVelocity() ) <
                (sCurrState.dLimelightDesiredShooterSpeed+100.0) *
                                                              4096 / 600 ) &&
              ( (sCurrState.dLimelightDesiredShooterSpeed-50.0) * 4096 / 600 <
                   abs( m_motorBotShooter.GetSelectedSensorVelocity() ) ) &&
              ( abs( m_motorBotShooter.GetSelectedSensorVelocity() ) <
                (sCurrState.dLimelightDesiredShooterSpeed+50.0) *
                                                              4096 / 600 ) ) {
                                         // run the conveyor to shoot the balls
            sCurrState.iConveyPercent = 100;
            m_motorConveyMaster.Set( ControlMode::PercentOutput, 1.0 );
            m_motorIntake.Set( ControlMode::PercentOutput, 1.0 ); // be strong
         }
                     // Else if we were previously shooting under the control
                     // of the limelight, and now are not
      } else if ( BUTTON_TARGET_PREV && BUTTON_REVERSE_PREV ) { 
                                           // Then turn off conveyor and intake
         sPrevState.iConveyPercent = 0;
         sCurrState.iConveyPercent = 0;
         m_motorConveyMaster.Set( ControlMode::PercentOutput, 0.0 );
         m_motorIntake.Set( ControlMode::PercentOutput, 0.0 );
      }
   }    // Shoot()


      /*---------------------------------------------------------------------*/
      /* RunIntake()                                                         */
      /* Run the intake motor, to move balls into the robot.                 */
      /*---------------------------------------------------------------------*/
   void RunIntake( void ) {

      if ( autoConveyor ) {  // If fully-automatic conveyor is selected
                // If there is no cargo in intake, then we can run the intake;
                // or if no cargo at position 5, then we can run the intake.
                // But if cargo at both places, then we have 2 cargo already
                // and cannot accept any more, so don't want to run the intake.
                // So we only run the intake if the driver is driving forward,
                // and either of those locations has no cargo.
         if ( !BUTTON_REVERSE &&
              ( !sCurrState.cargoInIntake ||
                !sCurrState.cargoInPosition5 ) ) {
            sCurrState.iIntakePercent = 0.6;
            m_motorIntake.Set( ControlMode::PercentOutput, 0.6 );
         } else {
            sCurrState.iIntakePercent = 0.0;
            m_motorIntake.Set( ControlMode::PercentOutput, 0.0 );
         }

      } else if ( BUTTON_RUNINTAKE )   {                 // Run intake forward.
         if (  sCurrState.cargoInIntake ) {       // if a cargo ball in intake
            sCurrState.iIntakePercent = 0.6;
            m_motorIntake.Set( ControlMode::PercentOutput, 0.6 ); // be gentle
         } else {
            sCurrState.iIntakePercent = 0.6;
            m_motorIntake.Set( ControlMode::PercentOutput, 0.6 ); // be strong
         }
                            // If necessary (for any reason), stop the intake.
                                 // if the "RUN INTAKE" button was previously
                                 // pressed (we were in manual intake mode)
      } else if ( BUTTON_RUNINTAKE_PREV ) {
         sCurrState.iIntakePercent = 0;                // Then stop the intake.
         m_motorIntake.Set( ControlMode::PercentOutput, 0.0 );
                                 // or if button 1 was previously pressed
                                 // and button 2 was not previously pressed
                                 // (which would have caused us to call
                                 // DriveToCargo, which runs the intake).
      } else if ( BUTTON_TARGET_PREV && !BUTTON_REVERSE_PREV ) {
         sCurrState.iIntakePercent = 0;                // Then stop the intake.
         m_motorIntake.Set( ControlMode::PercentOutput, 0.0 );
                    // Or if we were previously driving the conveyor backwards
                    // manually, but are not doing that now (we just stopped)
      } else if ( BUTTON_CONVEYORBACKWARD_PREV && !BUTTON_CONVEYORBACKWARD ) {
            sCurrState.iIntakePercent = 0;            // Then stop the intake.
            m_motorIntake.Set( ControlMode::PercentOutput, 0.0 );
      } else {
              // No reason to run the intake, so every once in a while stop it.
         if ( 17 == iCallCount%100 ) {                // Every 2 seconds
            sCurrState.iIntakePercent = 0;            // stop the intake.
            m_motorIntake.Set( ControlMode::PercentOutput, 0.0 );
         }
      }

   }   // RunIntake()


      /*---------------------------------------------------------------------*/
      /* RunConveyor()                                                       */
      /* Run the conveyor belt motors, to move balls into and through the    */
      /* conveyor system.                                                    */
      /*---------------------------------------------------------------------*/
   void RunConveyor( void ) {
      //print out if there is a cargo ball in the intake or not. 
      if ( sPrevState.cargoInIntake != sCurrState.cargoInIntake ) {
         if ( sCurrState.cargoInIntake ) {
            cout << "cargo ball in the intake." << endl;
         } else {
            cout << "cargo ball NOT in the intake." << endl; 
         } 
      }
      // print out if there is a ball in position 5. 
      if ( sPrevState.cargoInPosition5 !=
                                          sCurrState.cargoInPosition5 ) {
         if ( sCurrState.cargoInPosition5 ) {// if this sensor is blocked
            cout << "cargo ball in position 5" << endl;
         } else {
            cout << "cargo ball NOT in position 5" << endl; 
         } 
      }
                                 // If driver is pushing BOTH conveyor buttons
      if ( BUTTON_CONVEYORFORWARD && BUTTON_CONVEYORBACKWARD ) {
                   // Then run the conveyor according to the Joystick throttle
         sCurrState.iConveyPercent = -0.5*sCurrState.joyZ;
         m_motorIntake.Set( ControlMode::PercentOutput,
                            -1.0 * sCurrState.joyZ );
      } else if ( BUTTON_CONVEYORFORWARD )   {        // Run conveyor forward.
         sCurrState.iConveyPercent = 80;
      } else if ( BUTTON_CONVEYORBACKWARD ) {     // Run conveyor backward.
         sCurrState.iConveyPercent =  -80;
         sCurrState.iIntakePercent = -60;     // Run intake backwards, too.
         m_motorIntake.Set( ControlMode::PercentOutput, -0.6 );
                                          // else if the shooter isn't running
      } else if ( !(sCurrState.conX >  0.5) && !(sCurrState.conY >  0.5) && 
                  !(sCurrState.conX < -0.5) && !(sCurrState.conY < -0.5) &&
                  !( BUTTON_TARGET && BUTTON_REVERSE && ( 1 == limev ) ) ) {
                        // run the conveyor as needed (when cargo is in intake)
                        // unless in manual conveyor mode
         static int ConveyorCounter = 0;
         if ( sCurrState.cargoInIntake && !sCurrState.cargoInPosition5 ) {
           sCurrState.iConveyPercent = 60;
           ConveyorCounter = 6;
         } else if (( 0 < ConveyorCounter ) && !sCurrState.cargoInPosition5 ) {
           sCurrState.iConveyPercent = 60;
           ConveyorCounter--;
         } else {
            //if (  sPrevState.cargoInIntake && 
            //     !sPrevState.cargoInPosition5 ) {
             sCurrState.iConveyPercent = 0;
            // }
         } 
      }
      if ( sPrevState.iConveyPercent != sCurrState.iConveyPercent ) {
         m_motorConveyMaster.Set( ControlMode::PercentOutput,
                                  (double)sCurrState.iConveyPercent / 100.0 );
      } else if ( 19 == iCallCount%100 ) {
         sCurrState.iConveyPercent = 0;
         m_motorConveyMaster.Set( ControlMode::PercentOutput,
                                  (double)sCurrState.iConveyPercent / 0.0 );
      }
   }   // RunConveyor()


      /*---------------------------------------------------------------------*/
      /* RunBlueClimberPole()                                                    */
      /* Extend or retract the telescoping climber pole.                     */
      /*---------------------------------------------------------------------*/
   void RunBlueClimberPole( rev::CANSparkMax & m_motorClimberPole,
                       rev::SparkMaxLimitSwitch m_ClimberForwardLimitSwitch,
                       rev::SparkMaxLimitSwitch m_ClimberReverseLimitSwitch ) {
      static int iCallCount = 0;
      static bool limitSwitchHasBeenHit = false;
      iCallCount++;

                     // if BOTH climber buttons pressed, run the climber motor
                     // according to the throttle (Z-Axis) on the joystick
      if ( BUTTON_BLUECLIMBERUP && BUTTON_BLUECLIMBERDOWN ) {
         m_motorClimberPole.Set( 0.5*sCurrState.joyZ );
      
      } else if ( BUTTON_BLUECLIMBERUP ){
         //if ( !BUTTON_CLIMBERUP_PREV ) {       // if button 1 has just been
           // limitSwitchHasBeenHit = false;       // pressed, reset to start
           // m_compressor.Stop();
         //}
         if ( limitSwitchHasBeenHit ) {
                 // we are at the top; just supply a little power to stay there
            m_motorClimberPole.Set( 0.10 );
         } else {
                                                   // apply full climbing power
            m_motorClimberPole.Set( 0.7 );
            // if ( m_ClimberForwardLimitSwitch.Get() ) {
            //   limitSwitchHasBeenHit = true;
            // }
         }
      } else if (BUTTON_BLUECLIMBERDOWN ) {
         m_motorClimberPole.Set( -0.7 );
           
      } else { 
         // Else neither button is currently being pressed.  If either was
         // previously pressed, stop sending power to climber pole motor
         if ( BUTTON_BLUECLIMBERUP_PREV || BUTTON_BLUECLIMBERDOWN_PREV ) {
            m_motorClimberPole.Set( 0.0 );
         }
      }
      

      if ( 0 == iCallCount%103 ) { // every 2 seconds
         if ( BUTTON_BLUECLIMBERUP || BUTTON_BLUECLIMBERDOWN ) {
            if ( BUTTON_BLUECLIMBERUP ) {
               cout << "ClimberUp: ";
            } else {
               cout << "ClimberDown: ";
            }
            cout << setw(5) <<
               m_motorClimberPole.GetOutputCurrent() << "A" << endl;
            if ( m_ClimberForwardLimitSwitch.Get() ) {
               cout << "Climber pole at top." << endl;
            } else if ( m_ClimberReverseLimitSwitch.Get() ) {
               cout << "Climber pole at bottom." << endl;
            }  
         }
      }
   }      // RunBlueClimberPole() 

      /*---------------------------------------------------------------------*/
      /* RunRedClimberPole()                                                    */
      /* Extend or retract the telescoping climber pole.                     */
      /*---------------------------------------------------------------------*/
   void RunRedClimberPole( rev::CANSparkMax & m_motorClimberPole,
                       rev::SparkMaxLimitSwitch m_ClimberForwardLimitSwitch,
                       rev::SparkMaxLimitSwitch m_ClimberReverseLimitSwitch ) {
      static int iCallCount = 0;
      static bool limitSwitchHasBeenHit = false;
      iCallCount++;

                     // if BOTH climber buttons pressed, run the climber motor
                     // according to the throttle (Z-Axis) on the joystick
      if ( BUTTON_REDCLIMBERUP && BUTTON_REDCLIMBERDOWN ) {
         m_motorClimberPole.Set( 0.5*sCurrState.joyZ );
      
      } else if ( BUTTON_REDCLIMBERUP ){
         //if ( !BUTTON_CLIMBERUP_PREV ) {       // if button 1 has just been
           // limitSwitchHasBeenHit = false;       // pressed, reset to start
           // m_compressor.Stop();
         //}
         if ( limitSwitchHasBeenHit ) {
                 // we are at the top; just supply a little power to stay there
            m_motorClimberPole.Set( 0.10 );
         } else {
                                                   // apply full climbing power
            m_motorClimberPole.Set( 0.7 );
            // if ( m_ClimberForwardLimitSwitch.Get() ) {
            //   limitSwitchHasBeenHit = true;
            // }
         }
      } else if (BUTTON_REDCLIMBERDOWN ) {
         m_motorClimberPole.Set( -0.7 );
           
      } else { 
         // Else neither button is currently being pressed.  If either was
         // previously pressed, stop sending power to climber pole motor
         if ( BUTTON_REDCLIMBERUP_PREV || BUTTON_REDCLIMBERDOWN_PREV ) {
            m_motorClimberPole.Set( 0.0 );
         }
      }
      

      if ( 0 == iCallCount%103 ) { // every 2 seconds
         if ( BUTTON_REDCLIMBERUP || BUTTON_REDCLIMBERDOWN ) {
            if ( BUTTON_REDCLIMBERUP ) {
               cout << "ClimberUp: ";
            } else {
               cout << "ClimberDown: ";
            }
            cout << setw(5) <<
               m_motorClimberPole.GetOutputCurrent() << "A" << endl;
            if ( m_ClimberForwardLimitSwitch.Get() ) {
               cout << "Climber pole at top." << endl;
            } else if ( m_ClimberReverseLimitSwitch.Get() ) {
               cout << "Climber pole at bottom." << endl;
            }  
         }
      }
   }      // RunRedClimberPole() 


      /*---------------------------------------------------------------------*/
      /* MotorInitSparkBrushed()                                                    */
      /* Setup the initial configuration of a brushed motor, driven by a     */
      /* Spark Max controller.  These settings can be superseded after this  */
      /* function is called, for the needs of each specific SparkMax-driven  */
      /* brushed motor.                                                      */
      /*---------------------------------------------------------------------*/
   void MotorInitSparkBrushed( rev::CANSparkMax & m_motor ) {

      m_motor.RestoreFactoryDefaults( false );

      m_motor.EnableSoftLimit( rev::CANSparkMax::SoftLimitDirection::kForward,
                               false );
      m_motor.EnableSoftLimit( rev::CANSparkMax::SoftLimitDirection::kReverse,
                               false );
         /*
          * Configure Spark Max Output direction.
          * Sensor (encoder) direction would be set by
          * m_motorEncoder.SetInverted( true), but that doesn't work with
          * hall-effect encoders like we have on our Neo drive motors.
          */
      m_motor.SetInverted( true );  // invert direction of motor itself.

            /* Set limits to how much current will be sent through the motor */
#ifdef SAFETY_LIMITS
                 // 10 Amps below 5000 RPM, above 5000 RPM it ramps from
                 // 10 Amps down to  5 Amps at 5700 RPM
                 // At this low current limit, the robot will be unable to
                 // rotate-in-place on carpet.
      m_motor.SetSmartCurrentLimit( 10,  5, 5000 );
#else
                 // 30 Amps below 5000 RPM, above 5000 RPM it ramps from
                 // 30 Amps down to 10 Amps at 5700 RPM
      m_motor.SetSmartCurrentLimit( 30, 10, 5000 );
#endif

                                          // Config 100% motor output to 12.0V
      m_motor.EnableVoltageCompensation( 12.0 );

               /* Set ramp rate (how fast motor accelerates or decelerates) */
      m_motor.SetClosedLoopRampRate(0.1);
      m_motor.SetOpenLoopRampRate(  0.1);

      m_motor.SetIdleMode( rev::CANSparkMax::IdleMode::kCoast );

   }      // MotorInitSparkBrushed()


      /*---------------------------------------------------------------------*/
      /* MotorInitSpark()                                                    */
      /* Setup the initial configuration of a Neo motor, driven by a         */
      /* Spark Max controller.  These settings can be superseded after this  */
      /* function is called, for the needs of each specific SparkMax-driven  */
      /* motor.                                                              */
      /*---------------------------------------------------------------------*/
   void MotorInitSpark( rev::CANSparkMax             & m_motor,
                        rev::SparkMaxPIDController   & m_motorPID,
                        rev::SparkMaxRelativeEncoder & m_motorEncoder ) {

      m_motor.RestoreFactoryDefaults( false );

      m_motor.EnableSoftLimit( rev::CANSparkMax::SoftLimitDirection::kForward,
                               false );
      m_motor.EnableSoftLimit( rev::CANSparkMax::SoftLimitDirection::kReverse,
                               false );
         /*
          * Configure Spark Max Output direction.
          * Sensor (encoder) direction would be set by
          * m_motorEncoder.SetInverted( true), but that doesn't work with
          * hall-effect encoders like we have on our Neo drive motors.
          */
      m_motor.SetInverted( true );  // invert direction of motor itself.

                        /* Set relevant frame periods to be at least as fast */
                        /* as the periodic rate.                             */
//      m_motor.SetStatusFramePeriod(
//                         StatusFrameEnhanced::Status_13_Base_PIDF0,  10, 10 );
//      m_motor.SetStatusFramePeriod(
//                         StatusFrameEnhanced::Status_10_MotionMagic, 10, 10 );

                                         /* Set the peak and nominal outputs */
//      m_motor.ConfigNominalOutputForward( 0, 10 );
//      m_motor.ConfigNominalOutputReverse( 0, 10 );
//      m_motor.ConfigPeakOutputForward(    1, 10 );
//      m_motor.ConfigPeakOutputReverse(   -1, 10 );

            /* Set limits to how much current will be sent through the motor */
#ifdef SAFETY_LIMITS
                 // 10 Amps below 5000 RPM, above 5000 RPM it ramps from
                 // 10 Amps down to  5 Amps at 5700 RPM
                 // At this low current limit, the robot will be unable to
                 // rotate-in-place on carpet.
      m_motor.SetSmartCurrentLimit( 10,  5, 5000 );
#else
                 // 30 Amps below 5000 RPM, above 5000 RPM it ramps from
                 // 30 Amps down to 10 Amps at 5700 RPM
                 // We may have to try different CurrentLimits here to
                 // eliminate drivetrain chattering.
      m_motor.SetSmartCurrentLimit( 80, 10, 5000 );
#endif

                                          // Config 100% motor output to 12.0V
      m_motor.EnableVoltageCompensation( 12.0 );

                 /* Set Closed Loop PIDF gains in slot0 - see documentation */
                                                    // if encoder is connected
      m_motorPID.SetP(     DrivePIDkP );
      m_motorPID.SetI(     DrivePIDkI );
      m_motorPID.SetD(     DrivePIDkD );
      m_motorPID.SetIZone( DrivePIDkIz );
      m_motorPID.SetFF(    DrivePIDkFF );
      m_motorPID.SetOutputRange( DrivePIDkMinOutput, DrivePIDkMaxOutput );
#ifdef DISP_SMARTDASHBOARD
           // display PID coefficients on the SmartDashboard
      frc::SmartDashboard::PutNumber( "P Gain", DrivePIDkP );
      frc::SmartDashboard::PutNumber( "I Gain", DrivePIDkI );
      frc::SmartDashboard::PutNumber( "D Gain", DrivePIDkD );
      frc::SmartDashboard::PutNumber( "I Zone", DrivePIDkIz );
      frc::SmartDashboard::PutNumber( "Feed Forward", DrivePIDkFF );
      frc::SmartDashboard::PutNumber( "Min Output", DrivePIDkMinOutput );
      frc::SmartDashboard::PutNumber( "Max Output", DrivePIDkMaxOutput );
#endif

                /* Set acceleration and cruise velocity - see documentation */
      m_motorPID.SetSmartMotionMaxVelocity( 5000, 0 );         // RPM
      m_motorPID.SetSmartMotionMaxAccel(   5700, 0 );          // RPM/second

               /* Set ramp rate (how fast motor accelerates or decelerates) */
                 // We may have to try different RampRates here to
                 // eliminate drivetrain chattering.
      m_motor.SetClosedLoopRampRate(0.1);
      m_motor.SetOpenLoopRampRate(  0.1);

      m_motor.SetIdleMode( rev::CANSparkMax::IdleMode::kCoast );

   }      // MotorInitSpark()


      /*---------------------------------------------------------------------*/
      /* MotorInit()                                                         */
      /* Setup the initial configuration of a motor driven by a Talon or     */
      /* Victor motor controller.  These settings can be superseded after    */
      /* this function is called, for the needs of each specific motor.      */
      /*---------------------------------------------------------------------*/
   void MotorInit( WPI_TalonSRX & m_motor ) {

      m_motor.ConfigFactoryDefault( 10 );

                /* Configure Sensor Source for Primary PID */
          /* Config to stop motor immediately when limit switch is closed. */
                                                   // if encoder is connected
      if ( OK == m_motor.ConfigSelectedFeedbackSensor(
                     FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10 ) ) {
//       m_motor.ConfigForwardLimitSwitchSource(
//                   LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
//                   LimitSwitchNormal::LimitSwitchNormal_NormallyOpen );
//       m_motor.ConfigReverseLimitSwitchSource(
//                   LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
//                   LimitSwitchNormal::LimitSwitchNormal_NormallyOpen );
         m_motor.OverrideLimitSwitchesEnable(true);
      }

         /*
          * Configure Talon SRX Output and Sensor direction.
          * Invert Motor to have green LEDs when driving Talon Forward
          * ( Requesting Positive Output ),
          * Phase sensor to have positive increment when driving Talon Forward
          * (Green LED)
          */
      m_motor.SetSensorPhase(true);   // invert encoder value positive/negative
      m_motor.SetInverted(false);     // invert direction of motor itself.

                        /* Set relevant frame periods to be at least as fast */
                        /* as the periodic rate.                             */
      m_motor.SetStatusFramePeriod(
                         StatusFrameEnhanced::Status_13_Base_PIDF0,  10, 10 );
      m_motor.SetStatusFramePeriod(
                         StatusFrameEnhanced::Status_10_MotionMagic, 10, 10 );

                                         /* Set the peak and nominal outputs */
      m_motor.ConfigNominalOutputForward( 0, 10 );
      m_motor.ConfigNominalOutputReverse( 0, 10 );
      m_motor.ConfigPeakOutputForward(    1, 10 );
      m_motor.ConfigPeakOutputReverse(   -1, 10 );

            /* Set limits to how much current will be sent through the motor */
      m_motor.ConfigPeakCurrentDuration(1);  // 1000 milliseconds (for 60 Amps)
#ifdef SAFETY_LIMITS
      m_motor.ConfigPeakCurrentLimit(10);       // limit motor power severely
      m_motor.ConfigContinuousCurrentLimit(10); // to 10 Amps
#else
      m_motor.ConfigPeakCurrentLimit(60);        // 60 works here for miniCIMs,
                                                 // or maybe 40 Amps is enough,
                                                 // but we reduce to 10, 1, 10
      m_motor.ConfigContinuousCurrentLimit(60);  // for safety while debugging
#endif
      m_motor.EnableCurrentLimit(true);

                                          // Config 100% motor output to 12.0V
      m_motor.ConfigVoltageCompSaturation( 12.0 );
      m_motor.EnableVoltageCompensation( true );

                 /* Set Closed Loop PIDF gains in slot0 - see documentation */
                                                    // if encoder is connected
      if ( OK == m_motor.ConfigSelectedFeedbackSensor(
                          FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10 ) ) {
         m_motor.SelectProfileSlot( 0, 0 );
         m_motor.Config_kF( 0, 0.15,   10 );
         m_motor.Config_kP( 0, 0.2,    10 );
         m_motor.Config_kI( 0, 0.0002, 10 );
         m_motor.Config_kD( 0, 10.0,   10 );
      } else {
         m_motor.SelectProfileSlot( 0, 0 );
         m_motor.Config_kF( 0, 0.15, 10 );
         m_motor.Config_kP( 0, 0.0, 10 );
         m_motor.Config_kI( 0, 0.0, 10 );
         m_motor.Config_kD( 0, 0.0, 10 );
      }

                /* Set acceleration and cruise velocity - see documentation */
      m_motor.ConfigMotionCruiseVelocity( 1500, 10 );
      m_motor.ConfigMotionAcceleration(   1500, 10 );

               /* Set ramp rate (how fast motor accelerates or decelerates) */
      m_motor.ConfigClosedloopRamp(0.1);
      m_motor.ConfigOpenloopRamp(  0.1);

      m_motor.SetNeutralMode( NeutralMode::Coast );

   }      // MotorInit()


      /*---------------------------------------------------------------------*/
      /* executeManeuver()                                                   */
      /* This function is called with a maneuver struct, to perform a        */
      /* specified maneuver and return the completion status.                */
      /* It returns false if the maneuver is not yet completed, or           */
      /* true if the maneuver has been completed.                            */
      /*---------------------------------------------------------------------*/
   bool executeManeuver( struct maneuver mSeq ) {
      bool       bRetVal = false;
      static struct maneuver mSeqPrev = { 0, M_STOP, 0.0, false, 0.0 };
      // static int icmdSeqManeuverCallCount = 0;
      static int iShootCount = 200;
      static int iLimeLockCount = 100;
      static double dWaitCount =  50.0; // ticks to wait (50 ticks = 1 second)

      //              // print debugging info (this code should be removed later)
      // if ( ( mSeqPrev.index != mSeq.index ) ||
      //      ( mSeqPrev.type  != mSeq.type  )    ) {
      //       cout << "executeManeuver: Changed Maneuver, index: ";
      //       cout << mSeqPrev.index << " > " << mSeq.index << " ." << endl;
      //       cout << "                                    type: ";
      //       cout << mSeqPrev.type  << " > " << mSeq.type  << " ." << endl;
      //       cout << "                                distance: ";
      //       cout << mSeqPrev.distance << " > " << mSeq.distance << " ." << endl;
      //       cout << "                                     yaw: ";
      //       cout << mSeqPrev.yaw   << " > " << mSeq.yaw << " ." << endl;
      // }

      switch ( mSeq.type )
      {
      case M_STOP:
         Team4918Drive( 0.0, 0.0 );       // make sure drive motors are stopped
         bRetVal = true;                  // and exit this maneuver immediately
         break;

      case M_DRIVE_STRAIGHT:
                   // Drive straight at the specified yaw angle for a specified
                   // distance, (but if mSeq.bDivertToCargo is TRUE, and
                   // a cargo ball is seen by the camera, then drive toward
                   // that cargo ball instead).
                   // The DriveToDistance() function returns true
                   // when it has driven far enough.
         bRetVal = DriveToDistance( sCurrState.initialYaw + mSeq.yaw,
                                    mSeq.distance,
                                    mSeq.bDivertToCargo,
                                    false );
                                  // If we have driven far enough...
         if ( bRetVal ) {
            cout << "EM: DriveToDistance completed, heading: ";
            cout << sCurrState.yawPitchRoll[0]  << endl;
            cout << "            distance rotations, left/right: ";
            cout << sCurrState.dLSMasterPosition << " / ";
            cout << sCurrState.dRSMasterPosition << "." << endl;
         }
         break;

      case M_TURN_LEFT:
                // Tell the drive motors to turn left, with a radius of ~12"
                // We measured this with 0.4/-0.3, and that turns left with
                // a radius of 14"
                // The "mSeq.distance" parameter is only checked to see if it
                // is positive or negative, to determine which way to travel
                // while we are turning.
         if ( -0.001 < mSeq.distance ) {   // if requested distance is positive
            // Team4918Drive( 0.3, -0.24 );   // turn left while driving forward
            // Team4918Drive( 0.5, -0.40 );   // turn left while driving forward
            // Team4918Drive( 0.55, -0.44 );   // too much
            if ( mSeq.bDivertToCargo ) {
               Team4918Drive( 0.5, -0.47 );   // turn left tightly
            } else {
               Team4918Drive( 0.5, -0.44 );   // turn left widely
            }
         } else {                          // else
            Team4918Drive( -0.5, -0.44 );   // turn left tightly in reverse
         }
                                  // A left turn increases the angle
                                  // (like in trigonometry, not like a compass)
                                  // If we have turned far enough...
         if ( sCurrState.initialYaw + mSeq.yaw < sCurrState.yawPitchRoll[0] ) {
            cout << "EM: Left turn completed, heading: ";
            cout << sCurrState.yawPitchRoll[0]  << endl;
            bRetVal = true;                           // and exit this maneuver
         }
         break;

      case M_TURN_RIGHT:
                // Tell the drive motors to turn right, with a radius of ~12"
                // The "mSeq.distance" parameter is only checked to see if it
                // is positive or negative, to determine which way to travel
                // while we are turning.
         if ( -0.001 < mSeq.distance ) {   // if requested distance is positive
            // Team4918Drive( 0.3,  0.24 );   // turn right while driving forward
            // Team4918Drive( 0.5,  0.40 );   // turn right while driving forward
            // Team4918Drive( 0.5,  0.44 );   // turn right while driving forward
            // Team4918Drive( 0.55,  0.44 );   // too much
            if ( mSeq.bDivertToCargo ) {
               Team4918Drive( 0.5,  0.47 );   // turn right tightly
            } else {
               Team4918Drive( 0.5,  0.44 );   // turn right widely
            }
         } else {                          // else
            Team4918Drive( -0.5,  0.44 );   // turn right tightly in reverse
         }
                                  // A right turn decreases the angle
                                  // (like in trigonometry, not like a compass)
                                  // If we have turned far enough...
         if ( sCurrState.yawPitchRoll[0] < sCurrState.initialYaw + mSeq.yaw ) {
            cout << "EM: Right turn completed, heading: ";
            cout << sCurrState.yawPitchRoll[0]  << endl;
            bRetVal = true;                           // and exit this maneuver
         }
         break;

      case M_ROTATE:
                     // Rotate in place until reaching the specified yaw angle.
                     // The TurnToHeading() function returns true when it has
                     // rotated far enough.
         bRetVal = TurnToHeading( sCurrState.initialYaw + mSeq.yaw,
                                  false );
                                  // If we have rotated far enough...
         if ( bRetVal ) {
            cout << "EM: Rotation completed, heading: ";
            cout << sCurrState.yawPitchRoll[0]  << endl;
         }
         break;

      case M_WAIT:
         if ( mSeqPrev.type != mSeq.type ) {          // first call to M_WAIT?
                                         // yes; store number of ticks to wait
            dWaitCount = (int)mSeq.distance;
         } else {
            dWaitCount -= 1.0;
         }
                  // If we have waited long enough, or have been told to watch
                  // for a ball in the intake and there *is* one in the intake
         if ( ( dWaitCount <= 0.0 ) ||
              ( mSeq.bDivertToCargo && sCurrState.cargoInIntake ) ) {
            bRetVal = true;          // done waiting; change to next maneuver.
         } else {
            bRetVal = false;         // stay in this maneuver; keep waiting.
         }
         break;

      case M_LIMELOCK:
         sCurrState.joyX = 0.0;
         sCurrState.joyY = -0.1;  // Allow crawling forward (towards the
                                  // limelight target), to ease any turns.
                                  // This hurts our odometry, but is worth it.
         sCurrState.joyZ = 0.0;   // Set throttle to 1/2 power.
             // (together, joyY and joyZ result in about -0.025 forward speed)
         BUTTON_TARGET  = true;
         BUTTON_REVERSE = true;
         DriveToLimelightTarget();
         if ( mSeqPrev.type != mSeq.type ) {      // first call to M_LIMELOCK?
            iLimeLockCount = 100;   // yes; stay in M_LIMELOCK at least 2 secs
         } else {
            iLimeLockCount--;
         }
                         // If we are locked onto the goal (within 5 degrees),
                         // or have timed out
         if ( ( abs( limex ) < 5.0 ) || ( iLimeLockCount <= 0 ) ) {
            bRetVal = true; // done locking onto goal; change to next maneuver.
         } else {
            bRetVal = false;     // stay in this maneuver; keep trying to lock.
         }
         break;

      case M_SHOOT:
         Team4918Drive( 0.0, 0.0 );       // Make sure drive motors are stopped
         if ( mSeq.bDivertToCargo ) {
            BUTTON_TARGET  = true;    // Shoot based on limelight data.
            BUTTON_REVERSE = true;
         } else {
            sPrevState.conX = 0.0;    // Shoot at fixed 2350/2150 RPM.
            sCurrState.conX = 1.0;
         }
         RunShooter();
         Shoot();
         if ( mSeqPrev.type != mSeq.type ) {          // first call to M_SHOOT?
            iShootCount = 100;  // yes; stay in M_SHOOT for at least 2 seconds
         } else if ( ( iShootCount < 75 ) &&         // else if at least
              ( sCurrState.cargoInIntake ||       // 1 ball in conveyor
                sCurrState.cargoInPosition5 ) ) {
            iShootCount = 75;  // Then keep shooting for at least 1.5 seconds,
                               // even after no ball is detected in conveyor.
         } else {
            iShootCount--;
         }
         if ( 0 < iShootCount ) {
            bRetVal = false; // stay in this maneuver until balls are all gone.
         } else {
            bRetVal = true;  // done shooting; change to next maneuver.
         }
         break;

      case M_TERMINATE_SEQ:
         Team4918Drive( 0.0, 0.0 );       // Make sure drive motors are stopped
         if ( mSeqPrev.type != mSeq.type ) {
            cout << "EM: M_TERMINATE_SEQ: no further movement will happen!";
         }
         bRetVal = false;                 // and stay in this maneuver forever.
         break;

      default:
         cout << "EM: ERROR: Unknown maneuver type: ";
         cout << mSeq.type << "." << endl;
         break;
      }
              // failsafe maneuvers limit:
              // If more than 2000 seconds have passed, stop all maneuvers
      // if ( (2000 * 50) < icmdSeqManeuverCallCount ) {
      //    bRetVal = true;
      // }

      mSeqPrev = mSeq;   // Save a copy of this maneuver, to compare with
                         // the next maneuver we get.
 
      return bRetVal;
   }      // executeManeuver()


      /*---------------------------------------------------------------------*/
      /* executeManeuverSeq()                                                */
      /* This function is called with an index into the maneuver array,      */
      /* to perform a sequence of maneuvers starting at that index.          */
      /* Each maneuver will be performed to completion, then the next        */
      /* maneuver in the array will be started.                              */
      /* This function returns the index of the maneuver which should be     */
      /* executed on the next call (20 milliseconds later); until a          */
      /* maneuver is completed this will be same index it was called with.   */
      /*---------------------------------------------------------------------*/
   int executeManeuverSequence( int maneuverIndex ) {
      struct maneuver mSeqNext;
      // static int icmdSeqManeuverCallCount = 0;

                      // execute the current maneuver, and if it is finished...
      if ( executeManeuver( mSeq[ maneuverIndex ] ) ) {
         maneuverIndex++;                                // go to next maneuver
         mSeqNext = mSeq[maneuverIndex];
                                    // if next maneuver is a drive to distance,
         if ( M_DRIVE_STRAIGHT == mSeqNext.type ) {
                                    // then initialize the starting point
            DriveToDistance( mSeqNext.yaw,
                             mSeqNext.distance,
                             mSeqNext.bDivertToCargo,
                             true );
            // } else if ( M_ROTATE == mSeqNext.type ) {
                      // Else if next maneuver is a rotate, then set the
                      // initial (starting) yaw angle.
                      // NO: call TurnToHeading(..., true) just once in
                      // AutonomousInit(), and nowhere else, so all yaw values
                      // are always based on the initial yaw of the robot when
                      // AutonomousInit() was called.
                      // This allows all yaw values in the maneuver struct
                      // to be absolute, and relative to the initial yaw angle
                      // of the robot at AutonomousInit() time.
            //    TurnToHeading( mSeqNext.yaw, true ) ) {
         }
      }
                              // return either the current maneuver index, or
                              // if that just finished, the next maneuver index
      return maneuverIndex;
   }


      /*---------------------------------------------------------------------*/
      /* RobotInit()                                                         */
      /* This function is called once when the robot is powered up.          */
      /* It performs preliminary initialization of all hardware that will    */
      /* be used in Autonomous or Teleop modes.                              */
      /*---------------------------------------------------------------------*/
   void RobotInit() {
      static int iRobotInitCallCount = 0;

      // wpi::PortForwarder::GetInstance().Add(5800, "limelight.local", 5800);
      // wpi::PortForwarder::GetInstance().Add(5801, "limelight.local", 5801);
      // wpi::PortForwarder::GetInstance().Add(5805, "limelight.local", 5805);

      // see /home/team4918/wpilib/2022/utility/resources/app/resources/cpp/
      //      src/examples/DigitalCommunication/cpp/Robot.cpp
      // and /home/team4918/.gradle/caches/transforms-3/
      //      f997ad03a3ad7475ff8f27c762137217/transformed/
      //      wpilibc-cpp-2022.2.1-headers/frc/DriverStation.h
      if ( frc::DriverStation::kRed == frc::DriverStation::GetAlliance() ) {
         WeAreOnRedAlliance = true;    // we are on red alliance
         cout << "RobotInit(): We are on the Red Alliance." << endl;
      } else {
         WeAreOnRedAlliance = false;   // we are on blue alliance
         cout << "RobotInit(): We are on the Blue Alliance." << endl;
      }

      iRobotInitCallCount++;

      if ( 1 == iRobotInitCallCount ) {
         gyro.Calibrate();    // Calibrate Gyro on powerup, but never afterward
#ifdef VISION_PROCESSING
                                // start a thread processing USB camera images
         std::thread visionThread(VisionThread);
         visionThread.detach();
#endif  // if VISION_PROCESSING
      }

      cargoOnVideo.TestMode = false;

      m_motorLSFollow.Follow( m_motorLSMaster );    // For SparkMax/Neo motors
      m_motorRSFollow.Follow( m_motorRSMaster );
                  // Reduce CANbus bandwidth utilization for follower motors
                  // since the info the status frames provide isn't necessary
                  // (it is mostly just duplicates of the Master motor's info).
      m_motorLSFollow.SetPeriodicFramePeriod(
                      rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 100 );
      m_motorRSFollow.SetPeriodicFramePeriod(
                      rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 100 );
      m_motorLSFollow.SetPeriodicFramePeriod(
                      rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 500 );
      m_motorRSFollow.SetPeriodicFramePeriod(
                      rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 500 );
      m_motorLSFollow.SetPeriodicFramePeriod(
                      rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 500 );
      m_motorRSFollow.SetPeriodicFramePeriod(
                      rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 500 );

                                          // SparkMax/Neo motors (drive motors)
      MotorInitSpark( m_motorLSMaster, m_LSMasterPID, m_LSMasterEncoder );
      MotorInitSpark( m_motorRSMaster, m_RSMasterPID, m_RSMasterEncoder );
      m_LSMasterForwardLimitSwitch.EnableLimitSwitch( false );
      m_LSMasterReverseLimitSwitch.EnableLimitSwitch( false );
      m_RSMasterForwardLimitSwitch.EnableLimitSwitch( false );
      m_RSMasterReverseLimitSwitch.EnableLimitSwitch( false );

      MotorInit( m_motorTopShooter );
      MotorInit( m_motorBotShooter );
      m_motorTopShooter.ConfigPeakOutputReverse(  0.0, 10 );
      m_motorBotShooter.ConfigPeakOutputReverse(  0.0, 10 );
              // reduce CANbus bandwidth utilization for shooter motors,
              // since we don't need error info more than every 20 milliseconds
      // m_motorTopShooter.SetStatusFramePeriod(
      //    ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General,
      //      20, 0 );
      // m_motorBotShooter.SetStatusFramePeriod(
      //    ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General,
      //      20, 0 );

      MotorInitSparkBrushed( m_motorLSClimber );
      MotorInitSparkBrushed( m_motorRSClimber );
      m_motorLSClimber.SetIdleMode( rev::CANSparkMax::IdleMode::kBrake );
      m_motorRSClimber.SetIdleMode( rev::CANSparkMax::IdleMode::kBrake );
      m_motorLSClimber.SetSmartCurrentLimit( 60, 10, 5000 );
      m_motorRSClimber.SetSmartCurrentLimit( 60, 10, 5000 );

                    // Reduce CANbus bandwidth utilization for climber motors,
                    // since we don't care about their position, and they
                    // don't have encoders to report velocity anyway.
      m_motorLSClimber.SetPeriodicFramePeriod(
                      rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 500 );
      m_motorRSClimber.SetPeriodicFramePeriod(
                      rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 500 );
      m_motorLSClimber.SetPeriodicFramePeriod(
                      rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 500 );
      m_motorRSClimber.SetPeriodicFramePeriod(
                      rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 500 );
      m_LSClimberForwardLimitSwitch.EnableLimitSwitch( true );
      m_LSClimberReverseLimitSwitch.EnableLimitSwitch( true );
      m_RSClimberForwardLimitSwitch.EnableLimitSwitch( true );
      m_RSClimberReverseLimitSwitch.EnableLimitSwitch( true );

         // Reduce CANbus bandwidth utilization for intake and conveyor motors,
         // since we don't need error info more than every 20 milliseconds,
         // and don't need position or velocity at all (these two motors are
         // driven by victors without encoders, and can't provide values for
         // position or velocity).
      m_motorIntake.SetStatusFramePeriod(
            ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General,
                                          20, 0 );
      m_motorConveyMaster.SetStatusFramePeriod(
            ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General,
                                                20, 0 );
      m_motorIntake.SetStatusFramePeriod(
          ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0,
                                          255, 0 );
      m_motorConveyMaster.SetStatusFramePeriod(
          ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0,
                                                255, 0 );

                                    // invert encoder value positive/negative
                                    // and motor direction, for some motors.
      //    m_motorLSMaster.SetSensorPhase(true);
//    m_motorLSMaster.SetSensorPhase(false);
      //    m_motorRSMaster.SetSensorPhase(true);
//    m_motorRSMaster.SetSensorPhase(false);
  //  m_motorRSMaster.SetInverted(true);   // added 29jan2022
      //    m_motorTopShooter.SetSensorPhase(false);
      m_motorTopShooter.SetSensorPhase(false);  // this is necessary for PID
                                                // settings to work correctly
      m_motorTopShooter.SetInverted(false);
      m_motorBotShooter.SetSensorPhase(true);
      m_motorBotShooter.SetInverted(false);
//      m_motorLeftSideClimberPole.SetSensorPhase(false);
//      m_motorLeftSideClimberPole.SetInverted(false);
      m_motorLSClimber.SetInverted(false);
//      m_motorRightSideClimberPole.SetSensorPhase(false);
//      m_motorRightSideClimberPole.SetInverted(false);
      m_motorRSClimber.SetInverted(false);

  //  m_motorLSMaster.SetSelectedSensorPosition( 0, 0, 10 );
  //  m_motorRSMaster.SetSelectedSensorPosition( 0, 0, 10 );
  //  m_motorLSMaster.SetIntegralAccumulator( 0.0 );
  //  m_motorRSMaster.SetIntegralAccumulator( 0.0 );
  //  LSMotorState.targetVelocity_UnitsPer100ms = 0.0 * 4096 / 600;
  //  RSMotorState.targetVelocity_UnitsPer100ms = 0.0 * 4096 / 600;
      LSMotorState.targetVelocityRPM = 0.0;
      RSMotorState.targetVelocityRPM = 0.0;
      Team4918Drive( 0.0, 0.0 );          // make sure drive motors are stopped

                                                     // if encoder is connected
      if ( OK == m_motorTopShooter.ConfigSelectedFeedbackSensor(
                          FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10 ) ) {
         m_motorTopShooter.SelectProfileSlot( 0, 0 );
         m_motorTopShooter.Config_kF( 0, 0.035,    10 ); // 0.03 0.02 0.01
         m_motorTopShooter.Config_kP( 0, 0.20,    10 ); // 0.01 0.2 0.08 0.35
         m_motorTopShooter.Config_kI( 0, 0.0,     10 ); // was 0.0 0.00008
         m_motorTopShooter.Config_kD( 0, 0.0,     10 ); // 0.8 4.0
      } else {
         m_motorTopShooter.SelectProfileSlot( 0, 0 );
         m_motorTopShooter.Config_kF( 0, 0.01, 10 );   // may have to be higher
         m_motorTopShooter.Config_kP( 0, 0.0,  10 );
         m_motorTopShooter.Config_kI( 0, 0.0,  10 );
         m_motorTopShooter.Config_kD( 0, 0.0,  10 );
      }

                                                     // if encoder is connected
      if ( OK == m_motorBotShooter.ConfigSelectedFeedbackSensor(
                          FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10 ) ) {
         m_motorBotShooter.SelectProfileSlot( 0, 0 );
         m_motorBotShooter.Config_kF( 0, 0.035,    10 ); // 0.02 0.01
         m_motorBotShooter.Config_kP( 0, 0.20,    10 ); // 0.3 0.2 0.08 0.35
         m_motorBotShooter.Config_kI( 0, 0.0,     10 ); // 0.0 0.00008
         m_motorBotShooter.Config_kD( 0, 0.0,     10 ); // 0.8 4.0
      } else {
         m_motorBotShooter.SelectProfileSlot( 0, 0 );
         m_motorBotShooter.Config_kF( 0, 0.01, 10 );   // may have to be higher
         m_motorBotShooter.Config_kP( 0, 0.0,  10 );
         m_motorBotShooter.Config_kI( 0, 0.0,  10 );
         m_motorBotShooter.Config_kD( 0, 0.0,  10 );
      }

      m_motorConveyMaster.SetNeutralMode( NeutralMode::Brake );

      iCallCount++;
      sCurrState.teleop = false;

      // Default to a length of 100 LEDs (300 total; 100 sets of 3);
      // start with empty output on all of them.
      // length is expensive to set, so only set it once, then just update data
      m_led.SetLength( kLEDStripLength );
      m_led.SetData( m_ledBuffer );
      m_led.Start();

      LEDInit();   // initialize all LED sequences

   }      // RobotInit()


      /*---------------------------------------------------------------------*/
      /* RobotPeriodic()                                                     */
      /* This function is called every 20 milliseconds, regardless of what   */
      /* mode the robot is in (Autonomous, Teleop, or Test mode).            */
      /* If the robot is in one of those modes, this function is called      */
      /* immediately *after* the call to AutonomousPeriodic(),               */
      /* TeleopPeriodic(), or TestPeriodic().                                */
      /* If not in one of those 3 modes, the Roborio cannot drive any        */
      /* motors, but it can still check the joystick, joystick/console       */
      /* buttons, and sensors.                                               */
      /*---------------------------------------------------------------------*/
      void RobotPeriodic() override {
//       static int iCallCount = 0;      iCallCount++;
// 
//      GetAllVariables();
// 
//      SwitchCameraIfNecessary();
//      cout << "RobotPeriodic()" << endl;
      }


      /*---------------------------------------------------------------------*/
      /* DisabledInit()                                                      */
      /* This function is called once when the robot is disabled.            */
      /*---------------------------------------------------------------------*/
   void DisabledInit() override {

  //  m_motorLSMaster.SetSelectedSensorPosition( 0, 0, 10 );
  //  m_motorRSMaster.SetSelectedSensorPosition( 0, 0, 10 );
  //  m_motorLSMaster.SetIntegralAccumulator( 0.0 );
  //  m_motorRSMaster.SetIntegralAccumulator( 0.0 );
  //  LSMotorState.targetVelocity_UnitsPer100ms = 0.0 * 4096 / 600;
  //  RSMotorState.targetVelocity_UnitsPer100ms = 0.0 * 4096 / 600;
      LSMotorState.targetVelocityRPM = 0.0;
      RSMotorState.targetVelocityRPM = 0.0;
      Team4918Drive( 0.0, 0.0 );          // make sure drive motors are stopped

      cargoOnVideo.TestMode = false;
      // Should power off all motors here.
   }

      /*---------------------------------------------------------------------*/
      /* DisabledPeriodic()                                                  */
      /* This function is called every 20 milliseconds, as long as the robot */
      /* is in Disabled mode.                                                */
      /*---------------------------------------------------------------------*/
   void DisabledPeriodic() override {
   }

      /*---------------------------------------------------------------------*/
      /* TestInit()                                                          */
      /* This function is called once when the robot enters Test mode.       */
      /*---------------------------------------------------------------------*/
   void TestInit() override {
      limenttable->PutNumber( "ledMode", 1 );                  // turn LEDs off
      cargoOnVideo.TestMode = true;      // display the center pixel HSV values
   }

   void LEDBlue() {
      for ( int i = 0; i < kLEDStripLength; i++ ) {
                     // range of arg1 (hue) is 0-180;
                     // range of arg2 and arg3 (saturation and value) is 0-255
         // m_ledBuffer[i].SetHSV(100, 255, 128);  // half-bright blue
         m_ledBuffer[i].SetRGB( 0, 0, 255); // fully-saturated pure blue
         // m_ledBuffer[i].SetRGB( 255, 0, 0 );  // fully-saturated pure red
      }
      m_led.SetData( m_ledBuffer );
   }

   void LEDAllianceColor() {
      for ( int i = 0; i < kLEDStripLength; i++ ) {
         if ( WeAreOnRedAlliance ) {
            m_ledBuffer[i].SetRGB( 255, 0, 0 );  // fully-saturated pure red
         } else {
            m_ledBuffer[i].SetRGB( 0, 0, 255); // fully-saturated pure blue
         }
      }
      m_led.SetData( m_ledBuffer );
   }

   void LEDRainbow() {
              // for every pixel...
      for ( int i = 0; i < kLEDStripLength; i++ ) {
              // calculate the hue - hue is easier for rainbows because the
              // color shape is a circle, so only one value needs to increment
         const auto pixelHue = ( firstLEDStripPixelHue +
                                 (i * 7 + 180/kLEDStripLength) ) % 180;
              // set the value
         m_ledBuffer[i].SetHSV( pixelHue, 255, 128 );
      }
      m_led.SetData( m_ledBuffer );
         // Increase the first pixel hue, to make the rainbow move every 20 ns
      firstLEDStripPixelHue += 10;
         // check bounds
      firstLEDStripPixelHue %= 180;
      m_led.SetData( m_ledBuffer );
   }

   void LEDTest() {
      static int color=0;
               // Change the code below to reduce CPU congestion:
               // Use code based on the global iCallCount to perform various
               // parts of the LED buffer setting and enabling, culminating
               // in refreshing the LED strip values.  For example:
               // if ( 0 == iCallCount%5 )         {   // every 0.1 seconds
               //    Set up LEDs for tentacle 1
               // } else if ( 1 == iCallCount%5 )  {
               //    Set up LEDs for tentacle 2
               //      (... etc. ...)
               // } else if ( 2 == iCallCount%5 )  {
               //    m_led.SetData( m_ledBuffer );
               // }
               // Another change: For simple sequences (for example, lights
               // cascading up or down the tentacles), create a new function
               // LEDInit() to set up all of the m_ledBufferArr[x] buffers,
               // then change the below code to simply call functions like
               // these every 0.1 second or so:
               //     m_led.SetData( m_ledBufferArr[iCallCount%kNumLEDBufs] );
               // or, to go through the buffers in opposite order (3,2,1,0):
               //     m_led.SetData( m_ledBufferArr[
               //        (-iCallCount%kNumLEDBufs+kNumLEDBufs)%kNumLEDBufs] );
      // for every pixel...
      for ( int i = 0; i < kLEDStripLength; i++ ) {
         // int iabs = ( 179 * abs(50 - i) ) / 50;
         if ( ( 40 < i ) && ( i < 59 ) ) {         // if LEDs facing intake
            if ( WeAreOnRedAlliance ) {
               m_ledBuffer[i].SetRGB( 255, 0, 0 ); // fully-saturated pure red
            } else {
               m_ledBuffer[i].SetRGB( 0, 0, 255); // fully-saturated pure blue
            }
         } else if ( i == (color*kLEDStripLength)/180 ) {
              // set the value
            m_ledBuffer[i].SetHSV( color, 255, 128 );
         } else if ( ( 33 < i ) && ( i < 66 ) ) {  // LEDs curving up to top
            // m_ledBuffer[iabs].SetHSV( i*3, 255, 16 );
            m_ledBuffer[i].SetHSV( i*3, 255, 16 );
         } else if ( ( 30 < i ) && ( i < 69 ) ) { // LEDs curving down from top
            m_ledBuffer[i].SetHSV( i*3, 255, 16 );
         } else if ( ( 17 < i ) && ( i < 82 ) ) { // out to tip of aft tentacle
            m_ledBuffer[i].SetHSV( i*3, 255, 16 );
         } else {
            m_ledBuffer[i].SetHSV( i*3, 255, 16 );
         }
      }
      m_led.SetData( m_ledBuffer );
      color = (color+1)%180;
   }

                       // Blank all the LEDs in the strip.  This function is
                       // for convenience in testing only, so we don't have to
                       // look at the bright LEDs all the time while testing.
   void LEDBlank() {
      for ( int iBufNum = 0; iBufNum < kNumLEDBufs; iBufNum++ ) {
         for ( int iLEDNum = 0; iLEDNum < kLEDStripLength; iLEDNum++ ) {
            m_ledBufferArr[iBufNum][iLEDNum].SetRGB( 0, 0, 0 );
         }
      }
   }

                       // Set up all the LEDs buffers for the LED strip.
                       // This function fills all the m_ledBufferArr[x]
                       // buffers so that when written to the LED strip
                       // in sequence, orange LEDs cascade up or down,
                       // separated by unlit LEDs.
   void LEDInit() {
      for ( int iBufNum = 0; iBufNum < kNumLEDBufs; iBufNum++ ) {
         for ( int iLEDNum = 0; iLEDNum < kLEDStripLength; iLEDNum++ ) {
                                                       // if LEDs facing intake
            if ( ( 40 < iLEDNum ) && ( iLEDNum < 59 ) ) {
                        // Light these LEDs with our alliance color, to help
                        // the USB videocamera detect the cargo balls we want.
               if ( WeAreOnRedAlliance ) {
                                                  // fully-saturated pure red
                  m_ledBufferArr[iBufNum][iLEDNum].SetRGB( 192, 0, 0 );
               } else {
                                                  // fully-saturated pure blue
                  m_ledBufferArr[iBufNum][iLEDNum].SetRGB( 0, 0, 192);
               }
            } else if ( ( 71 < iLEDNum ) && ( iLEDNum < 77 ) ) {
                       // Light these LEDs green to help the limelight camera.
               m_ledBufferArr[iBufNum][iLEDNum].SetRGB( 0, 192, 0);
            } else if ( ( 21 < iLEDNum ) && ( iLEDNum < 27 ) ) {
                       // Light these LEDs green to help the limelight camera.
               m_ledBufferArr[iBufNum][iLEDNum].SetRGB( 0, 192, 0);
            } else {
                        // Light these LEDs with a moving cascade of orange.
               if ( iBufNum == ( abs(50 - iLEDNum) % kNumLEDBufs ) ) {
                  m_ledBufferArr[iBufNum][iLEDNum].SetHSV( 14, 255,  64 );
               } else {
                  // m_ledBufferArr[iBufNum][iLEDNum].SetHSV(
                  //                  (iLEDNum*180)/kLEDStripLength, 255, 16 );
                  m_ledBufferArr[iBufNum][iLEDNum].SetRGB( 0, 0, 0 );
               }
            }
         }
      }
   }  // LEDInit()

   void LEDCompetition() {
      const int iTicksTillChange = 5;    // Change every 5 0.2-second ticks
                                         // (every 0.1 second)
      if ( 0 == iCallCount%iTicksTillChange ) {
         int iFrame = iCallCount/iTicksTillChange;
         if ( BUTTON_REVERSE ) {    // if "reverse direction" button is pressed
                                 // move the lit LEDs in the opposite direction
            m_led.SetData(
                      m_ledBufferArr[ kNumLEDBufs-1 - (iFrame%kNumLEDBufs) ] );
         } else {
                                 // move the lit LEDs in the forward direction
            m_led.SetData( m_ledBufferArr[iFrame%kNumLEDBufs] );
         }
      }
   }  // LEDCompetition()

      /*---------------------------------------------------------------------*/
      /* TestPeriodic()                                                      */
      /* This function is called every 20 milliseconds, as long as the robot */
      /* is in Test mode.                                                    */
      /* In this mode the Roborio cannot drive any motors, but it can read   */
      /* the joystick, joystick/console buttons, and sensors.                */
      /*---------------------------------------------------------------------*/
   void TestPeriodic() override {
      GetAllVariables();
      iCallCount++;
      SwitchCameraIfNecessary();

      if ( 0 == iCallCount%10007 ) {                       // every 200 seconds
         // cout << "Sonar0 sensor 0: " << distSensor0.GetAverageValue() << endl;
         // cout << "Sonar0 average voltage:  " << distSensor0.GetAverageVoltage()
         //      << endl;
         // cout << "Sonar0 voltage:  " << distSensor0.GetVoltage() << endl;
             // convert to inches, with 100 centimeters/volt and 2.54 cm/inch
//       cout << "Sonar0 distance: " << distSensor0.GetVoltage() * 100 / 2.54
//            << " inches (" << distSensor0.GetVoltage() << ")." << endl; 
//       cout << "Sonar1 distance: " << distSensor1.GetVoltage() * 100 / 2.54
//            << " inches (" << distSensor1.GetVoltage() << ")." << endl; 
      }

      if ( 0 == iCallCount%100 )  {   // every 2 seconds
         // JoystickDisplay();
      }
//      cout << "TestPeriodic()" << endl;

      // LEDAllianceColor();         // light the entire LED strip blue
      // LEDBlue();         // light the entire LED strip blue
      // LEDRainbow();   // light the LED strip with a moving rainbow
      LEDTest();
   }


      /*---------------------------------------------------------------------*/
      /* AutonomousInit()                                                    */
      /* This function is called once when the robot enters Autonomous mode. */
      /*---------------------------------------------------------------------*/
   void AutonomousInit() override {
      RobotInit();
      m_compressor.EnableDigital();
      iCallCount=0;
                                                            // drop the intake
      m_flipperSolenoid.Set( frc::DoubleSolenoid::Value::kReverse);
      GetAllVariables();
      sCurrState.teleop = false;
      sCurrState.initialYaw = sCurrState.yawPitchRoll[0]; 
      LSMotorState.targetVelocityRPM = 0.0;
      RSMotorState.targetVelocityRPM = 0.0;
      Team4918Drive( 0.0, 0.0 );          // make sure drive motors are stopped

                             // mSeqIndex can be set to different values,
                             // based on the console switches.
      if ( BUTTON_SWITCH1 ) {
         mSeqIndex =  0;               // simple drive auto (no shooting)
      } else if ( BUTTON_SWITCH2 ) {
         mSeqIndex = 10;               // 2-ball auto
      } else if ( BUTTON_SWITCH3 ) {
         mSeqIndex = 30;               // 3-ball auto
      } else if ( BUTTON_SWITCH4 ) {
         mSeqIndex = 60;               // 5-ball auto
      } else {
         mSeqIndex = 10;             // 2-ball auto
         // mSeqIndex =  8;             // no switch flipped, do nothing
                                     // (points to M_TERMINATE_SEQ, for safety)
      }

                             // Initialize yaw and distance, so next maneuvers
                             // are relative to these current settings.
      TurnToHeading( sCurrState.initialYaw, true );
      DriveToDistance( sCurrState.initialYaw, 0.0, false, true );
   }      // AutonomousInit()


      /*---------------------------------------------------------------------*/
      /* AutonomousPeriodic()                                                */
      /* This function is called every 20 milliseconds, as long as the robot */
      /* is in Autonomous mode.                                              */
      /*---------------------------------------------------------------------*/
   void AutonomousPeriodic() override {

      // static double dDesiredYaw = 0.0;

      GetAllVariables();

                // Set joyZ value, so throttle calculations in Team4918Drive()
                // turn out to be what we want them to be.
                //    dThrottle = ( 1.0-sCurrState.joyZ ) / 2.0;
                //    so 1.0 is minimal throttle, -1.0 is maximum)
      // sCurrState.joyZ =  1.0;   // minimum speed (about .1 * full throttle)
      // sCurrState.joyZ =  0.5;   // one-quarter speed
      // sCurrState.joyZ =  0.0;   // half speed
      sCurrState.joyZ = -0.5;   // three-quarter speed
      // sCurrState.joyZ = -1.0;   // full speed
// Comment this line out to test with actual throttle paddle
//    sCurrState.joyZ = 0.5;   // one-quarter speed

      iCallCount++;


      autoConveyor = true;    // Select fully-auto intake/conveyor mode
      RunIntake();
      RunConveyor();

      // m_drive.StopMotor();
      LSMotorState.targetVelocityRPM = 0.0;                // Left Side drive
      RSMotorState.targetVelocityRPM = 0.0;                // Right Side drive

      // dDesiredYaw = sCurrState.initialYaw;

                            // Perform a sequence of maneuvers, transitioning
                            // to next maneuver in the sequence when necessary.
      mSeqIndex = executeManeuverSequence( mSeqIndex );
      
      motorFindMinMaxVelocitySpark( m_LSMasterEncoder, LSMotorState );
      motorFindMinMaxVelocitySpark( m_RSMasterEncoder, RSMotorState );

      LEDCompetition();   // Alliance color at front, green limelight color at
                          // back, advancing orange LEDS separated by unlit
                          // LEDs everywhere else.

      return; 

   }      // AutonomousPeriodic()


      /*---------------------------------------------------------------------*/
      /* TeleopInit()                                                        */
      /* This function is called once when the robot enters Teleop mode.     */
      /*---------------------------------------------------------------------*/
   void TeleopInit() override {
      RobotInit();
      autoConveyor = false;
      m_compressor.EnableDigital();
                                                    // zero the drive encoders
  //  m_motorLSMaster.SetSelectedSensorPosition( 0, 0, 10 );
  //  m_motorRSMaster.SetSelectedSensorPosition( 0, 0, 10 );
  //  m_motorLSMaster.SetIntegralAccumulator( 0.0 );
  //  m_motorRSMaster.SetIntegralAccumulator( 0.0 );
  //  m_motorLSMaster.ConfigClosedloopRamp(0.0);
  //  m_motorRSMaster.ConfigClosedloopRamp(0.0);
  //  m_motorLSMaster.ConfigOpenloopRamp(  0.0);
  //  m_motorRSMaster.ConfigOpenloopRamp(  0.0);
  //  LSMotorState.targetVelocity_UnitsPer100ms = 0.0 * 4096 / 600;
  //  RSMotorState.targetVelocity_UnitsPer100ms = 0.0 * 4096 / 600;
      sCurrState.teleop = true;
      LSMotorState.targetVelocityRPM = 0.0;
      RSMotorState.targetVelocityRPM = 0.0;
      Team4918Drive( 0.0, 0.0 );          // make sure drive motors are stopped
   }      // TeleopInit()


      /*---------------------------------------------------------------------*/
      /* TeleopPeriodic()                                                    */
      /* This function is called every 20 milliseconds, as long as the robot */
      /* is in Teleop mode.                                                  */
      /*---------------------------------------------------------------------*/
   void TeleopPeriodic() override {
      static bool bFlipperState = false;                  // Is intake raised?
                          // (Assume it starts *not* raised, because that is
                          //  the state Autonomous will probably have left it)

      GetAllVariables();  // this is necessary if we use any Canbus variables.
      AdjustJoystickValues();  // adjust for joystick deadband and sensitivity
#ifdef DISP_SMARTDASHBOARD
      AdjustPIDValues();       // adjust Drive PID values, if changed on
                               // SmartDashboard
#endif

      // cout << "yawrate: " << sCurrState.rateXYZ[2] << endl;

                                                  // raise or lower the intake
      if ( !BUTTON_UPPYDOWNEY_PREV && BUTTON_UPPYDOWNEY ) {
         if ( bFlipperState ){
                                                           // lower the intake
            m_flipperSolenoid.Set( frc::DoubleSolenoid::Value::kReverse);
         } else {
                                                           // raise the intake
            m_flipperSolenoid.Set( frc::DoubleSolenoid::Value::kForward);
         }
         bFlipperState = !bFlipperState; 
      }

      RunDriveMotors();

      RunShooter();

      Shoot();

      RunIntake();
      RunConveyor();

      RunBlueClimberPole( m_motorLSClimber, m_LSClimberForwardLimitSwitch,
                                        m_LSClimberReverseLimitSwitch );
      RunRedClimberPole( m_motorRSClimber, m_RSClimberForwardLimitSwitch,
                                        m_RSClimberReverseLimitSwitch );
      SwitchCameraIfNecessary();

      if ( 0 == iCallCount%100000 )  {   // every 2000 seconds
         // cout << "TelPeriodic loop duration: ";
         // cout << frc::GetTime() - dTimeOfLastCall << endl;
               // use frc:Timer::GetFPGATimestamp() instead?
         // LEDInit();   // initialize LED sequences
      }
      // LEDAllianceColor();         // light the entire LED strip blue or red
      // LEDTest();       // Alliance color at intake, rainbow everywhere else
      LEDCompetition();   // Alliance color at front, green limelight color at
                          // back, advancing orange LEDS separated by unlit
                          // LEDs everywhere else.

      iCallCount++;
   }      // TeleopPeriodic()
 
};     // class Robot definition (derives from frc::TimedRobot )


Robot::sCargoOnVideo Robot::cargoOnVideo = { true, 0, 0, -1, false, false };
bool Robot::WeAreOnRedAlliance = { true };

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

