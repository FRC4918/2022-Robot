/*
 * FRC Team 4918 2021 Competition Robot C/C++ code.
 * (C) Team 4918, the Roboctopi, original creation on 20feb2020.
 * This code was derived from the MentorBot code.
 */

// #include <frc/WPILib.h>  // uncomment to include everything
#include "ctre/Phoenix.h"
#include "frc/AnalogInput.h"
#include "frc/BuiltInAccelerometer.h"
#include "frc/PneumaticsModuleType.h"
#include "frc/Compressor.h"
#include "frc/DigitalInput.h"
#include "frc/DigitalOutput.h"
#include "frc/DigitalSource.h"
#include "frc/DoubleSolenoid.h"
#include "frc/Joystick.h"
#include "frc/Servo.h"
#include "frc/Solenoid.h"
#include "frc/TimedRobot.h"
#include "frc/Timer.h"
// #include "frc/RobotDrive.h"
#include "frc/drive/DifferentialDrive.h"
#include "frc/AddressableLED.h"
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
#include <unistd.h>
#include <sstream>

using std::cout;
using std::endl;
using std::setw;
using std::setfill;         // so we can use "setfill('0') in cout streams
using std::abs;
using namespace cv;

#define JAG_LOOKING_FOR_RED 1
// #define JAG_LOOKING_FOR_BLUE 1
#ifndef JAG_LOOKING_FOR_RED
  #define JAG_LOOKING_FOR_BLUE 1   // if not looking for red, look for blue
#endif

class Robot : public frc::TimedRobot {
 private:

         // Note for future: Need to add a gyro here.
   WPI_TalonSRX m_motorLSMaster{      0 };   // Left  side drive motor
   WPI_TalonSRX m_motorRSMaster{     15 };   // Right side drive motor   
   WPI_VictorSPX m_motorLSSlave1{     1 };   // Left  side slave motor
   WPI_VictorSPX m_motorLSSlave2{     2 };   // Left side slave motor
   WPI_VictorSPX m_motorRSSlave1{    14 };   // Right side slave motor
   WPI_VictorSPX m_motorRSSlave2{    13 };   // Right side slave motor
   WPI_TalonSRX m_motorTopShooter{    3 };   // top motor on shooter
   WPI_TalonSRX m_motorBotShooter{   12 };  // bottom motor on shooter
   WPI_TalonSRX m_motorClimberPole{   4 };  // telescoping pole motor
   WPI_VictorSPX m_motorConveyMaster{ 5 };  // conveyor motor 1
   WPI_VictorSPX m_motorConveySlave{ 10 };  // conveyor motor 2
   WPI_TalonSRX m_motorClimberWinch{ 11 };  // winch motor
   WPI_VictorSPX m_motorIntake{       6 };  // intake motor
   WPI_VictorSPX m_motorFlippyFlippy{ 9 }; //  motor to spin the color wheel 
   frc::Compressor m_compressor{ 0, frc::PneumaticsModuleType::CTREPCM }; // CTRE compressor

   frc::Solenoid m_shiftingSolenoid{ 0, frc::PneumaticsModuleType::CTREPCM, 7};
   frc::DoubleSolenoid m_flipperSolenoid{ 0, frc::PneumaticsModuleType::CTREPCM, 0, 1};  // are these numbers correct?

   PigeonIMU    pigeonIMU{ 1 };

   frc::DigitalInput conveyorDIO0{0};
   frc::DigitalInput conveyorDIO1{1};
   
   int iAutoCount;
   float drivePowerFactor = 0.8;
   frc::Joystick m_stick{0};
   frc::Joystick m_console{1};
   // frc::DifferentialDrive m_drive{ m_motorLSMaster, m_motorRSMaster };
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
      M_SHIFT_LOW      = 5,  // shift into low gear
      M_SHIFT_HIGH     = 6,  // shift into high gear
      M_TERMINATE_SEQ  = 7 
   };

                 // create a struct which can contain a full maneuver
		 // (type, distance, yaw (heading), etc.)
   struct maneuver {
      int                index;      // index of this element in an array of
                                     // maneuvers
      enum MANEUVER_TYPE type;       // type of maneuver (stop, turn, etc.)
      double             distance;   // distance in feet
      double             yaw;        // yaw angle in degrees
      bool               bDivertToPcell;  // if true, divert to a powercell
                                          // if one is seen by the videocamera
   };

   int mSeqIndex = 0;

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
     // index command             (feet)    positive left)  powercell?
     // ----- ----------------     ----     --------        -----
      // index 0: (ballgrabber)
      {   0,  M_DRIVE_STRAIGHT,     4.0,       0.0,         false },
      {   1,  M_TURN_RIGHT,         0.0,     -26.0,         false },
      {   2,  M_DRIVE_STRAIGHT,     7.5,     -30.0,         true  },
      {   3,  M_TURN_LEFT,          0.0,      70.0,         false },
      {   4,  M_DRIVE_STRAIGHT,     8.0,      75.0,         true  },
      {   5,  M_ROTATE,             0.0,       0.0,         false },
      {   6,  M_DRIVE_STRAIGHT,     1.0,       0.0,         false },
      {   7,  M_DRIVE_STRAIGHT,     9.0,       0.0,         true  },
      {   8,  M_STOP,               0.0,       0.0,         false },
      {   9,  M_TERMINATE_SEQ,      0.0,       0.0,         false },

      // index 10:
      {  10,  M_STOP,               0.0,       0.0,         false },
      {  11,  M_STOP,               0.0,       0.0,         false },
      {  12,  M_DRIVE_STRAIGHT,     2.0,       0.0,         false },
      {  13,  M_TURN_LEFT,          0.0,      90.0,         false },
      {  14,  M_DRIVE_STRAIGHT,     3.0,      90.0,         false },
      {  15,  M_DRIVE_STRAIGHT,    -3.0,      90.0,         false },
      {  16,  M_ROTATE,             0.0,     -20.0,         false },
      {  17,  M_DRIVE_STRAIGHT,     6.0,     -60.0,         false },
      {  18,  M_TURN_LEFT,          0.0,      90.0,         false },
      {  19,  M_DRIVE_STRAIGHT,     4.0,      90.0,         false },
      {  20,  M_STOP,               0.0,       0.0,         false },
      {  21,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  22,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  23,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  24,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  25,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  26,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  27,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  28,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  29,  M_TERMINATE_SEQ,      0.0,       0.0,         false },

      // index 30:
      {  30,  M_DRIVE_STRAIGHT,     5.0,       0.0,         false },
      {  31,  M_TURN_RIGHT,         0.0,     -26.0,         false },
      {  32,  M_DRIVE_STRAIGHT,     5.5,     -26.0,         false },
      {  33,  M_TURN_LEFT,          0.0,      70.0,         false },
      {  34,  M_DRIVE_STRAIGHT,     5.5,      70.0,         false },
      {  35,  M_ROTATE,             0.0,       0.0,         false },
      {  36,  M_DRIVE_STRAIGHT,    11.0,       0.0,         false },
      {  37,  M_STOP,               0.0,       0.0,         false },
      {  38,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  39,  M_TERMINATE_SEQ,      0.0,       0.0,         false },

      // index 40:
      {  40,  M_TURN_LEFT,          0.0,     360.0,         false },
      {  41,  M_DRIVE_STRAIGHT,    10.0,     360.0,         false },
      {  42,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  43,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  44,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  45,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  46,  M_TURN_RIGHT,         0.0,       0.0,         false },
      {  47,  M_ROTATE,             0.0,     360.0,         false },
      {  48,  M_ROTATE,             0.0,       0.0,         false },
      {  49,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  50,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  51,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  52,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  53,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  54,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  55,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  56,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  57,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  58,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  59,  M_TERMINATE_SEQ,      0.0,       0.0,         false },

      // index 60: (barrel) (works)
      {  60,  M_STOP,               0.0,       0.0,         false },
      {  61,  M_DRIVE_STRAIGHT,     8.5,       0.0,         false },
      {  62,  M_TURN_RIGHT,         0.0,    -325.0,         true },
      {  63,  M_DRIVE_STRAIGHT,     8.5,    -330.0,         false },
      {  64,  M_TURN_LEFT,          0.0,     -68.0,         true }, // was -70
      {  65,  M_DRIVE_STRAIGHT,     6.7,     -55.0,         false }, // was 7.3
      {  66,  M_TURN_LEFT,          0.0,     148.0,         true },
      {  67,  M_SHIFT_HIGH,         0.0,       0.0,         true },
      {  68,  M_DRIVE_STRAIGHT,    22.0,     178.0,         false }, // was 180
      {  69,  M_STOP,               0.0,       0.0,         false },
      {  70,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  71,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  72,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  73,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  74,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  75,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  76,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  77,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  78,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  79,  M_TERMINATE_SEQ,      0.0,       0.0,         false },

      // index 80: (slalom) (works)
      {  80,  M_STOP,               0.0,       0.0,         false },
      {  81,  M_DRIVE_STRAIGHT,     1.0,       0.0,         false },
      {  82,  M_TURN_LEFT,          0.0,      40.0,         false }, // was 45
      {  83,  M_DRIVE_STRAIGHT,     1.5,      60.0,         false },
      {  84,  M_TURN_RIGHT,         0.0,      15.0,         false },
      {  85,  M_DRIVE_STRAIGHT,    10.0,       0.0,         false }, // was -5
      {  86,  M_TURN_RIGHT,         0.0,     -40.0,         true  }, // was -45
      {  87,  M_DRIVE_STRAIGHT,     2.2,     -50.0,         false },
      {  88,  M_TURN_LEFT,          0.0,     215.0,         true  },
      {  89,  M_DRIVE_STRAIGHT,     2.5,     230.0,         false },
      {  90,  M_TURN_RIGHT,         0.0,     194.0,         false },
      {  91,  M_DRIVE_STRAIGHT,     9.5,     184.0,         false },
      {  92,  M_TURN_RIGHT,         0.0,     140.0,         false },
      {  93,  M_DRIVE_STRAIGHT,     4.5,     135.0,         false },
      {  94,  M_STOP,               0.0,       0.0,         false },
      {  95,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  96,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  97,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  98,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      {  99,  M_TERMINATE_SEQ,      0.0,       0.0,         false },

      // index 100: (bounce) (works)
      { 100,  M_STOP,               0.0,       0.0,         false },
      { 101,  M_STOP,               0.0,       0.0,         false },
      { 102,  M_DRIVE_STRAIGHT,     1.2,       0.0,         false },
      { 103,  M_TURN_LEFT,          0.0,      75.0,         true },
      { 104,  M_DRIVE_STRAIGHT,     1.4,      90.0,         false },
      { 105,  M_DRIVE_STRAIGHT,    -0.5,      90.0,         false },
      { 106,  M_TURN_LEFT,         -1.0,     105.0,         true },
      { 107,  M_DRIVE_STRAIGHT,    -5.5,     120.0,         false },
      { 108,  M_TURN_LEFT,         -1.0,     165.0,         true },
      { 109,  M_DRIVE_STRAIGHT,    -0.5,     180.0,         false },
      { 110,  M_TURN_LEFT,         -1.0,     255.0,         true },
      { 111,  M_DRIVE_STRAIGHT,    -6.0,     270.0,         false },
      { 112,  M_DRIVE_STRAIGHT,     6.0,     270.0,         false },
      { 113,  M_TURN_LEFT,          1.0,     350.0,         true },
      { 114,  M_DRIVE_STRAIGHT,     3.0,     360.0,         false },
      { 115,  M_TURN_LEFT,          1.0,     430.0,         true },
      { 116,  M_DRIVE_STRAIGHT,     6.0,     450.0,         false },
      { 117,  M_TURN_LEFT,         -1.0,     470.0,         true },
      { 118,  M_DRIVE_STRAIGHT,    -2.0,     490.0,         false },
      { 119,  M_STOP,               0.0,       0.0,         false },
      { 120,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      { 121,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      { 122,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      { 123,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      { 124,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      { 125,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      { 126,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      { 127,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      { 128,  M_TERMINATE_SEQ,      0.0,       0.0,         false },
      { 129,  M_TERMINATE_SEQ,      0.0,       0.0,         false },

   };


   struct sState {
      double joyX;
      double joyY;
      double joyZ;
      double conX;
      double conY;
      bool   joyButton[12];
      bool   conButton[13];
      int    iLSMasterPosition;
      int    iRSMasterPosition;
      int    iTSMasterPosition; // Top Shooter
      int    iBSMasterPosition; // Bottom Shooter
      int    iLSMasterVelocity;
      int    iRSMasterVelocity;
      int    iTSMasterVelocity; // Top Shooter
      int    iBSMasterVelocity; // Bottom Shooter
      int    iIntakePercent;
      int    iConveyPercent;
      double yawPitchRoll[3];  // data from Pigeon IMU
      double initialYaw;
      bool   powercellInIntake;
      bool   powercellInPosition5;
      bool   highGear;
      bool   teleop;
   } sCurrState, sPrevState;

   struct sMotorState {
      double targetVelocity_UnitsPer100ms;
      double sensorVmax, sensorVmin;
   };

   struct sMotorState LSMotorState, RSMotorState, TSMotorState, BSMotorState;

   bool aBooleanVariable = false;    // just an example of a boolean variable
   int    iCallCount = 0;
   int    state = 0; 
   // double dTimeOfLastCall = 0.0;
   // units::time::second_t dTimeOfLastCall = (units::second_t) 0;

   static constexpr int kLEDStripLength = 60;
   // PWM port 9
   // This must be a PWM header, not MXP or DIO
   frc::AddressableLED m_led{9};
   std::array<frc::AddressableLED::LEDData, kLEDStripLength>
	   m_ledBuffer;  // Reuse the buffer
   // Store what the last hue of the first pixel is:
   int firstLEDStripPixelHue = 0;

 public:

   static struct sPowercellOnVideo {
      bool SeenByCamera;
      int  X;
      int  Y;
      int  Radius;
      bool SwitchToColorWheelCam;
      bool TestMode;
   } powercellOnVideo;

 private:
                 /* limelight variables: x: offset from centerline,         */
                 /*                      y: offset from centerline,         */
                 /*                      a: area of target, % (0-100),      */
                 /*                      v: whether the data is valid,      */
                 /*                      s: skew or rotation, deg (-90-0).  */
   double limex, limey, limea, limev, limes;

      /*---------------------------------------------------------------------*/
      /* VisionThread()                                                      */
      /* This function executes as a separate thread, to take 640x480-pixel  */
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

      int iBiggestCircleIndex = -1;
      int iBiggestCircleRadius = -1;

      bool bDiagnosticMode = false;

                           // jag; 08jan2022: use of GetInstance is deprecated;
			   // eliminate the commented-out lines once the direct
			   // static syntax is proved to work.
//    cs::UsbCamera camera =
//                 frc::CameraServer::GetInstance()->StartAutomaticCapture(1);
      cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture(1);
         //camera.SetResolution( 640, 480 );   // too detailed and slow
         //camera.SetResolution( 160, 120 );   // too coarse
      camera.SetResolution( 320, 240 );        // just right
//    cs::UsbCamera camera2 =
//                frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
      cs::UsbCamera camera2 = frc::CameraServer::StartAutomaticCapture(0);
      camera2.SetResolution( 320, 240 );

      powercellOnVideo.SeenByCamera = false;  // Make sure no other code thinks
      powercellOnVideo.X = 0;                 // we see a powercell until we
      powercellOnVideo.Y = 0;                 // actually see one!
      powercellOnVideo.Radius = -1;
      powercellOnVideo.SwitchToColorWheelCam = false;
      powercellOnVideo.TestMode = false;

//    cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();
      cs::CvSink cvSink = frc::CameraServer::GetVideo();
      cs::CvSource outputStreamStd =
//            frc::CameraServer::GetInstance()->PutVideo( "Gray", 320, 240 );
              frc::CameraServer::PutVideo( "Gray", 320, 240 );
              //frc::CameraServer::GetInstance()->GetVideo();
      cv::Mat source;
      cv::Mat output;
      cv::Mat hsvImg;
      cv::Mat threshImg;
      cv::Mat *pOutput;
#ifdef JAG_LOOKING_FOR_RED
      cv::Mat lower_red_hue_range;
      cv::Mat upper_red_hue_range;
#endif

      std::vector<Vec3f> v3fCircles;      // 3-element vector of floats;
                                          // this will be the pass-by reference
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
          // determine good values for a red (2/29?) or a blue (180/230?) ball,
          // then add initialization code to switch to the red or blue values
          // depending on which alliance we are on for each match.
          // And don't forget to check under different lighting conditions!

#ifdef JAG_LOOKING_FOR_RED
      int     lowH = 200;      // Set Hue range for red (range is 0-255)
      int     highH = 40;      //   (these values select 200-255 OR 0-40)
#elif JAG_LOOKING_FOR_YELLOW
      int     lowH = 30;       // Set Hue for yellow
      int     highH = 65;      //
#elif JAG_LOOKING_FOR_BLUE
      int     lowH = 120;      // Set Hue range for blue
      int     highH = 180;     //
#else   // else assume we are looking for blue
      int     lowH = 120;      // Set Hue range for blue
      int     highH = 180;     //
#endif

      int     lowS = 110;       // Set Saturation (prev: 0)
      int     highS = 255;

      int     lowV = 0;         // Set Value (orig: 102)
      int     highV = 255;      // (orig: 225)

      cvSink.SetSource( camera );
      while( true ) {
         static int iFrameCount = 0;
         static int iLoopCount  = 0;

         if ( powercellOnVideo.SwitchToColorWheelCam &&
              !prevSwitchToColorWheelCam ) {
            cvSink.SetSource( camera2 );       // switch to color wheel camera
            prevSwitchToColorWheelCam = powercellOnVideo.SwitchToColorWheelCam;
         } else if ( !powercellOnVideo.SwitchToColorWheelCam &&
                     prevSwitchToColorWheelCam ) {
            cvSink.SetSource( camera );     // switch back to powercell camera
            prevSwitchToColorWheelCam = powercellOnVideo.SwitchToColorWheelCam;
            bDiagnosticMode = !bDiagnosticMode;       // toggle diagnostic mode
            cout << "mel says diagnostic mode was changed" <<endl;
            cout << bDiagnosticMode <<endl; 
         }

                   // set the pointer to the frame to be sent to DriverStation
         if ( bDiagnosticMode ) {
            pOutput = &threshImg;   // diagnostic image (shows where yellow is,
                                    //                         or red or blue))
         } else if ( powercellOnVideo.SwitchToColorWheelCam ) {
            pOutput = &source;          // full-color, direct from videocamera
         } else {
            pOutput = &output;            // gray-scale image, for low latency
         }

         usleep( 1000 );                               // wait one millisecond
         if ( cvSink.GrabFrame(source) )
         {
            // dTimeOfLastCall = frc::GetTime();
            iFrameCount++;
            cvtColor( source, output, cv::COLOR_BGR2GRAY );
            cvtColor( source, hsvImg, cv::COLOR_BGR2HSV );
#ifdef JAG_LOOKING_FOR_RED
	    // jag; code adapted from:
	    //  https://cppsecrets.com/users/
	    //  18989711511997116104103495564103495564103109971051084699111109/
	    //  C00-OpenCV-cvinRange.php
	    //  Threshold the HSV image; keep only the red pixels
            cv::inRange( hsvImg, cv::Scalar(0, lowS, lowV),
                         cv::Scalar(highH, highS, highV), lower_red_hue_range );
            cv::inRange( hsvImg, cv::Scalar( lowH, lowS, lowV),
                         cv::Scalar(255, highS, highV), upper_red_hue_range );
	    // Combine the above two images
	    cv::addWeighted( lower_red_hue_range, 1.0,
			     upper_red_hue_range, 1.0, 0.0, threshImg );
#else
            cv::inRange( hsvImg, cv::Scalar(lowH, lowS, lowV),
                         cv::Scalar(highH, highS, highV), threshImg );
#endif
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
                              84,  // p2: increase this to reduce false circles
                              24,  // was 20           // minimum circle radius
                              64 );  // was 56         // maximum circle radius
                              // was: threshImg.rows / 4, 100, 50, 10, 800 );

            iBiggestCircleIndex = -1;     // init to an impossible index
            iBiggestCircleRadius = -1;    // init to an impossibly-small radius

                                                             // for each circle
            for ( unsigned int i = 0; i < v3fCircles.size(); i++ ) {

               // if ( 0 == iFrameCount%6007 ) {     // every 200 seconds or so
               if ( 0 == iFrameCount%100 ) {         // every 4 seconds or so
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
               powercellOnVideo.X = (int)v3fCircles[iBiggestCircleIndex][0] -
                                                         (int)threshImg.cols/2;
               powercellOnVideo.Y = (int)threshImg.rows/2 -
                                       (int)v3fCircles[iBiggestCircleIndex][1];
               powercellOnVideo.Radius = iBiggestCircleRadius;
               powercellOnVideo.SeenByCamera = true;
            } else {
               powercellOnVideo.SeenByCamera = false;
            }

            if ( powercellOnVideo.TestMode ) {              // if in Test Mode
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
         iLoopCount++;
      }   // do while 
   }      // VisionThread() 

 public:

      /*---------------------------------------------------------------------*/
      /* SwitchCameraIfNecessary()                                           */
      /* This function switches USB cameras, in case they were initialized   */
      /* in a different order than usual.                                    */
      /*---------------------------------------------------------------------*/
   void SwitchCameraIfNecessary( void ) {
              // Switch cameras if console button 5 pressed.
      if ( !sPrevState.conButton[5] &&
            sCurrState.conButton[5]    ) {

         powercellOnVideo.SwitchToColorWheelCam =
                   !powercellOnVideo.SwitchToColorWheelCam;

         cout << "Switching camera to ";
         if ( powercellOnVideo.SwitchToColorWheelCam ) {
            cout << "ColorWheelCam" << endl;
         } else {
            cout << "PowercellCam" << endl;
         }
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

                                // use frc::Timer::GetFPGATimestamp() instead?
      // dTimeOfLastCall = frc::GetTime();
//      cout << std::setw( 20 ) << std::setprecision( 16 ) << dTimeOfLastCall << " ";


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

      sCurrState.iLSMasterPosition = m_motorLSMaster.GetSelectedSensorPosition();
      sCurrState.iRSMasterPosition = m_motorRSMaster.GetSelectedSensorPosition();
      sCurrState.iTSMasterPosition = m_motorTopShooter.GetSelectedSensorPosition();
      sCurrState.iBSMasterPosition = m_motorBotShooter.GetSelectedSensorPosition();
      sCurrState.iLSMasterVelocity = m_motorLSMaster.GetSelectedSensorVelocity();
      sCurrState.iRSMasterVelocity = m_motorRSMaster.GetSelectedSensorVelocity();
      sCurrState.iTSMasterVelocity = m_motorTopShooter.GetSelectedSensorVelocity();
      sCurrState.iBSMasterVelocity = m_motorBotShooter.GetSelectedSensorVelocity();

      pigeonIMU.GetYawPitchRoll( sCurrState.yawPitchRoll );

                  // Record the positions of powercells in the conveyor system.
                  // The Digital I/O (DIO) connections are made to IR sensors,
                  // which return false if the IR beam is blocked (which means
                  // there is a powercell there) -- so we invert them here. 
      sCurrState.powercellInIntake    = !conveyorDIO0.Get();
      sCurrState.powercellInPosition5 = !conveyorDIO1.Get();

      limev = limenttable->GetNumber("tv",0.0);  // valid
      limex = limenttable->GetNumber("tx",0.0);  // x position
      limea = limenttable->GetNumber("ta",0.0);  // area
      limey = limenttable->GetNumber("ty",0.0);  // y position
      limes = limenttable->GetNumber("ts",0.0);  // skew
   }      // GetAllVariables()


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
         cout << "pigeonIMU (yaw/pitch/roll): " <<
               sCurrState.yawPitchRoll[0] << "/" <<
               sCurrState.yawPitchRoll[1] << "/" <<
               sCurrState.yawPitchRoll[2] << endl;
//         cout << "pigeontemp: " << pigeonIMU.GetTemp() << endl; 
   }      // IMUOrientationDisplay()


      /*---------------------------------------------------------------------*/
      /* MotorDisplay()                                                      */
      /* Display all the current motor values for a specified motor on the   */
      /* console log.                                                        */
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
      /* motorFindMinMaxVelocity()                                           */
      /* Keep a motor's Vmin and Vmax updated, for display on the            */
      /* console log.                                                        */
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
      double  leftMotorOutput = 0.0;
      double rightMotorOutput = 0.0;
      // m_drive.StopMotor();
      if ( ( -0.045 < desiredForward ) && ( desiredForward < 0.025 ) ) {
         desiredForward = 0.0;
      }
      if ( ( -0.105 < desiredTurn ) && ( desiredTurn < 0.025 ) ) {
         desiredTurn = 0.0;
      }
      leftMotorOutput  = -desiredForward - desiredTurn;
      rightMotorOutput = +desiredForward - desiredTurn;
      leftMotorOutput  = std::min(  1.0, leftMotorOutput );
      rightMotorOutput = std::min(  1.0, rightMotorOutput );
      leftMotorOutput  = std::max( -1.0, leftMotorOutput );
      rightMotorOutput = std::max( -1.0, rightMotorOutput );
      m_motorLSMaster.Set( ControlMode::Velocity, 
                          leftMotorOutput  * 5200.0 * 4096 / 600 );
      m_motorRSMaster.Set( ControlMode::Velocity, 
                           rightMotorOutput * 5200.0 * 4096 / 600 );
      LSMotorState.targetVelocity_UnitsPer100ms = leftMotorOutput  *
	                                          5200.0 * 4096 / 600 ;
      RSMotorState.targetVelocity_UnitsPer100ms = rightMotorOutput *
	                                          5200.0 * 4096 / 600 ;
      //m_motorLSMaster.Set( ControlMode::PercentOutput, 
                           //leftMotorOutput  * 500.0 * 4096 / 600 / 1024);
      //m_motorRSMaster.Set( ControlMode::PercentOutput, 
                           //rightMotorOutput * 500.0 * 4096 / 600 / 1024);
   }      // Team4918Drive()


      /*---------------------------------------------------------------------*/
      /* DriveByJoystick()                                                   */
      /* Drive robot according to the commanded Y and X joystick position.   */
      /*---------------------------------------------------------------------*/
   void DriveByJoystick( void ) {
      double desiredForward;
      double desiredTurn;

         // m_drive.ArcadeDrive( m_stick.GetY(), -m_stick.GetX() );
              // our joystick increases Y when pulled BACKWARDS, and increases
              // X when pushed to the right.
      //if ( sCurrState.joyButton[2] ) {
      //   desiredForward = sCurrState.joyY*abs(sCurrState.joyY);
      //} else {
      //   desiredForward = -sCurrState.joyY*abs(sCurrState.joyY);
      //}
      //desiredTurn = sCurrState.joyX*abs(sCurrState.joyX);
      desiredForward = sCurrState.joyY;
      desiredTurn = sCurrState.joyX;

      if ( ( -0.045 < desiredForward ) && ( desiredForward < 0.025 ) ) {
         desiredForward = 0.0;
      } else {
         if ( sCurrState.joyButton[2] ) {     // if "reverse" button is pressed
            desiredForward = sCurrState.joyY*abs(sCurrState.joyY);
         } else {
            desiredForward = -sCurrState.joyY*abs(sCurrState.joyY);
         }
      }
      if ( ( -0.105 < desiredTurn ) && ( desiredTurn < 0.025 ) ) {
         desiredTurn = 0.0;
      } else {
         desiredTurn = sCurrState.joyX*abs(sCurrState.joyX);
      }
      Team4918Drive( desiredForward, desiredTurn );
   }      // DriveByJoystick()


      /*---------------------------------------------------------------------*/
      /* DriveToLimelightTarget()                                            */
      /* DriveToLimelightTarget() drives autonomously towards a limelight    */
      /* vision target.                                                      */
      /* It returns true if the limelight data is valid, false otherwise.    */
      /*---------------------------------------------------------------------*/
   bool DriveToLimelightTarget()  {

      bool returnVal = true;
      static int  iCallCount = 0;

      iCallCount++;

      limenttable->PutNumber( "ledMode", 3 );                   // turn LEDs on

      if ( 1 == limev )  {                       // if limelight data is valid
         double autoDriveSpeed;
             // limea is the area of the target seen by the limelight camera
             // and is in percent (between 0 and 100) of the whole screen area.
             // limey is the height above the center of the field of view
             // Could change the if/else statements below to calculate
             // autoDriveSpeed by using a math expression based on limey.
         if        ( 15 < limey ) {
            autoDriveSpeed = -0.1;
         } else if ( 12 < limey )  { // if we're really close...
            autoDriveSpeed = 0.0;    //   stop (or 0.08 to go slow)
         } else if (  8 < limey ) {  // if we're a little farther...
            autoDriveSpeed = 0.1;    //   go a little faster
         } else if (  2 < limey ) {  // if we're farther still...
            autoDriveSpeed = 0.15;   //   go a little faster still
         } else {                    // else we must be really far...
            autoDriveSpeed = 0.20;   //   go as fast as we dare
         }

                          // LATER: May want to modify autoDriveSpeed depending
                          // on the distance from the target determined
                          // by sonar transducers.

         // May have to add/subtract a constant from limex here, to account
         // for the offset of the camera away from the centerline of the robot.
         if ( aBooleanVariable ) {
            // m_drive.CurvatureDrive( -autoDriveSpeed, 0, 0 );
            Team4918Drive( -autoDriveSpeed, 0.0 );
         } else if ( 0 <= limex )  {
                             // if target to the right, turn towards the right
            // m_drive.CurvatureDrive( -autoDriveSpeed, -(limex/30.0), 1 );
            Team4918Drive( -autoDriveSpeed, -(limex/30.0) );
         } else if ( limex < 0 ) {
                               // if target to the left, turn towards the left
            // m_drive.CurvatureDrive( -autoDriveSpeed, -(limex/30.0), 1 );
            Team4918Drive( -autoDriveSpeed, -(limex/30.0) );
         } else {
            // m_drive.CurvatureDrive( -autoDriveSpeed, 0, 0 );
            Team4918Drive( -autoDriveSpeed, 0.0 );
         }

      } else {                    // else limelight data is not valid any more
         // should we continue forward here?
         // m_drive.CurvatureDrive( 0.0, 0, 0 );                // stop the robot
         Team4918Drive( 0.0, 0.0 );
         // DriveByJoystick();     // no powercell seen, drive according to joystick
         returnVal = false;
      }

      if ( 0 == iCallCount%100 )  {
         cout << "lime: " << limev << ":" << limex << "/" << limey;
         cout << ", " << limea << ":" << limes << "." << endl;
      }

      return returnVal;

   }  /* DriveToLimelightTarget() */


      /*---------------------------------------------------------------------*/
      /* DriveToPowercell()                                                  */
      /* DriveToPowercell() drives autonomously towards a vision target.     */
      /* It returns true if the usb vision data has detected a power cell,   */
      /* false otherwise.                                                    */
      /*---------------------------------------------------------------------*/
   bool DriveToPowercell()  {

      bool returnVal = true;
      static int  iCallCount = 0;

      iCallCount++;
      m_motorIntake.Set( ControlMode::PercentOutput, -0.4 );
      RunConveyor();

      if ( powercellOnVideo.SeenByCamera ) {      // if USB video data is valid
         double autoDriveSpeed;
	     // jag; 08jan2022: should change these comments to say we may have
	     // found a "red" or "blue" cargo rather than a yellow powercell.
             // If powercellOnVideo.SeenByCamera is true, that means that the
             // vision-processing code in VisionThread() has found a yellow
             // circle in the latest video frame from the USB videocamera (and
             // we hope that yellow circle is a powercell).
             // powercellOnVideo.Y is the Y-position in the video frame of the
             //    powercell; the range is from -120 to +120 (pixels).
             // powercellOnVideo.X is the X-position in the video frame of the
             //    powercell; the range is from -160 to +160 (pixels).
             // powercellOnVideo.Radius is the radius of the powercell;
             //    the range is from 20 to 70 (pixels).
             // In the code below, we use those powercellOnVideo values to
             // determine how fast and in which direction to drive, to go
             // towards the powercell.
             // We could change the if/else statements below to calculate
             // autoDriveSpeed by using a math expression based on
             // powercellOnVideo.Y values.
	     // jag; 22mar2021: all these values have been changed; it may be
	     // useful to compare with the original working code in
	     // ~/Desktop/2020-Robot/Robot.cpp
//         if        ( powercellOnVideo.Y < -50 ) {  // if we're super close
//            autoDriveSpeed = -0.35;   //   go backward slowly
//         } else if ( powercellOnVideo.Y < -30 ) {  // if we're super close
//            autoDriveSpeed = -0.25;   //   go backward slowly
//            autoDriveSpeed = -0.35 * float( - 30 - powercellOnVideo.Y ) / 20.0;
//         } else if ( powercellOnVideo.Y < 0 )   { // if we're really close...
//            autoDriveSpeed = 0.0;     //   stop (or 0.08 to go slow)
         /* } else */ if ( powercellOnVideo.Y <  20 ) {  // if we're a little farther
            autoDriveSpeed = 0.15;    //   go a little faster
            autoDriveSpeed = 0.20 * float( powercellOnVideo.Y ) / 20.0;
         } else if (  powercellOnVideo.Y < 40 ) {  // if we're farther still...
            autoDriveSpeed = 0.20;    //   go a little faster still
            autoDriveSpeed = 0.20 + 0.20 * float( powercellOnVideo.Y - 20 ) / 40.0;
         } else {                     // else we must be really far...
            autoDriveSpeed = 0.30;    //   go as fast as we dare
         }

                          // LATER: May want to modify autoDriveSpeed depending
                          // on the distance from the target determined
                          // by sonar transducers.

         // May have to add/subtract a constant from x-values here, to account
         // for the offset of the camera away from the centerline of the robot.
         if        ( 0 <= powercellOnVideo.X ) {
                             // if target to the right, turn towards the right
            //m_drive.CurvatureDrive( -autoDriveSpeed,
            //                        -sqrt((powercellOnVideo.X/300.0)), 1 );
            Team4918Drive( autoDriveSpeed, sqrt(powercellOnVideo.X/300.0) );
         } else if ( powercellOnVideo.X < 0 ) {
                               // if target to the left, turn towards the left
            //m_drive.CurvatureDrive( -autoDriveSpeed,
            //                        sqrt((-powercellOnVideo.X/300.0)), 1 );
            Team4918Drive( autoDriveSpeed, -sqrt(-powercellOnVideo.X/300.0) );           
         } else {
            //m_drive.CurvatureDrive( -autoDriveSpeed, 0, 0 );

            Team4918Drive( autoDriveSpeed, 0.0 );     // drive straight forward
         }

      } else {               // else USB videocamera data is not valid any more
         // should we continue forward here?
         // m_drive.CurvatureDrive( 0.0, 0, 0 );              // stop the robot
         //Team4918Drive( 0.0, 0.0 );
         DriveByJoystick();   // no powercell seen, drive according to joystick
         returnVal = false;
      }

      if ( 0 == iCallCount%100 )  {
         cout << "Powercell Seen flag " << powercellOnVideo.SeenByCamera <<
                 ": " << powercellOnVideo.X << "/" << powercellOnVideo.Y;
         cout << ", " << powercellOnVideo.Radius  << "." << endl;
      }

      return returnVal;

   }  /* DriveToPowercell() */


      /*---------------------------------------------------------------------*/
      /* RunDriveMotors()                                                    */
      /* RunDriveMotors() drives the robot.  It uses joystick and console    */
      /* inputs to determine what the robot should do, and then runs the     */
      /* m_motorLSMaster and m_motorRSMaster motors to make the robot do it. */
      /*---------------------------------------------------------------------*/
   bool RunDriveMotors( void ) {
      static int iCallCount = 0;
      iCallCount++;

      //m_shiftingSolenoid.Set(true);     // high gear?
      //m_shiftingSolenoid.Set(false);    // low gear?

                  /* If joystick button 5 pressed, use the joystick position */
                  /* to adjust some variables to specific speeds, so we can  */
                  /* set the drive motors to those speeds in later code.     */
      if ( ( 0 == iCallCount%100 )  &&
           sCurrState.joyButton[5]     ) {

         if ( 0.5 < m_stick.GetY() &&
              LSMotorState.targetVelocity_UnitsPer100ms < 790.0 * 4096 / 600 ) {
            LSMotorState.targetVelocity_UnitsPer100ms += 200.0 * 4096 / 600;
         } else if ( m_stick.GetY() < -0.5 &&
                     -790.0 * 4096 / 600 <
                                  LSMotorState.targetVelocity_UnitsPer100ms ) {
            LSMotorState.targetVelocity_UnitsPer100ms -= 200.0 * 4096 / 600;
         }

         if ( 0.5 < m_stick.GetX() &&
              RSMotorState.targetVelocity_UnitsPer100ms < 790.0 * 4096 / 600 ) {
            RSMotorState.targetVelocity_UnitsPer100ms += 200.0 * 4096 / 600;
         } else if ( m_stick.GetX() < -0.5 && 
                     -790.0 * 4096 / 600 <
                                  RSMotorState.targetVelocity_UnitsPer100ms ) {
            RSMotorState.targetVelocity_UnitsPer100ms -= 200.0 * 4096 / 600;
         }
      } 

      motorFindMinMaxVelocity( m_motorLSMaster, LSMotorState );
      motorFindMinMaxVelocity( m_motorRSMaster, RSMotorState );

    if ( sCurrState.teleop ) {
      if (sCurrState.joyButton[1]) { //when button 1 is pressed, shift into high gear until released
         sCurrState.highGear = true;
         m_shiftingSolenoid.Set( true );
      } else if (!sCurrState.joyButton[1]) {
         sCurrState.highGear = false;
         m_shiftingSolenoid.Set( false );
      }
    } else {
      if ( ( 15000 < abs(LSMotorState.sensorVmin) ) ||
           ( 15000 < abs(RSMotorState.sensorVmin) )    ) {
         if (!sCurrState.highGear){
            cout << "shifting to high" << endl; 
         }
         sCurrState.highGear = true; // could move inside if statement
         m_shiftingSolenoid.Set( true );  // high gear got 5700/5200
      } else if ( ( abs(LSMotorState.sensorVmin) < 14000 ) &&
                  ( abs(RSMotorState.sensorVmin) < 14000 )  ) {
         if (sCurrState.highGear){
            cout << "shifting to low" << endl;
         }
         sCurrState.highGear = false; // could move inside if statement
         m_shiftingSolenoid.Set( false );    // low gear got 2800/2700
      } 
    } 

      if ( 0 == iCallCount%100 )  {   // every 2 seconds
         // JoystickDisplay();

//         MotorDisplay( "LS:", m_motorLSMaster, LSMotorState );
//         MotorDisplay( "RS:", m_motorRSMaster, RSMotorState );
         //IMUOrientationDisplay();

         // max free speed for MinCims is about 6200
         //cout << "Accel: x/y/z: " << RoborioAccel.GetX() << "/";
         //cout << RoborioAccel.GetY() << "/";
         //cout << RoborioAccel.GetZ() << endl;
      }

                                       /* Button 1 is the trigger button on */
                                       /* the front of the joystick.        */
                                       /* Button 2 is the bottom button on  */
                                       /* the rear of the joystick.         */
      if ( ( sCurrState.joyButton[2] ) &&      // If driver is pressing the
           ( sCurrState.joyButton[1] ) &&      // "ReverseDrive" and the
           ( 1  == limev )                ) {  // "DriveToLimelightTarget"
                                 // buttons, and the limelight has a target,
                                 // then autonomously drive towards the target
         DriveToLimelightTarget();

                                           // If driver is pressing button four
      } else if ( ( sCurrState.joyButton[4] ) &&
                  ( powercellOnVideo.SeenByCamera ) ) { 
                              // trigger ("DriveToPowercell"), and
                              // the USB videocamera has seen a powercell,
         DriveToPowercell();  // then autonomously drive towards the powercell

                                        /* Button 3 is the topmost center   */
                                        /* button on the back of joystick.  */
      } else if ( sCurrState.joyButton[3] ) {

                     /* If button 3 pressed, drive the motors separately    */
                     /* (rather than with ArcadeDrive) with Percent Output, */
                     /* using the variables that have been set with the     */
                     /* joystick while button 3 is pressed.                 */
         if ( !sPrevState.joyButton[3] ) {  // if button has just been pressed
            // m_drive.StopMotor();
                                // Set current sensor positions to zero
            m_motorLSMaster.SetSelectedSensorPosition( 0, 0, 10 );
            m_motorRSMaster.SetSelectedSensorPosition( 0, 0, 10 );
         }
         // m_motorLSMaster.Set( ControlMode::PercentOutput,
         //                      -m_stick.GetY() );
         // m_motorRSMaster.Set( ControlMode::PercentOutput,
         //                      -m_stick.GetX() );
                        /* Use MotionMagic to set the position of the drive */
                        /* wheels, rather than setting their velocity.      */
         // m_motorLSMaster.Set( ControlMode::MotionMagic, sCurrState.joyY*4096 );
         // m_motorRSMaster.Set( ControlMode::MotionMagic, sCurrState.joyX*4096 );

                                      /* Button 5 is the Right-most button  */
                                      /* on the back of the joystick.       */
      } else if ( sCurrState.joyButton[5] ) {
         static int iButtonPressCallCount = 0;
         if ( !sPrevState.joyButton[5] ) {  // if button has just been pressed
            // m_drive.StopMotor();
            iButtonPressCallCount = 0;
         }
         // m_drive.StopMotor();    // this is needed to eliminate motor warnings

         if ( iButtonPressCallCount < 50 ) {       // turn motors on gently...
            // m_motorLSMaster.Set( ControlMode::Velocity,
            //       m_motorLSMaster.GetSelectedSensorVelocity() +
            //          0.2 * ( LSMotorState.targetVelocity_UnitsPer100ms -
            //                  m_motorLSMaster.GetSelectedSensorVelocity() ) );
            // m_motorRSMaster.Set( ControlMode::Velocity,
            //       m_motorRSMaster.GetSelectedSensorVelocity() +
            //          0.2 * ( RSMotorState.targetVelocity_UnitsPer100ms -
            //                  m_motorRSMaster.GetSelectedSensorVelocity() ) );
         } else {
            // m_motorLSMaster.Set( ControlMode::Velocity, 
            //                        LSMotorState.targetVelocity_UnitsPer100ms);
            // m_motorRSMaster.Set( ControlMode::Velocity, 
            //                        RSMotorState.targetVelocity_UnitsPer100ms);
         }
         iButtonPressCallCount++;
      } else {
                                    /* Drive the robot according to the     */
                                    /* commanded Y and X joystick position. */
         DriveByJoystick();
      }
      return true;
   }      // RunDriveMotors()


      /*---------------------------------------------------------------------*/
      /* TurnToHeading()                                                     */
      /* This function turns the robot to a specified heading.               */
      /* heading is a parameter, in the same degree units as the Pigeon IMU  */
      /* produces; it is positive for left turns (the same as trigonometry,  */
      /* but the opposite of ordinary 0-360 degree compass directions).      */
      /* bInit is a boolean that tells the function to initialize (to record */
      /* the current yaw as the starting yaw).                               */
      /* This function returns false if the yaw value has not been reached   */
      /* yet, and returns true when the yaw has been reached.                */
      /*---------------------------------------------------------------------*/
   bool OldTurnToHeading ( double heading ) {
      static bool bReturnValue = true;
      static double startYaw = 0;

      if ( bReturnValue ) {
         startYaw = sCurrState.yawPitchRoll[0];
	 cout << "TurnToHeading(): startYaw = " << startYaw << endl;
      }

      if ( sCurrState.yawPitchRoll[0] < heading ) { // do we need to turn left?
                                                // If we have a long way to go,  
         if ( sCurrState.yawPitchRoll[0] < heading-50.0 ) {   // turn left fast
            if ( sCurrState.yawPitchRoll[0]-50 <startYaw){
               LSMotorState.targetVelocity_UnitsPer100ms = 0.0 * 4096 / 600;
               //LSMotorState.targetVelocity_UnitsPer100ms = 0.0;
               RSMotorState.targetVelocity_UnitsPer100ms = 300.0 * 4096 / 600;
            } else {
               LSMotorState.targetVelocity_UnitsPer100ms = 300.0 * 4096 / 600;
               RSMotorState.targetVelocity_UnitsPer100ms = 300.0 * 4096 / 600;
            }
            bReturnValue = false;           // else if a medium way to go, turn
         } else if ( sCurrState.yawPitchRoll[0] < heading-5.0 ) {  // left slow
            LSMotorState.targetVelocity_UnitsPer100ms = 300.0 * 4096 / 600 *
		                   (heading - sCurrState.yawPitchRoll[0])/50.0;
            //LSMotorState.targetVelocity_UnitsPer100ms = 0.0;
            RSMotorState.targetVelocity_UnitsPer100ms = 300.0 * 4096 / 600 *
		                   (heading - sCurrState.yawPitchRoll[0])/50.0;
            bReturnValue = false;
         } else {                              // else we're done; stop turning
            LSMotorState.targetVelocity_UnitsPer100ms = 0.0 * 4096 / 600;
            RSMotorState.targetVelocity_UnitsPer100ms = 0.0 * 4096 / 600;  
            bReturnValue = true; 
         }
      } else {                                    // else we need to turn right
                                                // If we have a long way to go,
         if ( heading+50.0 < sCurrState.yawPitchRoll[0] ) {  // turn right fast
            LSMotorState.targetVelocity_UnitsPer100ms = -300.0 * 4096 / 600;
            //RSMotorState.targetVelocity_UnitsPer100ms = -500.0 * 4096 / 600;
            RSMotorState.targetVelocity_UnitsPer100ms = 0.0;
            bReturnValue = false;
                                            // else if a medium way to go, turn
         }else if ( heading+5.0 < sCurrState.yawPitchRoll[0] ) {  // right slow
            LSMotorState.targetVelocity_UnitsPer100ms = -300.0 * 4096 / 600 *
		                     (sCurrState.yawPitchRoll[0]-heading)/50.0;
            RSMotorState.targetVelocity_UnitsPer100ms = -100.0 * 4096 / 600 *
		                     (sCurrState.yawPitchRoll[0]-heading)/50.0;
            //RSMotorState.targetVelocity_UnitsPer100ms = 0.0;
            bReturnValue = false;
         } else {                              // else we're done; stop turning
            LSMotorState.targetVelocity_UnitsPer100ms = 0.0 * 4096 / 600;
            RSMotorState.targetVelocity_UnitsPer100ms = 0.0 * 4096 / 600;
            bReturnValue = true;
         }
      }
      if ( bReturnValue ) {
         cout << "TurnToHeading() returning TRUE!!!!!!!!!" << endl;
      }
      return bReturnValue;
   }      // OldTurnToHeading()


   bool TurnToHeading ( double desiredYaw, bool bInit ) {
      static bool   bReturnValue = true;
      static double startYaw = 0;

      double      currentYaw = sCurrState.yawPitchRoll[0];
      double      prevYaw    = sPrevState.yawPitchRoll[0];

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
        // calculate desired turn speed, with 100% if off by 50 degrees or more
      dDesiredTurn = ( currentYaw-desiredYaw ) * 1.0/50.0;

                       // If we're turning at less than 50 degrees per second,
      if ( std::abs( currentYaw - prevYaw ) < 1.0 ) {
           // Increase the forward speed, to reduce friction and ease the turn.
         dDesiredSpeed = std::min( 1.0,  dDesiredSpeed * 1.1 );
      } else {                // Else we're turning at a good speed, so
                              // reduce the forward speed, to tighten the turn.
         dDesiredSpeed = std::max( 0.01, dDesiredSpeed * 0.9 );
      }

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
      dDesiredSpeed = std::min(  1.0, dDesiredSpeed );
      dDesiredTurn  = std::min(  1.0, dDesiredTurn  );
      dDesiredSpeed = std::max( -1.0, dDesiredSpeed );
      dDesiredTurn  = std::max( -1.0, dDesiredTurn  );

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
      /*    IMU produces; it is positive for left turns (the same as         */
      /*    trigonometry, but the opposite of ordinary 0-360 degree compass  */
      /*    directions).                                                     */
      /* desiredDistance is a parameter, in feet; positive is in the         */
      /*    "forward" direction (toward the powercell-sucking side of        */
      /*    the robot).                                                      */
      /* bDivertToPowercell is a boolean that tells the function to ignore   */
      /*    the specified heading and drive toward a powercell instead,      */
      /*    if one is seen by the vision-processing thread.                  */
      /* bInit is a boolean that tells the function to initialize (to record */
      /*    the current position as the starting position).                  */
      /* This function returns false if the distance has not been reached    */
      /* yet, and returns true when the distance has been reached.           */
      /*---------------------------------------------------------------------*/
   bool DriveToDistance( double desiredYaw,
                         double desiredDistance,
                         bool   bDivertToPowercell,
                         bool   bInit            ) {
      static bool bReturnValue = true;
 //     static int  iLSStartPosition = 0;
 //     static int  iRSStartPosition = 0;
      static int  iTotalTicksHighGear = 0;
      static int  iTotalTicksLowGear  = 0;
      static int  iLSPrevPosition = 0;
      static int  iRSPrevPosition = 0;
      int         iDistanceDriven;    // distance driven in encoder ticks
      double      dDistanceDriven;    // distance driven in feet (floating pt.)
      double      dDesiredSpeed;  // -1.0 to +1.0, positive is forward
      double      dDesiredTurn;   // -1.0 to +1.0, positive is to the right
      static int  iCallCount = 0;
      iCallCount++;

          // If we've been told to initialize, or the last call returned
          // true (indicating that we finished the previous drive to distance).
      if ( bInit || bReturnValue ) {
//         iLSStartPosition = sCurrState.iLSMasterPosition;
//         iRSStartPosition = sCurrState.iRSMasterPosition;
         iLSPrevPosition = sCurrState.iLSMasterPosition;
         iRSPrevPosition = sCurrState.iRSMasterPosition;
	 iTotalTicksHighGear = 0;
	 iTotalTicksLowGear  = 0;
      }
      iDistanceDriven =
                   ( - ( sCurrState.iLSMasterPosition - iLSPrevPosition ) +
                     ( sCurrState.iRSMasterPosition - iRSPrevPosition ) ) / 2;
      if ( sCurrState.highGear ) {
         iTotalTicksHighGear += iDistanceDriven;
      } else {
         iTotalTicksLowGear += iDistanceDriven;
      }

            // Convert encoder ticks to feet, using the diameter of the wheels,
            // the number of ticks/revolution, and the number of inches/foot.
            // Also have to adjust for gear ratio, since the encoders are on 
            // the motors, before the gearbox, and we need the rotation of the
	    // wheels, after the gearbox.
	    // Apply the ticks separately, depending on whether they occurred
	    // when in low or high gear.

// NOTE: The gear ratio values below are guesses; the actual values should be
// one of these pairs, based on which gearbox the team built into the robot
// (which shifter-spread, and which 3rd-stage gear pair).
// See https://www.vexrobotics.com/3cimballshifter.html#fh28wu4 :
//
//            2.16x Shifter Spread  2.65x Shifter Spread  3.68x Shifter Spread
//            High Gear  Low Gear   High Gear  Low Gear   High Gear  Low Gear 
// 3rd Stage  Overall    Overall    Overall    Overall    Overall    Overall
// Gear Pair  Reduction  Reduction  Reduction  Reduction  Reduction  Reduction
// 64 to 20    9.07 : 1  19.61 : 1   9.07 : 1  24.00 : 1   9.07 : 1  33.33 : 1
// 60 to 24    7.08 : 1  15.32 : 1   7.08 : 1  18.75 : 1   7.08 : 1  26.04 : 1
// 54 to 30    5.10 : 1  11.03 : 1   5.10 : 1  13.50 : 1   5.10 : 1  18.75 : 1
// 50 to 34    4.17 : 1   9.01 : 1   4.17 : 1  11.03 : 1   4.17 : 1  15.32 : 1
// NO 3rd      2.83 : 1   6.13 : 1   2.83 : 1   7.50 : 1   2.83 : 1  10.42 : 1
//   Stage
//
// Measurements with the actual robot indicate that 13.50 is the best ratio,
// and that seems to be true whether we are in high gear or low gear --
// so the encoders in the gearbox must be after the high/low change.

      dDistanceDriven = 3.1415 * 8.0 / 4096.0 / 12.0 *
                        ( (double)iTotalTicksHighGear / 13.50 +
                          (double)iTotalTicksLowGear  / 13.50   );

                                         // if we haven't driven far enough yet
      if ( std::abs( dDistanceDriven ) < std::abs( desiredDistance ) ) {
         if ( 0.0 < desiredDistance ) {             // If we're driving forward
                                      // and still have more than 10 feet to go
            if ( dDistanceDriven < desiredDistance -  5.0 ) {
               dDesiredSpeed = 1.0;                            // go full speed
            } else {
                   // Otherwise speed is proportional to distance still needed.
               dDesiredSpeed = 0.5 +
		                  ( desiredDistance - dDistanceDriven ) / 10.0;
            }
         } else {                               // else we're driving backwards
                                      // and still have more than 10 feet to go
            if ( desiredDistance +  5.0 < dDistanceDriven ) {
               dDesiredSpeed = -1.0;               // go full speed (backwards)
            } else {
                   // Otherwise speed is proportional to distance still needed.
               dDesiredSpeed = -0.5 +
                                  ( desiredDistance - dDistanceDriven ) / 10.0;
            }
         }
                     // if more than 50 degrees to the right of desired course
         if ( sCurrState.yawPitchRoll[0] < desiredYaw-50.0 ) {
            dDesiredTurn = -0.2;                         // turn left strongly
                 // else if more than 50 degrees to the left of desired course
         } else if ( desiredYaw+50.0 < sCurrState.yawPitchRoll[0] ) {
            dDesiredTurn = 0.2;                         // turn right strongly
         } else {
                     // else calculate turn amount based on how far off we are
            dDesiredTurn = (sCurrState.yawPitchRoll[0]-desiredYaw) * 0.2/50.0;
            if ( ( 0.02 < dDesiredTurn ) && ( dDesiredTurn ) < 0.1 ) {
               dDesiredTurn = 0.1;
            } else if ( ( -0.1 < dDesiredTurn ) && ( dDesiredTurn < -0.02 ) ) {
               dDesiredTurn = -0.1;
            }
         }
         
	 if ( !bDivertToPowercell ) { // if we are ignoring powercells (we've
		                      // been told NOT to divert to powercells)
            // then just drive (don't change dDesiredxxx values)

                    // else (we've been told to divert to powercells when seen)
                    // if a powercell is seen
	 } else if ( powercellOnVideo.SeenByCamera ) {
                 // Then drive toward the powercell, by setting dDesiredSpeed
                 // and dDesiredTurn, based on the X,Y coordinates of the
		 // powercell in the camera view.
                 // powercellOnVideo.Y is the Y-position in the video frame of
                 // the powercell; the range is from -120 to +120 (pixels).
                 // powercellOnVideo.X is the X-position in the video frame of
                 // the powercell; the range is from -160 to +160 (pixels).
                 // (0,0) is right in the center of the field of view, with
                 // X increasing to the right, and Y increasing down towards
                 // the robot.
		 // We could use a little math to make the turns a little 
		 // sharper when the X offset is small, or when the powercell
                 // is very close to the robot (Y is large).

                                    // The speed should be between 0.1 and 0.3
            dDesiredSpeed = std::min( (120.0 - powercellOnVideo.Y) / 100.0,
			              0.3 );
	    if ( 0.5 < dDesiredSpeed ) {
               dDesiredSpeed = 0.5;
            }
                                    // The turn should be between -0.5 and 0.5
            if        ( 5 <= powercellOnVideo.X ) {
               // dDesiredTurn = sqrt(powercellOnVideo.X/300.0) );
               dDesiredTurn = std::min( (powercellOnVideo.X/600.0), 1.0 );
            } else if ( powercellOnVideo.X < -5 ) {
               // dDesiredTurn = -sqrt(-powercellOnVideo.X/300.0) );
               dDesiredTurn = std::max( (powercellOnVideo.X/600.0), -1.0 );
            } else {
               dDesiredTurn = 0.0;
            }

	 } else {         // else (we'd divert to a powercell if there was one,
                          // but no powercell is currently in view)
            // then just drive (don't change dDesiredxxx values)
	    dDesiredSpeed = dDesiredSpeed/4;
	    dDesiredTurn = dDesiredTurn/4;
	 }

         Team4918Drive( dDesiredSpeed, dDesiredTurn );    // then just drive
	 if ( bDivertToPowercell && sCurrState.powercellInIntake ) {
            bReturnValue = true;   // tell caller we are done
	 } else{
            bReturnValue = false;   // tell caller we are still driving
	 }
         if ( 0 == iCallCount%100 ) {                        // Every 2 seconds
            cout << "D2D(): curYaw/desYaw desTurn: " <<
                    sCurrState.yawPitchRoll[0] << 
                    "/" << desiredYaw << " " <<
                    " " << dDesiredTurn << endl;
            cout << "dDistanceDriven: " << dDistanceDriven <<
                    sCurrState.iLSMasterPosition <<
                    sCurrState.iRSMasterPosition << endl;
         }
      } else {
         Team4918Drive( 0.0, 0.0 );   // stop the robot
         m_motorLSMaster.SetIntegralAccumulator( 0.0 );
         m_motorRSMaster.SetIntegralAccumulator( 0.0 );
         bReturnValue = true;    // tell caller we've reached desired distance
      }

      if ( bReturnValue ) {
         cout << "DriveToDistance() returning TRUE!!!!!!!!" << endl;
         cout << "Final Distance: " <<  dDistanceDriven << endl;
         cout << " Final Yaw: " <<  sCurrState.yawPitchRoll[0] << endl;
      }

      iLSPrevPosition = sCurrState.iLSMasterPosition;
      iRSPrevPosition = sCurrState.iRSMasterPosition;

      return bReturnValue;
   }  // DriveToDistance()

      //create FollowWall function

   void FollowWall( double distLeft, double distRight,
		    double straight, double turnTo, double turnAway ) {
      double      wallDistance;
      double      currentYaw = sCurrState.yawPitchRoll[0];
      // double      prevYaw    = sPrevState.yawPitchRoll[0];
      
      if ( 6.0 < distLeft ){ //left side wall
         wallDistance = distSensor0.GetVoltage() * 100 / 2.54;
         if (wallDistance < distLeft - 1.0){ //too close to wall
            DriveToDistance (turnAway, 1.0, false, true);//turn away
            if ( 0 == iCallCount%1 ){
               printf( "right %f\n", currentYaw);
            }
         }else if (distLeft + 1.0 < wallDistance){ //too far from wall
            DriveToDistance (turnTo, 1.0, false, true);//turn to wall
            if ( 0 == iCallCount%25 ){
               printf("left %f\n", currentYaw);
            }
         }else  {//right distance from wall
                                                     // maintain speed/heading
            DriveToDistance (straight, 1.0, false, true);
            if ( 0 == iCallCount%25 ){
               printf("straight %f\n", currentYaw);
            }
          }
      }else{ //Right side wall
         wallDistance = distSensor1.GetVoltage() * 100 / 2.54;
         if (wallDistance < distRight + 1.0){ //too close to wall
            DriveToDistance (turnAway, 1.0, false, true);//turn away
         }else if (distRight - 1.0 < wallDistance){ //too far from wall
            DriveToDistance (turnTo, 1.0, false, true);//turn to wall
         }else { //right distance from wall
            DriveToDistance (straight, 1.0, false, true);
            //maintain speed/heading
          }  
       }  
   }  // FollowWall()

         /*------------------------------------------------------------------*/
         /* RunShooter()                                                     */
         /* RunShooter() drives the 2 shooter motors.                        */
         /*------------------------------------------------------------------*/
   bool RunShooter(void) {
      if (   ( 0.5 < sCurrState.conY ) &&           // if console "joystick" is
            !( 0.5 < sPrevState.conY ) ) {          // newly-pressed downward
         TSMotorState.targetVelocity_UnitsPer100ms =  2100 * 4096 / 600;
         BSMotorState.targetVelocity_UnitsPer100ms = -2900 * 4096 / 600;
         m_motorTopShooter.Set( ControlMode::Velocity, 
                                TSMotorState.targetVelocity_UnitsPer100ms );
         m_motorBotShooter.Set( ControlMode::Velocity, 
                                BSMotorState.targetVelocity_UnitsPer100ms );
      } else if ( !( 0.5 < sCurrState.conY ) &&
                   ( 0.5 < sPrevState.conY ) ) {     // newly-released downward
         TSMotorState.targetVelocity_UnitsPer100ms = 0 * 4096 / 600;
         BSMotorState.targetVelocity_UnitsPer100ms = 0 * 4096 / 600;
         m_motorTopShooter.Set( ControlMode::Velocity,
               0.95 * (double)m_motorTopShooter.GetSelectedSensorVelocity() );
         m_motorBotShooter.Set(ControlMode::Velocity,
               0.95 * (double)m_motorBotShooter.GetSelectedSensorVelocity() );
         
      } else if (  ( sCurrState.conY < -0.5 ) &&  // else if console "joystick"
                  !( sPrevState.conY < -0.5 ) ) { // is newly-pressed upward
         //TSMotorState.targetVelocity_UnitsPer100ms =  2200 * 4096 / 600;
         //BSMotorState.targetVelocity_UnitsPer100ms = -3000 * 4096 / 600;
         TSMotorState.targetVelocity_UnitsPer100ms =  2000 * 4096 / 600;
         BSMotorState.targetVelocity_UnitsPer100ms = -3000 * 4096 / 600;
         m_motorTopShooter.Set( ControlMode::Velocity, 
                                TSMotorState.targetVelocity_UnitsPer100ms );
         m_motorBotShooter.Set( ControlMode::Velocity, 
                                BSMotorState.targetVelocity_UnitsPer100ms );
      } else if ( !( sCurrState.conY < -0.5 ) &&
                   ( sPrevState.conY < -0.5 ) ) {     // newly-released upward
         TSMotorState.targetVelocity_UnitsPer100ms = 0 * 4096 / 600;
         BSMotorState.targetVelocity_UnitsPer100ms = 0 * 4096 / 600;
         m_motorTopShooter.Set( ControlMode::Velocity,
               0.95 * (double)m_motorTopShooter.GetSelectedSensorVelocity() );
         m_motorBotShooter.Set(ControlMode::Velocity,
               0.95 * (double)m_motorBotShooter.GetSelectedSensorVelocity() );
      }       /*** Following code for testing only - slowly spit out balls
               when console "joystick" is pushed in the positive direction ***/
      else if ( ( sCurrState.conX > 0.5 ) &&  //newly pressed rightward
               !( sPrevState.conX > 0.5 ) ) {
         TSMotorState.targetVelocity_UnitsPer100ms =   100 * 4096 / 600;
         BSMotorState.targetVelocity_UnitsPer100ms = -4300 * 4096 / 600;
         m_motorTopShooter.Set( ControlMode::Velocity, 
                                TSMotorState.targetVelocity_UnitsPer100ms );
         m_motorBotShooter.Set( ControlMode::Velocity, 
                                BSMotorState.targetVelocity_UnitsPer100ms );
      } else if ( !( sCurrState.conX < 0.5 ) && //newly released rightward
                   ( sPrevState.conX < 0.5 ) ) {
         TSMotorState.targetVelocity_UnitsPer100ms = 0 * 4096 / 600;
         BSMotorState.targetVelocity_UnitsPer100ms = 0 * 4096 / 600;
         m_motorTopShooter.Set( ControlMode::Velocity,
               0.95 * (double)m_motorTopShooter.GetSelectedSensorVelocity() );
         m_motorBotShooter.Set(ControlMode::Velocity,
               0.95 * (double)m_motorBotShooter.GetSelectedSensorVelocity() );
      } else if (sCurrState.conX > 0.5) {//keep running if pushed right
         m_motorTopShooter.Set( ControlMode::Velocity, 
                                TSMotorState.targetVelocity_UnitsPer100ms );
         m_motorBotShooter.Set( ControlMode::Velocity, 
                                BSMotorState.targetVelocity_UnitsPer100ms );
      /***End of testing code***/
      } else if ( ( -0.5 < sCurrState.conY       ) && 
                  (        sCurrState.conY < 0.5 ) ) {
         m_motorTopShooter.Set( ControlMode::Velocity,
               0.95 * (double)m_motorTopShooter.GetSelectedSensorVelocity() );
         m_motorBotShooter.Set(ControlMode::Velocity,
               0.95 * (double)m_motorBotShooter.GetSelectedSensorVelocity() );
      } 
      if ( 0 == iCallCount%100 )  {   // every 2 seconds
         if ( ( 100.0 < abs(m_motorTopShooter.GetSelectedSensorVelocity()) ) ||
              ( 100.0 < abs(m_motorBotShooter.GetSelectedSensorVelocity()) ) ) {
            MotorDisplay( "TS:", m_motorTopShooter, TSMotorState );
            MotorDisplay( "BS:", m_motorBotShooter, BSMotorState );
         }
      }
         
      return true;
   }     // RunShooter()


         /*------------------------------------------------------------------*/
         /* Shoot()                                                          */
         /* Shoot() waits until the shooter rollers are moving at a desired  */
         /* speed, then moves the conveyor to shoot powercells.              */
         /*------------------------------------------------------------------*/
   void Shoot( void ) {
      if ( 0.5 < sCurrState.conY ) {
   	 // if ( ( 1800 * 4096 / 600 <
   	 if ( ( 1800 * 4096 / 600 <
                   abs( m_motorTopShooter.GetSelectedSensorVelocity() ) ) &&
         //   ( 2600 * 4096 / 600 <
              ( 2600 * 4096 / 600 <
                   abs( m_motorBotShooter.GetSelectedSensorVelocity() ) )   ) {
	                                 // run the conveyor to shoot the balls
	    sCurrState.iConveyPercent = -80;
            m_motorConveyMaster.Set( ControlMode::PercentOutput, -0.8 );
         }
      } else if (sCurrState.conY < -0.5) {
         // if ( ( 1900 * 4096 / 600 <
         if ( ( 1800 * 4096 / 600 <
                   abs( m_motorTopShooter.GetSelectedSensorVelocity() ) ) &&
         //   ( 2700 * 4096 / 600 <
              ( 2800 * 4096 / 600 <
                   abs( m_motorBotShooter.GetSelectedSensorVelocity() ) )   ) {
	                                 // run the conveyor to shoot the balls
	    sCurrState.iConveyPercent = -80;
            m_motorConveyMaster.Set( ControlMode::PercentOutput, -0.8 );
         }
      } else if ( 0.5 < sCurrState.conX ) {
         if ( ( 0 * 4096 / 600 <
                   abs( m_motorTopShooter.GetSelectedSensorVelocity() ) ) &&
              ( 4000 * 4096 / 600 <
                   abs( m_motorBotShooter.GetSelectedSensorVelocity() ) )   ) {
                                         // run the conveyor to shoot the balls
            sCurrState.iConveyPercent = -80;
            m_motorConveyMaster.Set( ControlMode::PercentOutput, -0.8 );
         }
      }
   }


      /*---------------------------------------------------------------------*/
      /* RunConveyor()                                                       */
      /* Run the conveyor belt motors, to move balls into and through the    */
      /* conveyor system.                                                    */
      /*---------------------------------------------------------------------*/
   void RunConveyor( void ) {
      //print out if there is a powercell in the intake or not. 
      if ( sPrevState.powercellInIntake != sCurrState.powercellInIntake ) {
         if ( sCurrState.powercellInIntake ) {
            cout << "powercell in the intake." << endl;
         } else {
            cout << "powercell NOT in the intake." << endl; 
         } 
      }
      // print out if there is a ball in position 5. 
      if ( sPrevState.powercellInPosition5 !=
                                          sCurrState.powercellInPosition5 ) {
         if ( sCurrState.powercellInPosition5 ) {// if this sensor is blocked
            cout << "powercell in position 5" << endl;
         } else {
            cout << "powercell NOT in position 5" << endl; 
         } 
      }
      if ( sCurrState.conButton[11] ) {             // Is manual mode selected?
         if ( sCurrState.conButton[2] )   {            // Run conveyor forward.
	    sCurrState.iConveyPercent = -80;
            m_motorConveyMaster.Set( ControlMode::PercentOutput, -0.8 );
         } else if ( sCurrState.conButton[4] ) {     // Run conveyor backwards.
            sCurrState.iIntakePercent = 40;       // Run intake backwards, too.
	    sCurrState.iConveyPercent =  80;
            m_motorConveyMaster.Set( ControlMode::PercentOutput,  0.8 );
         } else {                                         // Stop the conveyor.
	   sCurrState.iConveyPercent = 0;
           if (sPrevState.conButton[2]||sPrevState.conButton[4]) {
              m_motorConveyMaster.Set( ControlMode::PercentOutput, 0.0);
	   }
         } 
      } else {  
         if ( !(sCurrState.conX > 0.5) && !(sCurrState.conY > 0.5) && 
              !(sCurrState.conY < -0.5) ) {
            static int ConveyorCounter = 0;
            if ( sCurrState.powercellInIntake &&
              !sCurrState.powercellInPosition5 ) {
	      sCurrState.iConveyPercent = -30;
              m_motorConveyMaster.Set( ControlMode::PercentOutput, -0.3 );
	      ConveyorCounter = 6;
            } else if ( ( 0 < ConveyorCounter ) &&
                        !sCurrState.powercellInPosition5 ) {
	      sCurrState.iConveyPercent = -30;
              m_motorConveyMaster.Set( ControlMode::PercentOutput, -0.3 );
	      ConveyorCounter--;
            } else {
            //if (  sPrevState.powercellInIntake && 
            //     !sPrevState.powercellInPosition5 ) {
	        sCurrState.iConveyPercent = 0;
                m_motorConveyMaster.Set( ControlMode::PercentOutput, 0.0 );
            // }
            } 
         }
      }
//    if ( sPrevState.iConveyPercent != sCurrState.iConveyPercent ) {
//       m_motorConveyMaster.Set( ControlMode::PercentOutput,
//	                          (double)sCurrState.iConveyPercent / 100.0 );
//    }
   }   // RunConveyor()

      /*---------------------------------------------------------------------*/
      /* RunColorWheel()                                                     */
      /* Extend or retract the color wheel flipper.                          */
      /*---------------------------------------------------------------------*/
   void RunColorWheel( void ) {
      static int iCallCount = 0;
      static bool bFlipperState = false;
      iCallCount++;

                                 // Console button 6 is the lowest-left button
      if ( !sPrevState.conButton[6] && sCurrState.conButton[6] ) {
         if ( bFlipperState ){
            m_flipperSolenoid.Set( frc::DoubleSolenoid::Value::kForward);
         } else {
            m_flipperSolenoid.Set( frc::DoubleSolenoid::Value::kReverse);
         }
         bFlipperState = !bFlipperState; 
      }
      if ( sCurrState.conButton[7]){
         m_motorFlippyFlippy.Set(ControlMode::PercentOutput, 0.2);
      } else {
         m_motorFlippyFlippy.Set(ControlMode::PercentOutput, 0.0);
      }
   }

      /*---------------------------------------------------------------------*/
      /* RunClimberPole()                                                    */
      /* Extend or retract the telescoping climber pole.                     */
      /*---------------------------------------------------------------------*/
   void RunClimberPole( void ) {
      static int iCallCount = 0;
      static bool limitSwitchHasBeenHit = false;
      iCallCount++;

      if (sCurrState.conButton[1] && sCurrState.conButton[3] ) {
         m_motorClimberPole.Set( ControlMode::PercentOutput,
                                 0.5*sCurrState.joyZ);
      
      } else if ( sCurrState.conButton[1] ){
         //if ( !sPrevState.conButton[1] ) {       // if button 1 has just been
           // limitSwitchHasBeenHit = false;       // pressed, reset to start
           // m_compressor.Stop();
         //}
         if ( limitSwitchHasBeenHit ) {
                 // we are at the top; just supply a little power to stay there
            m_motorClimberPole.Set( ControlMode::PercentOutput, 0.10 );
         } else {
                                                   // apply full climbing power
            m_motorClimberPole.Set( ControlMode::PercentOutput, 0.4 );
            if ( m_motorClimberPole.IsFwdLimitSwitchClosed() ) {
               limitSwitchHasBeenHit = true;
            }
         }
      } else if (sCurrState.conButton[3] ) {
         m_motorClimberPole.Set( ControlMode::PercentOutput, -0.05);
           
      } else { 
         // Else neither button is currently being pressed.  If either was
	 // previously pressed, stop sending power to climber pole motor
         if (sPrevState.conButton[1] || sPrevState.conButton[3] ) {
            m_motorClimberPole.Set( ControlMode::PercentOutput, 0.0);
         }
      }
      

      if ( 0 == iCallCount%103 ) { // every 2 seconds
         if ( sCurrState.conButton[1] || sCurrState.conButton[3] ) {
            if ( sCurrState.conButton[1] ) {
               cout << "ClimberUp: ";
            } else {
               cout << "ClimberDown: ";
            }
            cout << setw(5) <<
               m_motorClimberPole.GetStatorCurrent() << "A" << endl;
            if ( m_motorClimberPole.IsFwdLimitSwitchClosed() ) {
               cout << "Climber pole at top." << endl;
            } else if ( m_motorClimberPole.IsRevLimitSwitchClosed() ) {
               cout << "Climber pole at bottom." << endl;
            }  
         }
      }
   }      // RunClimberPole() 


      /*---------------------------------------------------------------------*/
      /* RunClimberWinch()                                                   */
      /* Run the winch motor, to make the robot climb.                       */
      /*---------------------------------------------------------------------*/
   void RunClimberWinch( void ) {
      static int iCallCount = 0;
      iCallCount++;

      if ( sCurrState.conButton[9] ){
         m_motorClimberWinch.Set( ControlMode::PercentOutput, 0.5 );
         if ( 0 == iCallCount%50 ) {
            cout << "ClimberWinch Current: " << setw(5) <<
                    m_motorClimberWinch.GetStatorCurrent() << "A" << endl;
         }
      } else { 
         m_motorClimberWinch.Set( ControlMode::PercentOutput, 0.0 );
      }
   }      // RunClimberWinch()


      /*---------------------------------------------------------------------*/
      /* MotorInit()                                                         */
      /* Setup the initial configuration of a motor.  These settings can be  */
      /* superseded after this function is called, for the needs of each     */
      /* specific motor.                                                     */
      /*---------------------------------------------------------------------*/
   void MotorInit( WPI_TalonSRX & m_motor ) {

      m_motor.ConfigFactoryDefault( 10 );

                /* Configure Sensor Source for Primary PID */
          /* Config to stop motor immediately when limit switch is closed. */
                                                   // if encoder is connected
      if ( OK == m_motor.ConfigSelectedFeedbackSensor(
                     FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10 ) ) {
         m_motor.ConfigForwardLimitSwitchSource(
                     LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
                     LimitSwitchNormal::LimitSwitchNormal_NormallyOpen );
         m_motor.ConfigReverseLimitSwitchSource(
                     LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
                     LimitSwitchNormal::LimitSwitchNormal_NormallyOpen );
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
      // m_motor.SetInverted(false);  // invert direction of motor itself.

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
      m_motor.ConfigPeakCurrentLimit(60);    // 60 works here for miniCIMs
      m_motor.ConfigPeakCurrentDuration(1);  // 1000 milliseconds (for 60 Amps)
                                             // works fine here, with 40 for
                                             // ConfigContinuousCurrentLimit(),
                                             // but we can reduce to 10, 1, 10
                                             // for safety while debugging
      m_motor.ConfigContinuousCurrentLimit(40);
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

                   // print debugging info (this code should be removed later)
      if ( ( mSeqPrev.index != mSeq.index ) ||
	   ( mSeqPrev.type  != mSeq.type  )    ) {
            cout << "executeManeuver: Changed Maneuver, index: ";
            cout << mSeqPrev.index << " > " << mSeq.index << " ." << endl;
            cout << "                                    type: ";
            cout << mSeqPrev.type  << " > " << mSeq.type  << " ." << endl;
            cout << "                                distance: ";
            cout << mSeqPrev.distance << " > " << mSeq.distance << " ." << endl;
            cout << "                                     yaw: ";
            cout << mSeqPrev.yaw   << " > " << mSeq.yaw << " ." << endl;
      }

      switch ( mSeq.type )
      {
      case M_STOP:
         Team4918Drive( 0.0, 0.0 );       // make sure drive motors are stopped
         bRetVal = true;                  // and exit this maneuver immediately
         break;

      case M_DRIVE_STRAIGHT:
                   // Drive straight at the specified yaw angle for a specified
                   // distance, (but if mSeq.bDivertToPcell is TRUE, and
                   // a powercell is seen by the camera, then drive toward that
		   // powercell instead).
                   // The DriveToDistance() function returns true
                   // when it has driven far enough.
         bRetVal = DriveToDistance( sCurrState.initialYaw + mSeq.yaw,
			            mSeq.distance,
				    mSeq.bDivertToPcell,
				    false );
				  // If we have driven far enough...
	 if ( bRetVal ) {
            cout << "EM: DriveToDistance completed, heading: ";
	    cout << sCurrState.yawPitchRoll[0]  << endl;
            cout << "            distance ticks, left/right: ";
	    cout << sCurrState.iLSMasterPosition << " / ";
	    cout << sCurrState.iRSMasterPosition << "." << endl;
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
            if ( mSeq.bDivertToPcell ) {
               Team4918Drive( 0.5, -0.47 );   // turn left tightly
            } else {
               Team4918Drive( 0.5, -0.44 );   // turn left widely
            }
	 } else {                          // else
            Team4918Drive( -0.5, -0.44 );   // turn left tightly
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
            if ( mSeq.bDivertToPcell ) {
               Team4918Drive( 0.5,  0.47 );   // turn right tightly
            } else {
               Team4918Drive( 0.5,  0.44 );   // turn right widely
            }
	 } else {                          // else
            Team4918Drive( -0.5,  0.44 );   // turn right tightly
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

      case M_SHIFT_LOW:
         bRetVal = true;                  // and exit this maneuver immediately
         sCurrState.highGear = false;
         m_shiftingSolenoid.Set( false );
         break;

      case M_SHIFT_HIGH:
         bRetVal = true;                  // and exit this maneuver immediately
         sCurrState.highGear = true;
         m_shiftingSolenoid.Set( true );
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
   }


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
                             mSeqNext.bDivertToPcell,
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

      iRobotInitCallCount++;

      if ( 1 == iRobotInitCallCount ) {
                                // start a thread processing USB camera images
         std::thread visionThread(VisionThread);
         visionThread.detach();
      }

      powercellOnVideo.TestMode = false;

      m_motorLSSlave1.Follow(m_motorLSMaster);
      m_motorLSSlave2.Follow(m_motorLSMaster);
      m_motorRSSlave1.Follow(m_motorRSMaster);
      m_motorRSSlave2.Follow(m_motorRSMaster);
      m_motorConveySlave.Follow(m_motorConveyMaster);

      MotorInit( m_motorLSMaster );
      MotorInit( m_motorRSMaster );
      MotorInit( m_motorTopShooter );
      MotorInit( m_motorBotShooter );
      MotorInit( m_motorClimberPole );
                                    // invert encoder value positive/negative
                                    // and motor direction, for some motors.
      //    m_motorLSMaster.SetSensorPhase(true);
      m_motorLSMaster.SetSensorPhase(false);
      // m_motorLSMaster.SetInverted(false);
      //    m_motorRSMaster.SetSensorPhase(true);
      m_motorRSMaster.SetSensorPhase(false);
      // m_motorRSMaster.SetInverted(false);
      //    m_motorTopShooter.SetSensorPhase(false);
      m_motorTopShooter.SetSensorPhase(true);
      // m_motorTopShooter.SetInverted(false);
      //    m_motorBotShooter.SetSensorPhase(false);
      m_motorBotShooter.SetSensorPhase(true);
      // m_motorBotShooter.SetInverted(false);
      m_motorClimberPole.SetSensorPhase(false);
      m_motorClimberPole.SetInverted(false);

            /* Set Closed Loop PIDF gains in slot0 - see documentation */
                                                    // if encoder is connected
      if ( OK == m_motorLSMaster.ConfigSelectedFeedbackSensor(
                            FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10 ) ) {
         m_motorLSMaster.SelectProfileSlot( 0, 0 );
         m_motorLSMaster.Config_kF( 0, 0.025,   10 );   // these work well for .15
         m_motorLSMaster.Config_kP( 0, 0.02,    10 );   // RPMs above ~200 0.2
         m_motorLSMaster.Config_kI( 0, 0.0,     10 );// 0.0002
         m_motorLSMaster.Config_kD( 0, 0.0,     10 );
         cout << "LSMaster encoder is okay" << endl;
      } else {
         m_motorLSMaster.SelectProfileSlot( 0, 0 );
         m_motorLSMaster.Config_kF( 0, 0.15, 10 );   // may have to be higher
         m_motorLSMaster.Config_kP( 0, 0.0,  10 );
         m_motorLSMaster.Config_kI( 0, 0.0,  10 );
         m_motorLSMaster.Config_kD( 0, 0.0,  10 );
         cout << "LSMaster encoder is DISCONNECTED" << endl;
      }

                                                    // if encoder is connected
      if ( OK == m_motorRSMaster.ConfigSelectedFeedbackSensor(
                          FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10 ) ) {
         m_motorRSMaster.SelectProfileSlot( 0, 0 );
         m_motorRSMaster.Config_kF( 0, 0.025,   10 ); // these work well for .15
         m_motorRSMaster.Config_kP( 0, 0.02,    10 ); // RPMs above ~200 0.2
         m_motorRSMaster.Config_kI( 0, 0.0, 10 ); // 0.0002
         m_motorRSMaster.Config_kD( 0, 0.0,   10 ); // 10.0
         cout << "RSMaster encoder is okay" << endl;

      } else {
         m_motorRSMaster.SelectProfileSlot( 0, 0 );
         m_motorRSMaster.Config_kF( 0, 0.15, 10 );   // may have to be higher
         m_motorRSMaster.Config_kP( 0, 0.0,  10 );
         m_motorRSMaster.Config_kI( 0, 0.0,  10 );
         m_motorRSMaster.Config_kD( 0, 0.0,  10 );
         cout << "RSMaster encoder is DISCONNECTED" << endl;
      }

      m_motorLSMaster.SetSelectedSensorPosition( 0, 0, 10 );
      m_motorRSMaster.SetSelectedSensorPosition( 0, 0, 10 );
      m_motorLSMaster.SetIntegralAccumulator( 0.0 );
      m_motorRSMaster.SetIntegralAccumulator( 0.0 );
      LSMotorState.targetVelocity_UnitsPer100ms = 0.0 * 4096 / 600;
      RSMotorState.targetVelocity_UnitsPer100ms = 0.0 * 4096 / 600;
      Team4918Drive( 0.0, 0.0 );          // make sure drive motors are stopped

                                                     // if encoder is connected
      if ( OK == m_motorTopShooter.ConfigSelectedFeedbackSensor(
                          FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10 ) ) {
         m_motorTopShooter.SelectProfileSlot( 0, 0 );
         m_motorTopShooter.Config_kF( 0, 0.02,    10 ); // 0.02 0.01
         m_motorTopShooter.Config_kP( 0, 0.3,     10 ); // 0.2 0.08
         m_motorTopShooter.Config_kI( 0, 0.0,     10 ); // was 0.0 0.00008
         m_motorTopShooter.Config_kD( 0, 0.8,     10 );
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
         m_motorBotShooter.Config_kF( 0, 0.02,    10 ); // 0.02 0.01
         m_motorBotShooter.Config_kP( 0, 0.3,     10 ); // 0.2 0.08
         m_motorBotShooter.Config_kI( 0, 0.0,     10 ); // 0.0 0.00008
         m_motorBotShooter.Config_kD( 0, 0.8,     10 );
      } else {
         m_motorBotShooter.SelectProfileSlot( 0, 0 );
         m_motorBotShooter.Config_kF( 0, 0.01, 10 );   // may have to be higher
         m_motorBotShooter.Config_kP( 0, 0.0,  10 );
         m_motorBotShooter.Config_kI( 0, 0.0,  10 );
         m_motorBotShooter.Config_kD( 0, 0.0,  10 );
      }

      m_motorConveyMaster.SetNeutralMode( NeutralMode::Brake );

      sCurrState.highGear = false;
      m_shiftingSolenoid.Set(false);
      iCallCount++;
      sCurrState.teleop = false;
      // new m_compressor(0);           // initialize compressor
      // new m_compressor = frc::Compressor(0); // initialize compressor

      // Default to a length of 60; start empty output
      // length is expensive to set, so only set it once, then just update data
      m_led.SetLength( kLEDStripLength );
      m_led.SetData( m_ledBuffer );
      m_led.Start();

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
//    void RobotPeriodic() override {
//       static int iCallCount = 0;      iCallCount++;
// 
//      GetAllVariables();
// 
//      SwitchCameraIfNecessary();
//      cout << "RobotPeriodic()" << endl;
//    }


      /*---------------------------------------------------------------------*/
      /* DisabledInit()                                                      */
      /* This function is called once when the robot is disabled.            */
      /*---------------------------------------------------------------------*/
   void DisabledInit() override {

      m_motorLSMaster.SetSelectedSensorPosition( 0, 0, 10 );
      m_motorRSMaster.SetSelectedSensorPosition( 0, 0, 10 );
      m_motorLSMaster.SetIntegralAccumulator( 0.0 );
      m_motorRSMaster.SetIntegralAccumulator( 0.0 );
      LSMotorState.targetVelocity_UnitsPer100ms = 0.0 * 4096 / 600;
      RSMotorState.targetVelocity_UnitsPer100ms = 0.0 * 4096 / 600;
      Team4918Drive( 0.0, 0.0 );          // make sure drive motors are stopped

      powercellOnVideo.TestMode = false;
      // Should power off all motors here.
   }


      /*---------------------------------------------------------------------*/
      /* TestInit()                                                          */
      /* This function is called once when the robot enters Test mode.       */
      /*---------------------------------------------------------------------*/
   void TestInit() override {
      limenttable->PutNumber( "ledMode", 1 );                  // turn LEDs off
      powercellOnVideo.TestMode = true;  // display the center pixel HSV values
   }

   void LEDBlue() {
      for ( int i = 0; i < kLEDStripLength; i++ ) {
                     // range of arg1 (hue) is 0-180;
                     // range of arg2 and arg3 (saturation and value) is 0-255
         m_ledBuffer[i].SetHSV(100, 255, 128);  // half-bright blue
      }
      m_led.SetData( m_ledBuffer );
   }

   void LEDRainbow() {
	      // for every pixel...
      for ( int i = 0; i < kLEDStripLength; i++ ) {
	      // calculate the hue - hue is easier for rainbows because the
	      // color shape is a circle, so only one value needs to increment
	 const auto pixelHue = ( firstLEDStripPixelHue +
			         (i + 180/kLEDStripLength) ) % 180;
              // set the value
         m_ledBuffer[i].SetHSV( pixelHue, 255, 128 );
      }
      m_led.SetData( m_ledBuffer );
         // Increase the first pixel hue, to make the rainbow move every 20 ns
      firstLEDStripPixelHue += 3;
         // check bounds
      firstLEDStripPixelHue %= 180;
      m_led.SetData( m_ledBuffer );
   }

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
      #if 1 
      //follow left wall
                     //pigeon units in trig directions, not compass directions
      // FollowWall(23.0, 0.0, 0.0, 35.0, -25.0);
      #else
      //follow right wall
      //FollowWall(0.0, 23.0, 0.0, -35.0, 35.0 );
      #endif
      SwitchCameraIfNecessary();

      if ( 0 == iCallCount%10007 ) {                       // every 200 seconds
         // cout << "Sonar0 sensor 0: " << distSensor0.GetAverageValue() << endl;
         // cout << "Sonar0 average voltage:  " << distSensor0.GetAverageVoltage()
         //      << endl;
         // cout << "Sonar0 voltage:  " << distSensor0.GetVoltage() << endl;
             // convert to inches, with 100 centimeters/volt and 2.54 cm/inch
         cout << "Sonar0 distance: " << distSensor0.GetVoltage() * 100 / 2.54
              << " inches (" << distSensor0.GetVoltage() << ")." << endl; 
         cout << "Sonar1 distance: " << distSensor1.GetVoltage() * 100 / 2.54
              << " inches (" << distSensor1.GetVoltage() << ")." << endl; 
      }

      if ( 0 == iCallCount%100 )  {   // every 2 seconds
         // JoystickDisplay();
      }
//      cout << "TestPeriodic()" << endl;

      LEDBlue();         // light the entire LED strip blue
      // LEDRainbow();   // light the LED strip with a moving rainbow
   }


      /*---------------------------------------------------------------------*/
      /* AutonomousInit()                                                    */
      /* This function is called once when the robot enters Autonomous mode. */
      /*---------------------------------------------------------------------*/
   void AutonomousInit() override {
      RobotInit();
      m_compressor.EnableDigital();
      limenttable->PutNumber( "ledMode", 3 );                   // turn LEDs on
      cout << "shoot 3 balls" << endl;
      // m_drive.StopMotor();
      iCallCount=0;
      GetAllVariables();
      sCurrState.teleop = false;
      sCurrState.initialYaw = sCurrState.yawPitchRoll[0]; 
      // TurnToHeading( sCurrState.initialYaw, true );
      // DriveToDistance( sCurrState.initialYaw, 0.0, false, true );
      m_motorLSMaster.SetSelectedSensorPosition( 0, 0, 10 );
      m_motorRSMaster.SetSelectedSensorPosition( 0, 0, 10 );
      m_motorLSMaster.SetIntegralAccumulator( 0.0 );
      m_motorRSMaster.SetIntegralAccumulator( 0.0 );
      m_motorLSMaster.ConfigClosedloopRamp(0.1);
      m_motorRSMaster.ConfigClosedloopRamp(0.1);
      m_motorLSMaster.ConfigOpenloopRamp(  0.1);
      m_motorRSMaster.ConfigOpenloopRamp(  0.1);
      LSMotorState.targetVelocity_UnitsPer100ms = 0.0 * 4096 / 600;
      RSMotorState.targetVelocity_UnitsPer100ms = 0.0 * 4096 / 600;
      Team4918Drive( 0.0, 0.0 );          // make sure drive motors are stopped

                             // mSeqIndex can be set to different values,
                             // based on the console switches.
      mSeqIndex = 0;         // ballgrabber sequence (untested)
      // mSeqIndex =  60;       // barrel sequence (works)
      // mSeqIndex =  80;       // slalom sequence (works)
      // mSeqIndex = 100;       // bounce sequence (works)

                             // Initialize yaw and distance, so next maneuvers
			     // are relative to these current settings.
      TurnToHeading( sCurrState.initialYaw, true );
      DriveToDistance (sCurrState.initialYaw, 0.0, false, true);
   }      // AutonomousInit()


      /*---------------------------------------------------------------------*/
      /* AutonomousPeriodic()                                                */
      /* This function is called every 20 milliseconds, as long as the robot */
      /* is in Autonomous mode.                                              */
      /*---------------------------------------------------------------------*/
   void AutonomousPeriodic() override {

      // static double dDesiredYaw = 0.0;

      GetAllVariables();

      iCallCount++;

      //m_motorIntake.Set(ControlMode::PercentOutput, -0.4);

//      if ( sCurrState.conButton[8] )   {            // Run intake forward.
         if (  sCurrState.powercellInIntake ) {       // if powercell in intake
            m_motorIntake.Set( ControlMode::PercentOutput, -0.1 ); // be gentle
	 } else {
            m_motorIntake.Set( ControlMode::PercentOutput, -0.4 ); // be strong
	 }
//      } else {                                         // Stop the intake.
//         m_motorIntake.Set( ControlMode::PercentOutput, 0.0 );
//      }

      RunConveyor();

      // m_drive.StopMotor();
      LSMotorState.targetVelocity_UnitsPer100ms = 0;        // Left Side drive
      RSMotorState.targetVelocity_UnitsPer100ms = 0;        // Right Side drive

      // dDesiredYaw = sCurrState.initialYaw;

                            // Perform a sequence of maneuvers, transitioning
                            // to next maneuver in the sequence when necessary.
      mSeqIndex = executeManeuverSequence( mSeqIndex );
      
#ifdef JAG_NOTDEFINED
      motorFindMinMaxVelocity( m_motorLSMaster, LSMotorState );
      motorFindMinMaxVelocity( m_motorRSMaster, RSMotorState );

      if ( 0 == iCallCount%50 ) {
//         MotorDisplay( "LS:", m_motorLSMaster, LSMotorState ); 
//         MotorDisplay( "RS:", m_motorRSMaster, RSMotorState );
         IMUOrientationDisplay();
      }
#endif

      return; 

   }      // AutonomousPeriodic()


      /*---------------------------------------------------------------------*/
      /* TeleopInit()                                                        */
      /* This function is called once when the robot enters Teleop mode.     */
      /*---------------------------------------------------------------------*/
   void TeleopInit() override {
      RobotInit();
      m_compressor.EnableDigital();
                                                    // zero the drive encoders
      m_motorLSMaster.SetSelectedSensorPosition( 0, 0, 10 );
      m_motorRSMaster.SetSelectedSensorPosition( 0, 0, 10 );
      m_motorLSMaster.SetIntegralAccumulator( 0.0 );
      m_motorRSMaster.SetIntegralAccumulator( 0.0 );
      m_motorLSMaster.ConfigClosedloopRamp(0.0);
      m_motorRSMaster.ConfigClosedloopRamp(0.0);
      m_motorLSMaster.ConfigOpenloopRamp(  0.0);
      m_motorRSMaster.ConfigOpenloopRamp(  0.0);
      LSMotorState.targetVelocity_UnitsPer100ms = 0.0 * 4096 / 600;
      RSMotorState.targetVelocity_UnitsPer100ms = 0.0 * 4096 / 600;
      sCurrState.teleop = true;
      Team4918Drive( 0.0, 0.0 );          // make sure drive motors are stopped
   }      // TeleopInit()


      /*---------------------------------------------------------------------*/
      /* TeleopPeriodic()                                                    */
      /* This function is called every 20 milliseconds, as long as the robot */
      /* is in Teleop mode.                                                  */
      /*---------------------------------------------------------------------*/
   void TeleopPeriodic() override {

      GetAllVariables();  // this is necessary if we use any
                          // of the Canbus variables.

      //m_motorIntake.Set(ControlMode::PercentOutput, -0.4);

      if ( sCurrState.conButton[8] )   {            // Run intake forward.
         if (  sCurrState.powercellInIntake ) {       // if powercell in intake
            m_motorIntake.Set( ControlMode::PercentOutput, -0.1 ); // be gentle
	 } else {
            m_motorIntake.Set( ControlMode::PercentOutput, -0.4 ); // be strong
	 }
      } else {                                         // Stop the intake.
         m_motorIntake.Set( ControlMode::PercentOutput, 0.0 );
      } 


      RunDriveMotors();

      RunShooter();

      Shoot();

      RunConveyor();

      RunColorWheel();

      RunClimberPole();
      RunClimberWinch();
      SwitchCameraIfNecessary();


      sPrevState = sCurrState;

      if ( 0 == iCallCount%100000 )  {   // every 20 seconds
         // cout << "TelPeriodic loop duration: ";
         // cout << frc::GetTime() - dTimeOfLastCall << endl;
               // use frc:Timer::GetFPGATimestamp() instead?
      }

      iCallCount++;
   }      // TeleopPeriodic()
 
};      // class Robot definition (derives from frc::TimedRobot )


Robot::sPowercellOnVideo Robot::powercellOnVideo = { true, 0, 0, -1, false, false };

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

