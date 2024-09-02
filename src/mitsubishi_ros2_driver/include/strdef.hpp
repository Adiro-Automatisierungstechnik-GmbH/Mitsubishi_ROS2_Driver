//************************************************************************************
// Real-time control sample program
// Communication packet data structure definition header file
//************************************************************************************
// strdef.hpp
#define VER_H7


/*************************************************************************/
/* Joint coordinate system (Set unused axis to 0) */
/* Refer to the instruction manual enclosed */
/* with each robot for details on each element. */
/************************************************************************/
typedef struct{
    float j1; // J1 axis angle (radian)
    float j2; // J2 axis angle (radian)
    float j3; // J3 axis angle (radian)
    float j4; // J4 axis angle (radian)
    float j5; // J5 axis angle (radian)
    float j6; // J6 axis angle (radian)
    float j7; // Additional axis 1 (J7 axis angle) (radian)
    float j8; // Additional axis 2 (J8 axis angle) (radian)
} JOINT;


/*************************************************************************/
/* XYZ coordinate system (Set unused axis to 0) */
/* Refer to the instruction manual enclosed */
/* with each robot for details on each element. */
/************************************************************************/
typedef struct{
    float x; // X axis coordinate value (mm)
    float y; // Y axis coordinate value (mm)
    float z; // Z axis coordinate value (mm)
    float a; // A axis coord= 0inate value (radian)
    float b; // B axis coordinate value (radian)
    float c; // C axis coordinate value (radian)
    float l1; // Additional axis 1 (mm or radian)
    float l2; // Additional axis 2 (mm or radian)
} WORLD;

typedef struct{
    WORLD w;
    unsigned intsflg1; // Structural flag 1
    unsigned intsflg2; // Structural flag 2
} POSE;

/*************************************************************************/
/* Pulse coordinate system (Set unused axis to 0) */
/* These coordinates express each joint */
/* with a motor pulse value. */
/*************************************************************************/
typedef struct{
    int p1; // Motor 1 axis
    int p2; // Motor 2 axis
    int p3; // Motor 3 axis
    int p4; // Motor 4 axis
    int p5; // Motor 5 axis
    int p6; // Motor 6 axis
    int p7; // Additional axis 1 (Motor 7 axis)
    int p8; // Additional axis 2 (Motor 8 axis)
} PULSE;

/************************************************************/
/* Real-time function communication data packet */
/************************************************************/
typedef struct enet_rtcmd_str 
{
    unsigned short Command; // Command
    #define MXT_CMD_NULL 0 // Real-time external command invalid
    #define MXT_CMD_MOVE 1 // Real-time external command valid
    #define MXT_CMD_END 255 // Real-time external command end

    unsigned short SendType; // Command data type designation
    unsigned short RecvType; // Monitor data type designation

                            //////////// Command or monitor data type ///
    #define MXT_TYP_NULL 0 // No data

                                // For the command and monitor ////////////////////
    #define MXT_TYP_POSE 1 // XYZ data
    #define MXT_TYP_JOINT 2 // Joint data
    #define MXT_TYP_PULSE 3 // pulse data

    ///////////// For position related monitor ///
    #define MXT_TYP_FPOSE 4 // XYZ data (after filter process)
    #define MXT_TYP_FJOINT 5 // Joint data (after filter process)
    #define MXT_TYP_FPULSE 6 // Pulse data (after filter process)
    #define MXT_TYP_FB_POSE 7 // XYZ data (Encoder feedback value)
    #define MXT_TYP_FB_JOINT 8 // Joint data (Encoder feedback value)
    #define MXT_TYP_FB_PULSE 9 // Pulse data (Encoder feedback value)

    // For current related monitors ////////////////////
    #define MXT_TYP_CMDCUR 10 // Electric current command
    #define MXT_TYP_FBKCUR 11 // Electric current feedback

    union rtdata // Command data
    { 
        POSE pos; // XYZ type [mm/rad]
        JOINT jnt; // Joint type [rad]
        PULSE pls; // Pulse type [pls]
        int lng1[8]; // Integer type [% / non-unit]
    } dat;

    unsigned short SendIOType; // Send input/output signal data designation
    unsigned short RecvIOType; // Return input/output signal data designation

#define MXT_IO_NULL 0 // No data
#define MXT_IO_OUT 1 // Output signal
#define MXT_IO_IN 2 // Input signal

    unsigned short BitTop; // Head bit No.
    unsigned short BitMask; // Transmission bit mask pattern designation (0x0001-0xffff)
    unsigned short IoData; // Input/output signal data (0x0000-0xffff)
    unsigned short TCount; // Timeout time counter value
    unsigned int CCount; // Transmission data counter value
    unsigned short RecvType1; // Reply data-type specification 1

    union rtdata1 { // Monitor data 1
        POSE pos1; // XYZ type [mm/rad]
        JOINT jnt1; // JOINT type [mm/rad]
        PULSE pls1; // PULSE type [mm/rad]
        int lng1[8]; // Integer type [% / non-unit]
    } dat1;

    unsigned short RecvType2; // Reply data-type specification 2

    union rtdata2 { // Monitor data 2
        POSE pos2; // XYZ type [mm/rad]
        JOINT jnt2; // JOINT type [mm/rad]
        PULSE pls2; // PULSE type [mm/rad] or Integer type [% / non-unit]
        int lng2[8]; // Integer type [% / non-unit]
    } dat2;

    unsigned short RecvType3; // Reply data-type specification 3

    union rtdata3 { // Monitor data 3
        POSE pos3; // XYZ type [mm/rad]
        JOINT jnt3; // JOINT type [mm/rad]
        PULSE pls3; // PULSE type [mm/rad] or Integer type [% / non-unit]
        int lng3[8]; // Integer type [% / non-unit]
    } dat3;

} MXTCMD;