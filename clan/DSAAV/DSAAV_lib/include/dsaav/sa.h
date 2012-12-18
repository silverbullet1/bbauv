/**
 * DSAAV Library - Sensor/Actuator Service.
 *
 * (c) 2006 Acoustic Research Laboratory,
 *          National University of Singapore.
 *
 * \file
 * \author    Mandar Chitre
 * \version   $Revision: 1.48 $
 */

#ifndef _DSAAV_SA_H_
#define _DSAAV_SA_H_

/**
 * Type of mesaurement.
 */
enum MeasurementType {
  MT_ALTITUDE = 0x00,          //!< Altitude of AUV above sea bed
  MT_ATTITUDE = 0x01,          //!< Pitch, yaw, tilt and bearing of AUV
  MT_VELOCITY = 0x02,          //!< Velocity of AUV
  MT_OBSTACLE = 0x03,          //!< Range, bearing of obstacles
  MT_POSITION = 0x04,          //!< X, Y position of AUV
  MT_DEPTH = 0x05,             //!< Depth of AUV from sea surface
  MT_RUDDER = 0x06,            //!< Rudder position
  MT_ELEVATORS = 0x07,         //!< Elevator positions
  MT_THRUST = 0x08,            //!< Forward thrust
  MT_RPM = 0x09,               //!< Thruster RPM
  MT_TEMPERATURE = 0x0A,       //!< Temperature
  MT_IMU = 0x0B,               //!< 6-axis IMU measurements
  MT_TIME = 0x0C,              //!< Current time (since midnight Jan 1 1970)
  MT_MEASUREMENT1 = 0x0D,      //!< example measurement type provided by component1
  MT_MEASUREMENT2 = 0x0E,      //!< example measurement type provided by component2
  MT_LAST = 0x400              //!< Dummy entry to allow counting, DO NOT modify, add new entries before this (must be less than 0x400)
};

/**
 * Mesaured quantity.
 */
enum Quantity {
  MQ_ALTITUDE = 0x00,          //!< Altitude of AUV above sea bed (m)
  MQ_PITCH = 0x01,             //!< Pitch of AUV (rad, ccw, +nosedown)
  MQ_MEASUREMENT1 = 0x02,      //!< example measurement quantity provided by component1
  MQ_MEASUREMENT2 = 0x03,      //!< example measurement quantity provided by component2
};

/**
 * Actuator type.
 */
enum ActuatorType {
  AT_RUDDER = 0x00,            //!< Rudder position (rad, ccw 0 neutral)
  AT_ACTUATOR2 = 0x01,         //!< example actuator type accepted by component2
  AT_LAST = 0x400              //!< Dummy entry to allow counting,DO NOT modify, add new entries before this (must be less than 0x400)
};

/**
 * Represents a measurement made by a sensor.
 */
class Measurement : public ParamSet {
  public:
    Measurement(int count, double timestamp);
    Measurement(Measurement* m);
    int count(void);
    float get(Quantity qty, int n = 0);
    void put(Quantity qty, float value);
    double getd(Quantity qty, int n = 0);
    void putd(Quantity qty, double value);
    float getAge(void);
    String* toString(void);
};

/**
 * Sensor or actuator service interface.
 */
class Sentuator {
  private:
    MsgSvcAddr* server;
    Rpc* rpc;
  public:
    Sentuator(Rpc* rpc, MsgSvcAddr* server);
    ~Sentuator(void);
    Measurement* get(MeasurementType type, float maxAge);
    int set(ActuatorType type, float value);
    int set(ActuatorType type, int count, float* values);
    void nbset(ActuatorType type, float value);
    void nbset(ActuatorType type, int count, float* values);
};

// constants

const float VAL_UNKNOWN = -1e9;     //!< Represents an unknown value
const float VAL_DISABLE = -2e9;     //!< Value used to disable a control set point / thruster
const float VAL_CURRENT = -3e9;     //!< Represents a current value (useful for set points in control systems)
const float VAL_ENABLE  = -4e9;     //!< Value used to re-enable a disabled thruster

#endif
