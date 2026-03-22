namespace avc_car
{
struct __attribute__((packed)) JETSONCOMMANDS {
  bool onoff;
  float steering_angle;
  float target_velocity;
} jetsonCommands;

struct __attribute__((packed)) ODOMETRY {
  float x; //in rad unit
  float y;
  float theta; //in deg
  float motorVelocity;
  float motorPosition;
  float servoPosition;
  int status;
} odometry;
}