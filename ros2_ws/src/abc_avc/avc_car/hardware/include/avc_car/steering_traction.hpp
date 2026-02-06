namespace avc_car
{
struct __attribute__((packed)) JETSONCOMMANDS {
  bool onoff;
  float steering_angle;
  float target_velocity;
} jetsonCommands;

struct __attribute__((packed)) ODOMETRY {
  float x;
  float y;
  float theta;
  float motorSpeed;
  int status;
} odometry;
}