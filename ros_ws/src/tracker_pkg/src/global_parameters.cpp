#include "tracker_pkg/global_parameters.h"


//Kalman filter setting
const double INIT_X = 0.0;
const double INIT_Y = 0.0;
const double INIT_Z = 0.0;
const double INIT_VX = 0.0;
const double INIT_VY = 0.0;
const double INIT_VZ = 0.0;
const double INIT_AX = 0.0;
const double INIT_AY = 0.0;
const double INIT_AZ = 0.0;
const double NOISE_POS =5e-1;//1e-4
const double NOISE_VEL = 5e-1;
const double NOISE_ACC = 5e-1;
const double NOISE_SENSOR = 1e1;//5e0

const int COUNTER_LOST = 100;//humman life span.

std::queue<Cam2Yolo> q_cam2yolo;
std::queue<Yolo2Tracking> q_yolo2tracking;
std::queue<bool> q_end;