#include "pid.h"

PID::PID()
{
  resetPID();
}


// đầu vào là error=theta_d-theta 
void PID::runPID()
{
  P_part = Kp * error;
  I_part = Ki * Ts / 2 * (error + pre_error);
  // D_part = Kd / Ts * (error - 2 * pre_error + pre_pre_error);
  D_part = (Kd * N / (1.0 + N * Ts)) * (error - 2 * pre_error + pre_pre_error) + (1.0 / (1.0 + N * Ts)) * D_part; 
  output = P_part + I_part + D_part;
  pre_pre_error = pre_error;
  pre_error = error;

  // P_part = Kp * (error-pre_error);
  // I_part = Ki * Ts / 2 * (error + pre_error);
  // // D_part = (Kd * N / (1.0 + N * Ts)) * (error - 2 * pre_error + pre_pre_error) + (1.0 / (1.0 + N * Ts)) * D_part; // Lọc đạo hàm bằng IIR bậc 1
  // D_part = Kd / Ts * (error - 2 * pre_error + pre_pre_error);
  // output =  pre_output + P_part + I_part + D_part;
  // // output = P_part + I_part + D_part;
  // pre_pre_error = pre_error;
  // pre_error = error;
  // pre_output=output;
// ========================================================================================


//   P_part = Kp * error;
// //   I_sum += (error + pre_error) * Ts * 0.5;
// I_sum += (error) * Ts ;

//   // Chống tràn tích phân (Anti-windup)
//   if (I_sum > I_max) I_sum = I_max;
//   if (I_sum < -I_max) I_sum = -I_max;

//   I_part = Ki * I_sum;

//   // D - Đạo hàm bậc 1 
//   D_part = Kd / Ts * (error - pre_error);

//   // Tổng đầu ra PID
//   output = P_part + I_part + D_part;

//   // Cập nhật lỗi cho vòng sau
//   pre_error = error;

}

void PID::resetPID()
{
  error = 0;
  pre_error = 0;
  pre_pre_error = 0;
  pre_output=0;
  I_sum = 0;
  output = 0;
}
