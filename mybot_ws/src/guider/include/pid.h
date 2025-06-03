#ifndef PID_H
#define PID_H

class PID
{
public:
  PID();

  double Kp ;
  double Ki ;
  double Kd ;

  double threshold;

  double P_part;
  double I_part;
  double D_part;
  double I_sum = 0;
  double I_max = 10.0; // Giới hạn tích phân tùy hệ
  
  double Ts;
  double error;
  double pre_error;
  double pre_pre_error;
  double output;
  double pre_output;
  // Tham số bộ lọc đạo hàm IIR
  double N=20;     // hệ số lọc đạo hàm (cutoff freq)

  void runPID();
  void resetPID();
};

#endif // PID_H
 
