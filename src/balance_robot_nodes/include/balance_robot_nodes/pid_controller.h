#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

/**
 * @brief PID 컨트롤러 클래스
 * 
 * 비례-적분-미분 제어를 위한 클래스입니다.
 * 와인드업 방지 및 출력 제한 기능을 포함합니다.
 */
class PIDController {
public:
    /**
     * @brief PID 컨트롤러 생성자
     * @param kp 비례 게인
     * @param ki 적분 게인
     * @param kd 미분 게인
     * @param min_output 최소 출력값
     * @param max_output 최대 출력값
     */
    PIDController(double kp = 1.0, double ki = 0.0, double kd = 0.0, 
                  double min_output = -100.0, double max_output = 100.0);
    
    /**
     * @brief PID 게인 설정
     * @param kp 비례 게인
     * @param ki 적분 게인  
     * @param kd 미분 게인
     */
    void setGains(double kp, double ki, double kd);
    
    /**
     * @brief 출력 제한값 설정
     * @param min_output 최소 출력값
     * @param max_output 최대 출력값
     */
    void setOutputLimits(double min_output, double max_output);
    
    /**
     * @brief PID 제어기 리셋
     */
    void reset();
    
    /**
     * @brief PID 제어 계산
     * @param setpoint 목표값
     * @param measured_value 측정값
     * @param dt 시간 간격 (초)
     * @return 제어 출력값
     */
    double compute(double setpoint, double measured_value, double dt);
    
    // PID 상태 접근자
    double getError() const { return error_; }
    double getIntegral() const { return integral_; }
    double getDerivative() const { return derivative_; }
    double getOutput() const { return output_; }
    double getKp() const { return kp_; }
    double getKi() const { return ki_; }
    double getKd() const { return kd_; }

private:
    // PID 게인
    double kp_, ki_, kd_;
    
    // PID 상태 변수
    double error_;
    double prev_error_;
    double integral_;
    double derivative_;
    double output_;
    
    // 출력 제한
    double min_output_, max_output_;
    
    // 적분 와인드업 방지
    double integral_max_;
    
    bool first_run_;
};

#endif // PID_CONTROLLER_H 