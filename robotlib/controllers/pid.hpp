
/**********************************************************************************************
 * @file pid.hpp
 * @brief PID library header modified from Arduino PID Library - Version 1.2.1
 * @author Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 * @date 2023
 *
 * @section license License
 * This PID Library is licensed under the MIT License
 *
 * - Modified by Robotics club, Pulchowk Campus
 *********************************************************************************************/

#ifndef _PID_HPP
#define _PID_HPP

#include "../common/processor_type.h"

/** @enum Mode
 *  @brief Enum for Auto or Manual Tuning modes.
 */
enum Mode
{
    MANUAL,   /**< Manual Tuning Mode */
    AUTOMATIC /**< Automatic Tuning Mode */
};

/** @enum ControlDirection
 *  @brief Enum for direct or reverse control direction.
 *
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.
 */
enum ControlDirection
{
    DIRECT, /**< Direct Control Direction */
    REVERSE /**< Reverse Control Direction */
};

/** @enum ProportionalType
 *  @brief Enum for proportional output based on error or measurement.
 */
enum ProportionalType
{
    PROPORTIONAL_ON_ERROR,      /**< Proportional on Error */
    PROPORTIONAL_ON_MEASUREMENT /**< Proportional on Measurement */
};

/** @class PID
 *  @brief PID Controller class.
 */
class PID
{
private:
    float kp_, ki_, kd_;
    float prev_kp_, prev_ki_, prev_kd_;
    float outMin_, outMax_;
    int16_t sampleTime_;
    float tol;
    ControlDirection controlDirection_;
    ProportionalType pType_;
    Mode mode_;

    float kpScaled_, kiScaled_, kdScaled_;
    float lastInput_ = 0.0f;
    uint32_t lastTime_;
    uint8_t allow_error_count = 0;

public:
    float input = 0.0f;
    float setpoint = 0.0f;
    float outputSum = 0.0f;
    float output = 0.0f;
    bool is_allowable_error = false;

    /** @brief Default constructor for PID class. */
    PID() = default;

    /** @brief Copy constructor for PID class. */
    PID(const PID &) = default;

    /** @brief Constructor for PID class.
     *  @param kp Proportional gain.
     *  @param ki Integral gain.
     *  @param kd Derivative gain.
     *  @param outMin Minimum output limit.
     *  @param outMax Maximum output limit.
     *  @param sampleTime Time between PID calculations.
     *  @param tol Tolerance value for allowable error
     *  @param controlDirection Control direction (Direct/Reverse).
     *  @param pType Type of proportional output (Error/Measurement).
     *  @param mode Tuning mode (Auto/Manual).
     */
    PID(float kp, float ki, float kd,
        float outMin, float outMax, int16_t sampleTime,
        float _tol = 1.0e-3f,
        ControlDirection controlDirection = DIRECT,
        ProportionalType pType = PROPORTIONAL_ON_ERROR,
        Mode mode = AUTOMATIC);

    /** @brief Initialization method for PID class. */
    virtual void init();

    /** @brief Performs the PID calculation.
     *  @return True if successful PID computation.
     *  @details It should be called every time loop() cycles. ON/OFF and
     *  calculation frequency can be set using SetMode and SetSampleTime respectively.
     */
    virtual bool compute();

    /** @brief Sets new tuning parameters.
     *  @param newKp New proportional gain.
     *  @param newKi New integral gain.
     *  @param newKd New derivative gain.
     */
    virtual void setTunings(float newKp, float newKi, float newKd);

    /** @brief Sets new output limits.
     *  @param newOutMin New minimum output limit.
     *  @param newOutMax New maximum output limit.
     */
    virtual void setOutputLimits(float newOutMin, float newOutMax);

    /** @brief Sets the tuning mode (Auto/Manual).
     *  @param newMode New tuning mode.
     */
    virtual void setMode(Mode newMode);

    /** @brief Sets the sample time for PID calculation.
     *  @param newSampleTime New sample time.
     */
    virtual void setSampleTime(uint16_t newSampleTime);

    virtual float getOutMax();
    virtual float getOutMin();

    virtual float get_error() { return setpoint - input; }
    virtual float get_kp() { return kp_; }
    virtual float get_ki() { return ki_; }
    virtual float get_kd() { return kd_; }

    /** @brief Destructor for PID class. */
    ~PID() = default;
};
#endif
