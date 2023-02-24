#include<stdbool.h>

//put definition here
#define VOLTAGE_REFERENCE   24.0    //in Volt
#define CURRENT_REFERENCE   3.0     //in Ampere
#define CURRENT_MINIMUM     0.3     //in Ampere
#define CC_PI_KP            1.0
#define CC_PI_KI            100.0
#define CC_PI_LIMIT_MIN
#define CC_PI_LIMIT_MAX
#define CV_PI_KP            1.0
#define CV_PI_KI            100.0
#define CV_PI_LIMIT_MIN
#define CV_PI_LIMIT_MAX
#define SAMPLING_PERIOD     0.0001

bool enable_command;
uint8_t state;
uint32_t PWM;
double voltage_feedback, voltage_reference;                     //Battery terminal voltage
double current_feedback, current_reference, current_minimum;    //Battery charging current
double charger_voltage;                                         //Charger DC voltage
double duty_feedback, duty_reference, duty_PWM;

typedef struct {
    bool enable;
    double Kp, Ki;
    double error;
    double output;
    double output_limit_min, output_limit_max;
    double integrator_state;
    double dT;
} PI_CONTROL;

PI_CONTROL cc_pi_control;
PI_CONTROL cv_pi_control;

void Initialization(void){
    /*
    * initialize variables here
    */
    state = 0;
    enable_command = 0;
    voltage_reference = VOLTAGE_REFERENCE;
    current_reference = CURRENT_REFERENCE;
    current_minimum = CURRENT_MINIMUM;

    cc_pi_control.integrator_state = 0.0;
    cc_pi_control.Kp = CC_PI_KP;
    cc_pi_control.Ki = CC_PI_KI;
    cc_pi_control.output_limit_min = CC_PI_LIMIT_MIN;
    cc_pi_control.output_limit_max = CC_PI_LIMIT_MAX;
    cc_pi_control.dT = SAMPLING_PERIOD;

    cv_pi_control.integrator_state = 0.0;
    cv_pi_control.Kp = CV_PI_KP;
    cv_pi_control.Ki = CV_PI_KI;
    cv_pi_control.output_limit_min = CV_PI_LIMIT_MIN;
    cv_pi_control.output_limit_max = CV_PI_LIMIT_MAX;
    cv_pi_control.dT = SAMPLING_PERIOD;

    duty_reference = 1.0;
}

void control_routine(void){
    /*
    * run the control algorithm here
    * Constant-Current state: run Current Control algorithm only
    * Constant-Voltage state: run Voltage Control algorithm and Current Control algorithm
    */
   
   //Get voltage_feedback and current_feedback value
   //Assume that ADC_Conversion() convert the ADC_input value from ADC_channel to Volt or Ampere
   voltage_feedback = ADC_Conversion(ADC_input, 1);
   current_feedback = ADC_Conversion(ADC_input, 2);

   //Voltage Control algorithm:
   if(cv_pi_control.enable){
    //Calculate error
    cv_pi_control.error = voltage_reference - voltage_feedback;

    //Calculate PI output
    cv_pi_control.output = cv_pi_control.integrator_state + cv_pi_control.Kp * cv_pi_control.error;
    
    //Limit PI output if necessary
    if(cv_pi_control.output < cv_pi_control.output_limit_min)
        cv_pi_control.output = cv_pi_control.output_limit_min;
    else if(cv_pi_control.output > cv_pi_control.output_limit_max)
        cv_pi_control.output = cv_pi_control.output_limit_max;

    //Calculate next PI integrator state value when enable = true or
    //Reset PI integrator state value when enable = false
    cv_pi_control.integrator_state += cv_pi_control.Ki * cv_pi_control.error *  cv_pi_control.dT;
   } else {
    cv_pi_control.integrator_state = 0.0;
   }
   

   //Current Control algorithm:
   if (cc_pi_control.enable){
    //Set current_reference value
    if (cv_pi_control.enable)
        current_reference = cv_pi_control.output;
    else
        current_reference = CURRENT_REFERENCE;

    //Calculate error
    cc_pi_control.error = current_reference - current_feedback;

    //Calculate PI output
    cc_pi_control.output = cc_pi_control.integrator_state + cc_pi_control.Kp * cc_pi_control.error;
    
    //Limit PI output if necessary
    if(cc_pi_control.output < cc_pi_control.output_limit_min)
        cc_pi_control.output = cc_pi_control.output_limit_min;
    else if(cc_pi_control.output > cc_pi_control.output_limit_max)
        cc_pi_control.output = cc_pi_control.output_limit_max;

    //Calculate next PI integrator state value when enable = true or
    //Reset PI integrator state value when enable = false
    cc_pi_control.integrator_state += cc_pi_control.Ki * cc_pi_control.error *  cc_pi_control.dT;
   } else {
    cc_pi_control.enable = 0.0;
   }

    //Assume that assume the PI output from Current control algorithm is used to calculate the PWM duty cycle of the MCU
    //As in the proposed control block diagram
    duty_feedback = duty_reference - (voltage_feedback/charger_voltage);
    duty_PWM = cc_pi_control.output + duty_feedback;

    //Generate PWM with duty cycle = duty_PWM, 0.0 <= duty_PWM <= 1.0
    //with Generate_PWM()
    Generate_PWM(duty_PWM);
}

void main_state_machine(void){
    /*
    * run the state transition here
    * Charging state for Constant-Current Constant-Voltage (CCCV) of charger:
    *   - state = 1 for Constant-Current State
    *   - state = 2 for Constant-Voltage State
    *   - default   for Idle State
    */

   enable_command = Digital_read(GPIO_input, 1);        //assume that enable_command value from GPIO_input 1

    switch(state){
        case 1:
        control_routine();
        //State transition condition
        if (enable_command == false){
            state = 0;
            cv_pi_control.enable = false;
            cc_pi_control.enable = false;
            Generate_PWM(0.0);
        }
        if(enable_command == true && voltage_feedback >= voltage_reference){
            state = 2;
            cv_pi_control.enable = false;
            cc_pi_control.enable = true;
        }
    break;

        case 2:
        control_routine();
        if (enable_command == false || current_feedback == current_minimum){
            state = 0;
            cv_pi_control.enable = false;
            cc_pi_control.enable = false;
            Generate_PWM(0.0);
        }
        if(enable_command == true && voltage_feedback < voltage_reference){
            state = 1;
            cv_pi_control.enable = false;
            cc_pi_control.enable = true;
        }
    break;

        default:
        if (enable_command == true && voltage_feedback < voltage_reference){
            state = 1;
            cv_pi_control.enable = false;
            cc_pi_control.enable = true;
        }
        else if (enable_command == true && voltage_feedback >= voltage_reference)
            state = 2;
            cv_pi_control.enable = false;
            cc_pi_control.enable = true;
    break;
    }
}

void main(void){
    Initialization();
    PieVectTable.EPWM1_INT = &control_routine;
    while(true){
        main_state_machine();
    }
}