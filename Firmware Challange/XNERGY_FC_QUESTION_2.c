#include<stdbool.h>

//put your definitions here
#define CURRENT_MINIMUM     0.3     //in Ampere
#define CC_PI_KP            1.0
#define CC_PI_KI            100.0
#define CC_PI_LIMIT_MIN
#define CC_PI_LIMIT_MAX
#define CV_PI_KP            1.0
#define CV_PI_KI            100.0
#define CV_PI_LIMIT_MIN
#define CV_PI_LIMIT_MAX
#define SAMPLING_PERIOD     0.001   //INT frequency is 1kHz

bool enable_command;
uint8_t charging_states, network_states;
uint32_t PWM;
double voltage_feedback, voltage_reference;                     //Battery terminal voltage
double current_feedback, current_reference, current_minimum;    //Battery charging current
double charger_voltage;                                         //Charger DC voltage
double duty_feedback, duty_reference, duty_PWM;

uint32_t prev_time_heartbeat, prev_time_bms_rx, prev_time_bms_tx;

//PI Control Struct
typedef struct {
    bool enable;
    double Kp, Ki;
    double error;
    double output;
    double output_limit_min, output_limit_max;
    double integrator_state;
    double dT;
} PI_CONTROL;

//CAN struct example
typedef struct {
    uint8_t Data[8];
    uint16_t Length;
    uint32_t ID;
} CAN_msg_typedef;

PI_CONTROL cc_pi_control;
PI_CONTROL cv_pi_control;

CAN_msg_typedef Can_tx;
CAN_msg_typedef Can_rx;

void CAN_write(CAN_msg_typedef *msg);
bool CAN_read(CAN_msg_typedef *msg);    //return true if there is received msg

uint32_t time_ms;
void Initialization(void){
    /*
    * initialize variables here
    */
    charging_states = 0;
    network_states = 0;
    enable_command = 0;

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

    network_management();
    prev_time_heartbeat = 0;
    prev_time_bms_rx = 0;
    prev_time_bms_tx = 0;
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

    time_ms++;      //assume INT frequency is 1kHz, for timing purpose
}

void main_state_machine(void){
    /*
    * run the state transition here
    * Charging state for Constant-Current Constant-Voltage (CCCV) of charger:
    *   - charging state = 1 for Constant-Current State
    *   - charging state = 2 for Constant-Voltage State
    *   - default   for Idle State
    */

    switch(charging_states){
        case 1:
        control_routine();
        //State transition condition
        if (enable_command == false){
            charging_states = 0;
            cv_pi_control.enable = false;
            cc_pi_control.enable = false;
            Generate_PWM(0.0);
        }
        if(enable_command == true && voltage_feedback >= voltage_reference){
            charging_states = 2;
            cv_pi_control.enable = false;
            cc_pi_control.enable = true;
        }
    break;

        case 2:
        control_routine();
        if (enable_command == false || current_feedback == current_minimum){
            charging_states = 0;
            cv_pi_control.enable = false;
            cc_pi_control.enable = false;
            Generate_PWM(0.0);
        }
        if(enable_command == true && voltage_feedback < voltage_reference){
            charging_states = 1;
            cv_pi_control.enable = false;
            cc_pi_control.enable = true;
        }
    break;

        default:
        if (enable_command == true && voltage_feedback < voltage_reference){
            charging_states = 1;
            cv_pi_control.enable = false;
            cc_pi_control.enable = true;
        }
        else if (enable_command == true && voltage_feedback >= voltage_reference)
            charging_states = 2;
            cv_pi_control.enable = false;
            cc_pi_control.enable = true;
    break;
    }
}

void CAN_write_handler(void){
    /*
    * CAN tx
    */

   if(Can_tx.ID == 0x181){
    //Prepare Can_tx.Data from voltage_feedback, current_feedback, enable_command
    uint16_t voltage_feedback_tx, current_feedback_tx;

    voltage_feedback_tx = (uint16_t) (voltage_feedback * 10);
    Can_tx.Data[0] = (uint8_t) (voltage_feedback_tx >> 8 & 0xFF);
    Can_tx.Data[1] = (uint8_t) (voltage_feedback_tx >> 0 & 0xFF);

    current_feedback_tx = (uint16_t) (current_feedback * 10);
    Can_tx.Data[2] = (uint8_t) (current_feedback_tx >> 8 & 0xFF);
    Can_tx.Data[3] = (uint8_t) (current_feedback_tx >> 0 & 0xFF);
    
    Can_tx.Data[4] = (uint8_t) enable_command;
   }
   CAN_write(&Can_tx);
   }

void CAN_read_handler(void){
    /*
    * CAN rx
    */
   if(Can_rx.ID == 201){
    if (Can_rx.Data[4]) {
        //Prepare Can_rx.Data for voltage_reference, current_reference, enable_command
        uint16_t voltage_reference_rx = 0;
        voltage_reference_rx += Can_rx.Data[1] << 0;
        voltage_reference_rx += Can_rx.Data[0] << 8;

        voltage_reference = ((double) voltage_reference_rx) / 10;

        uint16_t current_reference_rx = 0;
        current_reference_rx += Can_rx.Data[3] << 0;
        current_reference_rx += Can_rx.Data[2] << 8;

        current_reference = ((double) current_reference_rx) / 10;

        enable_command = Can_rx.Data[4];
        network_states = 2;
    } else {
        enable_command = false;
        network_states = 1;
    }
   }
}

void network_management(void){
    /*
    * run the network management here
    * Network states:
    *   0 : Initialization state
    *   1 : Pre-Operational state
    *   2 : Operational state
    * 
    * CAN ID:
    *   0x701 : Charger ID when transmit heartbeat
    *   0x181 : Charger ID when transmit data to BMS
    *   0x201 : BMS ID
    */

   //Initialization state
   if(network_states == 0){
    Can_tx.ID = 0x701;
    Can_tx.Length = 1;
    Can_tx.Data[0] = 0;
    CAN_write_handler();
    network_states = 1;    
    prev_time_heartbeat = time_ms;

    //Pre-Operational state
   } else if (network_states == 1){
    if((time_ms-prev_time_heartbeat) >= 1000){
        Can_tx.ID = 0x701;
        Can_tx.Length = 1;
        Can_tx.Data[0] = 1;

        CAN_write_handler();
        prev_time_heartbeat = time_ms;
    }

    if(CAN_read(&Can_rx) && Can_rx.ID == 0x201 && (time_ms-prev_time_bms_rx) >= 200){
        CAN_read_handler();
        prev_time_bms_rx = time_ms;
    }
   
   //Operational state
   } else if(network_states == 2){
    if((time_ms-prev_time_heartbeat) >= 1000){
        Can_tx.ID = 0x701;
        Can_tx.Length = 1;
        Can_tx.Data[0] = 2;
        CAN_write_handler();
        prev_time_heartbeat = time_ms;
    }

       if(CAN_read(&Can_rx) && Can_rx.ID == 0x201 && (time_ms-prev_time_bms_rx) >= 200 && (time_ms-prev_time_bms_rx < 5000)){
        CAN_read_handler();
        prev_time_bms_rx = time_ms;
    } else if (time_ms-prev_time_bms_rx >= 5000)
        network_states == 1;

       if((time_ms-prev_time_bms_tx) >= 200){
        Can_tx.ID = 0x181;
        Can_tx.Length = 5;
        CAN_write_handler();
    }
   }
}

void main(void){
    Initialization();
    PieVectTable.EPWM1_INT = &control_routine;
    while(true){
        main_state_machine();
        network_management();
    }
}