/*

I found this very useful as i forget sintax quite often:
https://www.w3schools.com/cpp/default.asp
Francesco

Notes:

Classes and Objects
for now, objects used to call each function are created right after each class.
Meaning, that functions are called normally withing the class, and using the object outside the class.
For example:
I wanna call the function log.
- within the "User_Interface" class I can call it using log(int, string, string);
- outside the "User_Interface" class I have to use the object "user_interface", so it becomes: user_interface.log(int, string, string);
*/

// VARIABLE DEFINITION AND DECLARATION

#include <P1AM.h>
#include <SD.h>
#include <ArduinoSTL.h>
#include <iostream>
#include <map>

//  --------------- GLOBAL ---------------
int vacuum_toggle_button_previous_state = 0; // the only toggle button in the interface, monitor it's state through machine loops to avoid perpetual switching
// Convention: pneumatic states indicate the condition of motion based on limit switch readings. We define cylinders as: 1=extended, 0=somwhere in between, -1=retracted. Vaccum: 1=on, 0=off
// "sense" class functions only read the limit switches, "update" class functions change the value of the following states based on the values returned by "sense" class functions
bool vacuum_state = 0;
int gantry_state = 0;
int suction_cup_state = 0;
unsigned long current_millis; // used to periodically print an "idle" state in the main void loop(), to limit the output printed on the serial monitor
unsigned long previous_millis;
const float dutyCycle = 50.15; // stepper motor duty cycle
const int frequency = 4000;    // stepper motor PWM frequency
// Modes of operation
const int manual_mode = 1;
const int step_mode = 2;
const int single_cycle_mode = 3;
const int continuous_cycle_mode = 4;
int mode_state = manual_mode; // what mode are we operating in? goes by level of automation, 1=manual, 2=step mode, 3=single cycle, 4=continuous cycle. for testing, set to manual for now
// SD CARD
String log_file_name = "logs.txt";
// LOGGING
const String log_setting = "debug"; // the detail of logged events. 1=debugging, 2=warning, 3=error.
using log_map_type = const std::map<String, int>;
log_map_type log_map = { // describe the severity level of debugging messages, to filter them in logs.txt
    {"debug", 1},
    {"warning", 2},
    {"error", 3}};

// --------------- P1AM Module Slots ---------------
const int discrete_input_slot_one = 1;
const int discrete_input_slot_two = 2;
const int discrete_output_slot = 3;
const int stepper_controller_slot = 4;
const char stepper_config[] = {0x06, 0x02, 0x06, 0x02}; // stepper_controller_slot configuration
char cfgData[] = {};
const int relay_slot = 5;

// Discrete Input ONE 16ND3 - slot 1
// const int home_button_channel = 1;
// const int calibrate_button_channel = 2;
// const int continuous_cycle_selector_channel = 3;
// const int single_cycle_selector_channel = 4;
// const int step_mode_selector_channel = 5;
// const int manual_mode_selector_channel = 6;
// const int run_button_channel = 7;
// const int cycle_stop_button_channel = 8;
// const int e_stop_button_channel = 9;
// const int set_index_button_channel = 10;
// const int to_index_button_channel = 11;
// const int to_bottom_button_channel = 12;
// const int jog_table_up_button_channel = 13;
// const int jog_table_down_button_channel = 14;
// const int gantry_extend_button_channel = 15;
// const int gantry_retract_button_channel = 16;

// Discrete Input TWO 16ND3 - slot 2
// const int suction_cup_extend_button_channel = 1;
// const int suction_cup_retract_button_channel = 2;
// const int vacuum_toggle_button_channel = 3;
// Pneumatic Limit Switches
const int suction_cup_retract_right_pneumatic_limit = 4;
const int suction_cup_retract_left_pneumatic_limit = 5;
const int gantry_retract_left_pneumatic_limit = 6;
const int gantry_retract_right_pneumatic_limit = 7;
const int gantry_extend_left_pneumatic_limit = 8;
const int gantry_extend_right_pneumatic_limit = 9;
// Mechanical Limit Switches
const int table_limit_switch_top_right = 10;
const int table_limit_switch_top_left = 11;
const int table_limit_switch_bottom_right = 12;
const int table_limit_switch_bottom_left = 13;
const int head_limit_switch_right = 14;
const int head_limit_switch_left = 15;

// Discrete Output Module (slot 3)
// const int led_home_button_channel = 1;
// const int led_calibrate_button_channel = 2;
// const int led_run_button_channel = 3;
// const int led_cycle_stop_button_channel = 4;
// const int led_set_index_button_channel = 5;
// const int led_to_index_button_channel = 6;
// const int led_jog_direction_button_channel = 7;
// const int led_to_bottom_button_channel = 8;
// const int led_gantry_extend_button_channel = 9;
// const int led_gantry_retract_button_channel = 10;
// const int led_suction_cup_extend_button_channel = 11;
// const int led_suction_cup_retract_button_channel = 12;
// const int led_vacuum_toggle_button_channel = 13;

// Steppers - P1-04PWM (slot 4)
const int right_stepper_dir_channel = 1;
const int right_stepper_pwm_channel = 2;
const int left_stepper_dir_channel = 3;
const int left_stepper_pwm_channel = 4;

// PNEUMATICS - RELAY MODULE (slot 5)
const int air_enable_channel = 1;
const int gantry_retract_channel = 2;
const int gantry_extend_channel = 3;
const int suction_cups_retract_channel = 4;
const int suction_cups_extend_channel = 5;
const int vacuum_channel = 6;

// --------------- Buttons ---------------
struct button_struct
{
    uint8_t id;          // button unique global identifier
    uint8_t slot;        // button slot
    uint8_t channel;     // button channel
    uint8_t led_slot;    // slot of the corresponding led
    uint8_t led_channel; // channel of the corresponding led
};

bool button_state[19] = {0};

constexpr button_struct button[19] = {
    {1, discrete_input_slot_one, 1, discrete_output_slot, 1},    // home
    {2, discrete_input_slot_one, 2, discrete_output_slot, 2},    // calibrate
    {3, discrete_input_slot_one, 3, 0, 0},                       // continuous cycle
    {4, discrete_input_slot_one, 4, 0, 0},                       // single cycle
    {5, discrete_input_slot_one, 5, 0, 0},                       // step mode
    {6, discrete_input_slot_one, 6, 0, 0},                       // manual mode
    {7, discrete_input_slot_one, 7, discrete_output_slot, 3},    // run
    {8, discrete_input_slot_one, 8, discrete_output_slot, 4},    // cycle stop
    {9, discrete_input_slot_one, 9, 0, 0},                       // e stop
    {10, discrete_input_slot_one, 10, discrete_output_slot, 5},  // set index
    {11, discrete_input_slot_one, 11, discrete_output_slot, 6},  // to index
    {12, discrete_input_slot_one, 12, discrete_output_slot, 8},  // to bottom
    {13, discrete_input_slot_one, 13, discrete_output_slot, 7},  // table up
    {14, discrete_input_slot_one, 14, discrete_output_slot, 7},  // table down
    {15, discrete_input_slot_one, 15, discrete_output_slot, 9},  // gantry extend
    {16, discrete_input_slot_one, 16, discrete_output_slot, 10}, // gantry retract
    {17, discrete_input_slot_two, 1, discrete_output_slot, 11},  // suction cup down
    {18, discrete_input_slot_two, 2, discrete_output_slot, 12},  // suction cup up
    {19, discrete_input_slot_two, 3, discrete_output_slot, 13}}; // vacuum

using ButtonMapType = std::map<String, int>;
ButtonMapType button_map = {
    {"home", 0},
    {"calibrate", 1},
    {"continuous cycle", 2},
    {"single cycle", 3},
    {"step mode", 4},
    {"manual mode", 5},
    {"run", 6},
    {"cycle stop", 7},
    {"e stop", 8},
    {"set index", 9},
    {"to index", 10},
    {"to bottom", 11},
    {"table up", 12},
    {"table down", 13},
    {"gantry extend", 14},
    {"gantry retract", 15},
    {"suction cups down", 16},
    {"suction cups up", 17},
    {"vacuum", 18}};

// CLASS DECLARATION

class User_Interface
{
public:
    void log(String log_layer, String log_location, String log_text);
    void confirm_motion_success_button_led(String button_request);
    void control_button_led(String button_request, bool state);
    void overwrite_read_button();
    void read_button();
    bool request_button_press(String button_request);
};
User_Interface user_interface;

class Motion
{
public:
    // for the following table functions we will be using the PI-04PWM module
    // some useful docs https://facts-engineering.github.io/modules/P1-04PWM/P1-04PWM.html
    bool table_up();
    bool table_down();
    void table_stop();
    bool table_to_index();
    bool table_to_bottom();
    bool gantry_extend();
    bool gantry_retract();
    bool gantry_home();
    bool suctioncups_extend();
    void suctioncups_retract();
    void vacuum_on();
    void vacuum_off();
    void activate_air();
};
Motion motion;

class Initialize
{
public:
    bool startup();
    bool SD_card();
};
Initialize initialize;

class Mode
{
public:
    void manual();
    // for step mode, use the separate class "Step"
};
Mode mode;

// class Step
// {
// public:
//     void index();
//     void pickup();
//     void place();
//     void gantry_return();
// };
// Step step;

class Sense
{
public:
    void update_global_states();
    bool protect_with_limit_switch(int limit_switch_array[], int payloadsize);
    bool protect_gantry_head(int limit_switch);
    bool enable_suction_cups();
    bool enable_gantry();
};
Sense sense;

// FUNCTION DEFINITION
void setup()
{
    Serial.begin(9600); // start serial interface
    // for testing only, wait for user to start machine
    // String command = "";
    // bool start = 0;
    // while (!start)
    // {
    //     Serial.println("Type 'start' please...");
    //     command = Serial.readStringUntil('\n'); // read string until meet newline character
    //     if (command == "start")
    //         start = 1;
    // }

    while (!initialize.SD_card())
    {
        ; // wait for the SD card to be initialized
    }

    // Initializing modules
    while (!P1.init())
    {
        ; // wait for the modules to come online
    }
    if (!P1.configureModule(stepper_config, stepper_controller_slot))
    {
        Serial.println("Cannot configure stepper module"); // send dir, pwm, dir, pwm configuration to stepper module
        while (1)
            ; // halt the machine
    }

    // executing a safe startup
    while (!initialize.startup())
    {
        user_interface.log("error", "setup()", "The safe startup failed. Press RUN to attempt another startup...");
        user_interface.request_button_press("run");
    }

    // PINMODE to be done at the end of development
}

void loop()
{
    // what mode are we in?
    user_interface.read_button(); // accept input from serial console
    mode_state = manual_mode;
    switch (mode_state)
    {
    case manual_mode:
        // Serial.println("entering manual mode");
        mode.manual();
        // turn off all manual buttons apart from set index, to bottom and to index
        break;
    case step_mode:
        /*
        if RUN is pressed
            // activate_air()
            switch(target_step)
                case 1
                    mode.step.index();
                case 2
                    mode.step.pickup();
                case 3
                    mode.step.place();
                case 4
                    mode.step.gantry_return();
                default:
                    // ERROR invalid step requested
        */
        break;
    case single_cycle_mode:
        // if RUN is pressed:
        // activate_air()
        // mode.single_cycle();
        break;
    case continuous_cycle_mode:
        // if RUN is pressed:
        // activate_air()
        // mode.continuous_cycle();
        break;
    default:
        Serial.println("Idle, no modes selected");
    }
    current_millis = millis();
    if (current_millis - previous_millis > 5000)
    {
        Serial.println("Idle");
        previous_millis = current_millis;
    }
}

void User_Interface::log(String log_layer, String log_location, String log_text)
{
    /*
    Logging modes:
    Logging mode is set with the global variable log_setting.
    - Debugging (=1) in this mode all messages are desplayed, starting from the debugging layer up to error layer
    - Warnings(=2) warnings and errors are displayed
    - Error (=3) only errors and unknown types of log layers
    - Unknown (=4) user requested a type of error that does not fall under the above ones. We set type to unknown
    */

    /*
 - validate
 - compose
 - write
    */

    // creating variables
    String log_string = "";
    String log_layer_text = "";
    int log_layer_number = 0;
    bool layer_found = 0;

    // validate incoming data, and compose
    for (std::pair<String, int> element : log_map)
    {
        if (log_layer == element.first)
        {
            log_layer_text = log_layer;
            log_layer_number = element.second;
            break;
        }
        else
        {
            log_layer_text = "Unknown";
            log_layer_number = 4; // output an unclassified error type regardless of log_setting
        }
    }

    // write message based on the log setting
    if (log_layer_number >= log_map.find(log_setting)->second)
    {
        // compose the message
        log_string = log_layer_text + " :: " + log_location + " | " + log_text;

        // opening log file
        File logFile = SD.open(log_file_name, FILE_WRITE);

        // if the file is available, write to it:
        if (logFile)
        {
            logFile.println(log_string);
            logFile.close();
            // print to the serial port
            Serial.println(log_string);
            return;
        }
        else
        {
            // file failed to open, using simpler serial output to notify the user
            Serial.println("error opening: " + log_file_name);
            return;
        }
    }
    else
    {
        return; // log layer is lower then what the user specified in "log_setting". Skip writing and return to calling method.
    }
}

void User_Interface::confirm_motion_success_button_led(String button_request)
{
    user_interface.log("debug", "user_interface.confirm_motion_success_button_led", "confirming motion success using button '" + button_request + "'");
    uint8_t blinks = 0;
    user_interface.control_button_led(button_request, false);
    while (blinks < 3) // make 3 blinks
    {
        delay(100);
        user_interface.control_button_led(button_request, true);
        delay(100);
        user_interface.control_button_led(button_request, false);
        blinks++;
    }
    user_interface.control_button_led(button_request, false); // ensure button is off
    user_interface.log("debug", "user_interface.confirm_motion_success_button_led", "successfully blinked button '" + button_request + "'");
}

void User_Interface::control_button_led(String button_request, bool state)
{
    P1.writeDiscrete(state, button[button_map.find(button_request)->second].led_slot, button[button_map.find(button_request)->second].led_channel);
    // user_interface.log("debug", "user_interface.control_button_led", "setting ' " + button_request + "' to " + String(state));
}

void User_Interface::overwrite_read_button()
{
    /*
    Allow coders to simulate button presses from the command interface, to test functions remotely without being at the machine and pressing buttons
    */
    if (Serial.available()) // if there is data comming
    {
        String command = Serial.readStringUntil('\n'); // read string until meet newline character

        for (std::pair<String, int> element : button_map)
        {
            // searching for a match in button map according to what the user requested
            if (command == element.first)
            {
                button_state[element.second] = true; // overwrite global state array, to simulate a button press
                break;                               // exit for loop
            }
            else
            {
                ;
            }
        }
    }
}

void User_Interface::read_button()
{
    /*
    Allow us coders to update the global state of a button, to inform actions and operational modes throughout the codebase
    Goal of this function:
    - minimize lookups to the input modules as they are expensive to execute
    - ensure function works even if builders decide to change pinout during development as needed. Assume builders will change global button attributes accordingly
    */
    int mask = 0;
    bool answer = 0;
    int i = 0;
    int all_states = 0;
    // read all input modules once and assign their binary output to a place holder variable
    int all_states_slot_one = P1.readDiscrete(discrete_input_slot_one);
    int all_states_slot_two = P1.readDiscrete(discrete_input_slot_two);

    // READING BUTTON MAP
    for (std::pair<String, int> element : button_map)
    {
        // evaluating in which slot the button is plugged in and setting all_states
        switch (button[element.second].slot)
        {
        case discrete_input_slot_one:
            all_states = all_states_slot_one;
            break;
        case discrete_input_slot_two:
            all_states = all_states_slot_two;
            break;
        }

        // searching for a true value in all_states of buttons based on channel position and updating global state array
        i = (button[element.second].channel - 1); // based on the button channel, create a position variable "i"
        mask = 1 << i;                            // used to find true bytes in all_states. shifting byte "1" by "i" positions according to the button channel
        answer = (all_states & mask) >> i;        // is byte of all_states at position i equal to 1? yes --> answer=1
        button_state[element.second] = answer;    // insert answer into global state array
    }
    user_interface.overwrite_read_button(); // for testing only, allow coder to simulate button press
}

bool User_Interface::request_button_press(String button_request)
{
    /*
    allow programmers to request a button press to the user, by leveraging the button structure, that incorporates
    LED slot and channel. Protect programmers from accidentally authorize actions via an always on button.
    Goals:
    ensure that programmers cannot request a press from a button, that is unable to do so (does not have an led).
    if programmers request such button, ensure the function does not return to the calling method, as this can be very dangerous:
    Programmer requests --> "manual mode selector"
    (this function) --> detects as active (selector is always ON)
    Machine --> procedes to authorize action, DANGER!
    */

    // validate if the button is eligible for a user request (ie. does it have an led?)
    if (button[button_map.find(button_request)->second].led_channel == 0)
    {
        user_interface.log("error", "user_interface.request_button_press", "button '" + button_request + "' is not eligible for user request. Halting the machine...");
        while (true)
        {
            ; // programmer requested a dangerous action. halt the machine
        }
    }

    // blink the button and listen for a press
    unsigned long button_previous_time = 0;
    bool led_state = 0;
    while (!button_state[button_map.find(button_request)->second])
    {
        // blink button
        if (millis() - button_previous_time > 750)
        {
            led_state = !led_state; // flip the state of the LED
            user_interface.control_button_led(button_request, led_state);
            button_previous_time = millis();
        }
        user_interface.read_button();
    }
    P1.writeDiscrete(LOW, button[button_map.find(button_request)->second].led_slot, button[button_map.find(button_request)->second].led_channel); // ensure button is off before proceding
    user_interface.read_button();
    return 1;
}

bool Motion::table_up()
{
    user_interface.control_button_led("table up", true);
    int limit_switch_array[4] = {head_limit_switch_right, head_limit_switch_left, table_limit_switch_top_right, table_limit_switch_top_left};
    if (!sense.protect_with_limit_switch(limit_switch_array, 4))
    {
        // set stepper direction for both steppers
        P1.writePWMDir(LOW, stepper_controller_slot, right_stepper_dir_channel);
        P1.writePWMDir(LOW, stepper_controller_slot, left_stepper_dir_channel);
        // send movement to control module
        P1.writePWM(dutyCycle, frequency, stepper_controller_slot, right_stepper_pwm_channel);
        P1.writePWM(dutyCycle, frequency, stepper_controller_slot, left_stepper_pwm_channel);
        user_interface.log("debug", "motion", "table: move UP");
        return 1;
    }
    else
    {
        return 0;
    }
}

bool Motion::table_down()
{
    user_interface.control_button_led("table down", true);
    int limit_switch_array[2] = {table_limit_switch_bottom_right, table_limit_switch_bottom_left};
    if (!sense.protect_with_limit_switch(limit_switch_array, 2))
    {
        // set stepper direction for both steppers
        P1.writePWMDir(HIGH, stepper_controller_slot, right_stepper_dir_channel);
        P1.writePWMDir(HIGH, stepper_controller_slot, left_stepper_dir_channel);
        // send movement to control module
        P1.writePWM(dutyCycle, frequency, stepper_controller_slot, right_stepper_pwm_channel); // slot 1 channel 2
        P1.writePWM(dutyCycle, frequency, stepper_controller_slot, left_stepper_pwm_channel);  // slot 1 channel 2
        user_interface.log("debug", "motion", "table: move DOWN");
        return 1;
    }
    else
    {
        return 0;
    }
}

void Motion::table_stop()
{
    user_interface.control_button_led("table up", false);
    P1.writePWM(0, 0, stepper_controller_slot, right_stepper_pwm_channel);   // slot 1 channel 2
    P1.writePWM(0, 0, stepper_controller_slot, left_stepper_pwm_channel);    // slot 1 channel 2
    P1.writePWMDir(LOW, stepper_controller_slot, right_stepper_dir_channel); // turn off direction pin
    P1.writePWMDir(LOW, stepper_controller_slot, left_stepper_dir_channel);  // turn off direction pin
    // user_interface.log("debug", "motion", "table: STOP");
    return;
}

bool Motion::table_to_index()
{
    /*
    solidly light the to_index led
    display "table going to index"
    sed the table to the top
            in a loop:
                check if limitswitch or stop is pressed or if radar has reached the set distance
                if true --> exit the loop
    turn off to_index led
    RETURN
    */
    user_interface.log("debug", "motion", "table: travelling to index");
}

bool Motion::table_to_bottom()
{
    /*
    solidly light the to_bottom led
    display "table going to bottom"
    send the table to the bottom
            in a loop:
                check if limitswitch or stop is pressed
                if true --> exit the loop
    turn off to_bottom led
    RETURN
    */
    user_interface.log("debug", "motion", "table: travelling to bottom");
}

bool Motion::gantry_extend()
{
    if (sense.enable_gantry())
    {
        P1.writeDiscrete(0, relay_slot, gantry_retract_channel);
        P1.writeDiscrete(1, relay_slot, gantry_extend_channel);
        user_interface.log("debug", "motion", "gantry: EXTEND");
        return 1;
    }
    else
    {
        user_interface.log("warning", "motion.gantry_extend", "cannot extend the gantry, ensure that suction cups are UP!");
        return 0;
    }
}

bool Motion::gantry_retract()
{
    if (sense.enable_gantry())
    {
        P1.writeDiscrete(0, relay_slot, gantry_extend_channel);
        P1.writeDiscrete(1, relay_slot, gantry_retract_channel);
        user_interface.log("debug", "motion", "gantry: RETRACT");
        return 1;
    }
    else
    {
        user_interface.log("warning", "motion.gantry_retract", "cannot retract the gantry, ensure that suction cups are UP!");
        return 0;
    }
}

bool Motion::gantry_home()
{
    motion.vacuum_off();
    delay(500);

    // First ensure suction cups are up
    motion.suctioncups_retract();
    int i = 0;
    while (suction_cup_state != -1)
    {
        delay(100);
        i++;
        sense.update_global_states();
        // suction_cup_state = -1; // for testing only
        if (i >= 40) // if motion time exedes 4s, abort motion as something has gone wrong
        {
            user_interface.log("error", "motion.gantry_home", "cannot detect suction cups as retracted. Cannot complete the homing");
            return 0; // something serious has gone wrong, cannot complete the homing
        }
    }

    // Then Move the gantry
    motion.gantry_retract();
    i = 0;
    while (gantry_state != -1)
    {
        delay(100);
        i++;
        sense.update_global_states();
        // gantry_state = -1; // for testing only
        if (i >= 40) // if motion time exedes 4s, abort motion as something has gone wrong
        {
            user_interface.log("error", "motion.gantry_home", "cannot detect gantry as retracted. Cannot complete the homing");
            return 0; // something serious has gone wrong, cannot complete the homing
        }
    }
    user_interface.log("debug", "motion.gantry_home", "gantry successfully homed");
    return 1;
}

bool Motion::suctioncups_extend()
{
    if (sense.enable_suction_cups())
    {
        P1.writeDiscrete(0, relay_slot, suction_cups_retract_channel);
        P1.writeDiscrete(1, relay_slot, suction_cups_extend_channel);
        user_interface.log("debug", "motion", "suction cups: EXTEND");
        return 1;
    }
    else
    {
        user_interface.log("warning", "motion.suctioncups_extend", "cannot extend suction cups, gantry may be moving!");
        return 0;
    }
}

void Motion::suctioncups_retract()
{
    P1.writeDiscrete(0, relay_slot, suction_cups_extend_channel);
    P1.writeDiscrete(1, relay_slot, suction_cups_retract_channel);
    user_interface.log("debug", "motion", "suction cups: RETRACT");
    return;
}

void Motion::vacuum_on()
{
    P1.writeDiscrete(1, relay_slot, vacuum_channel);
    user_interface.log("debug", "motion", "vacuum: ON");
    return;
}

void Motion::vacuum_off()
{
    P1.writeDiscrete(0, relay_slot, vacuum_channel);
    user_interface.log("debug", "motion", "vacuum: OFF");
    return;
}

void Motion::activate_air()
{
    /*
    after a safe startup this function is called to make sure that the air is on,
    */
    P1.writeDiscrete(HIGH, relay_slot, air_enable_channel);
    user_interface.log("debug", "motion.activate_air", "air: ON");
}

bool Initialize::startup()
{
    /*
    we want the user to be safe when turining on the machine, we disable all cylinders and vacuum at start up
    to ensure that no air suddenly rushes in the system unexpectedly moving components
    */

    P1.writeDiscrete(LOW, relay_slot, 0); // turn off all relays
    sense.update_global_states();
    // gantry_state = 1; // overwriting gantry statee to pass following check for testing only
    if (gantry_state == -1 || gantry_state == 1)
    {
        // Setting the initial position of the relays
        switch (gantry_state)
        {
        case -1:
            user_interface.log("debug", "initialize.startup", "Gantry detected as retracted. Setting relay accordingly...");
            P1.writeDiscrete(0, relay_slot, gantry_extend_channel);
            P1.writeDiscrete(1, relay_slot, gantry_retract_channel);
            break;
        case 1:
            user_interface.log("debug", "initialize.startup", "Gantry detected as extended. Setting relay accordingly...");
            P1.writeDiscrete(0, relay_slot, gantry_retract_channel);
            P1.writeDiscrete(1, relay_slot, gantry_extend_channel);
        }
        user_interface.log("debug", "initialize.startup", "Done initializing relays.");

        // according to the selected mode, home the gantry or continue to manual mode
        switch (mode_state)
        {
        case manual_mode:
            user_interface.log("debug", "initialize.startup", "Manual mode selected. Ready for movement? Press RUN to confirm.");
            user_interface.request_button_press("run");
            user_interface.log("debug", "initialize.startup", "button: 'run' pressed. Turning on the air...");
            motion.vacuum_off();
            motion.activate_air();
            user_interface.log("debug", "initialize.startup", "safe startup executed. Continuing...");
            return 1;
            break;
        case step_mode:
        case single_cycle_mode:
        case continuous_cycle_mode:
            user_interface.log("debug", "initialize.startup", "Automatic mode selected. Ready for movement? Press RUN to confirm.");
            user_interface.request_button_press("run");
            user_interface.log("debug", "initialize.startup", "button: 'run' pressed. Turning on the air...");
            motion.activate_air();
            if (motion.gantry_home())
            {
                user_interface.log("debug", "initialize.startup", "safe startup executed. Continuing...");
                return 1;
            }
            else
            {
                user_interface.log("debug", "initialize.startup", "The homing of the gantry failed, cannot continue.");
                return 0;
            }
            break;
        default:
            user_interface.log("debug", "initialize.startup", "invalid mode_state selected. cannot continue startup");
            return 0;
        }
    }
    else
    {
        user_interface.log("debug", "initialize.startup", "Please, move the gantry in the retracted or extended position. Press RUN when done...");
        user_interface.request_button_press("run");
        return 0; // a zero in the setup() method, will call this function again, to detect if the gantry is in an acceptable position to start the machine.
    }
}

bool Initialize::SD_card()
{
    // initialize SD card
    if (!SD.begin(SDCARD_SS_PIN))
    {
        Serial.println("setup: card failed, or not present");
        return 0;
    }
    else
    {
        // SD is available, testing writing abilities
        File testFile;
        testFile = SD.open("test.txt", FILE_WRITE);
        if (testFile)
        {
            Serial.print("Writing to test.txt...");
            testFile.println("test");
            testFile.close();
            Serial.println("done.");
            user_interface.log("debug", "initialize.SD_card: ", "SD card correctly initialized");
            return 1;
        }
        else
        {
            Serial.println("Error opening test file. Cannot write to SD card");
            return 0;
        }
    }
}

void Mode::manual()
{
    /*
    Allow coders to allow a certain set of actions in manual mode, shaping the way the machine responds to user requests.
    Goals:
    - in manual mode, detect only certain buttons that call respective motions or functions.
    - allow the machine to exit this function if nothing is pressed, returning to the main "void loop()", as there, we check for a change in mode of operation
    */

    // update state of buttons
    user_interface.read_button();

    // call action according to the pressed button
    if (button_state[button_map.find("home")->second])
    {
    }
    else if (button_state[button_map.find("calibrate")->second])
    {
    }
    else if (button_state[button_map.find("run")->second])
    {
    }
    else if (button_state[button_map.find("cycle stop")->second])
    {
    }
    else if (button_state[button_map.find("set index")->second])
    {
    }
    else if (button_state[button_map.find("to index")->second])
    {
    }
    else if (button_state[button_map.find("to bottom")->second])
    {
    }
    else if (button_state[button_map.find("table up")->second])
    {
        motion.table_up();
    }
    else if (button_state[button_map.find("table down")->second])
    {
        motion.table_down();
    }
    else if (button_state[button_map.find("gantry extend")->second])
    {
        motion.gantry_extend();
    }
    else if (button_state[button_map.find("gantry retract")->second])
    {
        motion.gantry_retract();
    }
    else if (button_state[button_map.find("suction cups down")->second])
    {
        motion.suctioncups_extend();
    }
    else if (button_state[button_map.find("suction cups up")->second])
    {
        motion.suctioncups_retract();
    }
    else if (button_state[button_map.find("vacuum")->second])
    {
        if (vacuum_toggle_button_previous_state == 0)
        {
            switch (vacuum_state)
            {
            case 0:
                motion.vacuum_on();
                break;
            case 1:
                motion.vacuum_off();
                break;
            default:
                user_interface.log("error", "mode.manual()", "vacuum_state is invalid, must be type bool");
            }
            vacuum_state = !vacuum_state;
        }
        vacuum_toggle_button_previous_state = 1; // set button state, to avoid
    }
    else
    {
        vacuum_toggle_button_previous_state = 0; // reset vacuum button state if it is not pressed
        motion.table_stop();                     // make sure to stop the table in case jog buttons are not pressed
    }
    return;
    user_interface.read_button();
}

void Sense::update_global_states()
{
    /*
    Allow the machine to update global variables that indicate the current status of the cylinders (retracted/extended)
    */

    if (P1.readDiscrete(discrete_input_slot_two, suction_cup_retract_right_pneumatic_limit) && P1.readDiscrete(discrete_input_slot_two, suction_cup_retract_left_pneumatic_limit))
    {
        user_interface.log("debug", "sense.update_global_states", "suction cups detected as retracted");
        suction_cup_state = -1; // we are currently running 1 limitswitch per cylinder on the suction cup movement, when two are added, a secondary check for extended will be added
    }
    else if (P1.readDiscrete(discrete_input_slot_two, gantry_extend_right_pneumatic_limit) && P1.readDiscrete(discrete_input_slot_two, gantry_extend_left_pneumatic_limit))
    {
        user_interface.log("debug", "sense.update_global_states", "gantry detected as extended");
        gantry_state = 1;
    }
    else if (P1.readDiscrete(discrete_input_slot_two, gantry_retract_left_pneumatic_limit) && P1.readDiscrete(discrete_input_slot_two, gantry_retract_right_pneumatic_limit))
    {
        user_interface.log("debug", "sense.update_global_states", "gantry detected as retracted");
        gantry_state = -1;
    }
    else
    {
        suction_cup_state = 0; // if the suction cup does not pass above checks, it has to be in mid-motion, so we set it to zero
        gantry_state = 0;      // if the gantry does not pass above checks, it has to be in mid-motion, so we set it to zero
        user_interface.log("debug", "sense.update_global_states", "cylinder detected at mid-motion");
    }
    return;
}

bool Sense::protect_with_limit_switch(int limit_switch_array[], int payloadsize)
{
    int not_safe = 1;
    String limit_switch = "";
    String message_modifier = "cannot move table, limit has been reached!";
    for (int i = 0; i < payloadsize; i++)
    {
        switch (limit_switch_array[i])
        {
        case table_limit_switch_bottom_left:
            limit_switch = "table_limit_switch_bottom_left";
            break;
        case table_limit_switch_bottom_right:
            limit_switch = "table_limit_switch_bottom_right";
            break;
        case table_limit_switch_top_left:
            limit_switch = "table_limit_switch_top_left";
            break;
        case table_limit_switch_top_right:
            limit_switch = "table_limit_switch_top_right";
            break;
        case head_limit_switch_right:
            limit_switch = "head_limit_switch_right";
            message_modifier = "cannot move table, material reached gantry head.";
            break;
        case head_limit_switch_left:
            limit_switch = "head_limit_switch_left";
            message_modifier = "cannot move table, material reached gantry head.";
            break;
        }
        user_interface.log("debug", "sense.protect_with_limit_switch", "checking limit switch: " + limit_switch);
        if (P1.readDiscrete(discrete_input_slot_two, limit_switch_array[i]))
        {
            motion.table_stop();
            user_interface.log("debug", "sense.protect_with_limit_switch", limit_switch + " has been triggered." + message_modifier);
            not_safe = 1;
            break; // do not check next limitswitch, we already know one has been triggered, not safe to move
        }
        else
        {
            user_interface.log("debug", "sense.protect_with_limit_switch", limit_switch + " not triggered");
            not_safe = 0;
        }
    }
    return not_safe;
}

bool Sense::enable_suction_cups()
{
    bool gantry_extended = (P1.readDiscrete(discrete_input_slot_two, gantry_extend_right_pneumatic_limit) && P1.readDiscrete(discrete_input_slot_two, gantry_extend_left_pneumatic_limit));
    bool gantry_retracted = (P1.readDiscrete(discrete_input_slot_two, gantry_retract_right_pneumatic_limit) && P1.readDiscrete(discrete_input_slot_two, gantry_retract_left_pneumatic_limit));
    return (gantry_extended || gantry_retracted);
}

bool Sense::enable_gantry()
{
    return (P1.readDiscrete(discrete_input_slot_two, suction_cup_retract_right_pneumatic_limit) && P1.readDiscrete(discrete_input_slot_two, suction_cup_retract_left_pneumatic_limit));
}
