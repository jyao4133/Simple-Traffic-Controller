#include <stdio.h>
#include <stdlib.h>
#include "sys/alt_alarm.h"
#include <system.h>
#include <altera_avalon_pio_regs.h>

// #Defines
#define LIGHT_TRANSITION_TIME 1000
#define CAMERA_TIMEOUT 2000
#define INTERSECTION_TIMEOUT 1
#define NEW_TIMEOUT_LENGTH 40
#define NUMBER_OF_TIMEOUT_VALUES 6

// ENUMS
enum OpperationMode {Mode1 = 1, Mode2 = 2, Mode3 = 3, Mode4 = 4};

// Function declarations
void UpdateMode(enum OpperationMode *currentMode);
void lcd_set_mode(enum OpperationMode currentMode);
void simple_tlc();
void pedestrian_tlc(void);
void init_buttons_pio(void* context);
void NSEW_ped_isr(void* context, alt_u32 id);
void configurable_tlc(enum OpperationMode *currentMode);
void timeout_data_handler(enum OpperationMode *currentMode);
void ResetAllStates(void);
int InSafeState (void);
int ParseNewTimeout(char *New_Timeout, int New_Timeout_Index);
void nextState(enum OpperationMode *currentMode);
void camera_tlc(enum OpperationMode *currentMode);
void handle_vehicle_button(enum OpperationMode *currentMode);
void takeSnapshot(void);
void handle_intersection_timer();
// ISR's
alt_u32 camera_timer_isr(void* context, alt_u32 id);
alt_u32 in_intersection_timer_isr(void* context, alt_u32 id);
alt_u32 tlc_timer_isr(void* context);


// Global variables
volatile alt_alarm timer; //Timer for main logic
volatile alt_alarm CameraTimer; // Timer for timer timeout.
volatile alt_alarm TimerInIntersection; // Keep track of how long a car was in intersection.

// ISR Flags
volatile int camera_has_started = 0;
volatile int even_button = 0;
volatile int timer_has_started = 0;
volatile int time_in_intersection = 0;

volatile int CurrentState = 0; // Current state for fsm.
volatile int timer_running = 0; // To prevent starting / scoping a timer twice.

// Global timeoutt values.
volatile int t0 = 500;
volatile int t1 = 6000;
volatile int t2 = 2000;
volatile int t3 = 500;
volatile int t4 = 6000;
volatile int t5 = 2000;
volatile int currentTimeOut = 6000;
// Pedestrian flags
volatile int EW_Ped = 0;
volatile int NS_Ped = 0;
volatile char New_Timeout[NEW_TIMEOUT_LENGTH];
// Uart
volatile FILE* fp;
volatile char letter;
volatile int recieve_new_data = 0; // Indicates the status of switch 17. (Indicates receiving new timeout values).


int main() {
	// Setup and start peripherals.
	enum OpperationMode currentMode = Mode1;
	lcd_set_mode(currentMode); // Display starting mode.
	void* CurrentModeContex = (void*) &currentMode;
	alt_alarm_start(&timer, currentTimeOut, tlc_timer_isr, CurrentModeContex); //Start the main loop timer
	timer_running = 1;
	init_buttons_pio(CurrentModeContex);
	fp = fopen(UART_NAME, "r+");
	ResetAllStates();
	int New_Timeout_Index = 0;
	int valid_new_timeout = 0;

	while(1){
	// Block until a new value is received via UART. This will get interrupted by the main loop timer to update states.
		letter = fgetc(fp);
		if (recieve_new_data == 1) {
			if (!valid_new_timeout){ //Keep receiving timeout updates until a valid sequence is received.
				//Keep retrieving new values until the \n or \r value is received. Then try to parse this input.
				if(letter != '\r' && letter != '\n' && letter != '\n\r'){
					if (New_Timeout_Index >= NEW_TIMEOUT_LENGTH){ // Prevent index out of bounds.
						New_Timeout_Index = 0;
						memset(New_Timeout, 0, sizeof(New_Timeout));
						break; //Got too many characters without a \n to be valid.
					}

					New_Timeout[New_Timeout_Index] = letter;
					New_Timeout_Index++;
					fprintf(fp, "New input: %s\n\r", New_Timeout);

				} else {
					// A \n or \r was received. Attempt to parse the input.
					printf("input to parse: %s\n\r", New_Timeout);
					valid_new_timeout = ParseNewTimeout(&New_Timeout, New_Timeout_Index);
					if (!valid_new_timeout){
						fprintf(fp, "Invalid input\n\r");
					}
					// Reset the timeout buffer.
					New_Timeout_Index = 0;
					memset(New_Timeout, 0, sizeof(New_Timeout));
				}
			} else {
				// Has received a valid input and updated global timeout values.
				//printf("Received new values. Restarting the timer\n");
				fprintf(fp, "Received. Unblocking\n\r");
				timeout_data_handler(&currentMode);
				if (recieve_new_data == 0){ // If switch 17 has been toggled low, Restart the timer (stop blocking)
					alt_alarm_start(&timer, currentTimeOut, tlc_timer_isr, CurrentModeContex);
					timer_running = 1;
				}
				else {
					printf("Switch still high, receiving new timeouts");
					valid_new_timeout = 0;
				}
			}
		}
	}
	return 0;
}

alt_u32 tlc_timer_isr(void* context) {
	// Main loop timer isr handler. This will check the current mode and call the appropriate handler.
	enum OpperationMode *currentMode = (unsigned int*) context;
	UpdateMode(currentMode);
	timeout_data_handler(currentMode);
	// Call tick function, then update the current state to next state.
	switch ((*currentMode)) {
	case Mode1:
		simple_tlc();
		nextState(currentMode);
		break;
	case Mode2:
		pedestrian_tlc();
		nextState(currentMode);
		break;
	case Mode3:
		configurable_tlc(currentMode);
		nextState(currentMode);
		break;
	case Mode4:
		camera_tlc(currentMode);
		nextState(currentMode);
		break;
	}

	return currentTimeOut;
}




void ResetAllStates(void){

	// Reset the two red pedestrian lights
	int current_red_led = IORD_ALTERA_AVALON_PIO_DATA(LEDS_RED_BASE);
	current_red_led = current_red_led & (~(1<<0)) & (~(1<<1));
	IOWR_ALTERA_AVALON_PIO_DATA(LEDS_RED_BASE, current_red_led);
	// Reset both the green pedestrian lights.
	int current_green_led = IORD_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE);
	current_green_led = current_green_led & (~(1<<6)) & (~(1<<7));
	IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, current_green_led);

	//Reset the button press flags.
	EW_Ped = 0;
	NS_Ped = 0;

}

void UpdateMode(enum OpperationMode *currentMode){

	if (InSafeState()) { //Only change mode when in a safe state.
		// Check which mode switch is asserted and update the current mode.
		// If the mode has changes since last time, update the lcd. (This will stop the lcd from flickering).
		unsigned int modeSwitchValue = IORD_ALTERA_AVALON_PIO_DATA(SWITCHES_BASE);
		if ((modeSwitchValue & 1<<0)) {
			if (*currentMode != Mode1){
				lcd_set_mode(Mode1);
				ResetAllStates();
			}
			(*currentMode) = Mode1;

		} else if ((modeSwitchValue & 1<<1)) {
			if (*currentMode != Mode2){
				lcd_set_mode(Mode2);
				ResetAllStates();
			}
			(*currentMode) = Mode2;
		} else if ((modeSwitchValue & 1<<2)) {
			if (*currentMode != Mode3){
				lcd_set_mode(Mode3);
				ResetAllStates();
			}
			(*currentMode) = Mode3;
		} else if ((modeSwitchValue & 1<<3)) {
			if (*currentMode != Mode4){
				lcd_set_mode(Mode4);
				ResetAllStates();
			}
			(*currentMode) = Mode4;
		}
	}
	return;
}

int InSafeState (void) {
	// Check if the fsm is in a red-red state.
	if (CurrentState == 0 || CurrentState == 3){
		return 1;
	} else {
		return 0;
	}
}

void lcd_set_mode(enum OpperationMode currentMode) {
	// Clear then write the current mode to the lcd.
	#define ESC 27
	#define CLEAR_LCD_STRING "[2J"
	FILE *lcd;
	lcd = fopen(LCD_NAME, "w");
	if(lcd != NULL){
		fprintf(lcd, "%c%s", ESC, CLEAR_LCD_STRING);
		fprintf(lcd, "MODE: %d\n", currentMode);
	}
	return;
}

void simple_tlc() {
	// Update the traffic light leds bused on the current state.
	switch ((CurrentState)) {
	case 0:
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, 0b00100100); //NS:Red EW:Red (safe state)
		currentTimeOut = t0;
		break;
	case 1:
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, 0b00001100); //NS:Green EW:Red
		currentTimeOut = t1;
		break;
	case 2:
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, 0b00010100); //NS:Yellow EW:Red
		currentTimeOut = t2;
		break;
	case 3:
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, 0b00100100); //NS:Red EW:Red (safe state)
		currentTimeOut = t3;
		break;
	case 4:
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, 0b00100001); //NS:Red EW:Green
		currentTimeOut = t4;
		break;
	case 5:
		IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, 0b00100010); //NS:Red EW:Yellow
		currentTimeOut = t5;
		break;
	}
}

void init_buttons_pio(void* context) {
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(KEYS_BASE, 0); // enable interrupts for buttons
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(KEYS_BASE, 0x7); // enable interrupts for all buttons.
	alt_irq_register(KEYS_IRQ,context, NSEW_ped_isr);
}

void nextState(enum OpperationMode *currentMode){
	// If receiving new data, stay in safe state.
	if (InSafeState() && (IORD_ALTERA_AVALON_PIO_DATA(SWITCHES_BASE) & (1<<17)) && (((*currentMode) == Mode3) || (*currentMode) == Mode4)){

	}else {
		// Proceed to next state.
		CurrentState++;
		CurrentState = CurrentState%6;
	}
}
void pedestrian_tlc(void) {
	//Mode 2. Implements the same logic as mode 1, adding pedestrian logic.
	int current_red_led = IORD_ALTERA_AVALON_PIO_DATA(LEDS_RED_BASE);
	switch ((CurrentState)) {
		case 0:
			IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, 0b00100100); //NS:Red EW:Red (safe state)
			currentTimeOut = t0;
			break;
		case 1:
			current_red_led = current_red_led & ~(1<<1);
			IOWR_ALTERA_AVALON_PIO_DATA(LEDS_RED_BASE, current_red_led);
			if (NS_Ped == 1){
				IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, 0b10001100); //NS:Green EW:Red
			}else {
				IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, 0b00001100); //NS:Green EW:Red
			}
			currentTimeOut = t1;
			even_button = 0;
			break;
		case 2:

			if (NS_Ped == 1){
				IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, 0b10010100); //NS:Yellow EW:Red

				NS_Ped = 0;
			} else {
				IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, 0b00010100); //NS:Yellow EW:Red
			}
			currentTimeOut = t2;
			break;
		case 3:
			IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, 0b00100100); //NS:Red EW:Red (safe state)
			currentTimeOut = t3;
			break;
		case 4:
			current_red_led = current_red_led & ~(1<<0);
			IOWR_ALTERA_AVALON_PIO_DATA(LEDS_RED_BASE, current_red_led);

			if (EW_Ped == 1) {
				IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, 0b01100001); //NS:Red EW:Green
			}else{
				IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, 0b00100001); //NS:Red EW:Green
			}
			currentTimeOut = t4;
			even_button = 0;
			break;
		case 5:
			if (EW_Ped == 1) {
				IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, 0b01100010); //NS:Red EW:Yellow
				EW_Ped = 0;
			} else {
				IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, 0b00100010); //NS:Red EW:Yellow
			}
			currentTimeOut = t5;
			break;
		}
}



void NSEW_ped_isr(void* context, alt_u32 id) {
	// ISR to handel pedestrian and car enter intersection buttons being pressed.
	unsigned int buttonValue = IORD_ALTERA_AVALON_PIO_DATA(KEYS_BASE);
	int current_red_led = IORD_ALTERA_AVALON_PIO_DATA(LEDS_RED_BASE);
	enum OpperationMode *currentMode = (unsigned int*) context;

	//Only use the buttons in mode 2,3,4
	if (*currentMode == Mode1) {
		return;
	}

	if (!(buttonValue & 1<<0)) { // If EW button has been pressed.
		// Only accept pedestrian button when condition matches x,R
		if (!(CurrentState == 4 || CurrentState == 5)) {
			// Toggle button press flag and assert the red led to indicate the pedestrian should wait.
			EW_Ped = 1;
			current_red_led = current_red_led | 0b01;
			IOWR_ALTERA_AVALON_PIO_DATA(LEDS_RED_BASE, current_red_led);

		}

	} else if (!(buttonValue & 1<<1)) {
		// Only accept pedestrian button when condition matches R,x
		if (!(CurrentState == 1 || CurrentState == 2)) {
			// Toggle button press flag and assert the red led to indicate the pedestrian should wait.
			NS_Ped = 1;
			current_red_led = current_red_led | 0b10;
			IOWR_ALTERA_AVALON_PIO_DATA(LEDS_RED_BASE, current_red_led);
		}
	} else if (!(buttonValue & 1<<2) && *currentMode == Mode4) {
		// Car enter intersection button pressed. Call corresponding handler.
		handle_vehicle_button(currentMode);
	}
	// Clear the edge capture.
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(KEYS_BASE, 0);
}

void configurable_tlc(enum OpperationMode *currentMode){
	// Mode 3
	pedestrian_tlc(); // Call mode 2. The additional functionality is handled with interrupts.
}

void camera_tlc(enum OpperationMode *currentMode){
	// Mode 4
	pedestrian_tlc(); // Call mode 3. The additional functionality is handled with interrupts.
}

void handle_vehicle_button(enum OpperationMode *currentMode){
	even_button = 1 - even_button; //Toggle the even button press flag.

	if (CurrentState == 0 || CurrentState == 3){ // Orange-Red or Red-Orange state.
		//Start camera timer
		if (even_button == 1){ //check if button is even (Enter intersection)
			if(camera_has_started == 0){
				alt_alarm_start(&CameraTimer, CAMERA_TIMEOUT, camera_timer_isr, (void*) currentMode); //Start the camera timer
				camera_has_started = 1;
				// Start the timer to check how long the car was in the intersection
				alt_alarm_start(&TimerInIntersection, INTERSECTION_TIMEOUT, in_intersection_timer_isr, (void*) currentMode);
				fprintf(fp,"Camera activated \n\r");
			}
		} else if(camera_has_started == 1){ // Car leaving intersection.
			// Stop the camera timers and display how long the car was in the intersection.
			alt_alarm_stop(&CameraTimer);
			alt_alarm_stop(&TimerInIntersection);
			fprintf(fp,"Vehicle left after %d milliseconds \n\r", time_in_intersection);
			time_in_intersection = 0;
			camera_has_started = 0;
		}
	} else if (CurrentState == 1 || CurrentState == 4){ // Red-Red state.
		if (even_button == 1){
			takeSnapshot();
			alt_alarm_stop(&TimerInIntersection);
		} else if(camera_has_started == 1){
			// The car entered in orange-red / red-orange and left in red-red.
			alt_alarm_stop(&CameraTimer);
			alt_alarm_stop(&TimerInIntersection);
			fprintf(fp,"Vehicle left after %d milliseconds \n\r", time_in_intersection);
			time_in_intersection = 0;
			camera_has_started = 0;
		}
	} else {
		// Ignore button press in Green-Red, Red-Green state.
		even_button = 0;
	}
}

void handle_intersection_timer(){
	// Only start the timer if it hasn't alreadyy been started.
	if (camera_has_started == 0){
		alt_alarm_stop(&TimerInIntersection);
		time_in_intersection = 0;
	}
}

alt_u32 camera_timer_isr(void* context, alt_u32 id){
	// Camera timer has expired, stop the timer and take a snapshot.
	enum OpperationMode *currentMode = (unsigned int*) context;
	camera_has_started = 0;
	even_button = 0;
	takeSnapshot();
	return 0;
}

alt_u32 in_intersection_timer_isr(void* context, alt_u32 id){
	// Count the number of 1ms overflows to keep track of how long the car has been in intersection.
	enum OpperationMode *currentMode = (unsigned int*) context;
	time_in_intersection++;
	handle_intersection_timer();
	return 1;

}

void takeSnapshot(void){
	// Indicate a snapshot has been taken.
	fprintf(fp,"Snapshot taken \n\r");
}

void timeout_data_handler(enum OpperationMode *currentMode){
	//If the traffic lights are in a safe state, in mode 3,4 and switch 17 is asserted, pole uart for new timeout values.
	if ((*currentMode == 3 || *currentMode == 4)) { //Mode 3 or 4 only.
		if (InSafeState()) { //Only update time values when in a safe state. (Red Red)
			unsigned int modeSwitchValue = IORD_ALTERA_AVALON_PIO_DATA(SWITCHES_BASE);
			if ((modeSwitchValue & 1<<17)) { // Check if switch 17 is asserted high (Indicating new timeout values).
				recieve_new_data = 1;
				if (timer_running == 1){
					printf("stopped the timer and expecting new values.");
					alt_alarm_stop(&timer);
					timer_running = 0;
				}
			} else {
				recieve_new_data = 0;
				//printf("Restart the timer lul \n.");
			}
		}
	}
}

int ParseNewTimeout(char *New_Timeout, int New_Timeout_Index){
	//Parse the input to retrieve the new timeout values.
	int TempValues[NUMBER_OF_TIMEOUT_VALUES];
	char *token;
	int numberOfTokens = 0;
	//Get the first token from received string (separated by commas).
	token = strtok(New_Timeout, ",");

	// Go through all tokens in the string.
	while(token != NULL) {
		if (numberOfTokens >= NUMBER_OF_TIMEOUT_VALUES){ //If there are more than 6 tokens, this is not valid.
			printf("Returned from too many tokens: %d.\n\r", numberOfTokens);
			return 0;
		}
		int temp = atoi(token); //Convert the string to integer. Note we are treating digits followed by characters as valid input.
		if (temp <= 0 || temp >= 9999) { //Values are only valid if they are 1-4 digits. atoi will return 0 for non numbers.
			printf("Timout value is not in valid range: %d.\n", temp);
			return 0;
		}
		TempValues[numberOfTokens] = temp; //Store valid values into a buffer.
		numberOfTokens++;
		printf( "%d,\t", temp );
		token = strtok(NULL, ","); // Get the next token.
	}
	printf("Finished passing all the inputs.\n");
	printf("Number of tokens = %d\n", numberOfTokens);
	if (numberOfTokens == NUMBER_OF_TIMEOUT_VALUES) { //There are 6 valid numbers received. Update global times.
		printf("Updating globals.\n");
		fprintf(fp, "Updating timout values.\n\r");
		t0 = TempValues[0];
		t1 = TempValues[1];
		t2 = TempValues[2];
		t3 = TempValues[3];
		t4 = TempValues[4];
		t5 = TempValues[5];
	}
	else {
		return 0;
	}
	
	return 1;
}
