#include <stdio.h>
#include <stdlib.h>
#include "sys/alt_alarm.h"
#include <system.h>
#include <altera_avalon_pio_regs.h>

// #Defines
#define LIGHT_TRANSITION_TIME 1000
#define NEW_TIMEOUT_LENGTH 40
#define NUMBER_OF_TIMEOUT_VALUES 6
// ENUMS
enum OpperationMode {Mode1 = 1, Mode2 = 2, Mode3 = 3, Mode4 = 4};
enum LightColour {Red = 1, Yellow = 2, Green = 3};

// Function declarations
void UpdateMode(enum OpperationMode *currentMode);
alt_u32 tlc_timer_isr(void* context);
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



// Global variables
volatile alt_alarm timer;
volatile int timer_has_started = 0;
// Mode variables
volatile int CurrentState = 0;
// Different timers
volatile int t0 = 500;
volatile int t1 = 6000;
volatile int t2 = 2000;
volatile int t3 = 500;
volatile int t4 = 6000;
volatile int t5 = 2000;
volatile int currentTimeOut = 0;
// Pedestrian flags
volatile int EW_Ped = 0;
volatile int NS_Ped = 0;
// Uart
volatile FILE* fp;

alt_u32 tlc_timer_isr(void* context) {
	enum OpperationMode *currentMode = (unsigned int*) context;
	UpdateMode(currentMode);

	switch ((*currentMode)) {
	case Mode1:
		simple_tlc();
		break;
	case Mode2:
		pedestrian_tlc();
		break;
	case Mode3:
		configurable_tlc(currentMode);
		break;
	case Mode4:
		break;
	}

	return currentTimeOut;
}



int main() {
	printf("Compilation Complete\n");
	enum OpperationMode currentMode = Mode1;
	lcd_set_mode(currentMode);
	void* CurrentModeContex = (void*) &currentMode;
	alt_alarm_start(&timer, LIGHT_TRANSITION_TIME, tlc_timer_isr, CurrentModeContex); //Start the timer
	init_buttons_pio(CurrentModeContex);
	fp = fopen(UART_NAME, "r+");
	ResetAllStates();
	printf("Got past uart open.\n");

	//printf("currentMode %d\n", *currentMode);

	while(1){
	}
	return 0;
}

void ResetAllStates(void){
	//CurrentState = 0;

	// Reset all the pedestrian lights
	int current_red_led = IORD_ALTERA_AVALON_PIO_DATA(LEDS_RED_BASE);
	current_red_led = current_red_led & (~(1<<0)) & (~(1<<1));
	IOWR_ALTERA_AVALON_PIO_DATA(LEDS_RED_BASE, current_red_led);
	int current_green_led = IORD_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE);
	current_green_led = current_green_led & (~(1<<6)) & (~(1<<7));
	IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, current_green_led);

	//Reset the button press state.
	EW_Ped = 0;
	NS_Ped = 0;

}

void UpdateMode(enum OpperationMode *currentMode){

	if (InSafeState()) { //Only change mode when in a safe state.
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
	if (CurrentState == 0 || CurrentState == 3){
		return 1;
	} else {
		return 0;
	}
}

void lcd_set_mode(enum OpperationMode currentMode) {
	// Write the current mode to the lcd
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
	CurrentState++;
	CurrentState = CurrentState%6;
}

void init_buttons_pio(void* context) {
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(KEYS_BASE, 0); // enable interrupts for buttons
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(KEYS_BASE, 0x3); // enable interrupts for two right buttons.
	alt_irq_register(KEYS_IRQ,context, NSEW_ped_isr);
}

void pedestrian_tlc(void) {
	//Main logic for Mode 2.
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
		CurrentState++;
		CurrentState = CurrentState%6;
}



void NSEW_ped_isr(void* context, alt_u32 id) {
	unsigned int buttonValue = IORD_ALTERA_AVALON_PIO_DATA(KEYS_BASE);
	int current_red_led = IORD_ALTERA_AVALON_PIO_DATA(LEDS_RED_BASE);
	enum OpperationMode *currentMode = (unsigned int*) context;

	//Only use the buttons in mode 2+
	if (*currentMode == Mode1) {
		return;
	}

	if (!(buttonValue & 1<<0)) {
		// Only accept pedestrian button when condition matches x,R
		if (!(CurrentState == 4 || CurrentState == 5)) {
			EW_Ped = 1;
			current_red_led = current_red_led | 0b01;
			IOWR_ALTERA_AVALON_PIO_DATA(LEDS_RED_BASE, current_red_led);

		}

	} else if (!(buttonValue & 1<<1)) {
		// Only accept pedestrian button when condition matches R,x
		if (!(CurrentState == 1 || CurrentState == 2)) {
			NS_Ped = 1;
			current_red_led = current_red_led | 0b10;
			IOWR_ALTERA_AVALON_PIO_DATA(LEDS_RED_BASE, current_red_led);
		}
	}
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(KEYS_BASE, 0);
}

void configurable_tlc(enum OpperationMode *currentMode){
	timeout_data_handler(currentMode);
	pedestrian_tlc();
}

void timeout_data_handler(enum OpperationMode *currentMode){
	//If the traffic lights are in a safe state, in mode 3+ and switch 17 is asserted, pole uart for new timeout values.
	if ((*currentMode == 3 || *currentMode == 4)) { //Mode 3 or 4 only.
		if (InSafeState()) { //Only update time values when in a safe state. (Red Red)
			unsigned int modeSwitchValue = IORD_ALTERA_AVALON_PIO_DATA(SWITCHES_BASE);
			if ((modeSwitchValue & 1<<17)) { // Check if switch 17 is asserted high (Indicating new timeout values).
				if (fp != NULL) { //Ensure the serial connection is valid.
					//TODO: Stop the timer.
					char New_Timeout[NEW_TIMEOUT_LENGTH] = "1000,1000,1000,1000,1000,1000,1000\n";
					int New_Timeout_Index = 0;
					int valid_new_timeout = 0;
					while (!valid_new_timeout){ //Keep receiving timeout updates untill a valid sequence is received.
						New_Timeout_Index = 6; //TODO: Set this back to 0 and reset New_Timeout to "".
						char letter = '0';
//						while(letter != 'n'){ //Keep retreiving new values untill the \n value is received.
//							if (New_Timeout_Index >= NEW_TIMEOUT_LENGTH){
//								break; //Got too many characters without a \n to be valid.
//							}
//							letter = fgetc(fp);
//							New_Timeout[New_Timeout_Index] = letter;
//							New_Timeout_Index++;
//							printf("%c", letter);
//						}
						printf("input to parse: %s", New_Timeout);
						valid_new_timeout = ParseNewTimeout(&New_Timeout, New_Timeout_Index);
						// TODO: Parse the new timeout.
						if (!valid_new_timeout){
							printf("Didnt pass parse");
						}
						printf("\n");
					}
					// TODO: start the timer again.
				}
			}
		}
	}
}

int ParseNewTimeout(char *New_Timeout, int New_Timeout_Index){
	//Parse the input to retrieve the new timeout values.
	int TempValues[NUMBER_OF_TIMEOUT_VALUES];
	char *token;
	int numberOfTokens = 0;
	//Get the first token from received string.
	token = strtok(New_Timeout, ",");

	// Go through all tokens in the string.
	while(token != NULL) {
		if (numberOfTokens >= NUMBER_OF_TIMEOUT_VALUES){ //If there are more than 6 values, this is not valid.
			printf("Returned from too many tokens: %d.\n", numberOfTokens);
			return 0;
		}
		int temp = atoi(token); //Convert the string to integer. Note we are treating digits followed by characters as valid input.
		if (temp <= 0 || temp > 9999) { //Values are only valid if they are 1-4 digits. atoi will return 0 for non numbers.
			printf("Returned from invalid range: %d.\n", temp);
			return 0;
		}
		TempValues[numberOfTokens] = temp; //Store valid values into a buffer.
		numberOfTokens++;
		printf( " %d\n", temp );
		token = strtok(NULL, ",");
	}
	printf("Finished pasing all the inputs.\n");
	printf("Number of tokens = %d\n", numberOfTokens);
	if (numberOfTokens == NUMBER_OF_TIMEOUT_VALUES) { //There are 6 valid numbers received. Update global times.
		printf("Updating globals.\n");

		t0 = TempValues[0];
		t1 = TempValues[1];
		t2 = TempValues[2];
		t3 = TempValues[3];
		t4 = TempValues[4];
		t5 = TempValues[5];
	}
	return 1;
}
