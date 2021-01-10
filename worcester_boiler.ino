
#define PIN_NRESET		4
#define	PIN_DATA		5
#define	PIN_CTRL		6
//#define	PIN_TXRDY		

enum {
	STATE_IDLE,
	STATE_TXSTART,
	STATE_TXSTART_WAIT,
	STATE_TX_PREAMBLE,
	STATE_TX_PREWAIT,
	STATE_TX_STARTBIT_A,
	STATE_TX_STARTBIT_B,
	STATE_TX_BIT_A,
	STATE_TX_BIT_B,
	STATE_TX_PARITY_A,
	STATE_TX_PARITY_B,
	STATE_TX_STOP_A,
	STATE_TX_STOP_B,
	STATE_TX_STOP_C,
	STATE_TX_STOP_D,
	STATE_TX_POWEROFF
};


#define RADIO_INVERT_TX


volatile unsigned char isrState = STATE_IDLE;
volatile unsigned char isrTxBuf[13];

// 16MHz clock, 256:1 prescaler, 2500Hz (400us) interrupt rate
const uint16_t TIMER_PERIOD = 16000000ul / 64 / 2500;

#ifdef RADIO_INVERT_TX
# define RADIO_DATA(x) digitalWrite(PIN_DATA, x);
#else
# define RADIO_DATA(x) digitalWrite(PIN_DATA, x?0:1);
#endif

/**
 * TIMER1 Interrupt handler
 * 
 * Implements the radio 
 */
ISR(TIMER1_COMPA_vect)
{
	static uint8_t isrBytePos;		// byte counter
	static uint8_t isrCounter;		// bit (and general purpose) counter
	static uint8_t parity;			// parity accumulator
	static uint8_t prebit;			// current preamble bit
	static uint8_t shift;			// byte transmit shift register

	switch (isrState) {
		case STATE_IDLE:			// Idle, nothing to do
			return;

		case STATE_TXSTART:			// Warm up
			isrBytePos = 0;					// start transmitting from byte 0
			isrCounter = (4000 / 400);		// 4ms warmup time, 400us interrupt rate
			digitalWrite(PIN_DATA, 1);		// Key the radio
			isrState = STATE_TXSTART_WAIT;	// proceed to wait for TXSTART expiry
			break;

		case STATE_TXSTART_WAIT:	// Wait for warmup
			isrCounter--;
			if (isrCounter == 0) {
				// Send 40 cycles of preamble (80 inversions)
				isrCounter = 88;
				isrState = STATE_TX_PREAMBLE;
				prebit = 0;
				OCR1A = (TIMER_PERIOD / 2)-1;	// 200us period for the fast preamble
			}
			break;

		case STATE_TX_PREAMBLE:		// Transmit Preamble -- 50% duty square wave
			RADIO_DATA(prebit);
			prebit = ~prebit & 1;
			isrCounter--;
			if (isrCounter == 0) {
				// Next a 1.6ms delay
				isrCounter = (1400 / 200)+1;
				isrState = STATE_TX_PREWAIT;
			}
			break;

		case STATE_TX_PREWAIT:		// Post-preamble wait
			RADIO_DATA(0);
			isrCounter--;
			if (isrCounter == 0) {
				isrState = STATE_TX_STARTBIT_A;
				OCR1A = TIMER_PERIOD-1;		// back to 400us period for the startbit
			}
			break;

		case STATE_TX_STARTBIT_A:	// First half of start bit
			RADIO_DATA(0);
			isrState = STATE_TX_STARTBIT_B;
			break;

		case STATE_TX_STARTBIT_B:	// Second half of start bit
			RADIO_DATA(1);

			// Prepare to transmit 8 bits
			parity = 0;
			isrCounter = 8;
			shift = isrTxBuf[isrBytePos];
			
			isrState = STATE_TX_BIT_A;
			break;

		case STATE_TX_BIT_A:		// Transmit first half-bit of a bit
			// Transmit current bit, first half-bit
			RADIO_DATA(shift & 1);
			// Update parity
			if (shift & 1) {
				parity++;
			}
			isrState = STATE_TX_BIT_B;
			break;

		case STATE_TX_BIT_B:		// Transmit first half-bit of a bit
			// Transmit current bit, second half-bit
			RADIO_DATA(~shift & 1);
			
			// Now advance to the next bit
			isrCounter--;
			if (isrCounter == 0) {
				// All 8 bits transmitted. Next send the parity bit
				/*
				// Pairing packets have inverted Parity within the data area
				if ((isrTxBuf[7] & 0x80) && ((isrBytePos >= 7) && (isrBytePos <= 10))) {
					parity++;
				}

				// State packets have inverted Parity for the ID section
				if (!(isrTxBuf[7] & 0x80) && ((isrBytePos >= 4) && (isrBytePos <= 6))) {
					parity++;
				}

				// State packets have inverted Parity for heating demand, if heating is being demanded
				if ((isrTxBuf[8] == 0xFF) && (isrBytePos == 8)) {
					parity++;
				}
				*/
				isrState = STATE_TX_PARITY_A;
			} else {
				// More bits to transmit -- bitshift, and continue
				shift >>= 1;
				isrState = STATE_TX_BIT_A;
			}
			break;

		case STATE_TX_PARITY_A:		// Transmit first half of parity bit
			// Transmit parity bit
			if (parity & 1) {
				RADIO_DATA(0);
			} else {
				RADIO_DATA(1);
			}
			isrState = STATE_TX_PARITY_B;
			break;

		case STATE_TX_PARITY_B:		// Transmit second half of parity bit
			if (parity & 1) {
				RADIO_DATA(1);
			} else {
				RADIO_DATA(0);
			}
			isrState = STATE_TX_STOP_A;
			break;

		case STATE_TX_STOP_A:		// Transmit first part of first stop bit
			RADIO_DATA(1);
			isrState = STATE_TX_STOP_B;
			break;
			
		case STATE_TX_STOP_B:		// Transmit first part of first stop bit
			RADIO_DATA(0);
			isrState = STATE_TX_STOP_C;
			break;
			
		case STATE_TX_STOP_C:		// Transmit first part of first stop bit
			RADIO_DATA(1);
			isrState = STATE_TX_STOP_D;
			break;
			
		case STATE_TX_STOP_D:		// Transmit first part of first stop bit
			RADIO_DATA(0);

			// One complete byte transmitted. Move onto the next byte.
			isrBytePos++;
			if (isrBytePos > 12) {
				// Transmitted all the bytes. Turn the radio off and go to idle
				isrState = STATE_TX_POWEROFF;
			} else {
				// More bytes to transmit
				parity = 0;
				isrCounter = 8;
				isrState = STATE_TX_STARTBIT_A;
			}
			break;

		case STATE_TX_POWEROFF:
			// TODO: Add 20ms poweroff delay.
			digitalWrite(PIN_DATA, 0);
			isrState = STATE_IDLE;
	}

}


// send RECOVER command to the RFM68
void rfm_recover(void)
{
	// This is Pulsed Recovery, to reduce the risk of going into TX mode during recovery
	for (uint8_t clks=0; clks<24; clks++){
		digitalWrite(PIN_DATA, HIGH);
		digitalWrite(PIN_CTRL, HIGH);
		digitalWrite(PIN_DATA, LOW);
		digitalWrite(PIN_CTRL, LOW);
	}
}

// send WRITE APPLICATION BITS command to the RFM68
void rfm_write_appbits(const uint16_t value)
{
	// first byte = 0 (write application bits)
	digitalWrite(PIN_DATA, LOW);
	for (uint8_t clks=0; clks<8; clks++){
		digitalWrite(PIN_CTRL, HIGH);
		digitalWrite(PIN_CTRL, LOW);
	}

	// parameters
	uint16_t shift = value;
	for (uint8_t clks=0; clks<16; clks++){
		digitalWrite(PIN_DATA, shift & 0x8000 ? 1 : 0);
		shift <<= 1;
		digitalWrite(PIN_CTRL, HIGH);
		digitalWrite(PIN_CTRL, LOW);
	}
}

#define READ_APPBITS 0x33
#define READ_VERSION 0x55

uint16_t rfm_read(const uint8_t cmd)
{
	uint8_t shift = cmd;
	for (uint8_t clks=0; clks<8; clks++){
		digitalWrite(PIN_CTRL, LOW);
		digitalWrite(PIN_DATA, shift & 0x80 ? 1 : 0);
		shift <<= 1;
		digitalWrite(PIN_CTRL, HIGH);
	}
	
	pinMode(PIN_DATA, INPUT);
	uint16_t val = 0;
	for (uint8_t clks=0; clks<16; clks++){
		digitalWrite(PIN_CTRL, LOW);
		val <<= 1;
		if (digitalRead(PIN_DATA)) {
			val |= 1;
		}
		digitalWrite(PIN_CTRL, HIGH);
	}
	digitalWrite(PIN_CTRL, LOW);
	pinMode(PIN_DATA, OUTPUT);

	return val;
}


void setup()
{
	// ----- Pin configuration -----
	pinMode(LED_BUILTIN,	OUTPUT);
	pinMode(PIN_NRESET,		OUTPUT);
	pinMode(PIN_DATA,		OUTPUT);
	pinMode(PIN_CTRL,		OUTPUT);

	// Set DATA low to prevent the transmitter from keying on boot
	digitalWrite(PIN_DATA,		LOW);

	// Set CTRL low to preload 868MHz defaults
	digitalWrite(PIN_CTRL,		LOW);


	// ----- Serial port initialisation -----

	Serial.begin(9600); // open the serial port at 9600 bps:


	// ----- Radio module initialisation -----

	// Reset the radio module
	digitalWrite(PIN_NRESET,	LOW);
	delay(10);	// RFM68: at least 100us
	digitalWrite(PIN_NRESET,	HIGH);

	// Wait T_START -- 200us + oscillator startup time
	delay(10);

	// Configure the radio module for a 20ms timeout
	// 
	rfm_recover();
	uint16_t appbits = rfm_read(READ_APPBITS);
	delay(1);
	rfm_write_appbits(appbits | 0x08);	// Sets RF timeout to  = 0x20DC


	// ----- Radio ISR setup -----

	// Have to hold PIN_DATA high at least 2ms to key the transmitter
	// Hold PIN_DATA low for 20ms (configured above -- default is 2ms) to power down the radio

	// initialize timer1 
	noInterrupts();				// disable all interrupts
	TCCR1A = 0;
	TCCR1B = 0;

	// Set up Timer1 for periodic interrupts
	OCR1A = TIMER_PERIOD-1;		// load timer
	TCCR1B |= (1 << CS11) | (1 << CS10);		// 64:1 prescaler 
	TCCR1B |= (1 << WGM12);		// set up for CTC (clear timer on compare) mode
	TIMSK1 |= (1 << OCIE1A);	// enable channel 1A interrupt
	interrupts();				// enable all interrupts
}


/***
 * MT10RF protocol implementation
 */

const uint8_t TXID[3] = {0xC1, 0x83, 0xA9}; //{ 0x12, 0x34, 0x56 };

//uint8_t n = (120/5);	// pair for 120 seconds, 5sec interval
//uint8_t seq = 10;		// sequence number


void sendPacket(bool pairing, bool heat, bool battery_low)
{
	// Packet header -- constant
	isrTxBuf[0]  = 0x68;
	isrTxBuf[1]  = 0x07;
	isrTxBuf[2]  = 0x07;
	isrTxBuf[3]  = 0x68;


	if (pairing) {
		// Pairing packet
		
		// Pairing indicator
		isrTxBuf[4]  = 0;
		isrTxBuf[5]  = 0;
		isrTxBuf[6]  = 0;
		isrTxBuf[7]  = 0x80;

		// Transmitter ID
		isrTxBuf[8]  = TXID[0];
		isrTxBuf[9]  = TXID[1];
		isrTxBuf[10] = TXID[2];
	} else {
		// State packet

		// Transmitter ID
		isrTxBuf[4]  = TXID[0];
		isrTxBuf[5]  = TXID[1];
		isrTxBuf[6]  = TXID[2];

		// Battery status?, 1 or 2
		isrTxBuf[7]  = battery_low ? 2 : 1;

		// Heating demand: 0x00 for off, 0xFF for on
		isrTxBuf[8] = heat ? 0xFF : 0x00;

		// Heating demand: 0x28, else 0x14
		isrTxBuf[9] = heat ? 0x28 : 0x14;

		// Fixed at 0x14
		isrTxBuf[10] = 0x14;
	}

	// Calculate checksum
	isrTxBuf[11] = 0;
	for (uint8_t i=4; i<11; i++) {
		isrTxBuf[11] += isrTxBuf[i];
	}

	// Terminator -- constant
	isrTxBuf[12] = 0x16;


	Serial.print("RFtx> ");
	for (uint8_t i=0; i<13; i++) {
		Serial.print(isrTxBuf[i], HEX);
		Serial.print(" ");
	}

	// Disable timer0 to prevent jitter
	//TIMSK0 &= ~_BV(TOIE0);

	// start transmit
	isrState = STATE_TXSTART;

	// wait for transmit complete
	while (isrState != STATE_IDLE);

	// Re-enable timer0
	//TIMSK0 |= _BV(TOIE0);

	Serial.println("  DONE");
}


void loop() {
	static bool heat   = false;
	static bool loBatt = false;
	
	// put your main code here, to run repeatedly:

	Serial.println("Type 'pair' to send pairing sequence.");
	Serial.println("Type 'batt' to toggle low battery flag.");
	Serial.println("Type 'heat' to toggle heating state.");

	// Read serial command
	while (Serial.available() == 0) {} 
	String s = Serial.readStringUntil('\n');
	// trim whitespace
	s.trim();

	// echo the command line
	Serial.println(s);
	Serial.println();

	if (s.equals("pair")) {
		// --- PAIRING ---
		for (uint8_t i=0; i<5; i++) {
			sendPacket(true, heat, loBatt);
			delay(4250);
		}
		return;	// prevent sending update sequence
	} else if (s.equals("batt")) {
		loBatt = !loBatt;
	} else if (s.equals("heat")) {
		heat = !heat;
	}

	Serial.print("Sequence: ");
	if (loBatt) Serial.print("LOW_BATT ");
	if (heat) Serial.print("HEAT ");
	Serial.println();
	for (uint8_t i=0; i<5; i++) {
		sendPacket(false, heat, loBatt);
		delay(4250);
	}
}
