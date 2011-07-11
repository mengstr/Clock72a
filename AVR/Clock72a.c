#define F_CPU 16000000UL
#define TWI_FREQ 100000L

#include <util/delay.h>
#include <util/delay_basic.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <compat/twi.h>

#define RTCADDR 0x6F


#define BUFFER_LENGTH 32
#define TWI_BUFFER_LENGTH 32

#define TWI_READY 0
#define TWI_MRX   1
#define TWI_MTX   2
#define TWI_SRX   3
#define TWI_STX   4



volatile uint8_t hour=1;
volatile uint8_t minute=11;
volatile uint8_t second=22;

volatile uint8_t buttonA;
volatile uint8_t buttonAcnt=0;
volatile uint8_t buttonB;
volatile uint8_t buttonBcnt=0;


uint8_t rxBuffer[BUFFER_LENGTH];
uint8_t rxBufferIndex = 0;
uint8_t rxBufferLength = 0;

uint8_t txAddress = 0;
uint8_t txBuffer[BUFFER_LENGTH];
uint8_t txBufferIndex = 0;
uint8_t txBufferLength = 0;

uint8_t transmitting = 0;



static volatile uint8_t twi_state;
static uint8_t twi_slarw;

static void (*twi_onSlaveTransmit)(void);
static void (*twi_onSlaveReceive)(uint8_t*, int);

static uint8_t twi_masterBuffer[TWI_BUFFER_LENGTH];
static volatile uint8_t twi_masterBufferIndex;
static uint8_t twi_masterBufferLength;

static uint8_t twi_txBuffer[TWI_BUFFER_LENGTH];
static volatile uint8_t twi_txBufferIndex;
static volatile uint8_t twi_txBufferLength;

static uint8_t twi_rxBuffer[TWI_BUFFER_LENGTH];
static volatile uint8_t twi_rxBufferIndex;

static volatile uint8_t twi_error;



void begin(void);
uint8_t requestFrom(uint8_t address, uint8_t quantity);
void beginTransmission(uint8_t address);
uint8_t endTransmission(void);
void send(uint8_t data);
uint8_t available(void);
void twi_init(void);
void twi_setAddress(uint8_t address);
uint8_t twi_readFrom(uint8_t address, uint8_t* data, uint8_t length);
uint8_t twi_writeTo(uint8_t address, uint8_t* data, uint8_t length, uint8_t wait);
uint8_t twi_transmit(uint8_t* data, uint8_t length);
//void twi_attachSlaveRxEvent( void (*function)(uint8_t*, int) );
//void twi_attachSlaveTxEvent( void (*function)(void) );
void twi_reply(uint8_t ack);
void twi_stop(void);
void twi_releaseBus(void);
uint8_t ReadRTCByte(const uint8_t adr);
void WriteRTCByte(const uint8_t adr, const uint8_t data);
uint8_t DisplayRTCData(const uint8_t adr, const uint8_t validbits);
void AllOff();
void Light(uint8_t led);
void BrightnessTest();
void GetTime();
void ReverseHMS();
void StandardHMS();





// !!!
void begin(void) {
  rxBufferIndex = 0;
  rxBufferLength = 0;
  txBufferIndex = 0;
  txBufferLength = 0;
  twi_init();
}


// !!!
uint8_t requestFrom(uint8_t address, uint8_t quantity) {
  // clamp to buffer length
  if(quantity > BUFFER_LENGTH){
    quantity = BUFFER_LENGTH;
  }
  // perform blocking read into buffer
  uint8_t read = twi_readFrom(address, rxBuffer, quantity);
  // set rx buffer iterator vars
  rxBufferIndex = 0;
  rxBufferLength = read;

  return read;
}


// !!!
void beginTransmission(uint8_t address) {
  // indicate that we are transmitting
  transmitting = 1;
  // set address of targeted slave
  txAddress = address;
  // reset tx buffer iterator vars
  txBufferIndex = 0;
  txBufferLength = 0;
}



// !!!
uint8_t endTransmission(void) {
  // transmit buffer (blocking)
  int8_t ret = twi_writeTo(txAddress, txBuffer, txBufferLength, 1);
  // reset tx buffer iterator vars
  txBufferIndex = 0;
  txBufferLength = 0;
  // indicate that we are done transmitting
  transmitting = 0;
  return ret;
}




// must be called in:
// slave tx event callback
// or after beginTransmission(address)
// !!!
void send(uint8_t data) {
	// don't bother if buffer is full
	if(txBufferLength >= BUFFER_LENGTH){
	  return;
	}
	// put byte in tx buffer
	txBuffer[txBufferIndex] = data;
	++txBufferIndex;
	// update amount in buffer   
	txBufferLength = txBufferIndex;
	}


// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
// !!!
uint8_t available(void) {
  return rxBufferLength - rxBufferIndex;
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
// !!!
uint8_t receive(void) {
  // default to returning null char
  // for people using with char strings
  uint8_t value = '\0';
  
  // get each successive byte on each call
  if(rxBufferIndex < rxBufferLength){
    value = rxBuffer[rxBufferIndex];
    ++rxBufferIndex;
  }

  return value;
}

// ------------------------------------------------







/* 
 * Function twi_init
 * Desc     readys twi pins and sets twi bitrate
 * Input    none
 * Output   none
 */
// !!!
void twi_init(void) {
  // initialize state
  twi_state = TWI_READY;

  #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega328P__)
    // activate internal pull-ups for twi
    // as per note from atmega8 manual pg167
    PORTC |= (1<<4);
    PORTC |= (1<<5);
  #else
    // activate internal pull-ups for twi
    // as per note from atmega128 manual pg204
    PORTD |= (1<<0);
    PORTD |= (1<<1);
  #endif

  // initialize twi prescaler and bit rate
  TWSR&= ~(1<<TWPS0);
  TWSR&= ~(1<<TWPS1);
  TWBR = ((F_CPU / TWI_FREQ) - 16) / 2;

  /* twi bit rate formula from atmega128 manual pg 204
  SCL Frequency = CPU Clock Frequency / (16 + (2 * TWBR))
  note: TWBR should be 10 or higher for master mode
  It is 72 for a 16mhz Wiring board with 100kHz TWI */

  // enable twi module, acks, and twi interrupt
  TWCR = (1<<TWEN) | (1<<TWIE) | (1<<TWEA);
}




/* 
 * Function twi_readFrom
 * Desc     attempts to become twi bus master and read a
 *          series of bytes from a device on the bus
 * Input    address: 7bit i2c device address
 *          data: pointer to byte array
 *          length: number of bytes to read into array
 * Output   number of bytes read
 */
// !!!
uint8_t twi_readFrom(uint8_t address, uint8_t* data, uint8_t length) {
  uint8_t i;

  // ensure data will fit into buffer
  if(TWI_BUFFER_LENGTH < length){
    return 0;
  }

  // wait until twi is ready, become master receiver
  while(TWI_READY != twi_state){
    continue;
  }
  twi_state = TWI_MRX;
  // reset error state (0xFF.. no error occured)
  twi_error = 0xFF;

  // initialize buffer iteration vars
  twi_masterBufferIndex = 0;
  twi_masterBufferLength = length-1;  // This is not intuitive, read on...
  // On receive, the previously configured ACK/NACK setting is transmitted in
  // response to the received byte before the interrupt is signalled. 
  // Therefor we must actually set NACK when the _next_ to last byte is
  // received, causing that NACK to be sent in response to receiving the last
  // expected byte of data.

  // build sla+w, slave device address + w bit
  twi_slarw = TW_READ;
  twi_slarw |= address << 1;

  // send start condition
  TWCR = (1<<TWEN) | (1<<TWIE) | (1<<TWEA) | (1<<TWINT) | (1<<TWSTA);

  // wait for read operation to complete
  while(TWI_MRX == twi_state){
    continue;
  }

  if (twi_masterBufferIndex < length)
    length = twi_masterBufferIndex;

  // copy twi buffer to data
  for(i = 0; i < length; ++i){
    data[i] = twi_masterBuffer[i];
  }
	
  return length;
}

/* 
 * Function twi_writeTo
 * Desc     attempts to become twi bus master and write a
 *          series of bytes to a device on the bus
 * Input    address: 7bit i2c device address
 *          data: pointer to byte array
 *          length: number of bytes in array
 *          wait: boolean indicating to wait for write or not
 * Output   0 .. success
 *          1 .. length to long for buffer
 *          2 .. address send, NACK received
 *          3 .. data send, NACK received
 *          4 .. other twi error (lost bus arbitration, bus error, ..)
 */
// !!!
uint8_t twi_writeTo(uint8_t address, uint8_t* data, uint8_t length, uint8_t wait) {
  uint8_t i;

  // ensure data will fit into buffer
  if(TWI_BUFFER_LENGTH < length){
    return 1;
  }

  // wait until twi is ready, become master transmitter
  while(TWI_READY != twi_state){
    continue;
  }
  twi_state = TWI_MTX;
  // reset error state (0xFF.. no error occured)
  twi_error = 0xFF;

  // initialize buffer iteration vars
  twi_masterBufferIndex = 0;
  twi_masterBufferLength = length;
  
  // copy data to twi buffer
  for(i = 0; i < length; ++i){
    twi_masterBuffer[i] = data[i];
  }
  
  // build sla+w, slave device address + w bit
  twi_slarw = TW_WRITE;
  twi_slarw |= address << 1;
  
  // send start condition
  TWCR = (1<<TWEN) | (1<<TWIE) | (1<<TWEA) | (1<<TWINT) | (1<<TWSTA);

  // wait for write operation to complete
  while(wait && (TWI_MTX == twi_state)){
    continue;
  }
  
  if (twi_error == 0xFF)
    return 0;	// success
  else if (twi_error == TW_MT_SLA_NACK)
    return 2;	// error: address send, nack received
  else if (twi_error == TW_MT_DATA_NACK)
    return 3;	// error: data send, nack received
  else
    return 4;	// other twi error
}

/* 
 * Function twi_reply
 * Desc     sends byte or readys receive line
 * Input    ack: byte indicating to ack or to nack
 * Output   none
 */
// !!!
void twi_reply(uint8_t ack) {
  // transmit master read ready signal, with or without ack
  if(ack){
    TWCR = (1<<TWEN) | (1<<TWIE) | (1<<TWINT) | (1<<TWEA);
  }else{
	  TWCR = (1<<TWEN) | (1<<TWIE) | (1<<TWINT);
  }
}

/* 
 * Function twi_stop
 * Desc     relinquishes bus master status
 * Input    none
 * Output   none
 */
// !!!
void twi_stop(void) {
  // send stop condition
  TWCR = (1<<TWEN) | (1<<TWIE) | (1<<TWEA) | (1<<TWINT) | (1<<TWSTO);

  // wait for stop condition to be exectued on bus
  // TWINT is not set after a stop condition!
  while(TWCR & (1<<TWSTO)){
    continue;
  }

  // update twi state
  twi_state = TWI_READY;
}


/* 
 * Function twi_releaseBus
 * Desc     releases bus control
 * Input    none
 * Output   none
 */
// !!!
void twi_releaseBus(void) {
  // release bus
  TWCR = (1<<TWEN) | (1<<TWIE) | (1<<TWEA) | (1<<TWINT);

  // update twi state
  twi_state = TWI_READY;
}




// !!!
SIGNAL(TWI_vect) {
  switch(TW_STATUS){
    // All Master
    case TW_START:     // sent start condition
    case TW_REP_START: // sent repeated start condition
      // copy device address and r/w bit to output register and ack
      TWDR = twi_slarw;
      twi_reply(1);
      break;

    // Master Transmitter
    case TW_MT_SLA_ACK:  // slave receiver acked address
    case TW_MT_DATA_ACK: // slave receiver acked data
      // if there is data to send, send it, otherwise stop 
      if(twi_masterBufferIndex < twi_masterBufferLength){
        // copy data to output register and ack
        TWDR = twi_masterBuffer[twi_masterBufferIndex++];
        twi_reply(1);
      }else{
        twi_stop();
      }
      break;
    case TW_MT_SLA_NACK:  // address sent, nack received
      twi_error = TW_MT_SLA_NACK;
      twi_stop();
      break;
    case TW_MT_DATA_NACK: // data sent, nack received
      twi_error = TW_MT_DATA_NACK;
      twi_stop();
      break;
    case TW_MT_ARB_LOST: // lost bus arbitration
      twi_error = TW_MT_ARB_LOST;
      twi_releaseBus();
      break;

    // Master Receiver
    case TW_MR_DATA_ACK: // data received, ack sent
      // put byte into buffer
      twi_masterBuffer[twi_masterBufferIndex++] = TWDR;
    case TW_MR_SLA_ACK:  // address sent, ack received
      // ack if more bytes are expected, otherwise nack
      if(twi_masterBufferIndex < twi_masterBufferLength){
        twi_reply(1);
      }else{
        twi_reply(0);
      }
      break;
    case TW_MR_DATA_NACK: // data received, nack sent
      // put final byte into buffer
      twi_masterBuffer[twi_masterBufferIndex++] = TWDR;
    case TW_MR_SLA_NACK: // address sent, nack received
      twi_stop();
      break;
    // TW_MR_ARB_LOST handled by TW_MT_ARB_LOST case

    // Slave Receiver
    case TW_SR_SLA_ACK:   // addressed, returned ack
    case TW_SR_GCALL_ACK: // addressed generally, returned ack
    case TW_SR_ARB_LOST_SLA_ACK:   // lost arbitration, returned ack
    case TW_SR_ARB_LOST_GCALL_ACK: // lost arbitration, returned ack
      // enter slave receiver mode
      twi_state = TWI_SRX;
      // indicate that rx buffer can be overwritten and ack
      twi_rxBufferIndex = 0;
      twi_reply(1);
      break;
    case TW_SR_DATA_ACK:       // data received, returned ack
    case TW_SR_GCALL_DATA_ACK: // data received generally, returned ack
      // if there is still room in the rx buffer
      if(twi_rxBufferIndex < TWI_BUFFER_LENGTH){
        // put byte in buffer and ack
        twi_rxBuffer[twi_rxBufferIndex++] = TWDR;
        twi_reply(1);
      }else{
        // otherwise nack
        twi_reply(0);
      }
      break;
    case TW_SR_STOP: // stop or repeated start condition received
      // put a null char after data if there's room
      if(twi_rxBufferIndex < TWI_BUFFER_LENGTH){
        twi_rxBuffer[twi_rxBufferIndex] = '\0';
      }
      // sends ack and stops interface for clock stretching
      twi_stop();
      // callback to user defined callback
      twi_onSlaveReceive(twi_rxBuffer, twi_rxBufferIndex);
      // since we submit rx buffer to "wire" library, we can reset it
      twi_rxBufferIndex = 0;
      // ack future responses and leave slave receiver state
      twi_releaseBus();
      break;
    case TW_SR_DATA_NACK:       // data received, returned nack
    case TW_SR_GCALL_DATA_NACK: // data received generally, returned nack
      // nack back at master
      twi_reply(0);
      break;
    
    // Slave Transmitter
    case TW_ST_SLA_ACK:          // addressed, returned ack
    case TW_ST_ARB_LOST_SLA_ACK: // arbitration lost, returned ack
      // enter slave transmitter mode
      twi_state = TWI_STX;
      // ready the tx buffer index for iteration
      twi_txBufferIndex = 0;
      // set tx buffer length to be zero, to verify if user changes it
      twi_txBufferLength = 0;
      // request for txBuffer to be filled and length to be set
      // note: user must call twi_transmit(bytes, length) to do this
      twi_onSlaveTransmit();
      // if they didn't change buffer & length, initialize it
      if(0 == twi_txBufferLength){
        twi_txBufferLength = 1;
        twi_txBuffer[0] = 0x00;
      }
      // transmit first byte from buffer, fall
    case TW_ST_DATA_ACK: // byte sent, ack returned
      // copy data to output register
      TWDR = twi_txBuffer[twi_txBufferIndex++];
      // if there is more to send, ack, otherwise nack
      if(twi_txBufferIndex < twi_txBufferLength){
        twi_reply(1);
      }else{
        twi_reply(0);
      }
      break;
    case TW_ST_DATA_NACK: // received nack, we are done 
    case TW_ST_LAST_DATA: // received ack, but we are done already!
      // ack future responses
      twi_reply(1);
      // leave slave receiver state
      twi_state = TWI_READY;
      break;

    // All
    case TW_NO_INFO:   // no state information
      break;
    case TW_BUS_ERROR: // bus error, illegal stop/start
      twi_error = TW_BUS_ERROR;
      twi_stop();
      break;
  }
}



uint8_t ReadRTC(const uint8_t adr){
  uint8_t data=0;
 
  beginTransmission(RTCADDR);
  send(adr);
  endTransmission();
  requestFrom(RTCADDR,1);
  while (available()) data=receive();

  return data;
}


void WriteRTCByte(const uint8_t adr, const uint8_t data){
  beginTransmission(RTCADDR);
  send(adr);
  send(data);
  endTransmission();
} 















void AllOff() {
	PORTD = 0;
	DDRD = 0x00;	// INPUT
	PORTB = 0;
	DDRB = 0x00;	// INPUT
}



void Light(uint8_t led) {
	uint8_t dgnd;
	uint8_t dpos;
	uint8_t dgndbitmask;
	uint8_t dposbitmask;
	uint8_t b1,b2;

	PORTD=0;
	PORTB=0;

	dgnd=led>>3;
	dpos=led%8;
	if (dgnd<=dpos) dpos++;
	dgndbitmask=1<<dgnd;
	dposbitmask=1<<dpos;
	b1=0;
	b2=0;
	if (dposbitmask==0) {b1=1; b2=1;}
	if (dgndbitmask==0) {b1=1; b2=0;}

	

	DDRB=b1;
	DDRD=dgndbitmask+dposbitmask;
	PORTD=dposbitmask;
	PORTB=b2;

}



void BrightnessTest() {
	uint8_t led;
	
	for (;;) {


		AllOff();
		for (int i=0; i<50; i++) {
			for (led=0; led<72; led++) {
				Light(led);
				_delay_ms(1);
			}
		}


		AllOff();
		DDRD=0xFF;
		for (int i=0; i<500; i++) {
			PORTD=0b11111110;
			_delay_ms(1);
			PORTD=0b11111101;
			_delay_ms(1);
			PORTD=0b11111011;
			_delay_ms(1);
			PORTD=0b11110111;
			_delay_ms(1);
			PORTD=0b11101111;
			_delay_ms(1);
			PORTD=0b11011111;
			_delay_ms(1);
			PORTD=0b10111111;
			_delay_ms(1);
			PORTD=0b01111111;
			_delay_ms(1);
		}
	}
}




void GetTime() {
	uint8_t tmp;

    tmp=ReadRTC(0);
	second=(tmp&0x0f)+10*((tmp>>4)&0x07);

    tmp=ReadRTC(1);
	minute=(tmp&0x0f)+10*((tmp>>4)&0x07);

    tmp=ReadRTC(2);
	hour=(tmp&0x0f)+10*((tmp>>4)&0x03);

	second=37;
	minute=0;
	hour=8;
}



#define X 10

void DisplayHMS(uint8_t brightnessH, uint8_t brightnessM, uint8_t brightnessS) {

	Light(60+(hour%12));
	for (int i=0; i<X; i++) 	_delay_loop_2(brightnessH);
	AllOff();
	for (int i=0; i<X; i++) _delay_loop_2(101-brightnessH);
/*
	Light(minute);
	_delay_loop_2(brightnessM);
	AllOff();
	_delay_loop_2(101-brightnessM);
*/
	Light(second);
	for (int i=0; i<X; i++) _delay_loop_2(brightnessS);
	AllOff();
	for (int i=0; i<X; i++) _delay_loop_2(101-brightnessS);
}





#define SHORTPRESSCNT 5

#define LONGPRESSCNT  20
#define SHORTPRESS 	0x01
#define LONGPRESS  	0x02
#define SHORTCLICK 	0x04
#define LONGCLICK 	0x08

/* this ISR is called when TIMER0 overflows */
ISR(TIMER0_OVF_vect) {

	if ((PINC & 0x01) == 0) {
		if (buttonAcnt==0) buttonA=0;
		if (buttonAcnt<250) { 
			buttonAcnt++;
			if (buttonAcnt==SHORTPRESSCNT) buttonA|=SHORTPRESS;
			if (buttonAcnt==LONGPRESSCNT) buttonA|=LONGPRESS;
		}
	} else {
		buttonAcnt=0;
		if ((buttonA&SHORTPRESS)!=0) buttonA|=SHORTCLICK;
		if ((buttonA&LONGPRESS)!=0) buttonA|=LONGCLICK;
		buttonA&=(SHORTCLICK|LONGCLICK);		
	}


	if ((PINC & 0x02) == 0) {
		if (buttonBcnt==0) buttonB=0;
		if (buttonBcnt<250) { 
			buttonBcnt++;
			if (buttonBcnt==SHORTPRESSCNT) buttonB|=SHORTPRESS;
			if (buttonBcnt==LONGPRESSCNT) buttonB|=LONGPRESS;
		}
	} else {
		buttonBcnt=0;
		if ((buttonB&SHORTPRESS)!=0) buttonB|=SHORTCLICK;
		if ((buttonB&LONGPRESS)!=0) buttonB|=LONGCLICK;
		buttonB&=(SHORTCLICK|LONGCLICK);		
	}
}






int main (void) {
	uint8_t bright=95;
//	BrightnessTest();

	MCUCR |= _BV(PUD);
	

    PORTC |= _BV(0);
    PORTC |= _BV(1);


	TCCR0B |= _BV(CS02) | _BV(CS00);
	/* Enable Timer Overflow Interrupts */
	TIMSK0 |= _BV(TOIE0);

	sei();
	AllOff();
	begin();		// Initialize i2C


/*
	WriteRTCByte(0,0);       //STOP RTC
	WriteRTCByte(1,0x18);    //MINUTE=18
	WriteRTCByte(2,0x08);    //HOUR=8
	WriteRTCByte(3,0x09);    //DAY=1(MONDAY) AND VBAT=1
	WriteRTCByte(4,0x28);    //DATE=28
	WriteRTCByte(5,0x02);    //MONTH=2
	WriteRTCByte(6,0x11);    //YEAR=11
	WriteRTCByte(0,0x80);    //START RTC, SECOND=00
*/
	



	for (;;) {

		if (buttonA & LONGCLICK) {
			buttonA=0;
			if (second==0) second=60;
			second--;
		}
		if (buttonA & SHORTCLICK) {
			buttonA&=~SHORTCLICK;
			second++;
			if (second>59) second=0;
		}

		if (buttonB&LONGCLICK) {
			buttonB=0;
			if (minute==0) minute=60;
			minute--;
		}
		if (buttonB & SHORTCLICK) {
			buttonB&= ~SHORTCLICK;
			minute++;
			if (minute>59) minute=0;
		}



		GetTime();
		for (int i=0; i<100; i++) {
			DisplayHMS(bright, bright, bright);
		}
	}


	return 1;
}





