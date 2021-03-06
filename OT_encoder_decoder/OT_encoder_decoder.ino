/*
 * OpenTherm encoder, by Derek Jennings 
 * based on works by: AMVV and Sebastian Wallin
 * description:
 * Example on opentherm communication
 *
 * This OT encoder implements a manchester encoder, which transmits a bit every 1ms,
 * so it fires every 0.5ms to make a transition.
 * it is not very clean code...
 * It listens to frames coming from the boiler(master) and thermostat(slave) and forwards them on when in monitor mode.
 * In control mode it intercepts boiler set point frames from the master and substitutes its own estimation of what the boiler set point should be.
 * It uploads statistical data over Ethernet to a data logging service (xively.com)
 * It communicates with the server to get a setpoint.
 * It sets the boiler to the received setpoint.
 * Hardware required is
 * 	Arduino Uno - or equivalent
 *	Ethernet module 
 *	Opentherm gateway circuit 
 *
 * TODO:
 * - Clean up the code
 *   - handling of the message to the boiler
 * - Enter fail safe mode - independently control set point
 *   - attempt to listen to weather station messages
 *
 * TODO for hardware revision - bus powered
 * - lower power (unnecessary for now)
 *
 * DONE
 * - expand message to server to include data read from the boiler.
 * - Control setpoint according to received message
 * - meaningfull messages on the display
 *   - nice pretty characters 
 * - Set set of messages to be exchanged, and corresponding periods.
 *
 * - now also receives the age of the set point, in seconds, which it uses to synchronise such as to ask for date soon after the calculation
 */
#include <MemoryFree.h>
#include <OpTh.h>
//#include <PortsLCD.h>
#include <EtherCard.h>
#include <util/parity.h>
/* Include the U8glib library */
#include "U8glib.h"
/* Define the SPI Chip Select pin */
#define CS_PIN (10)
#define RESET (9)	// to reset the Ethernet module
#define MASTER_OUTPUT_PIN (4)
#define SLAVE_OUTPUT_PIN (5)
#define OTPERIOD (996) //used a little trick to find mine, see below in loop

#define MAX_BOILER_TEMP (80)
#define MIN_BOILER_TEMP (35)
#define REPORTING_PERIOD (30000) // in milliseconds
// xively.com feed
//#define FEED    "MY FEED"
//#define APIKEY  "MY API"
#include "xively.h"  //contains xively.com account settings
//CHARACTER DEFINITIONS

#define CHAR_CH    (0)
#define CHAR_DHW   (1)
#define CHAR_RF    (2)
#define CHAR_FLAME (3)
#define CHAR_OT    (4)
#define CHAR_OK    (5)
#define CHAR_NOK   (6)

byte CH[8] = //OK
{
  B00100,
  B01110,
  B11111,
  B11111,
  B10001,
  B10101,
  B10101,
  B11111
};
byte DHW[8] = //OK
{
  B00100,
  B00100,
  B01110,
  B11111,
  B00000,
  B10101,
  B00000,
  B10101
};
byte OK[8] =
{
  B00001,
  B00011,
  B00010,
  B00110,
  B10100,
  B11100,
  B11000,
  B01000
};
byte NOK[8] =
{
  B10001,
  B11011,
  B01110,
  B00100,
  B01110,
  B11011,
  B10001,
  B10001
};
byte RF[8] =
{

  B00011,
  B00000,
  B00110,
  B00001,
  B01100,
  B00010,
  B11000,
  B11000
};
byte FLAME[8] =
{
  B00100,
  B00100,
  B00110,
  B00110,
  B01111,
  B11011,
  B11011,
  B01110
};
byte OTH[8] =
{
  B00100,
  B11111,
  B10001,
  B10001,
  B10001,
  B10001,
  B10101,
  B11111
};

// LCD strings
prog_char bad_data[] PROGMEM = "bad data";
prog_char par_error[] PROGMEM = "par err";
prog_char ss_error[] PROGMEM = "ss err";
prog_char frame_ok[] PROGMEM = "OK";
prog_char oth_error[] PROGMEM = "oth err";
char lcd_status[9];

// ethernet interface mac address, must be unique on the LAN
static byte mymac[] = { 0x74,0x69,0x69,0x2D,0x33,0x9 };
// ethernet interface ip address
static byte myip[] = { 192,168,1,38 };
// gateway ip address
static byte gwip[] = { 192,168,1,254 };
static byte hisip[] = { 216, 52, 233, 120 };
prog_char website[] PROGMEM = "api.xively.com";
byte Ethernet::buffer[450];
Stash stash;
static byte session;
byte res;
unsigned long previousMillis = 0;
unsigned long currentMillis;
/* Timer2 reload value, globally available */
unsigned int tcnt2;

/*my stuff amvv */
unsigned int cycles, inner_cycles = 0;
unsigned int total_cycles = 55;
int originalvalue = 0;
int y;
int bits_sent = 0;

byte flame = false;
byte strategy =2;    // set the default strategy
byte setpoint_history[10] = { MIN_BOILER_TEMP, MIN_BOILER_TEMP, MIN_BOILER_TEMP, MIN_BOILER_TEMP, MIN_BOILER_TEMP, MIN_BOILER_TEMP, MIN_BOILER_TEMP, MIN_BOILER_TEMP, MIN_BOILER_TEMP, MIN_BOILER_TEMP };  // set point for last 10 reporting periods

byte bit_out = HIGH;
boolean done = false;
byte state = 0; //0 - start bit; 1 - message: 2 - stop bit
byte output_pin = SLAVE_OUTPUT_PIN;

typedef struct {
  unsigned int house;
  unsigned int device;
  byte temp;
  byte setpoint;
  byte reqtemp;
  byte CHtemp;
  byte returntemp;
  byte boilerstatus;
  int roomtemp;
  int roomset;
  int outtemp;
  int h2o;
} 
OpenThermData;

OpenThermData buf;

typedef struct {
  unsigned int house;
  unsigned int device;
  unsigned int burner_starts;
  unsigned int CH_pump_starts;
  unsigned int DHW_pump_starts;
  unsigned int DHW_burner_starts;
  unsigned int burner_hours;
  unsigned int CH_pump_hours;
  unsigned int DHW_pump_hours;
  unsigned int DHW_burner_hours;
} 
OpenThermExtendedData;

OpenThermExtendedData extbuf;

//OTdata MM;
byte MM1, MM2, MM3, MM4 = 0;

OpTh OT = OpTh();  // create new OpTh class instance

/* Create an instance of the library for the 12864 LCD in SPI mode */
// use software SPI to avoid conflict with Ethercard
U8GLIB_ST7920_128X64_1X u8g(7, 6, 10);


/* Setup phase: configure and enable timer2 overflow interrupt */
void setup() {
  // assign default color value
  
  // assign output pins
  pinMode( MASTER_OUTPUT_PIN, OUTPUT);
  digitalWrite(MASTER_OUTPUT_PIN, HIGH);
  pinMode( SLAVE_OUTPUT_PIN, OUTPUT);		// idle high
  digitalWrite(SLAVE_OUTPUT_PIN, HIGH);
    
  Serial.begin(57600);
  Serial.print("freeMemory()=");
  Serial.println(freeMemory()); 
  // Set up Ethernet
  pinMode(RESET, OUTPUT);
  initialize_ethernet();    
  
 

 
  buf.house = 192;
  buf.device = 4;
  buf.temp = 0;
  buf.reqtemp =25;
  buf.setpoint = 0;
  buf.CHtemp = 0;
  buf.returntemp = 0;
  buf.boilerstatus = 0;
  buf.roomset = 0;
  buf.roomtemp = 0;
  buf.outtemp = 0;
  buf.h2o = 0;
  
  extbuf.house = 192;
  extbuf.device = 5;
  extbuf.burner_starts = 0;
  extbuf.CH_pump_starts = 0;
  extbuf.DHW_pump_starts = 0;
  extbuf.DHW_burner_starts = 0;
  extbuf.burner_hours = 0;
  extbuf.CH_pump_hours = 0;
  extbuf.DHW_pump_hours = 0;
  extbuf.DHW_burner_hours = 0;
 

  OT.init();
  OT.setPeriod(OTPERIOD);

  StopInterrupts();
  StartInterrupts();
}

void StopInterrupts()
{
  TIMSK2 &= ~(1<<TOIE2);
}

void StartInterrupts()
{   
  /* Configure timer2 in normal mode (pure counting, no PWM etc.) */
  TCCR2A &= ~((1<<WGM21) | (1<<WGM20));
  TCCR2B &= ~(1<<WGM22);

  /* Select clock source: internal I/O clock */
  ASSR &= ~(1<<AS2);

  /* Disable Compare Match A interrupt enable (only want overflow) */
  TIMSK2 &= ~(1<<OCIE2A);

  /* Now configure the prescaler to CPU clock divided by 128 */
  TCCR2B |= (1<<CS22)  | (1<<CS20); // Set bits
  TCCR2B &= ~(1<<CS21);             // Clear bit

  /* We need to calculate a proper value to load the timer counter.
   * The following loads the value 192 into the Timer 2 counter register
   * The math behind this is:
   * (CPU frequency) / (prescaler value) = 125000 Hz = 8us.
   * (desired period) / 8us = 62.5.
   * MAX(uint8) + 1 - 62 = 194;
   * amvv note: don't know why I made it 192, if the result is 194, but it seems to work...
   */
  /* Save value globally for later reload in ISR */

  tcnt2 = 192; 

  /* Finally load end enable the timer */
  TCNT2 = tcnt2;
  TIMSK2 |= (1<<TOIE2);


}

boolean logic_transition = false;

/*
 * Install the Interrupt Service Routine (ISR) for Timer2 overflow.
 * This is normally done by writing the address of the ISR in the
 * interrupt vector table but conveniently done by using ISR()  */
ISR(TIMER2_OVF_vect) {
  /* Reload the timer */
  TCNT2 = tcnt2;

  switch (state) {
  case 0: //start bit 
    if (logic_transition == false)
    {
      y = 1;
      logic_transition = true;
      //Serial.print(y);
    }
    else
    {
      y = !y;  
      logic_transition = false;
      state++;
    }
    break;

  case 1://message byte 1
    if (logic_transition == false)
    {
      y = ((MM1) & 128) >> 7;
      MM1 = MM1 << 1;
      logic_transition = true;
      //Serial.print(y);
    }
    else
    {
      y = !y;
      logic_transition = false;
    }
    break;
  case 2://message byte 2
    if (logic_transition == false)
    {
      y = ((MM2) & 128) >> 7;
      MM2 = MM2 << 1;
      logic_transition = true;
      //Serial.print(y);
    }
    else
    {
      y = !y;
      logic_transition = false;
    }
    break;
  case 3://message byte 3
    if (logic_transition == false)
    {
      y = ((MM3) & 128) >> 7;
      MM3 = MM3 << 1;
      logic_transition = true;
      //Serial.print(y);
    }
    else
    {
      y = !y;
      logic_transition = false;
    }
    break;
  case 4://message byte 3
    if (logic_transition == false)
    {
      y = ((MM4) & 128) >> 7;
      MM4 = MM4 << 1;
      logic_transition = true;
      //Serial.print(y);
    }
    else
    {
      y = !y;
      logic_transition = false;
    }
    break;
  case 5: //stop bit 
    if (logic_transition == false)
    {
      y = 1;
      logic_transition = true;
      //Serial.println(y);
    }
    else
    {
      y = !y;  
      logic_transition = false;
      state = 0;
    }
    break;
  }


  digitalWrite(output_pin, !y);
  //if (logic_transition == true)
  //Serial.print(y, DEC);

  bits_sent = bits_sent + 1;

  if (bits_sent == 18) //(sizeof(MM*3)*8*2 + 2))  MM1 sent
  {
    state++;
  }
  if (bits_sent == 34) //(sizeof(MM*3)*8*2 + 2))  MM2 sent
  {
    state++;
  }
  if (bits_sent == 50) //(sizeof(MM*3)*8*2 + 2))  MM3 sent
  {
    state++;
  }
  if (bits_sent == 66) //(sizeof(MM*3)*8*2 + 2)) MM4 sent
  {
    state++;
  }
  if (bits_sent == 68) //(sizeof(MM)*8*2 + 4))
  {
    digitalWrite(output_pin, HIGH);
    done = true;
    bits_sent = 0;
    StopInterrupts();
    state=0;
  }
  else
  {
    done = false;
  }
}

int error_reading_frame;


/* Main loop.*/
void loop() {
 // picture loop  refreshes LCD screen
// u8g.firstPage();  
// do {
//   draw();  // refresh LCD screen
//} while( u8g.nextPage() );
    
   
  ether.packetLoop(ether.packetReceive());
  if (done == true)
  {
    total_cycles++;
    OT.waitFrame();
    error_reading_frame = OT.readFrame();

    //uncomment the cycle below to get the measurement of the OT period, then adapt the #define above, and comment it again...

//        if(total_cycles == 11)
//        {    
//          Serial.print("measuring");
//          OT.measureOtPeriod();
//          int perper = OT.getPeriod();
//          Serial.print(perper);
//          Serial.print("us");
//        }
    switch(error_reading_frame) {
  case 1:  
//	Serial.print("1");//bad data");
//      strcpy_P(lcd_status, bad_data);   
      break;
  case 2:
//        Serial.print("2");//ss bit error");
//	strcpy_P(lcd_status, ss_error);
	break;
  case 3:
//        Serial.print("3");//parity error");
//	strcpy_P(lcd_status, par_error);   
	break;
  case 0:    // good frame
    	copyframe();   // copy frame to other interface
        display_frame();
//	Serial.print("OK");
//	strcpy_P(lcd_status, frame_ok);
	break;
  default:
	 Serial.print("9");//other error");
//	 strcpy_P(lcd_status, oth_error);
    }
    
// upload the data every REPORTING_PERIOD
  currentMillis = millis();
  if(currentMillis - previousMillis > REPORTING_PERIOD) {
    previousMillis = currentMillis;
    upload_stats();
// calculate the temperature control point based on the current strategy    
//    Serial.println("Calculating strategy");
    
    switch(strategy) {
      case 1:
	smoothed();
	break;
      case 2:
	distance();
      case 0:
      default:
	nostrategy();
    }
  }

  
  }
}  // end main loop



void upload_stats() {
      // By using a separate stash,
      // we can determine the size of the generated message ahead of time
      stash.cleanup();
      byte sd = stash.create();
      
//      if (res > 15){
//     	initialize_ethernet();	// if ethernet is away with the fairies reset it. 
//      }  
//      res ++;
      
      if (buf.temp > 0) 
      {
	stash.print("burner_temp,");
	stash.println(buf.temp);
      }
      if (buf.CHtemp > 0) 
      {
	stash.print("CHtemp,");
	stash.println(buf.CHtemp);
      }
      if (buf.returntemp > 0) 
      {
	stash.print("returntemp,");
	stash.println(buf.returntemp);
      }
      stash.print("CH,");
      stash.println((buf.boilerstatus & B00000010) >> 1);
      stash.print("DHW,");
      stash.println((buf.boilerstatus & B00000100) >> 2);      
      if (buf.roomset > 0) 
      {
	prepareStash("set_temp,", buf.roomset);
      }
      if (buf.roomtemp > 0) 
      {
	prepareStash("temp,", buf.roomtemp);
      }
       if (buf.outtemp > 0) 
      {
	prepareStash("out_temp,", buf.outtemp);
      }     
       if (buf.h2o > 0) 
      {
	prepareStash("H2O,", buf.h2o);
      }     

      stash.save();
    
      // generate the header with payload - note that the stash size is used,
      // and that a "stash descriptor" is passed in as argument using "$H"
      Stash::prepare(PSTR("PUT http://$F/v2/feeds/$F.csv HTTP/1.0" "\r\n"
                          "Host: $F" "\r\n"
                          "X-ApiKey: $F" "\r\n"
                          "Content-Length: $D" "\r\n"
                          "\r\n"
                          "$H"),
              website, PSTR(FEED), website, PSTR(APIKEY), stash.size(), sd);

      // send the packet - this also releases all stash buffers once done
      session = ether.tcpSend();  
      
      const char* reply = ether.tcpReply(session);
      if (reply !=0){
	res = 0;
	Serial.print(reply);
//      }  else {
//          Serial.print("freeMemory()=");
//          Serial.println(freeMemory()); 
      }
 
      //HERE RESET CHTEMP, RETURNTEMP, BOILERSTATUS, TEMP
      buf.CHtemp = 0;
      buf.returntemp = 0;
      buf.boilerstatus = 0;
      buf.h2o = 0;

      done = false;
      StartInterrupts();
}

void initialize_ethernet(void){
   //Restart if needed
   restart: 
 
   //Reinitialize ethernet module
   Serial.println("Resetting Ethernet...");
   digitalWrite(RESET, LOW);
   delay(20);
   digitalWrite(RESET, HIGH);
   delay(20);
   if (ether.begin(sizeof Ethernet::buffer, mymac) == 0){ 
     Serial.println( "Failed to access Ethernet controller");
     goto restart;
   }
   
   ether.staticSetup(myip, gwip);
   ether.copyIp(ether.hisip, hisip);
   
//   if (!ether.dhcpSetup()){
//     Serial.println("DHCP failed");
//     goto restart;
//   }
//   if (!ether.dnsLookup(website)) {
//     Serial.println("DNS failed");
//    goto restart;
//   }
   ether.printIp("IP:  ", ether.myip);
   ether.printIp("GW:  ", ether.gwip);  
//   ether.printIp("DNS: ", ether.dnsip);  
   ether.printIp("SRV: ", ether.hisip);
 
 //reset init value
 res = 0;
 }



//Draw LCD screen
void draw() {
  u8g.setFont(u8g_font_unifont);
  u8g.drawStr( 0, 10, lcd_status);
}


// copy frame from master to slave or vice versa
void copyframe() {
  if (OT.isMaster()) {
    output_pin = SLAVE_OUTPUT_PIN;
  }
  else
  {
    output_pin = MASTER_OUTPUT_PIN;
  }
  uint32_t frame = OT.getFrame();
  MM4 = lowByte(frame);
  frame >>= 8; // strip off lowest byte
  MM3 = lowByte(frame);
  frame >>= 8; // strip off lowest byte
  MM2 = lowByte(frame);
  frame >>= 8; // strip off lowest byte
  MM1 = frame;
  
  
  // If controls have set a setpoint greater than 20, but have signalled no demand, because room is up to temperature
  // then change signal to demand to keep the pump going to keep the radiators warm
 //   if (OT.getDataId() == 0 && buf.reqtemp > 20 && !(MM2 & B00000010) && OT.isMaster()) {
 //   MM3 = MM3 | B00000001;
 // }
  
  if (OT.getDataId() == 1) {    // Temp Control set point
    if (OT.isMaster()) {
      MM3 = buf.setpoint;  // Tell the boiler what temperature to set according to current strategy
      MM4 = 0;
    }
   }
   
   
   // without this section Remaha controls hangs when updating time/date
   // that Intergas boiler ignores
  if (OT.getDataId() == 20 && OT.isMaster() ) {    // If controller sends a date set command echo it back again with a NACK
    output_pin = MASTER_OUTPUT_PIN;

    MM1 = B01110000;
    MM3 = 0x53;
    MM4 = 0xc8;
    delay(50);
//    Serial.println("ignore");
//    StartInterrupts();
    }
   
  parity();   // check parity
  StartInterrupts();
}



unsigned char parity() {
  
  unsigned char noofones = 0; 
  uint32_t ino = MM4 + (MM3 << 8) + ((uint32_t)MM2 << 16) + ((uint32_t)MM1 << 24);  //put the frame together

  while(ino != 0) 
  { 
    noofones++; 
    ino &= (ino-1); // the loop will execute once for each bit of ino set
  } 

  if (noofones & 1) { 
    MM1 ^= B10000000;  // XOR the parity bit
  }
   /* if noofones is odd, least significant bit will be 1 */ 
  return (noofones & 1); 
}

void prepareStash( char txt[], int dat) {
  stash.print(txt);
  stash.print(highByte(dat));
  stash.print(".");
  stash.println(lowByte(dat)*100/256);  
}


void display_frame() {
  byte msg_type = OT.getMsgType();
  byte data_id = OT.getDataId();
  unsigned int data_value = OT.getDataValue();

  unsigned int ut;
  unsigned int t;
  
Serial.print("from ");
if (OT.isMaster()) {
  Serial.print("master ");
} else {
  Serial.print("slave  ");
}
Serial.print("   type ");
Serial.print(msg_type);
Serial.print("   data ID ");
Serial.print(data_id);
Serial.print("   data value ");
Serial.println(data_value);



  switch(data_id) {
  case 0:  // status
    if (OT.isMaster()) { // only interested in slave status
      break;
    }
    t = lowByte(data_value);
//    if (t & B00000010) {  // perform bitmasking on status frame
//      Serial.print("CH active   ");//CH
//    }
//    if (t & B00000100) {  
//      Serial.print("DHW active   ");//DHW
//    }
    if (t & B00001000) {
//      Serial.println("flame! ");//FLAME
      flame = true;
    }
    else {
//      Serial.println("no flame ");
      flame = false;
    }
    buf.boilerstatus = buf.boilerstatus | t;  // OR the current status with previous status in this reporting period
//    Serial.print("Status ");
//    Serial.println((byte)t);
    break;
  case 1:  // Control setpoint
    t = data_value / 256;  // don't care about decimal values
    if (OT.isMaster()) { 
      buf.reqtemp = (byte)t;
    }
    else {
    buf.temp = (byte)t;
    }
//    Serial.print("set burner to ");
//    Serial.print(t);
//    Serial.println("C");
    break;
  case 3:  //slave configs
//    Serial.println("slave flags OK    ");
    break;

  case 5:  //faults water and flame
//    Serial.println("faults flags OK    ");
    break;
    
  case 14:  //max modulation
  case 17:  //modulation level not reported
  case 19:  //DHW flow rate
  case 26:  //DHW temp - does not get reported properly in my boiler AMVV
  case 33:  //exhaust temp
    break;    

  case 16:  // room setpoint
    buf.roomset = data_value;
//    Serial.print("room set to ");
//    Serial.print(highByte(buf.roomset));
//    Serial.print(".");
//    Serial.print(lowByte(buf.roomset)*100/256);
//    Serial.println("C");
    break;
  case 18:  //water pressure
    buf.h2o = data_value;
//    Serial.print("H2O press: ");
//    Serial.print(highByte(buf.h2o));
//    Serial.print(".");
//    Serial.println(lowByte(buf.h2o)*100/256);
    break;
   case 20:  //Date-Time
    t = data_value / 256;  // don't care about decimal values
//    Serial.print("Date ");
//    Serial.print(data_value);
//    Serial.println("     ");
    break;  
   case 24:  // room actual temperature
    if (OT.isMaster()) {
      buf.roomtemp = data_value;
//      Serial.print("Room temp ");
//      Serial.println(highByte(buf.roomtemp));
//      Serial.println("C");
    }
    break;
  case 25:  //CH temp
    if (OT.isMaster()) { // only interested in slave status
      break;
    }
    t = data_value / 256;  // don't care about decimal values
    buf.CHtemp = max(buf.CHtemp, (byte)t);
//    Serial.print("CH temp: ");
//    Serial.print(t);
//    Serial.println(" ");
    break;
   case 27:  //outside temp
    buf.outtemp = data_value;  
//    Serial.print("outside temp: ");
//    Serial.println(highByte(buf.outtemp));
    break;   
   case 28:  //return temp
    t = data_value / 256;  // don't care about decimal values
    buf.returntemp = max(buf.returntemp, byte(t));
//    Serial.print("ret temp: ");
//    Serial.print(t);
//    Serial.println(" ");
    break;
  case 116:  //burner starts
    ut = (unsigned int)data_value;  // don't care about decimal values
    extbuf.burner_starts = ut;
//    Serial.print("bur st: ");
//    Serial.print(ut);
//    Serial.println("     ");
    break;
  case 117:  //CH pump starts
    ut = (unsigned int)data_value;  // don't care about decimal values
    extbuf.CH_pump_starts = ut;
//    Serial.print("CH pump st: ");
//    Serial.print(ut);
//    Serial.println("     ");
    break;
  case 118:  //DHW starts
    ut = (unsigned int)data_value;  // don't care about decimal values
    extbuf.DHW_pump_starts = ut;
//    Serial.print("DHW st: ");
//    Serial.print(ut);
//    Serial.println("     ");
    break;
  case 119:  //DHW burner starts
    ut = (unsigned int)data_value;  // don't care about decimal values
    extbuf.DHW_burner_starts = ut;
//    Serial.print("DHW bur st: ");
//    Serial.print(ut);
//    Serial.println("     ");
    break;
  case 120:  //burner hours
    ut = (unsigned int)data_value;  // don't care about decimal values
    extbuf.burner_hours = ut;
    //Serial.print("bur h: ");
    //Serial.print(ut);
    //Serial.print("     ");
    break;
  case 121:  //CH pump hours
    ut = (unsigned int)data_value;  // don't care about decimal values
    extbuf.CH_pump_hours = ut;
    //Serial.print("CH p h: ");
    //Serial.print(ut);
    //Serial.print("     ");
    break;
  case 122:  //DHW pump hours
    ut = (unsigned int)data_value;  // don't care about decimal values
    extbuf.DHW_pump_hours = ut;
    //Serial.print("DHW p h: ");
    //Serial.print(ut);
    //Serial.print("     ");
    break;
  case 123:  //DHW burner hours
    ut = (unsigned int)data_value;  // don't care about decimal values
    extbuf.DHW_burner_hours = ut;
    //Serial.print("DHW b h: ");
    //Serial.print(ut);
    //Serial.print("     ");
    break; 
  default:
//    Serial.print("incorrect message ID   ");
//    Serial.println(data_id);
    break;

  }  // switch  
}


// This section contains a number of heating strategies that can be selected.
// Strategy 'smoothed'
// -----------
// Smooth Thermostat set point - Allow the room thermostat to shoose a control setpoint,
// but smooth the set point to provide more gradual operation.
// Smoothing takes the average of the requested setpoint for the last 10 reporting periods (5 min)

// Strategy 'nostrategy'
// -----------
// Allows the room thermostat to shoose the control setpoint,
// but only updates it once every reporting period so rapid changes are not communicated to the boiler.



// Room control setting   buf.roomset
 // Current setpoint reported by slave   buf.temp
 // Setpoint given to slave buf.setpoint
 // Requested setpoint  buf.reqtemp
 // Current water temperature  buf.CHtemp


void nostrategy() {
  
// Tell the slave to set water temperature to whatever wall controls request
// Since strategy is set every 30 seconds so more frequent change requests will be ignored
// so this strategy is not QUITE transparent  
  buf.setpoint = buf.reqtemp;
}

// Tell the slave to set water temperature to whatever wall controls request
// averaged over the last 10 reporting periods (5 mins)
// This strategy should avoid sudden temperature changes 
void smoothed() {
  byte i;
  unsigned int x = buf.reqtemp;
  for ( i =9; i>0; i-- ) {
    x = x + setpoint_history[i];
 //   Serial.print("   ");
 //   Serial.print(x);
    setpoint_history[i] = setpoint_history[i-1]; 
  }
  if (buf.reqtemp < 30) {
    setpoint_history[0] = MIN_BOILER_TEMP;  // set minimum flow temperature
  } else {
    setpoint_history[0] = buf.reqtemp;
  }
  buf.setpoint = x/10;	// average of current point and last 9
}


//If the distance between the desired room temperature is more than half a degree C less than the actual temperature,
// increase the water temperature to 55 deg
void distance() {
  if ( buf.roomtemp + 128 <  buf.roomset  && buf.reqtemp < 55 ) {
    buf.reqtemp = 55;
  }
    smoothed();
}
