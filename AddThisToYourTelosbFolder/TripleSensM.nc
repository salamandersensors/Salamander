//$Id: TripleSensM.nc,v 1.0 2010/05/14 22:18:07 ckharnett Exp $
//@author ckharnett <c0harn01@louisville.edu> 


module TripleSensM
{
  provides interface TripleSens;
  uses interface SensorChainPin;
}
implementation
{
  uint8_t m_id[8];   //the sensor id that will be sent in the current message
  uint8_t ls_byte1,ms_byte1,ms_byte2,ls_byte2;  //four bytes of sensor data; on triple sensor board, from DS2450  bytes 1 are photocell and bytes 2 are flowsensor.
  static uint8_t idlist[13][8];   //array keeps found ID numbers available to this function until board reset
  int numroms=12; //max number of 1-wire devices to find --subtract 1 from 1st array limit above. This will address up to 6 TripleSens boards which each have 2 devices.




  // definitions
#define FALSE 0
#define TRUE 1

// Global variables used during SearchROM
uint8_t ROM_NO[8];  //the current ID found in the SearchROM function
int LastDiscrepancy;
int LastFamilyDiscrepancy;
int LastDeviceFlag;
unsigned char crc8;

int x;
int count=0;


  void uwait( uint16_t usec )
  {
    const uint16_t t0 = TAR;
    while( (TAR-t0) < usec );
  }

  enum
  {
    STD_A = 6,
    STD_B = 64,
    STD_C = 60,
    STD_D = 10,
    STD_E = 9,
    STD_F = 55,
    STD_G = 0,
    STD_H = 480,
    STD_I = 70,
    STD_J = 410,
  };

  void init_pins()
  {
    TOSH_MAKE_ONEWIRE_INPUT();
    TOSH_CLR_ONEWIRE_PIN();
  }

  bool reset() // >= 960us
  {
    int present;
    call SensorChainPin.output_low();
    uwait(STD_H); //t_RSTL
    call SensorChainPin.prepare_read();
    uwait(STD_I);  //t_MSP
    present = call SensorChainPin.read();
    uwait(STD_J);  //t_REC
    return (present == 0);
  }

  void write_bit_one() // >= 70us
  {
    call SensorChainPin.output_low();
    uwait(STD_A);  //t_W1L
    call SensorChainPin.output_high();
    uwait(STD_B);  //t_SLOT - t_W1L
  }

  void write_bit_zero() // >= 70us
  {
    call SensorChainPin.output_low();
    uwait(STD_C);  //t_W0L
    call SensorChainPin.output_high();
    uwait(STD_D);  //t_SLOT - t_W0L
  }
  void write_bit( int is_one ) // >= 70us
  {
    if(is_one)
      write_bit_one();
    else
      write_bit_zero();
  }

  int read_bit() // >= 70us
  {
    int bit;
    call SensorChainPin.output_low();
    uwait(STD_A);  //t_RL
    call SensorChainPin.prepare_read();
    uwait(STD_E); //near-max t_MSR
    bit = call SensorChainPin.read();
    uwait(STD_F);  //t_REC
    return bit;
  }

  void write_byte( uint8_t byte ) // >= 560us
  {
    uint8_t bit;
    for( bit=0x01; bit!=0; bit<<=1 )
      write_bit( byte & bit );
  }

  uint8_t read_byte() // >= 560us
  {
    uint8_t byte = 0;
    uint8_t bit;
    for( bit=0x01; bit!=0; bit<<=1 )
    {
      if( read_bit() )
	byte |= bit;
    }
    return byte;
  }


  void matchrom(int whichrom)
  {
   int bytenum;
   write_byte(0x55); //MatchROM command
   for (bytenum=0; bytenum < 8; bytenum++)
   {
   write_byte(idlist[whichrom][bytenum]);
   } 
  }
  
  bool istemperature(int whichrom)
  {
   if (idlist[whichrom][0]==0x10) return TRUE;  //0x10=family code for DS18S20 devices on TripleSensBoard
   else return FALSE;
  }

  bool isADConverter (int whichrom)
  { 
   if (idlist[whichrom][0]==0x20) return TRUE; //0x20=family code for DS2450 devices on TripleSensBord
   else return FALSE;
  }

  bool isBattMon (int whichrom)
  { 
   if (idlist[whichrom][0]==0x26) return TRUE; //0x26=family code for DS2438 Smart Battery Monitor
   else return FALSE;
  }

  uint8_t crc8_byte( uint8_t crc, uint8_t byte )  //the crc routine from DS2411.nc
  { 
    int i;
    crc ^= byte;
    for( i=0; i<8; i++ )
    {
      if( crc & 1 )
        crc = (crc >> 1) ^ 0x8c;
      else
        crc = crc >> 1;
    }
    return crc;
  }

  uint8_t crc8_bytes( uint8_t crc, uint8_t* bytes, uint8_t len )
  {
    uint8_t* end = bytes+len;
    while( bytes != end )
      crc = crc8_byte( crc, *bytes++ );
    return crc;
  }


//Table for crc from 1-W search algorithm code
static unsigned char dscrc_table[] = {
0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,
157,195, 33,127,252,162, 64, 30, 95, 1,227,189, 62, 96,130,220,
35,125,159,193, 66, 28,254,160,225,191, 93, 3,128,222, 60, 98,
190,224, 2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,
70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89, 7,
219,133,103, 57,186,228, 6, 88, 25, 71,165,251,120, 38,196,154,
101, 59,217,135, 4, 90,184,230,167,249, 27, 69,198,152,122, 36,
248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91, 5,231,185,
140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,
17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,
175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,
50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,
202,148,118, 40,171,245, 23, 73, 8, 86,180,234,105, 55,213,139,
87, 9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,
233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,
116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53};

//--------------------------------------------------------------------------
// Calculate the CRC8 of the byte value provided with the current
// global 'crc8' value. From DS Application Note 187.
// Returns current global crc8 value
//
unsigned char docrc8(unsigned char value)
{
	// See Application Note 27
	// TEST BUILD
	crc8 = dscrc_table[crc8 ^ value];
	return crc8;
}


//--------------------------------------------------------------------------
// Perform the 1-Wire Search Algorithm on the 1-Wire bus using the existing
// search state. From DS Application Note 187 with minor modifications.
// Return TRUE : device found, ROM number in ROM_NO buffer
// FALSE : device not found, end of search
//
int OWSearch()
{
	int id_bit_number;
	int last_zero, rom_byte_number, search_result;
	int id_bit, cmp_id_bit;
	uint8_t rom_byte_mask, search_direction;
	// initialize for search
	id_bit_number = 1;
	last_zero = 0;
	rom_byte_number = 0;
	rom_byte_mask = 0x01;
	search_result = 0;
	crc8 = 0;
	// if the last call was not the last one
	if (!LastDeviceFlag)
	{
	
	     // 1-Wire reset
	     call SensorChainPin.init();
           if (reset()){}; //For whatever reason, bus is low before 1st reset, and devices don't respond until second reset below.
           uwait(100);//make it look more like the prop chip signal
	     if (!reset())
		{	
			// reset the search
			LastDiscrepancy = 0;
			LastDeviceFlag = FALSE;
			LastFamilyDiscrepancy = 0;
			return FALSE;  
		}
	// issue the search command
	write_byte(0xF0);           
      
	// loop to do the search
	do
	{

		// read a bit and its complement
		id_bit = read_bit();
		cmp_id_bit = read_bit();


		// check for no devices on 1-wire
		if ((id_bit == 1) && (cmp_id_bit == 1))
            {
            //if it gets in here there are no active devices
		break;
            }
	else
	{
		// all devices coupled have 0 or 1, maybe different for each device
	if (id_bit != cmp_id_bit)
	{    
            //all devices have the same bit at this position
		search_direction = id_bit; // bit write value for search
	}
	else
	{
		// if this discrepancy is before the Last Discrepancy
		// on a previous search then pick the same as last time
		if (id_bit_number < LastDiscrepancy)
			search_direction = ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0);  
		else
		
			// if equal to last pick 1, if not then pick 0
			search_direction = (id_bit_number == LastDiscrepancy);

			// if 0 was picked then record its position in LastZero
		if (search_direction == 0)
		{
			last_zero = id_bit_number;

			// check for Last discrepancy in family
			if (last_zero < 9)
				LastFamilyDiscrepancy = last_zero;
		}
	}


// set or clear the bit in the ROM byte rom_byte_number
// with mask rom_byte_mask

if (search_direction)        //means if search_direction is not zero
{
	ROM_NO[rom_byte_number] |= rom_byte_mask;
      
}
else
{
	ROM_NO[rom_byte_number] &= ~rom_byte_mask;
}
// serial number search direction write bit

write_bit(search_direction);

// increment the byte counter id_bit_number
// and shift the mask rom_byte_mask
id_bit_number++;
rom_byte_mask<<=1;

// if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
if (rom_byte_mask ==0)  
{
	docrc8(ROM_NO[rom_byte_number]); // accumulate the CRC using the routine from DS
	rom_byte_number++;
	rom_byte_mask = 1;
}

}

}while(rom_byte_number < 8); // loop until through all ROM bytes 0-7
// if the search was successful then

if (!((id_bit_number < 65) || (crc8 != 0)))
{

	// search successful so set LastDiscrepancy,LastDeviceFlag,search_result
	LastDiscrepancy = last_zero;
	// check for last device
	if (LastDiscrepancy == 0)
		LastDeviceFlag = TRUE;
	search_result = TRUE;

}
}

// if no device found then reset counters so next 'search' will be like a first
if (!search_result || !ROM_NO[0])
{
	LastDiscrepancy = 0;
	LastDeviceFlag = FALSE;
	LastFamilyDiscrepancy = 0;
	search_result = FALSE;
}


return search_result;
}

//--------------------------------------------------------------------------
// Find the 'first' devices on the 1-Wire bus
// Return TRUE : device found, ROM number in ROM_NO buffer
// FALSE : no device present
//
int OWFirst()
{
	// reset the search state
	LastDiscrepancy = 0;
	LastDeviceFlag = FALSE;
	LastFamilyDiscrepancy = 0;
	count=0;
	return OWSearch();
}


//--------------------------------------------------------------------------
// Find the 'next' devices on the 1-Wire bus
// Return TRUE : device found, ROM number in ROM_NO buffer
// FALSE : device not found, end of search
//
int OWNext()
{
	// leave the search state alone
	return OWSearch();
}



//--------------------------------------------------------------------------
// Verify the device with the ROM number in ROM_NO buffer is present.
// Return TRUE : device verified present
// FALSE : device not present
//
int OWVerify()
{
	unsigned char rom_backup[8];
	int i,rslt,ld_backup,ldf_backup,lfd_backup;

// keep a backup copy of the current state
for (i = 0; i < 8; i++)
	rom_backup[i] = ROM_NO[i];

ld_backup = LastDiscrepancy;
ldf_backup = LastDeviceFlag;
lfd_backup = LastFamilyDiscrepancy;

// set search to find the same device
LastDiscrepancy = 64;
LastDeviceFlag = FALSE;

if (OWSearch())
{

	// check if same device found
	rslt = TRUE;
	for (i = 0; i < 8; i++)
	{
		if (rom_backup[i] != ROM_NO[i])
		{
			rslt = FALSE;
			break;
		}
	}
}

else
	rslt = FALSE;

// restore the search state
for (i = 0; i < 8; i++)
	ROM_NO[i] = rom_backup[i];

LastDiscrepancy = ld_backup;
LastDeviceFlag = ldf_backup;
LastFamilyDiscrepancy = lfd_backup;
// return the result of the verify
return rslt;

}








void SearchRom() //Populate the idlist with up to numroms items. 
{
//comment out WHOLE THING if you just want to hardcode the sensor serial numbers at the top of the file where idlist[][] is initialized.
int rslt=0;
int i=0;

int cnt=0;
for (cnt=0;cnt<(numroms+1);cnt++)
{
   for (i=0; i<7; i++)
   {
           idlist[cnt][i]=0x00;
    }
   idlist[cnt][7]=0xFF; //this makes the first empty ID fail the CRC and we start over again. It doesn't display or send this bogus one.
}


cnt=0;
rslt = OWFirst();   
while(rslt)
{

//Assign the ROM_NO to the idlist
	for(i=0; i<8; i++)
	{
		idlist[cnt][i]=ROM_NO[i];
	}
	
   cnt++;
   if (cnt < numroms) 
       {
       rslt=OWNext();
       }
   else 
       {
       rslt = FALSE;
       }
}
//here is where you'd put the end to comments if hardcoding serial numbers instead of doing SearchROM.

}



  command result_t TripleSens.init()
  {
      SearchRom();
      return SUCCESS;
  }



  command result_t TripleSens.getdata(int whichrom) // >= 6000us
  {
    int retry = 5;
    int counter=0;
    uint8_t* confirm;
    bzero( m_id, 8 );
    while( retry-- > 0 )
    {
      int crc = 0;
        ls_byte1=0x00; //don't re-send previous data if there's a glitch
        ms_byte1=0x00;
        ls_byte2=0x00;
        ms_byte2=0x00;
      
        if (istemperature(whichrom))//do temp measurement on DS18B20 device--assuming Vcc power rather than parasite pwr
        {
           call SensorChainPin.init(); //do analog to digital conversion of data from temp sensor
           if (reset())
           {
			  matchrom (whichrom);
              write_byte (0x44); //Convert command
              while (!read_bit()); //wait till 1s are received, indicating that conversion is finished
           }
           
           call SensorChainPin.init(); //read temp value from scratchpad
           if (reset())
           {
			  matchrom (whichrom);
              write_byte (0xBE); //Read Scratchpad command
              ls_byte1=read_byte();
 	      ms_byte1=read_byte(); //first two bytes are temperature data. Subsequent 7 bytes contain alarms and CRC (not read yet)
              ls_byte2=0x00; //unused field in temperature data
              ms_byte2=0x00; //unused field in temperature data
          }
         
        } //end temperature sensor routine
		
	 if (isADConverter(whichrom))
      { 
        call SensorChainPin.init();
        if (reset())
        {
          //tell the device we are using vcc rather than parasite power
          matchrom(whichrom);
	    write_byte(0x55); //Issue "Write Memory" command--following Usage Example in DS2450 datasheet
	    write_byte(0x1C); //set first byte of address for configuring VCC Control Byte
	    write_byte(0x00); //set second byte of address for configuring channels
          write_byte(0x40); //set vcc power byte indicating we are supplying power--this enables bus traffic during A/D conversions.
          *confirm=read_byte(); //read 1st crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read 2nd crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read back data for verification--should get 40 if this data is viewed.
        }
      
        call SensorChainPin.init();  //Set up channels for sensor measurement
    	  if( reset() )
        {
          matchrom(whichrom);
          write_byte(0x55); //Issue "Write Memory" command--Following Usage Example in DS2450 datasheet
	    write_byte(0x08); //set first byte of address for configuring status and control bits
	    write_byte(0x00); //set second byte of address for configuring status and control bits

          write_byte(0x88); //turn on channel A (Photocell circuit power)
	    *confirm=read_byte(); //read 1st crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read 2nd crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read back data for verification--should get 88 if this data is viewed.
	    write_byte(0x8C); //write second byte of control data for pin A (don't care)
          *confirm=read_byte(); //read 1st crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read 2nd crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read back data for verification--should get 8C

	    write_byte(0x0C); //make channel B (Photocell signal) a 12-bit A/D input 
	    *confirm=read_byte(); //read 1st crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read 2nd crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read back data for verification--should get C8
	    write_byte(0x8D); //make it have 0-5V input range
          *confirm=read_byte(); //read 1st crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read 2nd crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read back data for verification--should get 8C

	    write_byte(0x0C); //make channel C a 12-bit A/D input (Signal from flow sensor)
	    *confirm=read_byte(); //read 1st crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read 2nd crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read back data for verification--should get 08
	    write_byte(0x8D); //write second byte of control data for pin C, making its input range 0-5V
          *confirm=read_byte(); //read 1st crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read 2nd crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read back data for verification--should get 8D

          write_byte(0x88); //turn on channel D (flow sensor circuit)
	    *confirm=read_byte(); //read 1st crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read 2nd crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read back data for verification--should get 88
	    write_byte(0x8C); //write second byte of control data for pin D (don't care)
          *confirm=read_byte(); //read 1st crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read 2nd crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read back data for verification--should get 8C
        }

        uwait(10000); //wait 10000 us to stabilize signal

        call SensorChainPin.init(); //do analog to digital conversion of data from lightsensor on pin B and flowsensor pinC
        if (reset())
        {
     	    matchrom (whichrom);
          write_byte (0x3C); //Convert command
          write_byte (0x06); //Input mask--select channels B and C
          write_byte (0x00); //readout control byte  
          *confirm=read_byte(); //read 1st crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read 2nd crc16 byte--currently throw away this data in a variable called confirm
          while (!read_bit()); //wait till 1s are received, indicating that conversion is finished
        }

        call SensorChainPin.init(); //read the converted data
        if (reset())
        {
	    matchrom(whichrom);
          write_byte(0xAA); //Read Memory command
          write_byte (0x02); //Address (Least sig. byte of data from Channel B)
          write_byte (0x00); //The rest of this address (0002)
          ls_byte1=read_byte();//collect least significant byte of photocell conversion data -chan B
          ms_byte1=read_byte();//collect most significant byte of photocell conversion data (next byte)
          ls_byte2=read_byte();//collect LSB of flowsensor conversion data-chanC
          ms_byte2=read_byte();//collect msb of flowsensor--chan C
        }
        
        call SensorChainPin.init();  //finally, turn off sensors
        if (reset () )
        {
	    matchrom(whichrom);
	    write_byte(0x55); //Issue "Write Memory" command--Following Usage Example in DS2450 datasheet
	    write_byte(0x08); //set first byte of address for configuring channels
	    write_byte(0x00); //set second byte of address for configuring channels
     

          write_byte(0xC8); //turn channel A off (sensor power)
	    *confirm=read_byte(); //read 1st crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read 2nd crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read back data for verification--should get C8 if this data is viewed.
	    write_byte(0x8C); //write second byte of control data for pin A (don't care)
          *confirm=read_byte(); //read 1st crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read 2nd crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read back data for verification--should get 8C

	    write_byte(0x0C); //Keep channel B an A/D input
	    *confirm=read_byte(); //read 1st crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read 2nd crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read back data for verification--should get 0C
	    write_byte(0x8D); //write second byte of control data for pin B (don't care)
          *confirm=read_byte(); //read 1st crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read 2nd crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read back data for verification--should get 8D

	    write_byte(0x0C); //Keep channel C an A/D input (0-to-5V signal from OPT101 light sensor)
          //note, the usage example has channel D as an input, but we make it C because of the turbidity board's wiring.
	    *confirm=read_byte(); //read 1st crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read 2nd crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read back data for verification--should get 0C
	    write_byte(0x8D); //write second byte of control data for pin C (don't care)
          *confirm=read_byte(); //read 1st crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read 2nd crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read back data for verification--should get 8D

          write_byte(0xC8); //turn OFF channel D (front LED)--a switch rather than an input as in usage example.
	    *confirm=read_byte(); //read 1st crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read 2nd crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read back data for verification--should get C8
	    write_byte(0x8C); //write second byte of control data for pin D (don't care)
          *confirm=read_byte(); //read 1st crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read 2nd crc16 byte--currently throw away this data in a variable called confirm
	    *confirm=read_byte(); //read back data for verification--should get 8C
        }
       }//end AtoD routine	
       
       if (isBattMon(whichrom))//do temp and voltage measurement on DS2438 device--assuming Vcc power rather than parasite pwr
        {  //Mote power boards can have a DS2438 to monitor input voltage to sensors' voltage dividers. Also get temp at mote.
           call SensorChainPin.init(); //do analog to digital conversion of data from temp sensor
           if (reset())
           {
		  matchrom (whichrom);
              write_byte (0x44); //ConvertTemperature command
              while (!read_bit()); //wait till 1s are received, indicating that conversion is finished
           }
 
           call SensorChainPin.init(); //do analog to digital conversion of data from voltage sensor
           if (reset())
           {
		  matchrom (whichrom);
              write_byte (0xB4); //Convert Voltage command
              while (!read_bit()); //wait till 1s are received, indicating that conversion is finished
           }
          
           
           call SensorChainPin.init(); //issue Recall Memory Page 00h command
           if (reset())
           {
		  matchrom (whichrom);
              write_byte (0xB8); //Recall Memory
              write_byte (0x00); //page 0 
           }
           
           call SensorChainPin.init(); //read temp and voltage value from scratchpad
           if (reset())
           {
	        matchrom (whichrom);
              write_byte (0xBE); //Read Scratchpad command
              write_byte (0x00); //page 0
              read_byte(); //skip this byte--it's status/config
              ls_byte1=read_byte(); //temperature lsb
 	        ms_byte1=read_byte(); //temperature msb
              ls_byte2=read_byte(); //read vdd lsb
              ms_byte2=read_byte(); //read vdd msb All 10 Bits 1 means 10.23V
          }
         
        } //end BatteryMonitor routine
          for (counter=0; counter<8; counter++)
          crc = crc8_byte(crc, idlist[whichrom][counter]); //doing crc on stored number not necessary but it works--save as example
	    if( crc == 0 )
          {
	      memcpy( m_id, idlist[whichrom], 8 );
	      return SUCCESS;
	    }
        
      
           
      
    }//end while-retry loop

       return FAIL;

  } //end of init subroutine--now Salamander can get the ID code and data by calling the following commands.

  command uint8_t TripleSens.get_id_byte( uint8_t index )
  {
    return (index < 6) ? m_id[index+1] : 0;
  }

  command void TripleSens.copy_id( uint8_t* id )
  {
    memcpy( id, m_id+1, 6 );
  }

  command uint8_t TripleSens.get_lsdata1()
  {
    return ls_byte1;
  }

  command uint8_t TripleSens.get_msdata1()
  {
    return ms_byte1;
  }

  command uint8_t TripleSens.get_lsdata2()
  {
    return ls_byte2;
  }

  command uint8_t TripleSens.get_msdata2()
  {
    return ms_byte2;
  }
  command uint8_t TripleSens.get_family()
  {
    return m_id[7];
  }

  command uint8_t TripleSens.get_crc()
  {
    return m_id[0];
  }

  uint8_t calc_crc()
  {
    return crc8_bytes( 0, m_id+1, 7 );
  }

  command bool TripleSens.is_crc_okay()
  {
    return (call TripleSens.get_crc() == calc_crc());
  }
}

