// $Id: SalamanderM.nc,v 1.0 2010/05/14 00:26:11 ckharnett Exp $
// @author CKHarnett <c0harn01@louisville.edu>
// Recognizes and polls "Salamander" sensor boards from U of L. It extends the original SerialIDSend app from UCB 
// to sensor boards with an onboard DS2450 (4 chan a-d converter) and DS1820 (temperature sensor).

module SalamanderM
{
  provides interface StdControl;
  uses interface Timer;
  uses interface SendMsg;
  uses interface Leds;
  uses interface TripleSens;
  uses interface SequencerOscOn;//puts signal on output pin, doesn't interfere bec pin unused on old pwr boards
  uses interface SensorChainOn; //puts signal on output pin, doesn't interfere bc pin unused on on old pwr boards
  uses interface SplitControl; //new addition May 2009: turn off radio: Nitin Matnani 
}
implementation
{
  TOS_Msg m_msg;
  int m_int;
  int whichrom=0; //which ID number is currently being polled
  int shortTime=500; //time in ms between polling each sensor
  uint32_t longTime=120000; //time after polling last sensor before restarting from beginning
  bool m_sending;

  typedef struct SalamanderMsg
  {
    uint16_t src;
    uint8_t id[8];
    uint8_t msdata1;//Conversion data from DS2450-Most sig byte--temp or photo
    uint8_t lsdata1;//Conversion data from DS2450-Least sig byte--temp or photo
    uint8_t msdata2;//Conversion data from DS2450-Most sig byte--flow
    uint8_t lsdata2;//Conversion data from DS2450-Least sig byte--flow
  } SalamanderMsg;

  command result_t StdControl.init()
  {
    m_sending = FALSE;
    call Leds.init();
    call SplitControl.init();
    return SUCCESS;
  }

  command result_t StdControl.start()
  {
    call SequencerOscOn.init();
    call SequencerOscOn.output_low(); //turn off oscillator using ADC pin -for later board
    call SensorChainOn.init();  //turn on sensor chain using another ADC pin-for later board
    call SensorChainOn.output_high();
    call TripleSens.init(); //populate list of sensor ID numbers on startup only. Toggle power switch to detect newly attached sensors.
    call Timer.start( TIMER_REPEAT, shortTime ); //1000=1 second.  

    return SUCCESS;
  }

  command result_t StdControl.stop()
  {
    return SUCCESS;
    call SensorChainOn.output_low();//turn off sensor chain on new board, save pwr on later board
  }

  event result_t Timer.fired()
  {
    if( m_sending == FALSE )
    {       
      SalamanderMsg* body = (SalamanderMsg*)m_msg.data;
      body->src = TOS_LOCAL_ADDRESS;
      if( call TripleSens.getdata(whichrom) == SUCCESS)
      {  whichrom=whichrom+1;
         if (whichrom==1){
         call Timer.stop();
         call Timer.start (TIMER_REPEAT, shortTime);
         }
         call TripleSens.copy_id( &(body->id[1]) );
         body->id[0] = call TripleSens.get_crc();
         body->id[7] = call TripleSens.get_family();
	   body->msdata1 = call TripleSens.get_msdata1();
         body->lsdata1 = call TripleSens.get_lsdata1();
         body->msdata2 = call TripleSens.get_msdata2();
         body->lsdata2 = call TripleSens.get_lsdata2();
         call SplitControl.start();
         if( call SendMsg.send( TOS_BCAST_ADDR, sizeof(SalamanderMsg), &m_msg ) == SUCCESS )//osc circuit on future board works if this commented out
         {
	     m_sending = TRUE;
	     //call Leds.redToggle();  //shut this off to save power
         }
      }
      else {
           whichrom=0;
           call Timer.stop();
           call Timer.start (TIMER_ONE_SHOT,longTime);
           }
     
    }
    return SUCCESS;
  }

  event result_t SendMsg.sendDone( TOS_MsgPtr msg, result_t success )
  {
    m_sending = FALSE;
    call SplitControl.stop();
    return SUCCESS;
  }

  event result_t SplitControl.initDone()
  {
  return SUCCESS;
  }

  event result_t SplitControl.startDone()
  {
  return SUCCESS;
  }
 
  event result_t SplitControl.stopDone()
  {
  return SUCCESS;
  }
}

