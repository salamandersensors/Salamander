//$Id: SequencerOscOnM.nc,v 1.1 2008/11/11 Exp $



//author ckh

module SequencerOscOnM
{
  provides interface SequencerOscOn;
  uses interface MSP430GeneralIO as MSP430Pin;
}
implementation
{
  command void SequencerOscOn.init()
  {
    call MSP430Pin.selectIOFunc();
    call MSP430Pin.makeOutput(); 
    call MSP430Pin.setLow();  
  }

  command void SequencerOscOn.output_low()
  {
    call MSP430Pin.makeOutput();
    call MSP430Pin.setLow();
  }

  command void SequencerOscOn.output_high()
  {
    call MSP430Pin.makeOutput();
    call MSP430Pin.setHigh();
  }

  }

