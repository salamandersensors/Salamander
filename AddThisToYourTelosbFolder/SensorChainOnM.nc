//$Id: SensorChainOnM.nc,v 1.0 2010/05/14 22:18:07 ckharnett Exp $
//@author ckharnett <c0harn01@louisville.edu>

module SensorChainOnM
{
  provides interface SensorChainOn;
  uses interface MSP430GeneralIO as MSP430Pin;
}
implementation
{
  command void SensorChainOn.init()
  {
    call MSP430Pin.selectIOFunc();
    call MSP430Pin.makeOutput(); 
    call MSP430Pin.setLow();  
  }

  command void SensorChainOn.output_low()
  {
    call MSP430Pin.makeOutput();
    call MSP430Pin.setLow();
  }

  command void SensorChainOn.output_high()
  {
    call MSP430Pin.makeOutput();
    call MSP430Pin.setHigh();
  }

  }

