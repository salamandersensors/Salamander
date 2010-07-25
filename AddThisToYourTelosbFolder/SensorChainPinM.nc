//$Id: SensorChainPinM.nc,v 1.0 2010/05/14 22:18:07 ckharnett Exp $
//@author ckharnett <c0harn01@louisville.edu>

module SensorChainPinM
{
  provides interface SensorChainPin;
  uses interface MSP430GeneralIO as MSP430Pin;
}
implementation
{
  command void SensorChainPin.init()
  {
    call MSP430Pin.selectIOFunc();
    call MSP430Pin.makeInput();
    call MSP430Pin.setLow();
  }

  command void SensorChainPin.output_low()
  {
    call MSP430Pin.makeOutput();
  }

  command void SensorChainPin.output_high()
  {
    call MSP430Pin.makeInput();
  }

  command void SensorChainPin.prepare_read()
  {
    call MSP430Pin.makeInput();
  }

  command uint8_t SensorChainPin.read()
  {
    return call MSP430Pin.getRaw();
  }
}

