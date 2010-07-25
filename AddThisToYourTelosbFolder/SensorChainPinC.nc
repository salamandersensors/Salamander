//$Id: SensorChainPinC.nc,v 1.0 2010/05/14 22:18:07 ckharnett Exp $
//@author ckharnett <c0harn01@louisville.edu>

configuration SensorChainPinC
{
  provides interface SensorChainPin;
}
implementation
{
  components SensorChainPinM, MSP430GeneralIOC;

  SensorChainPin = SensorChainPinM;
  SensorChainPinM.MSP430Pin -> MSP430GeneralIOC.Port26;
}

