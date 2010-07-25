//$Id: SensorChainOnC.nc,v 1.0 2010/05/14 22:18:07 ckharnett Exp $
//@author ckharnett <c0harn01@louisville.edu>

configuration SensorChainOnC
{
  provides interface SensorChainOn;
}
implementation
{
  components SensorChainOnM, MSP430GeneralIOC;

  SensorChainOn = SensorChainOnM;
  SensorChainOnM.MSP430Pin -> MSP430GeneralIOC.Port63; //this uses the ADC3 to put out 0 or 3V 
}

