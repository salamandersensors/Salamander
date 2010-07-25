//$Id: SensorChainOn.nc,v 1.0 2010/05/14 22:18:07 ckharnett Exp $
//@author ckharnett <c0harn01@louisville.edu>

//To turn on sensor chain on oscillator board

interface SensorChainOn
{
  command void init();
  command void output_low();
  command void output_high();
}

