//$Id: SensorChainPin.nc,v 1.0 2010/05/14 22:18:07 ckharnett Exp $
//@author ckharnett <c0harn01@louisville.edu>

interface SensorChainPin
{
  command void init();
  command void output_low();
  command void output_high();
  command void prepare_read();
  command uint8_t read(); //zero=0, nonzero=1
}

