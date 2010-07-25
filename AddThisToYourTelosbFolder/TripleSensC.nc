//$Id: TripleSensC.nc,v 1.0 2010/05/14 22:18:07 ckharnett Exp $
//@author ckharnett <c0harn01@louisville.edu>

configuration TripleSensC
{
  provides interface TripleSens;
}
implementation
{
  components TripleSensM, SensorChainPinC;
  TripleSens = TripleSensM;
  TripleSensM.SensorChainPin -> SensorChainPinC.SensorChainPin;
}

