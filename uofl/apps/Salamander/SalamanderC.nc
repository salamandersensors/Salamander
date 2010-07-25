// $Id: SalamanderC.nc,v 1.0 2010/5/13 00:26:11 ckharnett Exp $

// @author CKHarnett <c0harn01@louisville.edu>

configuration SalamanderC
{
}
implementation
{
  components Main
           , SalamanderM
	   , TimerC
	   , GenericComm
	   , LedsC
	   , TripleSensC
           , SequencerOscOnC
	   , SensorChainOnC
           , CC2420ControlM
	   ;
  
  Main.StdControl -> SalamanderM;
  Main.StdControl -> GenericComm;
  Main.StdControl -> TimerC;

  SalamanderM.Timer -> TimerC.Timer[unique("Timer")];
  SalamanderM.SendMsg -> GenericComm.SendMsg[130];
  SalamanderM.Leds -> LedsC.Leds;
  SalamanderM.TripleSens -> TripleSensC;
  SalamanderM.SequencerOscOn -> SequencerOscOnC;
  SalamanderM.SensorChainOn -> SensorChainOnC;
  SalamanderM.SplitControl -> CC2420ControlM.SplitControl;
}

