//$Id: SequencerOscOnC.nc 11-11-2008
//@author CKH

configuration SequencerOscOnC
{
  provides interface SequencerOscOn;
}
implementation
{
  components SequencerOscOnM, MSP430GeneralIOC;

  SequencerOscOn = SequencerOscOnM;
  SequencerOscOnM.MSP430Pin -> MSP430GeneralIOC.Port62; //this uses the ADC2 to put out 0 or 3V on pin 7 of 10-pin expansion connector
}

