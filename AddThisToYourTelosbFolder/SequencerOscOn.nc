//$Id: SequencerOscOn.nc,v 1.0 2010/05/13 22:18:07 ckh Exp $
//@author ckh -To turn on sequencing oscillator on oscillator board

interface SequencerOscOn
{
  command void init();
  command void output_low();
  command void output_high();
}

