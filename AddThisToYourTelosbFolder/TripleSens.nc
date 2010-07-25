//$Id: TripleSens.nc,v 1.0 2010/05/14 22:18:07 ckharnett Exp $
//@author ckharnett <c0harn01@louisville.edu>

interface TripleSens
{
  command result_t init();
  command result_t getdata(int whichrom);

  command void copy_id( uint8_t* id ); // 6 bytes
  command uint8_t get_lsdata1(); 
  command uint8_t get_msdata1();
  command uint8_t get_lsdata2(); 
  command uint8_t get_msdata2();
  command uint8_t get_id_byte( uint8_t index );

  command uint8_t get_family();

  command uint8_t get_crc();
  command bool is_crc_okay();
}

