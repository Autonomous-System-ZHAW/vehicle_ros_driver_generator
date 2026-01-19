#include <pix_nina_driver/v_e_c_t_o_r___i_n_d_e_p_e_n_d_e_n_t__s_i_g__m_s_g.hpp>


VECTORINDEPENDENTSIGMSG::VECTORINDEPENDENTSIGMSG() {}

void VECTORINDEPENDENTSIGMSG::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void VECTORINDEPENDENTSIGMSG::Parse() {
  enabled_applications_ = enabledapplications();
}


// config detail: {'bit': 0, 'is_signed_var': False, 'len': 12, 'name': 'enabled_applications', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int VECTORINDEPENDENTSIGMSG::enabledapplications() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 4);

  Byte t1(*(bytes + 0));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

