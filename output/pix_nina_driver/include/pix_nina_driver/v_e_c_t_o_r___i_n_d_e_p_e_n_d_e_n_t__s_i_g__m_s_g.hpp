#pragma once

#include <pix_nina_driver/Byte.hpp>
#include <iostream>

class VECTORINDEPENDENTSIGMSG {
public:
    static const uint32_t ID = 0x20000000;
    VECTORINDEPENDENTSIGMSG();
    void Parse();
    void update_bytes(uint8_t bytes_data[8]);
    // singal
    int enabled_applications_;
    

private:
    uint8_t bytes[8];
    
  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 12, 'name': 'enabled_applications', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int enabledapplications();
};



