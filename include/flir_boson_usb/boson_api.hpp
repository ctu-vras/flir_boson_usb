// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <cstdint>
#include <string>

namespace flir_boson_usb {

class BosonAPI {
public:
  explicit BosonAPI(const std::string& uart_port_name);
  ~BosonAPI();

  std::string getCameraProductNumber() const;
  uint32_t getCameraSerialNumber() const;
};

}  // namespace flir_boson_usb
