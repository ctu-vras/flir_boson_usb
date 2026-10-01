// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <cstring>
#include <ios>
#include <sstream>
#include <stdexcept>
#include <string>

typedef bool _Bool;

extern "C" {
#include <Client_API.h>
#include <EnumTypes.h>
#include <UART_Connector.h>

// FSLP_64.so is a pure C library, the declaration must not be C++-mangled.
int32_t FSLP_lookup_port_id(char*port_name, int32_t len);
}

#include <flir_boson_usb/boson_api.hpp>

template<int N>
std::string to_string(const uint8_t value[N]) {
  std::ostringstream convert;
  for (int a = 0; a < N; a++) {
    convert << std::uppercase << std::hex << static_cast<int>(value[a]);
  }
  return convert.str();
}

namespace flir_boson_usb {

BosonAPI::BosonAPI(const std::string& uart_port_name) {
  if (uart_port_name.size() > 16) {
    throw std::runtime_error("UART port name is too long");
  }
  char name[16];
  strncpy(name, uart_port_name.c_str(), uart_port_name.length());
  const auto port_id = FSLP_lookup_port_id(name, uart_port_name.size());
  if (port_id == -1) {
    throw std::runtime_error("Failed to find UART port");
  }

  Initialize(port_id, 921600);
}

BosonAPI::~BosonAPI() {
  Close();
}

std::string BosonAPI::getCameraProductNumber() const {
  FLR_BOSON_PARTNUMBER_T pn;
  if (const auto res = bosonGetCameraPN(&pn); res != R_SUCCESS) {
    throw std::runtime_error("Failed to get camera product number");
  }
  return to_string<sizeof(pn.value)>(pn.value);
}

uint32_t BosonAPI::getCameraSerialNumber() const {
  uint32_t sn;
  if (const auto res = bosonGetCameraSN(&sn); res != R_SUCCESS) {
    throw std::runtime_error("Failed to get camera serial number");
  }
  return sn;
}

}  // namespace flir_boson_usb
