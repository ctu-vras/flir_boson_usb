// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <array>
#include <cstring>
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

std::string to_string(const uint8_t* value, const size_t len) {
  return std::string{reinterpret_cast<const char*>(value), len};
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
  return to_string(pn.value, sizeof(pn.value));
}

uint32_t BosonAPI::getCameraSerialNumber() const {
  uint32_t sn;
  if (const auto res = bosonGetCameraSN(&sn); res != R_SUCCESS) {
    throw std::runtime_error("Failed to get camera serial number");
  }
  return sn;
}

std::array<uint32_t, 3> BosonAPI::getCameraFirmwareVersion() const {
  uint32_t major, minor, patch;
  if (const auto res = bosonGetSoftwareRev(&major, &minor, &patch); res != R_SUCCESS) {
    throw std::runtime_error("Failed to get camera firmware version");
  }
  return {major, minor, patch};
}

double BosonAPI::getSensorTemperature() const {
  int16_t temp;
  if (const auto res = bosonlookupFPATempDegCx10(&temp); res != R_SUCCESS) {
    throw std::runtime_error("Failed to get sensor temperature");
  }
  return temp / 10.0;
}

bool BosonAPI::isRadiometric() const {
  FLR_ENABLE_E capable;
  if (const auto res = radiometryGetRadiometryCapable(&capable); res != R_SUCCESS) {
    throw std::runtime_error("Failed to get radiometry capability");
  }
  return capable == FLR_ENABLE;
}

uint32_t BosonAPI::getUptime() const {
  uint32_t uptime;
  if (const auto res = sysctrlGetUptimeSecs(&uptime); res != R_SUCCESS) {
    throw std::runtime_error("Failed to get uptime");
  }
  return uptime;
}

bool BosonAPI::isTelemetryEnabled() const {
  FLR_ENABLE_E enabled;
  if (const auto res = telemetryGetState(&enabled); res != R_SUCCESS) {
    throw std::runtime_error("Failed to get telemetry state");
  }
  return enabled == FLR_ENABLE;
}

void BosonAPI::enableTelemetry(const bool enable, const bool bottom, const int32_t packing, const bool swap_bytes) {
  const FLR_ENABLE_E state = enable ? FLR_ENABLE : FLR_DISABLE;
  const FLR_TELEMETRY_LOC_E location = bottom ? FLR_TELEMETRY_LOC_BOTTOM : FLR_TELEMETRY_LOC_TOP;
  const auto pack = static_cast<FLR_TELEMETRY_PACKING_E>(packing);
  const FLR_TELEMETRY_ORDER_E order = swap_bytes ? FLR_TELEMETRY_ORDER_SWAP16B : FLR_TELEMETRY_ORDER_DEFAULT;

  if (enable) {
    if (const auto res = telemetrySetLocation(location); res != R_SUCCESS) {
      throw std::runtime_error("Failed to set telemetry location");
    }
    if (const auto res = telemetrySetPacking(pack); res != R_SUCCESS) {
      throw std::runtime_error("Failed to set telemetry packing");
    }
    if (const auto res = telemetrySetOrder(order); res != R_SUCCESS) {
      throw std::runtime_error("Failed to set telemetry order");
    }
  }
  if (const auto res = telemetrySetState(state); res != R_SUCCESS) {
    throw std::runtime_error("Failed to set telemetry state");
  }
}

}  // namespace flir_boson_usb
