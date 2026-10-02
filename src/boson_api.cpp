// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <array>
#include <cstring>
#include <stdexcept>
#include <string>

#ifdef HAS_BOSON_SDK
typedef bool _Bool;

extern "C" {
#include <Client_API.h>
#include <EnumTypes.h>
#include <UART_Connector.h>

int32_t FSLP_lookup_port_id(char*port_name, int32_t len);
}
#endif

#include <flir_boson_usb/boson_api.hpp>

std::string to_string(const uint8_t* value, const size_t len) {
  return std::string{reinterpret_cast<const char*>(value), len};
}

namespace flir_boson_usb {

BosonAPI::BosonAPI(const std::string& uart_port_name) {
#ifdef HAS_BOSON_SDK
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
#else
  throw std::runtime_error("Boson SDK not available");
#endif
}

BosonAPI::~BosonAPI() {
#ifdef HAS_BOSON_SDK
  Close();
#endif
}

bool BosonAPI::isValid() const {
  return hasSDK();
}

bool BosonAPI::hasSDK() const {
#ifdef HAS_BOSON_SDK
  return true;
#else
  return false;
#endif
}

BosonAPI::operator bool() const {
  return hasSDK();
}

cras::expected<std::string, std::string> BosonAPI::getCameraProductNumber() const {
#ifdef HAS_BOSON_SDK
  FLR_BOSON_PARTNUMBER_T pn;
  if (const auto res = bosonGetCameraPN(&pn); res != R_SUCCESS) {
    return cras::make_unexpected("Failed to get camera product number");
  }
  return to_string(pn.value, sizeof(pn.value));
#else
  return cras::make_unexpected("Boson SDK not available");
#endif
}

cras::expected<uint32_t, std::string> BosonAPI::getCameraSerialNumber() const {
#ifdef HAS_BOSON_SDK
  uint32_t sn;
  if (const auto res = bosonGetCameraSN(&sn); res != R_SUCCESS) {
    return cras::make_unexpected("Failed to get camera serial number");
  }
  return sn;
#else
  return cras::make_unexpected("Boson SDK not available");
#endif
}

cras::expected<std::array<uint32_t, 3>, std::string> BosonAPI::getCameraFirmwareVersion() const {
#ifdef HAS_BOSON_SDK
  uint32_t major, minor, patch;
  if (const auto res = bosonGetSoftwareRev(&major, &minor, &patch); res != R_SUCCESS) {
    return cras::make_unexpected("Failed to get camera firmware version");
  }
  return std::array<uint32_t, 3>{major, minor, patch};
#else
  return cras::make_unexpected("Boson SDK not available");
#endif
}

cras::expected<double, std::string> BosonAPI::getSensorTemperature() const {
#ifdef HAS_BOSON_SDK
  int16_t temp;
  if (const auto res = bosonlookupFPATempDegCx10(&temp); res != R_SUCCESS) {
    return cras::make_unexpected("Failed to get sensor temperature");
  }
  return temp / 10.0;
#else
  return cras::make_unexpected("Boson SDK not available");
#endif
}

cras::expected<float, std::string> BosonAPI::getTimestamp(const int32_t type) const {
#ifdef HAS_BOSON_SDK
  if (type < 0 || type >= static_cast<int32_t>(FLR_BOSON_TIMESTAMPTYPE_END)) {
    return cras::make_unexpected("Invalid timestamp type");
  }
  const auto timestampType = static_cast<FLR_BOSON_TIMESTAMPTYPE_E>(type);
  float stamp;
  if (const auto res = bosonGetTimeStamp(timestampType, &stamp); res != R_SUCCESS) {
    return cras::make_unexpected("Failed to get timestamp");
  }
  return stamp;
#else
  return cras::make_unexpected("Boson SDK not available");
#endif
}

cras::expected<bool, std::string> BosonAPI::isRadiometric() const {
#ifdef HAS_BOSON_SDK
  FLR_ENABLE_E capable;
  if (const auto res = radiometryGetRadiometryCapable(&capable); res != R_SUCCESS) {
    const auto maybe_product = getCameraProductNumber();
    if (maybe_product.value_or("").length() == 15) {
      return (*maybe_product)[13] == 'R';
    }
    return cras::make_unexpected("Failed to get radiometry capability");
  }
  return capable == FLR_ENABLE;
#else
  return cras::make_unexpected("Boson SDK not available");
#endif
}

cras::expected<uint32_t, std::string> BosonAPI::getUptime() const {
#ifdef HAS_BOSON_SDK
  uint32_t uptime;
  if (const auto res = sysctrlGetUptimeSecs(&uptime); res != R_SUCCESS) {
    return cras::make_unexpected("Failed to get uptime");
  }
  return uptime;
#else
  return cras::make_unexpected("Boson SDK not available");
#endif
}

cras::expected<bool, std::string> BosonAPI::isTelemetryEnabled() const {
#ifdef HAS_BOSON_SDK
  FLR_ENABLE_E enabled;
  if (const auto res = telemetryGetState(&enabled); res != R_SUCCESS) {
    return cras::make_unexpected("Failed to get telemetry state");
  }
  return enabled == FLR_ENABLE;
#else
  return cras::make_unexpected("Boson SDK not available");
#endif
}

cras::expected<void, std::string> BosonAPI::enableTelemetry(
    const bool enable, const bool bottom, const int32_t packing) {
#ifdef HAS_BOSON_SDK
  if (packing < 0 || packing >= static_cast<int32_t>(FLR_TELEMETRY_PACKING_END)) {
    return cras::make_unexpected("Invalid telemetry packing");
  }
  const FLR_ENABLE_E state = enable ? FLR_ENABLE : FLR_DISABLE;
  const FLR_TELEMETRY_LOC_E location = bottom ? FLR_TELEMETRY_LOC_BOTTOM : FLR_TELEMETRY_LOC_TOP;
  const auto pack = static_cast<FLR_TELEMETRY_PACKING_E>(packing);

  if (enable) {
    if (const auto res = telemetrySetLocation(location); res != R_SUCCESS) {
      return cras::make_unexpected("Failed to set telemetry location");
    }
    if (const auto res = telemetrySetPacking(pack); res != R_SUCCESS) {
      return cras::make_unexpected("Failed to set telemetry packing");
    }
  }
  if (const auto res = telemetrySetState(state); res != R_SUCCESS) {
    return cras::make_unexpected("Failed to set telemetry state");
  }
  return {};
#else
  return cras::make_unexpected("Boson SDK not available");
#endif
}

}  // namespace flir_boson_usb
