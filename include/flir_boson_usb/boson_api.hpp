// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <array>
#include <cstdint>
#include <string>

#include <cras_cpp_common/expected.hpp>

namespace flir_boson_usb {

class BosonAPI {
public:
  explicit BosonAPI(const std::string& uart_port_name);
  ~BosonAPI();

  bool isValid() const;
  bool hasSDK() const;

  explicit operator bool() const;

  cras::expected<std::string, std::string> getCameraProductNumber() const;
  cras::expected<uint32_t, std::string> getCameraSerialNumber() const;
  cras::expected<std::array<uint32_t, 3>, std::string> getCameraFirmwareVersion() const;
  cras::expected<double, std::string> getSensorTemperature() const;
  cras::expected<float, std::string> getTimestamp(int32_t type) const;
  cras::expected<bool, std::string> isRadiometric() const;
  cras::expected<uint32_t, std::string> getUptime() const;
  cras::expected<bool, std::string> isTelemetryEnabled() const;
  cras::expected<void, std::string> enableTelemetry(bool enable, bool bottom = true, int32_t packing = 0);
};

}  // namespace flir_boson_usb
