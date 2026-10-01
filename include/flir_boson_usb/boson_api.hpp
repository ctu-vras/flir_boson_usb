// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <array>
#include <cstdint>
#include <string>

namespace flir_boson_usb {

class BosonAPI {
public:
  explicit BosonAPI(const std::string& uart_port_name);
  ~BosonAPI();

  std::string getCameraProductNumber() const;
  uint32_t getCameraSerialNumber() const;
  std::array<uint32_t, 3> getCameraFirmwareVersion() const;
  double getSensorTemperature() const;
  bool isRadiometric() const;
  uint32_t getUptime() const;
  bool isTelemetryEnabled() const;
  void enableTelemetry(bool enable, bool bottom = true, int32_t packing = 0, bool swap_bytes = false);
};

}  // namespace flir_boson_usb
