#ifndef __CAM_CONFIG_H__
#define __CAM_CONFIG_H__

#include <json/json.h>
#include <string>

class CamConfig {
private:
    Json::Value config_root;
    Json::Value camera_index;
    Json::Value resolution;

public:
    CamConfig() = delete;
    CamConfig(std::string config_path);
    ~CamConfig();

    const std::string leftCameraFdIdx() const;
    const std::string rightCameraFdIdx() const;

    const uint32_t camWidth() const;
    const uint32_t camHeight() const;
    const uint8_t camFps() const;

};

#endif
