#include "stream_image_communication/CamConfig.h"
#include "utils.h"

#include <fmt/core.h>
#include <fmt/color.h>

#include <fstream>

CamConfig::CamConfig(std::string config_path) {

    std::ifstream json_file;
    json_file.open(config_path, std::ifstream::in);
    if (!json_file.is_open()) {
        alert::critic_runtime_error("Json configuration file is not opened.");
    }

    Json::Reader config_reader;
    if (!config_reader.parse(json_file, config_root)) {
        alert::critic_runtime_error("Json file cannot be parsed.");
    }

    if (config_root.isMember("camera_index")) {
        camera_index = config_root["camera_index"];
    } else {
        alert::critic_runtime_error("camera_index is not a member of configuration.json.");
    }

    if (config_root.isMember("resolution")) {
        resolution = config_root["resolution"];
    } else {
        alert::critic_runtime_error("resolution is not a member of configuration.json.");
    }

    json_file.close();

}

const std::string CamConfig::leftCameraFdIdx() const {

    if (!camera_index.isMember("left")) {
        alert::critic_runtime_error("left is not a member of \"camera_index\"");
    }

    return camera_index["left"].asString();
}

const std::string CamConfig::rightCameraFdIdx() const {

    if (!camera_index.isMember("right")) {
        alert::critic_runtime_error("right is not a member of \"camera_index\"");
    }

    return camera_index["right"].asString();
}

const uint32_t CamConfig::camWidth() const {

    if (!resolution.isMember("width")) {
        alert::critic_runtime_error("width is not a member of \"resolution\"");
    }

    return resolution["width"].asUInt();
}

const uint32_t CamConfig::camHeight() const {

    if (!resolution.isMember("height")) {
        alert::critic_runtime_error("height is not a member of \"resolution\"");
    }

    return resolution["height"].asUInt();
}

const uint8_t CamConfig::camFps() const {

    if (!config_root.isMember("fps")) {
        alert::critic_runtime_error("fps is not a list of json file.");
    }

    return static_cast<uint8_t>(config_root["fps"].asUInt());
}
