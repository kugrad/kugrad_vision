#include "cam_calrec/CalibConfig.h"
#include "utils.h"

#include <fmt/core.h>
#include <fmt/color.h>

#include <fstream>

CalibConfig::CalibConfig(std::string config_path) {

    std::ifstream json_file;
    json_file.open(config_path, std::ifstream::in);
    if (!json_file.is_open()) {
        alert::critic_runtime_error("Json configuration file is not opened.");
    }

    Json::Reader config_reader;
    if (!config_reader.parse(json_file, config_root)) {
        alert::critic_runtime_error("Json file cannot be parsed.");
    }

    if (config_root.isMember("chessboard")) {
        chessboard = config_root["chessboard"];
    } else {
        alert::critic_runtime_error("chessboard is not a member of configuration.json.");
    }

    json_file.close();
}

CalibConfig::~CalibConfig() { /* ! Currently do  nothing */}

const int CalibConfig::numHorizontalCorner() const {

    if (!chessboard.isMember("num_hor_corner")) {
        alert::critic_runtime_error("num_hor_corner is not a member of \"chessboard\"");
    }

    return chessboard["num_hor_corner"].asInt();
}

const int CalibConfig::numVerticalCorner() const {

    if (!chessboard.isMember("num_ver_corner")) {
        alert::critic_runtime_error("num_ver_corner is not a member of \"chessboard\"");
    }

    return chessboard["num_ver_corner"].asInt();
}

const int CalibConfig::chessboardSquareLength() const {

    if (!chessboard.isMember("square_length_mm")) {
        alert::critic_runtime_error("square_length_mm is not a member of \"chessboard\"");
    }

    return chessboard["square_length_mm"].asInt();
}