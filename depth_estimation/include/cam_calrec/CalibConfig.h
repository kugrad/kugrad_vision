#ifndef __CAM_CONFIG_H__
#define __CAM_CONFIG_H__

#include <json/json.h>
#include <string>

class CalibConfig {
private:
    Json::Value config_root;
    Json::Value chessboard;

public:
    CalibConfig() = delete;
    CalibConfig(std::string config_path);
    ~CalibConfig();

    const int numHorizontalCorner() const;
    const int numVerticalCorner() const;
    const int chessboardSquareLength() const;
};

#endif
