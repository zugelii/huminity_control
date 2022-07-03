/*
 * @Author: gabe
 * @Date: 2022-06-25 09:17:29
 * @LastEditors: gabe
 * @LastEditTime: 2022-06-25 11:39:52
 * @Description: 
 */
#pragma once

#include <string>

#include <spdlog/spdlog.h>

class Log
{
public:
	static int MAXIMUM_ROTATING_FILES;
	static int ROTATING_FILE_SIZE;
protected:
    static std::shared_ptr<spdlog::logger> spdlogger;
public:
    std::string static GetPath(std::string dir = "./log");
    bool static Init(std::string appName);
};
