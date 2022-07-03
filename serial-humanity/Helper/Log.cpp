#include "Log.h"

#include <iostream>
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_sinks.h>

int Log::MAXIMUM_ROTATING_FILES = 5;
int Log::ROTATING_FILE_SIZE = 5 * 1024 * 1024;
std::shared_ptr<spdlog::logger> Log::spdlogger = nullptr;

//if the `dir` is not specified, the log will be saved in `/data/log` folder.
//Pass in an parameter for dir like "/home/root" if you like
std::string Log::GetPath(std::string dir /*= "/data/log"*/)
{
	if(fs::exists(dir))
	{
		spdlog::info(dir + " already exists.");
	}
	else
	{
		bool ret = false;
		try
		{
			ret = fs::create_directory(dir);
			if(ret)
				spdlog::info(dir + " is created.");
		}
		catch (const std::exception&)
		{
			ret = false;
		}

		if(ret == false)
		{
			spdlog::error("cannot create log folder of: " + dir);
			dir = fs::current_path().string();

			spdlog::info("use current folder: " + dir);
		}
	}

	auto perms = fs::status(dir).permissions();
	if((perms & fs::perms::owner_write) == fs::perms::none
		&& (perms & fs::perms::group_write) == fs::perms::none)
	{
		spdlog::error( dir + ": log write permission denied");
		dir = fs::current_path().string();
		spdlog::error("use current folder: " + dir);
	}

	return dir;
}

bool Log::Init(std::string appName)
{
	std::string dir = GetPath();

	auto logPath = fs::path(dir) / fs::path(appName + ".log");

	auto fileSink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(logPath.string(), ROTATING_FILE_SIZE, MAXIMUM_ROTATING_FILES);
	auto consoleSink = std::make_shared<spdlog::sinks::stdout_sink_mt>();
	fileSink->set_level(spdlog::level::info);
    consoleSink->set_level(spdlog::level::trace);

	std::vector<spdlog::sink_ptr> sinks;
    sinks.push_back(fileSink);
    sinks.push_back(consoleSink);
    spdlogger = std::make_shared<spdlog::logger>(appName, sinks.begin(), sinks.end());
	spdlog::set_default_logger(spdlogger);

	spdlog::flush_on(spdlog::level::info);

	spdlog::info("build time is: " + std::string(__DATE__) + " " + std::string(__TIME__));
	spdlog::info("Log started");

    return true;
}
