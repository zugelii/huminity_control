# Helper
Helper files

# Build

cmake version: 3.12.2, which is the cmake version in yocto 2.6.

# Spdlog
Use spdlog as the common log library. https://github.com/gabime/spdlog

eCal is using an old version of `spdlog` internally as a library, to avoid the conflict with it, `spdlog` is used here as **headers**.

to use it, include this repo as a submodule,
```
git submodule add git@github.conti.de:Corp-SnT-SmartRoad/Helper.git Helper
git submodule update --init --recursive
```
then in the CMakeLists.txt:
```
add_subdirectory(Helper)
target_link_libraries(${PROJECT_NAME} Helper)
```

call `Log::Init()` at the beginning of the code,
```
#include <Log.h>	//from Helper
Log::Init("MyAppName");
```
Then use `spdlog` functions like:
```
spdlog::info("some thing happened");
spdlog::warn("some warning");
spdlog::error("some error");
```
Then a log file named `MyAppName.log` is created in the same folder of the app (_currently_).

## boost
`boost` will create big temp file size in yocto so it's removed.

Instead, `<experimental/filesystem>` is used and can be compiled with _gcc5_ and _gcc8_ (yocto).

Add in the CMakeLists.txt file:
```cmake
target_link_libraries(${PROJECT_NAME} stdc++fs)
```
