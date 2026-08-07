#include <spdlog/spdlog.h>

int main()
{
    spdlog::info("spdlog Version: {}", SPDLOG_VERSION);
    return 0;
}