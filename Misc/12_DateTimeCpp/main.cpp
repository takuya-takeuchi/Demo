#include <iomanip>
#include <iostream>
#include <chrono>
#include <ctime>

void output(const std::string header, const std::tm* tm, const uint32_t ms = 0)
{
    // struct tm
    // {
    //     int tm_sec;   // seconds after the minute - [0, 60] including leap second
    //     int tm_min;   // minutes after the hour - [0, 59]
    //     int tm_hour;  // hours since midnight - [0, 23]
    //     int tm_mday;  // day of the month - [1, 31]
    //     int tm_mon;   // months since January - [0, 11]
    //     int tm_year;  // years since 1900
    //     int tm_wday;  // days since Sunday - [0, 6]
    //     int tm_yday;  // days since January 1 - [0, 365]
    //     int tm_isdst; // daylight savings time flag
    // };

    std::cout << header.c_str();
    std::cout << tm->tm_year + 1900;
    std::cout << "/";
    std::cout << std::setfill('0') << std::right << std::setw(2) << tm->tm_mon;
    std::cout << "/";
    std::cout << std::setfill('0') << std::right << std::setw(2) << tm->tm_mday;
    std::cout << " ";
    std::cout << std::setfill('0') << std::right << std::setw(2) << tm->tm_hour;
    std::cout << ":";
    std::cout << std::setfill('0') << std::right << std::setw(2) << tm->tm_min;
    std::cout << ":";
    std::cout << std::setfill('0') << std::right << std::setw(2) << tm->tm_sec;
    std::cout << ".";
    std::cout << std::setfill('0') << std::right << std::setw(3) << ms << std::endl;
    // or 
    // std::cout << std::put_time(tm, "%Y/%m/%d %H:%M:%S") << std::endl;
}

int main()
{
    // get current time
    std::cout << "from time_point" << std::endl;
    const std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    const std::time_t time = std::chrono::system_clock::to_time_t(now);

    // calendar time (Greenwich Mean Time, GMT)
    std::tm* gt = std::gmtime(&time);
    output("   gmtime: ", gt);

    // calendar time (local time)
    std::tm* lt = std::localtime(&time);
    output("localtime: ", lt);

    std::cout << std::endl;
    std::cout << "from Unix Time Stamp millisecond" << std::endl;
    const std::chrono::system_clock::duration duration = now.time_since_epoch();
    const long long ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    std::cout << "       ms: " << ms << std::endl;

    const long long sec = ms / 1000;
    gt = std::gmtime(&sec);
    output("   gmtime: ", gt, ms % 1000);

    lt = std::localtime(&sec);
    output("localtime: ", lt, ms % 1000);

    return 0;
}