#pragma once

#ifdef WIN32
#include <windows.h>
#endif

#include <chrono>
#include <string>
#include <vector>
#include <map>

class PerformanceMetrics {
public:
    static void startMeasurement(const std::string& label);
    static double endMeasurement(const std::string& label);
    static void logResults();
    static void resetMetrics();
    
    struct Measurement {
        double totalTime = 0.0;
        int count = 0;
        double minTime = 1.7976931348623157E+308;  // DBL_MAX value
        double maxTime = 0.0;
    };

private:
    static std::map<std::string, std::chrono::time_point<std::chrono::high_resolution_clock>> startTimes;
    static std::map<std::string, Measurement> measurements;
    
    // Helper functions to avoid macro conflicts
    static double getMin(double a, double b) { return (a < b) ? a : b; }
    static double getMax(double a, double b) { return (a > b) ? a : b; }
};