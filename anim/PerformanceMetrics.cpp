#ifdef WIN32
#include <windows.h>
#endif

#include "PerformanceMetrics.h"
#include "animTcl.h"

std::map<std::string, std::chrono::time_point<std::chrono::high_resolution_clock>> PerformanceMetrics::startTimes;
std::map<std::string, PerformanceMetrics::Measurement> PerformanceMetrics::measurements;

void PerformanceMetrics::startMeasurement(const std::string& label) {
    startTimes[label] = std::chrono::high_resolution_clock::now();
}

double PerformanceMetrics::endMeasurement(const std::string& label) {
    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration<double, std::milli>(endTime - startTimes[label]);
    double ms = duration.count();
    
    auto& measurement = measurements[label];
    measurement.totalTime += ms;
    measurement.count++;
    measurement.minTime = getMin(measurement.minTime, ms);
    measurement.maxTime = getMax(measurement.maxTime, ms);
    
    return ms;
}

void PerformanceMetrics::logResults() {
    for (const auto& pair : measurements) {
        const auto& label = pair.first;
        const auto& measurement = pair.second;
        double avgTime = measurement.totalTime / measurement.count;
        
        animTcl::OutputMessage("\n%s Statistics:", label.c_str());
        animTcl::OutputMessage("  Avg: %.3f ms", avgTime);
        animTcl::OutputMessage("  Min: %.3f ms", measurement.minTime);
        animTcl::OutputMessage("  Max: %.3f ms", measurement.maxTime);
        animTcl::OutputMessage("  Samples: %d", measurement.count);
    }
}

void PerformanceMetrics::resetMetrics() {
    startTimes.clear();
    measurements.clear();
}