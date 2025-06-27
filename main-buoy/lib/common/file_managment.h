#ifndef FILE_MANAGMENT_H
#define FILE_MANAGMENT_H

#include <vector>
#include <string>
#include <algorithm> 
#include <cstdio>    
#include <dirent.h>  
#include <sys/stat.h>   
#include <cstring> 
#include <cerrno>  
#include "esp_err.h"
#include "esp_log.h"
#include "esp_littlefs.h"
#include "esp_partition.h"
#include "params_config.h"


/**
 * @brief Ensure that the logs directory "/spiffs/logs" exists.
 */
esp_err_t ensure_log_directory_exists();
/**
 * @brief Initialize the file system.
 */
esp_err_t initialize_filesystem();
/**
 * @brief Get and log partition information for a specific label.
 * 
 * @param label The label of the partition to find and log information about. (i.e., "storage", "spiffs")
 */
void get_and_log_partition_info(const char* label);

/**
 * @brief Get the date string in YYYYMMDD format from a timestamp in milliseconds.
 * 
 * @param timestamp_ms Timestamp in milliseconds since epoch.
 * @return std::string Date string in YYYYMMDD format.
 */
std::string get_date_string(uint64_t timestamp_ms); 

/**
 * @brief Get the full path to the logs directory.
 * This function retrieves the full path to the logs directory in the following format:
 * "/spiffs/logs/YYYYMMDD_imu_log.csv", where YYYYMMDD is the date in year-month-day format.
 * 
 * @return Full path to the logs directory.
 */
std::vector<std::string> get_sorted_log_file_paths(); 

#endif // FILE_MANAGMENT_H