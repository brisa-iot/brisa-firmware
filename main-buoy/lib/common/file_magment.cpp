#include "file_managment.h"

static const char* TAG_LOGGER = "FileManagement"; 


esp_err_t ensure_log_directory_exists(const char* dir_path) {
    struct stat st;
    if (stat(dir_path, &st) == -1) {
        //ESP_LOGI(TAG_LOGGER, "Creating directory: %s", dir_path);
        if (mkdir(dir_path, 0777) == 0) {
            return ESP_OK;
        } else {
            ESP_LOGE(TAG_LOGGER, "Failed to create directory %s", dir_path);
            return ESP_FAIL;
        }
    }
    return ESP_OK; 
}

esp_err_t initialize_filesystem() {;
    esp_err_t ret; 

    esp_vfs_littlefs_conf_t conf = {
        .base_path = FS_BASE_PATH,             
        .partition_label = PARTITION_LABEL,  
        .format_if_mount_failed = true,
        .dont_mount = false,
    };

    ret = esp_vfs_littlefs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_LOGGER, "Failed to mount littlefs (%s)", esp_err_to_name(ret));
        /**
         * TODO: Handle the error appropriately.
         * For example, you might want to format the filesystem or take other actions.
         */
    } else {
        ESP_LOGI(TAG_LOGGER, "littlefs mounted at %s", conf.base_path);
    }

    size_t total_bytes = 0, used_bytes = 0;
    ret = esp_littlefs_info(conf.partition_label, &total_bytes, &used_bytes);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_LOGGER, "Failed to get littlefs info (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG_LOGGER, "Partition size: %d, used bytes: %d", total_bytes, used_bytes);
    }

    ret = ensure_log_directory_exists(LOGS_DIR);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_LOGGER, "Failed to ensure logs directory exists (%s)", esp_err_to_name(ret));
        return ret; 
    } else {
        ESP_LOGI(TAG_LOGGER, "Logs directory ensured at %s", LOGS_DIR);
    }

    return ESP_OK;
}


void get_and_log_partition_info(const char* label) {
    ESP_LOGI("PART_INFO", "Attempting to find partition with label: %s", label);

    const esp_partition_t *partition = esp_partition_find_first(
                                        ESP_PARTITION_TYPE_ANY,      
                                        ESP_PARTITION_SUBTYPE_ANY,   
                                        label);                     
    if (partition != NULL) {
        ESP_LOGI("PART_INFO", "--- Partition Found! ---");
        ESP_LOGI("PART_INFO", "  Label: %s", partition->label);
        ESP_LOGI("PART_INFO", "  Type: %s (%u)",
                (partition->type == ESP_PARTITION_TYPE_APP) ? "APP" : "DATA",
                partition->type);
        ESP_LOGI("PART_INFO", "  SubType: %s (%u)",
                (partition->subtype == ESP_PARTITION_SUBTYPE_APP_FACTORY) ? "FACTORY" :
                (partition->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0) ? "OTA_0" :
                (partition->subtype == ESP_PARTITION_SUBTYPE_DATA_NVS) ? "NVS" :
                (partition->subtype == ESP_PARTITION_SUBTYPE_DATA_SPIFFS) ? "SPIFFS" : "UNKNOWN",
                 partition->subtype);
        ESP_LOGI("PART_INFO", "  Address: 0x%06x", partition->address);
        ESP_LOGI("PART_INFO", "  Size: 0x%06x (%u KB)", partition->size, partition->size / 1024);
        ESP_LOGI("PART_INFO", "------------------------");
    } else {
        ESP_LOGE("PART_INFO", "Partition with label '%s' not found on device!", label);
    }
}


std::string get_date_string(uint64_t timestamp_ms) {
    time_t unix_time = timestamp_ms / 1000ULL; 
    struct tm timeinfo;

    localtime_r(&unix_time, &timeinfo);
    char date_str[9]; // YYYYMMDD + null terminator
    strftime(date_str, sizeof(date_str), "%Y%m%d", &timeinfo);
    return std::string(date_str);
}


std::vector<std::string> get_sorted_log_file_paths() {
    std::vector<std::string> files;
    // Optimize 1: Reserve capacity to avoid frequent vector reallocations.
    files.reserve(MAX_NUM_DAYS_LOG_FILES + 5); 

    DIR* dir = opendir(LOGS_DIR);
    if (dir == NULL) {
        ESP_LOGE(TAG_LOGGER, "Failed to open logs directory %s: %s", LOGS_DIR, strerror(errno));
        return files;
    }

    struct dirent* entry;
    // Optimize 2: Use a fixed-size C-style buffer for constructing full paths.
    char full_path_buffer[256]; // Ensure this buffer is large enough for your longest possible path

    while ((entry = readdir(dir)) != NULL) {

        if (entry->d_type != DT_REG) {
            //ESP_LOGD(TAG_LOGGER, "Skipping non-regular entry: %s (Type: %d)", entry->d_name, entry->d_type);
            continue; 
        }

        std::string filename_str = entry->d_name;
        
        // File names are YYYYMMDD_imu_log.csv
        // Length: 8 (YYYYMMDD) + 4 (_imu) + 4 (_log) + 4 (.csv) = 20 characters
        const size_t EXPECTED_LENGTH = 20;
        const std::string SUFFIX_PART1 = "_imu";
        const std::string SUFFIX_PART2 = "_log.csv";

        if (filename_str.length() == EXPECTED_LENGTH &&
            filename_str.substr(8, SUFFIX_PART1.length()) == SUFFIX_PART1 && // Check for "_imu"
            filename_str.substr(8 + SUFFIX_PART1.length(), SUFFIX_PART2.length()) == SUFFIX_PART2) { // Check for "_log.csv"
            
            bool is_date_numeric = true;
            for (int i = 0; i < 8; ++i) {
                if (!isdigit(filename_str[i])) {
                    is_date_numeric = false;
                    break;
                }
            }

            if (is_date_numeric) {
                // Optimize 3: Use snprintf to concatenate directly into the buffer.
                int chars_written = snprintf(full_path_buffer, sizeof(full_path_buffer), "%s/%s", LOGS_DIR, filename_str.c_str());
                if (chars_written > 0 && chars_written < sizeof(full_path_buffer)) {
                    files.push_back(full_path_buffer); // Constructs std::string from char*
                    //ESP_LOGD(TAG_LOGGER, "Added matching log file: %s", full_path_buffer); // Debug log
                } else {
                    //ESP_LOGW(TAG_LOGGER, "Buffer too small (%d chars) or error for path: %s/%s", chars_written, LOGS_DIR, filename_str.c_str());
                }
            } else {
                //ESP_LOGD(TAG_LOGGER, "Skipping file %s: Non-numeric date prefix.", filename_str.c_str()); // Debug log
            }
        } else {
            //ESP_LOGD(TAG_LOGGER, "Skipping file %s: Does not match expected format or length.", filename_str.c_str()); // Debug log
        }
    }
    closedir(dir); 
    
    // Sort files lexicographically. Since names are YYYYMMDD_imu_log.csv, this sorts by date.
    std::sort(files.begin(), files.end());

    return files; 
}