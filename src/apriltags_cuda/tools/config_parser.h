#pragma once

#include <string>
#include <map>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <cctype>
#include <iostream>

// Simple config parser for JSON-like config file
// Since we don't want heavy dependencies, we'll parse a simple format
class ConfigParser {
public:
  ConfigParser(const std::string& config_file) {
    load_config(config_file);
  }
  
  std::string get_string(const std::string& key, const std::string& default_val = "") const {
    auto it = values_.find(key);
    if (it != values_.end()) return it->second;
    return default_val;
  }
  
  double get_double(const std::string& key, double default_val = 0.0) const {
    auto it = values_.find(key);
    if (it != values_.end()) {
      return std::atof(it->second.c_str());
    }
    return default_val;
  }
  
  int get_int(const std::string& key, int default_val = 0) const {
    auto it = values_.find(key);
    if (it != values_.end()) {
      return std::atoi(it->second.c_str());
    }
    return default_val;
  }
  
  bool get_bool(const std::string& key, bool default_val = false) const {
    auto it = values_.find(key);
    if (it != values_.end()) {
      std::string val = it->second;
      // Convert to lowercase for comparison
      for (char& c : val) c = std::tolower(c);
      return (val == "true" || val == "1" || val == "yes");
    }
    return default_val;
  }
  
private:
  std::map<std::string, std::string> values_;
  
  void load_config(const std::string& config_file) {
    std::ifstream file(config_file);
    if (!file.is_open()) {
      std::cerr << "Warning: Could not open config file: " << config_file 
                << ", using defaults\n";
      return;
    }
    
    std::string line;
    std::string current_section;
    
    while (std::getline(file, line)) {
      // Remove comments
      size_t comment_pos = line.find("//");
      if (comment_pos != std::string::npos) {
        line = line.substr(0, comment_pos);
      }
      
      // Trim whitespace
      line.erase(0, line.find_first_not_of(" \t"));
      line.erase(line.find_last_not_of(" \t") + 1);
      
      if (line.empty()) continue;
      
      // Check for section header [section]
      if (line[0] == '[' && line.back() == ']') {
        current_section = line.substr(1, line.length() - 2);
        continue;
      }
      
      // Parse key: value or "key": value
      size_t colon_pos = line.find(':');
      if (colon_pos != std::string::npos) {
        std::string key = line.substr(0, colon_pos);
        std::string value = line.substr(colon_pos + 1);
        
        // Remove quotes and commas from key
        key.erase(0, key.find_first_not_of(" \t\""));
        key.erase(key.find_last_not_of(" \t\"") + 1);
        
        // Remove quotes, commas, and trailing chars from value
        value.erase(0, value.find_first_not_of(" \t\""));
        size_t last_char = value.find_last_not_of(" \t\",");
        if (last_char != std::string::npos) {
          value = value.substr(0, last_char + 1);
        }
        
        // Add section prefix if in a section
        std::string full_key = current_section.empty() ? key : current_section + "." + key;
        values_[full_key] = value;
      }
    }
  }
};

