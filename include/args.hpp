#pragma once
#include <string>
#include <unordered_map>
#include <stdexcept>
#include <iostream>
#include <vector>
#include <optional>

class Args {
public:
    void parse(int argc, char* argv[]) {
        for (int i = 1; i < argc; ++i) {
            std::string arg(argv[i]);

            if (arg.rfind("--", 0) == 0) {
                auto eq_pos = arg.find('=');
                if (eq_pos != std::string::npos) {
                    std::string key = arg.substr(2, eq_pos - 2);
                    std::string value = arg.substr(eq_pos + 1);
                    args_[key] = value;
                } else {
                    std::string key = arg.substr(2);
                    args_[key] = "true";  // treat as flag
                }
            } else {
                positional_.emplace_back(arg);
            }
        }
    }

    std::optional<std::string> get(const std::string& key) const {
        auto it = args_.find(key);
        if (it != args_.end()) return it->second;
        return std::nullopt;
    }

    std::string get_or(const std::string& key, const std::string& default_value) const {
        auto val = get(key);
        return val.has_value() ? *val : default_value;
    }

    bool has(const std::string& key) const {
        return args_.count(key);
    }

    const std::vector<std::string>& positional() const {
        return positional_;
    }

private:
    std::unordered_map<std::string, std::string> args_;
    std::vector<std::string> positional_;
};
