#define MAVLINK_USE_MESSAGE_INFO
#include "common/mavlink.h"
#include <algorithm>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

class Dumper {
public:
  void print_field(const mavlink_field_info_t &field, const void *ptr) {

    switch (field.type) {
    case MAVLINK_TYPE_FLOAT:
      std::cout << *reinterpret_cast<const float *>(ptr);
      break;
    case MAVLINK_TYPE_DOUBLE:
      std::cout << *reinterpret_cast<const double *>(ptr);
      break;
    case MAVLINK_TYPE_UINT8_T:
      std::cout << static_cast<uint32_t>(*reinterpret_cast<const uint8_t *>(
          ptr)); // cast to avoid char display
      break;
    case MAVLINK_TYPE_INT8_T:
      std::cout << static_cast<int32_t>(*reinterpret_cast<const int8_t *>(ptr));
      break;
    case MAVLINK_TYPE_UINT16_T:
      std::cout << *reinterpret_cast<const uint16_t *>(ptr);
      break;
    case MAVLINK_TYPE_INT16_T:
      std::cout << *reinterpret_cast<const int16_t *>(ptr);
      break;
    case MAVLINK_TYPE_UINT32_T:
      std::cout << *reinterpret_cast<const uint32_t *>(ptr);
      break;
    case MAVLINK_TYPE_INT32_T:
      std::cout << *reinterpret_cast<const int32_t *>(ptr);
      break;
    case MAVLINK_TYPE_UINT64_T:
      std::cout << *reinterpret_cast<const uint64_t *>(ptr);
      break;
    case MAVLINK_TYPE_INT64_T:
      std::cout << *reinterpret_cast<const int64_t *>(ptr);
      break;
    case MAVLINK_TYPE_CHAR:
      std::cout << *reinterpret_cast<const char *>(ptr);
      break;
    default:
      std::cout << "<unsupported_type>";
      break;
    }
  }

  std::vector<std::string> collect_mavlink_message_names(std::istream &input) {
    std::unordered_set<std::string>
        name_set; // <- local, gets cleared each call
    std::vector<std::string> names;
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t byte;

    while (input.read(reinterpret_cast<char *>(&byte), 1)) {
      if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
        const mavlink_message_info_t *info = mavlink_get_message_info(&msg);
        if (info) {
          std::string name(info->name);
          if (name_set.insert(name).second) {
            names.push_back(name);
          }
        }
      }
    }

    return names;
  }

  void dump_all_messages(std::istream &input,
                         const std::string &target_name_upper, bool &issue) {
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t byte;
    bool printedsome = false;
    while (input.read(reinterpret_cast<char *>(&byte), 1)) {
      if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
        const mavlink_message_info_t *info = mavlink_get_message_info(&msg);
        if (!info)
          continue;

        if (target_name_upper == info->name) {
          std::cout << info->name << ":\n";
          printedsome = true;

          for (uint8_t i = 0; i < info->num_fields; ++i) {
            const mavlink_field_info_t &field = info->fields[i];
            const void *field_ptr = _MAV_PAYLOAD(&msg) + field.structure_offset;

            std::cout << "  " << field.name << " = ";

            if (field.array_length > 0) {
              // Array field (char[], uint8_t[], etc.)
              for (int j = 0; j < field.array_length; ++j) {
                std::cout << "[" << j << "] ";
                print_field(field, static_cast<const uint8_t *>(field_ptr) +
                                       j * field.array_length);
              }
            } else {
              // Single value
              print_field(field, field_ptr);
            }

            std::cout << '\n';
          }

          std::cout << "----\n";
        }
      }
    }
    issue = !printedsome;
  }
};
