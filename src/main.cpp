#define MAVLINK_USE_MESSAGE_INFO
#include <cstdint>
#include <fstream>
#include <iostream>
#include <unordered_set>
#include <vector>

#include <args.hpp>
#include <common/mavlink.h>
#include <dumper.hpp>
#include <vis.hpp>


void print_usage() {
    std::cout << "Usage:\n";
    std::cout << "  --file=<path>                  Path to .tlog file (required)\n";
    std::cout << "  --dump --names                 List all unique message types\n";
    std::cout << "  --dump --item=<MSG_NAME>       Print all data for one message type\n";
    std::cout << "  --help                         Show this message\n";
}

[[noreturn]] void throw_user_error(const std::string &msg) {
  std::cerr << "Error: " << msg << "\n\n";
  print_usage();
  std::exit(1);
}

std::fstream open_file(const std::string &filename) {
  if (filename.empty()) {
    throw_user_error("File name is empty.");
  }

  std::fstream file(filename, std::ios::in | std::ios::binary);
  if (!file.is_open()) {
    throw_user_error("Failed to open file: " + filename);
  }

  file.clear();
  file.seekg(0, std::ios::beg);
  if (!file) {
    throw_user_error("Failed to rewind file: " + filename);
  }

  return file;
}

int main(int argc, char *argv[]) {
  Args args;

  try {
    args.parse(argc, argv);
  } catch (const std::exception &e) {
    throw_user_error("Argument parsing failed: " + std::string(e.what()));
  }

  if (args.has("help") || !args.has("file")) {
    print_usage();
    return 0;
  }

  const std::string filename = args.get_or("file", "");
  // TODO I will need to do this smarter, probably steam it. Log files can be massive in size.
  std::fstream file = open_file(filename);
  Dumper dumper;

  if (args.has("dump") || args.has("replay")) {
    if (args.has("item")) {
      const std::string msg_name = args.get_or("item", "");
      if (msg_name.empty()) {
        throw_user_error("You must specify a message name for --item");
      }
      bool hadIssue = false;
      dumper.dump_all_messages(file, msg_name, hadIssue);
      if (hadIssue)
        throw_user_error(
            "Item name was not found, double check the item word i.e ALTITUDE");
    } else if (args.has("names")) {
      std::vector<std::string> msg_list =
          dumper.collect_mavlink_message_names(file);
      std::cout << "Available message names (" << msg_list.size() << "):\n";
      for (const auto &name : msg_list) {
        std::cout << "- " << name << "\n";
      }
    } else if (args.has("replay")) {
      Vis vis;
      std::vector<StateFrame> frames = vis.collect_state_frames(file);
      vis.visualize_attitude(frames);

    } else {
      throw_user_error(
          "Missing sub-argument to --dump (use --names or --item=MSG_NAME)");
    }
  } else {
    throw_user_error("Missing --dump argument (use --dump --names or --dump "
                     "--item=MSG_NAME)");
  }

  file.close();
  return 0;
}
