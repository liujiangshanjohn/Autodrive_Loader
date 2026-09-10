#include <dirent.h>
#include <fcntl.h>
#include <glob.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstddef>
#include <fstream>
#include <string>
#include "log.h"

#include "google/protobuf/util/json_util.h"
#include <google/protobuf/text_format.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>




bool GetProtoFromASCIIFile(const std::string &file_name,
                           google::protobuf::Message *message) {
  using google::protobuf::TextFormat;
  using google::protobuf::io::FileInputStream;
  using google::protobuf::io::ZeroCopyInputStream;
  int file_descriptor = open(file_name.c_str(), O_RDONLY);
  if (file_descriptor < 0) {
    //AWARN << "Failed to open file " << file_name << " in text mode.";

    LOG_E("Failed to open file, file name = {}", file_name);
    // Failed to open;
    return false;
  }

  ZeroCopyInputStream *input = new FileInputStream(file_descriptor);
  bool success = TextFormat::Parse(input, message);
  if (!success) {
    LOG_E("Failed to parse file, file name = {}", file_name);
  }
  delete input;
  close(file_descriptor);
  return success;
}