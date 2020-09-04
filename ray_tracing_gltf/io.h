#ifndef NVCOPY_IO_H
#define NVCOPY_IO_H

#include "types.h"
#include <string>
#include <string_view>
#include <vector>

namespace io {

std::string LoadTextFile(const std::string& file_name, const std::vector<std::string>& directories);
std::string LoadBinaryFile(const std::string&              file_name,
                           const std::vector<std::string>& directories);
std::string FindFile(const std::string& file_name, const std::vector<std::string>& directories);
}  // namespace io

#endif  // !NVCOPY_IO_H
