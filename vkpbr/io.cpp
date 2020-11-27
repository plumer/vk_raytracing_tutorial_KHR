#include "io.h"

#include "logging.h"
#include <fstream>

namespace io {

std::string FindFile(const std::string& file_name, const std::vector<std::string>& directories)
{
    std::ifstream file_in;
    // See if the file is in the current working directory.
    file_in.open(std::string(file_name), std::ios::in);
    if (file_in.is_open()) {
        return std::string(file_name);
    }

    // Sees if the file is in some of the directories provided.
    for (const auto& dir : directories) {
        std::string full_path = dir + '/' + file_name;
        file_in.open(full_path, std::ios::in);
        if (file_in.is_open()) {
            return full_path;
        }
    }
    return {};
}


std::string LoadBinaryFile(const std::string&              file_name,
                               const std::vector<std::string>& directories)
{
    auto full_path = FindFile(file_name, directories);
    if (full_path.empty()) {
        LOG(ERROR) << "file named " << file_name << " isn't found in the given directories";
        return {};
    } else {
        std::string   result;
        std::ifstream stream(full_path, std::ios::ate | std::ios::binary);
        if (!stream.is_open())
            return result;
        result.reserve(stream.tellg());
        stream.seekg(0, std::ios::beg);
        result.assign((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
        return result;
    }
}

std::string LoadTextFile(const std::string&              file_name,
                             const std::vector<std::string>& directories)
{
    auto full_path = FindFile(file_name, directories);
    if (full_path.empty()) {
        LOG(ERROR) << "file named " << file_name << " isn't found in given directories";
        return "";
    } else {
        std::string   text;
        std::ifstream stream(full_path, std::ios::ate);
        CHECK(stream.is_open());
        text.reserve(stream.tellg());
        stream.seekg(0, std::ios::beg);
        text.assign(std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>());
        return text;
    }
}

}  // namespace io