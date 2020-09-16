#include "tokenizer.h"

#include <filesystem>
#include <fstream>
#include <cassert>
#include "tree.h"

namespace pbr {
    
void Tokenizer::set_directory(const std::string &file_name) {
    file_root_path_ = std::filesystem::path(file_name).make_preferred();
    file_root_path_ = file_root_path_.parent_path();
    std::cout << "PBRT file root directory = " << file_root_path_.string() << std::endl;
    file_name_stack_.push_back(file_name);
}

bool Tokenizer::push_include_file(const std::string &file_name) {
    std::string stripped = strip_quotes(file_name);
    
    std::string absolute_path = (file_root_path_ / stripped).string();
    std::ifstream file_in(absolute_path);
    assert(file_in);
    file_streams_stack_.push_back(std::move(file_in));
    
    yy_buffer_state* new_yy_buffer = yy_create_buffer(file_streams_stack_.back(), 16384);
    yypush_buffer_state(new_yy_buffer);
    
    file_name_stack_.push_back(absolute_path);
    lineno_stack_.push_back(yylineno);
    
    yylineno = 1;
    return true;
}

bool Tokenizer::pop_include_file() {
    assert(!file_name_stack_.empty());
    if (file_name_stack_.size() == 1)
        return false;
    std::string fn = file_name_stack_.back();
    
    std::cout << "popping back to file " << fn << ", fn_stack.size() = " << file_name_stack_.size()
            << ", fs_stack_.size() = " << file_streams_stack_.size() << std::endl;
    
    file_name_stack_.pop_back();
    file_streams_stack_.pop_back();
    yylineno = lineno_stack_.back();
    lineno_stack_.pop_back();
    
    yypop_buffer_state();
    return true;
}

}