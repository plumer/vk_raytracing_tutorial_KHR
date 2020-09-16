#if !defined(TOKENIZER_H)
#define TOKENIZER_H

#ifndef yyFlexLexerOnce
#include <FlexLexer.h>
#endif
#include <filesystem>
#include <vector>
#include <fstream>
#include "syntax.tab.hpp"


namespace pbr {

class Tokenizer : public yyFlexLexer {
  public:
    Tokenizer(std::istream *in) : yyFlexLexer(in) {};
    virtual ~Tokenizer() = default;
    
    using FlexLexer::yylex;
    
    virtual int yylex(Parser::semantic_type *const lval);
    
    int yywrap() override;
    
    void set_directory(const std::string &file_name);
    
    bool push_include_file(const std::string &file_name);
    bool pop_include_file();
    
  private:
    Parser::semantic_type *yylval = nullptr;
    std::filesystem::path file_root_path_;
    std::vector<std::string> file_name_stack_;
    std::vector<std::ifstream> file_streams_stack_;
    std::vector<int> lineno_stack_;
};

}  // namespace mj


#endif // TOKENIZER_H
