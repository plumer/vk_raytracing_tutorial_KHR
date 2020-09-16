%{
#include <cstdio>
#include <cstdlib>
#include "syntax.tab.hpp"
#include "tokenizer.h"
#include "tree.h"

bool push_include_file(const char *);
#define LEX_DEBUG
#ifndef LEX_DEBUG
#define ldebug(...) \
    do { printf(__VA_ARGS__); putchar('\n'); } while (0)
#else
#define ldebug(...) 0
#endif

using token = pbr::Parser::token_type;

#undef YY_DECL
#define YY_DECL int pbr::Tokenizer::yylex(Parser::semantic_type *const lval)
%}

%option yylineno
%option debug
%option nodefault
%option yyclass="pbr::Tokenizer"
%option never-interactive
%option c++

%%

Include                     {
    ldebug("KW: Include");
    // printf("dummy lval\n");
    Parser::semantic_type dummy_lval;
    yylex(&dummy_lval);
    printf("file name after include: %s\n", yytext);
    push_include_file(dummy_lval.as<std::string>());
    dummy_lval.destroy<std::string>();
}

LookAt              {ldebug("KW: Lookat");        return token::KW_LOOKAT;}
Camera              {ldebug("KW: camera");        return token::KW_CAMERA;}
Integrator          {ldebug("KW: Integrator");    return token::KW_INTEGRATOR;}
Accelerator         {ldebug("KW: Accelerator");   return token::KW_ACCELERATOR;}
Sampler             {ldebug("KW: Sampler");       return token::KW_SAMPLER;}
Film                {ldebug("KW: Film");          return token::KW_FILM;}
PixelFilter         {ldebug("KW: Filter");        return token::KW_FILTER;}
Filter              {ldebug("Should not appear"); std::exit(1);}

WorldBegin          {ldebug("KW: WBegin");  return token::KW_WORLD_BEG;}
WorldEnd            {ldebug("KW: WEnd  ");  return token::KW_WORLD_END;}

AttributeBegin      {ldebug("KW: ABegin");      return token::KW_ATTR_BEG;}
AttributeEnd        {ldebug("KW: AEnd  ");      return token::KW_ATTR_END;}
TransformBegin      {ldebug("KW: TransBegin");  return token::KW_TRANS_BEG;}
TransformEnd        {ldebug("KW: TransEnd");    return token::KW_TRANS_END;}

LightSource         {ldebug("KW: LSrc");      return token::KW_LIGHT_SRC;}
AreaLightSource     {ldebug("KW: AreaLSrc");  return token::KW_AREA_LIGHT_SRC;}
Material            {ldebug("KW: Material");  return token::KW_MATERIAL;}
Shape               {ldebug("KW: Shape");     return token::KW_SHAPE;}
Texture             {ldebug("KW: Texture");   return token::KW_TEXTURE;}

Identity            {ldebug("KW: Identity");        return token::KW_IDENTITY;}
Translate           {ldebug("KW: Translate");       return token::KW_TRANSLATE;}
Scale               {ldebug("KW: Scale");           return token::KW_SCALE;}
Rotate              {ldebug("KW: Rotate");          return token::KW_ROTATE;}
CoordinateSystem    {ldebug("KW: Coord System");    return token::KW_COORD_SYSTEM;}
CoordSysTransform   {ldebug("KW: CoordSysTrans");   return token::KW_COORD_SYSTEM_TRANS;}
Transform           {ldebug("KW: Transfrom");       return token::KW_TRANSFORM;}
ConcatTransform     {ldebug("KW: ConcatTransform"); return token::KW_CONCAT_TRANSFORM;}

ReverseOrientation  {ldebug("KW: RevOrientation");  return token::KW_REVERSE_ORIENTATION;}

MediumInterface     {ldebug("KW: MediumInterface");   return token::KW_MEDIUM_INTERFACE;}
NamedMedium         {ldebug("KW: NamedMedium");       return token::KW_NAMED_MEDIUM;}
MakeNamedMedium     {ldebug("KW: MakeNamedMedium");   return token::KW_MAKE_NAMED_MEDIUM;}
NamedMaterial       {ldebug("KW: NamedMaterial");     return token::KW_NAMED_MATERIAL;}
MakeNamedMaterial   {ldebug("KW: MakeNamedMaterial"); return token::KW_MAKE_NAMED_MATERIAL;}

ObjectBegin         {ldebug("KW: OBegin");          return token::KW_OBJ_BEG;}
ObjectEnd           {ldebug("KW: OEnd  ");          return token::KW_OBJ_END;}
ObjectInstance      {ldebug("KW: ObjectInstance");  return token::KW_OBJ_INST;}

"["                 {ldebug("TOKEN: LB");  return '[';}
"]"                 {ldebug("TOKEN: RB");  return ']';}
[ \n\t\r]           {}

\-?[0-9]+(\.[0-9]*)?(e[\+\-]?[0-9]+)?        {ldebug("FLOAT: %s", yytext); 
    float f_value = std::atof(yytext);
    assert(lval);
    lval->build(f_value);
    return token::NUMBER;
}

\-?[0-9]*\.[0-9]+(e[\+\-]?[0-9]+)?           {ldebug("FLOAT: %s", yytext);
    assert(lval);
    float f_value = std::atof(yytext);
    lval->build(f_value);
    return token::NUMBER;
}

\"[^\"]*\"  {ldebug("STR: %s\n", yytext);
    assert(yytext[0] == '\"');
    assert(yytext[yyleng-1] == '\"');
    yytext[yyleng-1] = '\0';
    assert(lval);
    lval->build(std::string(yytext+1));
    return token::STRLIT;
}

\#[^\n]*\n                  {ldebug("comments");}

. {printf("Unrecognized token \'%s\' at line %d\n", yytext, yylineno); std::exit(1);}

%%

int pbr::Tokenizer::yywrap() {
    if (pop_include_file()) {
        return 0;
    } else {
        return 1;
    }
}

int yyFlexLexer::yywrap() {
    std::cerr << "shouldn't be reached";
    return 1;
}