// A Bison parser, made by GNU Bison 3.5.1.

// Skeleton implementation for Bison LALR(1) parsers in C++

// Copyright (C) 2002-2015, 2018-2020 Free Software Foundation, Inc.

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

// As a special exception, you may create a larger work that contains
// part or all of the Bison parser skeleton and distribute that work
// under terms of your choice, so long as that work isn't itself a
// parser generator using the skeleton or a modified version thereof
// as a parser skeleton.  Alternatively, if you modify or redistribute
// the parser skeleton itself, you may (at your option) remove this
// special exception, which will cause the skeleton and the resulting
// Bison output files to be licensed under the GNU General Public
// License without this special exception.

// This special exception was added by the Free Software Foundation in
// version 2.2 of Bison.

// Undocumented macros, especially those whose name start with YY_,
// are private implementation details.  Do not rely on them.





#include "syntax.tab.hpp"


// Unqualified %code blocks.
#line 26 "syntax.ypp"

#include <iostream>
#include "tree.h"
#include "tokenizer.h"

#undef yylex
#define yylex tokenizer.yylex

#line 54 "syntax.tab.cpp"


#ifndef YY_
# if defined YYENABLE_NLS && YYENABLE_NLS
#  if ENABLE_NLS
#   include <libintl.h> // FIXME: INFRINGES ON USER NAME SPACE.
#   define YY_(msgid) dgettext ("bison-runtime", msgid)
#  endif
# endif
# ifndef YY_
#  define YY_(msgid) msgid
# endif
#endif

// Whether we are compiled with exception support.
#ifndef YY_EXCEPTIONS
# if defined __GNUC__ && !defined __EXCEPTIONS
#  define YY_EXCEPTIONS 0
# else
#  define YY_EXCEPTIONS 1
# endif
#endif



// Enable debugging if requested.
#if YYDEBUG

// A pseudo ostream that takes yydebug_ into account.
# define YYCDEBUG if (yydebug_) (*yycdebug_)

# define YY_SYMBOL_PRINT(Title, Symbol)         \
  do {                                          \
    if (yydebug_)                               \
    {                                           \
      *yycdebug_ << Title << ' ';               \
      yy_print_ (*yycdebug_, Symbol);           \
      *yycdebug_ << '\n';                       \
    }                                           \
  } while (false)

# define YY_REDUCE_PRINT(Rule)          \
  do {                                  \
    if (yydebug_)                       \
      yy_reduce_print_ (Rule);          \
  } while (false)

# define YY_STACK_PRINT()               \
  do {                                  \
    if (yydebug_)                       \
      yystack_print_ ();                \
  } while (false)

#else // !YYDEBUG

# define YYCDEBUG if (false) std::cerr
# define YY_SYMBOL_PRINT(Title, Symbol)  YYUSE (Symbol)
# define YY_REDUCE_PRINT(Rule)           static_cast<void> (0)
# define YY_STACK_PRINT()                static_cast<void> (0)

#endif // !YYDEBUG

#define yyerrok         (yyerrstatus_ = 0)
#define yyclearin       (yyla.clear ())

#define YYACCEPT        goto yyacceptlab
#define YYABORT         goto yyabortlab
#define YYERROR         goto yyerrorlab
#define YYRECOVERING()  (!!yyerrstatus_)

#line 5 "syntax.ypp"
namespace pbr {
#line 127 "syntax.tab.cpp"


  /// Build a parser object.
  Parser::Parser (Tokenizer &tokenizer_yyarg, scene_t * scene_root_yyarg)
#if YYDEBUG
    : yydebug_ (false),
      yycdebug_ (&std::cerr),
#else
    :
#endif
      tokenizer (tokenizer_yyarg),
      scene_root (scene_root_yyarg)
  {}

  Parser::~Parser ()
  {}

  Parser::syntax_error::~syntax_error () YY_NOEXCEPT YY_NOTHROW
  {}

  /*---------------.
  | Symbol types.  |
  `---------------*/

  // basic_symbol.
#if 201103L <= YY_CPLUSPLUS
  template <typename Base>
  Parser::basic_symbol<Base>::basic_symbol (basic_symbol&& that)
    : Base (std::move (that))
    , value ()
  {
    switch (this->type_get ())
    {
      case 38: // NUMBER
        value.move< float > (std::move (that.value));
        break;

      case 43: // Scene
        value.move< int > (std::move (that.value));
        break;

      case 37: // STRLIT
        value.move< std::string > (std::move (that.value));
        break;

      case 52: // Transform
        value.move< std::unique_ptr<transform_node_t>  > (std::move (that.value));
        break;

      case 67: // NumberList
        value.move< std::vector<float>  > (std::move (that.value));
        break;

      case 69: // ParamList
        value.move< std::vector<param_node_t>  > (std::move (that.value));
        break;

      case 68: // StrList
        value.move< std::vector<std::string>  > (std::move (that.value));
        break;

      case 44: // SWOptionList
        value.move< std::vector<swoption_t>  > (std::move (that.value));
        break;

      case 53: // World
      case 54: // WorldItemList
        value.move< std::vector<worlditem_node_t> > (std::move (that.value));
        break;

      case 45: // SWOptionItem
      case 46: // Camera
      case 47: // Sampler
      case 48: // Film
      case 49: // Filter
      case 50: // Integrator
      case 51: // Accel
        value.move< swoption_t > (std::move (that.value));
        break;

      case 55: // WorldItem
      case 56: // Attribute
      case 57: // Object
      case 58: // TransformPair
      case 59: // Shape
      case 60: // Light
      case 61: // AreaLight
      case 62: // Material
      case 63: // Texture
      case 64: // ParticipatingMedia
      case 65: // NamedMaterial
      case 66: // ObjectInst
        value.move< worlditem_node_t > (std::move (that.value));
        break;

      default:
        break;
    }

  }
#endif

  template <typename Base>
  Parser::basic_symbol<Base>::basic_symbol (const basic_symbol& that)
    : Base (that)
    , value ()
  {
    switch (this->type_get ())
    {
      case 38: // NUMBER
        value.copy< float > (YY_MOVE (that.value));
        break;

      case 43: // Scene
        value.copy< int > (YY_MOVE (that.value));
        break;

      case 37: // STRLIT
        value.copy< std::string > (YY_MOVE (that.value));
        break;

      case 52: // Transform
        value.copy< std::unique_ptr<transform_node_t>  > (YY_MOVE (that.value));
        break;

      case 67: // NumberList
        value.copy< std::vector<float>  > (YY_MOVE (that.value));
        break;

      case 69: // ParamList
        value.copy< std::vector<param_node_t>  > (YY_MOVE (that.value));
        break;

      case 68: // StrList
        value.copy< std::vector<std::string>  > (YY_MOVE (that.value));
        break;

      case 44: // SWOptionList
        value.copy< std::vector<swoption_t>  > (YY_MOVE (that.value));
        break;

      case 53: // World
      case 54: // WorldItemList
        value.copy< std::vector<worlditem_node_t> > (YY_MOVE (that.value));
        break;

      case 45: // SWOptionItem
      case 46: // Camera
      case 47: // Sampler
      case 48: // Film
      case 49: // Filter
      case 50: // Integrator
      case 51: // Accel
        value.copy< swoption_t > (YY_MOVE (that.value));
        break;

      case 55: // WorldItem
      case 56: // Attribute
      case 57: // Object
      case 58: // TransformPair
      case 59: // Shape
      case 60: // Light
      case 61: // AreaLight
      case 62: // Material
      case 63: // Texture
      case 64: // ParticipatingMedia
      case 65: // NamedMaterial
      case 66: // ObjectInst
        value.copy< worlditem_node_t > (YY_MOVE (that.value));
        break;

      default:
        break;
    }

  }



  template <typename Base>
  bool
  Parser::basic_symbol<Base>::empty () const YY_NOEXCEPT
  {
    return Base::type_get () == empty_symbol;
  }

  template <typename Base>
  void
  Parser::basic_symbol<Base>::move (basic_symbol& s)
  {
    super_type::move (s);
    switch (this->type_get ())
    {
      case 38: // NUMBER
        value.move< float > (YY_MOVE (s.value));
        break;

      case 43: // Scene
        value.move< int > (YY_MOVE (s.value));
        break;

      case 37: // STRLIT
        value.move< std::string > (YY_MOVE (s.value));
        break;

      case 52: // Transform
        value.move< std::unique_ptr<transform_node_t>  > (YY_MOVE (s.value));
        break;

      case 67: // NumberList
        value.move< std::vector<float>  > (YY_MOVE (s.value));
        break;

      case 69: // ParamList
        value.move< std::vector<param_node_t>  > (YY_MOVE (s.value));
        break;

      case 68: // StrList
        value.move< std::vector<std::string>  > (YY_MOVE (s.value));
        break;

      case 44: // SWOptionList
        value.move< std::vector<swoption_t>  > (YY_MOVE (s.value));
        break;

      case 53: // World
      case 54: // WorldItemList
        value.move< std::vector<worlditem_node_t> > (YY_MOVE (s.value));
        break;

      case 45: // SWOptionItem
      case 46: // Camera
      case 47: // Sampler
      case 48: // Film
      case 49: // Filter
      case 50: // Integrator
      case 51: // Accel
        value.move< swoption_t > (YY_MOVE (s.value));
        break;

      case 55: // WorldItem
      case 56: // Attribute
      case 57: // Object
      case 58: // TransformPair
      case 59: // Shape
      case 60: // Light
      case 61: // AreaLight
      case 62: // Material
      case 63: // Texture
      case 64: // ParticipatingMedia
      case 65: // NamedMaterial
      case 66: // ObjectInst
        value.move< worlditem_node_t > (YY_MOVE (s.value));
        break;

      default:
        break;
    }

  }

  // by_type.
  Parser::by_type::by_type ()
    : type (empty_symbol)
  {}

#if 201103L <= YY_CPLUSPLUS
  Parser::by_type::by_type (by_type&& that)
    : type (that.type)
  {
    that.clear ();
  }
#endif

  Parser::by_type::by_type (const by_type& that)
    : type (that.type)
  {}

  Parser::by_type::by_type (token_type t)
    : type (yytranslate_ (t))
  {}

  void
  Parser::by_type::clear ()
  {
    type = empty_symbol;
  }

  void
  Parser::by_type::move (by_type& that)
  {
    type = that.type;
    that.clear ();
  }

  int
  Parser::by_type::type_get () const YY_NOEXCEPT
  {
    return type;
  }


  // by_state.
  Parser::by_state::by_state () YY_NOEXCEPT
    : state (empty_state)
  {}

  Parser::by_state::by_state (const by_state& that) YY_NOEXCEPT
    : state (that.state)
  {}

  void
  Parser::by_state::clear () YY_NOEXCEPT
  {
    state = empty_state;
  }

  void
  Parser::by_state::move (by_state& that)
  {
    state = that.state;
    that.clear ();
  }

  Parser::by_state::by_state (state_type s) YY_NOEXCEPT
    : state (s)
  {}

  Parser::symbol_number_type
  Parser::by_state::type_get () const YY_NOEXCEPT
  {
    if (state == empty_state)
      return empty_symbol;
    else
      return yystos_[+state];
  }

  Parser::stack_symbol_type::stack_symbol_type ()
  {}

  Parser::stack_symbol_type::stack_symbol_type (YY_RVREF (stack_symbol_type) that)
    : super_type (YY_MOVE (that.state))
  {
    switch (that.type_get ())
    {
      case 38: // NUMBER
        value.YY_MOVE_OR_COPY< float > (YY_MOVE (that.value));
        break;

      case 43: // Scene
        value.YY_MOVE_OR_COPY< int > (YY_MOVE (that.value));
        break;

      case 37: // STRLIT
        value.YY_MOVE_OR_COPY< std::string > (YY_MOVE (that.value));
        break;

      case 52: // Transform
        value.YY_MOVE_OR_COPY< std::unique_ptr<transform_node_t>  > (YY_MOVE (that.value));
        break;

      case 67: // NumberList
        value.YY_MOVE_OR_COPY< std::vector<float>  > (YY_MOVE (that.value));
        break;

      case 69: // ParamList
        value.YY_MOVE_OR_COPY< std::vector<param_node_t>  > (YY_MOVE (that.value));
        break;

      case 68: // StrList
        value.YY_MOVE_OR_COPY< std::vector<std::string>  > (YY_MOVE (that.value));
        break;

      case 44: // SWOptionList
        value.YY_MOVE_OR_COPY< std::vector<swoption_t>  > (YY_MOVE (that.value));
        break;

      case 53: // World
      case 54: // WorldItemList
        value.YY_MOVE_OR_COPY< std::vector<worlditem_node_t> > (YY_MOVE (that.value));
        break;

      case 45: // SWOptionItem
      case 46: // Camera
      case 47: // Sampler
      case 48: // Film
      case 49: // Filter
      case 50: // Integrator
      case 51: // Accel
        value.YY_MOVE_OR_COPY< swoption_t > (YY_MOVE (that.value));
        break;

      case 55: // WorldItem
      case 56: // Attribute
      case 57: // Object
      case 58: // TransformPair
      case 59: // Shape
      case 60: // Light
      case 61: // AreaLight
      case 62: // Material
      case 63: // Texture
      case 64: // ParticipatingMedia
      case 65: // NamedMaterial
      case 66: // ObjectInst
        value.YY_MOVE_OR_COPY< worlditem_node_t > (YY_MOVE (that.value));
        break;

      default:
        break;
    }

#if 201103L <= YY_CPLUSPLUS
    // that is emptied.
    that.state = empty_state;
#endif
  }

  Parser::stack_symbol_type::stack_symbol_type (state_type s, YY_MOVE_REF (symbol_type) that)
    : super_type (s)
  {
    switch (that.type_get ())
    {
      case 38: // NUMBER
        value.move< float > (YY_MOVE (that.value));
        break;

      case 43: // Scene
        value.move< int > (YY_MOVE (that.value));
        break;

      case 37: // STRLIT
        value.move< std::string > (YY_MOVE (that.value));
        break;

      case 52: // Transform
        value.move< std::unique_ptr<transform_node_t>  > (YY_MOVE (that.value));
        break;

      case 67: // NumberList
        value.move< std::vector<float>  > (YY_MOVE (that.value));
        break;

      case 69: // ParamList
        value.move< std::vector<param_node_t>  > (YY_MOVE (that.value));
        break;

      case 68: // StrList
        value.move< std::vector<std::string>  > (YY_MOVE (that.value));
        break;

      case 44: // SWOptionList
        value.move< std::vector<swoption_t>  > (YY_MOVE (that.value));
        break;

      case 53: // World
      case 54: // WorldItemList
        value.move< std::vector<worlditem_node_t> > (YY_MOVE (that.value));
        break;

      case 45: // SWOptionItem
      case 46: // Camera
      case 47: // Sampler
      case 48: // Film
      case 49: // Filter
      case 50: // Integrator
      case 51: // Accel
        value.move< swoption_t > (YY_MOVE (that.value));
        break;

      case 55: // WorldItem
      case 56: // Attribute
      case 57: // Object
      case 58: // TransformPair
      case 59: // Shape
      case 60: // Light
      case 61: // AreaLight
      case 62: // Material
      case 63: // Texture
      case 64: // ParticipatingMedia
      case 65: // NamedMaterial
      case 66: // ObjectInst
        value.move< worlditem_node_t > (YY_MOVE (that.value));
        break;

      default:
        break;
    }

    // that is emptied.
    that.type = empty_symbol;
  }

#if YY_CPLUSPLUS < 201103L
  Parser::stack_symbol_type&
  Parser::stack_symbol_type::operator= (const stack_symbol_type& that)
  {
    state = that.state;
    switch (that.type_get ())
    {
      case 38: // NUMBER
        value.copy< float > (that.value);
        break;

      case 43: // Scene
        value.copy< int > (that.value);
        break;

      case 37: // STRLIT
        value.copy< std::string > (that.value);
        break;

      case 52: // Transform
        value.copy< std::unique_ptr<transform_node_t>  > (that.value);
        break;

      case 67: // NumberList
        value.copy< std::vector<float>  > (that.value);
        break;

      case 69: // ParamList
        value.copy< std::vector<param_node_t>  > (that.value);
        break;

      case 68: // StrList
        value.copy< std::vector<std::string>  > (that.value);
        break;

      case 44: // SWOptionList
        value.copy< std::vector<swoption_t>  > (that.value);
        break;

      case 53: // World
      case 54: // WorldItemList
        value.copy< std::vector<worlditem_node_t> > (that.value);
        break;

      case 45: // SWOptionItem
      case 46: // Camera
      case 47: // Sampler
      case 48: // Film
      case 49: // Filter
      case 50: // Integrator
      case 51: // Accel
        value.copy< swoption_t > (that.value);
        break;

      case 55: // WorldItem
      case 56: // Attribute
      case 57: // Object
      case 58: // TransformPair
      case 59: // Shape
      case 60: // Light
      case 61: // AreaLight
      case 62: // Material
      case 63: // Texture
      case 64: // ParticipatingMedia
      case 65: // NamedMaterial
      case 66: // ObjectInst
        value.copy< worlditem_node_t > (that.value);
        break;

      default:
        break;
    }

    return *this;
  }

  Parser::stack_symbol_type&
  Parser::stack_symbol_type::operator= (stack_symbol_type& that)
  {
    state = that.state;
    switch (that.type_get ())
    {
      case 38: // NUMBER
        value.move< float > (that.value);
        break;

      case 43: // Scene
        value.move< int > (that.value);
        break;

      case 37: // STRLIT
        value.move< std::string > (that.value);
        break;

      case 52: // Transform
        value.move< std::unique_ptr<transform_node_t>  > (that.value);
        break;

      case 67: // NumberList
        value.move< std::vector<float>  > (that.value);
        break;

      case 69: // ParamList
        value.move< std::vector<param_node_t>  > (that.value);
        break;

      case 68: // StrList
        value.move< std::vector<std::string>  > (that.value);
        break;

      case 44: // SWOptionList
        value.move< std::vector<swoption_t>  > (that.value);
        break;

      case 53: // World
      case 54: // WorldItemList
        value.move< std::vector<worlditem_node_t> > (that.value);
        break;

      case 45: // SWOptionItem
      case 46: // Camera
      case 47: // Sampler
      case 48: // Film
      case 49: // Filter
      case 50: // Integrator
      case 51: // Accel
        value.move< swoption_t > (that.value);
        break;

      case 55: // WorldItem
      case 56: // Attribute
      case 57: // Object
      case 58: // TransformPair
      case 59: // Shape
      case 60: // Light
      case 61: // AreaLight
      case 62: // Material
      case 63: // Texture
      case 64: // ParticipatingMedia
      case 65: // NamedMaterial
      case 66: // ObjectInst
        value.move< worlditem_node_t > (that.value);
        break;

      default:
        break;
    }

    // that is emptied.
    that.state = empty_state;
    return *this;
  }
#endif

  template <typename Base>
  void
  Parser::yy_destroy_ (const char* yymsg, basic_symbol<Base>& yysym) const
  {
    if (yymsg)
      YY_SYMBOL_PRINT (yymsg, yysym);
  }

#if YYDEBUG
  template <typename Base>
  void
  Parser::yy_print_ (std::ostream& yyo,
                                     const basic_symbol<Base>& yysym) const
  {
    std::ostream& yyoutput = yyo;
    YYUSE (yyoutput);
    symbol_number_type yytype = yysym.type_get ();
#if defined __GNUC__ && ! defined __clang__ && ! defined __ICC && __GNUC__ * 100 + __GNUC_MINOR__ <= 408
    // Avoid a (spurious) G++ 4.8 warning about "array subscript is
    // below array bounds".
    if (yysym.empty ())
      std::abort ();
#endif
    yyo << (yytype < yyntokens_ ? "token" : "nterm")
        << ' ' << yytname_[yytype] << " (";
    YYUSE (yytype);
    yyo << ')';
  }
#endif

  void
  Parser::yypush_ (const char* m, YY_MOVE_REF (stack_symbol_type) sym)
  {
    if (m)
      YY_SYMBOL_PRINT (m, sym);
    yystack_.push (YY_MOVE (sym));
  }

  void
  Parser::yypush_ (const char* m, state_type s, YY_MOVE_REF (symbol_type) sym)
  {
#if 201103L <= YY_CPLUSPLUS
    yypush_ (m, stack_symbol_type (s, std::move (sym)));
#else
    stack_symbol_type ss (s, sym);
    yypush_ (m, ss);
#endif
  }

  void
  Parser::yypop_ (int n)
  {
    yystack_.pop (n);
  }

#if YYDEBUG
  std::ostream&
  Parser::debug_stream () const
  {
    return *yycdebug_;
  }

  void
  Parser::set_debug_stream (std::ostream& o)
  {
    yycdebug_ = &o;
  }


  Parser::debug_level_type
  Parser::debug_level () const
  {
    return yydebug_;
  }

  void
  Parser::set_debug_level (debug_level_type l)
  {
    yydebug_ = l;
  }
#endif // YYDEBUG

  Parser::state_type
  Parser::yy_lr_goto_state_ (state_type yystate, int yysym)
  {
    int yyr = yypgoto_[yysym - yyntokens_] + yystate;
    if (0 <= yyr && yyr <= yylast_ && yycheck_[yyr] == yystate)
      return yytable_[yyr];
    else
      return yydefgoto_[yysym - yyntokens_];
  }

  bool
  Parser::yy_pact_value_is_default_ (int yyvalue)
  {
    return yyvalue == yypact_ninf_;
  }

  bool
  Parser::yy_table_value_is_error_ (int yyvalue)
  {
    return yyvalue == yytable_ninf_;
  }

  int
  Parser::operator() ()
  {
    return parse ();
  }

  int
  Parser::parse ()
  {
    int yyn;
    /// Length of the RHS of the rule being reduced.
    int yylen = 0;

    // Error handling.
    int yynerrs_ = 0;
    int yyerrstatus_ = 0;

    /// The lookahead symbol.
    symbol_type yyla;

    /// The return value of parse ().
    int yyresult;

#if YY_EXCEPTIONS
    try
#endif // YY_EXCEPTIONS
      {
    YYCDEBUG << "Starting parse\n";


    /* Initialize the stack.  The initial state will be set in
       yynewstate, since the latter expects the semantical and the
       location values to have been already stored, initialize these
       stacks with a primary value.  */
    yystack_.clear ();
    yypush_ (YY_NULLPTR, 0, YY_MOVE (yyla));

  /*-----------------------------------------------.
  | yynewstate -- push a new symbol on the stack.  |
  `-----------------------------------------------*/
  yynewstate:
    YYCDEBUG << "Entering state " << int (yystack_[0].state) << '\n';

    // Accept?
    if (yystack_[0].state == yyfinal_)
      YYACCEPT;

    goto yybackup;


  /*-----------.
  | yybackup.  |
  `-----------*/
  yybackup:
    // Try to take a decision without lookahead.
    yyn = yypact_[+yystack_[0].state];
    if (yy_pact_value_is_default_ (yyn))
      goto yydefault;

    // Read a lookahead token.
    if (yyla.empty ())
      {
        YYCDEBUG << "Reading a token: ";
#if YY_EXCEPTIONS
        try
#endif // YY_EXCEPTIONS
          {
            yyla.type = yytranslate_ (yylex (&yyla.value));
          }
#if YY_EXCEPTIONS
        catch (const syntax_error& yyexc)
          {
            YYCDEBUG << "Caught exception: " << yyexc.what() << '\n';
            error (yyexc);
            goto yyerrlab1;
          }
#endif // YY_EXCEPTIONS
      }
    YY_SYMBOL_PRINT ("Next token is", yyla);

    /* If the proper action on seeing token YYLA.TYPE is to reduce or
       to detect an error, take that action.  */
    yyn += yyla.type_get ();
    if (yyn < 0 || yylast_ < yyn || yycheck_[yyn] != yyla.type_get ())
      {
        goto yydefault;
      }

    // Reduce or error.
    yyn = yytable_[yyn];
    if (yyn <= 0)
      {
        if (yy_table_value_is_error_ (yyn))
          goto yyerrlab;
        yyn = -yyn;
        goto yyreduce;
      }

    // Count tokens shifted since error; after three, turn off error status.
    if (yyerrstatus_)
      --yyerrstatus_;

    // Shift the lookahead token.
    yypush_ ("Shifting", state_type (yyn), YY_MOVE (yyla));
    goto yynewstate;


  /*-----------------------------------------------------------.
  | yydefault -- do the default action for the current state.  |
  `-----------------------------------------------------------*/
  yydefault:
    yyn = yydefact_[+yystack_[0].state];
    if (yyn == 0)
      goto yyerrlab;
    goto yyreduce;


  /*-----------------------------.
  | yyreduce -- do a reduction.  |
  `-----------------------------*/
  yyreduce:
    yylen = yyr2_[yyn];
    {
      stack_symbol_type yylhs;
      yylhs.state = yy_lr_goto_state_ (yystack_[yylen].state, yyr1_[yyn]);
      /* Variants are always initialized to an empty instance of the
         correct type. The default '$$ = $1' action is NOT applied
         when using variants.  */
      switch (yyr1_[yyn])
    {
      case 38: // NUMBER
        yylhs.value.emplace< float > ();
        break;

      case 43: // Scene
        yylhs.value.emplace< int > ();
        break;

      case 37: // STRLIT
        yylhs.value.emplace< std::string > ();
        break;

      case 52: // Transform
        yylhs.value.emplace< std::unique_ptr<transform_node_t>  > ();
        break;

      case 67: // NumberList
        yylhs.value.emplace< std::vector<float>  > ();
        break;

      case 69: // ParamList
        yylhs.value.emplace< std::vector<param_node_t>  > ();
        break;

      case 68: // StrList
        yylhs.value.emplace< std::vector<std::string>  > ();
        break;

      case 44: // SWOptionList
        yylhs.value.emplace< std::vector<swoption_t>  > ();
        break;

      case 53: // World
      case 54: // WorldItemList
        yylhs.value.emplace< std::vector<worlditem_node_t> > ();
        break;

      case 45: // SWOptionItem
      case 46: // Camera
      case 47: // Sampler
      case 48: // Film
      case 49: // Filter
      case 50: // Integrator
      case 51: // Accel
        yylhs.value.emplace< swoption_t > ();
        break;

      case 55: // WorldItem
      case 56: // Attribute
      case 57: // Object
      case 58: // TransformPair
      case 59: // Shape
      case 60: // Light
      case 61: // AreaLight
      case 62: // Material
      case 63: // Texture
      case 64: // ParticipatingMedia
      case 65: // NamedMaterial
      case 66: // ObjectInst
        yylhs.value.emplace< worlditem_node_t > ();
        break;

      default:
        break;
    }



      // Perform the reduction.
      YY_REDUCE_PRINT (yyn);
#if YY_EXCEPTIONS
      try
#endif // YY_EXCEPTIONS
        {
          switch (yyn)
            {
  case 2:
#line 74 "syntax.ypp"
                       {
        // Fills in the content to parse parameters.
        assert(scene_root);
        scene_root->swoptions = std::move(yystack_[1].value.as < std::vector<swoption_t>  > ());
        scene_root->world_items = std::move(yystack_[0].value.as < std::vector<worlditem_node_t> > ());
    }
#line 1092 "syntax.tab.cpp"
    break;

  case 3:
#line 83 "syntax.ypp"
                              {
        yylhs.value.as < std::vector<swoption_t>  > () = std::move(yystack_[1].value.as < std::vector<swoption_t>  > ());
        yylhs.value.as < std::vector<swoption_t>  > ().push_back(std::move(yystack_[0].value.as < swoption_t > ()));
    }
#line 1101 "syntax.tab.cpp"
    break;

  case 4:
#line 87 "syntax.ypp"
      {}
#line 1107 "syntax.tab.cpp"
    break;

  case 5:
#line 92 "syntax.ypp"
                    {yylhs.value.as < swoption_t > () = std::move(yystack_[0].value.as < swoption_t > ());}
#line 1113 "syntax.tab.cpp"
    break;

  case 6:
#line 93 "syntax.ypp"
                    {yylhs.value.as < swoption_t > () = std::move(yystack_[0].value.as < swoption_t > ());}
#line 1119 "syntax.tab.cpp"
    break;

  case 7:
#line 94 "syntax.ypp"
                    {yylhs.value.as < swoption_t > () = std::move(yystack_[0].value.as < swoption_t > ());}
#line 1125 "syntax.tab.cpp"
    break;

  case 8:
#line 95 "syntax.ypp"
                    {yylhs.value.as < swoption_t > () = std::move(yystack_[0].value.as < swoption_t > ());}
#line 1131 "syntax.tab.cpp"
    break;

  case 9:
#line 96 "syntax.ypp"
                    {yylhs.value.as < swoption_t > () = std::move(yystack_[0].value.as < swoption_t > ());}
#line 1137 "syntax.tab.cpp"
    break;

  case 10:
#line 97 "syntax.ypp"
                    {yylhs.value.as < swoption_t > () = std::move(yystack_[0].value.as < swoption_t > ());}
#line 1143 "syntax.tab.cpp"
    break;

  case 11:
#line 98 "syntax.ypp"
                {
        yylhs.value.as < swoption_t > ().directive = swoption_t::directive_t::Transform;
        yylhs.value.as < swoption_t > ().trans = std::move(yystack_[0].value.as < std::unique_ptr<transform_node_t>  > ());
    }
#line 1152 "syntax.tab.cpp"
    break;

  case 12:
#line 104 "syntax.ypp"
                               {
        yylhs.value.as < swoption_t > ().directive = swoption_t::directive_t::Camera;
        yylhs.value.as < swoption_t > ().implementation = yystack_[1].value.as < std::string > ();
        yylhs.value.as < swoption_t > ().params = std::move(yystack_[0].value.as < std::vector<param_node_t>  > ());
    }
#line 1162 "syntax.tab.cpp"
    break;

  case 13:
#line 112 "syntax.ypp"
                                {
        yylhs.value.as < swoption_t > ().directive = swoption_t::directive_t::Sampler;
        yylhs.value.as < swoption_t > ().implementation = yystack_[1].value.as < std::string > ();
        yylhs.value.as < swoption_t > ().params = std::move(yystack_[0].value.as < std::vector<param_node_t>  > ());
    }
#line 1172 "syntax.tab.cpp"
    break;

  case 14:
#line 120 "syntax.ypp"
                             {
        yylhs.value.as < swoption_t > ().directive = swoption_t::directive_t::Film;
        yylhs.value.as < swoption_t > ().implementation = yystack_[1].value.as < std::string > ();
        yylhs.value.as < swoption_t > ().params = std::move(yystack_[0].value.as < std::vector<param_node_t>  > ());
    }
#line 1182 "syntax.tab.cpp"
    break;

  case 15:
#line 128 "syntax.ypp"
                               {
        yylhs.value.as < swoption_t > ().directive = swoption_t::directive_t::Filter;
        yylhs.value.as < swoption_t > ().implementation = yystack_[1].value.as < std::string > ();
        yylhs.value.as < swoption_t > ().params = std::move(yystack_[0].value.as < std::vector<param_node_t>  > ());
    }
#line 1192 "syntax.tab.cpp"
    break;

  case 16:
#line 136 "syntax.ypp"
                                   {
        yylhs.value.as < swoption_t > ().directive = swoption_t::directive_t::Integrator;
        yylhs.value.as < swoption_t > ().implementation = yystack_[1].value.as < std::string > ();
        yylhs.value.as < swoption_t > ().params = std::move(yystack_[0].value.as < std::vector<param_node_t>  > ());
    }
#line 1202 "syntax.tab.cpp"
    break;

  case 17:
#line 144 "syntax.ypp"
                                    {
        yylhs.value.as < swoption_t > ().directive = swoption_t::directive_t::Accel;
        yylhs.value.as < swoption_t > ().implementation = yystack_[1].value.as < std::string > ();
        yylhs.value.as < swoption_t > ().params = std::move(yystack_[0].value.as < std::vector<param_node_t>  > ());
    }
#line 1212 "syntax.tab.cpp"
    break;

  case 18:
#line 152 "syntax.ypp"
                { 
        yylhs.value.as < std::unique_ptr<transform_node_t>  > () = std::make_unique<transform_node_t>();
        yylhs.value.as < std::unique_ptr<transform_node_t>  > ()->t = transform_node_t::type::Identity;
    }
#line 1221 "syntax.tab.cpp"
    break;

  case 19:
#line 156 "syntax.ypp"
                                        {
        yylhs.value.as < std::unique_ptr<transform_node_t>  > () = std::make_unique<transform_node_t>();
        yylhs.value.as < std::unique_ptr<transform_node_t>  > ()->t = transform_node_t::type::Translate;
        yylhs.value.as < std::unique_ptr<transform_node_t>  > ()->numbers = {yystack_[2].value.as < float > (), yystack_[1].value.as < float > (), yystack_[0].value.as < float > ()};
    }
#line 1231 "syntax.tab.cpp"
    break;

  case 20:
#line 161 "syntax.ypp"
                                        {
        yylhs.value.as < std::unique_ptr<transform_node_t>  > () = std::make_unique<transform_node_t>();
        yylhs.value.as < std::unique_ptr<transform_node_t>  > ()->t = transform_node_t::type::Scale;
        yylhs.value.as < std::unique_ptr<transform_node_t>  > ()->numbers = {yystack_[2].value.as < float > (), yystack_[1].value.as < float > (), yystack_[0].value.as < float > ()};
    }
#line 1241 "syntax.tab.cpp"
    break;

  case 21:
#line 166 "syntax.ypp"
                                               {
        yylhs.value.as < std::unique_ptr<transform_node_t>  > () = std::make_unique<transform_node_t>();
        yylhs.value.as < std::unique_ptr<transform_node_t>  > ()->t = transform_node_t::type::Rotate;
        yylhs.value.as < std::unique_ptr<transform_node_t>  > ()->numbers = {yystack_[3].value.as < float > (), yystack_[2].value.as < float > (), yystack_[1].value.as < float > (), yystack_[0].value.as < float > ()};
    }
#line 1251 "syntax.tab.cpp"
    break;

  case 22:
#line 171 "syntax.ypp"
                              {
        yylhs.value.as < std::unique_ptr<transform_node_t>  > () = std::make_unique<transform_node_t>();
        yylhs.value.as < std::unique_ptr<transform_node_t>  > ()->t = transform_node_t::type::LookAt;
        yylhs.value.as < std::unique_ptr<transform_node_t>  > ()->numbers = std::move(yystack_[0].value.as < std::vector<float>  > ()); // collect_numbers($2, 9);
    }
#line 1261 "syntax.tab.cpp"
    break;

  case 23:
#line 176 "syntax.ypp"
                             {
        yylhs.value.as < std::unique_ptr<transform_node_t>  > () = std::make_unique<transform_node_t>();
        yylhs.value.as < std::unique_ptr<transform_node_t>  > ()->t = transform_node_t::type::CoordSys;
        yylhs.value.as < std::unique_ptr<transform_node_t>  > ()->str = yystack_[0].value.as < std::string > ();
    }
#line 1271 "syntax.tab.cpp"
    break;

  case 24:
#line 181 "syntax.ypp"
                                   {
        yylhs.value.as < std::unique_ptr<transform_node_t>  > () = std::make_unique<transform_node_t>();
        yylhs.value.as < std::unique_ptr<transform_node_t>  > ()->t = transform_node_t::type::CoordSysTrans;
        yylhs.value.as < std::unique_ptr<transform_node_t>  > ()->str = yystack_[0].value.as < std::string > ();
    }
#line 1281 "syntax.tab.cpp"
    break;

  case 25:
#line 186 "syntax.ypp"
                              {
        yylhs.value.as < std::unique_ptr<transform_node_t>  > () = std::make_unique<transform_node_t>();
        yylhs.value.as < std::unique_ptr<transform_node_t>  > ()->t = transform_node_t::type::Mat4;
        yylhs.value.as < std::unique_ptr<transform_node_t>  > ()->numbers = std::move(yystack_[0].value.as < std::vector<float>  > ()); // collect_numbers($2, 16);
    }
#line 1291 "syntax.tab.cpp"
    break;

  case 26:
#line 191 "syntax.ypp"
                                      {
        yylhs.value.as < std::unique_ptr<transform_node_t>  > () = std::make_unique<transform_node_t>();
        yylhs.value.as < std::unique_ptr<transform_node_t>  > ()->t = transform_node_t::type::Mat4;
        yylhs.value.as < std::unique_ptr<transform_node_t>  > ()->numbers = std::move(yystack_[1].value.as < std::vector<float>  > ()); // collect_numbers($3, 16);
    }
#line 1301 "syntax.tab.cpp"
    break;

  case 27:
#line 196 "syntax.ypp"
                                     {
        yylhs.value.as < std::unique_ptr<transform_node_t>  > () = std::make_unique<transform_node_t>();
        yylhs.value.as < std::unique_ptr<transform_node_t>  > ()->t = transform_node_t::type::ConcatMat4;
        yylhs.value.as < std::unique_ptr<transform_node_t>  > ()->numbers = std::move(yystack_[0].value.as < std::vector<float>  > ()); // collect_numbers($2, 16);
    }
#line 1311 "syntax.tab.cpp"
    break;

  case 28:
#line 201 "syntax.ypp"
                                             {
        yylhs.value.as < std::unique_ptr<transform_node_t>  > () = std::make_unique<transform_node_t>();
        yylhs.value.as < std::unique_ptr<transform_node_t>  > ()->t = transform_node_t::type::ConcatMat4;
        yylhs.value.as < std::unique_ptr<transform_node_t>  > ()->numbers = std::move(yystack_[1].value.as < std::vector<float>  > ()); // collect_numbers($3, 16);
    }
#line 1321 "syntax.tab.cpp"
    break;

  case 29:
#line 210 "syntax.ypp"
                                            { yylhs.value.as < std::vector<worlditem_node_t> > () = std::move(yystack_[1].value.as < std::vector<worlditem_node_t> > ()); }
#line 1327 "syntax.tab.cpp"
    break;

  case 30:
#line 214 "syntax.ypp"
                            { 
        yylhs.value.as < std::vector<worlditem_node_t> > () = std::move(yystack_[1].value.as < std::vector<worlditem_node_t> > ());
        yylhs.value.as < std::vector<worlditem_node_t> > ().push_back(std::move(yystack_[0].value.as < worlditem_node_t > ()));
    }
#line 1336 "syntax.tab.cpp"
    break;

  case 31:
#line 217 "syntax.ypp"
                  { yylhs.value.as < std::vector<worlditem_node_t> > ().push_back(std::move(yystack_[0].value.as < worlditem_node_t > ())); }
#line 1342 "syntax.tab.cpp"
    break;

  case 32:
#line 221 "syntax.ypp"
              {
        // $$ = new worlditem_node_t;
        yylhs.value.as < worlditem_node_t > ().directive = worlditem_node_t::directive_t::Transform;
        yylhs.value.as < worlditem_node_t > ().trans = std::move(yystack_[0].value.as < std::unique_ptr<transform_node_t>  > ());
    }
#line 1352 "syntax.tab.cpp"
    break;

  case 33:
#line 227 "syntax.ypp"
                    { yylhs.value.as < worlditem_node_t > () = std::move(yystack_[0].value.as < worlditem_node_t > ()); }
#line 1358 "syntax.tab.cpp"
    break;

  case 34:
#line 228 "syntax.ypp"
                    { yylhs.value.as < worlditem_node_t > () = std::move(yystack_[0].value.as < worlditem_node_t > ()); }
#line 1364 "syntax.tab.cpp"
    break;

  case 35:
#line 229 "syntax.ypp"
                    { yylhs.value.as < worlditem_node_t > () = std::move(yystack_[0].value.as < worlditem_node_t > ()); }
#line 1370 "syntax.tab.cpp"
    break;

  case 36:
#line 231 "syntax.ypp"
                { yylhs.value.as < worlditem_node_t > () = std::move(yystack_[0].value.as < worlditem_node_t > ()); }
#line 1376 "syntax.tab.cpp"
    break;

  case 37:
#line 232 "syntax.ypp"
                { yylhs.value.as < worlditem_node_t > () = std::move(yystack_[0].value.as < worlditem_node_t > ()); }
#line 1382 "syntax.tab.cpp"
    break;

  case 38:
#line 233 "syntax.ypp"
                { yylhs.value.as < worlditem_node_t > () = std::move(yystack_[0].value.as < worlditem_node_t > ()); }
#line 1388 "syntax.tab.cpp"
    break;

  case 39:
#line 234 "syntax.ypp"
                { yylhs.value.as < worlditem_node_t > () = std::move(yystack_[0].value.as < worlditem_node_t > ()); }
#line 1394 "syntax.tab.cpp"
    break;

  case 40:
#line 235 "syntax.ypp"
                { yylhs.value.as < worlditem_node_t > () = std::move(yystack_[0].value.as < worlditem_node_t > ()); }
#line 1400 "syntax.tab.cpp"
    break;

  case 41:
#line 237 "syntax.ypp"
                            { yylhs.value.as < worlditem_node_t > () = std::move(yystack_[0].value.as < worlditem_node_t > ()); }
#line 1406 "syntax.tab.cpp"
    break;

  case 42:
#line 238 "syntax.ypp"
                            {yylhs.value.as < worlditem_node_t > () = std::move(yystack_[0].value.as < worlditem_node_t > ());}
#line 1412 "syntax.tab.cpp"
    break;

  case 43:
#line 239 "syntax.ypp"
                             {
        yylhs.value.as < worlditem_node_t > ().directive = worlditem_node_t::directive_t::ReverseOrientation;
    }
#line 1420 "syntax.tab.cpp"
    break;

  case 44:
#line 242 "syntax.ypp"
                    {yylhs.value.as < worlditem_node_t > () = std::move(yystack_[0].value.as < worlditem_node_t > ());}
#line 1426 "syntax.tab.cpp"
    break;

  case 45:
#line 246 "syntax.ypp"
                                          {
        yylhs.value.as < worlditem_node_t > ().directive = worlditem_node_t::directive_t::AttrPair;
        yylhs.value.as < worlditem_node_t > ().child = std::move(yystack_[1].value.as < std::vector<worlditem_node_t> > ());
    }
#line 1435 "syntax.tab.cpp"
    break;

  case 46:
#line 250 "syntax.ypp"
                              {  }
#line 1441 "syntax.tab.cpp"
    break;

  case 47:
#line 255 "syntax.ypp"
                                               {
        yylhs.value.as < worlditem_node_t > ().directive = worlditem_node_t::directive_t::ObjPair;
        yylhs.value.as < worlditem_node_t > ().object_name = yystack_[2].value.as < std::string > ();
        yylhs.value.as < worlditem_node_t > ().child = std::move(yystack_[1].value.as < std::vector<worlditem_node_t> > ());
    }
#line 1451 "syntax.tab.cpp"
    break;

  case 48:
#line 260 "syntax.ypp"
                                                                         {
        yylhs.value.as < worlditem_node_t > ().directive = worlditem_node_t::directive_t::ObjPair;
        yylhs.value.as < worlditem_node_t > ().object_name = yystack_[3].value.as < std::string > ();
        yylhs.value.as < worlditem_node_t > ().child = std::move(yystack_[2].value.as < std::vector<worlditem_node_t> > ());
    }
#line 1461 "syntax.tab.cpp"
    break;

  case 49:
#line 265 "syntax.ypp"
                                                           { }
#line 1467 "syntax.tab.cpp"
    break;

  case 50:
#line 269 "syntax.ypp"
                                            {
        yylhs.value.as < worlditem_node_t > ().directive = worlditem_node_t::directive_t::TransPair;
        yylhs.value.as < worlditem_node_t > ().child = std::move(yystack_[1].value.as < std::vector<worlditem_node_t> > ());
    }
#line 1476 "syntax.tab.cpp"
    break;

  case 51:
#line 277 "syntax.ypp"
                              {
        yylhs.value.as < worlditem_node_t > ().directive = worlditem_node_t::directive_t::Shape;
        yylhs.value.as < worlditem_node_t > ().implementation = yystack_[1].value.as < std::string > ();
        yylhs.value.as < worlditem_node_t > ().params = std::move(yystack_[0].value.as < std::vector<param_node_t>  > ());
    }
#line 1486 "syntax.tab.cpp"
    break;

  case 52:
#line 285 "syntax.ypp"
                                  {
        yylhs.value.as < worlditem_node_t > ().directive = worlditem_node_t::directive_t::Light;
        yylhs.value.as < worlditem_node_t > ().implementation = yystack_[1].value.as < std::string > ();
        yylhs.value.as < worlditem_node_t > ().params = std::move(yystack_[0].value.as < std::vector<param_node_t>  > ());
    }
#line 1496 "syntax.tab.cpp"
    break;

  case 53:
#line 293 "syntax.ypp"
                                       {
        yylhs.value.as < worlditem_node_t > ().directive = worlditem_node_t::directive_t::AreaLight;
        yylhs.value.as < worlditem_node_t > ().implementation = yystack_[1].value.as < std::string > ();
        yylhs.value.as < worlditem_node_t > ().params = std::move(yystack_[0].value.as < std::vector<param_node_t>  > ());
    }
#line 1506 "syntax.tab.cpp"
    break;

  case 54:
#line 301 "syntax.ypp"
                                 {
        yylhs.value.as < worlditem_node_t > ().directive = worlditem_node_t::directive_t::Material;
        yylhs.value.as < worlditem_node_t > ().implementation = yystack_[1].value.as < std::string > ();
        yylhs.value.as < worlditem_node_t > ().params = std::move(yystack_[0].value.as < std::vector<param_node_t>  > ());
    }
#line 1516 "syntax.tab.cpp"
    break;

  case 55:
#line 309 "syntax.ypp"
                                              {
        yylhs.value.as < worlditem_node_t > ().directive = worlditem_node_t::directive_t::Texture;
        yylhs.value.as < worlditem_node_t > ().implementation = yystack_[1].value.as < std::string > (); // TODO move the strings but not copy them.
        yylhs.value.as < worlditem_node_t > ().tex_type = yystack_[2].value.as < std::string > ();
        yylhs.value.as < worlditem_node_t > ().tex_name = yystack_[3].value.as < std::string > ();
        yylhs.value.as < worlditem_node_t > ().params = std::move(yystack_[0].value.as < std::vector<param_node_t>  > ());
    }
#line 1528 "syntax.tab.cpp"
    break;

  case 56:
#line 319 "syntax.ypp"
                                          {
        yylhs.value.as < worlditem_node_t > ().directive = worlditem_node_t::directive_t::MakeMedium;
        yylhs.value.as < worlditem_node_t > ().maked_name = yystack_[1].value.as < std::string > ();
        yylhs.value.as < worlditem_node_t > ().params = std::move(yystack_[0].value.as < std::vector<param_node_t>  > ());
    }
#line 1538 "syntax.tab.cpp"
    break;

  case 57:
#line 325 "syntax.ypp"
                                      {
        yylhs.value.as < worlditem_node_t > ().directive = worlditem_node_t::directive_t::MediaInst;
        yylhs.value.as < worlditem_node_t > ().inst_name = yystack_[0].value.as < std::string > ();
    }
#line 1547 "syntax.tab.cpp"
    break;

  case 58:
#line 332 "syntax.ypp"
                                            {
        // $$ = new struct worlditem_node_t;
        yylhs.value.as < worlditem_node_t > ().directive = worlditem_node_t::directive_t::MakeMaterial;
        yylhs.value.as < worlditem_node_t > ().maked_name = yystack_[1].value.as < std::string > ();
        yylhs.value.as < worlditem_node_t > ().params = std::move(yystack_[0].value.as < std::vector<param_node_t>  > ());
    }
#line 1558 "syntax.tab.cpp"
    break;

  case 59:
#line 338 "syntax.ypp"
                                {
        // $$ = new struct worlditem_node_t;
        yylhs.value.as < worlditem_node_t > ().directive = worlditem_node_t::directive_t::MaterialInst;
        yylhs.value.as < worlditem_node_t > ().inst_name = yystack_[0].value.as < std::string > ();
    }
#line 1568 "syntax.tab.cpp"
    break;

  case 60:
#line 346 "syntax.ypp"
                       {
        yylhs.value.as < worlditem_node_t > ().directive = worlditem_node_t::directive_t::ObjInst;
        yylhs.value.as < worlditem_node_t > ().inst_name = yystack_[0].value.as < std::string > ();
    }
#line 1577 "syntax.tab.cpp"
    break;

  case 61:
#line 353 "syntax.ypp"
                      {
        yylhs.value.as < std::vector<float>  > () = std::move(yystack_[1].value.as < std::vector<float>  > ());
        yylhs.value.as < std::vector<float>  > ().push_back(yystack_[0].value.as < float > ());
    }
#line 1586 "syntax.tab.cpp"
    break;

  case 62:
#line 356 "syntax.ypp"
               {
        yylhs.value.as < std::vector<float>  > ().push_back(yystack_[0].value.as < float > ());
    }
#line 1594 "syntax.tab.cpp"
    break;

  case 63:
#line 362 "syntax.ypp"
                   {
        yylhs.value.as < std::vector<std::string>  > () = std::move(yystack_[1].value.as < std::vector<std::string>  > ());
        yylhs.value.as < std::vector<std::string>  > ().push_back(yystack_[0].value.as < std::string > ());
    }
#line 1603 "syntax.tab.cpp"
    break;

  case 64:
#line 365 "syntax.ypp"
               {
        yylhs.value.as < std::vector<std::string>  > ().push_back(yystack_[0].value.as < std::string > ());
    }
#line 1611 "syntax.tab.cpp"
    break;

  case 65:
#line 370 "syntax.ypp"
                                        {
        param_node_t parameter;
        parameter.key = std::move(yystack_[3].value.as < std::string > ());
        parameter.numbers = std::move(yystack_[1].value.as < std::vector<float>  > ());
        
        yylhs.value.as < std::vector<param_node_t>  > () = std::move(yystack_[4].value.as < std::vector<param_node_t>  > ());
        yylhs.value.as < std::vector<param_node_t>  > ().push_back(std::move(parameter));
    }
#line 1624 "syntax.tab.cpp"
    break;

  case 66:
#line 377 "syntax.ypp"
                                        {
        param_node_t parameter;
        parameter.key = std::move(yystack_[3].value.as < std::string > ());
        parameter.strings = std::move(yystack_[1].value.as < std::vector<std::string>  > ());
        
        yylhs.value.as < std::vector<param_node_t>  > () = std::move(yystack_[4].value.as < std::vector<param_node_t>  > ());
        yylhs.value.as < std::vector<param_node_t>  > ().push_back(std::move(parameter));
    }
#line 1637 "syntax.tab.cpp"
    break;

  case 67:
#line 384 "syntax.ypp"
                                {
        param_node_t parameter;
        parameter.key = std::move(yystack_[1].value.as < std::string > ());
        parameter.strings.push_back(std::move(yystack_[0].value.as < std::string > ()));
        
        yylhs.value.as < std::vector<param_node_t>  > () = std::move(yystack_[2].value.as < std::vector<param_node_t>  > ());
        yylhs.value.as < std::vector<param_node_t>  > ().push_back(std::move(parameter));
    }
#line 1650 "syntax.tab.cpp"
    break;

  case 68:
#line 391 "syntax.ypp"
                                    {
        param_node_t parameter;
        parameter.key = std::move(yystack_[1].value.as < std::string > ());
        parameter.numbers = std::move(yystack_[0].value.as < std::vector<float>  > ());
        
        yylhs.value.as < std::vector<param_node_t>  > () = std::move(yystack_[2].value.as < std::vector<param_node_t>  > ());
        yylhs.value.as < std::vector<param_node_t>  > ().push_back(std::move(parameter));
    }
#line 1663 "syntax.tab.cpp"
    break;

  case 69:
#line 398 "syntax.ypp"
        {}
#line 1669 "syntax.tab.cpp"
    break;


#line 1673 "syntax.tab.cpp"

            default:
              break;
            }
        }
#if YY_EXCEPTIONS
      catch (const syntax_error& yyexc)
        {
          YYCDEBUG << "Caught exception: " << yyexc.what() << '\n';
          error (yyexc);
          YYERROR;
        }
#endif // YY_EXCEPTIONS
      YY_SYMBOL_PRINT ("-> $$ =", yylhs);
      yypop_ (yylen);
      yylen = 0;
      YY_STACK_PRINT ();

      // Shift the result of the reduction.
      yypush_ (YY_NULLPTR, YY_MOVE (yylhs));
    }
    goto yynewstate;


  /*--------------------------------------.
  | yyerrlab -- here on detecting error.  |
  `--------------------------------------*/
  yyerrlab:
    // If not already recovering from an error, report this error.
    if (!yyerrstatus_)
      {
        ++yynerrs_;
        error (yysyntax_error_ (yystack_[0].state, yyla));
      }


    if (yyerrstatus_ == 3)
      {
        /* If just tried and failed to reuse lookahead token after an
           error, discard it.  */

        // Return failure if at end of input.
        if (yyla.type_get () == yyeof_)
          YYABORT;
        else if (!yyla.empty ())
          {
            yy_destroy_ ("Error: discarding", yyla);
            yyla.clear ();
          }
      }

    // Else will try to reuse lookahead token after shifting the error token.
    goto yyerrlab1;


  /*---------------------------------------------------.
  | yyerrorlab -- error raised explicitly by YYERROR.  |
  `---------------------------------------------------*/
  yyerrorlab:
    /* Pacify compilers when the user code never invokes YYERROR and
       the label yyerrorlab therefore never appears in user code.  */
    if (false)
      YYERROR;

    /* Do not reclaim the symbols of the rule whose action triggered
       this YYERROR.  */
    yypop_ (yylen);
    yylen = 0;
    goto yyerrlab1;


  /*-------------------------------------------------------------.
  | yyerrlab1 -- common code for both syntax error and YYERROR.  |
  `-------------------------------------------------------------*/
  yyerrlab1:
    yyerrstatus_ = 3;   // Each real token shifted decrements this.
    {
      stack_symbol_type error_token;
      for (;;)
        {
          yyn = yypact_[+yystack_[0].state];
          if (!yy_pact_value_is_default_ (yyn))
            {
              yyn += yy_error_token_;
              if (0 <= yyn && yyn <= yylast_ && yycheck_[yyn] == yy_error_token_)
                {
                  yyn = yytable_[yyn];
                  if (0 < yyn)
                    break;
                }
            }

          // Pop the current state because it cannot handle the error token.
          if (yystack_.size () == 1)
            YYABORT;

          yy_destroy_ ("Error: popping", yystack_[0]);
          yypop_ ();
          YY_STACK_PRINT ();
        }


      // Shift the error token.
      error_token.state = state_type (yyn);
      yypush_ ("Shifting", YY_MOVE (error_token));
    }
    goto yynewstate;


  /*-------------------------------------.
  | yyacceptlab -- YYACCEPT comes here.  |
  `-------------------------------------*/
  yyacceptlab:
    yyresult = 0;
    goto yyreturn;


  /*-----------------------------------.
  | yyabortlab -- YYABORT comes here.  |
  `-----------------------------------*/
  yyabortlab:
    yyresult = 1;
    goto yyreturn;


  /*-----------------------------------------------------.
  | yyreturn -- parsing is finished, return the result.  |
  `-----------------------------------------------------*/
  yyreturn:
    if (!yyla.empty ())
      yy_destroy_ ("Cleanup: discarding lookahead", yyla);

    /* Do not reclaim the symbols of the rule whose action triggered
       this YYABORT or YYACCEPT.  */
    yypop_ (yylen);
    while (1 < yystack_.size ())
      {
        yy_destroy_ ("Cleanup: popping", yystack_[0]);
        yypop_ ();
      }

    return yyresult;
  }
#if YY_EXCEPTIONS
    catch (...)
      {
        YYCDEBUG << "Exception caught: cleaning lookahead and stack\n";
        // Do not try to display the values of the reclaimed symbols,
        // as their printers might throw an exception.
        if (!yyla.empty ())
          yy_destroy_ (YY_NULLPTR, yyla);

        while (1 < yystack_.size ())
          {
            yy_destroy_ (YY_NULLPTR, yystack_[0]);
            yypop_ ();
          }
        throw;
      }
#endif // YY_EXCEPTIONS
  }

  void
  Parser::error (const syntax_error& yyexc)
  {
    error (yyexc.what ());
  }

  // Generate an error message.
  std::string
  Parser::yysyntax_error_ (state_type, const symbol_type&) const
  {
    return YY_("syntax error");
  }


  const signed char Parser::yypact_ninf_ = -63;

  const signed char Parser::yytable_ninf_ = -1;

  const short
  Parser::yypact_[] =
  {
     -63,    13,   343,   -63,   311,   -22,   -16,    -7,   -63,    -4,
       0,   -35,   -17,     3,     5,     6,     7,     8,     9,    10,
     -63,   -63,   -63,   -63,   -63,   -63,   -63,   -63,   -63,   107,
      11,   311,    12,    17,    18,    19,    20,    21,    22,    30,
      40,    42,   -63,   -63,   141,   -63,   -63,   -63,   -63,   -63,
     -63,   -63,   -63,   -63,   -63,   -63,   -63,    43,    44,    48,
     -63,   -63,   -63,     3,    56,     3,    56,    56,   -63,   -63,
     -63,   -63,   -63,   -63,   -63,    58,   175,   311,   209,   -63,
      60,   -63,   -63,   -63,    61,   -63,   -63,   -63,   -63,   -63,
     -63,    62,    63,    64,   -26,   -63,    -9,    66,    66,    66,
      66,    66,    66,   243,   -63,   277,   -63,    66,    67,    66,
      66,   -63,    66,    66,    66,   -63,   -63,    68,   -63,   -63,
     -10,    91,    57,   -63,   -63,   -63,   -63,     2,    56,   -63,
      97,    66,   -63,    -3,   -23,   -63,   -63,   -63,   -63
  };

  const signed char
  Parser::yydefact_[] =
  {
       4,     0,     0,     1,     0,     0,     0,     0,    18,     0,
       0,     0,     0,     0,     0,     0,     0,     0,     0,     0,
       3,     5,     6,     7,     8,     9,    10,    11,     2,     0,
       0,     0,     0,     0,     0,     0,     0,     0,     0,     0,
       0,     0,    43,    32,     0,    31,    33,    34,    35,    36,
      38,    39,    37,    40,    42,    44,    41,     0,     0,     0,
      23,    24,    62,     0,    25,     0,    27,    22,    69,    69,
      69,    69,    69,    69,    46,     0,     0,     0,     0,    69,
       0,    69,    69,    60,     0,    59,    69,    69,    69,    29,
      30,     0,     0,     0,     0,    61,     0,    12,    16,    13,
      14,    17,    15,     0,    45,     0,    50,    54,     0,    51,
      56,    57,    58,    52,    53,    19,    20,     0,    26,    28,
       0,     0,     0,    47,    69,    21,    67,     0,    68,    49,
       0,    55,    64,     0,     0,    48,    65,    63,    66
  };

  const signed char
  Parser::yypgoto_[] =
  {
     -63,   -63,   -63,   -63,   -63,   -63,   -63,   -63,   -63,   -63,
     105,   -63,   -25,   -42,   -63,   -63,   -63,   -63,   -63,   -63,
     -63,   -63,   -63,   -63,   -63,   -12,   -63,   -62
  };

  const short
  Parser::yydefgoto_[] =
  {
      -1,     1,     2,    20,    21,    22,    23,    24,    25,    26,
      43,    28,    44,    45,    46,    47,    48,    49,    50,    51,
      52,    53,    54,    55,    56,    64,   134,    97
  };

  const unsigned char
  Parser::yytable_[] =
  {
      66,    67,    90,    62,    76,    63,    78,    98,    99,   100,
     101,   102,    95,     3,   137,   118,    57,   107,   138,   109,
     110,    62,    58,    65,   112,   113,   114,   126,    62,    95,
     127,    59,   119,    60,    90,    95,    90,    61,   136,   132,
      62,    62,    68,    69,    70,    71,    72,    73,    77,    79,
       0,    94,   105,    96,    80,    81,    82,    83,    84,    85,
      29,   130,   131,    90,    30,   123,    31,    86,     5,     6,
       7,     8,     9,    10,    11,    12,    13,    87,   122,    88,
      90,    91,    92,    32,    33,    34,    93,    35,    36,    37,
      38,    39,    40,    41,    95,   103,    42,   108,   111,   129,
     115,   116,   117,   120,   124,   135,   125,    27,   128,     0,
      29,    74,     0,     0,    75,   133,    31,     0,     5,     6,
       7,     8,     9,    10,    11,    12,    13,     0,     0,     0,
       0,     0,     0,    32,    33,    34,     0,    35,    36,    37,
      38,    39,    40,    41,    29,     0,    42,    89,    30,     0,
      31,     0,     5,     6,     7,     8,     9,    10,    11,    12,
      13,     0,     0,     0,     0,     0,     0,    32,    33,    34,
       0,    35,    36,    37,    38,    39,    40,    41,    29,   104,
      42,     0,    30,     0,    31,     0,     5,     6,     7,     8,
       9,    10,    11,    12,    13,     0,     0,     0,     0,     0,
       0,    32,    33,    34,     0,    35,    36,    37,    38,    39,
      40,    41,    29,     0,    42,     0,    30,     0,    31,   106,
       5,     6,     7,     8,     9,    10,    11,    12,    13,     0,
       0,     0,     0,     0,     0,    32,    33,    34,     0,    35,
      36,    37,    38,    39,    40,    41,    29,   121,    42,     0,
      30,     0,    31,     0,     5,     6,     7,     8,     9,    10,
      11,    12,    13,     0,     0,     0,     0,     0,     0,    32,
      33,    34,     0,    35,    36,    37,    38,    39,    40,    41,
      29,     0,    42,     0,    30,   123,    31,     0,     5,     6,
       7,     8,     9,    10,    11,    12,    13,     0,     0,     0,
       0,     0,     0,    32,    33,    34,     0,    35,    36,    37,
      38,    39,    40,    41,    29,     0,    42,     0,    30,     0,
      31,     0,     5,     6,     7,     8,     9,    10,    11,    12,
      13,     0,     0,     0,     0,     0,     0,    32,    33,    34,
       0,    35,    36,    37,    38,    39,    40,    41,     4,     0,
      42,     0,     0,     0,     5,     6,     7,     8,     9,    10,
      11,    12,    13,    14,    15,    16,    17,    18,    19
  };

  const signed char
  Parser::yycheck_[] =
  {
      12,    13,    44,    38,    29,    40,    31,    69,    70,    71,
      72,    73,    38,     0,    37,    41,    38,    79,    41,    81,
      82,    38,    38,    40,    86,    87,    88,    37,    38,    38,
      40,    38,    41,    37,    76,    38,    78,    37,    41,    37,
      38,    38,    37,    37,    37,    37,    37,    37,    37,    37,
      -1,    63,    77,    65,    37,    37,    37,    37,    37,    37,
       3,     4,   124,   105,     7,     8,     9,    37,    11,    12,
      13,    14,    15,    16,    17,    18,    19,    37,   103,    37,
     122,    38,    38,    26,    27,    28,    38,    30,    31,    32,
      33,    34,    35,    36,    38,    37,    39,    37,    37,     8,
      38,    38,    38,    37,    37,     8,    38,     2,   120,    -1,
       3,     4,    -1,    -1,     7,   127,     9,    -1,    11,    12,
      13,    14,    15,    16,    17,    18,    19,    -1,    -1,    -1,
      -1,    -1,    -1,    26,    27,    28,    -1,    30,    31,    32,
      33,    34,    35,    36,     3,    -1,    39,     6,     7,    -1,
       9,    -1,    11,    12,    13,    14,    15,    16,    17,    18,
      19,    -1,    -1,    -1,    -1,    -1,    -1,    26,    27,    28,
      -1,    30,    31,    32,    33,    34,    35,    36,     3,     4,
      39,    -1,     7,    -1,     9,    -1,    11,    12,    13,    14,
      15,    16,    17,    18,    19,    -1,    -1,    -1,    -1,    -1,
      -1,    26,    27,    28,    -1,    30,    31,    32,    33,    34,
      35,    36,     3,    -1,    39,    -1,     7,    -1,     9,    10,
      11,    12,    13,    14,    15,    16,    17,    18,    19,    -1,
      -1,    -1,    -1,    -1,    -1,    26,    27,    28,    -1,    30,
      31,    32,    33,    34,    35,    36,     3,     4,    39,    -1,
       7,    -1,     9,    -1,    11,    12,    13,    14,    15,    16,
      17,    18,    19,    -1,    -1,    -1,    -1,    -1,    -1,    26,
      27,    28,    -1,    30,    31,    32,    33,    34,    35,    36,
       3,    -1,    39,    -1,     7,     8,     9,    -1,    11,    12,
      13,    14,    15,    16,    17,    18,    19,    -1,    -1,    -1,
      -1,    -1,    -1,    26,    27,    28,    -1,    30,    31,    32,
      33,    34,    35,    36,     3,    -1,    39,    -1,     7,    -1,
       9,    -1,    11,    12,    13,    14,    15,    16,    17,    18,
      19,    -1,    -1,    -1,    -1,    -1,    -1,    26,    27,    28,
      -1,    30,    31,    32,    33,    34,    35,    36,     5,    -1,
      39,    -1,    -1,    -1,    11,    12,    13,    14,    15,    16,
      17,    18,    19,    20,    21,    22,    23,    24,    25
  };

  const signed char
  Parser::yystos_[] =
  {
       0,    43,    44,     0,     5,    11,    12,    13,    14,    15,
      16,    17,    18,    19,    20,    21,    22,    23,    24,    25,
      45,    46,    47,    48,    49,    50,    51,    52,    53,     3,
       7,     9,    26,    27,    28,    30,    31,    32,    33,    34,
      35,    36,    39,    52,    54,    55,    56,    57,    58,    59,
      60,    61,    62,    63,    64,    65,    66,    38,    38,    38,
      37,    37,    38,    40,    67,    40,    67,    67,    37,    37,
      37,    37,    37,    37,     4,     7,    54,    37,    54,    37,
      37,    37,    37,    37,    37,    37,    37,    37,    37,     6,
      55,    38,    38,    38,    67,    38,    67,    69,    69,    69,
      69,    69,    69,    37,     4,    54,    10,    69,    37,    69,
      69,    37,    69,    69,    69,    38,    38,    38,    41,    41,
      37,     4,    54,     8,    37,    38,    37,    40,    67,     8,
       4,    69,    37,    67,    68,     8,    41,    37,    41
  };

  const signed char
  Parser::yyr1_[] =
  {
       0,    42,    43,    44,    44,    45,    45,    45,    45,    45,
      45,    45,    46,    47,    48,    49,    50,    51,    52,    52,
      52,    52,    52,    52,    52,    52,    52,    52,    52,    53,
      54,    54,    55,    55,    55,    55,    55,    55,    55,    55,
      55,    55,    55,    55,    55,    56,    56,    57,    57,    57,
      58,    59,    60,    61,    62,    63,    64,    64,    65,    65,
      66,    67,    67,    68,    68,    69,    69,    69,    69,    69
  };

  const signed char
  Parser::yyr2_[] =
  {
       0,     2,     2,     2,     0,     1,     1,     1,     1,     1,
       1,     1,     3,     3,     3,     3,     3,     3,     1,     4,
       4,     5,     2,     2,     2,     2,     4,     2,     4,     3,
       2,     1,     1,     1,     1,     1,     1,     1,     1,     1,
       1,     1,     1,     1,     1,     3,     2,     4,     6,     5,
       3,     3,     3,     3,     3,     5,     3,     3,     3,     2,
       2,     2,     1,     2,     1,     5,     5,     3,     3,     0
  };


#if YYDEBUG
  // YYTNAME[SYMBOL-NUM] -- String name of the symbol SYMBOL-NUM.
  // First, the terminals, then, starting at \a yyntokens_, nonterminals.
  const char*
  const Parser::yytname_[] =
  {
  "$end", "error", "$undefined", "KW_ATTR_BEG", "KW_ATTR_END",
  "KW_WORLD_BEG", "KW_WORLD_END", "KW_OBJ_BEG", "KW_OBJ_END",
  "KW_TRANS_BEG", "KW_TRANS_END", "KW_TRANSLATE", "KW_SCALE", "KW_ROTATE",
  "KW_IDENTITY", "KW_COORD_SYSTEM", "KW_COORD_SYSTEM_TRANS",
  "KW_TRANSFORM", "KW_CONCAT_TRANSFORM", "KW_LOOKAT", "KW_CAMERA",
  "KW_INTEGRATOR", "KW_SAMPLER", "KW_FILM", "KW_ACCELERATOR", "KW_FILTER",
  "KW_MATERIAL", "KW_TEXTURE", "KW_SHAPE", "KW_NAMED_MEDIUM",
  "KW_MAKE_NAMED_MEDIUM", "KW_OBJ_INST", "KW_MEDIUM_INTERFACE",
  "KW_NAMED_MATERIAL", "KW_MAKE_NAMED_MATERIAL", "KW_LIGHT_SRC",
  "KW_AREA_LIGHT_SRC", "STRLIT", "NUMBER", "KW_REVERSE_ORIENTATION", "'['",
  "']'", "$accept", "Scene", "SWOptionList", "SWOptionItem", "Camera",
  "Sampler", "Film", "Filter", "Integrator", "Accel", "Transform", "World",
  "WorldItemList", "WorldItem", "Attribute", "Object", "TransformPair",
  "Shape", "Light", "AreaLight", "Material", "Texture",
  "ParticipatingMedia", "NamedMaterial", "ObjectInst", "NumberList",
  "StrList", "ParamList", YY_NULLPTR
  };


  const short
  Parser::yyrline_[] =
  {
       0,    74,    74,    83,    87,    92,    93,    94,    95,    96,
      97,    98,   104,   112,   120,   128,   136,   144,   152,   156,
     161,   166,   171,   176,   181,   186,   191,   196,   201,   210,
     214,   217,   221,   227,   228,   229,   231,   232,   233,   234,
     235,   237,   238,   239,   242,   246,   250,   255,   260,   265,
     269,   277,   285,   293,   301,   309,   319,   325,   332,   338,
     346,   353,   356,   362,   365,   370,   377,   384,   391,   398
  };

  // Print the state stack on the debug stream.
  void
  Parser::yystack_print_ ()
  {
    *yycdebug_ << "Stack now";
    for (stack_type::const_iterator
           i = yystack_.begin (),
           i_end = yystack_.end ();
         i != i_end; ++i)
      *yycdebug_ << ' ' << int (i->state);
    *yycdebug_ << '\n';
  }

  // Report on the debug stream that the rule \a yyrule is going to be reduced.
  void
  Parser::yy_reduce_print_ (int yyrule)
  {
    int yylno = yyrline_[yyrule];
    int yynrhs = yyr2_[yyrule];
    // Print the symbols being reduced, and their result.
    *yycdebug_ << "Reducing stack by rule " << yyrule - 1
               << " (line " << yylno << "):\n";
    // The symbols being reduced.
    for (int yyi = 0; yyi < yynrhs; yyi++)
      YY_SYMBOL_PRINT ("   $" << yyi + 1 << " =",
                       yystack_[(yynrhs) - (yyi + 1)]);
  }
#endif // YYDEBUG

  Parser::token_number_type
  Parser::yytranslate_ (int t)
  {
    // YYTRANSLATE[TOKEN-NUM] -- Symbol number corresponding to
    // TOKEN-NUM as returned by yylex.
    static
    const token_number_type
    translate_table[] =
    {
       0,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,    40,     2,    41,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     1,     2,     3,     4,
       5,     6,     7,     8,     9,    10,    11,    12,    13,    14,
      15,    16,    17,    18,    19,    20,    21,    22,    23,    24,
      25,    26,    27,    28,    29,    30,    31,    32,    33,    34,
      35,    36,    37,    38,    39
    };
    const int user_token_number_max_ = 294;

    if (t <= 0)
      return yyeof_;
    else if (t <= user_token_number_max_)
      return translate_table[t];
    else
      return yy_undef_token_;
  }

#line 5 "syntax.ypp"
} // pbr
#line 2154 "syntax.tab.cpp"

#line 401 "syntax.ypp"


void pbr::Parser::error(const std::string &message) {
    std::cerr << "parsing error: " << message << std::endl;
}
