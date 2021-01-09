// A Bison parser, made by GNU Bison 3.5.1.

// Skeleton interface for Bison LALR(1) parsers in C++

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


/**
 ** \file syntax.tab.hpp
 ** Define the pbr::parser class.
 */

// C++ LALR(1) parser skeleton written by Akim Demaille.

// Undocumented macros, especially those whose name start with YY_,
// are private implementation details.  Do not rely on them.

#ifndef YY_YY_SYNTAX_TAB_HPP_INCLUDED
# define YY_YY_SYNTAX_TAB_HPP_INCLUDED
// "%code requires" blocks.
#line 8 "syntax.ypp"

#include "tree.h"
#include <string>
namespace pbr {
class Tokenizer;
}
#ifndef YY_NULLPTR
# if defined __cplusplus && __cplusplus >= 201103L
#  define YY_NULLPTR nullptr
# else
#  define YY_NULLPTR 0
# endif
#endif

#line 63 "syntax.tab.hpp"

# include <cassert>
# include <cstdlib> // std::abort
# include <iostream>
# include <stdexcept>
# include <string>
# include <vector>

#if defined __cplusplus
# define YY_CPLUSPLUS __cplusplus
#else
# define YY_CPLUSPLUS 199711L
#endif

// Support move semantics when possible.
#if 201103L <= YY_CPLUSPLUS
# define YY_MOVE           std::move
# define YY_MOVE_OR_COPY   move
# define YY_MOVE_REF(Type) Type&&
# define YY_RVREF(Type)    Type&&
# define YY_COPY(Type)     Type
#else
# define YY_MOVE
# define YY_MOVE_OR_COPY   copy
# define YY_MOVE_REF(Type) Type&
# define YY_RVREF(Type)    const Type&
# define YY_COPY(Type)     const Type&
#endif

// Support noexcept when possible.
#if 201103L <= YY_CPLUSPLUS
# define YY_NOEXCEPT noexcept
# define YY_NOTHROW
#else
# define YY_NOEXCEPT
# define YY_NOTHROW throw ()
#endif

// Support constexpr when possible.
#if 201703 <= YY_CPLUSPLUS
# define YY_CONSTEXPR constexpr
#else
# define YY_CONSTEXPR
#endif

#include <typeinfo>
#ifndef YY_ASSERT
# include <cassert>
# define YY_ASSERT assert
#endif


#ifndef YY_ATTRIBUTE_PURE
# if defined __GNUC__ && 2 < __GNUC__ + (96 <= __GNUC_MINOR__)
#  define YY_ATTRIBUTE_PURE __attribute__ ((__pure__))
# else
#  define YY_ATTRIBUTE_PURE
# endif
#endif

#ifndef YY_ATTRIBUTE_UNUSED
# if defined __GNUC__ && 2 < __GNUC__ + (7 <= __GNUC_MINOR__)
#  define YY_ATTRIBUTE_UNUSED __attribute__ ((__unused__))
# else
#  define YY_ATTRIBUTE_UNUSED
# endif
#endif

/* Suppress unused-variable warnings by "using" E.  */
#if ! defined lint || defined __GNUC__
# define YYUSE(E) ((void) (E))
#else
# define YYUSE(E) /* empty */
#endif

#if defined __GNUC__ && ! defined __ICC && 407 <= __GNUC__ * 100 + __GNUC_MINOR__
/* Suppress an incorrect diagnostic about yylval being uninitialized.  */
# define YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN                            \
    _Pragma ("GCC diagnostic push")                                     \
    _Pragma ("GCC diagnostic ignored \"-Wuninitialized\"")              \
    _Pragma ("GCC diagnostic ignored \"-Wmaybe-uninitialized\"")
# define YY_IGNORE_MAYBE_UNINITIALIZED_END      \
    _Pragma ("GCC diagnostic pop")
#else
# define YY_INITIAL_VALUE(Value) Value
#endif
#ifndef YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
# define YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
# define YY_IGNORE_MAYBE_UNINITIALIZED_END
#endif
#ifndef YY_INITIAL_VALUE
# define YY_INITIAL_VALUE(Value) /* Nothing. */
#endif

#if defined __cplusplus && defined __GNUC__ && ! defined __ICC && 6 <= __GNUC__
# define YY_IGNORE_USELESS_CAST_BEGIN                          \
    _Pragma ("GCC diagnostic push")                            \
    _Pragma ("GCC diagnostic ignored \"-Wuseless-cast\"")
# define YY_IGNORE_USELESS_CAST_END            \
    _Pragma ("GCC diagnostic pop")
#endif
#ifndef YY_IGNORE_USELESS_CAST_BEGIN
# define YY_IGNORE_USELESS_CAST_BEGIN
# define YY_IGNORE_USELESS_CAST_END
#endif

# ifndef YY_CAST
#  ifdef __cplusplus
#   define YY_CAST(Type, Val) static_cast<Type> (Val)
#   define YY_REINTERPRET_CAST(Type, Val) reinterpret_cast<Type> (Val)
#  else
#   define YY_CAST(Type, Val) ((Type) (Val))
#   define YY_REINTERPRET_CAST(Type, Val) ((Type) (Val))
#  endif
# endif
# ifndef YY_NULLPTR
#  if defined __cplusplus
#   if 201103L <= __cplusplus
#    define YY_NULLPTR nullptr
#   else
#    define YY_NULLPTR 0
#   endif
#  else
#   define YY_NULLPTR ((void*)0)
#  endif
# endif

/* Debug traces.  */
#ifndef YYDEBUG
# define YYDEBUG 1
#endif

#line 5 "syntax.ypp"
namespace pbr {
#line 198 "syntax.tab.hpp"




  /// A Bison parser.
  class Parser
  {
  public:
#ifndef YYSTYPE
  /// A buffer to store and retrieve objects.
  ///
  /// Sort of a variant, but does not keep track of the nature
  /// of the stored data, since that knowledge is available
  /// via the current parser state.
  class semantic_type
  {
  public:
    /// Type of *this.
    typedef semantic_type self_type;

    /// Empty construction.
    semantic_type () YY_NOEXCEPT
      : yybuffer_ ()
      , yytypeid_ (YY_NULLPTR)
    {}

    /// Construct and fill.
    template <typename T>
    semantic_type (YY_RVREF (T) t)
      : yytypeid_ (&typeid (T))
    {
      YY_ASSERT (sizeof (T) <= size);
      new (yyas_<T> ()) T (YY_MOVE (t));
    }

    /// Destruction, allowed only if empty.
    ~semantic_type () YY_NOEXCEPT
    {
      YY_ASSERT (!yytypeid_);
    }

# if 201103L <= YY_CPLUSPLUS
    /// Instantiate a \a T in here from \a t.
    template <typename T, typename... U>
    T&
    emplace (U&&... u)
    {
      YY_ASSERT (!yytypeid_);
      YY_ASSERT (sizeof (T) <= size);
      yytypeid_ = & typeid (T);
      return *new (yyas_<T> ()) T (std::forward <U>(u)...);
    }
# else
    /// Instantiate an empty \a T in here.
    template <typename T>
    T&
    emplace ()
    {
      YY_ASSERT (!yytypeid_);
      YY_ASSERT (sizeof (T) <= size);
      yytypeid_ = & typeid (T);
      return *new (yyas_<T> ()) T ();
    }

    /// Instantiate a \a T in here from \a t.
    template <typename T>
    T&
    emplace (const T& t)
    {
      YY_ASSERT (!yytypeid_);
      YY_ASSERT (sizeof (T) <= size);
      yytypeid_ = & typeid (T);
      return *new (yyas_<T> ()) T (t);
    }
# endif

    /// Instantiate an empty \a T in here.
    /// Obsolete, use emplace.
    template <typename T>
    T&
    build ()
    {
      return emplace<T> ();
    }

    /// Instantiate a \a T in here from \a t.
    /// Obsolete, use emplace.
    template <typename T>
    T&
    build (const T& t)
    {
      return emplace<T> (t);
    }

    /// Accessor to a built \a T.
    template <typename T>
    T&
    as () YY_NOEXCEPT
    {
      YY_ASSERT (yytypeid_);
      YY_ASSERT (*yytypeid_ == typeid (T));
      YY_ASSERT (sizeof (T) <= size);
      return *yyas_<T> ();
    }

    /// Const accessor to a built \a T (for %printer).
    template <typename T>
    const T&
    as () const YY_NOEXCEPT
    {
      YY_ASSERT (yytypeid_);
      YY_ASSERT (*yytypeid_ == typeid (T));
      YY_ASSERT (sizeof (T) <= size);
      return *yyas_<T> ();
    }

    /// Swap the content with \a that, of same type.
    ///
    /// Both variants must be built beforehand, because swapping the actual
    /// data requires reading it (with as()), and this is not possible on
    /// unconstructed variants: it would require some dynamic testing, which
    /// should not be the variant's responsibility.
    /// Swapping between built and (possibly) non-built is done with
    /// self_type::move ().
    template <typename T>
    void
    swap (self_type& that) YY_NOEXCEPT
    {
      YY_ASSERT (yytypeid_);
      YY_ASSERT (*yytypeid_ == *that.yytypeid_);
      std::swap (as<T> (), that.as<T> ());
    }

    /// Move the content of \a that to this.
    ///
    /// Destroys \a that.
    template <typename T>
    void
    move (self_type& that)
    {
# if 201103L <= YY_CPLUSPLUS
      emplace<T> (std::move (that.as<T> ()));
# else
      emplace<T> ();
      swap<T> (that);
# endif
      that.destroy<T> ();
    }

# if 201103L <= YY_CPLUSPLUS
    /// Move the content of \a that to this.
    template <typename T>
    void
    move (self_type&& that)
    {
      emplace<T> (std::move (that.as<T> ()));
      that.destroy<T> ();
    }
#endif

    /// Copy the content of \a that to this.
    template <typename T>
    void
    copy (const self_type& that)
    {
      emplace<T> (that.as<T> ());
    }

    /// Destroy the stored \a T.
    template <typename T>
    void
    destroy ()
    {
      as<T> ().~T ();
      yytypeid_ = YY_NULLPTR;
    }

  private:
    /// Prohibit blind copies.
    self_type& operator= (const self_type&);
    semantic_type (const self_type&);

    /// Accessor to raw memory as \a T.
    template <typename T>
    T*
    yyas_ () YY_NOEXCEPT
    {
      void *yyp = yybuffer_.yyraw;
      return static_cast<T*> (yyp);
     }

    /// Const accessor to raw memory as \a T.
    template <typename T>
    const T*
    yyas_ () const YY_NOEXCEPT
    {
      const void *yyp = yybuffer_.yyraw;
      return static_cast<const T*> (yyp);
     }

    /// An auxiliary type to compute the largest semantic type.
    union union_type
    {
      // NUMBER
      char dummy1[sizeof (float)];

      // Scene
      char dummy2[sizeof (int)];

      // Transform
      char dummy3[sizeof (std::shared_ptr<transform_node_t> )];

      // STRLIT
      char dummy4[sizeof (std::string)];

      // NumberList
      char dummy5[sizeof (std::vector<float> )];

      // ParamList
      char dummy6[sizeof (std::vector<param_node_t> )];

      // StrList
      char dummy7[sizeof (std::vector<std::string> )];

      // SWOptionList
      char dummy8[sizeof (std::vector<swoption_t> )];

      // World
      // WorldItemList
      char dummy9[sizeof (std::vector<worlditem_node_t>)];

      // SWOptionItem
      // Camera
      // Sampler
      // Film
      // Filter
      // Integrator
      // Accel
      char dummy10[sizeof (swoption_t)];

      // WorldItem
      // Attribute
      // Object
      // TransformPair
      // Shape
      // Light
      // AreaLight
      // Material
      // Texture
      // ParticipatingMedia
      // NamedMaterial
      // ObjectInst
      char dummy11[sizeof (worlditem_node_t)];
    };

    /// The size of the largest semantic type.
    enum { size = sizeof (union_type) };

    /// A buffer to store semantic values.
    union
    {
      /// Strongest alignment constraints.
      long double yyalign_me;
      /// A buffer large enough to store any of the semantic values.
      char yyraw[size];
    } yybuffer_;

    /// Whether the content is built: if defined, the name of the stored type.
    const std::type_info *yytypeid_;
  };

#else
    typedef YYSTYPE semantic_type;
#endif

    /// Syntax errors thrown from user actions.
    struct syntax_error : std::runtime_error
    {
      syntax_error (const std::string& m)
        : std::runtime_error (m)
      {}

      syntax_error (const syntax_error& s)
        : std::runtime_error (s.what ())
      {}

      ~syntax_error () YY_NOEXCEPT YY_NOTHROW;
    };

    /// Tokens.
    struct token
    {
      enum yytokentype
      {
        KW_ATTR_BEG = 258,
        KW_ATTR_END = 259,
        KW_WORLD_BEG = 260,
        KW_WORLD_END = 261,
        KW_OBJ_BEG = 262,
        KW_OBJ_END = 263,
        KW_TRANS_BEG = 264,
        KW_TRANS_END = 265,
        KW_TRANSLATE = 266,
        KW_SCALE = 267,
        KW_ROTATE = 268,
        KW_IDENTITY = 269,
        KW_COORD_SYSTEM = 270,
        KW_COORD_SYSTEM_TRANS = 271,
        KW_TRANSFORM = 272,
        KW_CONCAT_TRANSFORM = 273,
        KW_LOOKAT = 274,
        KW_CAMERA = 275,
        KW_INTEGRATOR = 276,
        KW_SAMPLER = 277,
        KW_FILM = 278,
        KW_ACCELERATOR = 279,
        KW_FILTER = 280,
        KW_MATERIAL = 281,
        KW_TEXTURE = 282,
        KW_SHAPE = 283,
        KW_NAMED_MEDIUM = 284,
        KW_MAKE_NAMED_MEDIUM = 285,
        KW_OBJ_INST = 286,
        KW_MEDIUM_INTERFACE = 287,
        KW_NAMED_MATERIAL = 288,
        KW_MAKE_NAMED_MATERIAL = 289,
        KW_LIGHT_SRC = 290,
        KW_AREA_LIGHT_SRC = 291,
        STRLIT = 292,
        NUMBER = 293,
        KW_REVERSE_ORIENTATION = 294
      };
    };

    /// (External) token type, as returned by yylex.
    typedef token::yytokentype token_type;

    /// Symbol type: an internal symbol number.
    typedef int symbol_number_type;

    /// The symbol type number to denote an empty symbol.
    enum { empty_symbol = -2 };

    /// Internal symbol number for tokens (subsumed by symbol_number_type).
    typedef signed char token_number_type;

    /// A complete symbol.
    ///
    /// Expects its Base type to provide access to the symbol type
    /// via type_get ().
    ///
    /// Provide access to semantic value.
    template <typename Base>
    struct basic_symbol : Base
    {
      /// Alias to Base.
      typedef Base super_type;

      /// Default constructor.
      basic_symbol ()
        : value ()
      {}

#if 201103L <= YY_CPLUSPLUS
      /// Move constructor.
      basic_symbol (basic_symbol&& that);
#endif

      /// Copy constructor.
      basic_symbol (const basic_symbol& that);

      /// Constructor for valueless symbols, and symbols from each type.
#if 201103L <= YY_CPLUSPLUS
      basic_symbol (typename Base::kind_type t)
        : Base (t)
      {}
#else
      basic_symbol (typename Base::kind_type t)
        : Base (t)
      {}
#endif
#if 201103L <= YY_CPLUSPLUS
      basic_symbol (typename Base::kind_type t, float&& v)
        : Base (t)
        , value (std::move (v))
      {}
#else
      basic_symbol (typename Base::kind_type t, const float& v)
        : Base (t)
        , value (v)
      {}
#endif
#if 201103L <= YY_CPLUSPLUS
      basic_symbol (typename Base::kind_type t, int&& v)
        : Base (t)
        , value (std::move (v))
      {}
#else
      basic_symbol (typename Base::kind_type t, const int& v)
        : Base (t)
        , value (v)
      {}
#endif
#if 201103L <= YY_CPLUSPLUS
      basic_symbol (typename Base::kind_type t, std::shared_ptr<transform_node_t> && v)
        : Base (t)
        , value (std::move (v))
      {}
#else
      basic_symbol (typename Base::kind_type t, const std::shared_ptr<transform_node_t> & v)
        : Base (t)
        , value (v)
      {}
#endif
#if 201103L <= YY_CPLUSPLUS
      basic_symbol (typename Base::kind_type t, std::string&& v)
        : Base (t)
        , value (std::move (v))
      {}
#else
      basic_symbol (typename Base::kind_type t, const std::string& v)
        : Base (t)
        , value (v)
      {}
#endif
#if 201103L <= YY_CPLUSPLUS
      basic_symbol (typename Base::kind_type t, std::vector<float> && v)
        : Base (t)
        , value (std::move (v))
      {}
#else
      basic_symbol (typename Base::kind_type t, const std::vector<float> & v)
        : Base (t)
        , value (v)
      {}
#endif
#if 201103L <= YY_CPLUSPLUS
      basic_symbol (typename Base::kind_type t, std::vector<param_node_t> && v)
        : Base (t)
        , value (std::move (v))
      {}
#else
      basic_symbol (typename Base::kind_type t, const std::vector<param_node_t> & v)
        : Base (t)
        , value (v)
      {}
#endif
#if 201103L <= YY_CPLUSPLUS
      basic_symbol (typename Base::kind_type t, std::vector<std::string> && v)
        : Base (t)
        , value (std::move (v))
      {}
#else
      basic_symbol (typename Base::kind_type t, const std::vector<std::string> & v)
        : Base (t)
        , value (v)
      {}
#endif
#if 201103L <= YY_CPLUSPLUS
      basic_symbol (typename Base::kind_type t, std::vector<swoption_t> && v)
        : Base (t)
        , value (std::move (v))
      {}
#else
      basic_symbol (typename Base::kind_type t, const std::vector<swoption_t> & v)
        : Base (t)
        , value (v)
      {}
#endif
#if 201103L <= YY_CPLUSPLUS
      basic_symbol (typename Base::kind_type t, std::vector<worlditem_node_t>&& v)
        : Base (t)
        , value (std::move (v))
      {}
#else
      basic_symbol (typename Base::kind_type t, const std::vector<worlditem_node_t>& v)
        : Base (t)
        , value (v)
      {}
#endif
#if 201103L <= YY_CPLUSPLUS
      basic_symbol (typename Base::kind_type t, swoption_t&& v)
        : Base (t)
        , value (std::move (v))
      {}
#else
      basic_symbol (typename Base::kind_type t, const swoption_t& v)
        : Base (t)
        , value (v)
      {}
#endif
#if 201103L <= YY_CPLUSPLUS
      basic_symbol (typename Base::kind_type t, worlditem_node_t&& v)
        : Base (t)
        , value (std::move (v))
      {}
#else
      basic_symbol (typename Base::kind_type t, const worlditem_node_t& v)
        : Base (t)
        , value (v)
      {}
#endif

      /// Destroy the symbol.
      ~basic_symbol ()
      {
        clear ();
      }

      /// Destroy contents, and record that is empty.
      void clear ()
      {
        // User destructor.
        symbol_number_type yytype = this->type_get ();
        basic_symbol<Base>& yysym = *this;
        (void) yysym;
        switch (yytype)
        {
       default:
          break;
        }

        // Type destructor.
switch (yytype)
    {
      case 38: // NUMBER
        value.template destroy< float > ();
        break;

      case 43: // Scene
        value.template destroy< int > ();
        break;

      case 52: // Transform
        value.template destroy< std::shared_ptr<transform_node_t>  > ();
        break;

      case 37: // STRLIT
        value.template destroy< std::string > ();
        break;

      case 67: // NumberList
        value.template destroy< std::vector<float>  > ();
        break;

      case 69: // ParamList
        value.template destroy< std::vector<param_node_t>  > ();
        break;

      case 68: // StrList
        value.template destroy< std::vector<std::string>  > ();
        break;

      case 44: // SWOptionList
        value.template destroy< std::vector<swoption_t>  > ();
        break;

      case 53: // World
      case 54: // WorldItemList
        value.template destroy< std::vector<worlditem_node_t> > ();
        break;

      case 45: // SWOptionItem
      case 46: // Camera
      case 47: // Sampler
      case 48: // Film
      case 49: // Filter
      case 50: // Integrator
      case 51: // Accel
        value.template destroy< swoption_t > ();
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
        value.template destroy< worlditem_node_t > ();
        break;

      default:
        break;
    }

        Base::clear ();
      }

      /// Whether empty.
      bool empty () const YY_NOEXCEPT;

      /// Destructive move, \a s is emptied into this.
      void move (basic_symbol& s);

      /// The semantic value.
      semantic_type value;

    private:
#if YY_CPLUSPLUS < 201103L
      /// Assignment operator.
      basic_symbol& operator= (const basic_symbol& that);
#endif
    };

    /// Type access provider for token (enum) based symbols.
    struct by_type
    {
      /// Default constructor.
      by_type ();

#if 201103L <= YY_CPLUSPLUS
      /// Move constructor.
      by_type (by_type&& that);
#endif

      /// Copy constructor.
      by_type (const by_type& that);

      /// The symbol type as needed by the constructor.
      typedef token_type kind_type;

      /// Constructor from (external) token numbers.
      by_type (kind_type t);

      /// Record that this symbol is empty.
      void clear ();

      /// Steal the symbol type from \a that.
      void move (by_type& that);

      /// The (internal) type number (corresponding to \a type).
      /// \a empty when empty.
      symbol_number_type type_get () const YY_NOEXCEPT;

      /// The symbol type.
      /// \a empty_symbol when empty.
      /// An int, not token_number_type, to be able to store empty_symbol.
      int type;
    };

    /// "External" symbols: returned by the scanner.
    struct symbol_type : basic_symbol<by_type>
    {
      /// Superclass.
      typedef basic_symbol<by_type> super_type;

      /// Empty symbol.
      symbol_type () {}

      /// Constructor for valueless symbols, and symbols from each type.
#if 201103L <= YY_CPLUSPLUS
      symbol_type (int tok)
        : super_type(token_type (tok))
      {
        YY_ASSERT (tok == 0 || tok == token::KW_ATTR_BEG || tok == token::KW_ATTR_END || tok == token::KW_WORLD_BEG || tok == token::KW_WORLD_END || tok == token::KW_OBJ_BEG || tok == token::KW_OBJ_END || tok == token::KW_TRANS_BEG || tok == token::KW_TRANS_END || tok == token::KW_TRANSLATE || tok == token::KW_SCALE || tok == token::KW_ROTATE || tok == token::KW_IDENTITY || tok == token::KW_COORD_SYSTEM || tok == token::KW_COORD_SYSTEM_TRANS || tok == token::KW_TRANSFORM || tok == token::KW_CONCAT_TRANSFORM || tok == token::KW_LOOKAT || tok == token::KW_CAMERA || tok == token::KW_INTEGRATOR || tok == token::KW_SAMPLER || tok == token::KW_FILM || tok == token::KW_ACCELERATOR || tok == token::KW_FILTER || tok == token::KW_MATERIAL || tok == token::KW_TEXTURE || tok == token::KW_SHAPE || tok == token::KW_NAMED_MEDIUM || tok == token::KW_MAKE_NAMED_MEDIUM || tok == token::KW_OBJ_INST || tok == token::KW_MEDIUM_INTERFACE || tok == token::KW_NAMED_MATERIAL || tok == token::KW_MAKE_NAMED_MATERIAL || tok == token::KW_LIGHT_SRC || tok == token::KW_AREA_LIGHT_SRC || tok == token::KW_REVERSE_ORIENTATION || tok == 91 || tok == 93);
      }
#else
      symbol_type (int tok)
        : super_type(token_type (tok))
      {
        YY_ASSERT (tok == 0 || tok == token::KW_ATTR_BEG || tok == token::KW_ATTR_END || tok == token::KW_WORLD_BEG || tok == token::KW_WORLD_END || tok == token::KW_OBJ_BEG || tok == token::KW_OBJ_END || tok == token::KW_TRANS_BEG || tok == token::KW_TRANS_END || tok == token::KW_TRANSLATE || tok == token::KW_SCALE || tok == token::KW_ROTATE || tok == token::KW_IDENTITY || tok == token::KW_COORD_SYSTEM || tok == token::KW_COORD_SYSTEM_TRANS || tok == token::KW_TRANSFORM || tok == token::KW_CONCAT_TRANSFORM || tok == token::KW_LOOKAT || tok == token::KW_CAMERA || tok == token::KW_INTEGRATOR || tok == token::KW_SAMPLER || tok == token::KW_FILM || tok == token::KW_ACCELERATOR || tok == token::KW_FILTER || tok == token::KW_MATERIAL || tok == token::KW_TEXTURE || tok == token::KW_SHAPE || tok == token::KW_NAMED_MEDIUM || tok == token::KW_MAKE_NAMED_MEDIUM || tok == token::KW_OBJ_INST || tok == token::KW_MEDIUM_INTERFACE || tok == token::KW_NAMED_MATERIAL || tok == token::KW_MAKE_NAMED_MATERIAL || tok == token::KW_LIGHT_SRC || tok == token::KW_AREA_LIGHT_SRC || tok == token::KW_REVERSE_ORIENTATION || tok == 91 || tok == 93);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      symbol_type (int tok, float v)
        : super_type(token_type (tok), std::move (v))
      {
        YY_ASSERT (tok == token::NUMBER);
      }
#else
      symbol_type (int tok, const float& v)
        : super_type(token_type (tok), v)
      {
        YY_ASSERT (tok == token::NUMBER);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      symbol_type (int tok, std::string v)
        : super_type(token_type (tok), std::move (v))
      {
        YY_ASSERT (tok == token::STRLIT);
      }
#else
      symbol_type (int tok, const std::string& v)
        : super_type(token_type (tok), v)
      {
        YY_ASSERT (tok == token::STRLIT);
      }
#endif
    };

    /// Build a parser object.
    Parser (Tokenizer &tokenizer_yyarg, scene_t * scene_root_yyarg);
    virtual ~Parser ();

    /// Parse.  An alias for parse ().
    /// \returns  0 iff parsing succeeded.
    int operator() ();

    /// Parse.
    /// \returns  0 iff parsing succeeded.
    virtual int parse ();

#if YYDEBUG
    /// The current debugging stream.
    std::ostream& debug_stream () const YY_ATTRIBUTE_PURE;
    /// Set the current debugging stream.
    void set_debug_stream (std::ostream &);

    /// Type for debugging levels.
    typedef int debug_level_type;
    /// The current debugging level.
    debug_level_type debug_level () const YY_ATTRIBUTE_PURE;
    /// Set the current debugging level.
    void set_debug_level (debug_level_type l);
#endif

    /// Report a syntax error.
    /// \param msg    a description of the syntax error.
    virtual void error (const std::string& msg);

    /// Report a syntax error.
    void error (const syntax_error& err);

    // Implementation of make_symbol for each symbol type.
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_ATTR_BEG ()
      {
        return symbol_type (token::KW_ATTR_BEG);
      }
#else
      static
      symbol_type
      make_KW_ATTR_BEG ()
      {
        return symbol_type (token::KW_ATTR_BEG);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_ATTR_END ()
      {
        return symbol_type (token::KW_ATTR_END);
      }
#else
      static
      symbol_type
      make_KW_ATTR_END ()
      {
        return symbol_type (token::KW_ATTR_END);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_WORLD_BEG ()
      {
        return symbol_type (token::KW_WORLD_BEG);
      }
#else
      static
      symbol_type
      make_KW_WORLD_BEG ()
      {
        return symbol_type (token::KW_WORLD_BEG);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_WORLD_END ()
      {
        return symbol_type (token::KW_WORLD_END);
      }
#else
      static
      symbol_type
      make_KW_WORLD_END ()
      {
        return symbol_type (token::KW_WORLD_END);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_OBJ_BEG ()
      {
        return symbol_type (token::KW_OBJ_BEG);
      }
#else
      static
      symbol_type
      make_KW_OBJ_BEG ()
      {
        return symbol_type (token::KW_OBJ_BEG);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_OBJ_END ()
      {
        return symbol_type (token::KW_OBJ_END);
      }
#else
      static
      symbol_type
      make_KW_OBJ_END ()
      {
        return symbol_type (token::KW_OBJ_END);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_TRANS_BEG ()
      {
        return symbol_type (token::KW_TRANS_BEG);
      }
#else
      static
      symbol_type
      make_KW_TRANS_BEG ()
      {
        return symbol_type (token::KW_TRANS_BEG);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_TRANS_END ()
      {
        return symbol_type (token::KW_TRANS_END);
      }
#else
      static
      symbol_type
      make_KW_TRANS_END ()
      {
        return symbol_type (token::KW_TRANS_END);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_TRANSLATE ()
      {
        return symbol_type (token::KW_TRANSLATE);
      }
#else
      static
      symbol_type
      make_KW_TRANSLATE ()
      {
        return symbol_type (token::KW_TRANSLATE);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_SCALE ()
      {
        return symbol_type (token::KW_SCALE);
      }
#else
      static
      symbol_type
      make_KW_SCALE ()
      {
        return symbol_type (token::KW_SCALE);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_ROTATE ()
      {
        return symbol_type (token::KW_ROTATE);
      }
#else
      static
      symbol_type
      make_KW_ROTATE ()
      {
        return symbol_type (token::KW_ROTATE);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_IDENTITY ()
      {
        return symbol_type (token::KW_IDENTITY);
      }
#else
      static
      symbol_type
      make_KW_IDENTITY ()
      {
        return symbol_type (token::KW_IDENTITY);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_COORD_SYSTEM ()
      {
        return symbol_type (token::KW_COORD_SYSTEM);
      }
#else
      static
      symbol_type
      make_KW_COORD_SYSTEM ()
      {
        return symbol_type (token::KW_COORD_SYSTEM);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_COORD_SYSTEM_TRANS ()
      {
        return symbol_type (token::KW_COORD_SYSTEM_TRANS);
      }
#else
      static
      symbol_type
      make_KW_COORD_SYSTEM_TRANS ()
      {
        return symbol_type (token::KW_COORD_SYSTEM_TRANS);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_TRANSFORM ()
      {
        return symbol_type (token::KW_TRANSFORM);
      }
#else
      static
      symbol_type
      make_KW_TRANSFORM ()
      {
        return symbol_type (token::KW_TRANSFORM);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_CONCAT_TRANSFORM ()
      {
        return symbol_type (token::KW_CONCAT_TRANSFORM);
      }
#else
      static
      symbol_type
      make_KW_CONCAT_TRANSFORM ()
      {
        return symbol_type (token::KW_CONCAT_TRANSFORM);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_LOOKAT ()
      {
        return symbol_type (token::KW_LOOKAT);
      }
#else
      static
      symbol_type
      make_KW_LOOKAT ()
      {
        return symbol_type (token::KW_LOOKAT);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_CAMERA ()
      {
        return symbol_type (token::KW_CAMERA);
      }
#else
      static
      symbol_type
      make_KW_CAMERA ()
      {
        return symbol_type (token::KW_CAMERA);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_INTEGRATOR ()
      {
        return symbol_type (token::KW_INTEGRATOR);
      }
#else
      static
      symbol_type
      make_KW_INTEGRATOR ()
      {
        return symbol_type (token::KW_INTEGRATOR);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_SAMPLER ()
      {
        return symbol_type (token::KW_SAMPLER);
      }
#else
      static
      symbol_type
      make_KW_SAMPLER ()
      {
        return symbol_type (token::KW_SAMPLER);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_FILM ()
      {
        return symbol_type (token::KW_FILM);
      }
#else
      static
      symbol_type
      make_KW_FILM ()
      {
        return symbol_type (token::KW_FILM);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_ACCELERATOR ()
      {
        return symbol_type (token::KW_ACCELERATOR);
      }
#else
      static
      symbol_type
      make_KW_ACCELERATOR ()
      {
        return symbol_type (token::KW_ACCELERATOR);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_FILTER ()
      {
        return symbol_type (token::KW_FILTER);
      }
#else
      static
      symbol_type
      make_KW_FILTER ()
      {
        return symbol_type (token::KW_FILTER);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_MATERIAL ()
      {
        return symbol_type (token::KW_MATERIAL);
      }
#else
      static
      symbol_type
      make_KW_MATERIAL ()
      {
        return symbol_type (token::KW_MATERIAL);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_TEXTURE ()
      {
        return symbol_type (token::KW_TEXTURE);
      }
#else
      static
      symbol_type
      make_KW_TEXTURE ()
      {
        return symbol_type (token::KW_TEXTURE);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_SHAPE ()
      {
        return symbol_type (token::KW_SHAPE);
      }
#else
      static
      symbol_type
      make_KW_SHAPE ()
      {
        return symbol_type (token::KW_SHAPE);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_NAMED_MEDIUM ()
      {
        return symbol_type (token::KW_NAMED_MEDIUM);
      }
#else
      static
      symbol_type
      make_KW_NAMED_MEDIUM ()
      {
        return symbol_type (token::KW_NAMED_MEDIUM);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_MAKE_NAMED_MEDIUM ()
      {
        return symbol_type (token::KW_MAKE_NAMED_MEDIUM);
      }
#else
      static
      symbol_type
      make_KW_MAKE_NAMED_MEDIUM ()
      {
        return symbol_type (token::KW_MAKE_NAMED_MEDIUM);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_OBJ_INST ()
      {
        return symbol_type (token::KW_OBJ_INST);
      }
#else
      static
      symbol_type
      make_KW_OBJ_INST ()
      {
        return symbol_type (token::KW_OBJ_INST);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_MEDIUM_INTERFACE ()
      {
        return symbol_type (token::KW_MEDIUM_INTERFACE);
      }
#else
      static
      symbol_type
      make_KW_MEDIUM_INTERFACE ()
      {
        return symbol_type (token::KW_MEDIUM_INTERFACE);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_NAMED_MATERIAL ()
      {
        return symbol_type (token::KW_NAMED_MATERIAL);
      }
#else
      static
      symbol_type
      make_KW_NAMED_MATERIAL ()
      {
        return symbol_type (token::KW_NAMED_MATERIAL);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_MAKE_NAMED_MATERIAL ()
      {
        return symbol_type (token::KW_MAKE_NAMED_MATERIAL);
      }
#else
      static
      symbol_type
      make_KW_MAKE_NAMED_MATERIAL ()
      {
        return symbol_type (token::KW_MAKE_NAMED_MATERIAL);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_LIGHT_SRC ()
      {
        return symbol_type (token::KW_LIGHT_SRC);
      }
#else
      static
      symbol_type
      make_KW_LIGHT_SRC ()
      {
        return symbol_type (token::KW_LIGHT_SRC);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_AREA_LIGHT_SRC ()
      {
        return symbol_type (token::KW_AREA_LIGHT_SRC);
      }
#else
      static
      symbol_type
      make_KW_AREA_LIGHT_SRC ()
      {
        return symbol_type (token::KW_AREA_LIGHT_SRC);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_STRLIT (std::string v)
      {
        return symbol_type (token::STRLIT, std::move (v));
      }
#else
      static
      symbol_type
      make_STRLIT (const std::string& v)
      {
        return symbol_type (token::STRLIT, v);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_NUMBER (float v)
      {
        return symbol_type (token::NUMBER, std::move (v));
      }
#else
      static
      symbol_type
      make_NUMBER (const float& v)
      {
        return symbol_type (token::NUMBER, v);
      }
#endif
#if 201103L <= YY_CPLUSPLUS
      static
      symbol_type
      make_KW_REVERSE_ORIENTATION ()
      {
        return symbol_type (token::KW_REVERSE_ORIENTATION);
      }
#else
      static
      symbol_type
      make_KW_REVERSE_ORIENTATION ()
      {
        return symbol_type (token::KW_REVERSE_ORIENTATION);
      }
#endif


  private:
    /// This class is not copyable.
    Parser (const Parser&);
    Parser& operator= (const Parser&);

    /// Stored state numbers (used for stacks).
    typedef unsigned char state_type;

    /// Generate an error message.
    /// \param yystate   the state where the error occurred.
    /// \param yyla      the lookahead token.
    virtual std::string yysyntax_error_ (state_type yystate,
                                         const symbol_type& yyla) const;

    /// Compute post-reduction state.
    /// \param yystate   the current state
    /// \param yysym     the nonterminal to push on the stack
    static state_type yy_lr_goto_state_ (state_type yystate, int yysym);

    /// Whether the given \c yypact_ value indicates a defaulted state.
    /// \param yyvalue   the value to check
    static bool yy_pact_value_is_default_ (int yyvalue);

    /// Whether the given \c yytable_ value indicates a syntax error.
    /// \param yyvalue   the value to check
    static bool yy_table_value_is_error_ (int yyvalue);

    static const signed char yypact_ninf_;
    static const signed char yytable_ninf_;

    /// Convert a scanner token number \a t to a symbol number.
    /// In theory \a t should be a token_type, but character literals
    /// are valid, yet not members of the token_type enum.
    static token_number_type yytranslate_ (int t);

    // Tables.
    // YYPACT[STATE-NUM] -- Index in YYTABLE of the portion describing
    // STATE-NUM.
    static const short yypact_[];

    // YYDEFACT[STATE-NUM] -- Default reduction number in state STATE-NUM.
    // Performed when YYTABLE does not specify something else to do.  Zero
    // means the default is an error.
    static const signed char yydefact_[];

    // YYPGOTO[NTERM-NUM].
    static const signed char yypgoto_[];

    // YYDEFGOTO[NTERM-NUM].
    static const short yydefgoto_[];

    // YYTABLE[YYPACT[STATE-NUM]] -- What to do in state STATE-NUM.  If
    // positive, shift that token.  If negative, reduce the rule whose
    // number is the opposite.  If YYTABLE_NINF, syntax error.
    static const unsigned char yytable_[];

    static const signed char yycheck_[];

    // YYSTOS[STATE-NUM] -- The (internal number of the) accessing
    // symbol of state STATE-NUM.
    static const signed char yystos_[];

    // YYR1[YYN] -- Symbol number of symbol that rule YYN derives.
    static const signed char yyr1_[];

    // YYR2[YYN] -- Number of symbols on the right hand side of rule YYN.
    static const signed char yyr2_[];


#if YYDEBUG
    /// For a symbol, its name in clear.
    static const char* const yytname_[];

    // YYRLINE[YYN] -- Source line where rule number YYN was defined.
    static const short yyrline_[];
    /// Report on the debug stream that the rule \a r is going to be reduced.
    virtual void yy_reduce_print_ (int r);
    /// Print the state stack on the debug stream.
    virtual void yystack_print_ ();

    /// Debugging level.
    int yydebug_;
    /// Debug stream.
    std::ostream* yycdebug_;

    /// \brief Display a symbol type, value and location.
    /// \param yyo    The output stream.
    /// \param yysym  The symbol.
    template <typename Base>
    void yy_print_ (std::ostream& yyo, const basic_symbol<Base>& yysym) const;
#endif

    /// \brief Reclaim the memory associated to a symbol.
    /// \param yymsg     Why this token is reclaimed.
    ///                  If null, print nothing.
    /// \param yysym     The symbol.
    template <typename Base>
    void yy_destroy_ (const char* yymsg, basic_symbol<Base>& yysym) const;

  private:
    /// Type access provider for state based symbols.
    struct by_state
    {
      /// Default constructor.
      by_state () YY_NOEXCEPT;

      /// The symbol type as needed by the constructor.
      typedef state_type kind_type;

      /// Constructor.
      by_state (kind_type s) YY_NOEXCEPT;

      /// Copy constructor.
      by_state (const by_state& that) YY_NOEXCEPT;

      /// Record that this symbol is empty.
      void clear () YY_NOEXCEPT;

      /// Steal the symbol type from \a that.
      void move (by_state& that);

      /// The (internal) type number (corresponding to \a state).
      /// \a empty_symbol when empty.
      symbol_number_type type_get () const YY_NOEXCEPT;

      /// The state number used to denote an empty symbol.
      /// We use the initial state, as it does not have a value.
      enum { empty_state = 0 };

      /// The state.
      /// \a empty when empty.
      state_type state;
    };

    /// "Internal" symbol: element of the stack.
    struct stack_symbol_type : basic_symbol<by_state>
    {
      /// Superclass.
      typedef basic_symbol<by_state> super_type;
      /// Construct an empty symbol.
      stack_symbol_type ();
      /// Move or copy construction.
      stack_symbol_type (YY_RVREF (stack_symbol_type) that);
      /// Steal the contents from \a sym to build this.
      stack_symbol_type (state_type s, YY_MOVE_REF (symbol_type) sym);
#if YY_CPLUSPLUS < 201103L
      /// Assignment, needed by push_back by some old implementations.
      /// Moves the contents of that.
      stack_symbol_type& operator= (stack_symbol_type& that);

      /// Assignment, needed by push_back by other implementations.
      /// Needed by some other old implementations.
      stack_symbol_type& operator= (const stack_symbol_type& that);
#endif
    };

    /// A stack with random access from its top.
    template <typename T, typename S = std::vector<T> >
    class stack
    {
    public:
      // Hide our reversed order.
      typedef typename S::reverse_iterator iterator;
      typedef typename S::const_reverse_iterator const_iterator;
      typedef typename S::size_type size_type;
      typedef typename std::ptrdiff_t index_type;

      stack (size_type n = 200)
        : seq_ (n)
      {}

      /// Random access.
      ///
      /// Index 0 returns the topmost element.
      const T&
      operator[] (index_type i) const
      {
        return seq_[size_type (size () - 1 - i)];
      }

      /// Random access.
      ///
      /// Index 0 returns the topmost element.
      T&
      operator[] (index_type i)
      {
        return seq_[size_type (size () - 1 - i)];
      }

      /// Steal the contents of \a t.
      ///
      /// Close to move-semantics.
      void
      push (YY_MOVE_REF (T) t)
      {
        seq_.push_back (T ());
        operator[] (0).move (t);
      }

      /// Pop elements from the stack.
      void
      pop (std::ptrdiff_t n = 1) YY_NOEXCEPT
      {
        for (; 0 < n; --n)
          seq_.pop_back ();
      }

      /// Pop all elements from the stack.
      void
      clear () YY_NOEXCEPT
      {
        seq_.clear ();
      }

      /// Number of elements on the stack.
      index_type
      size () const YY_NOEXCEPT
      {
        return index_type (seq_.size ());
      }

      std::ptrdiff_t
      ssize () const YY_NOEXCEPT
      {
        return std::ptrdiff_t (size ());
      }

      /// Iterator on top of the stack (going downwards).
      const_iterator
      begin () const YY_NOEXCEPT
      {
        return seq_.rbegin ();
      }

      /// Bottom of the stack.
      const_iterator
      end () const YY_NOEXCEPT
      {
        return seq_.rend ();
      }

      /// Present a slice of the top of a stack.
      class slice
      {
      public:
        slice (const stack& stack, index_type range)
          : stack_ (stack)
          , range_ (range)
        {}

        const T&
        operator[] (index_type i) const
        {
          return stack_[range_ - i];
        }

      private:
        const stack& stack_;
        index_type range_;
      };

    private:
      stack (const stack&);
      stack& operator= (const stack&);
      /// The wrapped container.
      S seq_;
    };


    /// Stack type.
    typedef stack<stack_symbol_type> stack_type;

    /// The stack.
    stack_type yystack_;

    /// Push a new state on the stack.
    /// \param m    a debug message to display
    ///             if null, no trace is output.
    /// \param sym  the symbol
    /// \warning the contents of \a s.value is stolen.
    void yypush_ (const char* m, YY_MOVE_REF (stack_symbol_type) sym);

    /// Push a new look ahead token on the state on the stack.
    /// \param m    a debug message to display
    ///             if null, no trace is output.
    /// \param s    the state
    /// \param sym  the symbol (for its value and location).
    /// \warning the contents of \a sym.value is stolen.
    void yypush_ (const char* m, state_type s, YY_MOVE_REF (symbol_type) sym);

    /// Pop \a n symbols from the stack.
    void yypop_ (int n = 1);

    /// Some specific tokens.
    static const token_number_type yy_error_token_ = 1;
    static const token_number_type yy_undef_token_ = 2;

    /// Constants.
    enum
    {
      yyeof_ = 0,
      yylast_ = 368,     ///< Last index in yytable_.
      yynnts_ = 28,  ///< Number of nonterminal symbols.
      yyfinal_ = 3, ///< Termination state number.
      yyntokens_ = 42  ///< Number of tokens.
    };


    // User arguments.
    Tokenizer &tokenizer;
    scene_t * scene_root;
  };


#line 5 "syntax.ypp"
} // pbr
#line 1803 "syntax.tab.hpp"





#endif // !YY_YY_SYNTAX_TAB_HPP_INCLUDED
