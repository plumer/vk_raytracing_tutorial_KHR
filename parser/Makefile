all: syntax.tab.cpp lex.yy.cpp
CXXFILES = syntax.tab.cpp lex.yy.cpp tree.cpp tokenizer.cpp
CXXFLAGS = -std=c++17 -g
OBJS=$(CXXFILES:.cpp=.o)
HEADERS = syntax.tab.hpp lex.yy.hpp tokenizer.h tree.h

parser: $(HEADERS) $(OBJS) local_test.cpp
	g++ -DUSE_MAIN_TEST $(CXXFLAGS) $(OBJS) local_test.cpp -o parser

syntax.tab.cpp: syntax.ypp
	bison syntax.ypp

lex.yy.cpp: lexical++.l
	flex --nounistd --header-file=lex.yy.hpp --outfile=lex.yy.cpp lexical++.l
	
.PHONY: clean

clean:
	rm -rf lex.yy.cpp lex.yy.h syntax.tab.h syntax.tab.cpp syntax.output parser
