
#ifdef USE_MAIN_TEST


#include <iostream>
#include <fstream>

#include "tokenizer.h"
#include "syntax.tab.hpp"

int main(int argc, const char *argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << "<filename.pbrt>" << std::endl;
        return 1;
    }
    
    std::ifstream file_in(argv[1]);
    if (!file_in) {
        std::cerr << "Couldn't open " << argv[1] << std::endl;
        return 2;
    }
    
    pbr::Tokenizer tokenizer(&file_in);
    tokenizer.set_directory(argv[1]);
    scene_t scene_root;
    pbr::Parser parser(tokenizer, &scene_root);
    
    parser.parse();
    
    print_scene(scene_root);
    return 0;
}
#endif
