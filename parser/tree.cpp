#include "tree.h"

#include <iostream>
#include <vector>
#include <cassert>

static int depth = 0;
static void print_indent() {
    for (int i = 0; i < depth * 2; ++i) putchar(' ');
}

void print_swoption(const swoption_t & opt);
void print_param(const std::vector<param_node_t> &parameters);
void print_transform(const transform_node_t &trans);
void print_worlditem(const worlditem_node_t &w);

void print_scene(const scene_t & scene) {

    printf("%zu scene-wide options:\n", scene.swoptions.size());
    const int nOptions = scene.swoptions.size();
    for (size_t i = 0; i < scene.swoptions.size(); ++i) {
        const auto &option = scene.swoptions[i];
        printf("Option %zu/%d:\n", i+1, nOptions);
        print_swoption(option);
    }

    printf("%zu world items:\n", scene.world_items.size());
    for (size_t i = 0; i < scene.world_items.size(); ++i) {
        const auto & w_item = scene.world_items[i];
        printf("World item %zu/%zu\n", i+1, scene.world_items.size());
        print_worlditem(w_item);
    }
}

void print_swoption(const swoption_t &opt) {
    depth++;
    print_indent();
    static const char * dir_to_str[] = {
        "Camera", "Sampler", "Film", "Filter", "Integrator", "Accel", "Transform"
    };
    printf("Directive %s with implementation %s\n",
        dir_to_str[int(opt.directive)], opt.implementation.c_str());
    if (opt.directive == swoption_t::directive_t::Transform) {
        assert(opt.trans);
        print_transform(*opt.trans);
    } else {
        print_param(opt.params);
    }
    depth--;
}

void print_param(const std::vector<param_node_t> & params) {
    depth++;

    int nParams = params.size();
    print_indent();
    printf("%d parameters:\n", nParams);
        
    for (size_t i = 0; i < params.size(); ++i) {
        const auto &param = params[i];
        print_indent();
        printf("Parameter %zu/%d:\n", i+1 , nParams);
        depth++;
        print_indent();
        printf("%s: [", param.key.c_str());

        if (!param.strings.empty()) {
            for (const std::string &s : param.strings) {
                printf(" %s ", s.c_str());
            }
        } else if (!param.numbers.empty()) {
            for (float f : param.numbers) {
                printf(" %g ", f);
            }
        }
        puts("]");
        depth--;
    }

    depth--;
    fflush(stdout);
}

void print_transform(const transform_node_t & trans) {
    depth++;
    using trans_t = transform_node_t::type;
    print_indent();
    switch(trans.t) {
        case trans_t::Identity:
            printf("Identity\n");
            break;
        case trans_t::Translate:
            printf("Translation by [%g %g %g]\n", trans.numbers[0], trans.numbers[1], 
                   trans.numbers[2]);
            break;
        case trans_t::Scale:
            printf("Scaling by [%g %g %g]\n", trans.numbers[0], trans.numbers[1], trans.numbers[2]);
            break;
        case trans_t::Rotate:
            printf("Rotation by %g: [%g %g %g]\n", 
                    trans.numbers[0], trans.numbers[1], trans.numbers[2], trans.numbers[3]);
            break;
        case trans_t::LookAt:
            {
                const auto & n = trans.numbers;
                printf("LookAt by [%g %g %g] -> [%g %g %g], y = [%g %g %g]\n",
                    n[0], n[1], n[2], n[3], n[4], n[5], n[6], n[7], n[8]);
            }
            break;
        case trans_t::CoordSys:
            printf("CoordSys : %s\n", trans.str.c_str());
            break;
        case trans_t::CoordSysTrans:
            printf("CoordSysTrans : %s\n", trans.str.c_str());
            break;
        case trans_t::Mat4: 
            printf("mat4x4: [");
            for (float x : trans.numbers) printf("%g, ", x);
            printf("]\n");
            break;
        case trans_t::ConcatMat4: 
            printf("mat4x4: [");
            for (float x : trans.numbers) printf("%g, ", x);
            printf("]\n");
            break;
        default:
            printf("Unhandled enumeration: %d\n", static_cast<int>(trans.t));
            assert(false);
    }
    depth--;
}

void print_worlditem(const worlditem_node_t &world_item) {
    depth++;
    using wi_t = worlditem_node_t::directive_t;
    print_indent();
    static const char * directive_names[] = {
        "Transform",
        "Shape", "Material", "Light", "AreaLight", "Media", "Texture",
        "AttrPair", "ObjPair", "TransPair",
        "MakeMedium", "MakeMaterial",
        "ReverseOrientation",
        "ObjInst", "MaterialInst", "MediaInst"
    };
    switch(world_item.directive) {
        case wi_t::Transform:
            printf("Transform:\n");
            print_transform(*world_item.trans);
            break;
        
        case wi_t::Shape:
        case wi_t::Material:
        case wi_t::Light:
        case wi_t::AreaLight:
        case wi_t::Media:
            printf("Directive \'%s\' with implementation %s\n",
                    directive_names[ static_cast<int>(world_item.directive) ],
                    world_item.implementation.c_str());
            print_param(world_item.params);
            break;
        case wi_t::Texture:
            printf("Directive \'%s\' with implementation %s, ",
                    directive_names[ static_cast<int>(world_item.directive) ],
                    world_item.implementation.c_str());
            printf("Texture type %s and class %s\n", world_item.tex_type.c_str(), 
                   world_item.implementation.c_str());
            print_param(world_item.params);
            break;
        case wi_t::AttrPair:
        case wi_t::ObjPair:
        case wi_t::TransPair:
            printf("Environment pair \'%s\'", 
                   directive_names[ static_cast<int>(world_item.directive)]);
            if (world_item.directive == wi_t::ObjPair) {
                printf(" with name %s", world_item.object_name.c_str());
            }
            putchar('\n');
            {
                print_indent();
                printf("%zu world items:\n", world_item.child.size());
                for (size_t i = 0; i < world_item.child.size(); ++i) {
                    const auto &child_item = world_item.child[i];
                    print_indent();
                    printf("World item %zu/%zu\n", i+1, world_item.child.size());
                    print_worlditem(child_item);
                }                
            }
            break;
        case wi_t::MakeMaterial:
        case wi_t::MakeMedium:
            printf("Maker \'%s\'", directive_names[ static_cast<int>(world_item.directive) ] );
            puts("unimplemented: print_param(world_item.maked_params)");
            break;
        case wi_t::ObjInst:
        case wi_t::MaterialInst:
        case wi_t::MediaInst:
            printf("%s instantiation with name %s\n", 
                directive_names[ static_cast<int>(world_item.directive) ],
                world_item.inst_name.c_str());
            break;
        default:
            printf("Not implemented to print\n");
    }

    depth--;
}