#ifndef TREE_H
#define TREE_H
#include <vector>
#include <string>
#include <cassert>
#include <memory>

static inline std::string strip_quotes(const std::string & in) {
    std::string out;
    if (in.front() == '\"')
        out = in.substr(1);
    else
        out = in;
    if (out.back() == '\"')
        out.pop_back();
    return out;
}

// key data structure
struct param_node_t {
    std::string key;
    std::vector<std::string> strings;
    std::vector<float> numbers;

    param_node_t() = default;
};

struct transform_node_t {
    enum class type {
        Identity, Translate, Scale, Rotate, LookAt, CoordSys, CoordSysTrans,
        Mat4, ConcatMat4
    } t;
    std::vector<float> numbers;
    std::string str;
};

struct swoption_t {
    enum class directive_t {
        Camera, Sampler, Film, Filter, Integrator, Accel, Transform
    } directive;
    // every directive has its own implementation
    std::string implementation;
    std::vector<param_node_t> params ;

    // except Transforms
    std::shared_ptr<struct transform_node_t> trans = nullptr;

    swoption_t() = default;
    explicit swoption_t(directive_t d) : directive{ d } {}
};

struct worlditem_node_t {
    enum class directive_t {
        Transform,
        Shape, Material, Light, AreaLight, Media, Texture,
        AttrPair, ObjPair, TransPair,
        MakeMedium, MakeMaterial,
        ReverseOrientation,
        ObjInst, MaterialInst, MediaInst
    } directive;

    // for shape, material, light, arealight and media
    std::string implementation;
    std::string tex_type, tex_name;
    std::vector <param_node_t> params;

    // for attr_pair, obj_pair and trans_pair
    std::vector<struct worlditem_node_t> child;
    std::string object_name;

    // for transform
    std::shared_ptr<struct transform_node_t> trans = nullptr;

    // for makers
    std::string maked_name;
    struct param_node_t * maked_params = nullptr;

    // for instantiators
    std::string inst_name;

    worlditem_node_t() = default;
    explicit worlditem_node_t(directive_t d) : directive{ d } {}
};

struct scene_t {
    std::vector<swoption_t> swoptions;
    std::vector<worlditem_node_t> world_items;
};

void print_scene(const scene_t &);

void print_worlditem(const worlditem_node_t &w);

void print_param(const std::vector<param_node_t> & parameters);

#endif // TREE_H

