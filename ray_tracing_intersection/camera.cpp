#include "camera.h"

#include <chrono>
#include <glm/ext/matrix_transform.hpp>
#include <glm/geometric.hpp>

#include "logging.h"

namespace vkpbr {


CameraNavigator::Actions CameraNavigator::MouseMove(int x, int y, const Inputs& inputs) {
    if (!inputs.left_mouse && !inputs.right_mouse && !inputs.middle_mouse) {
        SetMousePosition(x, y);
        return Actions::kNoAction;
    }

    Actions action = Actions::kNoAction;
    if (inputs.left_mouse) {
        if (inputs.alt_key || (inputs.ctrl_key && inputs.shift_key))
            action = mode_ == Modes::kExamine ? Actions::kLookAround : Actions::kOrbit;
        else if (inputs.shift_key)
            action = Actions::kDolly;
        else if (inputs.ctrl_key)
            action = Actions::kPan;
        else
            action = mode_ == Modes::kExamine ? Actions::kOrbit : Actions::kLookAround;
    } else if (inputs.middle_mouse) {
        action = Actions::kPan;
    } else if (inputs.right_mouse) {
        action = Actions::kDolly;
    }

    if (action != Actions::kNoAction)
        Motion(x, y, action);
    return action;
}

void CameraNavigator::SetLookAt(const glm::vec3& eye, const glm::vec3& target, const glm::vec3& up,
                                bool animated) {
    if (!animated) {
        current_    = {eye, target, up};
        goal_       = current_;
        start_time_ = 0;
    } else {
        goal_       = {eye, target, up};
        snapshot_   = current_;
        start_time_ = GetSystemTime();
        // Find bezier points.
        {
            glm::vec3 p0 = current_.eye;
            glm::vec3 p2 = goal_.eye;
            glm::vec3 p1, pc;

            auto pi = (goal_.center + current_.center) * 0.5f;

            auto  p02    = (p0 + p2) * 0.5f;
            float radius = (glm::distance(p0, pi) + glm::distance(p2, pi)) * 0.5f;
            auto  p02pi  = p02 - pi;

            p02pi = glm::normalize(p02pi);
            p02pi *= radius;
            pc   = pi + p02pi;
            p1   = 2.0f * pc - p0 * 0.5f - p2 * 0.5f;
            p1.y = p02.y;

            bezier_[0] = p0;
            bezier_[1] = p1;
            bezier_[2] = p2;
        }
    }
    Update();
}

void CameraNavigator::UpdateAnimation() {
    auto elapse = cast_f32(GetSystemTime() - start_time_) / 1000.f;
    if (elapse > duration_)
        return;

    float t = elapse / cast_f32(duration_);
    t       = t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);

#define LERP(a, b, t) (a) * (1 - t) + (b)*t
    current_.center = LERP(snapshot_.center, goal_.center, t);
    current_.up     = LERP(snapshot_.up, goal_.up, t);
    // Computes the eye position using the bezier control points.
    current_.eye = LERP(LERP(bezier_[0], bezier_[1], t), LERP(bezier_[1], bezier_[2], t), t);

    Update();
}


void CameraNavigator::SetMatrix(glm::mat4 matrix) {
    current_.eye         = glm::vec3(matrix[3]);
    auto rotation_matrix = glm::mat3(matrix);
    current_.center      = {0.0f, 0.0f, -1.0f};
    current_.center      = rotation_matrix * current_.center;
    current_.up          = {0.0f, 1.0f, 0.0};

    goal_       = current_;
    start_time_ = 0;

    view_matrix_ = glm::inverse(matrix);
}

void CameraNavigator::Motion(int x, int y, Actions action) {
    f32 dx = cast_f32(x - mouse_position_.x) / cast_f32(width_);
    f32 dy = cast_f32(y - mouse_position_.y) / cast_f32(height_);
    switch (action) {
        case Actions::kOrbit:
            if (mode_ == Modes::kTrackball)
                Orbit(dx, dy, true);
            else
                Orbit(dx, dy, false);
            break;
        case Actions::kDolly:
            Dolly(dx, dy);
            break;
        case Actions::kPan:
            Pan(dx, dy);
            break;
        case Actions::kLookAround:
            if (mode_ == Modes::kTrackball)
                Trackball(x, y);
            else
                Orbit(dx, -dy, true);
            break;
        default:
            LOG(FATAL) << "unimplemented";
    }

    // Resets animation.
    start_time_ = 0;

    Update();

    mouse_position_ = glm::vec2(x, y);
}

void CameraNavigator::Wheel(int value, const Inputs& input) {
    f32 dx = cast_f32(value * std::abs(value)) / cast_f32(width_);
    if (input.shift_key)
        fov_ += value;
    else {
        glm::vec3 z = current_.eye - current_.center;

        f32 length = glm::length(z) * 0.1f;
        length     = std::max(length, 0.001f);

        Dolly(dx * speed_, dx * speed_);
        Update();
    }
}

void CameraNavigator::Fit(const glm::vec3& box_min, const glm::vec3 box_max, bool animated) {
    Camera    v_camera   = GetLookAt();
    glm::vec3 view_dir   = glm::normalize(v_camera.eye - v_camera.center);
    glm::vec3 box_size   = (box_max - box_min) * 0.5f;
    glm::vec3 box_center = box_min + box_size;

    const float kAspectRatio = 1;
    const float kFov         = glm::radians(Fov());

    // Projects the box to the camera.
    float     radius = 0;
    float     offset = 0;
    glm::mat4 mat    = glm::lookAt(v_camera.eye, box_center, v_camera.up);
    mat[3]           = glm::vec4(0, 0, 0, 1);

    for (int i = 0; i < 8; ++i) {
        glm::vec3 corner = box_size;
        if (i & 1)
            corner.x *= -1;
        if (i & 2)
            corner.y *= -1;
        if (i & 4)
            corner.z *= -1;
        corner     = mat * glm::vec4(corner, 1);
        float dist = std::max(std::abs(corner.x), std::abs(corner.y) / kAspectRatio);
        radius     = std::max(radius, dist);
        offset     = std::max(offset, std::abs(corner.z));
    }

    // Places the camera back.
    float dist   = radius / std::tan(kFov * 0.5f);
    v_camera.eye = box_center + view_dir * (dist + offset);
    SetLookAt(v_camera.eye, box_center, v_camera.up, animated);
}

void CameraNavigator::Update() {
    view_matrix_ = glm::lookAt(current_.eye, current_.center, current_.up);
    if (roll_ != 0.0f) {
        glm::mat4 rotation = glm::rotate(glm::mat4(1.0f), roll_, {0, 0, 1});
        view_matrix_       = view_matrix_ * rotation;
    }
}

void CameraNavigator::Pan(float dx, float dy) {
    if (mode_ == Modes::kFly) {
        dx *= -1;
        dy *= -1;
    }

    glm::vec3 z      = glm::normalize(current_.eye - current_.center);
    glm::vec3 x      = glm::normalize(glm::cross(current_.up, z));
    glm::vec3 y      = glm::normalize(glm::cross(z, x));
    f32       length = cast_f32(glm::length(z)) / 0.785f;

    x *= -dx * length;
    y *= dy * length;

    current_.eye += x + y;
    current_.center += x + y;
}

void CameraNavigator::Orbit(float dx, float dy, bool invert) {
    if (dx == 0 && dy == 0)
        return;

    dx *= glm::two_pi<f32>();
    dy *= glm::two_pi<f32>();

    glm::vec3 origin = invert ? current_.eye : current_.center;
    glm::vec3 target = invert ? current_.center : current_.eye;

    glm::vec3 center_to_eye = target - origin;
    float     radius        = glm::length(center_to_eye);
    center_to_eye           = glm::normalize(center_to_eye);

    glm::mat4 rotation_x, rotation_y;

    glm::vec3 axe_z = center_to_eye;
    rotation_y      = glm::rotate(glm::mat4(1.0f), -dx, current_.up);

    // Applies the y-rotation to the eye-center vector.
    center_to_eye = rotation_y * glm::vec4(center_to_eye, 0.0f);

    glm::vec3 axe_x = glm::normalize(glm::cross(current_.up, axe_z));
    rotation_x      = glm::rotate(glm::mat4(1.0f), -dy, axe_x);
    glm::vec3 tmp   = rotation_x * glm::vec4(center_to_eye, 0.0f);
    if (glm::sign(tmp.x) == glm::sign(center_to_eye.x))
        center_to_eye = tmp;

    center_to_eye *= radius;

    glm::vec3 new_target = center_to_eye + origin;

    if (invert)
        current_.center = new_target;
    else
        current_.eye = new_target;
}

void CameraNavigator::Dolly(float dx, float dy) {
    glm::vec3 z      = current_.center - current_.eye;
    float     length = glm::length(z);

    if (length < 1e-6f)  // We are very close to the point of interest, so don't move.
        return;

    float dd     = (mode_ != Modes::kExamine) ? -dy : (std::abs(dx) > std::abs(dy) ? dx : -dy);
    float factor = speed_ * dd / length;

    length = std::max(0.001f, length / 10);
    factor *= length;

    if (factor >= 1.0f)
        return;

    z *= factor;

    if (mode_ == Modes::kWalk) {
        if (current_.up.y > current_.up.z)
            z.y = 0;
        else
            z.z = 0;
    }

    current_.eye += z;
    if (mode_ != Modes::kExamine)
        current_.center += z;
}

void CameraNavigator::Trackball(int x, int y) {
#pragma warning(disable : 26451)
    glm::vec2 p0(2.0 * (mouse_position_[0] - width_ / 2.0) / double(width_),
                 2.0 * (height_ / 2.0 - mouse_position_[1]) / double(height_));
    glm::vec2 p1(2.0 * (x - width_ / 2) / cast_f64(width_),
                 2 * (height_ / 2 - y) / cast_f64(height_));

    glm::vec3 p0_trackball(p0, ProjectOntoTrackballSphere(p0));
    glm::vec3 p1_trackball(p1, ProjectOntoTrackballSphere(p1));

    glm::vec3 axis = glm::normalize(glm::cross(p0_trackball, p1_trackball));

    double t = glm::distance(p0_trackball, p1_trackball) / (2.0 * trackball_size_);
    t        = glm::clamp(t, -1.0, 1.0);

    float angle_rad = cast_f32(2.0 * std::asin(t));
    {
        glm::vec3 rotation_axis   = view_matrix_ * glm::vec4(axis, 0.0);
        glm::mat4 rotation_matrix = glm::rotate(glm::mat4(1.0f), angle_rad, rotation_axis);

        glm::vec3 point = current_.eye - current_.center;
        point           = rotation_matrix * glm::vec4(point, 1.0f);
        current_.eye    = current_.center + glm::vec3(point);
        glm::vec4 up    = rotation_matrix * glm::vec4(current_.up, 0.0f);
        current_.up     = glm::vec3(up);
    }
}

double CameraNavigator::ProjectOntoTrackballSphere(const glm::vec2& p) const {
    double z;
    double d = std::hypot(p.x, p.y);
    if (d < trackball_size_ * glm::quarter_pi<f32>()) {
        // Inside sphere
        z = std::sqrt(trackball_size_ * trackball_size_ - d * d);
    } else {
        // On Hyperbola
        double t = trackball_size_ / glm::root_two<f32>();
        z        = t * t / d;
    }
    return z;
}

double CameraNavigator::GetSystemTime() {
    auto now      = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(duration).count() / 1000.0;
}

// +------------------------------------------------------------------+
// | Inertia Camera implementation
// +------------------------------------------------------------------+

InertiaCamera::InertiaCamera(const glm::vec3 eye, const glm::vec3 focus, const glm::vec3 object)
    : eye_(eye)
    , focus_(focus)
    , object_(object)
    , current_eye_(eye)
    , current_focus_(focus)
    , current_object_(object) {
    view_matrix_ = glm::lookAt(current_eye_, current_focus_, kYBase);
}

void InertiaCamera::RotateH(float speed, bool panning) {
    glm::vec3 p = eye_, o = focus_;
    glm::vec3 po = p - o;
    float     l  = glm::distance(eye_, focus_);

    glm::vec3 dv = glm::cross(po, glm::vec3(0, 1, 0));
    p += dv * speed;
    float l2 = glm::distance(p, focus_);
    l        = l2 - l;
    eye_ = p - l/l2 * po;
    if (panning)
        focus_ += dv * speed;
}

void InertiaCamera::RotateV(float speed, bool panning) {
    float l = glm::distance(eye_, focus_);
    glm::vec3 dv = glm::normalize(glm::cross(eye_ - focus_, glm::vec3(0, -1, 0)));
    glm::vec3 dv2 = glm::cross(eye_ - focus_, dv);
    glm::vec3 p   = eye_ + dv2 * speed;
    float     l2  = glm::distance(p, focus_);

    if (panning)
        focus_ += dv2 * speed;

    // Avoids gimbal lock.
    if (std::abs(glm::dot((p - focus_) / l2, kYBase)) > 0.99)
        return;

    l = l2 - 1l;
    eye_ = p - l / l2 * (p - focus_);
}

void InertiaCamera::Move(float speed, bool panning) {
    glm::vec3 translation = (eye_ - focus_) * speed;
    if (panning)
        focus_ -= translation;
    eye_ -= translation;
}

bool InertiaCamera::Update(float dt_sec) {
    dt_sec = std::max(dt_sec, 1.0f / 60.0f);

    bool b_continue = false;
    static glm::vec3 eye_velocity(0.0f, 0.0f, 0.0f);
    static glm::vec3 eye_acceleration(0.0f, 0.0f, 0.0f);
    eye_d_ = glm::distance(current_eye_, eye_);

    if (eye_d_ > epsilon_) {
        b_continue = true;
        glm::vec3 dV = current_eye_ - eye_;
        eye_acceleration = (-2.0f / tau_) * eye_velocity - dV / (tau_ * tau_);
        // Integrates velocity and acceleration.
        eye_velocity += eye_acceleration * dt_sec;
        current_eye_ += eye_velocity * dt_sec;
    } else {
        eye_velocity = eye_acceleration = glm::vec3(0, 0, 0);
    }

    static glm::vec3 focus_velocity(0.0f, 0.0f, 0.0f);
    static glm::vec3 focus_acceleration(0.0f, 0.0f, 0.0f);
    focus_d_ = glm::distance(current_focus_, focus_);
    if (focus_d_ > epsilon_) {
        b_continue = true;
        glm::vec3 dV = current_focus_ - focus_;
        focus_acceleration = (-2.0f / tau_) * focus_velocity - dV / (tau_ * tau_);
        focus_velocity += focus_acceleration * dt_sec;
        current_focus_ += focus_velocity * dt_sec;
    } else {
        focus_velocity = focus_acceleration = glm::vec3(0, 0, 0);
    }

    static glm::vec3 target_velocity(0.0f, 0.0f, 0.0f);
    static glm::vec3 target_acceleration(0.0f, 0.0f, 0.0f);
    object_d_ = glm::distance(current_object_, object_);
    if (object_d_ > epsilon_) {
        b_continue = true;
        glm::vec3 dV = current_object_ - object_;
        target_acceleration = (-2.0f / tau_) * target_velocity - dV / (tau_ * tau_);
        
        target_velocity += target_acceleration * dt_sec;
        current_object_ += target_velocity * dt_sec;
    } else {
        // Resets the velocity and acceleration.
        target_velocity = target_acceleration = glm::vec3(0, 0, 0);
    }

    // Updates view matrix.
    view_matrix_ = glm::lookAt(current_eye_, current_focus_, glm::vec3(0, 1, 0));
    return b_continue;
}

void InertiaCamera::SetLookAt(glm::vec3 eye, glm::vec3 target, bool reset) {
    eye_ = eye;
    focus_ = target;
    if (reset) {
        current_eye_ = eye;
        current_focus_ = target;

        view_matrix_ = glm::lookAt(current_eye_, current_focus_, glm::vec3(0, 1, 0));
    }
}

std::string InertiaCamera::to_string() const {
    thread_local char buffer[128];
    memset(buffer, 0, sizeof(buffer));
    sprintf_s(buffer, "[%5.2f, %5.2f, %5.2f] => [%5.2f, %5.2f, %5.2f]", eye_.x, eye_.y, eye_.z,
            focus_.x, focus_.y, focus_.z);
    
    return buffer;
}

}  // namespace vkpbr