#ifndef NVCOPY_CAMERA_H_
#define NVCOPY_CAMERA_H_

#include <array>
#include <string>
#include <glm/glm.hpp>

#include "types.h"

namespace vkpbr {

class CameraNavigator
{
  public:
    CameraNavigator() { Update(); }
    // clang-format off
    enum class Modes {
        kExamine, kFly, kWalk, kTrackball
    };
    enum class Actions {
        kNoAction, kOrbit, kDolly, kPan, kLookAround
    };
    struct Inputs {
        bool left_mouse   = false;
        bool middle_mouse = false;
        bool right_mouse  = false;
        bool shift_key    = false;
        bool ctrl_key     = false;
        bool alt_key      = false;
    };
    
    struct Camera {
        glm::vec3 eye{10.f, 10.f, 10.f};
        glm::vec3 center{0.f, 0.f, 0.f};
        glm::vec3 up{0.f, 1.0f, 0.f};
    };
    // clang-format on

    // On application mouse move, call this function with the current mouse position, mouse button
    // presses and keyboard modifier. The camera matrix will be updated.


    /**
     * Maps the given pressed mouse buttons and pressed keys on the keyboard to an action, and then
     * calls the corresponding action function. If no mouse buttons are pressed, simply calls
     * SetMousePosition(x, y) and no action is performed.
     * 
     * Current implementation: 
     * middle mouse drag => pan, 
     * right mouse drag => dolly, 
     * left mouse + ctrl => pan, 
     * left mouse + shift => Dolly, 
     * left mouse + alt => orbit or lookaround if mode is examine
     * 
     * \param x current mouse position.
     * \param y current mouse position.
     * \param inputs: A collection of states whether certain keys or mouse buttons are pressed.
     * \return 
     */
    Actions MouseMove(int x, int y, const Inputs &inputs);

    // Sets the camera to look at the target point.
    void SetLookAt(const glm::vec3 &eye, const glm::vec3 &target, const glm::vec3 &up,
                   bool animated = false);

    void UpdateAnimation();

    // Changes the size of the window.
    void SetWindowSize(int width, int height) {
        width_  = width;
        height_ = height;
    }

    Camera GetLookAt() const { return current_; }

    Modes           Mode() const { return mode_; }
    f32             Roll() const { return roll_; }
    f32             Speed() const { return speed_; }
    const glm::mat4 ViewMatrix() const { return view_matrix_; }
    glm::vec2       MousePosition() const { return mouse_position_; }
    u32             Width() const { return width_; }
    u32             Height() const { return height_; }
    float           Fov() const { return fov_; }
    double          Duration() const { return duration_; }


    void SetMode(Modes mode) { mode_ = mode; }
    void SetRoll(float roll) { roll_ = roll; }
    void SetSpeed(f32 speed) { speed_ = speed; }
    void SetMatrix(glm::mat4 matrix);
    // Sets the current mouse position, to call on mouse button down.
    void SetMousePosition(int x, int y) { mouse_position_ = {cast_f32(x), cast_f32(y)}; }
    void SetFov(float fov_degrees) { fov_ = fov_degrees; }
    void SetDuration(double duration) { duration_ = duration; }

    void Motion(int x, int y, Actions action = Actions::kOrbit);
    void Wheel(int value, const Inputs &input);

    void Fit(const glm::vec3 &box_min, const glm::vec3 box_max, bool animated = false);

  protected:
    void Update();  // Updates the internal matrix.

    // The following functions changes the current camera positions and/or orientation, but doesn't
    // update the view matrix. If needed, Update() should be called to recompute the view matrix.

    void Pan(float dx, float dy);  // Moves the camera parallel to the screen.
    void Orbit(float dx, float dy, bool invert = false);  // Rotates around the center of interest.
    void Dolly(float dx, float dy);                       // Moves toward a target.
    void Trackball(int x, int y);                         // Moves like the object is inside a ball.

    double ProjectOntoTrackballSphere(const glm::vec2 &p) const;

    // Returns the system time in milliseconds, precise to 1 microsecond, e.g., 34839.897 ms.
    double GetSystemTime();


    f32       roll_ = 0.f;  // Rotation around the Z axis in radians.
    f32       fov_  = 60.0f;
    glm::mat4 view_matrix_{1.0f};

    Camera current_;   // Current camera position.
    Camera goal_;      // Targeted camera position.
    Camera snapshot_;  // Current camera the moment a set lookat is done.

    std::array<glm::vec3, 3> bezier_;
    f64                      start_time_ = 0.0;
    f64                      duration_   = 0.5;

    u32 width_  = 1;
    u32 height_ = 1;

    f32       speed_ = 30;
    glm::vec2 mouse_position_{0.f, 0.f};

    bool button_pressed_ = false;
    bool mouse_moving_   = false;
    f32  trackball_size_ = 0.8f;

    Modes mode_ = Modes::kExamine;
};  // class CameraNavigator

class InertiaCamera
{
    glm::vec3 current_eye_, current_focus_, current_object_;
    glm::vec3 eye_, focus_, object_;

    float     tau_ = 0.2f;
    float     epsilon_ = 0.001f;
    float     eye_d_   = 0.0f;
    float     focus_d_ = 0.0f;
    float     object_d_ = 0.0f;
    glm::mat4 view_matrix_ = glm::mat4(1.0f);

  public:
    InertiaCamera(const glm::vec3 eye    = glm::vec3(0.0f, 1.0f, -3.0f),
                  const glm::vec3 focus  = glm::vec3(0, 0, 0),
                  const glm::vec3 object = glm::vec3(0, 0, 0));
    void SetTau(float tau) { tau_ = tau; }
    void RotateH(float speed, bool panning = false);
    void RotateV(float speed, bool panning = false);

    void Move(float speed, bool panning = false);

    bool Update(float dt_sec);

    void SetLookAt(glm::vec3 eye, glm::vec3 target, bool reset = false);

    std::string to_string() const;

    static constexpr glm::vec3 kYBase{0.0f, 1.0f, 0.0f};
};

}  // namespace vkpbr

#endif  // !NVCOPY_CAMERA_H_
