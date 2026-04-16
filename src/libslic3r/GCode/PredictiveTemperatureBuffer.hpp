///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_PredictiveTemperatureBuffer_hpp_
#define slic3r_PredictiveTemperatureBuffer_hpp_

#include <string>
#include <cstddef>
#include <memory>

#include "libslic3r/libslic3r.h"
#include "libslic3r/Point.hpp"

namespace Slic3r {

class GCodeGenerator;
class PrintConfig;

// Post-processing filter inserted after the CoolingBuffer in the G-code generation
// pipeline.  For every layer it analyses the finalized G-code (with feedrates settled
// by the cooling buffer) and inserts M104 commands so that the hotend reaches the
// correct temperature for critical features (external perimeters, top solid infill).
//
// Temperature model:  T_target = T_base + k_flow * volumetric_flow
// (no per-feature offsets, purely flow-based).
//
// The buffer accumulates multiple layers (enough to cover the thermal look-ahead
// window tau = max(tau_heat, tau_cool)) and schedules M104 commands across layer
// boundaries so that critical features always start at their correct temperature.
// Non-critical features may have their temperature adjusted to facilitate transitions.
class PredictiveTemperatureBuffer {
public:
    explicit PredictiveTemperatureBuffer(GCodeGenerator &gcodegen);
    ~PredictiveTemperatureBuffer();

    void        set_current_extruder(unsigned int extruder_id) { m_current_extruder = extruder_id; }
    void        reset(const Vec3d &position);
    std::string process_layer(std::string &&gcode, std::size_t layer_id, bool flush);
    std::string process_layer(const std::string &gcode, std::size_t layer_id, bool flush)
        { return this->process_layer(std::string(gcode), layer_id, flush); }
    // Emit all buffered layers after the pipeline finishes.
    std::string flush_pending();

private:
    PredictiveTemperatureBuffer& operator=(const PredictiveTemperatureBuffer&) = delete;

    // Pimpl — all data structures (ParsedLayer, LineInfo, deque buffer) live in Impl.
    struct Impl;
    std::unique_ptr<Impl> m_impl;

    GCodeGenerator     &m_gcodegen;
    const PrintConfig  &m_config;

    unsigned int  m_current_extruder { 0 };
    // Tracked state carried between layers.
    float         m_x { 0.f };
    float         m_y { 0.f };
    float         m_f_mm_min { 0.f };
    float         m_width  { 0.4f };
    float         m_height { 0.2f };
    // Last nozzle temperature setpoint observed/emitted (°C). -1 means unknown.
    int           m_last_emitted_temp { -1 };
    // Thermal ramp model state carried between layers for preview accuracy.
    float         m_ramp_from_T  { 0.f };  // temperature when last M104 was issued
    float         m_ramp_target_T { 0.f }; // target of last M104
    float         m_ramp_elapsed { 0.f };  // time elapsed since last M104 at layer boundary
};

} // namespace Slic3r

#endif
