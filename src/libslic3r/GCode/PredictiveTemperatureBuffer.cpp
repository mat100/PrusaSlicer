///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "PredictiveTemperatureBuffer.hpp"

#include "libslic3r/GCode.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/ExtrusionRole.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <string_view>
#include <vector>

namespace Slic3r {

namespace {

// Parse a float suffix after a single-letter axis key (e.g. "X123.4") starting at `p`.
// On success advances `p` past the number and writes the value into `out`; returns true.
static bool parse_axis(const char *&p, float &out)
{
    char *end = nullptr;
    const float v = std::strtof(p, &end);
    if (end == p)
        return false;
    out = v;
    p = end;
    return true;
}

// Conservative trapezoidal-profile segment time estimate.
static float segment_time_sec(float length_mm, float v_mm_s, float accel_mm_s2)
{
    if (length_mm <= 0.f || v_mm_s <= 0.f)
        return 0.f;
    if (accel_mm_s2 <= 0.f)
        return length_mm / v_mm_s;
    const float L_reach = (v_mm_s * v_mm_s) / accel_mm_s2;
    if (length_mm <= L_reach)
        return 2.f * std::sqrt(length_mm / accel_mm_s2);
    return (length_mm - L_reach) / v_mm_s + 2.f * v_mm_s / accel_mm_s2;
}

struct LineInfo {
    std::string_view    text;               // view into owning layer's gcode string
    float               start_time { 0.f }; // cumulative time from start of this layer
    float               duration   { 0.f }; // time this segment takes
    int                 target_T   { -1 };  // T_base + k_flow * flow, clamped. -1 = non-extrusion
    bool                is_critical{ false };// ExternalPerimeter or TopSolidInfill
    bool                suppress   { false };// existing M104 to suppress
};

struct ParsedLayer {
    std::string              gcode;      // owns the string data
    std::vector<LineInfo>    lines;
    float                    total_time { 0.f };
    std::size_t              layer_id   { 0 };
    bool                     has_toolchange { false };
};

} // anonymous namespace

struct PredictiveTemperatureBuffer::Impl {
    std::deque<ParsedLayer>  buffer;
    float                    buffer_time { 0.f }; // total accumulated time across all buffered layers
};

PredictiveTemperatureBuffer::PredictiveTemperatureBuffer(GCodeGenerator &gcodegen)
    : m_impl(std::make_unique<Impl>())
    , m_gcodegen(gcodegen)
    , m_config(gcodegen.config())
{}

PredictiveTemperatureBuffer::~PredictiveTemperatureBuffer() = default;

void PredictiveTemperatureBuffer::reset(const Vec3d &position)
{
    m_x = float(position.x());
    m_y = float(position.y());
    m_f_mm_min = 0.f;
    m_last_emitted_temp = -1;
    m_impl->buffer.clear();
    m_impl->buffer_time = 0.f;
}

// ---------------------------------------------------------------------------
// Parsing
// ---------------------------------------------------------------------------

static ParsedLayer parse_layer(
    std::string &&gcode, std::size_t layer_id,
    const PrintConfig &config, unsigned int extruder_id,
    float &x, float &y, float &f_mm_min, float &width, float &height,
    int &last_emitted_temp, unsigned int &current_extruder)
{
    ParsedLayer layer;
    layer.gcode    = std::move(gcode);
    layer.layer_id = layer_id;

    const float k_flow     = float(config.filament_heat_transfer_coeff.get_at(extruder_id));
    const int   t_clamp_lo = config.filament_temp_clamp_min.get_at(extruder_id);
    const int   t_clamp_hi = config.filament_temp_clamp_max.get_at(extruder_id);
    const int   t_base     = config.temperature.get_at(extruder_id);
    const float accel      = std::max(float(config.default_acceleration.value), 100.f);

    const std::string &tag_type   = GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Role);
    const std::string &tag_width  = GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Width);
    const std::string &tag_height = GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Height);

    GCodeExtrusionRole role = GCodeExtrusionRole::None;
    float t_accum = 0.f;

    const char *const begin = layer.gcode.data();
    const char *const end   = begin + layer.gcode.size();
    const char *p = begin;

    layer.lines.reserve(layer.gcode.size() / 24u + 16u);

    while (p < end) {
        const char *line_begin = p;
        const char *line_end = static_cast<const char*>(std::memchr(p, '\n', end - p));
        if (line_end == nullptr)
            line_end = end;
        else
            ++line_end; // include '\n'

        std::string_view line(line_begin, line_end - line_begin);
        p = line_end;

        LineInfo li;
        li.text       = line;
        li.start_time = t_accum;

        const char *c = line_begin;
        while (c < line_end && (*c == ' ' || *c == '\t')) ++c;
        if (c >= line_end) { layer.lines.push_back(li); continue; }

        if (*c == ';') {
            std::string_view content(c + 1, line_end - (c + 1));
            while (!content.empty() && (content.back() == '\n' || content.back() == '\r'))
                content.remove_suffix(1);
            if (content.rfind(tag_type, 0) == 0)
                role = string_to_gcode_extrusion_role(content.substr(tag_type.size()));
            else if (content.rfind(tag_width, 0) == 0)
                try { width = std::stof(std::string(content.substr(tag_width.size()))); } catch (...) {}
            else if (content.rfind(tag_height, 0) == 0)
                try { height = std::stof(std::string(content.substr(tag_height.size()))); } catch (...) {}
            layer.lines.push_back(li);
            continue;
        }

        // Toolchange T<n>
        if ((*c == 'T' || *c == 't') && c + 1 < line_end && std::isdigit(*(c + 1))) {
            const char *q = c + 1;
            char *qend = nullptr;
            long tnum = std::strtol(q, &qend, 10);
            if (qend != q && tnum >= 0) {
                current_extruder = static_cast<unsigned int>(tnum);
                layer.has_toolchange = true;
                layer.lines.push_back(li);
                // Append remainder verbatim.
                if (p < end)
                    layer.lines.push_back(LineInfo{ std::string_view(p, end - p), t_accum, 0.f, -1, false, false });
                p = end;
                break;
            }
        }

        // G0/G1/G2/G3 moves
        if ((*c == 'G' || *c == 'g') && c + 1 < line_end) {
            const char *q = c + 1;
            char *qend = nullptr;
            long gnum = std::strtol(q, &qend, 10);
            if (qend != q && (gnum == 0 || gnum == 1 || gnum == 2 || gnum == 3)) {
                float nx = x, ny = y;
                bool has_e = false;
                float new_f = f_mm_min;
                const char *r = qend;
                while (r < line_end) {
                    while (r < line_end && (*r == ' ' || *r == '\t')) ++r;
                    if (r >= line_end || *r == ';' || *r == '\n' || *r == '\r') break;
                    char axis = *r++;
                    float v = 0.f;
                    if (! parse_axis(r, v)) break;
                    switch (axis) {
                        case 'X': case 'x': nx = v; break;
                        case 'Y': case 'y': ny = v; break;
                        case 'E': case 'e': has_e = (v > 0.f); break;
                        case 'F': case 'f': new_f = v; break;
                        default: break;
                    }
                }
                const float dx = nx - x;
                const float dy = ny - y;
                const float length = std::sqrt(dx * dx + dy * dy);
                const float f_eff = (new_f > 0.f) ? new_f : f_mm_min;
                const float v_mm_s = f_eff / 60.f;
                const float dt = segment_time_sec(length, v_mm_s, accel);
                li.duration = dt;
                t_accum += dt;

                if (has_e && length > 0.f && width > 0.f && height > 0.f) {
                    // Use commanded feedrate (already adjusted by CoolingBuffer).
                    const float flow = width * height * v_mm_s;
                    float t = float(t_base) + k_flow * flow;
                    int ti = std::clamp(int(std::lround(t)), t_clamp_lo, t_clamp_hi);
                    li.target_T = ti;
                    li.is_critical = (role == GCodeExtrusionRole::ExternalPerimeter
                                   || role == GCodeExtrusionRole::TopSolidInfill);
                }
                x = nx; y = ny; f_mm_min = new_f;
                layer.lines.push_back(li);
                continue;
            }
        }

        // M104/M109
        if ((*c == 'M' || *c == 'm') && c + 1 < line_end) {
            const char *q = c + 1;
            char *qend = nullptr;
            long mnum = std::strtol(q, &qend, 10);
            if (qend != q && (mnum == 104 || mnum == 109)) {
                const char *r = qend;
                while (r < line_end) {
                    while (r < line_end && (*r == ' ' || *r == '\t')) ++r;
                    if (r >= line_end || *r == ';' || *r == '\n' || *r == '\r') break;
                    char axis = *r++;
                    float v = 0.f;
                    if (! parse_axis(r, v)) break;
                    if (axis == 'S' || axis == 's')
                        last_emitted_temp = int(std::lround(v));
                }
                // Suppress M104 — the predictive algorithm replaces them.
                // Keep M109 (wait) as they come from toolchange or start gcode.
                if (mnum == 104)
                    li.suppress = true;
            }
        }

        layer.lines.push_back(li);
    }

    layer.total_time = t_accum;
    return layer;
}

// ---------------------------------------------------------------------------
// Scheduling & Output
// ---------------------------------------------------------------------------

// Schedule M104 insertions for the front layer, considering all buffered layers.
// Returns the assembled G-code for the front layer with M104 commands inserted.
// Compute tau (look-ahead time in seconds) from temperature delta and speed.
static float compute_tau(int from_T, int to_T, float heat_speed, float cool_speed)
{
    if (from_T < 0)
        return 0.f; // unknown setpoint, emit immediately
    const float delta = float(std::abs(to_T - from_T));
    if (delta < 1.f)
        return 0.f;
    const float speed = (to_T > from_T) ? heat_speed : cool_speed;
    return (speed > 0.f) ? (delta / speed) : 0.f;
}

static std::string schedule_and_emit(
    std::deque<ParsedLayer> &buffer,
    int &last_emitted_temp,
    float heat_speed, float cool_speed, float hysteresis,
    float tau_max)
{
    if (buffer.empty())
        return {};

    ParsedLayer &front = buffer.front();
    const float front_total_time = front.total_time;

    // Step 1: Collect critical events across the entire buffer as a global timeline.
    // Consecutive critical segments at the same temperature are merged into a single
    // event with accumulated dwell time, so that the reachability check in Step 2
    // can judge whether the nozzle has enough time to reach the target.
    struct CriticalEvent {
        float global_time;
        int   required_T;
        float dwell;        // total duration of consecutive critical segments at this temp
    };
    std::vector<CriticalEvent> critical_events;

    float cumulative_time = 0.f;
    for (const auto &layer : buffer) {
        for (const auto &line : layer.lines) {
            if (line.is_critical && line.target_T >= 0) {
                const float gt = cumulative_time + line.start_time;
                // Merge into the previous event if the temperature is the same (within hysteresis).
                if (! critical_events.empty()
                    && std::abs(critical_events.back().required_T - line.target_T) < hysteresis)
                {
                    critical_events.back().dwell += line.duration;
                } else {
                    critical_events.push_back({ gt, line.target_T, line.duration });
                }
            }
        }
        cumulative_time += layer.total_time;
    }

    // Step 2: Schedule M104 insertions for the front layer.
    struct Insert { std::size_t line_idx; int temp; };
    std::vector<Insert> inserts;

    int running_setpoint = last_emitted_temp;

    // Process critical events in chronological order.
    for (std::size_t ci = 0; ci < critical_events.size(); ++ci) {
        const auto &crit = critical_events[ci];
        if (running_setpoint >= 0 && std::abs(crit.required_T - running_setpoint) < hysteresis)
            continue;

        // Reachability check: compute the effective dwell — the time this setpoint
        // would remain active before the next *different* critical event replaces it.
        // Using just crit.dwell (the critical segment's own duration) is not enough:
        // a 5-second ExternalPerimeter at 217°C passes dwell check, but if the next
        // critical event at 221°C arrives 5.6s later the nozzle barely reaches the
        // target before being told to go back, causing PID oscillation.
        if (running_setpoint >= 0) {
            // Time until the next critical event with a different temperature.
            float effective_dwell = crit.dwell;
            if (ci + 1 < critical_events.size()) {
                const float time_to_next = critical_events[ci + 1].global_time
                                         - (crit.global_time + crit.dwell);
                effective_dwell = crit.dwell + std::max(0.f, time_to_next);
            }
            const float required_delta = float(std::abs(crit.required_T - running_setpoint));
            const float speed = (crit.required_T > running_setpoint) ? heat_speed : cool_speed;
            // The nozzle must be able to reach the target AND hold it for a meaningful
            // period.  We require: time to reach target (tau) + tau/2 hold <= effective_dwell.
            // Simplified: tau * 1.5 <= effective_dwell, i.e. effective_dwell >= 1.5 * tau.
            const float tau = (speed > 0.f) ? (required_delta / speed) : 0.f;
            if (tau > 0.f && effective_dwell < tau * 1.5f)
                continue;
        }

        // Compute tau dynamically from temperature delta and speed.
        const float tau = compute_tau(running_setpoint, crit.required_T, heat_speed, cool_speed);
        const float insertion_time = crit.global_time - tau;

        // Only schedule if the insertion point falls within the front layer.
        if (insertion_time > front_total_time)
            continue; // This critical event is far in the future, doesn't affect front layer.

        // Find the line in the front layer at or after the insertion time.
        const float effective_time = std::max(0.f, insertion_time);
        std::size_t j = 0;
        for (; j < front.lines.size(); ++j) {
            if (front.lines[j].start_time >= effective_time)
                break;
        }
        if (j >= front.lines.size())
            j = front.lines.size() > 0 ? front.lines.size() - 1 : 0;

        // Coalesce: if the previous insert is at the same line, overwrite with the newer target.
        if (! inserts.empty() && inserts.back().line_idx == j)
            inserts.back().temp = crit.required_T;
        else
            inserts.push_back({ j, crit.required_T });

        running_setpoint = crit.required_T;
    }

    // Step 3: Between critical events, schedule natural flow-based temperatures
    // for non-critical segments when no critical feature is imminent.
    // Reset running setpoint to track what we've actually scheduled.
    //
    // Helper: compute how long the nozzle stays at approximately the same target
    // temperature starting from line i (consecutive non-critical extrusion segments).
    auto dwell_time_at_temp = [&](std::size_t i, int target_T) -> float {
        float total = 0.f;
        for (std::size_t k = i; k < front.lines.size(); ++k) {
            const auto &ln = front.lines[k];
            if (ln.target_T >= 0 && std::abs(ln.target_T - target_T) < hysteresis)
                total += ln.duration;
            else if (ln.target_T >= 0)
                break; // different temperature — end of group
            // target_T == -1 (travel, comments) don't break the group
        }
        return total;
    };

    {
        int sp = last_emitted_temp;
        // Merge insert tracking with forward scan.
        std::size_t next_insert = 0;

        for (std::size_t i = 0; i < front.lines.size(); ++i) {
            // Update sp from scheduled inserts at this position.
            while (next_insert < inserts.size() && inserts[next_insert].line_idx == i) {
                sp = inserts[next_insert].temp;
                ++next_insert;
            }

            const auto &line = front.lines[i];
            if (line.target_T < 0 || line.is_critical)
                continue;

            // Check if there's a critical event within tau_max seconds.
            const float line_global_time = line.start_time; // front layer starts at global time 0
            bool critical_nearby = false;
            for (const auto &crit : critical_events) {
                if (crit.global_time >= line_global_time && (crit.global_time - line_global_time) <= tau_max) {
                    critical_nearby = true;
                    break;
                }
            }
            if (critical_nearby)
                continue; // Critical event dominates, skip natural temp.

            // Schedule natural flow-based temp for this non-critical segment.
            if (sp >= 0 && std::abs(line.target_T - sp) < hysteresis)
                continue;

            // Reachability check: only schedule if the nozzle can reach the target
            // AND hold it for a meaningful period (tau * 1.5) before the group ends.
            {
                const float dwell = dwell_time_at_temp(i, line.target_T);
                const float tau_seg = compute_tau(sp, line.target_T, heat_speed, cool_speed);
                if (tau_seg > 0.f && dwell < tau_seg * 1.5f)
                    continue; // nozzle can't meaningfully reach this temperature
            }

            // Find insertion point: tau seconds before this line.
            const float tau = compute_tau(sp, line.target_T, heat_speed, cool_speed);

            const float ins_time = std::max(0.f, line.start_time - tau);
            std::size_t j = 0;
            for (; j < front.lines.size(); ++j) {
                if (front.lines[j].start_time >= ins_time)
                    break;
            }
            if (j >= front.lines.size())
                j = front.lines.size() > 0 ? front.lines.size() - 1 : 0;

            // Don't insert if it would conflict with an existing insert.
            bool conflicts = false;
            for (const auto &ins : inserts) {
                if (ins.line_idx == j) {
                    conflicts = true;
                    break;
                }
            }
            if (conflicts)
                continue;

            inserts.push_back({ j, line.target_T });
            sp = line.target_T;

            // Skip forward past consecutive lines at the same target temperature
            // to avoid re-evaluating each G1 within the same feature group.
            for (std::size_t k = i + 1; k < front.lines.size(); ++k) {
                if (front.lines[k].target_T >= 0
                    && std::abs(front.lines[k].target_T - line.target_T) >= hysteresis)
                    break;
                i = k; // the for-loop will increment past this
            }
        }

        // Sort inserts by line_idx for output assembly.
        std::sort(inserts.begin(), inserts.end(),
                  [](const Insert &a, const Insert &b) { return a.line_idx < b.line_idx; });
    }

    // Step 4: Assemble output.
    // Emit _PREDICTIVE_TEMP: tags before extrusion lines so that GCodeProcessor
    // can show the correct target temperature in the preview (not the M104 setpoint).
    std::string out;
    out.reserve(front.gcode.size() + inserts.size() * 48u + front.lines.size() * 24u);

    std::size_t next_ins = 0;
    for (std::size_t i = 0; i < front.lines.size(); ++i) {
        while (next_ins < inserts.size() && inserts[next_ins].line_idx == i) {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "M104 S%d ; predictive nozzle temperature\n",
                          inserts[next_ins].temp);
            out.append(buf);
            last_emitted_temp = inserts[next_ins].temp;
            ++next_ins;
        }
        // Emit current setpoint tag before extrusion lines for preview visualization.
        // Use last_emitted_temp (the actual M104 setpoint) rather than the per-segment
        // flow-based target, so the preview matches what the firmware actually sees.
        if (front.lines[i].target_T >= 0 && last_emitted_temp >= 0) {
            char buf[48];
            std::snprintf(buf, sizeof(buf), ";_PREDICTIVE_TEMP:%d\n", last_emitted_temp);
            out.append(buf);
        }
        if (! front.lines[i].suppress)
            out.append(front.lines[i].text.data(), front.lines[i].text.size());
    }
    // Any trailing inserts (past last line).
    while (next_ins < inserts.size()) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "M104 S%d ; predictive nozzle temperature\n",
                      inserts[next_ins].temp);
        out.append(buf);
        last_emitted_temp = inserts[next_ins].temp;
        ++next_ins;
    }

    return out;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

std::string PredictiveTemperatureBuffer::process_layer(std::string &&gcode, std::size_t layer_id, bool /*flush*/)
{
    if (! m_config.enable_predictive_nozzle_temperature.value)
        return std::move(gcode);

    const float heat_speed = float(m_config.nozzle_heating_speed.get_at(m_current_extruder));
    const float cool_speed = float(m_config.nozzle_cooling_speed.get_at(m_current_extruder));
    if (heat_speed <= 0.f && cool_speed <= 0.f)
        return std::move(gcode);
    const float hyst = float(m_config.predictive_temp_hysteresis.value);

    // Compute worst-case tau_max from the full temperature clamp range.
    const int t_clamp_lo = m_config.filament_temp_clamp_min.get_at(m_current_extruder);
    const int t_clamp_hi = m_config.filament_temp_clamp_max.get_at(m_current_extruder);
    const float max_delta = float(t_clamp_hi - t_clamp_lo);
    const float min_speed = std::min(
        heat_speed > 0.f ? heat_speed : cool_speed,
        cool_speed > 0.f ? cool_speed : heat_speed);
    const float tau_max = (min_speed > 0.f) ? (max_delta / min_speed) : 0.f;

    // Parse the incoming layer.
    ParsedLayer new_layer = parse_layer(
        std::move(gcode), layer_id, m_config, m_current_extruder,
        m_x, m_y, m_f_mm_min, m_width, m_height,
        m_last_emitted_temp, m_current_extruder);

    std::string result;

    // If the new layer has a toolchange, flush everything before it.
    if (new_layer.has_toolchange) {
        result = flush_pending();
        // Emit the toolchange layer as-is (no predictive scheduling across toolchange).
        for (const auto &line : new_layer.lines) {
            if (! line.suppress)
                result.append(line.text.data(), line.text.size());
        }
        return result;
    }

    // Add the new layer to the buffer.
    m_impl->buffer_time += new_layer.total_time;
    m_impl->buffer.push_back(std::move(new_layer));

    // Check if we have enough buffered time (after the front layer) to cover tau_max.
    while (m_impl->buffer.size() > 1) {
        const float time_after_front = m_impl->buffer_time - m_impl->buffer.front().total_time;
        if (time_after_front < tau_max)
            break;

        // We have enough look-ahead. Process and emit the front layer.
        std::string layer_out = schedule_and_emit(m_impl->buffer, m_last_emitted_temp, heat_speed, cool_speed, hyst, tau_max);
        result += layer_out;

        m_impl->buffer_time -= m_impl->buffer.front().total_time;
        m_impl->buffer.pop_front();
    }

    return result;
}

std::string PredictiveTemperatureBuffer::flush_pending()
{
    if (m_impl->buffer.empty())
        return {};

    const float heat_speed = float(m_config.nozzle_heating_speed.get_at(m_current_extruder));
    const float cool_speed = float(m_config.nozzle_cooling_speed.get_at(m_current_extruder));
    const float hyst       = float(m_config.predictive_temp_hysteresis.value);
    const int   t_clamp_lo = m_config.filament_temp_clamp_min.get_at(m_current_extruder);
    const int   t_clamp_hi = m_config.filament_temp_clamp_max.get_at(m_current_extruder);
    const float min_speed  = std::min(
        heat_speed > 0.f ? heat_speed : cool_speed,
        cool_speed > 0.f ? cool_speed : heat_speed);
    const float tau_max = (min_speed > 0.f) ? (float(t_clamp_hi - t_clamp_lo) / min_speed) : 0.f;

    std::string result;
    while (! m_impl->buffer.empty()) {
        std::string layer_out = schedule_and_emit(m_impl->buffer, m_last_emitted_temp, heat_speed, cool_speed, hyst, tau_max);
        result += layer_out;

        m_impl->buffer_time -= m_impl->buffer.front().total_time;
        m_impl->buffer.pop_front();
    }
    m_impl->buffer_time = 0.f;
    return result;
}

} // namespace Slic3r
