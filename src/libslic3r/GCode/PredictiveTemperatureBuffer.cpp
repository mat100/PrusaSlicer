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

// Find the first line with start_time >= time. Clamps to last line if past end.
static std::size_t find_line_at_time(const std::vector<LineInfo> &lines, float time)
{
    auto it = std::lower_bound(lines.begin(), lines.end(), time,
        [](const LineInfo &li, float t) { return li.start_time < t; });
    if (it == lines.end() && ! lines.empty())
        return lines.size() - 1;
    return std::size_t(it - lines.begin());
}

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
            else if (content.rfind(tag_width, 0) == 0) {
                char *we = nullptr;
                float wv = std::strtof(content.data() + tag_width.size(), &we);
                if (we != content.data() + tag_width.size()) width = wv;
            } else if (content.rfind(tag_height, 0) == 0) {
                char *he = nullptr;
                float hv = std::strtof(content.data() + tag_height.size(), &he);
                if (he != content.data() + tag_height.size()) height = hv;
            }
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
                float arc_i = 0.f, arc_j = 0.f;
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
                        case 'I': case 'i': arc_i = v; break;
                        case 'J': case 'j': arc_j = v; break;
                        default: break;
                    }
                }
                float length;
                if ((gnum == 2 || gnum == 3) && (arc_i != 0.f || arc_j != 0.f)) {
                    const float cx = x + arc_i, cy = y + arc_j;
                    const float radius = std::sqrt(arc_i * arc_i + arc_j * arc_j);
                    float sweep = std::atan2(ny - cy, nx - cx) - std::atan2(y - cy, x - cx);
                    if (gnum == 2) { // CW
                        if (sweep >= 0.f) sweep -= 2.f * float(M_PI);
                    } else {          // CCW
                        if (sweep <= 0.f) sweep += 2.f * float(M_PI);
                    }
                    length = radius * std::abs(sweep);
                } else {
                    const float dx = nx - x, dy = ny - y;
                    length = std::sqrt(dx * dx + dy * dy);
                }
                const float f_eff = (new_f > 0.f) ? new_f : f_mm_min;
                const float v_mm_s = f_eff / 60.f;
                const float dt = segment_time_sec(length, v_mm_s, accel);
                li.duration = dt;
                t_accum += dt;

                if (has_e && length > 0.f && width > 0.f && height > 0.f
                    && role != GCodeExtrusionRole::OverhangPerimeter
                    && role != GCodeExtrusionRole::BridgeInfill) {
                    // Use commanded feedrate (already adjusted by CoolingBuffer).
                    // Skip overhang/bridge segments — CoolingBuffer reduces their feedrate
                    // which would cause the flow model to compute a lower temperature,
                    // but short overhang dips destabilise the PID controller.
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
                    if (axis == 'S' || axis == 's') {
                        // Only track M109 (non-suppressed). M104 will be suppressed
                        // and replaced by the predictive algorithm, so don't let it
                        // overwrite last_emitted_temp which tracks the actual output.
                        if (mnum == 109)
                            last_emitted_temp = int(std::lround(v));
                    }
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

    // Step 1: Collect all critical events across the entire buffer as a global timeline.
    struct CriticalEvent {
        float global_time;
        int   required_T;
    };
    std::vector<CriticalEvent> critical_events;

    float cumulative_time = 0.f;
    for (const auto &layer : buffer) {
        for (const auto &line : layer.lines) {
            if (line.is_critical && line.target_T >= 0)
                critical_events.push_back({ cumulative_time + line.start_time, line.target_T });
        }
        cumulative_time += layer.total_time;
    }

    // Step 2: Schedule flow-based M104 for non-critical segments.
    struct Insert { std::size_t line_idx; int temp; };
    std::vector<Insert> inserts;

    {
        int sp = last_emitted_temp;
        for (std::size_t i = 0; i < front.lines.size(); ++i) {
            const auto &line = front.lines[i];
            if (line.target_T < 0 || line.is_critical)
                continue;

            // Check if there's a critical event within tau_max seconds (binary search).
            const float line_time = line.start_time;
            auto ce_it = std::lower_bound(critical_events.begin(), critical_events.end(), line_time,
                [](const CriticalEvent &e, float t) { return e.global_time < t; });
            if (ce_it != critical_events.end() && (ce_it->global_time - line_time) <= tau_max)
                continue; // Critical event dominates, skip natural temp.

            if (sp >= 0 && std::abs(line.target_T - sp) < hysteresis)
                continue;

            const float tau = compute_tau(sp, line.target_T, heat_speed, cool_speed);
            const std::size_t j = find_line_at_time(front.lines, std::max(0.f, line.start_time - tau));

            // If an insert already exists at this line, replace it with the
            // later (more urgent) target — its segment comes sooner in time.
            bool replaced = false;
            for (auto &ins : inserts) {
                if (ins.line_idx == j) { ins.temp = line.target_T; replaced = true; break; }
            }
            if (! replaced)
                inserts.push_back({ j, line.target_T });
            sp = line.target_T;
        }
    }

    // Step 3: Schedule M104 for critical events, accounting for flow-based and other inserts.
    {
        const auto sort_inserts = [](std::vector<Insert> &v) {
            std::sort(v.begin(), v.end(),
                      [](const Insert &a, const Insert &b) { return a.line_idx < b.line_idx; });
        };
        sort_inserts(inserts);

        // Iteratively process critical events. Multiple passes needed because a
        // critical insert (e.g. M104 S211) can change the preceding setpoint for
        // later events (e.g. S240 that was skipped when preceding_sp = last_emitted).
        for (int pass = 0; pass < 3; ++pass) {
            sort_inserts(inserts);
            const std::size_t inserts_before = inserts.size();

            for (const auto &crit : critical_events) {
                if (crit.global_time > front_total_time)
                    continue;

                const std::size_t j = find_line_at_time(front.lines, crit.global_time);

                // Find preceding setpoint from ALL sorted inserts.
                int preceding_sp = last_emitted_temp;
                for (std::size_t k = 0; k < inserts_before; ++k) {
                    if (inserts[k].line_idx <= j)
                        preceding_sp = inserts[k].temp;
                    else
                        break;
                }

                if (preceding_sp >= 0 && std::abs(crit.required_T - preceding_sp) < hysteresis)
                    continue;

                const float tau = compute_tau(preceding_sp, crit.required_T, heat_speed, cool_speed);
                inserts.push_back({ find_line_at_time(front.lines, std::max(0.f, crit.global_time - tau)),
                                    crit.required_T });
            }

            if (inserts.size() == inserts_before)
                break;
        }

        // Pass 2: Process critical events in future layers that need preheat in the front layer.
        sort_inserts(inserts);
        // Find the actual last setpoint from all inserts (highest line_idx = last in sorted order).
        int end_sp = last_emitted_temp;
        if (! inserts.empty())
            end_sp = inserts.back().temp;

        for (const auto &crit : critical_events) {
            if (crit.global_time <= front_total_time)
                continue;

            if (end_sp >= 0 && std::abs(crit.required_T - end_sp) < hysteresis)
                continue;

            const float tau = compute_tau(end_sp, crit.required_T, heat_speed, cool_speed);
            const float insertion_time = crit.global_time - tau;
            if (insertion_time > front_total_time)
                continue;

            inserts.push_back({ find_line_at_time(front.lines, std::max(0.f, insertion_time)),
                                crit.required_T });
            // Update end_sp so subsequent future events see the correct preceding setpoint.
            end_sp = crit.required_T;
        }

        // Final sort + dedup for output assembly.
        sort_inserts(inserts);
        if (inserts.size() > 1) {
            auto out = inserts.begin();
            for (auto it = inserts.begin() + 1; it != inserts.end(); ++it) {
                if (it->line_idx == out->line_idx)
                    out->temp = it->temp;
                else
                    *(++out) = *it;
            }
            inserts.erase(out + 1, inserts.end());
        }

        // Remove redundant M104 where temp matches the preceding effective setpoint.
        // Inserts are sorted by line_idx; two consecutive inserts with the same temp
        // are redundant because the firmware is already at that setpoint.
        {
            int prev_temp = last_emitted_temp;
            auto out = inserts.begin();
            for (auto it = inserts.begin(); it != inserts.end(); ++it) {
                if (it->temp != prev_temp) {
                    *out++ = *it;
                    prev_temp = it->temp;
                }
            }
            inserts.erase(out, inserts.end());
        }
    }

    // Step 4: Assemble output — insert M104 commands at scheduled positions.
    // The thermal ramp model for preview is applied later by GCodeProcessor.
    std::string out;
    out.reserve(front.gcode.size() + inserts.size() * 48u);

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
        // Don't suppress M104 here — toolchange temperature commands must be preserved.
        for (const auto &line : new_layer.lines)
            result.append(line.text.data(), line.text.size());
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
