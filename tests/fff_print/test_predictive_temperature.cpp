#include <catch2/catch_test_macros.hpp>

#include <memory>
#include <string>

#include "libslic3r/Config.hpp"
#include "libslic3r/ExtrusionRole.hpp"
#include "libslic3r/GCode.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/GCode/PredictiveTemperatureBuffer.hpp"
#include "libslic3r/libslic3r.h"

using namespace Slic3r;

namespace {

std::unique_ptr<PredictiveTemperatureBuffer> make_predictive_temperature_buffer(
    GCodeGenerator           &gcodegen,
    const DynamicPrintConfig &config)
{
    PrintConfig print_config;
    print_config.apply(config, true);
    gcodegen.apply_print_config(print_config);
    gcodegen.set_layer_count(10);
    gcodegen.writer().set_extruders({ 0 });
    gcodegen.writer().set_extruder(0);

    auto buffer = std::make_unique<PredictiveTemperatureBuffer>(gcodegen);
    buffer->set_current_extruder(0);
    buffer->reset(Vec3d::Zero());
    return buffer;
}

DynamicPrintConfig predictive_temperature_config()
{
    return DynamicPrintConfig::full_print_config_with({
        { "temperature", 200 },
        { "first_layer_temperature", 200 },
        { "nozzle_heating_speed", 30 },
        { "nozzle_cooling_speed", 5 },
        { "filament_heat_transfer_coeff", 10 },
        { "filament_temp_clamp_min", 180 },
        { "filament_temp_clamp_max", 260 },
        { "predictive_temp_hysteresis", 0.5 },
        { "default_acceleration", 100 }
    });
}

std::string role_tag(GCodeExtrusionRole role)
{
    return ";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Role) +
        gcode_extrusion_role_to_string(role) + "\n";
}

const std::string width_height_tags = ";WIDTH:1\n;HEIGHT:1\n";

std::string width_height_tags_for(double width, double height)
{
    return ";WIDTH:" + std::to_string(width) + "\n;HEIGHT:" + std::to_string(height) + "\n";
}

std::string process_all(PredictiveTemperatureBuffer &buffer, std::string layer)
{
    std::string out = buffer.process_layer(std::move(layer), 0, false);
    out += buffer.flush_pending();
    return out;
}

void require_ordered(const std::string &text, const std::initializer_list<std::string> &needles)
{
    size_t pos = 0;
    for (const std::string &needle : needles) {
        const size_t found = text.find(needle, pos);
        REQUIRE(found != std::string::npos);
        pos = found + needle.size();
    }
}

} // namespace

TEST_CASE("Predictive temperature rejects flow changes that would compromise critical features", "[PredictiveTemperature]")
{
    GCodeGenerator gcodegen;
    auto buffer = make_predictive_temperature_buffer(gcodegen, predictive_temperature_config());

    const std::string gcode =
        "M109 S200\n" +
        role_tag(GCodeExtrusionRole::InternalInfill) +
        width_height_tags +
        "G1 X1 E1 F120\n" + // 220 C flow target, but too close to the critical segment.
        role_tag(GCodeExtrusionRole::ExternalPerimeter) +
        "G1 X2 E2 F60\n";   // 210 C critical target.

    const std::string out = process_all(*buffer, gcode);

    CHECK(out.find("M104 S220 ; predictive nozzle temperature") == std::string::npos);
    CHECK(out.find("M104 S210 ; predictive nozzle temperature") != std::string::npos);
}

TEST_CASE("Predictive temperature keeps non-critical flow control when critical recovery remains possible", "[PredictiveTemperature]")
{
    GCodeGenerator gcodegen;
    auto buffer = make_predictive_temperature_buffer(gcodegen, predictive_temperature_config());

    const std::string gcode =
        "M109 S200\n" +
        role_tag(GCodeExtrusionRole::InternalInfill) +
        width_height_tags +
        "G1 X20 E1 F120\n" + // 220 C flow target.
        "G1 X80 E2 F120\n" +
        role_tag(GCodeExtrusionRole::ExternalPerimeter) +
        "G1 X90 E3 F60\n";   // 210 C critical target, with enough cooling time.

    const std::string out = process_all(*buffer, gcode);

    const size_t flow_pos = out.find("M104 S220 ; predictive nozzle temperature");
    const size_t critical_pos = out.find("M104 S210 ; predictive nozzle temperature");
    REQUIRE(flow_pos != std::string::npos);
    REQUIRE(critical_pos != std::string::npos);
    CHECK(flow_pos < critical_pos);
}

TEST_CASE("Predictive temperature ignores bridge flow but may preheat across it for a critical feature", "[PredictiveTemperature]")
{
    GCodeGenerator gcodegen;
    auto buffer = make_predictive_temperature_buffer(gcodegen, predictive_temperature_config());

    const std::string external_role = role_tag(GCodeExtrusionRole::ExternalPerimeter);
    const std::string gcode =
        "M109 S200\n" +
        role_tag(GCodeExtrusionRole::BridgeInfill) +
        width_height_tags +
        "G1 X100 E1 F300\n" + // Would be 250 C by flow, but bridges are ignored.
        external_role +
        "G1 X110 E2 F60\n";   // 210 C critical target.

    const std::string out = process_all(*buffer, gcode);

    CHECK(out.find("M104 S250 ; predictive nozzle temperature") == std::string::npos);
    const size_t critical_temp = out.find("M104 S210 ; predictive nozzle temperature");
    const size_t external_tag = out.find(external_role);
    REQUIRE(critical_temp != std::string::npos);
    REQUIRE(external_tag != std::string::npos);
    CHECK(critical_temp < external_tag);
}

TEST_CASE("Predictive temperature suppresses source M104 commands", "[PredictiveTemperature]")
{
    GCodeGenerator gcodegen;
    auto buffer = make_predictive_temperature_buffer(gcodegen, predictive_temperature_config());

    const std::string gcode =
        "M109 S200\n"
        "M104 S240\n" +
        role_tag(GCodeExtrusionRole::ExternalPerimeter) +
        width_height_tags +
        "G1 X10 E1 F60\n";

    const std::string out = process_all(*buffer, gcode);

    CHECK(out.find("M104 S240") == std::string::npos);
    CHECK(out.find("M104 S210 ; predictive nozzle temperature") != std::string::npos);
}

TEST_CASE("Predictive temperature treats top solid infill as a critical feature", "[PredictiveTemperature]")
{
    GCodeGenerator gcodegen;
    auto buffer = make_predictive_temperature_buffer(gcodegen, predictive_temperature_config());

    const std::string gcode =
        "M109 S200\n" +
        role_tag(GCodeExtrusionRole::InternalInfill) +
        width_height_tags +
        "G1 X1 E1 F120\n" + // 220 C flow target, but too close to the critical segment.
        role_tag(GCodeExtrusionRole::TopSolidInfill) +
        "G1 X2 E2 F60\n";   // 210 C critical target.

    const std::string out = process_all(*buffer, gcode);

    CHECK(out.find("M104 S220 ; predictive nozzle temperature") == std::string::npos);
    CHECK(out.find("M104 S210 ; predictive nozzle temperature") != std::string::npos);
}

TEST_CASE("Predictive temperature ignores overhang perimeter flow", "[PredictiveTemperature]")
{
    GCodeGenerator gcodegen;
    auto buffer = make_predictive_temperature_buffer(gcodegen, predictive_temperature_config());

    const std::string external_role = role_tag(GCodeExtrusionRole::ExternalPerimeter);
    const std::string gcode =
        "M109 S200\n" +
        role_tag(GCodeExtrusionRole::OverhangPerimeter) +
        width_height_tags +
        "G1 X100 E1 F300\n" + // Would be 250 C by flow, but overhang perimeters are ignored.
        external_role +
        "G1 X110 E2 F60\n";

    const std::string out = process_all(*buffer, gcode);

    CHECK(out.find("M104 S250 ; predictive nozzle temperature") == std::string::npos);
    const size_t critical_temp = out.find("M104 S210 ; predictive nozzle temperature");
    const size_t external_tag = out.find(external_role);
    REQUIRE(critical_temp != std::string::npos);
    REQUIRE(external_tag != std::string::npos);
    CHECK(critical_temp < external_tag);
}

TEST_CASE("Predictive temperature can preheat for a critical feature across a layer boundary", "[PredictiveTemperature]")
{
    GCodeGenerator gcodegen;
    auto buffer = make_predictive_temperature_buffer(gcodegen, predictive_temperature_config());

    const std::string next_layer_marker = ";NEXT_LAYER_MARKER\n";
    const std::string layer0 =
        "M109 S200\n" +
        role_tag(GCodeExtrusionRole::BridgeInfill) +
        width_height_tags +
        "G1 X100 E1 F300\n" +
        next_layer_marker;
    const std::string layer1 =
        role_tag(GCodeExtrusionRole::TopSolidInfill) +
        width_height_tags +
        "G1 X110 E2 F60\n";

    std::string out = buffer->process_layer(layer0, 0, false);
    out += buffer->process_layer(layer1, 1, false);
    out += buffer->flush_pending();

    const size_t critical_temp = out.find("M104 S210 ; predictive nozzle temperature");
    const size_t next_layer = out.find(next_layer_marker);
    REQUIRE(critical_temp != std::string::npos);
    REQUIRE(next_layer != std::string::npos);
    CHECK(critical_temp < next_layer);
}

TEST_CASE("Predictive temperature clamps high flow targets to the configured maximum", "[PredictiveTemperature]")
{
    GCodeGenerator gcodegen;
    auto buffer = make_predictive_temperature_buffer(gcodegen, predictive_temperature_config());

    const std::string gcode =
        "M109 S200\n" +
        role_tag(GCodeExtrusionRole::ExternalPerimeter) +
        width_height_tags +
        "G1 X10 E1 F600\n"; // 300 C raw target, clamped to 260 C.

    const std::string out = process_all(*buffer, gcode);

    CHECK(out.find("M104 S260 ; predictive nozzle temperature") != std::string::npos);
    CHECK(out.find("M104 S300 ; predictive nozzle temperature") == std::string::npos);
}

TEST_CASE("Predictive temperature follows slow non-critical flow ramps", "[PredictiveTemperature]")
{
    GCodeGenerator gcodegen;
    auto buffer = make_predictive_temperature_buffer(gcodegen, predictive_temperature_config());

    const std::string gcode =
        "M109 S200\n" +
        role_tag(GCodeExtrusionRole::InternalInfill) +
        width_height_tags +
        "G1 X30 E1 F60\n" +  // 210 C, long enough to settle.
        "G1 X60 E2 F120\n" + // 220 C.
        "G1 X90 E3 F180\n";  // 230 C.

    const std::string out = process_all(*buffer, gcode);

    const size_t temp_210 = out.find("M104 S210 ; predictive nozzle temperature");
    const size_t temp_220 = out.find("M104 S220 ; predictive nozzle temperature");
    const size_t temp_230 = out.find("M104 S230 ; predictive nozzle temperature");
    REQUIRE(temp_210 != std::string::npos);
    REQUIRE(temp_220 != std::string::npos);
    REQUIRE(temp_230 != std::string::npos);
    CHECK(temp_210 < temp_220);
    CHECK(temp_220 < temp_230);
}

TEST_CASE("Predictive temperature allows slow recovery from high flow before a critical feature", "[PredictiveTemperature]")
{
    GCodeGenerator gcodegen;
    auto buffer = make_predictive_temperature_buffer(gcodegen, predictive_temperature_config());

    const std::string external_role = role_tag(GCodeExtrusionRole::ExternalPerimeter);
    const std::string gcode =
        "M109 S200\n" +
        role_tag(GCodeExtrusionRole::InternalInfill) +
        width_height_tags +
        "G1 X100 E1 F300\n" + // 250 C flow target with enough time to cool afterwards.
        external_role +
        "G1 X110 E2 F60\n";   // 210 C critical target.

    const std::string out = process_all(*buffer, gcode);

    const size_t high_flow = out.find("M104 S250 ; predictive nozzle temperature");
    const size_t critical = out.find("M104 S210 ; predictive nozzle temperature");
    const size_t external = out.find(external_role);
    REQUIRE(high_flow != std::string::npos);
    REQUIRE(critical != std::string::npos);
    REQUIRE(external != std::string::npos);
    CHECK(high_flow < critical);
    CHECK(critical < external);
}

TEST_CASE("Predictive temperature blocks rapid non-critical oscillation before a critical feature", "[PredictiveTemperature]")
{
    GCodeGenerator gcodegen;
    auto buffer = make_predictive_temperature_buffer(gcodegen, predictive_temperature_config());

    const std::string gcode =
        "M109 S200\n" +
        role_tag(GCodeExtrusionRole::InternalInfill) +
        width_height_tags +
        "G1 X1 E1 F300\n" +  // 250 C flow target, too close to the critical segment.
        "G1 X2 E2 F60\n" +   // 210 C flow target.
        "G1 X3 E3 F300\n" +  // Another rapid high-flow spike.
        role_tag(GCodeExtrusionRole::ExternalPerimeter) +
        "G1 X4 E4 F60\n";    // 210 C critical target.

    const std::string out = process_all(*buffer, gcode);

    CHECK(out.find("M104 S250 ; predictive nozzle temperature") == std::string::npos);
    CHECK(out.find("M104 S210 ; predictive nozzle temperature") != std::string::npos);
}

TEST_CASE("Predictive temperature gives priority to external perimeters around short infill", "[PredictiveTemperature]")
{
    GCodeGenerator gcodegen;
    auto buffer = make_predictive_temperature_buffer(gcodegen, predictive_temperature_config());

    const std::string external_role = role_tag(GCodeExtrusionRole::ExternalPerimeter);
    const std::string gcode =
        "M109 S200\n" +
        external_role +
        width_height_tags +
        "G1 X1 E1 F60\n" +    // 210 C critical target.
        role_tag(GCodeExtrusionRole::InternalInfill) +
        "G1 X2 E2 F300\n" +   // 250 C short infill target, too close to next critical.
        external_role +
        "G1 X3 E3 F60\n";     // 210 C critical target must dominate.

    const std::string out = process_all(*buffer, gcode);

    CHECK(out.find("M104 S250 ; predictive nozzle temperature") == std::string::npos);
    CHECK(out.find("M104 S210 ; predictive nozzle temperature") != std::string::npos);
}

TEST_CASE("Predictive temperature allows long infill between external perimeters", "[PredictiveTemperature]")
{
    GCodeGenerator gcodegen;
    auto buffer = make_predictive_temperature_buffer(gcodegen, predictive_temperature_config());

    const std::string external_role = role_tag(GCodeExtrusionRole::ExternalPerimeter);
    const std::string gcode =
        "M109 S200\n" +
        external_role +
        width_height_tags +
        "G1 X10 E1 F60\n" +    // 210 C critical target.
        role_tag(GCodeExtrusionRole::InternalInfill) +
        "G1 X100 E2 F300\n" +  // 250 C infill target, long enough to cool afterwards.
        external_role +
        "G1 X110 E3 F60\n";    // 210 C critical target.

    const std::string out = process_all(*buffer, gcode);

    const size_t first_external = out.find("M104 S210 ; predictive nozzle temperature");
    const size_t infill = out.find("M104 S250 ; predictive nozzle temperature");
    const size_t second_external = out.find("M104 S210 ; predictive nozzle temperature", first_external + 1);
    REQUIRE(first_external != std::string::npos);
    REQUIRE(infill != std::string::npos);
    REQUIRE(second_external != std::string::npos);
    CHECK(first_external < infill);
    CHECK(infill < second_external);
}

TEST_CASE("Predictive temperature heating tau moves critical preheat earlier when heating is slower", "[PredictiveTemperature][Tau]")
{
    const std::string first_travel = "G1 X20 F600\n";
    const std::string second_travel = "G1 X40 F600\n";
    const std::string gcode =
        "M109 S200\n" +
        first_travel +
        ";AFTER_FIRST_TRAVEL\n" +
        second_travel +
        ";AFTER_SECOND_TRAVEL\n" +
        role_tag(GCodeExtrusionRole::ExternalPerimeter) +
        width_height_tags +
        "G1 X41 E1 F180\n"; // 230 C critical target.

    {
        GCodeGenerator gcodegen;
        auto config = predictive_temperature_config();
        config.set_deserialize_strict({ { "nozzle_heating_speed", 30 } }); // 30 C delta => 1 s tau.
        auto buffer = make_predictive_temperature_buffer(gcodegen, config);
        const std::string out = process_all(*buffer, gcode);

        require_ordered(out, {
            second_travel,
            "M104 S230 ; predictive nozzle temperature\n",
            ";AFTER_SECOND_TRAVEL\n"
        });
    }

    {
        GCodeGenerator gcodegen;
        auto config = predictive_temperature_config();
        config.set_deserialize_strict({ { "nozzle_heating_speed", 10 } }); // 30 C delta => 3 s tau.
        auto buffer = make_predictive_temperature_buffer(gcodegen, config);
        const std::string out = process_all(*buffer, gcode);

        require_ordered(out, {
            first_travel,
            "M104 S230 ; predictive nozzle temperature\n",
            ";AFTER_FIRST_TRAVEL\n",
            second_travel
        });
    }
}

TEST_CASE("Predictive temperature cooling tau controls whether long infill may be used before an external perimeter", "[PredictiveTemperature][Tau]")
{
    const std::string infill_move = "G1 X80 E2 F300\n";
    const std::string recovery_travel = "G1 X90 F600\n";
    const std::string external_role = role_tag(GCodeExtrusionRole::ExternalPerimeter);
    const std::string gcode =
        "M109 S200\n" +
        external_role +
        width_height_tags +
        "G1 X10 E1 F60\n" +    // 210 C critical target.
        role_tag(GCodeExtrusionRole::InternalInfill) +
        infill_move +           // 250 C infill target.
        ";AFTER_INFILL\n" +
        recovery_travel +       // Time available for cooling before the next critical segment.
        ";AFTER_RECOVERY_TRAVEL\n" +
        external_role +
        "G1 X100 E3 F60\n";     // 210 C critical target.

    {
        GCodeGenerator gcodegen;
        auto config = predictive_temperature_config();
        config.set_deserialize_strict({ { "nozzle_cooling_speed", 5 } }); // 40 C delta => 8 s tau.
        auto buffer = make_predictive_temperature_buffer(gcodegen, config);
        const std::string out = process_all(*buffer, gcode);

        require_ordered(out, {
            "M104 S250 ; predictive nozzle temperature\n",
            "M104 S210 ; predictive nozzle temperature\n",
            ";AFTER_INFILL\n",
            recovery_travel,
            ";AFTER_RECOVERY_TRAVEL\n"
        });
    }

    {
        GCodeGenerator gcodegen;
        auto config = predictive_temperature_config();
        config.set_deserialize_strict({ { "nozzle_cooling_speed", 2 } }); // 40 C delta => 20 s tau, too slow.
        auto buffer = make_predictive_temperature_buffer(gcodegen, config);
        const std::string out = process_all(*buffer, gcode);

        CHECK(out.find("M104 S250 ; predictive nozzle temperature") == std::string::npos);
        CHECK(out.find("M104 S210 ; predictive nozzle temperature") != std::string::npos);
    }
}

TEST_CASE("Predictive temperature handles bridge to infill to critical perimeter sequence", "[PredictiveTemperature]")
{
    GCodeGenerator gcodegen;
    auto buffer = make_predictive_temperature_buffer(gcodegen, predictive_temperature_config());

    const std::string external_role = role_tag(GCodeExtrusionRole::ExternalPerimeter);
    const std::string gcode =
        "M109 S200\n" +
        role_tag(GCodeExtrusionRole::BridgeInfill) +
        width_height_tags +
        "G1 X20 E1 F300\n" + // Bridge flow ignored.
        role_tag(GCodeExtrusionRole::InternalInfill) +
        "G1 X100 E2 F120\n" + // 220 C flow target, enough time before external perimeter.
        external_role +
        "G1 X110 E3 F60\n";   // 210 C critical target.

    const std::string out = process_all(*buffer, gcode);

    CHECK(out.find("M104 S250 ; predictive nozzle temperature") == std::string::npos);
    const size_t infill = out.find("M104 S220 ; predictive nozzle temperature");
    const size_t critical = out.find("M104 S210 ; predictive nozzle temperature");
    const size_t external = out.find(external_role);
    REQUIRE(infill != std::string::npos);
    REQUIRE(critical != std::string::npos);
    REQUIRE(external != std::string::npos);
    CHECK(infill < critical);
    CHECK(critical < external);
}

TEST_CASE("Predictive temperature schedules cross-layer cooling from non-critical flow to critical top solid", "[PredictiveTemperature]")
{
    GCodeGenerator gcodegen;
    auto buffer = make_predictive_temperature_buffer(gcodegen, predictive_temperature_config());

    const std::string next_layer_marker = ";NEXT_LAYER_COOLING_MARKER\n";
    const std::string layer0 =
        "M109 S200\n" +
        role_tag(GCodeExtrusionRole::InternalInfill) +
        width_height_tags +
        "G1 X100 E1 F300\n" + // 250 C flow target.
        next_layer_marker;
    const std::string layer1 =
        role_tag(GCodeExtrusionRole::TopSolidInfill) +
        width_height_tags +
        "G1 X110 E2 F60\n";   // 210 C critical target.

    std::string out = buffer->process_layer(layer0, 0, false);
    out += buffer->process_layer(layer1, 1, false);
    out += buffer->flush_pending();

    const size_t high_flow = out.find("M104 S250 ; predictive nozzle temperature");
    const size_t critical = out.find("M104 S210 ; predictive nozzle temperature");
    const size_t next_layer = out.find(next_layer_marker);
    REQUIRE(high_flow != std::string::npos);
    REQUIRE(critical != std::string::npos);
    REQUIRE(next_layer != std::string::npos);
    CHECK(high_flow < critical);
    CHECK(critical < next_layer);
}

TEST_CASE("Predictive temperature handles Benchy first-layer external perimeter width changes", "[PredictiveTemperature][BenchyDerived]")
{
    GCodeGenerator gcodegen;
    auto config = predictive_temperature_config();
    config.set_deserialize_strict({ { "predictive_temp_hysteresis", 3.0 } });
    auto buffer = make_predictive_temperature_buffer(gcodegen, config);

    const std::string gcode =
        "M109 S200\n" +
        role_tag(GCodeExtrusionRole::ExternalPerimeter) +
        width_height_tags_for(0.499999, 0.2) +
        "G1 F2700\n"
        "G1 X10 E1\n" +       // Benchy first layer: around 245 C.
        "M104 S213\n" +       // Existing commands from source G-code are suppressed.
        width_height_tags_for(0.474053, 0.2) +
        "G1 X20 E2\n" +       // Around 243 C, within hysteresis.
        width_height_tags_for(0.522282, 0.2) +
        "G1 X30 E3\n";        // Around 247 C, within hysteresis.

    const std::string out = process_all(*buffer, gcode);

    CHECK(out.find("M104 S213") == std::string::npos);
    CHECK(out.find("M104 S245 ; predictive nozzle temperature") != std::string::npos);
    CHECK(out.find("M104 S243 ; predictive nozzle temperature") == std::string::npos);
    CHECK(out.find("M104 S247 ; predictive nozzle temperature") == std::string::npos);
}

TEST_CASE("Predictive temperature handles Benchy solid bridge solid perimeter external chain", "[PredictiveTemperature][BenchyDerived]")
{
    GCodeGenerator gcodegen;
    auto buffer = make_predictive_temperature_buffer(gcodegen, predictive_temperature_config());

    const std::string external_role = role_tag(GCodeExtrusionRole::ExternalPerimeter);
    const std::string gcode =
        "M109 S200\n" +
        role_tag(GCodeExtrusionRole::SolidInfill) +
        width_height_tags_for(0.45, 0.2) +
        "G1 F6000\n"
        "G1 X100 E1\n" +      // High-flow solid infill pattern found around top decks.
        role_tag(GCodeExtrusionRole::BridgeInfill) +
        width_height_tags_for(0.50, 0.2) +
        "G1 F6000\n"
        "G1 X120 E2\n" +      // Bridge target ignored even with high speed.
        role_tag(GCodeExtrusionRole::SolidInfill) +
        width_height_tags_for(0.45, 0.2) +
        "G1 F1200\n"
        "G1 X160 E3\n" +
        role_tag(GCodeExtrusionRole::Perimeter) +
        width_height_tags_for(0.62, 0.2) +
        "G1 F1200\n"
        "G1 X180 E4\n" +
        external_role +
        width_height_tags_for(0.45, 0.2) +
        "G1 F1200\n"
        "G1 X190 E5\n";

    const std::string out = process_all(*buffer, gcode);

    CHECK(out.find("M104 S250 ; predictive nozzle temperature") == std::string::npos);
    const size_t solid_high = out.find("M104 S260 ; predictive nozzle temperature");
    const size_t external = out.find("M104 S218 ; predictive nozzle temperature");
    REQUIRE(solid_high == std::string::npos);
    REQUIRE(external != std::string::npos);
}

TEST_CASE("Predictive temperature handles Benchy overhang external alternation", "[PredictiveTemperature][BenchyDerived]")
{
    GCodeGenerator gcodegen;
    auto buffer = make_predictive_temperature_buffer(gcodegen, predictive_temperature_config());

    const std::string gcode =
        "M109 S200\n" +
        role_tag(GCodeExtrusionRole::Perimeter) +
        width_height_tags_for(0.62, 0.2) +
        "G1 F1200\n"
        "G1 X20 E1\n" +
        role_tag(GCodeExtrusionRole::OverhangPerimeter) +
        width_height_tags_for(0.62, 0.2) +
        "G1 F6000\n"
        "G1 X40 E2\n" +      // Would be clamped high if overhang flow were used.
        role_tag(GCodeExtrusionRole::ExternalPerimeter) +
        width_height_tags_for(0.45, 0.2) +
        "G1 F1200\n"
        "G1 X60 E3\n" +
        role_tag(GCodeExtrusionRole::OverhangPerimeter) +
        width_height_tags_for(0.62, 0.2) +
        "G1 F6000\n"
        "G1 X80 E4\n" +
        role_tag(GCodeExtrusionRole::ExternalPerimeter) +
        width_height_tags_for(0.45, 0.2) +
        "G1 F1200\n"
        "G1 X100 E5\n";

    const std::string out = process_all(*buffer, gcode);

    CHECK(out.find("M104 S260 ; predictive nozzle temperature") == std::string::npos);
    CHECK(out.find("M104 S218 ; predictive nozzle temperature") != std::string::npos);
}

TEST_CASE("Predictive temperature handles Benchy final layer perimeter external top-solid sequence", "[PredictiveTemperature][BenchyDerived]")
{
    GCodeGenerator gcodegen;
    auto buffer = make_predictive_temperature_buffer(gcodegen, predictive_temperature_config());

    const std::string top_role = role_tag(GCodeExtrusionRole::TopSolidInfill);
    const std::string gcode =
        "M109 S200\n" +
        role_tag(GCodeExtrusionRole::Perimeter) +
        width_height_tags_for(0.622282, 0.2) +
        "G1 F1200\n"
        "G1 X40 E1\n" +
        role_tag(GCodeExtrusionRole::ExternalPerimeter) +
        width_height_tags_for(0.449999, 0.2) +
        "G1 F1200\n"
        "G1 X80 E2\n" +
        top_role +
        width_height_tags_for(0.424476, 0.2) +
        "G1 F1200\n"
        "G1 X120 E3\n" +
        "G1 F6000\n"
        "G1 X140 E4\n";      // Benchy top solid alternates short slow and faster infill strokes.

    const std::string out = process_all(*buffer, gcode);

    const size_t external = out.find("M104 S218 ; predictive nozzle temperature");
    const size_t top_slow = out.find("M104 S217 ; predictive nozzle temperature");
    const size_t top_fast = out.find("M104 S260 ; predictive nozzle temperature");
    const size_t top_tag = out.find(top_role);
    REQUIRE(external != std::string::npos);
    REQUIRE(top_slow != std::string::npos);
    REQUIRE(top_fast != std::string::npos);
    REQUIRE(top_tag != std::string::npos);
    CHECK(external < top_tag);
    CHECK(top_slow < top_fast);
}
