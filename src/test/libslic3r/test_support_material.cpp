#include <catch.hpp>
#include <libslic3r/IO/TMF.hpp>
#include "GCodeReader.hpp"
#include "test_data.hpp"

using namespace std;
using namespace Slic3r;
using namespace Slic3r::Test;

// Sub test functions.
void test_1_check(Print &print, bool &a, bool &b, bool &c, bool &d);
bool test_6_check(config_ptr &config);

// Testing 0.1: supports material member functions.
//TEST_CASE("A", "A")
//{
//    auto config{Config::new_from_defaults()};
//    config->set("raft_layers", 3);
//    config->set("support_material", 1); // TODO @samir55 Add more checks
//
//    Slic3r::Model model;
//    auto print{Slic3r::Test::init_print({TestMesh::cube_20x20x20}, model, config)};
//    print.get()->objects.front()->generate_support_material();
//
//    REQUIRE(print.get()->objects.front()->support_layer_count() == 3);
//
//}

// Test 1.
SCENARIO("SupportMaterial: support_layers_z and contact_distance")
{
    GIVEN("A print object having one modelObject") {
        // Create a mesh & modelObject.
        TriangleMesh mesh = TriangleMesh::make_cube(20, 20, 20);

        // Create modelObject.
        Model model = Model();
        ModelObject *object = model.add_object();
        object->add_volume(mesh);
        model.add_default_instances();

        // Align to origin.
        model.align_instances_to_origin();
        // Create Print.
        Print print = Print();
        print.default_object_config.set_deserialize("support_material", "1");

        WHEN("First layer height = 0.4") {
            print.default_object_config.set_deserialize("layer_height", "0.2");
            print.default_object_config.set_deserialize("first_layer_height", "0.4");

            print.add_model_object(model.objects[0]);
            print.objects.front()->slice();
            bool a, b, c, d;

            test_1_check(print, a, b, c, d);
            THEN("First layer height is honored") {
                REQUIRE(a == true);
            }
            THEN("No null or negative support layers") {
                REQUIRE(b == true);
            }
            THEN("No layers thicker than nozzle diameter") {
                REQUIRE(c == true);
            }
            THEN("Layers above top surfaces are spaced correctly") {
                REQUIRE(d == true);
            }
        }
        WHEN("Layer height = 0.2 and, first layer height = 0.3") {
            print.default_object_config.set_deserialize("layer_height", "0.2");
            print.default_object_config.set_deserialize("first_layer_height", "0.3");
            print.add_model_object(model.objects[0]);
            print.objects.front()->_slice();
            bool a, b, c, d;

            test_1_check(print, a, b, c, d);
            THEN("First layer height is honored") {
                REQUIRE(a == true);
            }
            THEN("No null or negative support layers") {
                REQUIRE(b == true);
            }
            THEN("No layers thicker than nozzle diameter") {
                REQUIRE(c == true);
            }
            THEN("Layers above top surfaces are spaced correctly") {
                REQUIRE(d == true);
            }
        }


        WHEN("Layer height = nozzle_diameter[0]") {
            print.default_object_config.set_deserialize("layer_height", "0.2");
            print.default_object_config.set_deserialize("first_layer_height", "0.3");
            print.add_model_object(model.objects[0]);
            print.objects.front()->_slice();
            bool a, b, c, d;

            test_1_check(print, a, b, c, d);
            THEN("First layer height is honored") {
                REQUIRE(a == true);
            }
            THEN("No null or negative support layers") {
                REQUIRE(b == true);
            }
            THEN("No layers thicker than nozzle diameter") {
                REQUIRE(c == true);
            }
            THEN("Layers above top surfaces are spaced correctly") {
                REQUIRE(d == true);
            }
        }
    }
}

// Test 2.
TEST_CASE("Support Material: check support extruders are used for supports only", "[!mayfail]")
{
    auto config{Config::new_from_defaults()};
    config->set("raft_layers", 3);
    config->set("brim_width", 0);
    config->set("skirts", 0);
    config->set("support_material_extruder", 2);
    config->set("support_material_interface_extruder", 2);
    config->set("layer_height", 0.4);
    config->set("first_layer_height", 0.4);

    // Initialize a print object from the config.
    Slic3r::Model model;
    auto gcode{std::stringstream("")};

    auto print{Slic3r::Test::init_print({TestMesh::overhang}, model, config)};
    Slic3r::Test::gcode(gcode, print); // 'no conflict between raft/support and brim'

    REQUIRE(print.get()->objects.front()->support_layer_count() == 3);

    int tool = 0;
    auto parser{Slic3r::GCodeReader()};
    parser.parse_stream(gcode,
                        [&tool, &config](Slic3r::GCodeReader &self,
                                         const Slic3r::GCodeReader::GCodeLine &line)
                        {
                            smatch cmd_match;
                            std::regex_match(line.cmd, cmd_match, regex("^T(\\d+)"));

                            if (cmd_match.size() > 1) {
                                tool = stoi(cmd_match[1]);
                            }
                            else if (line.extruding()) {
                                if (self.Z <= (config->getInt("raft_layers") * config->getFloat("layer_height"))) {
                                    REQUIRE(tool == config->getInt("support_material_extruder") - 1);
                                    // not extruding raft with support material extruder.
                                }
                                else {
                                    REQUIRE(tool != config->getInt("support_material_extruder") - 1);
                                    // support material exceeds raft layers.
                                    // TODO: we should test that full support is generated when we use raft too
                                }
                            }
                        });
}


// Test 6.
SCENARIO("SupportMaterial: Checking bridge speed")
{
    GIVEN("Print object") {
        auto config{Config::new_from_defaults()};
        config->set("brim_width", 0);
        config->set("skirts", 0);
        config->set("support_material", 1);
        config->set("top_solid_layers", 0); // so that we don't have the internal bridge over infill.
        config->set("bridge_speed", 99);
        config->set("cooling", 0);
        config->set("first_layer_speed", "100%");

        WHEN("support_material_contact_distance = 0.2") {
            config->set("support_material_contact_distance", 0.2);

            bool check = test_6_check(config);
            REQUIRE(check == true); // bridge speed is used.
        }

        WHEN("support_material_contact_distance = 0") {
            config->set("support_material_contact_distance", 0);

            bool check = test_6_check(config);
            REQUIRE(check == false); // bridge speed is not used.
        }

        WHEN("support_material_contact_distance = 0.2 & raft_layers = 5") {
            config->set("support_material_contact_distance", 0.2);
            config->set("raft_layers", 5);

            bool check = test_6_check(config);
            REQUIRE(check == true); // bridge speed is used.
        }

        WHEN("support_material_contact_distance = 0 & raft_layers = 5") {
            config->set("support_material_contact_distance", 0);
            config->set("raft_layers", 5);

            bool check = test_6_check(config);
            REQUIRE(check == false); // bridge speed is not used.
        }
    }
}

// Test 8.
TEST_CASE("SupportMaterial: forced support is generated", "")
{
    // Create a mesh & modelObject.
    TriangleMesh mesh = TriangleMesh::make_cube(20, 20, 20);

    Model model = Model();
    ModelObject *object = model.add_object();
    object->add_volume(mesh);
    model.add_default_instances();
    model.align_instances_to_origin();

    Print print = Print();

    vector<coord_t> contact_z = {scale_(1.9)};
    vector<coord_t> top_z = {scale_(1.1)};
    print.default_object_config.support_material_enforce_layers = 100;
    print.default_object_config.support_material = 0;
    print.default_object_config.layer_height = 0.2;
    print.default_object_config.set_deserialize("first_layer_height", "0.3");

    print.add_model_object(model.objects[0]);
    print.objects.front()->_slice();

    SupportMaterial *support = print.objects.front()->_support_material();
    vector<coord_t>
        support_z =
        support->support_layers_z(contact_z, top_z, scale_(double(print.default_object_config.layer_height)));

    bool check = true;
    for (size_t i = 1; i < support_z.size(); i++) {
        if (support_z[i] - support_z[i - 1] <= 0)
            check = false;
    }

    REQUIRE(check == true);
}

void test_1_check(Print &print, bool &a, bool &b, bool &c, bool &d)
{
    vector<coord_t> contact_z = {scale_(1.9)};
    vector<coord_t> top_z = {scale_(1.1)};

    SupportMaterial *support = print.objects.front()->_support_material();

    vector<coord_t>
        support_z =
        support->support_layers_z(contact_z, top_z, scale_(double(print.default_object_config.layer_height)));

    a = (support_z[0] == scale_(print.default_object_config.first_layer_height.value));

    b = true;
    for (size_t i = 1; i < support_z.size(); ++i)
        if (support_z[i] - support_z[i - 1] <= 0) b = false;


    c = true;
    for (size_t i = 1; i < support_z.size(); ++i)
        if (support_z[i] - support_z[i - 1] > scale_(print.config.nozzle_diameter.get_at(0)))
            c = false;

    coordf_t expected_top_spacing = scale_(support
                                               ->contact_distance(print.default_object_config.layer_height,
                                                                  print.config.nozzle_diameter.get_at(0)));

    bool wrong_top_spacing = 0;
    for (coordf_t top_z_el : top_z) {
        // find layer index of this top surface.
        size_t layer_id = -1;
        for (size_t i = 0; i < support_z.size(); i++) {
            if (abs(support_z[i] - top_z_el) == 0) {
                layer_id = i;
                i = static_cast<int>(support_z.size());
            }
        }

        // check that first support layer above this top surface (or the next one) is spaced with nozzle diameter
        if (abs(support_z[layer_id + 1] - support_z[layer_id] - expected_top_spacing) > EPSILON
            && abs(support_z[layer_id + 2] - support_z[layer_id] - expected_top_spacing) > EPSILON) {
            wrong_top_spacing = 1;
        }
    }
    d = !wrong_top_spacing;
}

bool test_6_check(config_ptr &config)
{
    bool has_bridge_speed = false;

    // Initialize a print object from the config.
    Slic3r::Model model;
    auto print{Slic3r::Test::init_print({TestMesh::overhang}, model, config)};

    // Get gcode.
    auto gcode{std::stringstream("")};
    Slic3r::Test::gcode(gcode, print);

    auto parser{Slic3r::GCodeReader()};
    parser.parse_stream(gcode,
                        [&has_bridge_speed, &config](Slic3r::GCodeReader &self,
                                                     const Slic3r::GCodeReader::GCodeLine &line)
                        {
                            if (line.extruding() && line.new_F() == config->getFloat("bridge_speed") * 60) {
                                has_bridge_speed = true;
                            }
                        });

    return has_bridge_speed;
}
