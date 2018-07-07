#include <catch.hpp>
//#include "/home/ahmedsamir/FreeTime/SamirSlic3r/Slic3r/src/Catch2-2.0.1/include/catch.hpp" // It's kept for IDE suggestions.

#include "Config.hpp"
#include "test_utils.hpp"
#include "SupportMaterial.hpp"

// Check that supports are not created for object that doesn't need support.
TEST_CASE("Supports_Test_1", "T1")
{
    // Create a mesh & modelObject.
    Model model = model.read_from_file("../src/test/models/CubeShape.3mf");

    // Create Print.
    Print print = Print();

    // Configure the printObjectConfig.
    print.default_object_config.set_deserialize("support_material", "1");

    // Add the modelObject.
    print.add_model_object(model.objects[0]);

    // Generate supports
    print.get_object(0)->_generate_support_material();

    REQUIRE(print.get_object(0)->support_layer_count() == 0);

    // Add raft_layers and change configs.
    print.default_object_config.set_deserialize("raft_layers", "3");
    print.default_object_config.set_deserialize("first_layer_height", "0.4");
    print.default_object_config.set_deserialize("layer_height", "0.3");

    print.reload_object(0);
    print.get_object(0)->_slice();
    print.get_object(0)->_generate_support_material();

    REQUIRE(print.get_object(0)->support_layer_count() == 3);
}

// Check intermediate support material shall be extruded at a layer height of maximum_support_layer_height
// The default is 0.8 * nozzle diameter.
TEST_CASE("Supports_Test_2", "T2") {
    // Create a mesh.
    TriangleMesh mesh = TestUtils::init_print("overhangs");

    // Create modelObject.
    Model model;
    ModelObject* object = model.add_object();
    object->add_volume(mesh);
    model.add_default_instances();

    // Align to origin.
    model.align_instances_to_origin();

    // Create Print.
    Print print = Print();
    print.config.set_deserialize("nozzle_diameter", "0.4");
    print.default_object_config.set_deserialize("raft_layers", "0");

    print.reload_object(0);
    print.get_object(0)->_slice();
    print.get_object(0)->_generate_support_material();

    REQUIRE(print.get_object(0)->support_layer_count() > 0);

    REQUIRE(abs(print.get_object(0)->support_layers[1]->height - 0.4 * 0.8) < EPSILON);
}