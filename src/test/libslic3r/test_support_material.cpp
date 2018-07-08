//#include <catch.hpp>
#include <utility>

#include "/home/ahmedsamir/FreeTime/SamirSlic3r/Slic3r/src/Catch2-2.0.1/include/catch.hpp" // It's kept for IDE suggestions.

#include "Config.hpp"
#include "test_utils.hpp"
#include "SupportMaterial.hpp"
#include "IO.hpp"

Model* create_model(string model_type) {
    // Create a mesh.
    TriangleMesh mesh = TestUtils::init_print(std::move(model_type));

    // Create modelObject.
    Model * model = new Model();
    ModelObject* object = model->add_object();
    object->add_volume(mesh);
    model->add_default_instances();

    // Align to origin.
    model->align_instances_to_origin();

    return model;
}

// Check that supports are not created for object that doesn't need support.
TEST_CASE("supports_test_1", "T1")
{
    // Create a mesh & modelObject.
    Model model = model.read_from_file("../src/test/libslic3r/models/CubeShape.3mf");

    // Create Print.
    Print print = Print();

    // Configure the printObjectConfig.
    print.default_object_config.set_deserialize("support_material", "1");

    // Add the modelObject.
    print.add_model_object(model.objects[0]);

    // Generate supports
    PrintObject* print_object = print.objects.front();
    print_object->_slice();
    print_object->_generate_support_material();

    REQUIRE(print_object->support_layer_count() == 0);

    model = model.read_from_file("../src/test/libslic3r/models/CubeShape.3mf");
    print = Print();

    // Add raft_layers and change configs.
    print.default_object_config.set_deserialize("raft_layers", "3");
    print.default_object_config.set_deserialize("first_layer_height", "0.4");
//    print.default_object_config.set_deserialize("layer_height", "0.3");

    print.add_model_object(model.objects[0]);
    print_object = print.objects.front();
    print_object->_slice();
    print_object->_generate_support_material();

    REQUIRE(print_object->support_layer_count() == 3);
//    REQUIRE (1 == 1);
}
//
//// Check intermediate support material shall be extruded at a layer height of maximum_support_layer_height
//// The default is 0.8 * nozzle diameter.
//TEST_CASE("supports_test_2", "T2") {
//    // Create a model.
//    Model* model = create_model("overhangs");
//
//    // Create Print.
//    Print print = Print();
//    print.config.set_deserialize("nozzle_diameter", "0.4");
//    print.default_object_config.set_deserialize("raft_layers", "0");
//
//    print.add_model_object(model->objects[0]);
//
//    PrintObject* print_object = print.objects.front();
//    print_object->_slice();
//    print_object->_generate_support_material();
//
//    REQUIRE (1 == 1);
//
////    REQUIRE(print_object->support_layer_count() > 0);
////    REQUIRE(abs(print_object->support_layers[1]->height - 0.4 * 0.8) < EPSILON);
//}
//
///// Test for this requirement.
///*
// The distance from the top of the support to the bottom of the supported structure shall be defined as:
// contact_distance_z === or as support_material_contact_distance in print config.
// contact_distance_z represents the real gap between the top of the generated support and the desired bottom of the above (bridging) layer.
// contact_distance_z of 0 is useful for soluble supports.
// Note: contact_distance_z + nozzle_diameter is the distance from the top of the bridging layer to the top of the support.
// */
//TEST_CASE("supports_test_3", "T3") {
//    // Create a model.
//    Model* model = create_model("20mm_cube");
//
//    // Create Print.
//    Print print = Print();
//
//    // Configure the printObjectConfig.
//    print.default_object_config.set_deserialize("raft_layers", "0");
//    print.default_object_config.set_deserialize("support_material", "1");
//    print.default_object_config.set_deserialize("support_material_enforce_layers", "100");
//    print.default_object_config.set_deserialize("first_layer_height", "0.3");
//    print.default_object_config.set_deserialize("layer_height", "0.2");
//    print.default_object_config.set_deserialize("support_material_contact_distance", "0"); // Make it soluble contact distance.
//
//    // Add the modelObject.
//    print.add_model_object(model->objects[0]);
//
//    PrintObject* print_object = print.objects.front();
//    print_object->_slice();
//    print_object->_generate_support_material();
//
//    PrintObjectSupportMaterial *support_object = print_object->get_support_material_object();
//
//    REQUIRE(support_object->m_raft_layers.empty());
//
//    bool layers_z_correct = true;
//    for (auto layer : support_object->m_top_contacts)
//        layers_z_correct = (layer->bottom_z + layer->height) > layer->print_z + EPSILON;
//
//    for (auto layer : support_object->m_intermediate_layers)
//        layers_z_correct = (layer->bottom_z + layer->height) > layer->print_z + EPSILON;
//
//    for (auto layer : support_object->m_interface_layers)
//        layers_z_correct = (layer->bottom_z + layer->height) > layer->print_z + EPSILON;
//
//    for (auto layer : support_object->m_bottom_contacts)
//        layers_z_correct = (layer->bottom_z + layer->height) > layer->print_z + EPSILON;
//
//    REQUIRE (1 == 1);
////    REQUIRE(1 + layers_z_correct);
//}

