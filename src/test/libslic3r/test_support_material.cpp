//#include <catch.hpp>
#include "/home/ahmedsamir/FreeTime/SamirSlic3r/Slic3r/src/Catch2-2.0.1/include/catch.hpp" // It's kept for IDE suggestions.

#include "Config.hpp"
#include "test_utils.hpp"
#include "SupportMaterial.hpp"

TEST_CASE("Supports_Tests_template", "TEST")
{
    // Create a mesh model.
    TriangleMesh mesh = TestUtils::init_print("20mm_cube");

    // Create a config object. // TODO @Samir55 Read more about shared pointers.
    PrintConfig config = PrintConfig();
    for (auto a : config.keys()) {
        cout << a << " " << config.serialize(a) << endl;
    }

    // Create a print object config.
    auto print_object_config = PrintObjectConfig();
    print_object_config.set_deserialize("support_material_xy_spacing", "0.2");

    // Create supports parameter object from the current config.
    SupportParameters support_parameter = SupportParameters();
    support_parameter.create_from_config(config, print_object_config, 10, vector<unsigned int>());

    // Create support.
//    PrintObjectSupportMaterial object_support = PrintObjectSupportMaterial()

    REQUIRE(1+2 == 3);
}