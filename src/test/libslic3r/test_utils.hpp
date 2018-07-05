#ifndef SLIC3R_TEST_UTILS_H
#define SLIC3R_TEST_UTILS_H

#include <iostream>
#include <string>
#include "libslic3r.h"
#include "TriangleMesh.hpp"

using namespace std;

class TestUtils
{
public:
    static TriangleMesh init_print(string model_type)
    {
        if (model_type == "20mm_cube")
            return TriangleMesh::make_cube(20, 20, 20);
        else
            return TriangleMesh();
    }
};

#endif //SLIC3R_TEST_UTILS_H
