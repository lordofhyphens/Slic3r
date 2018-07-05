#ifndef SLIC3R_TEST_UTILS_H
#define SLIC3R_TEST_UTILS_H

#include <iostream>
#include <string>
#include <fstream>
#include "libslic3r.h"
#include "TriangleMesh.hpp"

#define EPSILON 1e-4

using namespace std;

class TestUtils
{
public:
    static TriangleMesh init_print(string model_type)
    {
        if (model_type == "20mm_cube")
            return TriangleMesh::make_cube(20, 20, 20);

        else if (model_type == "overhangs") {
            ifstream file;

            file.open("../src/test/models/overhangs.model");

            Pointf3s vertices;
            for (auto i = 0; i < 61; i++) {
                int x, y, z;
                file >> x;
                file >> y;
                file >> z;
                vertices.emplace_back(x, y, z);
            }

            vector<Point3> facets(118);
            for (auto i = 0; i < 118; i++) {
                int a, b, c;
                file >> a;
                file >> b;
                file >> b;
                facets.emplace_back(a, b, c);
            }

            file.close();

            return TriangleMesh(vertices, facets);
        }
        else
            return TriangleMesh();
    }
};

#endif //SLIC3R_TEST_UTILS_H
