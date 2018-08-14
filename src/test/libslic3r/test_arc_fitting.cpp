#include <catch.hpp>
#include "libslic3r.h"
#include "Point.hpp"
#include "Geometry.hpp"
#include "GCode/ArcFitting.hpp"
#include "Log.hpp"
#include "test_data.hpp"
using namespace Slic3r;
using namespace Slic3r::Test;

void make_circle(Points &output, size_t n, double r, Point center = Point(0,0), double a = 0.0, double b = 2*PI, double noise = 0.0){
    double arcdiff = (b-a)/n;
    for(auto i = 0U; i < n; i++){
        output.push_back(Point(r*cos(a)+center.x,r*sin(a))+center.y);
        a += arcdiff;
    }
}

SCENARIO("Arc detection works"){
    GIVEN("A simple line"){
        Polyline pl;
        pl.points = {Point(5,15),Point(10,15),Point(15,15), Point(20,15), Point(25,15)};
        THEN("No arcs are detected"){
            auto arcs = arc_detection(pl, 4, 1.0);
            REQUIRE(arcs.size() == pl.points.size());
            for(auto &arc : arcs){
                REQUIRE(arc == 0);
            }
        }
    }
    GIVEN("An arc with too few points"){
        Polyline pl;
        make_circle(pl.points,4,9000);
        THEN("No arcs are detected"){
            auto arcs = arc_detection(pl, 4, 2.0);
            REQUIRE(arcs.size() == pl.points.size());
            for(auto &arc : arcs){
                REQUIRE(arc == 0);
            }
        }
    }
    GIVEN("A circle"){
        Polyline pl;
        make_circle(pl.points,20,9000, Point(100,100));
        auto arcs = arc_detection(pl, 4, 10.0);
        THEN("Output is valid"){
            REQUIRE(arcs.size() == pl.points.size());
        }
        THEN("The last N points don't detect an arc"){
            for(auto i = arcs.size()-4; i < arcs.size();i++){
                REQUIRE(arcs[i] == 0);
            }
        }
        THEN("All other points detect an arc"){
            for(auto i = 0U; i < arcs.size()-4;i++){
                INFO(i);
                REQUIRE(arcs.at(i) != 0);
            }
        }
     }
    GIVEN("A zig zag line"){
        Polyline pl;
        pl.points = {Point(0,0),Point(10,10),
                     Point(20,0),Point(30,10),
                     Point(40,0),Point(50,10),
                     Point(60,0),Point(70,10),
                     Point(80,0),Point(90,10),
                     Point(100,0),Point(110,10)};
        THEN("No arcs are detected"){
            auto arcs = arc_detection(pl,4,1.0);
            REQUIRE(arcs.size() == pl.points.size());
            for(auto &arc : arcs){
                REQUIRE(arc == 0);
            }
         }
    }
    GIVEN("A line followed by an arc followed by a line"){
        Polyline pl;
        pl.points = {Point(-100,900),Point(-70,900),Point(-50,900),Point(-20,900)};
        make_circle(pl.points,20,900, PI/2, 0);
        pl.points.push_back(Point(900,-20));
        pl.points.push_back(Point(900,-50));
        pl.points.push_back(Point(900,-70));
        pl.points.push_back(Point(900,-100));
        THEN("Then arcs are only detected in a contiguous region surrounded by non-arcs"){
            auto arcs = arc_detection(pl,4,1.0);
            REQUIRE(arcs.size() == pl.points.size());
            auto i = 0U;
            for(; i < arcs.size();i++){
                if(arcs[i] != 0)break;
            }
            for(; i < arcs.size();i++){
                if(arcs[i] == 0)break;
            }
            REQUIRE(i < arcs.size());
            for(; i < arcs.size();i++){
                REQUIRE(arcs[i] == 0);
            }
            
        }
    }
}

SCENARIO("Arc clipping works"){
    GIVEN("One arc that covers the rest"){
        THEN("It will be selected"){
//            REQUIRE( false );
        }
    }
    
    // -----
    //   ----- 
    //      ----
    GIVEN("Three arcs, two of which perfectly cover"){
        THEN("Those two will be selected"){
//            REQUIRE( false );
        }
    }

    // Sanity check I used for debugging
    GIVEN("An aritrary path"){
        Polyline pl;
        pl.points = {Point(-100,900),Point(-70,900),Point(-50,900),Point(-20,900)};
        make_circle(pl.points,20,900, PI/2, 0);
        pl.points.push_back(Point(900,-20));
        pl.points.push_back(Point(900,-50));
        pl.points.push_back(Point(900,-70));
        pl.points.push_back(Point(900,-100));
        auto arcs = arc_detection(pl,4,1.0);
        auto arc_pairs = arc_fitting(arcs,pl);
        THEN("All of the final intervals are valid"){
            for(auto arc : arc_pairs){
                REQUIRE(0 <= arc.first);
                REQUIRE(arc.first < pl.points.size());
                REQUIRE(0 <= arc.second);
                REQUIRE(arc.second < pl.points.size());
            }
        }
  
    }
}

SCENARIO("Arcfitting successfully finds and replaces arcs in polylines"){
    GIVEN("A simple line"){
        Polyline pl;
        pl.points = {Point(5,15),Point(10,15),Point(15,15), Point(20,15), Point(25,15)};
        THEN("No Arcs are detected and the polyline is unchanged"){
            auto processed = arcify(pl,4,4);
            REQUIRE(pl.points.size() == processed.points.size());
            for(auto i = 0U; i < pl.points.size(); i++){
                REQUIRE(pl.points.at(i) == processed.points.at(i));
                REQUIRE(processed.arc_types.at(i) == LINE);
            }
        }
    }

    GIVEN("A circle"){
        Polyline pl;
        make_circle(pl.points,20,9000);
        auto processed = arcify(pl, 5, 10.0);
        THEN("There are fewer segments"){
            REQUIRE(pl.points.size() > processed.points.size());
        }
        THEN("There are only (CCW) arcs in the output"){
            INFO(processed.arc_types.size());
            for(auto i = 1U; i < processed.arc_types.size();i++){
                REQUIRE(processed.arc_types.at(i) == CCW);        
            }
        }
        THEN("The centers are all at 0,0"){
            for(auto point : processed.centers){
                REQUIRE(point == Point(0,0));
            }
        }
    }

    GIVEN("A line followed by an arc followed by a line"){
        Polyline pl;
        pl.points = {Point(-100,900),Point(-70,900),Point(-50,900),Point(-20,900)};
        make_circle(pl.points,20,900, PI/2, 0);
        pl.points.push_back(Point(900,-20));
        pl.points.push_back(Point(900,-50));
        pl.points.push_back(Point(900,-70));
        pl.points.push_back(Point(900,-100));
        auto processed = arcify(pl, 5, 2.0);
        THEN("There are fewer segments"){
            REQUIRE(pl.points.size() > processed.points.size());
        }
        THEN("Exactly one arc is outputed"){
            auto i = 0U;
            for(; i < processed.points.size()-1;i++){
                if(processed.arc_types.at(i+1) != LINE)break;
                REQUIRE(processed.points.at(i) == pl.points.at(i));
            }
            i++; 
            REQUIRE(i < processed.points.size());
            REQUIRE(processed.arc_types.at(i) == CW);
            for(; i < processed.points.size();i++){
                REQUIRE(processed.arc_types.at(i) == LINE);
            }
        }
    }
    
    GIVEN("An arc slit"){
        int w = 9000;
        int h = 9000;

        
        Point right(w,0), left(-w,0);
        double r = sqrt(h*h+w*w);
        double angle = atan(h/w);
        Polyline pl;
        make_circle(pl.points,20,r,left,angle, -angle);
        make_circle(pl.points,20,r,right,PI+angle, PI-angle);
        Log::warn("test") << std::endl;
        auto processed = arcify(pl,5,10);
        THEN("There are fewer segments"){
            REQUIRE(pl.points.size() > processed.points.size());
        }
        THEN("Three arcs (with correct centers) will be outputed"){
            INFO(processed.arc_types.at(0));
            INFO(processed.arc_types.at(1));
            INFO(processed.arc_types.at(2));
            REQUIRE(processed.arc_types.size() == 3);
            for(auto i = 0U; i < 4; i++){
                REQUIRE(processed.arc_types.at(i) != LINE);
            }
            REQUIRE(processed.centers.at(0) == left);
            REQUIRE(processed.centers.at(1) == right);
        }
        
    } 

    GIVEN("A Reuleaux triangle"){
        int R = 900000;
        Point top(0,(int)(R*sin(PI/3))), right(R/2,0), left(-R/2,0);
        Polyline pl;
        make_circle(pl.points,20,R,top,5*PI/3, 4*PI/3);
        make_circle(pl.points,20,R,right,PI, 2*PI/3);
        make_circle(pl.points,20,R,left,PI/3, 0);
        Log::warn("test") << std::endl;
        auto processed = arcify(pl,5,10);
        THEN("There are fewer segments"){
            REQUIRE(pl.points.size() > processed.points.size());
        }
        THEN("Three arcs (with correct centers) will be outputed"){
            INFO(processed.arc_types.at(0));
            INFO(processed.arc_types.at(1));
            INFO(processed.arc_types.at(2));
            REQUIRE(processed.arc_types.size() == 4);
            for(auto i = 0U; i < 4; i++){
                REQUIRE(processed.arc_types.at(i) != LINE);
            }
            REQUIRE(processed.centers.at(0) == top);
            REQUIRE(processed.centers.at(1) == right);
            REQUIRE(processed.centers.at(2) == left);
        }
        
    } 
}

SCENARIO("Arcfitting gcode functionality"){
    GIVEN("A simple cube print"){
        auto config {Slic3r::Config::new_from_defaults()};
        auto no_arcs {std::stringstream("")};
        auto arcs {std::stringstream("")};
        config->set("first_layer_extrusion_width", 0);
        config->set("start_gcode", "");
        {
            Slic3r::Model model;
            auto print {Slic3r::Test::init_print({TestMesh::sphere_50mm}, model, config)};
            print->process();
            Slic3r::Test::gcode(no_arcs, print);
        }
        {
            Slic3r::Model model;
            config->set("gcode_arcs", true);
            auto print {Slic3r::Test::init_print({TestMesh::sphere_50mm}, model, config)};
            print->process();
            Slic3r::Test::gcode(arcs, print);
        }
        THEN("Toggling on arc fitting doesn't change the gcode"){
            std::string a, b;
            std::regex comments(";.*$");
            a = std::regex_replace(arcs.str(),comments,";");
            //a.erase(0,200);
            //a.erase(a.end()-2000,a.end());
            //a.resize(24000);
            //auto b = no_arcs.str();
            b = std::regex_replace(no_arcs.str(),comments,";");
            //b.erase(0,200);
            //b.erase(b.end()-2000,b.end());
           // b.resize(24000);
            REQUIRE(a == b);
            //REQUIRE(arcs.str() == no_arcs.str());
        }
    }
}
