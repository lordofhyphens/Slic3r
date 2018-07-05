#ifndef slic3r_SupportMaterial_hpp_
#define slic3r_SupportMaterial_hpp_

#include "Polygon.hpp"
#include "ExPolygon.hpp"
#include "ClipperUtils.hpp"
#include "PrintConfig.hpp"
#include "Print.hpp"
#include "EdgeGrid.hpp"
#include "Geometry.hpp"

#include <tbb/parallel_for.h>
#include <tbb/atomic.h>
#include <tbb/spin_mutex.h>
#include <tbb/task_group.h>

#define MIN_LAYER_HEIGHT 0.01
#define MIN_LAYER_HEIGHT_DEFAULT 0.07

#define SUPPORT_SURFACES_OFFSET_PARAMETERS ClipperLib::jtSquare, 0.

// Increment used to reach MARGIN in steps to avoid trespassing thin objects
#define NUM_MARGIN_STEPS 3

using namespace std;

namespace Slic3r
{

// TODO @Samir55, Refactor later to be phased out if possible.
class SupportParameters
{
public:
    SupportParameters() { memset(this, 0, sizeof(SupportParameters)); }

    static SupportParameters
    create_from_config(
        const PrintConfig &print_config,
        const PrintObjectConfig &object_config,
        coordf_t object_height,
        const std::set<size_t> &object_extruders);

    // Has any raft layers?
    bool
    has_raft() const { return raft_layers() > 0; }

    size_t
    raft_layers() const { return base_raft_layers + interface_raft_layers; }

    // Is the 1st object layer height fixed, or could it be varied?
    bool
    first_object_layer_height_fixed() const { return !has_raft() || first_object_layer_bridging; }

    // Height of the object to be printed. This value does not contain the raft height.
    coordf_t
    object_print_z_height() const { return object_print_z_max - object_print_z_min; }

    // Number of raft layers.
    size_t base_raft_layers;
    // Number of interface layers including the contact layer.
    size_t interface_raft_layers;

    // Layer heights of the raft (base, interface and a contact layer).
    coordf_t base_raft_layer_height;
    coordf_t interface_raft_layer_height;
    coordf_t contact_raft_layer_height;
    bool contact_raft_layer_height_bridging;

    // The regular layer height, applied for all but the first layer, if not overridden by layer ranges
    // or by the variable layer thickness table.
    coordf_t layer_height;
    // Minimum / maximum layer height, to be used for the automatic adaptive layer height algorithm,
    // or by an interactive layer height editor.
    coordf_t min_layer_height;
    coordf_t max_layer_height;
    coordf_t max_suport_layer_height;

    // First layer height of the print, this may be used for the first layer of the raft
    // or for the first layer of the print.
    coordf_t first_print_layer_height;

    // Thickness of the first layer. This is either the first print layer thickness if printed without a raft,
    // or a bridging flow thickness if printed over a non-soluble raft,
    // or a normal layer height if printed over a soluble raft.
    coordf_t first_object_layer_height;

    // If the object is printed over a non-soluble raft, the first layer may be printed with a briding flow.
    bool first_object_layer_bridging;

    // Soluble interface? (PLA soluble in water, HIPS soluble in lemonen)
    // otherwise the interface must be broken off.
    bool soluble_interface;
    // Gap when placing object over raft.
    coordf_t gap_raft_object;
    // Gap when placing support over object.
    coordf_t gap_object_support;
    // Gap when placing object over support.
    coordf_t gap_support_object;

    // Bottom and top of the printed object.
    // If printed without a raft, object_print_z_min = 0 and object_print_z_max = object height.
    // Otherwise object_print_z_min is equal to the raft height.
    coordf_t raft_base_top_z;
    coordf_t raft_interface_top_z;
    coordf_t raft_contact_top_z;
    // In case of a soluble interface, object_print_z_min == raft_contact_top_z, otherwise there is a gap between the raft and the 1st object layer.
    coordf_t object_print_z_min;
    coordf_t object_print_z_max;
};

// how much we extend support around the actual contact area
constexpr coordf_t SUPPORT_MATERIAL_MARGIN = 1.5;

/// This class manages raft and supports for a single PrintObject.
/// Instantiated by Slic3r::Print::Object->_support_material()
// TODO @Samir55 This class is instantiated before the slicing starts as Object.pm will query
// TODO @Samir55 the parameters of the raft to determine the 1st layer height and thickness.
class PrintObjectSupportMaterial
{
public:

    enum SupportLayerType
    {
        sltUnknown = 0,
        // Raft support layers.
            sltRaftBase,        // Raft base layer, to be printed with the support material.
        sltRaftInterface,   // Raft interface layer, to be printed with the support interface material.

        // Support layers. TODO @Samir55 ASK.
            sltBase,            // Sparse base support layer, to be printed with a support material.

        sltBottomContact,   // Bottom contact layer placed over a top surface of an object. To be printed with a support interface material.
        sltBottomInterface, // Dense interface layer, to be printed with the support interface material. This layer is separated from an object by an sltBottomContact layer.

        sltTopInterface,    // Dense interface layer, to be printed with the support interface material. This layer is separated from an object with sltTopContact layer.
        sltTopContact,      // Top contact layer directly supporting an overhang. To be printed with a support interface material.

        sltIntermediate,    // Some undecided type yet. It will turn into sltBase first, then it may turn into sltBottomInterface or sltTopInterface.
    }; ///< Support layer type to be used by MyLayer.

    /// A support layer type used internally by the SupportMaterial class. This class carries a much more detailed
    /// information about the support layer than the layers stored in the PrintObject, mainly
    /// the MyLayer is aware of the bridging flow and the interface gaps between the object and the support.
    class MyLayer
    {
    public:
        MyLayer() :
            layer_type(sltUnknown),
            print_z(0.0),
            bottom_z(0.0),
            height(0.0),
            idx_object_layer_above(size_t(-1)),
            idx_object_layer_below(size_t(-1)),
            bridging(false),
            contact_polygons(nullptr),
            overhang_polygons(nullptr) {}

        ~MyLayer()
        {
            delete contact_polygons;
            contact_polygons = nullptr;
            delete overhang_polygons;
            overhang_polygons = nullptr;
        }

        bool
        operator==(const MyLayer &layer2) const
        {
            return print_z == layer2.print_z && height == layer2.height && bridging == layer2.bridging;
        }

        /// Order the layers lexicographically by an increasing print_z and a decreasing layer height.
        bool
        operator<(const MyLayer &layer2) const
        {
            if (print_z < layer2.print_z)
            {
                return true;
            }
            else if (print_z == layer2.print_z)
            {
                if (height > layer2.height)
                    return true;
                else if (height == layer2.height)
                {
                    // Bridging layers first.
                    return bridging && !layer2.bridging;
                }
                else
                    return false;
            }
            else
                return false;
        }

        /// For the bridging flow, bottom_print_z will be above bottom_z to account for the vertical separation.
        /// For the non-bridging flow, bottom_print_z will be equal to bottom_z. TODO @Samir55 ASK.
        coordf_t
        bottom_print_z() const { return print_z - height; }

        /// To sort the extremes of top / bottom interface layers. TODO @Samir55 ASK.
        coordf_t
        extreme_z() const { return (this->layer_type == sltTopContact) ? this->bottom_z : this->print_z; }

        SupportLayerType layer_type;

        coordf_t print_z;
        ///< Z used for printing, in unscaled coordinates.

        coordf_t bottom_z;
        ///< Bottom Z of this layer. For soluble layers, bottom_z + height = print_z, TODO @Samir55 ASK.
        ///< otherwise bottom_z + gap + height = print_z.

        coordf_t height;
        ///< Layer height in unscaled coordinates.

        size_t idx_object_layer_above;
        ///< Index of a PrintObject layer_id supported by this layer. This will be set for top contact layers.
        ///< If this is not a contact layer, it will be set to size_t(-1).

        size_t idx_object_layer_below;
        ///< Index of a PrintObject layer_id, which supports this layer. This will be set for bottom contact layers.
        ///< If this is not a contact layer, it will be set to size_t(-1).

        bool bridging;
        ///< Use a bridging flow when printing this support layer.

        Polygons polygons;
        // Polygons to be filled by the support pattern.

        Polygons *contact_polygons;
        ///< Currently for the contact layers only.
        Polygons *overhang_polygons;
        ///< MyLayer owns the contact_polygons and overhang_polygons, they are freed by the destructor.
    };

    typedef std::deque<MyLayer> MyLayerStorage;
    ///< Layers are allocated and owned by a deque. Once a layer is allocated, it is maintained
    ///< up to the end of a generate() method. The layer storage may be replaced by an allocator class in the future,
    ///< which would allocate layers by multiple chunks. TODO @Samir55 REMOVE.
    typedef std::vector<MyLayer *> MyLayersPtr;

public:
    PrintObjectSupportMaterial(const PrintObject *object,
                               const SupportParameters &support_params);

    /// Is raft enabled?
    bool
    has_raft() const;

    /// Has any support?
    bool
    has_support() const;

    /// Build supports on plate only allowed
    bool
    build_plate_only() const;

    ///
    /// \return
    bool
    synchronize_layers() const;

    ///
    /// \return
    bool
    has_contact_loops() const;

    /// Generate support material for the object.
    /// New support layers will be added to the object,
    /// with extrusion paths and islands filled in for each support layer.
    /// \param object
    void
    generate(PrintObject &object);

private:
    /// Generate top contact layers supporting overhangs. TODO @Samir55 ASK.
    /// For a soluble interface material synchronize the layer heights with the object, otherwise leave the layer height undefined.
    /// If supports over bed surface only are requested, don't generate contact layers over an object.
    /// \param object
    /// \param layer_storage
    /// \return
    MyLayersPtr
    top_contact_layers(const PrintObject &object, MyLayerStorage &layer_storage) const;

    /// Generate bottom contact layers supporting the top contact layers. TODO @Samir55 ASK.
    /// For a soluble interface material synchronize the layer heights with the object,
    /// otherwise set the layer height to a bridging flow of a support interface nozzle.
    /// \param object
    /// \param top_contacts
    /// \param layer_storage
    /// \param layer_support_areas
    /// \return
    MyLayersPtr
    bottom_contact_layers_and_layer_support_areas(
        const PrintObject &object, const MyLayersPtr &top_contacts, MyLayerStorage &layer_storage,
        std::vector<Polygons> &layer_support_areas) const;

    /// Trim the top_contacts layers with the bottom_contacts layers if they overlap, so there would not be enough vertical space for both of them. TODO @Samir55 ASK.
    /// \param object
    /// \param bottom_contacts
    /// \param top_contacts
    void
    trim_top_contacts_by_bottom_contacts(const PrintObject &object, const MyLayersPtr &bottom_contacts,
                                         MyLayersPtr &top_contacts) const;

    /// Generate raft layers and the intermediate support layers between the bottom contact and top contact surfaces.
    /// \param object
    /// \param bottom_contacts
    /// \param top_contacts
    /// \param layer_storage
    /// \return
    MyLayersPtr
    raft_and_intermediate_support_layers(
        const PrintObject &object,
        const MyLayersPtr &bottom_contacts,
        const MyLayersPtr &top_contacts,
        MyLayerStorage &layer_storage) const;

    /// Fill in the base layers with polygons.
    void
    generate_base_layers(
        const PrintObject &object,
        const MyLayersPtr &bottom_contacts,
        const MyLayersPtr &top_contacts,
        MyLayersPtr &intermediate_layers,
        const std::vector<Polygons> &layer_support_areas) const;

    /// Generate raft layers, also expand the 1st support layer
    /// in case there is no raft layer to improve support adhesion.
    /// \param top_contacts
    /// \param interface_layers
    /// \param base_layers
    /// \param layer_storage
    /// \return
    MyLayersPtr
    generate_raft_base(
        const MyLayersPtr &top_contacts,
        const MyLayersPtr &interface_layers,
        const MyLayersPtr &base_layers,
        MyLayerStorage &layer_storage) const;

    /// Turn some of the base layers into interface layers.
    MyLayersPtr
    generate_interface_layers(
        const MyLayersPtr &bottom_contacts,
        const MyLayersPtr &top_contacts,
        MyLayersPtr &intermediate_layers,
        MyLayerStorage &layer_storage) const;

    /// Trim support layers by an object to leave a defined gap between
    /// the support volume and the object.
    /// \param object
    /// \param support_layers
    /// \param gap_extra_above
    /// \param gap_extra_below
    /// \param gap_xy
    void
    trim_support_layers_by_object(
        const PrintObject &object,
        MyLayersPtr &support_layers,
        coordf_t gap_extra_above,
        coordf_t gap_extra_below,
        coordf_t gap_xy) const;

    /// Produce the actual G-code.
    /// \param object
    /// \param raft_layers
    /// \param bottom_contacts
    /// \param top_contacts
    /// \param intermediate_layers
    /// \param interface_layers
    void
    generate_toolpaths(
        const PrintObject &object,
        const MyLayersPtr &raft_layers,
        const MyLayersPtr &bottom_contacts,
        const MyLayersPtr &top_contacts,
        const MyLayersPtr &intermediate_layers,
        const MyLayersPtr &interface_layers) const;

    // Following objects are not owned by SupportMaterial class.
    const PrintObject *m_object; ///<
    const PrintConfig *m_print_config; ///<
    const PrintObjectConfig *m_object_config; ///<

    // Pre-calculated parameters shared between the object slicer and the support generator,
    // carrying information on a raft, 1st layer height, 1st object layer height, gap between the raft and object etc.
    SupportParameters m_support_params; ///<

    Flow m_first_layer_flow; ///<
    Flow m_support_material_flow; ///<
    Flow m_support_material_interface_flow; ///<

    bool m_can_merge_support_regions;
    ///< Is merging of regions allowed? Could the interface & base support regions be printed with the same extruder?

    coordf_t m_support_layer_height_min; ///<
    coordf_t m_support_layer_height_max; ///<
    coordf_t m_gap_xy; ///<
};

/// Support grid pattern class. TODO @Samir Add contour based method also if applicable.
class SupportGridPattern
{
public:
    SupportGridPattern(
        const Polygons &support_polygons,
        const Polygons &trimming_polygons,
        coordf_t support_spacing,
        coordf_t support_angle) :
        m_support_polygons(&support_polygons), m_trimming_polygons(&trimming_polygons),
        m_support_angle(support_angle), m_support_spacing(support_spacing)
    {
        if (m_support_angle != 0.0)
        {

            // Create a copy of the rotated contours.
            m_support_polygons_rotated = support_polygons;
            m_trimming_polygons_rotated = trimming_polygons;
            m_support_polygons = &m_support_polygons_rotated;
            m_trimming_polygons = &m_trimming_polygons_rotated;
            polygons_rotate(m_support_polygons_rotated, -support_angle);
            polygons_rotate(m_trimming_polygons_rotated, -support_angle);
        }
        // Create an EdgeGrid, initialize it with projection, initialize signed distance field.
        coord_t grid_resolution = coord_t(scale_(m_support_spacing));
        BoundingBox bbox = get_extents(*m_support_polygons);
        bbox.offset(20);
        bbox.align_to_grid(grid_resolution);
        m_grid.set_bbox(bbox);
        m_grid.create(*m_support_polygons, grid_resolution);
        m_grid.calculate_sdf();
        // Extract a bounding contour from the grid, trim by the object.
        m_island_samples = island_samples(*m_support_polygons);
    }

    /// Extract polygons from the grid, offset by offset_in_grid,
    /// and trim the extracted polygons by trimming_polygons.
    /// Trimming by the trimming_polygons may split the extracted polygons into pieces.
    /// Remove all the pieces, which do not contain any of the island_samples.
    /// \param offset_in_grid
    /// \return
    Polygons
    extract_support(const coord_t offset_in_grid)
    {
        // Generate islands, so each island may be tested for overlap with m_island_samples.
        ExPolygons islands ;

        // Extract polygons, which contain some of the m_island_samples.
        Polygons out;
        std::vector<std::pair<Point, bool>> samples_inside;

        for (ExPolygon &island : islands)
        {
            BoundingBox bbox = get_extents(island.contour);
            auto it_lower = std::lower_bound(m_island_samples.begin(), m_island_samples.end(), bbox.min - Point(1, 1));
            auto it_upper = std::upper_bound(m_island_samples.begin(), m_island_samples.end(), bbox.max + Point(1, 1));
            samples_inside.clear();
            for (auto it = it_lower; it != it_upper; ++it)
                if (bbox.contains(*it))
                    samples_inside.push_back(std::make_pair(*it, false));
            if (!samples_inside.empty())
            {
                // For all samples_inside count the boundary crossing.
                for (size_t i_contour = 0; i_contour <= island.holes.size(); ++i_contour)
                {
//                    Polygon &contour = (i_contour == 0) ? island.contour : island.holes[i_contour - 1]; TODO @Samir55
                    Polygon &contour = *((i_contour == 0) ? &island.contour : &island.holes[i_contour - 1]);
                    Points::const_iterator i = contour.points.begin();
                    Points::const_iterator j = contour.points.end() - 1;
                    for (; i != contour.points.end(); j = i++)
                    {
                        //FIXME this test is not numerically robust. Particularly, it does not handle horizontal segments at y == point.y well.
                        // Does the ray with y == point.y intersect this line segment?
                        for (auto &sample_inside : samples_inside)
                        {
                            if ((i->y > sample_inside.first.y) != (j->y > sample_inside.first.y))
                            {
                                double x1 = (double) sample_inside.first.x;
                                double x2 = (double) i->x
                                    + (double) (j->x - i->x) * (double) (sample_inside.first.y - i->y)
                                        / (double) (j->y - i->y);
                                if (x1 < x2)
                                    sample_inside.second = !sample_inside.second;
                            }
                        }
                    }
                }
                // If any of the sample is inside this island, add this island to the output.
                for (auto &sample_inside : samples_inside)
                    if (sample_inside.second)
                    {
                        polygons_append(out, std::move(island));
                        island.clear();
                        break;
                    }
            }
        }

        if (m_support_angle != 0.)
            polygons_rotate(out, m_support_angle);
        return out;
    }

private:
    ///
    /// \param rhs
    /// \return
    SupportGridPattern &
    operator=(const SupportGridPattern &rhs);

    /// Get some internal point of an expolygon, to be used as a representative
    /// sample to test, whether this island is inside another island.
    /// \param expoly
    /// \return
    static Point
    island_sample(const ExPolygon &expoly)
    {
        // Find the lowest point lexicographically.
        const Point *pt_min = &expoly.contour.points.front();
        for (size_t i = 1; i < expoly.contour.points.size(); ++i)
            if (expoly.contour.points[i] < *pt_min)
                pt_min = &expoly.contour.points[i];

        // Lowest corner will always be convex, in worst case denegenerate with zero angle.
        const Point &p1 = (pt_min == &expoly.contour.points.front()) ? expoly.contour.points.back() : *(pt_min - 1);
        const Point &p2 = *pt_min;
        const Point &p3 = (pt_min == &expoly.contour.points.back()) ? expoly.contour.points.front() : *(pt_min + 1);

        Vector v = (p3 - p2) + (p1 - p2);
        double l2 = double(v.x) * double(v.x) + double(v.y) * double(v.y);
        if (l2 == 0.)
            return p2;
        double coef = 20. / sqrt(l2);
        return Point(p2.x + coef * v.x, p2.y + coef * v.y);
    }

    ///
    /// \param expolygons
    /// \return
    static Points
    island_samples(const ExPolygons &expolygons)
    {
        Points pts;
        pts.reserve(expolygons.size());
        for (const ExPolygon &expoly : expolygons)
            if (expoly.contour.points.size() > 2)
            {
#if 0
                pts.push_back(island_sample(expoly));
#else
                Polygons polygons = offset(expoly, -20.f);
                for (const Polygon &poly : polygons)
                    if (!poly.points.empty())
                    {
                        pts.push_back(poly.points.front());
                        break;
                    }
#endif
            }
        // Sort the points lexicographically, so a binary search could be used to locate points inside a bounding box.
        std::sort(pts.begin(), pts.end());
        return pts;
    }

    static Points
    island_samples(const Polygons &polygons)
    {
        return island_samples(union_ex(polygons));
    }

    const Polygons *m_support_polygons; ///<
    const Polygons *m_trimming_polygons; ///<
    Polygons m_support_polygons_rotated; ///<
    Polygons m_trimming_polygons_rotated; ///<

    coordf_t m_support_angle; ///< Angle in radians, by which the whole support is rotated.
    coordf_t m_support_spacing; ///< X spacing of the support lines parallel with the Y axis.

    Slic3r::EdgeGrid::Grid m_grid;
    Points m_island_samples;
};

// Create support flow objects.
extern Flow support_material_flow(const PrintObject *object, float layer_height = 0.0f);
extern Flow support_material_1st_layer_flow(const PrintObject *object, float layer_height = 0.0f);
extern Flow support_material_interface_flow(const PrintObject *object, float layer_height = 0.0f);

}

#endif
