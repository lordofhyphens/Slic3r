#ifndef slic3r_SupportMaterial_hpp_
#define slic3r_SupportMaterial_hpp_

#include "PrintConfig.hpp"
#include "Print.hpp"

using namespace std;

namespace Slic3r {

// TODO @Samir55, Refactor later to be phased out if possible.
class SupportParameters{

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
            overhang_polygons(nullptr)
        {}

        ~MyLayer()
        {
            delete contact_polygons;
            contact_polygons = nullptr;
            delete overhang_polygons;
            overhang_polygons = nullptr;
        }

        bool operator==(const MyLayer &layer2) const {
            return print_z == layer2.print_z && height == layer2.height && bridging == layer2.bridging;
        }

        /// Order the layers lexicographically by an increasing print_z and a decreasing layer height.
        bool operator<(const MyLayer &layer2) const {
            if (print_z < layer2.print_z) {
                return true;
            } else if (print_z == layer2.print_z) {
                if (height > layer2.height)
                    return true;
                else if (height == layer2.height) {
                    // Bridging layers first.
                    return bridging && ! layer2.bridging;
                } else
                    return false;
            } else
                return false;
        }

        /// For the bridging flow, bottom_print_z will be above bottom_z to account for the vertical separation.
        /// For the non-bridging flow, bottom_print_z will be equal to bottom_z. TODO @Samir55 ASK.
        coordf_t bottom_print_z() const { return print_z - height; }

        /// To sort the extremes of top / bottom interface layers. TODO @Samir55 ASK.
        coordf_t extreme_z() const { return (this->layer_type == sltTopContact) ? this->bottom_z : this->print_z; }

        SupportLayerType layer_type;

        coordf_t print_z;
        ///< Z used for printing, in unscaled coordinates.

        coordf_t bottom_z;
        ///< Bottom Z of this layer. For soluble layers, bottom_z + height = print_z, TODO @Samir55 ASK.
        ///< otherwise bottom_z + gap + height = print_z.

        coordf_t height;
        ///< Layer height in unscaled coordinates.

        size_t 	 idx_object_layer_above;
        ///< Index of a PrintObject layer_id supported by this layer. This will be set for top contact layers.
        ///< If this is not a contact layer, it will be set to size_t(-1).

        size_t 	 idx_object_layer_below;
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
    PrintObjectSupportMaterial(const PrintObject *object, const PrintConfig *print_config, const PrintObjectConfig *print_object_config,
                               const SupportParameters &support_params);

    /// Is raft enabled?
    bool has_raft() const;

    /// Has any support?
    bool has_support() const;

    /// Build supports on plate only allowed
    bool build_plate_only() const;

    /// TODO @Samir55 ASK.
    bool synchronize_layers() const;

    /// TODO @Samir55 ASK.
    ///
    /// \return
    bool has_contact_loops() const;

    /// Generate support material for the object.
    /// New support layers will be added to the object,
    /// with extrusion paths and islands filled in for each support layer.
    /// \param object
    void generate(PrintObject &object);

private:
    /// Generate top contact layers supporting overhangs. TODO @Samir55 ASK.
    /// For a soluble interface material synchronize the layer heights with the object, otherwise leave the layer height undefined.
    /// If supports over bed surface only are requested, don't generate contact layers over an object.
    /// \param object
    /// \param layer_storage
    /// \return
    MyLayersPtr top_contact_layers(const PrintObject &object, MyLayerStorage &layer_storage) const;

    /// Generate bottom contact layers supporting the top contact layers. TODO @Samir55 ASK.
    /// For a soluble interface material synchronize the layer heights with the object,
    /// otherwise set the layer height to a bridging flow of a support interface nozzle.
    /// \param object
    /// \param top_contacts
    /// \param layer_storage
    /// \param layer_support_areas
    /// \return
    MyLayersPtr bottom_contact_layers_and_layer_support_areas(
        const PrintObject &object, const MyLayersPtr &top_contacts, MyLayerStorage &layer_storage,
        std::vector<Polygons> &layer_support_areas) const;

    /// Trim the top_contacts layers with the bottom_contacts layers if they overlap, so there would not be enough vertical space for both of them. TODO @Samir55 ASK.
    /// \param object
    /// \param bottom_contacts
    /// \param top_contacts
    void trim_top_contacts_by_bottom_contacts(const PrintObject &object, const MyLayersPtr &bottom_contacts,
                                              MyLayersPtr &top_contacts) const;

    /// Generate raft layers and the intermediate support layers between the bottom contact and top contact surfaces.
    /// \param object
    /// \param bottom_contacts
    /// \param top_contacts
    /// \param layer_storage
    /// \return
    MyLayersPtr raft_and_intermediate_support_layers(
        const PrintObject &object,
        const MyLayersPtr &bottom_contacts,
        const MyLayersPtr &top_contacts,
        MyLayerStorage &layer_storage) const;

    /// Fill in the base layers with polygons.
    void generate_base_layers(
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
    MyLayersPtr generate_raft_base(
        const MyLayersPtr &top_contacts,
        const MyLayersPtr &interface_layers,
        const MyLayersPtr &base_layers,
        MyLayerStorage &layer_storage) const;

    /// Turn some of the base layers into interface layers.
    MyLayersPtr generate_interface_layers(
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
    void trim_support_layers_by_object(
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
    void generate_toolpaths(
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

}

#endif
