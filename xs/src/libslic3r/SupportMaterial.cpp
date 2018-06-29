#include "SupportMaterial.hpp"

namespace Slic3r
{

/// Minimum layer height for the variable layer height algorithm.
/// \param print_config
/// \param idx_nozzle
/// \return
inline coordf_t min_layer_height_from_nozzle(const PrintConfig &print_config, int idx_nozzle)
{
    coordf_t min_layer_height = print_config.min_layer_height.get_at(idx_nozzle - 1);
    return (min_layer_height == 0.0) ? MIN_LAYER_HEIGHT_DEFAULT : std::max(MIN_LAYER_HEIGHT, min_layer_height);
}

/// Maximum layer height for the variable layer height algorithm, 3/4 of a nozzle dimaeter by default,
/// it should not be smaller than the minimum layer height.
/// \param print_config
/// \param idx_nozzle
/// \return
inline coordf_t max_layer_height_from_nozzle(const PrintConfig &print_config, int idx_nozzle)
{
    coordf_t min_layer_height = min_layer_height_from_nozzle(print_config, idx_nozzle);
    coordf_t max_layer_height = print_config.max_layer_height.get_at(idx_nozzle - 1);
    coordf_t nozzle_dmr       = print_config.nozzle_diameter.get_at(idx_nozzle - 1);
    return std::max(min_layer_height, (max_layer_height == 0.) ? (0.75 * nozzle_dmr) : max_layer_height);
}

// To Be Removed later oo.
SupportParameters
SupportParameters::create_from_config(const PrintConfig &print_config,
                                      const PrintObjectConfig &object_config,
                                      coordf_t object_height,
                                      const std::vector<unsigned int> &object_extruders)
{
    // Get the support extruder nozzle diameter.
    // TODO @Samir55 change this name.
    // If object_config.support_material_extruder == 0 resp. object_config.support_material_interface_extruder == 0,
    // which is consistent with the requirement that if support_material_extruder == 0 resp. support_material_interface_extruder == 0,
    // print_config.nozzle_diameter.get_at(size_t(-1)) returns the 0th nozzle diameter,
    // support will not trigger tool change, but it will use the current nozzle instead.
    // In that case all the nozzles have to be of the same diameter. // TODO @Samir55 This is intuitive?
    coordf_t support_material_extruder_dmr =
        print_config.nozzle_diameter.get_at(static_cast<size_t>(object_config.support_material_extruder.value - 1));

    coordf_t support_material_interface_extruder_dmr =
        print_config.nozzle_diameter.get_at(static_cast<size_t>(object_config.support_material_interface_extruder.value - 1));


    // Is the support of soluble type.
    bool soluble_interface = object_config.support_material_contact_distance.value == 0.0;

    // Get first layer height.
    coordf_t first_layer_height = (object_config.first_layer_height.value <= 0) ?
                                  object_config.layer_height.value :
                                  object_config.first_layer_height.get_abs_value(object_config.layer_height.value);

    // Create a support parameter object and return it.
    SupportParameters params;

    params.layer_height = object_config.layer_height.value;
    params.first_print_layer_height = first_layer_height;
    params.first_object_layer_height = first_layer_height;

    params.object_print_z_min = 0.0;
    params.object_print_z_max = object_height;

    params.base_raft_layers = static_cast<size_t>(object_config.raft_layers.value); // number of raft layers.
    params.soluble_interface = soluble_interface;

    // Minimum/maximum of the minimum layer height over all extruders.
    params.min_layer_height = MIN_LAYER_HEIGHT;
    params.max_layer_height = numeric_limits<double>::max();

    // Check if the print has some form of support. Add the support layers to the minimum / maximum layer height limits.
    if (object_config.support_material.value || params.base_raft_layers > 0)
    {
        params.min_layer_height = std::max(
            min_layer_height_from_nozzle(print_config, object_config.support_material_extruder),
            min_layer_height_from_nozzle(print_config, object_config.support_material_interface_extruder));

        params.max_layer_height = std::min(
            max_layer_height_from_nozzle(print_config, object_config.support_material_extruder),
            max_layer_height_from_nozzle(print_config, object_config.support_material_interface_extruder));

        params.max_suport_layer_height = params.max_layer_height;
    }

    if (object_extruders.empty())
    {
        params.min_layer_height = std::max(params.min_layer_height, min_layer_height_from_nozzle(print_config, 0));
        params.max_layer_height = std::min(params.max_layer_height, max_layer_height_from_nozzle(print_config, 0));
    } else {
        for (unsigned int extruder_id : object_extruders)
        {
            params.min_layer_height =
                std::max(params.min_layer_height, min_layer_height_from_nozzle(print_config, extruder_id));
            params.max_layer_height =
                std::min(params.max_layer_height, max_layer_height_from_nozzle(print_config, extruder_id));
        }
    }
    params.min_layer_height = std::min(params.min_layer_height, params.layer_height);
    params.max_layer_height = std::max(params.max_layer_height, params.layer_height);

    if (!soluble_interface)
    {
        params.gap_raft_object = object_config.support_material_contact_distance.value;
        params.gap_object_support = object_config.support_material_contact_distance.value;
        params.gap_support_object = object_config.support_material_contact_distance.value;
    }

    // TODO @Samir55 Check here.
    if (params.base_raft_layers > 0)
    {
        params.interface_raft_layers = (params.base_raft_layers + 1) / 2;
        params.base_raft_layers -= params.interface_raft_layers;

        // Use as large as possible layer height for the intermediate raft layers.
        params.base_raft_layer_height = std::max(params.layer_height, 0.75 * support_material_extruder_dmr);
        params.interface_raft_layer_height =
            std::max(params.layer_height, 0.75 * support_material_interface_extruder_dmr);
        params.contact_raft_layer_height_bridging = false;
        params.first_object_layer_bridging = false;

        params.contact_raft_layer_height =
            std::max(params.layer_height, 0.75 * support_material_interface_extruder_dmr);
        if (!soluble_interface)
        {
            // Compute the average of all nozzles used for printing the object over a raft.
            //FIXME It is expected, that the 1st layer of the object is printed with a bridging flow over a full raft. Shall it not be vice versa?
            coordf_t average_object_extruder_dmr = 0.;
            if (!object_extruders.empty())
            {
                for (unsigned int extruder_id : object_extruders)
                    average_object_extruder_dmr += print_config.nozzle_diameter.get_at(extruder_id);
                average_object_extruder_dmr /= coordf_t(object_extruders.size());
            }
            params.first_object_layer_height = average_object_extruder_dmr;
            params.first_object_layer_bridging = true;
        }
    }

    if (params.has_raft())
    {
        // Raise first object layer Z by the thickness of the raft itself plus the extra distance required by the support material logic.
        //FIXME The last raft layer is the contact layer, which shall be printed with a bridging flow for ease of separation. Currently it is not the case.
        if (params.raft_layers() == 1)
        {
            // There is only the contact layer.
            params.contact_raft_layer_height = first_layer_height;
            params.raft_contact_top_z = first_layer_height;
        }
        else
        {
            assert(params.base_raft_layers > 0);
            assert(params.interface_raft_layers > 0);
            // Number of the base raft layers is decreased by the first layer.
            params.raft_base_top_z =
                first_layer_height + coordf_t(params.base_raft_layers - 1) * params.base_raft_layer_height;
            // Number of the interface raft layers is decreased by the contact layer.
            params.raft_interface_top_z = params.raft_base_top_z
                + coordf_t(params.interface_raft_layers - 1) * params.interface_raft_layer_height;
            params.raft_contact_top_z = params.raft_interface_top_z + params.contact_raft_layer_height;
        }
        coordf_t print_z = params.raft_contact_top_z + params.gap_raft_object;
        params.object_print_z_min = print_z;
        params.object_print_z_max += print_z;
    }

    return params;
}

PrintObjectSupportMaterial::PrintObjectSupportMaterial(const PrintObject *object,
                                                       const SupportParameters &support_params) :
    m_object(object),
    m_print_config(&object->print()->config),
    m_object_config(&object->config),
    m_support_params(support_params),
    m_first_layer_flow(support_material_1st_layer_flow(object, float(support_params.first_print_layer_height))),
    m_support_material_flow(support_material_flow(object, float(support_params.layer_height))),
    m_support_material_interface_flow(support_material_interface_flow(object, float(support_params.layer_height)))
{
    // Calculate a minimum support layer height as a minimum over all extruders, but not smaller than 10um.

    // Evaluate the XY gap between the object outer perimeters and the support structures.
}

/// TODO @Samir55
/// \param layer_storage
/// \param layer_type
/// \return
inline PrintObjectSupportMaterial::MyLayer &
layer_allocate(deque<PrintObjectSupportMaterial::MyLayer> &layer_storage,
               PrintObjectSupportMaterial::SupportLayerType layer_type)
{
    // Create a new Mylayer and enqueue it at the deque.
    layer_storage.emplace_back();

    // Add this new layer type.
    layer_storage.back().layer_type = layer_type;

    return layer_storage.back();
}

// TODO @Samir55 Add the thread safe allocator.
/*
 inline PrintObjectSupportMaterial::MyLayer& layer_allocate(
    std::deque<PrintObjectSupportMaterial::MyLayer> &layer_storage,
    tbb::spin_mutex                                 &layer_storage_mutex,
    PrintObjectSupportMaterial::SupporLayerType      layer_type)
{
    layer_storage_mutex.lock();
    layer_storage.push_back(PrintObjectSupportMaterial::MyLayer());
    PrintObjectSupportMaterial::MyLayer *layer_new = &layer_storage.back();
    layer_storage_mutex.unlock();
    layer_new->layer_type = layer_type;
    return *layer_new;
}
 */

/// Append MyLayers to a destination vector.
/// \param dst MyLayers destination vector.
/// \param src MyLayers source vector.
inline void
layers_append(PrintObjectSupportMaterial::MyLayersPtr &dst, const PrintObjectSupportMaterial::MyLayersPtr &src)
{
    dst.insert(dst.end(), src.begin(), src.end());
}

/// Compare layers lexicographically.
struct MyLayersPtrCompare
{
    bool
    operator()(const PrintObjectSupportMaterial::MyLayer *layer1,
               const PrintObjectSupportMaterial::MyLayer *layer2) const
    {
        return *layer1 < *layer2;
    }
};

void
PrintObjectSupportMaterial::generate(PrintObject &object)
{

}

bool
PrintObjectSupportMaterial::has_raft() const
{
    return false;
}

bool
PrintObjectSupportMaterial::has_support() const
{
    return false;
}

bool
PrintObjectSupportMaterial::build_plate_only() const
{
    return false;
}

bool
PrintObjectSupportMaterial::synchronize_layers() const
{
    return false;
}

bool
PrintObjectSupportMaterial::has_contact_loops() const
{
    return false;
}

PrintObjectSupportMaterial::MyLayersPtr
PrintObjectSupportMaterial::top_contact_layers(const PrintObject &object,
                                               PrintObjectSupportMaterial::MyLayerStorage &layer_storage) const
{
    return Slic3r::PrintObjectSupportMaterial::MyLayersPtr();
}

PrintObjectSupportMaterial::MyLayersPtr
PrintObjectSupportMaterial::bottom_contact_layers_and_layer_support_areas(const PrintObject &object,
                                                                          const PrintObjectSupportMaterial::MyLayersPtr &top_contacts,
                                                                          PrintObjectSupportMaterial::MyLayerStorage &layer_storage,
                                                                          std::vector<Polygons> &layer_support_areas) const
{
    return Slic3r::PrintObjectSupportMaterial::MyLayersPtr();
}

void
PrintObjectSupportMaterial::trim_top_contacts_by_bottom_contacts(const PrintObject &object,
                                                                 const PrintObjectSupportMaterial::MyLayersPtr &bottom_contacts,
                                                                 PrintObjectSupportMaterial::MyLayersPtr &top_contacts) const
{

}

PrintObjectSupportMaterial::MyLayersPtr
PrintObjectSupportMaterial::raft_and_intermediate_support_layers(const PrintObject &object,
                                                                 const PrintObjectSupportMaterial::MyLayersPtr &bottom_contacts,
                                                                 const PrintObjectSupportMaterial::MyLayersPtr &top_contacts,
                                                                 PrintObjectSupportMaterial::MyLayerStorage &layer_storage) const
{
    return Slic3r::PrintObjectSupportMaterial::MyLayersPtr();
}

void
PrintObjectSupportMaterial::generate_base_layers(const PrintObject &object,
                                                 const PrintObjectSupportMaterial::MyLayersPtr &bottom_contacts,
                                                 const PrintObjectSupportMaterial::MyLayersPtr &top_contacts,
                                                 PrintObjectSupportMaterial::MyLayersPtr &intermediate_layers,
                                                 const std::vector<Polygons> &layer_support_areas) const
{

}
PrintObjectSupportMaterial::MyLayersPtr
PrintObjectSupportMaterial::generate_raft_base(const PrintObjectSupportMaterial::MyLayersPtr &top_contacts,
                                               const PrintObjectSupportMaterial::MyLayersPtr &interface_layers,
                                               const PrintObjectSupportMaterial::MyLayersPtr &base_layers,
                                               PrintObjectSupportMaterial::MyLayerStorage &layer_storage) const
{
    return Slic3r::PrintObjectSupportMaterial::MyLayersPtr();
}

PrintObjectSupportMaterial::MyLayersPtr
PrintObjectSupportMaterial::generate_interface_layers(const PrintObjectSupportMaterial::MyLayersPtr &bottom_contacts,
                                                      const PrintObjectSupportMaterial::MyLayersPtr &top_contacts,
                                                      PrintObjectSupportMaterial::MyLayersPtr &intermediate_layers,
                                                      PrintObjectSupportMaterial::MyLayerStorage &layer_storage) const
{
    return Slic3r::PrintObjectSupportMaterial::MyLayersPtr();
}

void
PrintObjectSupportMaterial::trim_support_layers_by_object(const PrintObject &object,
                                                          PrintObjectSupportMaterial::MyLayersPtr &support_layers,
                                                          const coordf_t gap_extra_above,
                                                          const coordf_t gap_extra_below,
                                                          const coordf_t gap_xy) const
{

}

void
PrintObjectSupportMaterial::generate_toolpaths(const PrintObject &object,
                                               const PrintObjectSupportMaterial::MyLayersPtr &raft_layers,
                                               const PrintObjectSupportMaterial::MyLayersPtr &bottom_contacts,
                                               const PrintObjectSupportMaterial::MyLayersPtr &top_contacts,
                                               const PrintObjectSupportMaterial::MyLayersPtr &intermediate_layers,
                                               const PrintObjectSupportMaterial::MyLayersPtr &interface_layers) const
{

}

// TODO @Samir55 Refactor
Flow
support_material_flow(const PrintObject *object, float layer_height)
{
    return Flow::new_from_config_width(
        frSupportMaterial,
        // The width parameter accepted by new_from_config_width is of type ConfigOptionFloatOrPercent, the Flow class takes care of the percent to value substitution.
        (object->config.support_material_extrusion_width.value > 0) ? object->config.support_material_extrusion_width : object->config.extrusion_width,
        // if object->config.support_material_extruder == 0 (which means to not trigger tool change, but use the current extruder instead), get_at will return the 0th component.
        float(object->print()->config.nozzle_diameter.get_at(object->config.support_material_extruder-1)),
        (layer_height > 0.0f) ? layer_height : float(object->config.layer_height.value),
        false);
}

// TODO @Samir55 Refactor.
Flow
support_material_1st_layer_flow(const PrintObject *object, float layer_height)
{
    const auto &width = (object->print()->config.first_layer_extrusion_width.value > 0) ? object->print()->config.first_layer_extrusion_width : object->config.support_material_extrusion_width;
    return Flow::new_from_config_width(
        frSupportMaterial,
        // The width parameter accepted by new_from_config_width is of type ConfigOptionFloatOrPercent, the Flow class takes care of the percent to value substitution.
        (width.value > 0) ? width : object->config.extrusion_width,
        float(object->print()->config.nozzle_diameter.get_at(object->config.support_material_extruder-1)),
        (layer_height > 0.0f) ? layer_height : float(object->config.first_layer_height.get_abs_value(object->config.layer_height.value)),
        false);
}

// TODO @Samir55 Refactor.
Flow
support_material_interface_flow(const PrintObject *object, float layer_height)
{
    return Flow::new_from_config_width(
        frSupportMaterialInterface,
        // The width parameter accepted by new_from_config_width is of type ConfigOptionFloatOrPercent, the Flow class takes care of the percent to value substitution.
        (object->config.support_material_extrusion_width > 0) ? object->config.support_material_extrusion_width : object->config.extrusion_width,
        // if object->config.support_material_interface_extruder == 0 (which means to not trigger tool change, but use the current extruder instead), get_at will return the 0th component.
        float(object->print()->config.nozzle_diameter.get_at(object->config.support_material_interface_extruder-1)),
        (layer_height > 0.0f) ? layer_height : float(object->config.layer_height.value),
        false);
}

}