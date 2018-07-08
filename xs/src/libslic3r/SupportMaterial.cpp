#include "SupportMaterial.hpp"
#include "SVG.hpp"

namespace Slic3r
{

// FN_HIGHER_EQUAL: the provided object pointer has a Z value >= of an internal threshold.
// Find the first item with Z value >= of an internal threshold of fn_higher_equal.
// If no vec item with Z value >= of an internal threshold of fn_higher_equal is found, return vec.size()
// If the initial idx is size_t(-1), then use binary search.
// Otherwise search linearly upwards.
template<typename T, typename FN_HIGHER_EQUAL>
size_t idx_higher_or_equal(const std::vector<T*> &vec, size_t idx, FN_HIGHER_EQUAL fn_higher_equal)
{
    if (vec.empty()) {
        idx = 0;
    } else if (idx == size_t(-1)) {
        // First of the batch of layers per thread pool invocation. Use binary search.
        int idx_low  = 0;
        int idx_high = std::max(0, int(vec.size()) - 1);
        while (idx_low + 1 < idx_high) {
            int idx_mid  = (idx_low + idx_high) / 2;
            if (fn_higher_equal(vec[idx_mid]))
                idx_high = idx_mid;
            else
                idx_low  = idx_mid;
        }
        idx =  fn_higher_equal(vec[idx_low])  ? idx_low  :
               (fn_higher_equal(vec[idx_high]) ? idx_high : vec.size());
    } else {
        // For the other layers of this batch of layers, search incrementally, which is cheaper than the binary search.
        while (idx < vec.size() && ! fn_higher_equal(vec[idx]))
            ++ idx;
    }
    return idx;
}

// FN_LOWER_EQUAL: the provided object pointer has a Z value <= of an internal threshold.
// Find the first item with Z value <= of an internal threshold of fn_lower_equal.
// If no vec item with Z value <= of an internal threshold of fn_lower_equal is found, return -1.
// If the initial idx is < -1, then use binary search.
// Otherwise search linearly downwards.
template<typename T, typename FN_LOWER_EQUAL>
int idx_lower_or_equal(const std::vector<T*> &vec, int idx, FN_LOWER_EQUAL fn_lower_equal)
{
    if (vec.empty()) {
        idx = -1;
    } else if (idx < -1) {
        // First of the batch of layers per thread pool invocation. Use binary search.
        int idx_low  = 0;
        int idx_high = std::max(0, int(vec.size()) - 1);
        while (idx_low + 1 < idx_high) {
            int idx_mid  = (idx_low + idx_high) / 2;
            if (fn_lower_equal(vec[idx_mid]))
                idx_low  = idx_mid;
            else
                idx_high = idx_mid;
        }
        idx =  fn_lower_equal(vec[idx_high]) ? idx_high :
               (fn_lower_equal(vec[idx_low ]) ? idx_low  : -1);
    } else {
        // For the other layers of this batch of layers, search incrementally, which is cheaper than the binary search.
        while (idx >= 0 && ! fn_lower_equal(vec[idx]))
            -- idx;
    }
    return idx;
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

inline PrintObjectSupportMaterial::MyLayer &
layer_allocate(
    std::deque<PrintObjectSupportMaterial::MyLayer> &layer_storage,
    tbb::spin_mutex &layer_storage_mutex,
    PrintObjectSupportMaterial::SupportLayerType layer_type)
{
    layer_storage_mutex.lock();
    layer_storage.push_back(PrintObjectSupportMaterial::MyLayer());
    PrintObjectSupportMaterial::MyLayer *layer_new = &layer_storage.back();
    layer_storage_mutex.unlock();
    layer_new->layer_type = layer_type;
    return *layer_new;
}

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

/// Append MyLayers to a destination vector.
/// \param dst MyLayers destination vector.
/// \param src MyLayers source vector.
inline void
layers_append(PrintObjectSupportMaterial::MyLayersPtr &dst, const PrintObjectSupportMaterial::MyLayersPtr &src)
{
    dst.insert(dst.end(), src.begin(), src.end());
}

/// Minimum layer height for the variable layer height algorithm.
/// \param print_config
/// \param idx_nozzle
/// \return
inline coordf_t
min_layer_height_from_nozzle(const PrintConfig &print_config, int idx_nozzle)
{
    coordf_t min_layer_height = print_config.min_layer_height.get_at(idx_nozzle - 1);
    return (min_layer_height == 0.0) ? MIN_LAYER_HEIGHT_DEFAULT : std::max(MIN_LAYER_HEIGHT, min_layer_height);
}

/// Maximum layer height for the variable layer height algorithm, 3/4 of a nozzle diameter by default,
/// it should not be smaller than the minimum layer height.
/// \param print_config
/// \param idx_nozzle
/// \return
inline coordf_t
max_layer_height_from_nozzle(const PrintConfig &print_config, int idx_nozzle)
{
    coordf_t min_layer_height = min_layer_height_from_nozzle(print_config, idx_nozzle);
    coordf_t max_layer_height = print_config.max_layer_height.get_at(idx_nozzle - 1);
    coordf_t nozzle_dmr = print_config.nozzle_diameter.get_at(idx_nozzle - 1);
    return std::max(min_layer_height, (max_layer_height == 0.) ? (0.75 * nozzle_dmr) : max_layer_height);
}

PrintObjectSupportMaterial::PrintObjectSupportMaterial(const PrintObject *object,
                                                       const SupportParameters &support_params)
    :
    m_object(object),
    m_print_config(&object->print()->config),
    m_object_config(&object->config),
    m_support_params(support_params),
    m_first_layer_flow(support_material_1st_layer_flow(object, float(support_params.first_print_layer_height))),
    m_support_material_flow(support_material_flow(object, float(support_params.layer_height))),
    m_support_material_interface_flow(support_material_interface_flow(object, float(support_params.layer_height)))
{
    // Calculate a minimum support layer height as a minimum over all extruders, but not smaller than 10um.
    m_support_layer_height_min = 1000000.0;
    for (auto lh : m_print_config->min_layer_height.values)
        m_support_layer_height_min = min(m_support_layer_height_min, max(0.1, lh));

    // if no interface layers allowed, print everything with the base support pattern.
    if (m_object_config->support_material_interface_layers.value == 0)
        m_support_material_interface_flow = m_support_material_flow;

    // TODO @Samir55 what's region ?
    // Evaluate the XY gap between the object outer perimeters and the support structures.
    coordf_t external_perimeter_width = 0.0;
    for (size_t region_id = 0; region_id < object->region_volumes.size(); ++region_id) {
        if (object->region_volumes.count(region_id) > 0 && !object->region_volumes.at(region_id).empty()) {
            const PrintRegionConfig &config = object->print()->get_region(region_id)->config;
            coordf_t width = config.external_perimeter_extrusion_width.get_abs_value(m_support_params.layer_height);
            if (width <= 0.0)
                width = m_print_config->nozzle_diameter.get_at(config.perimeter_extruder - 1);
            external_perimeter_width = std::max(external_perimeter_width, width);
        }
    }
    m_gap_xy = m_object_config->support_material_xy_spacing.get_abs_value(external_perimeter_width);

    // TODO @Samir see later.
    m_can_merge_support_regions =
        m_object_config->support_material_extruder.value == m_object_config->support_material_interface_extruder.value;
    if (!m_can_merge_support_regions && (m_object_config->support_material_extruder.value == 0
        || m_object_config->support_material_interface_extruder.value == 0)) {
        // One of the support extruders is of "don't care" type.
        auto object_extruders = m_object->print()->object_extruders();
        if (object_extruders.size() == 1 &&
            *object_extruders.begin() == std::max<unsigned int>(m_object_config->support_material_extruder.value,
                                                                m_object_config->support_material_interface_extruder
                                                                    .value))
            // Object is printed with the same extruder as the support.
            m_can_merge_support_regions = true;
    }
}

void
PrintObjectSupportMaterial::generate(PrintObject &object)
{
    printf("Samir has raft %d\n", m_support_params.has_raft());
    coordf_t max_object_layer_height = 0.;
    for (size_t i = 0; i < object.layer_count(); ++i)
        max_object_layer_height = max(max_object_layer_height, object.layers[i]->height);

    // Layer instances will be allocated by std::deque and they will be kept until the end of this function call.
    // The layers will be referenced by various LayersPtr (of type std::vector<Layer*>)
    MyLayerStorage layer_storage;
    printf("Samir 1\n");
    // Determine the top contact surfaces of the support, defined as:
    // contact = overhangs - clearance + margin
    // This method is responsible for identifying what contact surfaces
    // should the support material expose to the object in order to guarantee
    // that it will be effective, regardless of how it's built below.
    // If raft is to be generated, the 1st top_contact layer will contain the 1st object layer silhouette without holes.
    m_top_contacts = this->top_contact_layers(object, layer_storage);
    if (m_top_contacts.empty())
        // Nothing is supported, no supports are generated.
        return;
    printf("Samir 2\n");
    // Determine the bottom contact surfaces of the supports over the top surfaces of the object.
    // Depending on whether the support is soluble or not, the contact layer thickness is decided.
    // layer_support_areas contains the per object layer support areas. These per object layer support areas
    // may get merged and trimmed by this->generate_base_layers() if the support layers are not synchronized with object layers.
    std::vector<Polygons> layer_support_areas;
    m_bottom_contacts = this->bottom_contact_layers_and_layer_support_areas(
        object, m_top_contacts, layer_storage,
        layer_support_areas);

    // Allocate empty layers between the top / bottom support contact layers
    // as placeholders for the base and intermediate support layers.
    // The layers may or may not be synchronized with the object layers, depending on the configuration.
    // For example, a single nozzle multi material printing will need to generate a waste tower, which in turn
    // wastes less material, if there are as little tool changes as possible.
    m_intermediate_layers = this->raft_and_intermediate_support_layers(
        object, m_bottom_contacts, m_top_contacts, layer_storage);

    // TODO @Samir55
    this->trim_support_layers_by_object(object,
                                        m_top_contacts,
                                        m_support_params.soluble_interface ? 0. : m_support_layer_height_min,
                                        0.,
                                        m_gap_xy);

    // Fill in intermediate layers between the top / bottom support contact layers, trim them by the object.
    this->generate_base_layers(object, m_bottom_contacts, m_top_contacts, m_intermediate_layers, layer_support_areas);

    // TODO @Samir55
    // Because the top and bottom contacts are thick slabs, they may overlap causing over extrusion
    // and unwanted strong bonds to the object.
    // Rather trim the top contacts by their overlapping bottom contacts to leave a gap instead of over extruding
    // top contacts over the bottom contacts.
    this->trim_top_contacts_by_bottom_contacts(object, m_bottom_contacts, m_top_contacts);

    // Propagate top / bottom contact layers to generate interface layers.
    m_interface_layers = this->generate_interface_layers(
        m_bottom_contacts, m_top_contacts, m_intermediate_layers, layer_storage);

    // If raft is to be generated, the 1st top_contact layer will contain the 1st object layer silhouette with holes filled.
    // There is also a 1st intermediate layer containing bases of support columns.
    // Inflate the bases of the support columns and create the raft base under the object.
    m_raft_layers = this->generate_raft_base(m_top_contacts, m_interface_layers, m_intermediate_layers, layer_storage);


    // Install support layers into the object.
    // A support layer installed on a PrintObject has a unique print_z.
    m_layers_sorted.reserve(
        m_raft_layers.size() + m_bottom_contacts.size() + m_top_contacts.size() + m_intermediate_layers.size()
            + m_interface_layers.size());
    layers_append(m_layers_sorted, m_raft_layers);
    layers_append(m_layers_sorted, m_bottom_contacts);
    layers_append(m_layers_sorted, m_top_contacts);
    layers_append(m_layers_sorted, m_intermediate_layers);
    layers_append(m_layers_sorted, m_interface_layers);

    // Sort the layers lexicographically by a raising print_z and a decreasing height.
    std::sort(m_layers_sorted.begin(), m_layers_sorted.end(), MyLayersPtrCompare());

    assert(object.support_layers.empty());

    int layer_id = 0;
    for (int i = 0; i < int(m_layers_sorted.size());) {
        // Find the last layer with roughly the same print_z, find the minimum layer height of all.
        // Due to the floating point inaccuracies, the print_z may not be the same even if in theory they should.
        coordf_t z_max = m_layers_sorted[i]->print_z + EPSILON;

        // Assign an average print_z to the set of layers with nearly equal print_z.
        int j = i + 1;
        for (; j < m_layers_sorted.size() && m_layers_sorted[j]->print_z <= z_max; ++j);
        coordf_t z_avg = 0.5 * (m_layers_sorted[i]->print_z + m_layers_sorted[j - 1]->print_z);
        coordf_t height_min = m_layers_sorted[i]->height;

        bool empty = true;
        for (int u = i; u < j; ++u) {
            MyLayer &layer = *m_layers_sorted[u];

            if (!layer.polygons.empty())
                empty = false;

            layer.print_z = z_avg; // TODO @Samir55 Remove this comment the lower (start z)
            height_min = std::min(height_min, layer.height);
        }

        if (!empty) {
            // Here the upper_layer and lower_layer pointers are left to null at the support layers,
            // as they are never used. These pointers are candidates for removal.
            object.add_support_layer(layer_id++, height_min, z_avg);
        }
        i = j;
    }
    printf("Samir 5\n");
}

// Collect outer contours of all slices of this layer.
// This is useful for calculating the support base with holes filled.
Polygons collect_slices_outer(const Layer &layer)
{
    Polygons out;
    out.reserve(out.size() + layer.slices.expolygons.size());
    for (ExPolygons::const_iterator it = layer.slices.expolygons.begin(); it != layer.slices.expolygons.end(); ++it)
        out.push_back(it->contour);
    return out;
}

// Collect all polygons of all regions in a layer with a given surface type.
Polygons collect_region_slices_by_type(const Layer &layer, SurfaceType surface_type)
{
    // 1) Count the new polygons first.
    size_t n_polygons_new = 0;
    for (LayerRegionPtrs::const_iterator it_region = layer.regions.begin(); it_region != layer.regions.end(); ++ it_region) {
        const LayerRegion       &region = *(*it_region);
        const SurfaceCollection &slices = region.slices;
        for (Surfaces::const_iterator it = slices.surfaces.begin(); it != slices.surfaces.end(); ++ it) {
            const Surface &surface = *it;
            if (surface.surface_type == surface_type)
                n_polygons_new += surface.expolygon.holes.size() + 1;
        }
    }

    // 2) Collect the new polygons.
    Polygons out;
    out.reserve(n_polygons_new);
    for (LayerRegionPtrs::const_iterator it_region = layer.regions.begin(); it_region != layer.regions.end(); ++ it_region) {
        const LayerRegion       &region = *(*it_region);
        const SurfaceCollection &slices = region.slices;
        for (Surfaces::const_iterator it = slices.surfaces.begin(); it != slices.surfaces.end(); ++ it) {
            const Surface &surface = *it;
            if (surface.surface_type == surface_type)
                polygons_append(out, surface.expolygon);
        }
    }

    return out;
}

PrintObjectSupportMaterial::MyLayersPtr
PrintObjectSupportMaterial::top_contact_layers(const PrintObject &object,
                                               PrintObjectSupportMaterial::MyLayerStorage &layer_storage) const
{
    int i = 11;
    printf("Samir %d\n",i++);
    // Output layers, sorted by top Z.
    MyLayersPtr contact_out;

    // If user specified a custom angle threshold, convert it to radians.
    // Zero means automatic overhang detection.
    const double threshold_rad = (m_object_config->support_material_threshold.value > 0) ?
                                 M_PI * (m_object_config->support_material_threshold.value + 1) / 180.
                                                                                         : // +1 makes the threshold inclusive
                                 0.; //TODO @Samir55 Check.

    printf("Samir %d\n",i++);
    // Build support on a build plate only? If so, then collect and union all the surfaces below the current layer.
    // Unfortunately this is an inherently a sequential process.
    const bool build_plate_only = this->build_plate_only();
    std::vector<Polygons> build_plate_covered;

    if (build_plate_only) {
        build_plate_covered.assign(object.layers.size(), Polygons());
        for (size_t layer_id = 1; layer_id < object.layers.size(); ++layer_id) {
            const Layer &lower_layer = *object.layers[layer_id - 1];
            // Merge the new slices with the preceding slices.
            // Apply the safety offset to the newly added polygons, so they will connect
            // with the polygons collected before,
            // but don't apply the safety offset during the union operation as it would
            // inflate the polygons over and over. TODO @Samir55 Change this and see the differnece.
            Polygons &covered = build_plate_covered[layer_id];
            covered = build_plate_covered[layer_id - 1];
            polygons_append(covered, offset(lower_layer.slices.expolygons, scale_(0.01)));
            covered = union_(covered, false); // don't apply the safety offset.
        }
    }
    printf("Samir %d\n",i++);
    // Determine top contact areas.
    // If generating raft only (no support), only calculate top contact areas for the 0th layer.
    // If having a raft, start with 0th layer, otherwise with 1st layer.
    // Note that layer_id < layer->id when raft_layers > 0 as the layer->id incorporates the raft layers.
    // So layer_id == 0 means first object layer and layer->id == 0 means first print layer if there are no explicit raft layers.
    size_t num_layers = this->has_support() ? object.layer_count() : 1;
    contact_out.assign(num_layers, nullptr);
    tbb::spin_mutex layer_storage_mutex;
    tbb::parallel_for(tbb::blocked_range<size_t>(this->has_raft() ? 0 : 1, num_layers),
                      [this, &object, &build_plate_covered, threshold_rad, &layer_storage, &layer_storage_mutex, &contact_out](
                          const tbb::blocked_range<size_t> &range)
                      {
                          for (size_t layer_id = range.begin(); layer_id < range.end(); ++layer_id) {
                              const Layer &layer = *object.layers[layer_id];

                              // Detect overhangs and contact areas needed to support them.
                              // Collect overhangs and contacts of all regions of this layer supported by the layer immediately below.
                              Polygons overhang_polygons;
                              Polygons contact_polygons;
                              Polygons slices_margin_cached;
                              float slices_margin_cached_offset = -1.0;

                              if (layer_id == 0) {
                                  // This is the first object layer, so the object is being printed on a raft and
                                  // we're here just to get the object footprint for the raft.
                                  // We only consider contours and discard holes to get a more continuous raft.
                                  overhang_polygons = collect_slices_outer(layer);
                                  // Extend by SUPPORT_MATERIAL_MARGIN, which is 1.5mm
                                  // TODO @Samir55 ask, this the maximum allowed area for supports infill?
                                  contact_polygons = offset(overhang_polygons,
                                                            scale_(SUPPORT_MATERIAL_MARGIN),
                                                            ClipperLib::jtMiter,
                                                            3);
                              }
                              else {
                                  // Generate overhang / contact_polygons for non-raft layers.
                                  const Layer &lower_layer = *object.layers[layer_id - 1];
                                  for (LayerRegion *layerm : layer.regions) {
                                      // Extrusion width accounts for the rounding of the extrudates.
                                      // It is the maximum width of the extrudate.
                                      float fw = float(layerm->flow(frExternalPerimeter).scaled_width());
                                      float lower_layer_offset =
                                          (layer_id < this->m_object_config->support_material_enforce_layers.value) ?
                                          // Enforce a full possible support, ignore the overhang angle.
                                          0.0f :
                                          (threshold_rad > 0.0 ?
                                           // Overhang defined by an angle.
                                           float(scale_(lower_layer.height / tan(threshold_rad))) :
                                           // Overhang defined by half the extrusion width.
                                           0.5f * fw);
                                      // Overhang polygons for this layer and region.
                                      Polygons diff_polygons;
                                      Polygons layerm_polygons = to_polygons(layerm->slices);
                                      Polygons lower_layer_polygons = to_polygons(lower_layer.slices.expolygons);
                                      if (lower_layer_offset == 0.f) {
                                          // Support everything (the difference between this layer polygons and the lower layer polygon.
                                          diff_polygons = diff(layerm_polygons, lower_layer_polygons);
                                          if (!build_plate_covered.empty()) {
                                              // Don't support overhangs above the top surfaces.
                                              // This step is done before the contact surface is calculated by growing the overhang region.
                                              diff_polygons = diff(diff_polygons, build_plate_covered[layer_id]);
                                          }
                                      }
                                      else {
                                          // Get the regions needing a support, collapse very tiny spots.
                                          //FIXME cache the lower layer offset if this layer has multiple regions.
                                          diff_polygons = offset2(
                                              diff(layerm_polygons,
                                                   offset(lower_layer_polygons,
                                                          lower_layer_offset,
                                                          SUPPORT_SURFACES_OFFSET_PARAMETERS)),
                                              -0.1f * fw, +0.1f * fw);
                                          if (!build_plate_covered.empty()) {
                                              // Don't support overhangs above the top surfaces.
                                              // This step is done before the contact surface is calculated by growing the overhang region.
                                              diff_polygons = diff(diff_polygons, build_plate_covered[layer_id]);
                                          }
                                          if (diff_polygons.empty())
                                              continue;
                                          // Offset the support regions back to a full overhang, restrict them to the full overhang.
                                          diff_polygons = diff(
                                              intersection(offset(diff_polygons,
                                                                  lower_layer_offset,
                                                                  SUPPORT_SURFACES_OFFSET_PARAMETERS), layerm_polygons),
                                              lower_layer_polygons);
                                      }
                                      if (diff_polygons.empty())
                                          continue;

                                      if (this->m_object_config->dont_support_bridges) {
                                          // compute the area of bridging perimeters
                                          // Note: this is duplicate code from GCode.pm, we need to refactor
//                                          if (true) { // TODO @Samir55
                                          Polygons bridged_perimeters;
                                          {
                                              Flow bridge_flow = layerm->flow(frPerimeter, true);
                                              coordf_t nozzle_diameter = m_print_config->nozzle_diameter
                                                  .get_at(layerm->region()->config.perimeter_extruder - 1);
                                              Polygons lower_grown_slices = offset(lower_layer_polygons,
                                                                                   0.5f
                                                                                       * float(scale_(nozzle_diameter)),
                                                                                   SUPPORT_SURFACES_OFFSET_PARAMETERS);

                                              // Collect perimeters of this layer.
                                              // TODO: split_at_first_point() could split a bridge mid-way
                                              Polylines overhang_perimeters;
                                              for (ExtrusionEntity *extrusion_entity : layerm->perimeters.entities) {
                                                  const ExtrusionEntityCollection *island =
                                                      dynamic_cast<ExtrusionEntityCollection *>(extrusion_entity);
                                                  assert(island != NULL);
                                                  for (size_t i = 0; i < island->entities.size(); ++i) {
                                                      ExtrusionEntity *entity = island->entities[i];
                                                      ExtrusionLoop
                                                          *loop = dynamic_cast<Slic3r::ExtrusionLoop *>(entity);
                                                      overhang_perimeters.push_back(loop ?
                                                                                    loop->as_polyline() :
                                                                                    dynamic_cast<const Slic3r::ExtrusionPath *>(entity)
                                                                                        ->polyline);
                                                  }
                                              }

                                              // workaround for Clipper bug, see Slic3r::Polygon::clip_as_polyline()
                                              for (Polyline &polyline : overhang_perimeters)
                                                  polyline.points[0].x += 1;
                                              // Trim the perimeters of this layer by the lower layer to get the unsupported pieces of perimeters.
                                              overhang_perimeters = diff_pl(overhang_perimeters, lower_grown_slices);

                                              // only consider straight overhangs
                                              // only consider overhangs having endpoints inside layer's slices
                                              // convert bridging polylines into polygons by inflating them with their thickness
                                              // since we're dealing with bridges, we can't assume width is larger than spacing,
                                              // so we take the largest value and also apply safety offset to be ensure no gaps
                                              // are left in between
                                              float w = float(std::max(bridge_flow.scaled_width(),
                                                                       bridge_flow.scaled_spacing()));
                                              for (Polyline &polyline : overhang_perimeters)
                                                  if (polyline.is_straight()) {
                                                      // This is a bridge
                                                      polyline.extend_start(fw);
                                                      polyline.extend_end(fw);
                                                      // Is the straight perimeter segment supported at both sides?
                                                      if (layer.slices.contains(polyline.first_point())
                                                          && layer.slices.contains(polyline.last_point()))
                                                          // Offset a polyline into a thick line.
                                                          polygons_append(bridged_perimeters,
                                                                          offset(polyline, 0.5f * w + 10.f));
                                                  }
                                              bridged_perimeters = union_(bridged_perimeters);
                                          }
                                          // remove the entire bridges and only support the unsupported edges
                                          Polygons bridges;
                                          for (const Surface &surface : layerm->fill_surfaces.surfaces)
                                              if (surface.surface_type == stBottomBridge && surface.bridge_angle != -1)
                                                  polygons_append(bridges, surface.expolygon);
                                          diff_polygons = diff(diff_polygons, bridges, true);
                                          polygons_append(bridges, bridged_perimeters);
                                          polygons_append(diff_polygons,
                                                          intersection(
                                                              // Offset unsupported edges into polygons.
                                                              offset(layerm->unsupported_bridge_edges.polylines,
                                                                     scale_(SUPPORT_MATERIAL_MARGIN),
                                                                     ClipperLib::jtSquare,
                                                                     SUPPORT_SURFACES_OFFSET_PARAMETERS),
                                                              bridges));
//                                          } else {
                                          // just remove bridged areas
//                                              diff_polygons = diff(diff_polygons, layerm->bridged, true);
//                                          }
                                      } // if (m_objconfig->dont_support_bridges)

                                      if (diff_polygons.empty())
                                          continue;

                                      if (this->has_contact_loops())
                                          polygons_append(overhang_polygons, diff_polygons);

                                      // Let's define the required contact area by using a max gap of half the upper
                                      // extrusion width and extending the area according to the configured margin.
                                      // We increment the area in steps because we don't want our support to overflow
                                      // on the other side of the object (if it's very thin).
                                      {
                                          //FIMXE 1) Make the offset configurable, 2) Make the Z span configurable.
                                          float slices_margin_offset =
                                              std::min(lower_layer_offset, float(scale_(m_gap_xy)));
                                          if (slices_margin_cached_offset != slices_margin_offset) {
                                              slices_margin_cached_offset = slices_margin_offset;
                                              slices_margin_cached = (slices_margin_offset == 0.f) ?
                                                                     to_polygons(lower_layer.slices.expolygons) :
                                                                     offset(lower_layer.slices.expolygons,
                                                                            slices_margin_offset,
                                                                            SUPPORT_SURFACES_OFFSET_PARAMETERS);
                                              if (!build_plate_covered.empty()) {
                                                  // Trim the inflated contact surfaces by the top surfaces as well.
                                                  polygons_append(slices_margin_cached, build_plate_covered[layer_id]);
                                                  slices_margin_cached = union_(slices_margin_cached);
                                              }
                                          }
                                          // Offset the contact polygons outside.
                                          for (size_t i = 0; i < NUM_MARGIN_STEPS; ++i) {
                                              diff_polygons = diff(
                                                  offset(
                                                      diff_polygons,
                                                      SUPPORT_MATERIAL_MARGIN / NUM_MARGIN_STEPS,
                                                      ClipperLib::jtRound,
                                                      // round mitter limit
                                                      scale_(0.05)),
                                                  slices_margin_cached);
                                          }
                                      }
                                      polygons_append(contact_polygons, diff_polygons);
                                  } // for each layer.region
                              } // end of Generate overhang/contact_polygons for non-raft layers.

//                              // now apply the contact areas to the layer were they need to be made
                              if (!contact_polygons.empty()) {
                                  // get the average nozzle diameter used on this layer
                                  MyLayer
                                      &new_layer = layer_allocate(layer_storage, layer_storage_mutex, sltTopContact);
                                  new_layer.idx_object_layer_above = layer_id;
                                  if (m_support_params.soluble_interface) {
                                      // Align the contact surface height with a layer immediately below the supported layer.
                                      new_layer.print_z = layer.print_z - layer.height;
                                      if (layer_id == 0) {
                                          // This is a raft contact layer sitting directly on the print bed.
                                          new_layer.height = m_support_params.contact_raft_layer_height;
                                          new_layer.bottom_z = m_support_params.raft_interface_top_z;
                                      }
                                      else {
                                          // Interface layer will be synchronized with the object.
                                          assert(layer_id > 0);
                                          new_layer.height = object.layers[layer_id - 1]->height;
                                          new_layer.bottom_z =
                                              (layer_id == 1) ? m_support_params.object_print_z_min : object.layers[
                                                  layer_id - 2]->print_z;
                                      }
                                  }
                                  else {
                                      // Contact layer will be printed with a normal flow, but
                                      // it will support layers printed with a bridging flow.
                                      //FIXME Probably printing with the bridge flow? How about the unsupported perimeters? Are they printed with the bridging flow?
                                      // In the future we may switch to a normal extrusion flow for the supported bridges.
                                      // Get the average nozzle diameter used on this layer.
                                      coordf_t nozzle_dmr = 0.;
                                      for (const LayerRegion *region : layer.regions)
                                          nozzle_dmr += region->region()->nozzle_dmr_avg(*m_print_config);
                                      nozzle_dmr /= coordf_t(layer.regions.size());
                                      new_layer.print_z = layer.print_z - nozzle_dmr
                                          - m_object_config->support_material_contact_distance;
                                      new_layer.bottom_z = new_layer.print_z;
                                      new_layer.height = 0.;
                                      if (layer_id == 0) {
                                          // This is a raft contact layer sitting directly on the print bed.
                                          assert(this->has_raft());
                                          new_layer.bottom_z = m_support_params.raft_interface_top_z;
                                          new_layer.height = m_support_params.contact_raft_layer_height;
                                      }
                                      else {
                                          // Ignore this contact area if it's too low.
                                          // Don't want to print a layer below the first layer height as it may not stick well.
                                          //FIXME there may be a need for a single layer support, then one may decide to print it either as a bottom contact or a top contact
                                          // and it may actually make sense to do it with a thinner layer than the first layer height.
                                          if (new_layer.print_z < m_support_params.first_print_layer_height - EPSILON) {
                                              // This contact layer is below the first layer height, therefore not printable. Don't support this surface.
                                              continue;
                                          }
                                          else if (new_layer.print_z
                                              < m_support_params.first_print_layer_height + EPSILON) {
                                              // Align the layer with the 1st layer height.
                                              new_layer.print_z = m_support_params.first_print_layer_height;
                                              new_layer.bottom_z = 0;
                                              new_layer.height = m_support_params.first_print_layer_height;
                                          }
                                          else {
                                              // Don't know the height of the top contact layer yet. The top contact layer is printed with a normal flow and
                                              // its height will be set adaptively later on.
                                          }
                                      }
                                  }
//
                                  SupportGridPattern support_grid_pattern(
                                      // Support islands, to be stretched into a grid.
                                      contact_polygons,
                                      // Trimming polygons, to trim the stretched support islands.
                                      slices_margin_cached,
                                      // How much to offset the extracted contour outside of the grid.
                                      m_object_config->support_material_spacing.value
                                          + m_support_material_flow.spacing(),
                                      Geometry::deg2rad(m_object_config->support_material_angle.value));
                                  // 1) infill polygons, expand them by half the extrusion width + a tiny bit of extra.
                                  new_layer.polygons = support_grid_pattern
                                      .extract_support(m_support_material_flow.scaled_spacing() / 2 + 5);
                                  // 2) Contact polygons will be projected down. To keep the interface and base layers to grow, return a contour a tiny bit smaller than the grid cells.
                                  new_layer.contact_polygons = new Polygons(support_grid_pattern.extract_support(-3));
//
                                  // Even after the contact layer was expanded into a grid, some of the contact islands may be too tiny to be extruded.
                                  // Remove those tiny islands from new_layer.polygons and new_layer.contact_polygons.
//
                                  // Store the overhang polygons.
                                  // The overhang polygons are used in the path generator for planning of the contact loops.
                                  // if (this->has_contact_loops())
                                  new_layer.overhang_polygons = new Polygons(std::move(overhang_polygons));
                                  cout << "SIZE " << contact_out.size() << endl;
                                  cout << "overhangs " << new_layer.contact_polygons->size() << endl;
                                  cout << "SIZE " << new_layer.overhang_polygons->size() << endl;
                                  contact_out[layer_id] = &new_layer;
                              }
                          }
                      });
    printf("Samir %d\n",i++);
    // Compress contact_out, remove the nullptr items.
    remove_nulls(contact_out);
    printf("Samir 6\n");
    return contact_out;
}

PrintObjectSupportMaterial::MyLayersPtr
PrintObjectSupportMaterial::bottom_contact_layers_and_layer_support_areas(const PrintObject &object,
                                                                          const PrintObjectSupportMaterial::MyLayersPtr &top_contacts,
                                                                          PrintObjectSupportMaterial::MyLayerStorage &layer_storage,
                                                                          std::vector<Polygons> &layer_support_areas) const
{
    // Allocate empty surface areas, one per object layer.
    layer_support_areas.assign(object.total_layer_count(), Polygons());

    // find object top surfaces
    // we'll use them to clip our support and detect where does it stick
    MyLayersPtr bottom_contacts;

    if (!top_contacts.empty()) {
        // There is some support to be built, if there are non-empty top surfaces detected.
        // Sum of unsupported contact areas above the current layer.print_z.
        Polygons projection;
        // Last top contact layer visited when collecting the projection of contact areas.
        int contact_idx = int(top_contacts.size()) - 1;
        for (int layer_id = int(object.total_layer_count()) - 2; layer_id >= 0; --layer_id) {
            const Layer &layer = *object.get_layer(layer_id);
            // Collect projections of all contact areas above or at the same level as this top surface.
            for (; contact_idx >= 0 && top_contacts[contact_idx]->print_z >= layer.print_z; --contact_idx) {
                Polygons polygons_new;
                // Contact surfaces are expanded away from the object, trimmed by the object.
                // Use a slight positive offset to overlap the touching regions.
#if 0
                // Merge and collect the contact polygons. The contact polygons are inflated, but not extended into a grid form.
                polygons_append(polygons_new, offset(*top_contacts[contact_idx]->contact_polygons, SCALED_EPSILON));
#else
                // Consume the contact_polygons. The contact polygons are already expanded into a grid form.
                polygons_append(polygons_new, std::move(*top_contacts[contact_idx]->contact_polygons));
#endif
                // These are the overhang surfaces. They are touching the object and they are not expanded away from the object.
                // Use a slight positive offset to overlap the touching regions.
                polygons_append(polygons_new,
                                offset(*top_contacts[contact_idx]->overhang_polygons, float(SCALED_EPSILON)));
                polygons_append(projection, union_(polygons_new));
            }
            if (projection.empty())
                continue;
            Polygons projection_raw = union_(projection);

            // Top surfaces of this layer, to be used to stop the surface volume from growing down.
            tbb::task_group task_group;
            if (!m_object_config->support_material_buildplate_only)
                task_group
                    .run([this, &object, &top_contacts, contact_idx, &layer, layer_id, &layer_storage, &layer_support_areas, &bottom_contacts, &projection_raw]
                         {
                             Polygons top = collect_region_slices_by_type(layer, stTop);

                             // Now find whether any projection of the contact surfaces above layer.print_z not yet supported by any
                             // top surfaces above layer.print_z falls onto this top surface.
                             // Touching are the contact surfaces supported exclusively by this top surfaces.
                             // Don't use a safety offset as it has been applied during insertion of polygons.
                             if (!top.empty()) {
                                 Polygons touching = intersection(top, projection_raw, false);
                                 if (!touching.empty()) {
                                     // Allocate a new bottom contact layer.
                                     MyLayer &layer_new = layer_allocate(layer_storage, sltBottomContact);
                                     bottom_contacts.push_back(&layer_new);
                                     // Grow top surfaces so that interface and support generation are generated
                                     // with some spacing from object - it looks we don't need the actual
                                     // top shapes so this can be done here
                                     layer_new.height = m_support_params.soluble_interface ?
                                                        // Align the interface layer with the object's layer height.
                                                        object.layers[layer_id + 1]->height :
                                                        // Place a bridge flow interface layer over the top surface.
                                                        m_support_material_interface_flow.nozzle_diameter;
                                     layer_new.print_z =
                                         m_support_params.soluble_interface ? object.layers[layer_id + 1]->print_z :
                                         layer.print_z + layer_new.height
                                             + m_object_config->support_material_contact_distance.value;
                                     layer_new.bottom_z = layer.print_z;
                                     layer_new.idx_object_layer_below = layer_id;
                                     layer_new.bridging = !m_support_params.soluble_interface;
                                     //FIXME how much to inflate the top surface?
                                     layer_new.polygons = offset(touching,
                                                                 float(m_support_material_flow.scaled_width()),
                                                                 SUPPORT_SURFACES_OFFSET_PARAMETERS);
                                     if (!m_support_params.soluble_interface) {
                                         // Walk the top surfaces, snap the top of the new bottom surface to the closest top of the top surface,
                                         // so there will be no support surfaces generated with thickness lower than m_support_layer_height_min.
                                         for (size_t top_idx = size_t(std::max<int>(0, contact_idx));
                                              top_idx < top_contacts.size() && top_contacts[top_idx]->print_z
                                                  < layer_new.print_z + this->m_support_layer_height_min;
                                              ++top_idx) {
                                             if (top_contacts[top_idx]->print_z
                                                 > layer_new.print_z - this->m_support_layer_height_min) {
                                                 // A top layer has been found, which is close to the new bottom layer.
                                                 coordf_t diff = layer_new.print_z - top_contacts[top_idx]->print_z;
                                                 assert(std::abs(diff) <= this->m_support_layer_height_min);
                                                 if (diff > 0.) {
                                                     // The top contact layer is below this layer. Make the bridging layer thinner to align with the existing top layer.
                                                     assert(diff < layer_new.height + EPSILON);
                                                     assert(layer_new.height - diff
                                                                >= this->m_support_layer_height_min - EPSILON);
                                                     layer_new.print_z = top_contacts[top_idx]->print_z;
                                                     layer_new.height -= diff;
                                                 }
                                                 else {
                                                     // The top contact layer is above this layer. One may either make this layer thicker or thinner.
                                                     // By making the layer thicker, one will decrease the number of discrete layers with the price of extruding a bit too thick bridges.
                                                     // By making the layer thinner, one adds one more discrete layer.
                                                     layer_new.print_z = top_contacts[top_idx]->print_z;
                                                     layer_new.height -= diff;
                                                 }
                                                 break;
                                             }
                                         }
                                     }

                                     // Trim the already created base layers above the current layer intersecting with the new bottom contacts layer.
                                     touching = offset(touching, float(SCALED_EPSILON));
                                     for (int layer_id_above = layer_id + 1;
                                          layer_id_above < int(object.total_layer_count()); ++layer_id_above) {
                                         const Layer &layer_above = *object.layers[layer_id_above];
                                         if (layer_above.print_z > layer_new.print_z + EPSILON)
                                             break;
                                         if (!layer_support_areas[layer_id_above].empty()) {
                                             layer_support_areas[layer_id_above] =
                                                 diff(layer_support_areas[layer_id_above], touching);
                                         }
                                     }
                                 }
                             } // ! top.empty()
                         });

            Polygons &layer_support_area = layer_support_areas[layer_id];
            task_group.run([this, &projection, &projection_raw, &layer, &layer_support_area, layer_id]
                           {
                               // Remove the areas that touched from the projection that will continue on next, lower, top surfaces.
                               //            Polygons trimming = union_(to_polygons(layer.slices.expolygons), touching, true);
                               Polygons trimming = offset(layer.slices.expolygons, float(SCALED_EPSILON));
                               projection = diff(projection_raw, trimming, false);
                               remove_sticks(projection);
                               remove_degenerate(projection);
                               SupportGridPattern support_grid_pattern(
                                   // Support islands, to be stretched into a grid.
                                   projection,
                                   // Trimming polygons, to trim the stretched support islands.
                                   trimming,
                                   // How much to offset the extracted contour outside of the grid.
                                   m_object_config->support_material_spacing.value + m_support_material_flow.spacing(),
                                   Geometry::deg2rad(m_object_config->support_material_angle.value));
                               tbb::task_group task_group_inner;
                               // 1) Cache the slice of a support volume. The support volume is expanded by 1/2 of support material flow spacing
                               // to allow a placement of suppot zig-zag snake along the grid lines.
                               task_group_inner.run([this, &support_grid_pattern, &layer_support_area

                                                    ]
                                                    {
                                                        layer_support_area = support_grid_pattern.extract_support(
                                                            m_support_material_flow.scaled_spacing() / 2 + 25);

                                                    });
                               // 2) Support polygons will be projected down. To keep the interface and base layers from growing, return a contour a tiny bit smaller than the grid cells.
                               Polygons projection_new;
                               task_group_inner.run([&projection_new, &support_grid_pattern

                                                    ]
                                                    {
                                                        projection_new = support_grid_pattern.extract_support(-5);

                                                    });
                               task_group_inner.wait();
                               projection = std::move(projection_new);
                           });
            task_group.wait();
        }
        std::reverse(bottom_contacts.begin(), bottom_contacts.end());
        trim_support_layers_by_object(object,
                                      bottom_contacts,
                                      m_support_params.soluble_interface ? 0. : m_support_layer_height_min,
                                      0.,
                                      m_gap_xy);
    } // ! top_contacts.empty()

    return bottom_contacts;
}

void
PrintObjectSupportMaterial::trim_top_contacts_by_bottom_contacts(const PrintObject &object,
                                                                 const PrintObjectSupportMaterial::MyLayersPtr &bottom_contacts,
                                                                 PrintObjectSupportMaterial::MyLayersPtr &top_contacts) const
{
    tbb::parallel_for(tbb::blocked_range<int>(0, int(top_contacts.size())),
                      [this, &object, &bottom_contacts, &top_contacts](const tbb::blocked_range<int>& range) {
                          int idx_bottom_overlapping_first = -2;
                          // For all top contact layers, counting downwards due to the way idx_higher_or_equal caches the last index to avoid repeated binary search.
                          for (int idx_top = range.end() - 1; idx_top >= range.begin(); -- idx_top) {
                              MyLayer &layer_top = *top_contacts[idx_top];
                              // Find the first bottom layer overlapping with layer_top.
                              idx_bottom_overlapping_first = idx_lower_or_equal(bottom_contacts, idx_bottom_overlapping_first, [&layer_top](const MyLayer *layer_bottom){ return layer_bottom->bottom_print_z() - EPSILON <= layer_top.bottom_z; });
                              // For all top contact layers overlapping with the thick bottom contact layer:
                              for (int idx_bottom_overlapping = idx_bottom_overlapping_first; idx_bottom_overlapping >= 0; -- idx_bottom_overlapping) {
                                  const MyLayer &layer_bottom = *bottom_contacts[idx_bottom_overlapping];
                                  assert(layer_bottom.bottom_print_z() - EPSILON <= layer_top.bottom_z);
                                  if (layer_top.print_z < layer_bottom.print_z + EPSILON) {
                                      // Layers overlap. Trim layer_top with layer_bottom.
                                      layer_top.polygons = diff(layer_top.polygons, layer_bottom.polygons);
                                  } else
                                      break;
                              }
                          }
                      });
}

PrintObjectSupportMaterial::MyLayersPtr
PrintObjectSupportMaterial::raft_and_intermediate_support_layers(const PrintObject &object,
                                                                 const PrintObjectSupportMaterial::MyLayersPtr &bottom_contacts,
                                                                 const PrintObjectSupportMaterial::MyLayersPtr &top_contacts,
                                                                 PrintObjectSupportMaterial::MyLayerStorage &layer_storage) const
{
    MyLayersPtr intermediate_layers;

    // Collect and sort the extremes (bottoms of the top contacts and tops of the bottom contacts).
    MyLayersPtr extremes;
    extremes.reserve(top_contacts.size() + bottom_contacts.size());
    for (size_t i = 0; i < top_contacts.size(); ++ i)
        // Bottoms of the top contact layers. In case of non-soluble supports,
        // the top contact layer thickness is not known yet.
        extremes.push_back(top_contacts[i]);
    for (size_t i = 0; i < bottom_contacts.size(); ++ i)
        // Tops of the bottom contact layers.
        extremes.push_back(bottom_contacts[i]);
    if (extremes.empty())
        return intermediate_layers;

    auto layer_extreme_lower = [](const MyLayer *l1, const MyLayer *l2) {
        coordf_t z1 = l1->extreme_z();
        coordf_t z2 = l2->extreme_z();
        // If the layers are aligned, return the top contact surface first.
        return z1 < z2 || (z1 == z2 && l1->layer_type == PrintObjectSupportMaterial::sltTopContact && l2->layer_type == PrintObjectSupportMaterial::sltBottomContact);
    };
    std::sort(extremes.begin(), extremes.end(), layer_extreme_lower);

    assert(extremes.empty() ||
        (extremes.front()->extreme_z() > m_support_params.raft_interface_top_z - EPSILON &&
            (m_support_params.raft_layers() == 1 || // only raft contact layer
                extremes.front()->layer_type == sltTopContact || // first extreme is a top contact layer
                extremes.front()->extreme_z() > m_support_params.first_print_layer_height - EPSILON)));

    bool synchronize = this->synchronize_layers();

    // Generate intermediate layers.
    // The first intermediate layer is the same as the 1st layer if there is no raft,
    // or the bottom of the first intermediate layer is aligned with the bottom of the raft contact layer.
    // Intermediate layers are always printed with a normal extrusion flow (non-bridging).
    size_t idx_layer_object = 0;
    for (size_t idx_extreme = 0; idx_extreme < extremes.size(); ++ idx_extreme) {
        MyLayer      *extr2  = extremes[idx_extreme];
        coordf_t      extr2z = extr2->extreme_z();
        if (std::abs(extr2z - m_support_params.raft_interface_top_z) < EPSILON) {
            // This is a raft contact layer, its height has been decided in this->top_contact_layers().
            assert(extr2->layer_type == sltTopContact);
            continue;
        }
        if (std::abs(extr2z - m_support_params.first_print_layer_height) < EPSILON) {
            // This is a bottom of a synchronized (or soluble) top contact layer, its height has been decided in this->top_contact_layers().
            assert(extr2->layer_type == sltTopContact);
            assert(extr2->bottom_z == m_support_params.first_print_layer_height);
            assert(extr2->print_z >= m_support_params.first_print_layer_height + this->m_support_layer_height_min - EPSILON);
            if (intermediate_layers.empty() || intermediate_layers.back()->print_z < m_support_params.first_print_layer_height) {
                MyLayer &layer_new = layer_allocate(layer_storage, sltIntermediate);
                layer_new.bottom_z = 0.;
                layer_new.print_z  = m_support_params.first_print_layer_height;
                layer_new.height   = m_support_params.first_print_layer_height;
                intermediate_layers.push_back(&layer_new);
            }
            continue;
        }
        assert(extr2z >= m_support_params.raft_interface_top_z + EPSILON);
        assert(extr2z >= m_support_params.first_print_layer_height + EPSILON);
        MyLayer      *extr1  = (idx_extreme == 0) ? nullptr : extremes[idx_extreme - 1];
        // Fuse a support layer firmly to the raft top interface (not to the raft contacts).
        coordf_t      extr1z = (extr1 == nullptr) ? m_support_params.raft_interface_top_z : extr1->extreme_z();
        assert(extr2z >= extr1z);
        assert(extr2z > extr1z || (extr1 != nullptr && extr2->layer_type == sltBottomContact));
        if (std::abs(extr1z) < EPSILON) {
            // This layer interval starts with the 1st layer. Print the 1st layer using the prescribed 1st layer thickness.
            assert(! m_support_params.has_raft());
            assert(intermediate_layers.empty() || intermediate_layers.back()->print_z <= m_support_params.first_print_layer_height);
            // At this point only layers above first_print_layer_heigth + EPSILON are expected as the other cases were captured earlier.
            assert(extr2z >= m_support_params.first_print_layer_height + EPSILON);
            // Generate a new intermediate layer.
            MyLayer &layer_new = layer_allocate(layer_storage, sltIntermediate);
            layer_new.bottom_z = 0.;
            layer_new.print_z  = extr1z = m_support_params.first_print_layer_height;
            layer_new.height   = extr1z;
            intermediate_layers.push_back(&layer_new);
            // Continue printing the other layers up to extr2z.
        }
        coordf_t      dist   = extr2z - extr1z;
        assert(dist >= 0.);
        if (dist == 0.)
            continue;
        // The new layers shall be at least m_support_layer_height_min thick.
        assert(dist >= m_support_layer_height_min - EPSILON);
        if (synchronize) {
            // Emit support layers synchronized with the object layers.
            // Find the first object layer, which has its print_z in this support Z range.
            while (idx_layer_object < object.layers.size() && object.layers[idx_layer_object]->print_z < extr1z + EPSILON)
                ++ idx_layer_object;
            if (idx_layer_object == 0 && extr1z == m_support_params.raft_interface_top_z) {
                // Insert one base support layer below the object.
                MyLayer &layer_new = layer_allocate(layer_storage, sltIntermediate);
                layer_new.print_z  = m_support_params.object_print_z_min;
                layer_new.bottom_z = m_support_params.raft_interface_top_z;
                layer_new.height   = layer_new.print_z - layer_new.bottom_z;
                intermediate_layers.push_back(&layer_new);
            }
            // Emit all intermediate support layers synchronized with object layers up to extr2z.
            for (; idx_layer_object < object.layers.size() && object.layers[idx_layer_object]->print_z < extr2z + EPSILON; ++ idx_layer_object) {
                MyLayer &layer_new = layer_allocate(layer_storage, sltIntermediate);
                layer_new.print_z  = object.layers[idx_layer_object]->print_z;
                layer_new.height   = object.layers[idx_layer_object]->height;
                layer_new.bottom_z = (idx_layer_object > 0) ? object.layers[idx_layer_object - 1]->print_z : (layer_new.print_z - layer_new.height);
                assert(intermediate_layers.empty() || intermediate_layers.back()->print_z < layer_new.print_z + EPSILON);
                intermediate_layers.push_back(&layer_new);
            }
        } else {
            // Insert intermediate layers.
            size_t        n_layers_extra = size_t(ceil(dist / m_support_params.max_suport_layer_height));
            assert(n_layers_extra > 0);
            coordf_t      step   = dist / coordf_t(n_layers_extra);
            if (extr1 != nullptr && extr1->layer_type == sltTopContact &&
                extr1->print_z + this->m_support_layer_height_min > extr1->bottom_z + step) {
                // The bottom extreme is a bottom of a top surface. Ensure that the gap
                // between the 1st intermediate layer print_z and extr1->print_z is not too small.
                assert(extr1->bottom_z + this->m_support_layer_height_min < extr1->print_z + EPSILON);
                // Generate the first intermediate layer.
                MyLayer &layer_new = layer_allocate(layer_storage, sltIntermediate);
                layer_new.bottom_z = extr1->bottom_z;
                layer_new.print_z  = extr1z = extr1->print_z;
                layer_new.height   = extr1->height;
                intermediate_layers.push_back(&layer_new);
                dist = extr2z - extr1z;
                n_layers_extra = size_t(ceil(dist / m_support_params.max_suport_layer_height));
                if (n_layers_extra == 0)
                    continue;
                // Continue printing the other layers up to extr2z.
                step = dist / coordf_t(n_layers_extra);
            }
            if (! m_support_params.soluble_interface && extr2->layer_type == sltTopContact) {
                // This is a top interface layer, which does not have a height assigned yet. Do it now.
                assert(extr2->height == 0.);
                assert(extr1z > m_support_params.first_print_layer_height - EPSILON);
                extr2->height = step;
                extr2->bottom_z = extr2z = extr2->print_z - step;
                if (-- n_layers_extra == 0)
                    continue;
            }
            coordf_t extr2z_large_steps = extr2z;
            // Take the largest allowed step in the Z axis until extr2z_large_steps is reached.
            for (size_t i = 0; i < n_layers_extra; ++ i) {
                MyLayer &layer_new = layer_allocate(layer_storage, sltIntermediate);
                if (i + 1 == n_layers_extra) {
                    // Last intermediate layer added. Align the last entered layer with extr2z_large_steps exactly.
                    layer_new.bottom_z = (i == 0) ? extr1z : intermediate_layers.back()->print_z;
                    layer_new.print_z = extr2z_large_steps;
                    layer_new.height = layer_new.print_z - layer_new.bottom_z;
                }
                else {
                    // Intermediate layer, not the last added.
                    layer_new.height = step;
                    layer_new.bottom_z = extr1z + i * step;
                    layer_new.print_z = layer_new.bottom_z + step;
                }
                assert(intermediate_layers.empty() || intermediate_layers.back()->print_z <= layer_new.print_z);
                intermediate_layers.push_back(&layer_new);
            }
        }
    }

    return intermediate_layers;
}

void
PrintObjectSupportMaterial::generate_base_layers(const PrintObject &object,
                                                 const PrintObjectSupportMaterial::MyLayersPtr &bottom_contacts,
                                                 const PrintObjectSupportMaterial::MyLayersPtr &top_contacts,
                                                 PrintObjectSupportMaterial::MyLayersPtr &intermediate_layers,
                                                 const std::vector<Polygons> &layer_support_areas) const
{
    if (top_contacts.empty())
        // No top contacts -> no intermediate layers will be produced.
        return;

    // coordf_t fillet_radius_scaled = scale_(m_object_config->support_material_spacing);

    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, intermediate_layers.size()),
        [this, &object, &bottom_contacts, &top_contacts, &intermediate_layers, &layer_support_areas](const tbb::blocked_range<size_t>& range) {
            // index -2 means not initialized yet, -1 means intialized and decremented to 0 and then -1.
            int idx_top_contact_above           = -2;
            int idx_bottom_contact_overlapping  = -2;
            int idx_object_layer_above          = -2;
            // Counting down due to the way idx_lower_or_equal caches indices to avoid repeated binary search over the complete sequence.
            for (int idx_intermediate = int(range.end()) - 1; idx_intermediate >= int(range.begin()); -- idx_intermediate)
            {
                MyLayer &layer_intermediate = *intermediate_layers[idx_intermediate];
                // Layers must be sorted by print_z.
                assert(idx_intermediate == 0 || layer_intermediate.print_z >= intermediate_layers[idx_intermediate - 1]->print_z);

                // Find a top_contact layer touching the layer_intermediate from above, if any, and collect its polygons into polygons_new.
                idx_top_contact_above = idx_lower_or_equal(top_contacts, idx_top_contact_above,
                                                           [&layer_intermediate](const MyLayer *layer){ return layer->bottom_z <= layer_intermediate.print_z - EPSILON; });

                // New polygons for layer_intermediate.
                Polygons polygons_new;

                // Use the precomputed layer_support_areas.
                idx_object_layer_above = std::max(0, idx_lower_or_equal(object.layers, idx_object_layer_above,
                                                                        [&layer_intermediate](const Layer *layer){ return layer->print_z <= layer_intermediate.print_z + EPSILON; }));
                polygons_new = layer_support_areas[idx_object_layer_above];

                // Polygons to trim polygons_new.
                Polygons polygons_trimming;

                // Trimming the base layer with any overlapping top layer.
                // Following cases are recognized:
                // 1) top.bottom_z >= base.top_z -> No overlap, no trimming needed.
                // 2) base.bottom_z >= top.print_z -> No overlap, no trimming needed.
                // 3) base.print_z > top.print_z  && base.bottom_z >= top.bottom_z -> Overlap, which will be solved inside generate_toolpaths() by reducing the base layer height where it overlaps the top layer. No trimming needed here.
                // 4) base.print_z > top.bottom_z && base.bottom_z < top.bottom_z -> Base overlaps with top.bottom_z. This must not happen.
                // 5) base.print_z <= top.print_z  && base.bottom_z >= top.bottom_z -> Base is fully inside top. Trim base by top.
                int idx_top_contact_overlapping = idx_top_contact_above;
                while (idx_top_contact_overlapping >= 0 &&
                    top_contacts[idx_top_contact_overlapping]->bottom_z > layer_intermediate.print_z - EPSILON)
                    -- idx_top_contact_overlapping;
                // Collect all the top_contact layer intersecting with this layer.
                for (; idx_top_contact_overlapping >= 0; -- idx_top_contact_overlapping) {
                    MyLayer &layer_top_overlapping = *top_contacts[idx_top_contact_overlapping];
                    if (layer_top_overlapping.print_z < layer_intermediate.bottom_z + EPSILON)
                        break;
                    // Base must not overlap with top.bottom_z.
                    assert(! (layer_intermediate.print_z > layer_top_overlapping.bottom_z + EPSILON && layer_intermediate.bottom_z < layer_top_overlapping.bottom_z - EPSILON));
                    if (layer_intermediate.print_z <= layer_top_overlapping.print_z + EPSILON && layer_intermediate.bottom_z >= layer_top_overlapping.bottom_z - EPSILON)
                        // Base is fully inside top. Trim base by top.
                        polygons_append(polygons_trimming, layer_top_overlapping.polygons);
                }

                // Trimming the base layer with any overlapping bottom layer.
                // Following cases are recognized:
                // 1) bottom.bottom_z >= base.top_z -> No overlap, no trimming needed.
                // 2) base.bottom_z >= bottom.print_z -> No overlap, no trimming needed.
                // 3) base.print_z > bottom.bottom_z && base.bottom_z < bottom.bottom_z -> Overlap, which will be solved inside generate_toolpaths() by reducing the bottom layer height where it overlaps the base layer. No trimming needed here.
                // 4) base.print_z > bottom.print_z  && base.bottom_z >= bottom.print_z -> Base overlaps with bottom.print_z. This must not happen.
                // 5) base.print_z <= bottom.print_z && base.bottom_z >= bottom.bottom_z -> Base is fully inside top. Trim base by top.
                idx_bottom_contact_overlapping = idx_lower_or_equal(bottom_contacts, idx_bottom_contact_overlapping,
                                                                    [&layer_intermediate](const MyLayer *layer){ return layer->bottom_print_z() <= layer_intermediate.print_z - EPSILON; });
                // Collect all the bottom_contacts layer intersecting with this layer.
                for (int i = idx_bottom_contact_overlapping; i >= 0; -- i) {
                    MyLayer &layer_bottom_overlapping = *bottom_contacts[i];
                    if (layer_bottom_overlapping.print_z < layer_intermediate.bottom_print_z() + EPSILON)
                        break;
                    // Base must not overlap with bottom.top_z.
                    assert(! (layer_intermediate.print_z > layer_bottom_overlapping.print_z + EPSILON && layer_intermediate.bottom_z < layer_bottom_overlapping.print_z - EPSILON));
                    if (layer_intermediate.print_z <= layer_bottom_overlapping.print_z + EPSILON && layer_intermediate.bottom_z >= layer_bottom_overlapping.bottom_print_z() - EPSILON)
                        // Base is fully inside bottom. Trim base by bottom.
                        polygons_append(polygons_trimming, layer_bottom_overlapping.polygons);
                }

                // Trim the polygons, store them.
                if (polygons_trimming.empty())
                    layer_intermediate.polygons = std::move(polygons_new);
                else
                    layer_intermediate.polygons = diff(
                        polygons_new,
                        polygons_trimming,
                        true); // safety offset to merge the touching source polygons
                layer_intermediate.layer_type = sltBase;

#if 0
                // Fillet the base polygons and trim them again with the top, interface and contact layers.
                    $base->{$i} = diff(
                        offset2(
                            $base->{$i},
                            $fillet_radius_scaled,
                            -$fillet_radius_scaled,
                            # Use a geometric offsetting for filleting.
                            JT_ROUND,
                            0.2*$fillet_radius_scaled),
                        $trim_polygons,
                        false); // don't apply the safety offset.
                }
#endif
            }
        });
    trim_support_layers_by_object(object, intermediate_layers,  m_support_params.soluble_interface ? 0. : m_support_layer_height_min,  m_support_params.soluble_interface ? 0. : m_support_layer_height_min, m_gap_xy);
}

PrintObjectSupportMaterial::MyLayersPtr
PrintObjectSupportMaterial::generate_raft_base(const PrintObjectSupportMaterial::MyLayersPtr &top_contacts,
                                               const PrintObjectSupportMaterial::MyLayersPtr &interface_layers,
                                               const PrintObjectSupportMaterial::MyLayersPtr &base_layers,
                                               PrintObjectSupportMaterial::MyLayerStorage &layer_storage) const
{
    // How much to inflate the support columns to be stable. This also applies to the 1st layer, if no raft layers are to be printed.
    const float inflate_factor_fine      = float(scale_((m_support_params.raft_layers() > 1) ? 0.5 : EPSILON));
    const float inflate_factor_1st_layer = float(scale_(3.)) - inflate_factor_fine;
    MyLayer       *contacts      = top_contacts    .empty() ? nullptr : top_contacts    .front();
    MyLayer       *interfaces    = interface_layers.empty() ? nullptr : interface_layers.front();
    MyLayer       *columns_base  = base_layers     .empty() ? nullptr : base_layers     .front();
    if (contacts != nullptr && contacts->print_z > std::max(m_support_params.first_print_layer_height, m_support_params.raft_contact_top_z) + EPSILON)
        // This is not the raft contact layer.
        contacts = nullptr;
    if (interfaces != nullptr && interfaces->bottom_print_z() > m_support_params.raft_interface_top_z + EPSILON)
        // This is not the raft column base layer.
        interfaces = nullptr;
    if (columns_base != nullptr && columns_base->bottom_print_z() > m_support_params.raft_interface_top_z + EPSILON)
        // This is not the raft interface layer.
        columns_base = nullptr;

    Polygons interface_polygons;
    if (contacts != nullptr && ! contacts->polygons.empty())
        polygons_append(interface_polygons, offset(contacts->polygons, inflate_factor_fine, SUPPORT_SURFACES_OFFSET_PARAMETERS));
    if (interfaces != nullptr && ! interfaces->polygons.empty())
        polygons_append(interface_polygons, offset(interfaces->polygons, inflate_factor_fine, SUPPORT_SURFACES_OFFSET_PARAMETERS));

    // Output vector.
    MyLayersPtr raft_layers;

    if (m_support_params.raft_layers() > 1) {
        Polygons base;
        Polygons columns;
        if (columns_base != nullptr) {
            base = columns_base->polygons;
            columns = base;
            if (! interface_polygons.empty())
                // Trim the 1st layer columns with the inflated interface polygons.
                columns = diff(columns, interface_polygons);
        }
        if (! interface_polygons.empty()) {
            // Merge the untrimmed columns base with the expanded raft interface, to be used for the support base and interface.
            base = union_(base, interface_polygons);
        }
        // Do not add the raft contact layer, only add the raft layers below the contact layer.
        // Insert the 1st layer.
        {
            MyLayer &new_layer = layer_allocate(layer_storage, (m_support_params.base_raft_layers > 0) ? sltRaftBase : sltRaftInterface);
            raft_layers.push_back(&new_layer);
            new_layer.print_z = m_support_params.first_print_layer_height;
            new_layer.height  = m_support_params.first_print_layer_height;
            new_layer.bottom_z = 0.;
            new_layer.polygons = offset(base, inflate_factor_1st_layer);
        }
        // Insert the base layers.
        for (size_t i = 1; i < m_support_params.base_raft_layers; ++ i) {
            coordf_t print_z = raft_layers.back()->print_z;
            MyLayer &new_layer  = layer_allocate(layer_storage, sltRaftBase);
            raft_layers.push_back(&new_layer);
            new_layer.print_z  = print_z + m_support_params.base_raft_layer_height;
            new_layer.height   = m_support_params.base_raft_layer_height;
            new_layer.bottom_z = print_z;
            new_layer.polygons = base;
        }
        // Insert the interface layers.
        for (size_t i = 1; i < m_support_params.interface_raft_layers; ++ i) {
            coordf_t print_z = raft_layers.back()->print_z;
            MyLayer &new_layer = layer_allocate(layer_storage, sltRaftInterface);
            raft_layers.push_back(&new_layer);
            new_layer.print_z = print_z + m_support_params.interface_raft_layer_height;
            new_layer.height  = m_support_params.interface_raft_layer_height;
            new_layer.bottom_z = print_z;
            new_layer.polygons = interface_polygons;
            //FIXME misusing contact_polygons for support columns.
            new_layer.contact_polygons = new Polygons(columns);
        }
    } else if (columns_base != nullptr) {
        // Expand the bases of the support columns in the 1st layer.
        columns_base->polygons = diff(
            offset(columns_base->polygons, inflate_factor_1st_layer),
            offset(m_object->layers.front()->slices.expolygons, scale_(m_gap_xy), SUPPORT_SURFACES_OFFSET_PARAMETERS));
        if (contacts != nullptr)
            columns_base->polygons = diff(columns_base->polygons, interface_polygons);
    }

    return raft_layers;
}

PrintObjectSupportMaterial::MyLayersPtr
PrintObjectSupportMaterial::generate_interface_layers(const PrintObjectSupportMaterial::MyLayersPtr &bottom_contacts,
                                                      const PrintObjectSupportMaterial::MyLayersPtr &top_contacts,
                                                      PrintObjectSupportMaterial::MyLayersPtr &intermediate_layers,
                                                      PrintObjectSupportMaterial::MyLayerStorage &layer_storage) const
{
    //    my $area_threshold = $self->interface_flow->scaled_spacing ** 2;

    MyLayersPtr interface_layers;
    // Contact layer is considered an interface layer, therefore run the following block only if support_material_interface_layers > 1.
    if (! intermediate_layers.empty() && m_object_config->support_material_interface_layers.value > 1) {
        // For all intermediate layers, collect top contact surfaces, which are not further than support_material_interface_layers.
        interface_layers.assign(intermediate_layers.size(), nullptr);
        tbb::spin_mutex layer_storage_mutex;
        tbb::parallel_for(tbb::blocked_range<size_t>(0, intermediate_layers.size()),
                          [this, &bottom_contacts, &top_contacts, &intermediate_layers, &layer_storage, &layer_storage_mutex, &interface_layers](const tbb::blocked_range<size_t>& range) {
                              // Index of the first top contact layer intersecting the current intermediate layer.
                              size_t idx_top_contact_first = size_t(-1);
                              // Index of the first bottom contact layer intersecting the current intermediate layer.
                              size_t idx_bottom_contact_first = size_t(-1);
                              for (size_t idx_intermediate_layer = range.begin(); idx_intermediate_layer < range.end(); ++ idx_intermediate_layer) {
                                  MyLayer &intermediate_layer = *intermediate_layers[idx_intermediate_layer];
                                  // Top / bottom Z coordinate of a slab, over which we are collecting the top / bottom contact surfaces.
                                  coordf_t top_z    = intermediate_layers[std::min<int>(intermediate_layers.size()-1, idx_intermediate_layer + m_object_config->support_material_interface_layers - 1)]->print_z;
                                  coordf_t bottom_z = intermediate_layers[std::max<int>(0, int(idx_intermediate_layer) - int(m_object_config->support_material_interface_layers) + 1)]->bottom_z;
                                  // Move idx_top_contact_first up until above the current print_z.
                                  idx_top_contact_first = idx_higher_or_equal(top_contacts, idx_top_contact_first, [&intermediate_layer](const MyLayer *layer){ return layer->print_z >= intermediate_layer.print_z; });
                                  // Collect the top contact areas above this intermediate layer, below top_z.
                                  Polygons polygons_top_contact_projected;
                                  for (size_t idx_top_contact = idx_top_contact_first; idx_top_contact < top_contacts.size(); ++ idx_top_contact) {
                                      const MyLayer &top_contact_layer = *top_contacts[idx_top_contact];
                                      if (top_contact_layer.bottom_z - EPSILON > top_z)
                                          break;
                                      polygons_append(polygons_top_contact_projected, top_contact_layer.polygons);
                                  }
                                  // Move idx_bottom_contact_first up until touching bottom_z.
                                  idx_bottom_contact_first = idx_higher_or_equal(bottom_contacts, idx_bottom_contact_first, [bottom_z](const MyLayer *layer){ return layer->print_z >= bottom_z - EPSILON; });
                                  // Collect the top contact areas above this intermediate layer, below top_z.
                                  Polygons polygons_bottom_contact_projected;
                                  for (size_t idx_bottom_contact = idx_bottom_contact_first; idx_bottom_contact < bottom_contacts.size(); ++ idx_bottom_contact) {
                                      const MyLayer &bottom_contact_layer = *bottom_contacts[idx_bottom_contact];
                                      if (bottom_contact_layer.print_z - EPSILON > intermediate_layer.bottom_z)
                                          break;
                                      polygons_append(polygons_bottom_contact_projected, bottom_contact_layer.polygons);
                                  }

                                  if (polygons_top_contact_projected.empty() && polygons_bottom_contact_projected.empty())
                                      continue;

                                  // Insert a new layer into top_interface_layers.
                                  MyLayer &layer_new = layer_allocate(layer_storage, layer_storage_mutex,
                                                                      polygons_top_contact_projected.empty() ? sltBottomInterface : sltTopInterface);
                                  layer_new.print_z    = intermediate_layer.print_z;
                                  layer_new.bottom_z   = intermediate_layer.bottom_z;
                                  layer_new.height     = intermediate_layer.height;
                                  layer_new.bridging   = intermediate_layer.bridging;
                                  interface_layers[idx_intermediate_layer] = &layer_new;

                                  polygons_append(polygons_top_contact_projected, polygons_bottom_contact_projected);
                                  polygons_top_contact_projected = union_(polygons_top_contact_projected, true);
                                  layer_new.polygons = intersection(intermediate_layer.polygons, polygons_top_contact_projected);
                                  //FIXME filter layer_new.polygons islands by a minimum area?
                                  //                $interface_area = [ grep abs($_->area) >= $area_threshold, @$interface_area ];
                                  intermediate_layer.polygons = diff(intermediate_layer.polygons, polygons_top_contact_projected, false);
                              }
                          });

        // Compress contact_out, remove the nullptr items.
        remove_nulls(interface_layers);
    }

    return interface_layers;
}

void
PrintObjectSupportMaterial::trim_support_layers_by_object(const PrintObject &object,
                                                          PrintObjectSupportMaterial::MyLayersPtr &support_layers,
                                                          const coordf_t gap_extra_above,
                                                          const coordf_t gap_extra_below,
                                                          const coordf_t gap_xy) const
{
    const float gap_xy_scaled = float(scale_(gap_xy));

    // Collect non-empty layers to be processed in parallel.
    // This is a good idea as pulling a thread from a thread pool for an empty task is expensive.
    MyLayersPtr nonempty_layers;
    nonempty_layers.reserve(support_layers.size());
    for (size_t idx_layer = 0; idx_layer < support_layers.size(); ++ idx_layer) {
        MyLayer *support_layer = support_layers[idx_layer];
        if (! support_layer->polygons.empty() && support_layer->print_z >= m_support_params.raft_contact_top_z + EPSILON)
            // Non-empty support layer and not a raft layer.
            nonempty_layers.push_back(support_layer);
    }

    // For all intermediate support layers:
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, nonempty_layers.size()),
        [this, &object, &nonempty_layers, gap_extra_above, gap_extra_below, gap_xy_scaled](const tbb::blocked_range<size_t>& range) {
            size_t idx_object_layer_overlapping = size_t(-1);
            for (size_t idx_layer = range.begin(); idx_layer < range.end(); ++ idx_layer) {
                MyLayer &support_layer = *nonempty_layers[idx_layer];
                // BOOST_LOG_TRIVIAL(trace) << "Support generator - trim_support_layers_by_object - trimmming non-empty layer " << idx_layer << " of " << nonempty_layers.size();
                assert(! support_layer.polygons.empty() && support_layer.print_z >= m_support_params.raft_contact_top_z + EPSILON);
                // Find the overlapping object layers including the extra above / below gap.
                coordf_t z_threshold = support_layer.print_z - support_layer.height - gap_extra_below + EPSILON;
                idx_object_layer_overlapping = idx_higher_or_equal(
                    object.layers, idx_object_layer_overlapping,
                    [z_threshold](const Layer *layer){ return layer->print_z >= z_threshold; });
                // Collect all the object layers intersecting with this layer.
                Polygons polygons_trimming;
                size_t i = idx_object_layer_overlapping;
                for (; i < object.layers.size(); ++ i) {
                    const Layer &object_layer = *object.layers[i];
                    if (object_layer.print_z - object_layer.height > support_layer.print_z + gap_extra_above - EPSILON)
                        break;
                    polygons_append(polygons_trimming, (Polygons)object_layer.slices);
                }
                if (! this->m_support_params.soluble_interface) {
                    // Collect all bottom surfaces, which will be extruded with a bridging flow.
                    for (; i < object.layers.size(); ++ i) {
                        const Layer &object_layer = *object.layers[i];
                        bool some_region_overlaps = false;
                        for (LayerRegion* region : object_layer.regions) {
                            coordf_t nozzle_dmr = region->region()->nozzle_dmr_avg(*this->m_print_config);
                            if (object_layer.print_z - nozzle_dmr > support_layer.print_z + gap_extra_above - EPSILON)
                                break;
                            some_region_overlaps = true;
                            polygons_append(polygons_trimming, to_polygons(region->slices.filter_by_type(stBottomBridge)));
                        }
                        if (! some_region_overlaps)
                            break;
                    }
                }
                // $layer->slices contains the full shape of layer, thus including
                // perimeter's width. $support contains the full shape of support
                // material, thus including the width of its foremost extrusion.
                // We leave a gap equal to a full extrusion width.
                support_layer.polygons = diff(
                    support_layer.polygons,
                    offset(polygons_trimming, gap_xy_scaled, SUPPORT_SURFACES_OFFSET_PARAMETERS));
            }
        });
}

// TODO @Samir55
void
PrintObjectSupportMaterial::generate_toolpaths(const PrintObject &object,
                                               const PrintObjectSupportMaterial::MyLayersPtr &raft_layers,
                                               const PrintObjectSupportMaterial::MyLayersPtr &bottom_contacts,
                                               const PrintObjectSupportMaterial::MyLayersPtr &top_contacts,
                                               const PrintObjectSupportMaterial::MyLayersPtr &intermediate_layers,
                                               const PrintObjectSupportMaterial::MyLayersPtr &interface_layers) const
{

}

Flow
support_material_flow(const PrintObject *object, float layer_height)
{
    return Flow::new_from_config_width(
        frSupportMaterial,
        // The width parameter accepted by new_from_config_width is of type ConfigOptionFloatOrPercent, the Flow class takes care of the percent to value substitution.
        (object->config.support_material_extrusion_width.value > 0) ? object->config.support_material_extrusion_width
                                                                    : object->config.extrusion_width,
        // if object->config.support_material_extruder == 0 (which means to not trigger tool change, but use the current extruder instead), get_at will return the 0th component.
        float(object->print()->config.nozzle_diameter.get_at(object->config.support_material_extruder - 1)),
        (layer_height > 0.0f) ? layer_height : float(object->config.layer_height.value),
        false);
}

Flow
support_material_1st_layer_flow(const PrintObject *object, float layer_height)
{
    const auto &width = (object->print()->config.first_layer_extrusion_width.value > 0)
                        ? object->print()->config.first_layer_extrusion_width
                        : object->config.support_material_extrusion_width;
    return Flow::new_from_config_width(
        frSupportMaterial,
        // The width parameter accepted by new_from_config_width is of type ConfigOptionFloatOrPercent, the Flow class takes care of the percent to value substitution.
        (width.value > 0) ? width : object->config.extrusion_width,
        float(object->print()->config.nozzle_diameter.get_at(object->config.support_material_extruder - 1)),
        (layer_height > 0.0f) ? layer_height
                              : float(object->config.first_layer_height
                                          .get_abs_value(object->config.layer_height.value)),
        false);
}

Flow
support_material_interface_flow(const PrintObject *object, float layer_height)
{
    return Flow::new_from_config_width(
        frSupportMaterialInterface,
        // The width parameter accepted by new_from_config_width is of type ConfigOptionFloatOrPercent, the Flow class takes care of the percent to value substitution.
        (object->config.support_material_extrusion_width > 0) ? object->config.support_material_extrusion_width
                                                              : object->config.extrusion_width,
        // if object->config.support_material_interface_extruder == 0 (which means to not trigger tool change, but use the current extruder instead), get_at will return the 0th component.
        float(object->print()->config.nozzle_diameter.get_at(object->config.support_material_interface_extruder - 1)),
        (layer_height > 0.0f) ? layer_height : float(object->config.layer_height.value),
        false);
}

}