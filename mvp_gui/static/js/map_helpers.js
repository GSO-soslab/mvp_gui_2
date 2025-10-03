// ============================================================================
// SECTION 1: COLOR MANAGEMENT
// ============================================================================
const colorManager = {
    // A curated list of distinct, visually appealing colors.
    predefinedColors: [
        '#33C4FF',      // 1. Light Blue
        '#1dfa00ff',    // 2. Green
        '#FF6347',      // 3. Tomato (Orange-Red)
        '#40E0D0',      // 4. Turquoise (Cyan)
        '#FF8C00',      // 5. Dark Orange
        '#EE82EE',      // 6. Violet
        '#b7fa00ff',    // 7. Light Green
        '#F08080',      // 8. Light Coral (Pinkish)
        '#20B2AA',      // 9. Light Sea Green (Teal)
        '#C71585'       // 10. Medium Violet Red (Magenta)
    ],

    /**
     * Gathers all colors currently used by dynamic map elements.
     * @returns {Set<string>} A Set of hex color strings.
     */
    getUsedColors: function () {
        const used = new Set();
        // Get colors from dynamic GPS trails
        Object.values(dynamicGPSColors).forEach(color => used.add(color));
        // Get colors from user-defined markers
        userMarkersData.forEach(marker => {
            if (marker.color) {
                used.add(marker.color);
            }
        });
        return used;
    },

    /**
     * Assigns a unique color for a new map element.
     * It first tries to use an available color from the predefined list.
     * If all are taken, it generates a random color that is not in use.
     * @returns {string} A hex color string.
     */
    assignColor: function () {
        const usedColors = this.getUsedColors();

        // Try to find an unused predefined color first for consistency
        for (const color of this.predefinedColors) {
            if (!usedColors.has(color)) {
                return color;
            }
        }

        // Fallback: generate a random color if all predefined are used
        let randomColor;
        do {
            randomColor = `#${Math.floor(Math.random() * 16777215).toString(16).padStart(6, '0')}`;
        } while (usedColors.has(randomColor)); // Ensure it's not a fluke duplicate

        return randomColor;
    }
};

// ============================================================================
// SECTION 2: STATE PERSISTENCE
// ============================================================================
function saveMapState() {
    // We only save the view settings, history trails, and colors.
    const stateToSave = {
        mapType: currentMapType,
        vehicleHistory: vehicleHistory,
        dynamicGPSHistories: dynamicGPSHistories,
        dynamicGPSColors: dynamicGPSColors,
        userMarkers: userMarkersData, // Save user markers
    };
    if (map && isMapInitialized) {
        stateToSave.zoom = map.getZoom();
        stateToSave.center = map.getCenter();
    }
    sessionStorage.setItem(MAP_STATE_KEY, JSON.stringify(stateToSave));
}

function loadMapState() {
    const savedStateJSON = sessionStorage.getItem(MAP_STATE_KEY);
    if (savedStateJSON) {
        try {
            const savedState = JSON.parse(savedStateJSON);
            currentMapType = savedState.mapType ?? 'offline'; // Default to offline if not in saved state
            // Restore history trails and colors
            vehicleHistory = savedState.vehicleHistory || [];
            dynamicGPSHistories = savedState.dynamicGPSHistories || {};
            dynamicGPSColors = savedState.dynamicGPSColors || {};
            userMarkersData = savedState.userMarkers || []; // Load user markers

            // --- Patch loaded data: Assign colors to any user markers that don't have one ---
            // This ensures backward compatibility with old session data.
            userMarkersData.forEach(marker => {
                if (!marker.color) {
                    marker.color = colorManager.assignColor();
                }
            });

            // Return camera position if it exists
            if (savedState.center) {
                return [
                    [savedState.center.lng, savedState.center.lat], savedState.zoom
                ];
            }
        } catch (e) {
            console.error("Error parsing map state from sessionStorage:", e);
            sessionStorage.removeItem(MAP_STATE_KEY);
        }
    }
    return null;
}

// ============================================================================
// SECTION 3: MAP INITIALIZATION & MANAGEMENT
// ============================================================================
function initializeMap(centerCoords, zoomLevel = 19) {
    if (map) return; // Prevent creating multiple map objects

    let mapStyle;
    if (currentMapType === 'online') {
        mapStyle = 'https://api.maptiler.com/maps/hybrid/style.json?key=XiFHd4BzZGlB2Dsix5mK';
    } else if (currentMapType === 'noaa') {
        mapStyle = {
            "version": 8,
            "name": "Offline NOAA Chart Style",
            "sources": {
                "offline-noaa-tiles": {
                    "type": "raster",
                    "tiles": [`${serverUrlRoot}tiles/noaa_chart/{z}/{x}/{y}.png`],
                    "tileSize": 256,
                    "attribution": "NOAA",
                    "maxzoom": 20
                }
            },
            "layers": [{
                "id": "offline-noaa-layer",
                "type": "raster",
                "source": "offline-noaa-tiles"
            }]
        };
    } else { // 'offline' is the default, now treated as satellite
        mapStyle = {
            "version": 8,
            "name": "Offline Satellite Style",
            "sources": {
                "offline-satellite-tiles": {
                    "type": "raster",
                    "tiles": [`${serverUrlRoot}tiles/satellite/{z}/{x}/{y}.png`],
                    "tileSize": 256,
                    "maxzoom": 20
                }
            },
            "layers": [{
                "id": "offline-satellite-layer",
                "type": "raster",
                "source": "offline-satellite-tiles"
            }]
        };
    }

    map = new maplibregl.Map({
        container: 'map',
        style: mapStyle,
        center: centerCoords,
        zoom: zoomLevel,
        doubleClickZoom: false // Disable double click to zoom
    });
    map.on('moveend', saveMapState);
    map.on('zoomend', saveMapState);
    map.on('load', () => {
        // This code adds the sources and layers for the dynamic vector data (trails, paths).
        // It will work on top of both online and offline base maps.
        map.addSource('vehicle-trail', { type: 'geojson', data: { type: 'Feature', geometry: { type: 'LineString', coordinates: [] } } });
        map.addLayer({ id: 'vehicle-trail-layer', type: 'line', source: 'vehicle-trail', paint: { 'line-color': '#8400ffff', 'line-width': 3, 'line-opacity': 0.7 } });

        // Add source and layer for vehicle trail points
        map.addSource('vehicle-trail-points', { type: 'geojson', data: { type: 'FeatureCollection', features: [] } });
        map.addLayer({
            id: 'vehicle-trail-points-layer',
            type: 'circle',
            source: 'vehicle-trail-points',
            paint: {
                'circle-radius': 3,
                'circle-color': '#8400ffff',
                'circle-stroke-color': '#ffffffff', 
                'circle-stroke-width': 1,
                'circle-opacity': 0.7
            }
        });

        map.addSource('editable-waypoints-route', { type: 'geojson', data: { type: 'Feature', geometry: { type: 'LineString', coordinates: [] } } });
        map.addLayer({ id: 'editable-waypoints-route-layer', type: 'line', source: 'editable-waypoints-route', paint: EDITABLE_PATH_STYLE_DEFAULT });

        map.addSource('published-waypoints-route', { type: 'geojson', data: { type: 'Feature', geometry: { type: 'LineString', coordinates: [] } } });
        map.addLayer({ id: 'published-waypoints-route-layer', type: 'line', source: 'published-waypoints-route', paint: { 'line-color': '#ffd500', 'line-width': 3, 'line-dasharray': [4, 2] } });

        isMapInitialized = true;
        setupMeasurementTool(); // Initialize sources/layers for measurement tool

        // --- Map click logic for adding user marker ---
        map.on('click', (e) => {
            const addUserMarkerBtn = document.getElementById('add-user-marker-button');
            const addWaypointBtn = document.getElementById('add-waypoint-button');

            if (isAddingUserMarker) {
                const newId = `um-${Date.now()}`;
                const newMarkerData = {
                    id: newId,
                    lat: e.lngLat.lat,
                    lon: e.lngLat.lng,
                    name: `user-marker-${userMarkersData.length + 1}`,
                    color: colorManager.assignColor()
                };

                userMarkersData.push(newMarkerData);
                createUserMarker(newMarkerData);
                updateLegend();
                saveMapState();

                isAddingUserMarker = false;
                addUserMarkerBtn.classList.remove('btn-success');
                addUserMarkerBtn.classList.add('btn-info');
                addUserMarkerBtn.textContent = 'Add User Marker';
                map.getCanvas().style.cursor = '';

            } else if (isAddingWaypoint) {
                socket.emit('add_waypoint', {
                    lat: e.lngLat.lat,
                    lon: e.lngLat.lng
                });

                isAddingWaypoint = false;
                addWaypointBtn.classList.remove('btn-success');
                addWaypointBtn.classList.add('btn-primary');
                addWaypointBtn.textContent = 'Add Waypoint';
                map.getCanvas().style.cursor = '';
            } else if (isMeasuring) {
                handleMeasurementClick(e.lngLat);
            }
        });

        // Redraw everything that might have been restored or received while map was loading
        redrawUserMarkers();
        createEditableWaypointMarkers();
        redrawAllFromState();
        updateLegend(); // Update legend with static and restored dynamic items
    });
}

// ============================================================================
// SECTION 3.5: MEASUREMENT TOOL
// ============================================================================
/**
 * Sets up the GeoJSON source and layer for the measurement line.
 * Called once when the map is loaded.
 */
function setupMeasurementTool() {
    map.addSource('measurement-line-source', {
        type: 'geojson',
        data: { type: 'Feature', geometry: { type: 'LineString', coordinates: [] } }
    });
    map.addLayer({
        id: 'measurement-line-layer',
        type: 'line',
        source: 'measurement-line-source',
        paint: {
            'line-color': '#00ffff', // Cyan
            'line-width': 3,
            'line-dasharray': [2, 2]
        }
    });
}

/**
 * Handles clicks on the map when the measurement tool is active.
 * @param {maplibregl.LngLat} lngLat The coordinates of the click.
 */
function handleMeasurementClick(lngLat) {
    // Add a point marker
    const pointMarker = new maplibregl.Marker({ color: '#00ffff' })
        .setLngLat(lngLat)
        .addTo(map);
    measurementMarkers.push(pointMarker);
    measurementPoints.push([lngLat.lng, lngLat.lat]);

    const measureBtn = document.getElementById('measure-button');

    if (measurementPoints.length === 1) {
        measureBtn.textContent = 'Click to place 2nd point...';
    } else if (measurementPoints.length === 2) {
        // Draw line and label
        drawMeasurementLine();

        // Finalize state
        isMeasuring = false;
        map.getCanvas().style.cursor = '';
        measureBtn.textContent = 'Clear Measurement';
        measureBtn.classList.remove('btn-warning');
        measureBtn.classList.add('btn-danger');
    }
}

/**
 * Draws the measurement line and distance label after two points are selected.
 */
function drawMeasurementLine() {
    if (measurementPoints.length < 2) return;

    // Update the line source
    map.getSource('measurement-line-source').setData({
        type: 'Feature',
        geometry: { type: 'LineString', coordinates: measurementPoints }
    });

    // Calculate distance and midpoint for the label
    const p1 = { lon: measurementPoints[0][0], lat: measurementPoints[0][1] };
    const p2 = { lon: measurementPoints[1][0], lat: measurementPoints[1][1] };
    const distance = haversineDistance(p1, p2);
    const midpoint = {
        longitude: (p1.lon + p2.lon) / 2,
        latitude: (p1.lat + p2.lat) / 2
    };

    // Create label element
    const el = document.createElement('div');
    el.className = 'measurement-label';
    el.textContent = `${distance.toFixed(1)} m`;

    // Add label marker to map
    measurementLabel = new maplibregl.Marker({ element: el, anchor: 'bottom' })
        .setLngLat([midpoint.longitude, midpoint.latitude])
        .addTo(map);
}

/**
 * Clears all measurement artifacts from the map and resets the state.
 */
function clearMeasurement() {
    // Remove markers
    measurementMarkers.forEach(marker => marker.remove());
    measurementMarkers = [];
    if (measurementLabel) {
        measurementLabel.remove();
        measurementLabel = null;
    }

    // Clear line data
    if (map && map.getSource('measurement-line-source')) {
         map.getSource('measurement-line-source').setData({
            type: 'Feature',
            geometry: { type: 'LineString', coordinates: [] }
        });
    }

    // Reset state
    measurementPoints = [];

    const measureBtn = document.getElementById('measure-button');
    if(measureBtn) {
        measureBtn.textContent = 'Measure';
        measureBtn.classList.remove('btn-danger', 'btn-warning');
        measureBtn.classList.add('btn-secondary');
    }
}

// ============================================================================
// SECTION 4: DATA & PATH DRAWING FUNCTIONS
// ============================================================================

// --- Core Redraw Function ---
function redrawAllFromState() {
    if (!isMapInitialized) return;
    if (currentVehiclePose) updateVehicle(currentVehiclePose);
    // Redraw trails from restored history
    redrawAllTrails();
    redrawDynamicGpsTrails();
    // Redraw paths using live data
    updateEditablePathLine();
    drawPublishedPath();
}

// --- Vehicle Path ---
function updateVehicle(data) {
    if (!map || !data) return;
    const el = vehicleMarker ? vehicleMarker.getElement() : createVehicleMarkerElement();
    el.querySelector('.arrow-marker').style.transform = `rotate(${90 - data.yaw}deg)`;
    if (!vehicleMarker) {
        vehicleMarker = new maplibregl.Marker({ element: el }).setLngLat([data.lon, data.lat]).addTo(map);
    } else {
        vehicleMarker.setLngLat([data.lon, data.lat]);
    }
}

function redrawAllTrails() {
    if (!isMapInitialized) return;
    // Update line
    map.getSource('vehicle-trail')?.setData({ type: 'Feature', geometry: { type: 'LineString', coordinates: vehicleHistory } });

    // Update points
    const features = vehicleHistory.map(coords => ({
        type: 'Feature',
        geometry: { type: 'Point', coordinates: coords },
        properties: {}
    }));
    map.getSource('vehicle-trail-points')?.setData({
        type: 'FeatureCollection',
        features: features
    });
}

// --- Editable Mission Path ---
function updateEditablePathLine() {
    if (!isMapInitialized) return;
    editableAltitudeMarkers.forEach(m => m.remove());
    editableAltitudeMarkers = [];
    let pathPoints = editableWaypointsData.map(w => ({ lat: w.lat, lon: w.lon, alt: w.alt, surge: w.surge }));
    if (currentVehiclePose) {
        // The vehicle itself doesn't have a target surge, but its pose object from ROS has 'u'.
        // The first segment's label (vehicle to WP1) will correctly use WP1's surge.
        // We just need to ensure the vehicle point in our array has a surge property for consistency.
        pathPoints.unshift({ lat: currentVehiclePose.lat, lon: currentVehiclePose.lon, alt: currentVehiclePose.alt || 0, surge: currentVehiclePose.u || 0 });
    }

    editablePathTotalDistance = calculateTotalDistance(pathPoints);
    updateLegend();

    const lineCoords = pathPoints.map(p => [p.lon, p.lat]);
    map.getSource('editable-waypoints-route').setData({ type: 'Feature', geometry: { type: 'LineString', coordinates: lineCoords } });
    for (let i = 1; i < pathPoints.length; i++) {
        editableAltitudeMarkers.push(createSegmentLabel(calculateMidpoint(pathPoints[i - 1], pathPoints[i]), 'editable', '#fba5b0'));
    }
}

// --- Published Mission Path ---
function drawPublishedPath() {
    if (!isMapInitialized) return;
    // Clear old markers
    publishedWaypointMarkers.forEach(m => m.remove());
    publishedWaypointMarkers = [];
    publishedAltitudeMarkers.forEach(m => m.remove());
    publishedAltitudeMarkers = [];
    // This is the GeoPath data from ROS: [vehicle_pose, wp1_ros, wp2_ros, ...]
    const canonicalPath = lastPublishedPath || [];

    // The ROS node might send 'u' for surge, so handle both 'surge' and 'u' robustly.
    let linePoints = canonicalPath.map(p => ({ lat: p.lat, lon: p.lon, alt: p.alt, surge: p.surge ?? p.u ?? 0 }));
    publishedPathTotalDistance = calculateTotalDistance(linePoints);
    updateLegend();

    if (canonicalPath.length === 0) {
        map.getSource('published-waypoints-route').setData({ type: 'Feature', geometry: { type: 'LineString', coordinates: [] } });
        return;
    }
    // 1. Create markers for each waypoint *using the coordinates from the GeoPath message*.
    canonicalPath.forEach((point, index) => {
        publishedWaypointMarkers.push(new maplibregl.Marker({
            element: createPublishedWaypointMarkerElement(point, index), // index is the label
            anchor: 'bottom'
        }).setLngLat([point.lon, point.lat]).addTo(map));
    });
    // 2. The line's path points are also directly from the GeoPath message.
    // 3. Set the data for the line layer on the map.
    const lineCoords = linePoints.map(p => [p.lon, p.lat]);
    map.getSource('published-waypoints-route').setData({ type: 'Feature', geometry: { type: 'LineString', coordinates: lineCoords } });
    // 4. Create segment labels for the complete line.
    for (let i = 1; i < linePoints.length; i++) {
        const p1 = linePoints[i - 1];
        const p2 = linePoints[i];
        publishedAltitudeMarkers.push(createSegmentLabel(calculateMidpoint(p1, p2), 'published', '#fffca0'));
    }
}

// --- Dynamic GPS Trails ---
function updateDynamicGPS(topicName, data) {
    // Robustly check for valid data.
    if (!map || !data || data.lon == null || data.lat == null) {
        return;
    }

    let isNewTopic = false;
    // Initialize color and history for new topics. This is safe to do before the map is ready.
    if (!dynamicGPSColors[topicName]) {
        isNewTopic = true;
        dynamicGPSColors[topicName] = colorManager.assignColor();
        dynamicGPSHistories[topicName] = [];
    }

    const lngLat = [data.lon, data.lat];
    // Update history. This is also safe to do before map is ready.
    dynamicGPSHistories[topicName].push(lngLat);
    if (dynamicGPSHistories[topicName].length > MAX_HISTORY_POINTS_GPS) {
        dynamicGPSHistories[topicName].shift();
    }

    // --- All subsequent operations modify the map and must wait for it to be initialized ---
    if (!isMapInitialized) {
        return;
    }

    // Create/update marker for the most recent point (the "head").
    if (!dynamicGPSMarkers[topicName]) {
        dynamicGPSMarkers[topicName] = createDynamicGPSMarker(dynamicGPSColors[topicName], lngLat);
    } else {
        dynamicGPSMarkers[topicName].setLngLat(lngLat);
    }

    // --- Manage Point Trail (the "tail") ---
    const sanitizedTopicName = topicName.replace(/[^\w-]/g, '_');
    const sourceId = `gps-trail-points-${sanitizedTopicName}`;
    const layerId = `gps-trail-points-layer-${sanitizedTopicName}`;

    // Clean up any old line trail layer and source from previous versions.
    const oldLineSourceId = `gps-trail-${sanitizedTopicName}`;
    const oldLineLayerId = `gps-trail-layer-${sanitizedTopicName}`;
    if (map.getLayer(oldLineLayerId)) map.removeLayer(oldLineLayerId);
    if (map.getSource(oldLineSourceId)) map.removeSource(oldLineSourceId);

    // Create source and layer for points if they don't exist.
    if (!map.getSource(sourceId)) {
        map.addSource(sourceId, {
            type: 'geojson',
            data: { type: 'FeatureCollection', features: [] }
        });
        map.addLayer({
            id: layerId,
            type: 'circle',
            source: sourceId,
            paint: {
                'circle-radius': 4,
                'circle-color': dynamicGPSColors[topicName],
                'circle-opacity': [
                    'interpolate',
                    ['linear'],
                    ['get', 'age_ratio'],
                    0, 0.1, // Oldest point: opacity 0.1
                    1, 0.8 // Newest point: opacity 0.8
                ]
            }
        });
    }

    // Create GeoJSON FeatureCollection from history.
    const history = dynamicGPSHistories[topicName];
    const numPoints = history.length;
    const features = history.map((coords, index) => ({
        type: 'Feature',
        geometry: { type: 'Point', coordinates: coords },
        properties: { age_ratio: (numPoints > 1) ? (index / (numPoints - 1)) : 1.0 }
    }));

    // Update point trail data.
    map.getSource(sourceId).setData({
        type: 'FeatureCollection',
        features: features
    });

    if (isNewTopic) {
        updateLegend();
    }
}

function redrawDynamicGpsTrails() {
    if (!isMapInitialized) return;

    for (const topicName in dynamicGPSHistories) {
        if (dynamicGPSHistories.hasOwnProperty(topicName)) {
            const history = dynamicGPSHistories[topicName] || [];
            if (history.length === 0) continue;

            const sanitizedTopicName = topicName.replace(/[^\w-]/g, '_');
            const sourceId = `gps-trail-points-${sanitizedTopicName}`;
            const layerId = `gps-trail-points-layer-${sanitizedTopicName}`;

            // Clean up any old line trail layer and source from previous versions.
            const oldLineSourceId = `gps-trail-${sanitizedTopicName}`;
            const oldLineLayerId = `gps-trail-layer-${sanitizedTopicName}`;
            if (map.getLayer(oldLineLayerId)) map.removeLayer(oldLineLayerId);
            if (map.getSource(oldLineSourceId)) map.removeSource(oldLineSourceId);

            // Ensure color exists for this topic.
            if (!dynamicGPSColors[topicName]) {
                dynamicGPSColors[topicName] = colorManager.assignColor();
            }

            // Create source and layer for points if they don't exist.
            if (!map.getSource(sourceId)) {
                map.addSource(sourceId, {
                    type: 'geojson',
                    data: { type: 'FeatureCollection', features: [] }
                });
                map.addLayer({
                    id: layerId,
                    type: 'circle',
                    source: sourceId,
                    paint: {
                        'circle-radius': 4,
                        'circle-color': dynamicGPSColors[topicName],
                        'circle-opacity': [
                            'interpolate',
                            ['linear'],
                            ['get', 'age_ratio'],
                            0, 0.1,
                            1, 0.8
                        ]
                    }
                });
            }

            // Create GeoJSON FeatureCollection from history.
            const numPoints = history.length;
            const features = history.map((coords, index) => ({
                type: 'Feature',
                geometry: { type: 'Point', coordinates: coords },
                properties: { age_ratio: (numPoints > 1) ? (index / (numPoints - 1)) : 1.0 }
            }));

            // Update point trail data.
            map.getSource(sourceId).setData({
                type: 'FeatureCollection',
                features: features
            });

            // Re-create the head marker, as it's not persisted in sessionStorage.
            const lastPoint = history[history.length - 1];
            if (lastPoint) {
                if (dynamicGPSMarkers[topicName]) {
                    dynamicGPSMarkers[topicName].remove();
                }
                dynamicGPSMarkers[topicName] = createDynamicGPSMarker(dynamicGPSColors[topicName], lastPoint);
            }
        }
    }
}


// ============================================================================
// SECTION 5: MARKER MANAGEMENT FUNCTIONS
// ============================================================================

// --- Editable Waypoints ---
function createEditableWaypointMarkers() {
    editableWaypointMarkers.forEach(m => m.remove());
    editableWaypointMarkers = [];
    editableWaypointsData.forEach(item => {
        const el = createEditableWaypointMarkerElement(item);
        const marker = new maplibregl.Marker({ element: el, draggable: true, anchor: 'bottom' })
            .setLngLat([item.lon, item.lat])
            .addTo(map);

        // --- DRAG events ---
        marker.on('dragstart', () => {
            el.classList.add('marker-dragging');
            if (map.getLayer('editable-waypoints-route-layer')) {
                Object.keys(EDITABLE_PATH_STYLE_DRAGGING).forEach(prop => {
                    map.setPaintProperty('editable-waypoints-route-layer', prop, EDITABLE_PATH_STYLE_DRAGGING[prop]);
                });
            }
        });
        marker.on('drag', () => {
            const coords = marker.getLngLat();
            const draggedWaypoint = editableWaypointsData.find(w => w.id === item.id);
            if (draggedWaypoint) {
                draggedWaypoint.lon = coords.lng;
                draggedWaypoint.lat = coords.lat;
            }
            updateEditablePathLine();
        });
        marker.on('dragend', () => {
            el.classList.remove('marker-dragging');
            if (map.getLayer('editable-waypoints-route-layer')) {
                Object.keys(EDITABLE_PATH_STYLE_DEFAULT).forEach(prop => {
                    map.setPaintProperty('editable-waypoints-route-layer', prop, EDITABLE_PATH_STYLE_DEFAULT[prop]);
                });
            }
            const coords = marker.getLngLat();
            fetch('/waypoint_drag', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ id: item.id, lng: coords.lng, lat: coords.lat, alt: item.alt, surge: item.surge }) });
        });

        // --- DOUBLE CLICK event for editing ---
        el.addEventListener('dblclick', () => {
            // Populate the modal with the waypoint's data
            document.getElementById('editWaypointId').value = item.id;
            document.getElementById('editWaypointLat').value = item.lat;
            document.getElementById('editWaypointLon').value = item.lon;
            document.getElementById('editWaypointAlt').value = item.alt;
            document.getElementById('editWaypointSurge').value = item.surge;
            // Show the modal using jQuery
            $('#waypointEditModal').modal('show');
        });

        editableWaypointMarkers.push(marker);
    });
}

// --- User-Defined Markers ---
function redrawUserMarkers() {
    if (!isMapInitialized) return;
    Object.values(userMarkerObjects).forEach(marker => marker.remove());
    userMarkerObjects = {};
    userMarkersData.forEach(markerData => createUserMarker(markerData));
}


// ============================================================================
// SECTION 6: UI & ELEMENT CREATION HELPERS
// ============================================================================
// --- Map Enhancements ---
function addScaleToMap() {
    if (!map) return;
    // The 'load' event fires once per map instance after all necessary resources have been loaded.
    // It's the right place to add controls that depend on the map being fully ready.
    map.on('load', () => {
        // Add metric scale control (meters, kilometers)
        const scalemetric = new maplibregl.ScaleControl({
            maxWidth: 125, // in pixels
            unit: 'metric'
        });
        map.addControl(scalemetric, 'bottom-left');
    });
}

// --- Legend & Labels ---
function updateLegend() {
    const legendItemsContainer = document.getElementById('legend-items');
    if (!legendItemsContainer) return;

    legendItemsContainer.innerHTML = ''; // Clear existing legend

    // Static items
    const staticItems = [
        { color: '#8400ffff', label: 'Vehicle Path' },
        { color: '#ff190a', label: `Editable Mission: ${editablePathTotalDistance.toFixed(1)} m` },
        { color: '#ffd500', label: `Published Mission: ${publishedPathTotalDistance.toFixed(1)} m` }
    ];

    staticItems.forEach(item => {
        const li = document.createElement('li');
        li.innerHTML = `<span class="legend-color-box" style="background-color: ${item.color};"></span> ${item.label}`;
        legendItemsContainer.appendChild(li);
    });

    // Dynamic GPS items
    for (const topicName in dynamicGPSColors) {
        if (dynamicGPSColors.hasOwnProperty(topicName)) {
            const color = dynamicGPSColors[topicName];
            const displayName = getShortTopicName(topicName);
            const li = document.createElement('li');
            li.innerHTML = `<span class="legend-color-box" style="background-color: ${color};"></span> ${displayName}`;
            legendItemsContainer.appendChild(li);
        }
    }

    // User-defined markers - now each has its own entry
    userMarkersData.forEach(marker => {
        const li = document.createElement('li');
        const markerColor = marker.color || '#9400D3'; // Fallback for safety
        li.innerHTML = `<span class="legend-color-box" style="background-color: ${markerColor}; opacity: 0.8;"></span> ${marker.name}`;
        legendItemsContainer.appendChild(li);
    });
}

function createSegmentLabel(midpoint, type, color) {
    const el = document.createElement('div');
    el.className = `altitude-label-marker ${type}`; // e.g., 'editable' or 'published'
    el.style.color = color;
    // Use innerHTML to create three lines for altitude (z), surge (u), and distance (d)
    const surgeValue = midpoint.surge ?? 0;
    const distanceValue = midpoint.distance ?? 0;
    el.innerHTML = `z: ${midpoint.altitude.toFixed(1)}m<br>u: ${surgeValue.toFixed(1)}m/s<br>d: ${distanceValue.toFixed(1)}m`;
    return new maplibregl.Marker({
        element: el,
        anchor: 'center'
    }).setLngLat([midpoint.longitude, midpoint.latitude]).addTo(map);
}

function updateMapStatusText() {
    const statusEl = document.getElementById('map-status');
    if (statusEl) {
        let mapName;
        if (currentMapType === 'online') {
            mapName = 'Online (Satellite)';
        } else if (currentMapType === 'noaa') {
            mapName = 'NOAA Chart (Offline)';
        } else { // 'offline'
            mapName = 'Satellite (Offline)';
        }
        statusEl.textContent = `Current Map: ${mapName}`;
    }
}

// --- Marker Element Creators ---
function createVehicleMarkerElement() {
    const container = document.createElement('div');
    container.className = 'marker-container';
    const arrow = document.createElement('div');
    arrow.className = 'arrow-marker';
    container.appendChild(arrow);
    return container;
}

function createGroundStationElement() {
    const el = document.createElement('div');
    el.className = 'groundstation-marker';
    return el;
}

function createSecondaryElement() {
    const el = document.createElement('div');
    el.className = 'secondary-marker';
    return el;
}

function createEditableWaypointMarkerElement(item) {
    const markerElement = document.createElement('div');
    markerElement.className = 'editable-wpt-marker';
    const diamond = document.createElement('div');
    diamond.className = 'editable-wpt-diamond';
    const idDisplay = document.createElement('div');
    idDisplay.className = 'editable-wpt-id';
    idDisplay.innerText = item.id;
    diamond.appendChild(idDisplay);
    markerElement.appendChild(diamond);
    return markerElement;
}

function createPublishedWaypointMarkerElement(item, index) {
    const markerElement = document.createElement('div');
    markerElement.className = 'published-wpt-marker';
    const idDisplay = document.createElement('div');
    idDisplay.className = 'published-wpt-id';
    idDisplay.innerText = index;
    markerElement.append(idDisplay);
    return markerElement;
}

function createUserMarker(markerData) {
    const container = document.createElement('div');
    container.className = 'user-marker-container';

    const markerEl = document.createElement('div');
    markerEl.className = 'user-marker-shape';
    // --- Set color dynamically ---
    markerEl.style.backgroundColor = markerData.color || '#9400D3'; // Fallback to purple

    const labelEl = document.createElement('div');
    labelEl.className = 'user-marker-label';
    labelEl.innerText = markerData.name;
    // --- Set color dynamically ---
    labelEl.style.color = markerData.color || '#DA70D6'; // Fallback to orchid

    container.appendChild(markerEl);
    container.appendChild(labelEl);

    container.addEventListener('dblclick', () => {
        // Populate and show the modal for editing
        document.getElementById('userMarkerId').value = markerData.id;
        document.getElementById('userMarkerName').value = markerData.name;
        document.getElementById('userMarkerLat').value = markerData.lat;
        document.getElementById('userMarkerLon').value = markerData.lon;
        $('#userMarkerEditModal').modal('show');
    });

    const marker = new maplibregl.Marker({ element: container, anchor: 'center', draggable: false })
        .setLngLat([markerData.lon, markerData.lat])
        .addTo(map);

    userMarkerObjects[markerData.id] = marker;
}

function createDynamicGPSMarker(color, lngLat) {
    const el = document.createElement('div');
    el.className = 'dynamic-gps-marker';
    el.style.backgroundColor = color;
    el.style.width = '12px';
    el.style.height = '12px';
    el.style.borderRadius = '50%';
    el.style.border = '2px solid white';
    el.style.boxShadow = '0 0 5px rgba(0,0,0,0.5)';

    return new maplibregl.Marker({ element: el }).setLngLat(lngLat).addTo(map);
}


// ============================================================================
// SECTION 7: UTILITY & CALCULATION HELPERS
// ============================================================================
function getShortTopicName(topicName) {
    if (!topicName) return '';

    // Split by '/' and filter out empty strings that result from leading/trailing slashes
    const parts = topicName.split('/').filter(p => p.length > 0);

    // If there are 2 or fewer parts, return the original structure with a leading slash.
    // This handles cases like "/gps/fix" which should remain "/gps/fix".
    if (parts.length <= 2) {
        return '/' + parts.join('/');
    }

    // If there are 3 or more parts, apply the new shortening logic.
    // e.g., /mvp2_test_robot/remote/id_2/odometry/navsatfix -> /mvp2/odo/nav

    // 1. Get namespace_shorthand from the first part.
    // e.g., "mvp2_test_robot" -> "mvp2", "alpha_rise" -> "alpha", "xy" -> "xy"
    const namespacePart = parts[0];
    const underscoreIndex = namespacePart.indexOf('_');
    const namespaceShorthand = (underscoreIndex !== -1)
        ? namespacePart.substring(0, underscoreIndex)
        : namespacePart;

    // 2. Get the last two parts and shorten them to their first 3 letters.
    // e.g., "odometry" -> "odo", "navsatfix" -> "nav"
    const lastPart = parts[parts.length - 1];
    const secondLastPart = parts[parts.length - 2];

    const lastShort = lastPart.substring(0, 3);
    const secondLastShort = secondLastPart.substring(0, 3);

    // 3. Combine them in the format /namespace_shorthand/last_2_short/last_1_short
    return `/${namespaceShorthand}/${secondLastShort}/${lastShort}`;
}

function haversineDistance(p1, p2) {
    function toRad(x) { return x * Math.PI / 180; }
    const R = 6371e3; // Earth's radius in metres
    const lat1_Rad = toRad(p1.lat);
    const lat2_Rad = toRad(p2.lat);
    const delta_lat_Rad = toRad(p2.lat - p1.lat);
    const delta_lon_Rad = toRad(p2.lon - p1.lon);

    const a = Math.sin(delta_lat_Rad / 2) * Math.sin(delta_lat_Rad / 2) +
        Math.cos(lat1_Rad) * Math.cos(lat2_Rad) *
        Math.sin(delta_lon_Rad / 2) * Math.sin(delta_lon_Rad / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return R * c; // distance in meters
}

function calculateTotalDistance(pathPoints) {
    let totalDistance = 0;
    if (!pathPoints || pathPoints.length < 2) {
        return 0;
    }
    for (let i = 1; i < pathPoints.length; i++) {
        totalDistance += haversineDistance(pathPoints[i - 1], pathPoints[i]);
    }
    return totalDistance;
}

function updateHistory(historyArray, data) {
    historyArray.push([data.lon, data.lat]);
    if (historyArray.length > MAX_HISTORY_POINTS) {
        historyArray.shift();
    }
}

function calculateMidpoint(p1, p2) {
    const distance = haversineDistance(p1, p2);

    return {
        latitude: (p1.lat + p2.lat) / 2,
        longitude: (p1.lon + p2.lon) / 2,
        altitude: p2.alt,
        surge: p2.surge, // The surge of the destination waypoint.
        distance: distance // The calculated distance in meters.
    };
}