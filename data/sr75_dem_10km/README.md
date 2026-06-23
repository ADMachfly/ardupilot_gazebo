# SR-75 10 km Terrain Regeneration

The GeoTIFF DEM remains the source of truth, but the runtime terrain now uses a
mesh instead of a Gazebo `heightmap`.

Reason: the heightmap / GeoTIFF path loads on the server, but Ogre2 GUI hits
the Terra shader path and crashes in this WSL environment with
`0TerraShadowGenerator` compilation failures. A plain mesh visual avoids Terra.

## Current reference raster

- Source download: `data/sr75_dem_10km/raw/output_SRTMGL1.tif`
- Processed DEM: `data/sr75_dem_10km/processed/sr75_srtm_10km_utm43_30m.tif`
- Runtime mesh: `models/sr75_terrain_10km/meshes/sr75_terrain_10km.obj`

## Regenerate from the downloaded GeoTIFF

```sh
mkdir -p data/sr75_dem_10km/processed

gdalwarp \
  -t_srs EPSG:32643 \
  -r bilinear \
  -tr 30 30 \
  -ot Int16 \
  -dstnodata -32768 \
  data/sr75_dem_10km/raw/output_SRTMGL1.tif \
  data/sr75_dem_10km/processed/sr75_srtm_10km_utm43_30m.tif

python3 tools/sr75_generate_dem_mesh.py \
  data/sr75_dem_10km/processed/sr75_srtm_10km_utm43_30m.tif \
  models/sr75_terrain_10km/meshes/sr75_terrain_10km.obj \
  --step 6 \
  --reference center
```

## Sanity checks

Expected properties of the current projected DEM:

- Raster size: `336 x 334`
- Pixel size: `30 m x 30 m`
- Approximate terrain footprint: `10080 x 10020 m`
- Sample format: `Int16`
- NoData: `-32768`
- Elevation range: `219 m` to `249 m`
- Default mesh reference elevation: center sample (`236 m`)

The mesh generator decimates the DEM to a lower-resolution regular grid,
centers it around `(0, 0)`, and subtracts a reference elevation so the SR-75
launcher can remain near `z=0`.
