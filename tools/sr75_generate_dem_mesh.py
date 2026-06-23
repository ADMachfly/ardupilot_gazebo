#!/usr/bin/env python3
"""Generate a low-poly OBJ terrain mesh from the SR-75 processed DEM."""

from __future__ import annotations

import argparse
import os
import struct
from dataclasses import dataclass
from typing import Iterable


@dataclass
class DemRaster:
    width: int
    height: int
    pixel_size_x: float
    pixel_size_y: float
    nodata: int | None
    elevations: list[int]

    @property
    def valid_elevations(self) -> list[int]:
        if self.nodata is None:
            return self.elevations
        return [value for value in self.elevations if value != self.nodata]

    def at(self, x: int, y: int) -> int:
        return self.elevations[y * self.width + x]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a visual-only OBJ terrain mesh from a DEM GeoTIFF."
    )
    parser.add_argument(
        "input_tif",
        nargs="?",
        default="data/sr75_dem_10km/processed/sr75_srtm_10km_utm43_30m.tif",
        help="Input Int16 GeoTIFF DEM.",
    )
    parser.add_argument(
        "output_obj",
        nargs="?",
        default="models/sr75_terrain_10km/meshes/sr75_terrain_10km.obj",
        help="Output OBJ path.",
    )
    parser.add_argument(
        "--step",
        type=int,
        default=6,
        help="Pixel step for decimation. Larger values produce fewer vertices.",
    )
    parser.add_argument(
        "--reference",
        choices=("center", "min", "mean"),
        default="center",
        help="Reference elevation to subtract so the mesh stays near z=0.",
    )
    return parser.parse_args()


def load_dem_with_gdal(path: str) -> DemRaster:
    try:
        from osgeo import gdal
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "GDAL Python bindings are required. Install `python3-gdal` or "
            "make `osgeo.gdal` available in this environment."
        ) from exc

    dataset = gdal.Open(path, gdal.GA_ReadOnly)
    if dataset is None:
        raise ValueError(f"Unable to open DEM with GDAL: {path}")

    band = dataset.GetRasterBand(1)
    if band is None:
        raise ValueError(f"DEM does not contain band 1: {path}")
    if band.DataType != gdal.GDT_Int16:
        raise ValueError(
            "Expected a single-band Int16 DEM GeoTIFF; "
            f"got GDAL data type {gdal.GetDataTypeName(band.DataType)}"
        )

    geotransform = dataset.GetGeoTransform(can_return_null=True)
    if geotransform is None:
        raise ValueError(f"DEM is missing geotransform information: {path}")

    pixel_size_x = float(geotransform[1])
    pixel_size_y = float(abs(geotransform[5]))
    if pixel_size_x <= 0 or pixel_size_y <= 0:
        raise ValueError(f"DEM has invalid pixel spacing: {geotransform}")

    nodata_value = band.GetNoDataValue()
    nodata = int(nodata_value) if nodata_value is not None else None

    width = band.XSize
    height = band.YSize
    raw = band.ReadRaster(
        xoff=0,
        yoff=0,
        xsize=width,
        ysize=height,
        buf_xsize=width,
        buf_ysize=height,
        buf_type=gdal.GDT_Int16,
    )
    if raw is None:
        raise ValueError(f"GDAL failed to read DEM data: {path}")
    expected_bytes = width * height * 2
    if len(raw) != expected_bytes:
        raise ValueError(
            f"Unexpected raster byte count: expected {expected_bytes}, got {len(raw)}"
        )

    elevations = list(struct.unpack(f"<{width * height}h", raw))

    return DemRaster(
        width=width,
        height=height,
        pixel_size_x=pixel_size_x,
        pixel_size_y=pixel_size_y,
        nodata=nodata,
        elevations=elevations,
    )


def sampled_indices(length: int, step: int) -> list[int]:
    indices = list(range(0, length, step))
    if indices[-1] != length - 1:
        indices.append(length - 1)
    return indices


def choose_reference(raster: DemRaster, mode: str) -> float:
    valid = raster.valid_elevations
    if mode == "min":
        return float(min(valid))
    if mode == "mean":
        return float(sum(valid) / len(valid))
    center_x = raster.width // 2
    center_y = raster.height // 2
    return float(resolve_elevation(raster, center_x, center_y))


def resolve_elevation(raster: DemRaster, x: int, y: int) -> int:
    value = raster.at(x, y)
    if raster.nodata is None or value != raster.nodata:
        return value

    max_radius = max(raster.width, raster.height)
    for radius in range(1, max_radius):
        min_x = max(0, x - radius)
        max_x = min(raster.width - 1, x + radius)
        min_y = max(0, y - radius)
        max_y = min(raster.height - 1, y + radius)

        for search_y in range(min_y, max_y + 1):
            for search_x in range(min_x, max_x + 1):
                if (
                    search_x not in (min_x, max_x)
                    and search_y not in (min_y, max_y)
                ):
                    continue
                candidate = raster.at(search_x, search_y)
                if candidate != raster.nodata:
                    return candidate

    raise ValueError("Unable to resolve a valid elevation near a NoData pixel")


def vertex_position(
    raster: DemRaster, x_index: int, y_index: int, reference_elevation: float
) -> tuple[float, float, float]:
    total_width = raster.width * raster.pixel_size_x
    total_height = raster.height * raster.pixel_size_y
    x = -total_width / 2.0 + (x_index / (raster.width - 1)) * total_width
    y = total_height / 2.0 - (y_index / (raster.height - 1)) * total_height
    z = float(resolve_elevation(raster, x_index, y_index)) - reference_elevation
    return x, y, z


def write_obj(
    output_path: str,
    source_path: str,
    raster: DemRaster,
    x_indices: Iterable[int],
    y_indices: Iterable[int],
    reference_elevation: float,
    step: int,
) -> tuple[int, int]:
    x_indices = list(x_indices)
    y_indices = list(y_indices)
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    vertices: list[tuple[float, float, float]] = []
    for y_index in y_indices:
        for x_index in x_indices:
            vertices.append(
                vertex_position(raster, x_index, y_index, reference_elevation)
            )

    face_count = 0
    columns = len(x_indices)
    rows = len(y_indices)

    with open(output_path, "w", encoding="utf-8") as stream:
        stream.write("# SR-75 10 km terrain mesh generated from DEM\n")
        stream.write(f"# source_dem={os.path.abspath(source_path)}\n")
        stream.write(
            f"# grid={columns}x{rows} step={step} reference_elevation={reference_elevation:.3f}\n"
        )
        stream.write("o sr75_terrain_10km\n")

        for x, y, z in vertices:
            stream.write(f"v {x:.3f} {y:.3f} {z:.3f}\n")

        for row in range(rows - 1):
            for column in range(columns - 1):
                top_left = row * columns + column + 1
                top_right = top_left + 1
                bottom_left = top_left + columns
                bottom_right = bottom_left + 1
                stream.write(f"f {top_left} {bottom_left} {top_right}\n")
                stream.write(f"f {top_right} {bottom_left} {bottom_right}\n")
                face_count += 2

    return len(vertices), face_count


if __name__ == "__main__":
    args = parse_args()
    if args.step < 1:
        raise SystemExit("--step must be >= 1")

    raster = load_dem_with_gdal(args.input_tif)
    reference = choose_reference(raster, args.reference)
    x_grid = sampled_indices(raster.width, args.step)
    y_grid = sampled_indices(raster.height, args.step)
    vertex_count, face_count = write_obj(
        args.output_obj, args.input_tif, raster, x_grid, y_grid, reference, args.step
    )

    valid = raster.valid_elevations
    print(f"input_dem={args.input_tif}")
    print(f"output_obj={args.output_obj}")
    print(
        f"dem_dimensions={raster.width}x{raster.height} pixel_size_m={raster.pixel_size_x}x{raster.pixel_size_y}"
    )
    print(
        f"elevation_range_m=min:{min(valid)} max:{max(valid)} span:{max(valid) - min(valid)}"
    )
    print(f"reference_elevation_m={reference:.3f} mode={args.reference}")
    print(f"mesh_grid={len(x_grid)}x{len(y_grid)} step={args.step}")
    print(f"vertex_count={vertex_count}")
    print(f"face_count={face_count}")
