#!/usr/bin/env python3
"""
Satellite Map Downloader for Gazebo Simulation

Downloads satellite imagery tiles and stitches them into a single texture
for use in Gazebo Harmonic simulations.

Uses Esri World Imagery (free for non-commercial/educational use) or
OpenStreetMap tiles as fallback.

Usage:
    python3 download_satellite_map.py --lat 45.4303 --lon -122.8410 --size 200
    python3 download_satellite_map.py --config  # Use RTK base station from config
"""

import argparse
import math
import os
import sys
from pathlib import Path

try:
    import requests
    from PIL import Image
    HAS_DEPS = True
except ImportError:
    HAS_DEPS = False


def deg2num(lat_deg, lon_deg, zoom):
    """Convert lat/lon to tile numbers."""
    lat_rad = math.radians(lat_deg)
    n = 2.0 ** zoom
    xtile = int((lon_deg + 180.0) / 360.0 * n)
    ytile = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    return (xtile, ytile)


def num2deg(xtile, ytile, zoom):
    """Convert tile numbers to lat/lon (northwest corner)."""
    n = 2.0 ** zoom
    lon_deg = xtile / n * 360.0 - 180.0
    lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * ytile / n)))
    lat_deg = math.degrees(lat_rad)
    return (lat_deg, lon_deg)


def get_tile_bounds(xtile, ytile, zoom):
    """Get the lat/lon bounds of a tile."""
    nw_lat, nw_lon = num2deg(xtile, ytile, zoom)
    se_lat, se_lon = num2deg(xtile + 1, ytile + 1, zoom)
    return nw_lat, nw_lon, se_lat, se_lon


def meters_per_pixel(lat, zoom):
    """Calculate meters per pixel at given latitude and zoom level."""
    return 156543.03392 * math.cos(math.radians(lat)) / (2 ** zoom)


def download_tile(x, y, zoom, provider='esri'):
    """Download a single map tile."""
    if provider == 'esri':
        # Esri World Imagery - high quality satellite imagery
        url = f"https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{zoom}/{y}/{x}"
    elif provider == 'osm':
        # OpenStreetMap (not satellite, but useful fallback)
        url = f"https://tile.openstreetmap.org/{zoom}/{x}/{y}.png"
    else:
        raise ValueError(f"Unknown provider: {provider}")

    headers = {
        'User-Agent': 'Energia-Rover-Simulation/1.0 (Educational/Research Use)'
    }

    try:
        response = requests.get(url, headers=headers, timeout=30)
        response.raise_for_status()
        return response.content
    except requests.RequestException as e:
        print(f"  Warning: Failed to download tile ({x}, {y}): {e}")
        return None


def download_satellite_map(lat, lon, size_meters, output_dir, zoom=18):
    """
    Download satellite imagery centered on lat/lon covering size_meters x size_meters.

    Args:
        lat: Center latitude
        lon: Center longitude
        size_meters: Size of area to cover in meters (width and height)
        output_dir: Directory to save output files
        zoom: Tile zoom level (18 = ~0.6m/pixel, 17 = ~1.2m/pixel, 16 = ~2.4m/pixel)

    Returns:
        Path to the output image file
    """
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Calculate meters per pixel at this location
    mpp = meters_per_pixel(lat, zoom)
    print(f"Resolution: {mpp:.2f} meters/pixel at zoom {zoom}")

    # Calculate how many pixels we need
    pixels_needed = int(size_meters / mpp)
    print(f"Target size: {size_meters}m x {size_meters}m = ~{pixels_needed}x{pixels_needed} pixels")

    # Get center tile
    center_x, center_y = deg2num(lat, lon, zoom)

    # Calculate tile coverage needed (each tile is 256x256 pixels)
    tiles_needed = math.ceil(pixels_needed / 256) + 2  # Extra tiles for margin
    half_tiles = tiles_needed // 2

    print(f"Downloading {tiles_needed}x{tiles_needed} tiles...")

    # Download tiles
    tiles = {}
    total_tiles = tiles_needed * tiles_needed
    downloaded = 0

    for dx in range(-half_tiles, half_tiles + 1):
        for dy in range(-half_tiles, half_tiles + 1):
            tx, ty = center_x + dx, center_y + dy

            # Try Esri first, then fallback
            tile_data = download_tile(tx, ty, zoom, 'esri')

            if tile_data:
                tiles[(dx, dy)] = tile_data
                downloaded += 1

            # Progress indicator
            progress = (downloaded / total_tiles) * 100
            print(f"\r  Progress: {progress:.0f}% ({downloaded}/{total_tiles} tiles)", end='')

    print()  # Newline after progress

    if not tiles:
        print("Error: No tiles downloaded!")
        return None

    print(f"Downloaded {len(tiles)} tiles successfully")

    # Stitch tiles together
    print("Stitching tiles...")

    # Create output image
    tile_size = 256
    full_size = tiles_needed * tile_size
    stitched = Image.new('RGB', (full_size, full_size))

    for (dx, dy), tile_data in tiles.items():
        try:
            from io import BytesIO
            tile_img = Image.open(BytesIO(tile_data))

            # Calculate position in stitched image
            px = (dx + half_tiles) * tile_size
            py = (dy + half_tiles) * tile_size

            stitched.paste(tile_img, (px, py))
        except Exception as e:
            print(f"  Warning: Failed to process tile ({dx}, {dy}): {e}")

    # Crop to exact size centered
    crop_margin = (full_size - pixels_needed) // 2
    if crop_margin > 0:
        stitched = stitched.crop((
            crop_margin, crop_margin,
            crop_margin + pixels_needed, crop_margin + pixels_needed
        ))

    # Save output
    output_path = output_dir / "satellite_portland.jpg"
    stitched.save(output_path, "JPEG", quality=95)
    print(f"Saved: {output_path} ({stitched.size[0]}x{stitched.size[1]} pixels)")

    # Also save metadata
    metadata_path = output_dir / "satellite_metadata.txt"
    with open(metadata_path, 'w') as f:
        f.write(f"Center Latitude: {lat}\n")
        f.write(f"Center Longitude: {lon}\n")
        f.write(f"Size (meters): {size_meters}\n")
        f.write(f"Zoom Level: {zoom}\n")
        f.write(f"Resolution (m/pixel): {mpp:.4f}\n")
        f.write(f"Image Size (pixels): {stitched.size[0]}x{stitched.size[1]}\n")
        f.write(f"Actual Coverage (meters): {stitched.size[0] * mpp:.1f}x{stitched.size[1] * mpp:.1f}\n")
        f.write(f"Source: Esri World Imagery\n")
        f.write(f"Attribution: Esri, Maxar, Earthstar Geographics, and the GIS User Community\n")

    print(f"Saved metadata: {metadata_path}")

    return output_path


def create_gazebo_material(texture_path, output_dir):
    """Create Ogre material script for Gazebo."""
    material_dir = Path(output_dir) / "materials" / "scripts"
    material_dir.mkdir(parents=True, exist_ok=True)

    material_content = """material Energia/SatellitePortland
{
  technique
  {
    pass
    {
      ambient 1 1 1 1
      diffuse 1 1 1 1
      specular 0.1 0.1 0.1 1 12.5

      texture_unit
      {
        texture satellite_portland.jpg
        filtering trilinear
      }
    }
  }
}
"""

    material_path = material_dir / "satellite.material"
    with open(material_path, 'w') as f:
        f.write(material_content)

    print(f"Created material script: {material_path}")
    return material_path


def main():
    if not HAS_DEPS:
        print("Error: Required dependencies not installed.")
        print("Please install with: pip install requests pillow")
        sys.exit(1)

    parser = argparse.ArgumentParser(
        description='Download satellite imagery for Gazebo simulation'
    )
    parser.add_argument('--lat', type=float, default=45.4303333,
                        help='Center latitude (default: RTK base station)')
    parser.add_argument('--lon', type=float, default=-122.8410,
                        help='Center longitude (default: RTK base station)')
    parser.add_argument('--size', type=int, default=200,
                        help='Area size in meters (default: 200)')
    parser.add_argument('--zoom', type=int, default=18,
                        help='Tile zoom level 15-19 (default: 18, ~0.6m/pixel)')
    parser.add_argument('--output', type=str, default=None,
                        help='Output directory (default: energia_sim/textures)')

    args = parser.parse_args()

    # Default output directory
    if args.output is None:
        script_dir = Path(__file__).parent.parent
        output_dir = script_dir / "textures"
    else:
        output_dir = Path(args.output)

    print(f"=== Satellite Map Downloader ===")
    print(f"Location: {args.lat}, {args.lon}")
    print(f"Area: {args.size}m x {args.size}m")
    print(f"Output: {output_dir}")
    print()

    # Download satellite imagery
    texture_path = download_satellite_map(
        args.lat, args.lon, args.size, output_dir, args.zoom
    )

    if texture_path:
        # Create material script
        create_gazebo_material(texture_path, output_dir.parent)

        print()
        print("=== Next Steps ===")
        print("1. Copy texture to Gazebo models directory:")
        print(f"   cp {texture_path} ~/.gazebo/models/")
        print("2. Launch simulation with satellite world:")
        print("   ros2 launch energia_sim full_simulation.launch.py world:=satellite_portland")


if __name__ == '__main__':
    main()
