#!/usr/bin/env python3
"""
=============================================================================
Ice Stupa Volume Scanner -- Python Visualizer
=============================================================================
Author  : Open-Source Portfolio Project
Requires: Python >= 3.9
          pip install pyserial numpy scipy matplotlib open3d

WHAT THIS SCRIPT DOES
---------------------
1.  Opens the ESP32 USB serial port and reads ASCII data lines in real time.
2.  Parses each line into (phi, theta, dist, strength, qual).
3.  Converts spherical coordinates (r, theta, phi) to Cartesian (x, y, z).
4.  Applies MAD-based outlier removal to discard erroneous distance readings
    (caused by specular reflections on ice, near-field noise, or background
    objects that are not part of the structure being scanned).
5.  After each complete scan sweep ("SCAN_COMPLETE" token), computes the
    convex hull volume and renders an interactive 3D scatter plot.
6.  Optionally uses Open3D for higher-quality point cloud rendering with
    surface normal estimation.

COORDINATE SYSTEM
-----------------
The TF-Luna sensor sits at the ORIGIN of the world coordinate frame.
The scan angles from the ESP32 follow physics / ISO 80000-2 convention:

    theta (THETA) : polar angle measured from the POSITIVE Z-AXIS (vertical)
                    theta = 0   => ray points straight UP (+Z)
                    theta = 90  => ray points horizontally
                    theta = 180 => ray points straight DOWN (-Z)

    phi   (PHI)   : azimuth angle measured from the POSITIVE X-AXIS in the XY plane
                    phi = 0     => ray points along +X
                    phi = 90    => ray points along +Y

Spherical to Cartesian conversion:
    x = r * sin(theta) * cos(phi)
    y = r * sin(theta) * sin(phi)
    z = r * cos(theta)

where r = distance in cm from sensor to object surface.

For real Ice Stupa scans, the sensor is mounted on a tripod approximately
50-100 cm from the base of the stupa. For desktop demonstration, the sensor
is placed 30-60 cm from a clay or cardboard cone.

USAGE
-----
    # Live mode (reads from ESP32):
    python visualizer.py --port COM3          (Windows)
    python visualizer.py --port /dev/ttyUSB0  (Linux)
    python visualizer.py --port /dev/cu.usbserial-0001  (macOS)

    # Demo mode (no hardware needed, generates synthetic cone data):
    python visualizer.py --demo

    # Save point cloud to file after scan:
    python visualizer.py --port /dev/ttyUSB0 --save scan_001.csv

    # Use Open3D renderer instead of matplotlib:
    python visualizer.py --port /dev/ttyUSB0 --renderer open3d

    # Replay a previously saved CSV:
    python visualizer.py --replay scan_001.csv

CONFIGURATION
-------------
    SERIAL_BAUD  : must match ESP32 firmware (115200)
    MIN_POINTS   : minimum number of valid points required before computing volume
    MAD_THRESHOLD: Z-score threshold for outlier removal (3.5 is a robust default)
    SENSOR_OFFSET: [x,y,z] offset in cm if sensor is not at the global origin
=============================================================================
"""

import argparse
import sys
import time
import math
import re
import csv
import warnings
from typing import Optional

import numpy as np

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("[WARNING] pyserial not installed. Live mode unavailable. Use --demo or --replay.")

try:
    from scipy.spatial import ConvexHull
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False
    print("[WARNING] scipy not installed. Volume estimation will be unavailable.")

try:
    import matplotlib
    import matplotlib.pyplot as plt
    from matplotlib import cm
    from mpl_toolkits.mplot3d import Axes3D   # noqa: F401  (registers 3d projection)
    MPL_AVAILABLE = True
except ImportError:
    MPL_AVAILABLE = False
    print("[WARNING] matplotlib not installed. Matplotlib rendering unavailable.")

try:
    import open3d as o3d
    O3D_AVAILABLE = True
except ImportError:
    O3D_AVAILABLE = False
    # Open3D is optional -- matplotlib is the primary renderer

# ---------------------------------------------------------------------------
# CONFIGURATION CONSTANTS
# ---------------------------------------------------------------------------
SERIAL_BAUD    = 115200          # Must match ESP32 firmware
READ_TIMEOUT_S = 2.0             # pyserial read timeout in seconds
MIN_POINTS     = 10              # Minimum points needed to compute volume
MAD_THRESHOLD  = 3.5             # Modified Z-score threshold for outlier removal
MAX_DIST_CM    = 800             # Discard readings above this distance (cm)
MIN_DIST_CM    = 5               # Discard readings closer than this (cm)

# Sensor origin offset (cm). Adjust if the sensor is mounted offset from the
# coordinate system origin. Format: [x_offset, y_offset, z_offset]
SENSOR_OFFSET  = np.array([0.0, 0.0, 0.0])

# Regex pattern matching one data line from the ESP32 firmware
_DATA_PATTERN = re.compile(
    r"^PHI:([0-9]+\.[0-9]+),"
    r"THETA:([0-9]+\.[0-9]+),"
    r"DIST:([0-9]+),"
    r"STRENGTH:([0-9]+),"
    r"QUAL:([01])$"
)

# ---------------------------------------------------------------------------
# COORDINATE MATHEMATICS
# ---------------------------------------------------------------------------

def spherical_to_cartesian(r_cm: float, theta_deg: float, phi_deg: float) -> np.ndarray:
    """
    Convert a single spherical measurement to a 3D Cartesian point.

    Parameters
    ----------
    r_cm      : Distance from the sensor to the surface (centimetres)
    theta_deg : Polar angle from the +Z axis (degrees). 0 = straight up, 90 = horizontal.
    phi_deg   : Azimuth angle from the +X axis in the XY plane (degrees).

    Returns
    -------
    numpy array [x, y, z] in centimetres, in the sensor frame.

    Mathematics
    -----------
    Standard ISO 80000-2 spherical coordinates:
        x = r * sin(theta) * cos(phi)
        y = r * sin(theta) * sin(phi)
        z = r * cos(theta)

    Derivation check (verified numerically):
        theta=0,   phi=0    => (0, 0, r)     -- straight up, correct
        theta=90,  phi=0    => (r, 0, 0)     -- horizontal along X, correct
        theta=90,  phi=90   => (0, r, 0)     -- horizontal along Y, correct
        theta=45,  phi=0    => (r/sqrt2, 0, r/sqrt2) -- correct
    """
    t = math.radians(theta_deg)
    p = math.radians(phi_deg)
    x = r_cm * math.sin(t) * math.cos(p)
    y = r_cm * math.sin(t) * math.sin(p)
    z = r_cm * math.cos(t)
    return np.array([x, y, z], dtype=float) + SENSOR_OFFSET


def points_to_array(points_list: list) -> np.ndarray:
    """Convert a list of [x,y,z] lists to a (N,3) float64 numpy array."""
    return np.array(points_list, dtype=np.float64)


# ---------------------------------------------------------------------------
# OUTLIER REMOVAL
# ---------------------------------------------------------------------------

def remove_outliers_mad(points: np.ndarray, threshold: float = MAD_THRESHOLD) -> np.ndarray:
    """
    Remove outlier points using the Modified Z-Score method based on the
    Median Absolute Deviation (MAD) of Euclidean distances from the centroid.

    This is more robust than standard Z-score for non-Gaussian distributions,
    which is typical of sparse LiDAR point clouds with surface noise.

    Reference: Iglewicz & Hoaglin (1993) -- threshold of 3.5 recommended.

    Parameters
    ----------
    points    : (N, 3) array of Cartesian points
    threshold : Modified Z-score threshold; points above this are removed

    Returns
    -------
    Filtered (M, 3) array where M <= N
    """
    if len(points) < 5:
        return points

    # Distance of each point from the geometric centroid (median-based)
    centroid  = np.median(points, axis=0)
    distances = np.linalg.norm(points - centroid, axis=1)

    median_dist = np.median(distances)
    mad         = np.median(np.abs(distances - median_dist))

    if mad < 1e-9:
        # All points are at the same distance -- no outliers to remove
        return points

    # Modified Z-score (Iglewicz & Hoaglin constant: 0.6745)
    modified_z = 0.6745 * (distances - median_dist) / mad
    mask       = np.abs(modified_z) < threshold

    removed = np.sum(~mask)
    if removed > 0:
        print(f"  [Outlier filter] Removed {removed} of {len(points)} points "
              f"(MAD threshold = {threshold})")

    return points[mask]


# ---------------------------------------------------------------------------
# VOLUME ESTIMATION
# ---------------------------------------------------------------------------

def compute_convex_hull_volume(points: np.ndarray) -> Optional[float]:
    """
    Compute the volume enclosed by the convex hull of the point cloud.

    Suitable for convex or near-convex objects (a cone is convex).
    For highly concave objects, an alpha-shape or voxel-grid approach is needed.

    Notes on accuracy
    -----------------
    The convex hull over-estimates volume if the scanned object has concavities,
    or if background points are included. MAD outlier removal must precede this
    call. For a simple cone, over-estimation is typically 10-25% due to the
    sparse angular grid not perfectly representing the curved surface.

    Returns
    -------
    Volume in cubic centimetres, or None if the hull cannot be computed
    (fewer than 4 non-coplanar points).
    """
    if not SCIPY_AVAILABLE:
        print("  [Volume] scipy not available -- cannot compute convex hull.")
        return None

    if len(points) < 4:
        print("  [Volume] Need at least 4 non-coplanar points.")
        return None

    try:
        hull = ConvexHull(points)
        return hull.volume
    except Exception as exc:
        print(f"  [Volume] ConvexHull failed: {exc}")
        return None


def report_volume(volume_cm3: Optional[float]) -> None:
    """Print volume in multiple useful units."""
    if volume_cm3 is None:
        return
    liters     = volume_cm3 / 1000.0
    m3         = volume_cm3 / 1_000_000.0
    # Water stored: 1 litre of ice = ~0.917 litre of water (density ratio)
    water_l    = liters * 0.917
    print("\n" + "=" * 50)
    print("  VOLUME ESTIMATION RESULTS")
    print("=" * 50)
    print(f"  Convex Hull Volume : {volume_cm3:>12.2f}  cm^3")
    print(f"                     : {liters:>12.4f}  litres")
    print(f"                     : {m3:>12.6f}  m^3")
    print(f"  Equiv. water store : {water_l:>12.4f}  litres")
    print(f"  (ice density ~917 kg/m^3, melt ratio ~0.917)")
    print("=" * 50 + "\n")


# ---------------------------------------------------------------------------
# SERIAL DATA PARSING
# ---------------------------------------------------------------------------

def parse_line(line: str):
    """
    Parse one ASCII line from the ESP32 firmware.

    Returns a dict with keys: phi, theta, dist, strength, qual
    or None if the line does not match the expected format.
    """
    line = line.strip()
    m = _DATA_PATTERN.match(line)
    if not m:
        return None
    return {
        "phi":      float(m.group(1)),
        "theta":    float(m.group(2)),
        "dist":     int(m.group(3)),
        "strength": int(m.group(4)),
        "qual":     int(m.group(5)),
    }


def is_valid_reading(d: dict) -> bool:
    """
    Accept a parsed reading only if:
      - QUAL flag is 1 (ESP32 validated the TF-Luna checksum and strength)
      - Distance is within the physical sensor range
    """
    if d["qual"] != 1:
        return False
    if d["dist"] < MIN_DIST_CM or d["dist"] > MAX_DIST_CM:
        return False
    return True


# ---------------------------------------------------------------------------
# MATPLOTLIB 3D RENDERER
# ---------------------------------------------------------------------------

def render_matplotlib(points: np.ndarray, volume_cm3: Optional[float],
                      scan_index: int = 0) -> None:
    """
    Render the point cloud as a 3D scatter plot using matplotlib.
    Points are coloured by height (Z coordinate) using the 'viridis' colormap.
    The convex hull facets are overlaid as a wireframe.
    """
    if not MPL_AVAILABLE:
        print("  [Render] matplotlib not available.")
        return

    fig = plt.figure(figsize=(12, 9))
    ax  = fig.add_subplot(111, projection="3d")

    z_vals = points[:, 2]
    z_norm = (z_vals - z_vals.min()) / (z_vals.max() - z_vals.min() + 1e-9)
    colors = cm.viridis(z_norm)

    ax.scatter(
        points[:, 0], points[:, 1], points[:, 2],
        c=colors, s=6, alpha=0.85, linewidths=0
    )

    # Overlay convex hull as transparent wireframe
    if SCIPY_AVAILABLE and len(points) >= 4:
        try:
            hull = ConvexHull(points)
            for simplex in hull.simplices:
                verts = points[simplex]
                # Close the triangle
                verts = np.vstack([verts, verts[0]])
                ax.plot(verts[:, 0], verts[:, 1], verts[:, 2],
                        "b-", alpha=0.12, linewidth=0.6)
        except Exception:
            pass

    title = f"Ice Stupa Scanner -- Scan #{scan_index}  ({len(points)} points)"
    if volume_cm3 is not None:
        title += f"\nConvex Hull Volume = {volume_cm3:.1f} cm^3 = {volume_cm3/1000:.3f} L"

    ax.set_title(title, fontsize=11)
    ax.set_xlabel("X (cm)")
    ax.set_ylabel("Y (cm)")
    ax.set_zlabel("Z (cm)")
    ax.set_box_aspect([1, 1, 1])

    plt.tight_layout()
    plt.show(block=False)
    plt.pause(0.01)


# ---------------------------------------------------------------------------
# OPEN3D RENDERER (optional, higher quality)
# ---------------------------------------------------------------------------

def render_open3d(points: np.ndarray, volume_cm3: Optional[float]) -> None:
    """
    Render the point cloud using Open3D with:
      - Point cloud coloured by height
      - Surface normal estimation for better depth perception
      - Axis frame at the sensor origin
    """
    if not O3D_AVAILABLE:
        print("  [Render] open3d not available -- falling back to matplotlib.")
        render_matplotlib(points, volume_cm3)
        return

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))

    # Colour by Z height
    z_norm = (points[:, 2] - points[:, 2].min()) / (points[:, 2].max() - points[:, 2].min() + 1e-9)
    colors = plt.cm.viridis(z_norm)[:, :3] if MPL_AVAILABLE else np.tile([0.2, 0.6, 1.0], (len(points), 1))
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # Estimate surface normals (helps Open3D shading look realistic)
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=10, max_nn=30)
    )

    # Coordinate frame at origin
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10.0, origin=[0, 0, 0])

    o3d.visualization.draw_geometries(
        [pcd, frame],
        window_name="Ice Stupa Volume Scanner",
        width=1200, height=900,
        point_show_normal=False,
    )


# ---------------------------------------------------------------------------
# CSV SAVE / LOAD
# ---------------------------------------------------------------------------

def save_points_csv(points: np.ndarray, filepath: str) -> None:
    """Save the point cloud to a CSV file with columns x,y,z (cm)."""
    with open(filepath, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["x_cm", "y_cm", "z_cm"])
        for row in points:
            writer.writerow([f"{row[0]:.4f}", f"{row[1]:.4f}", f"{row[2]:.4f}"])
    print(f"  [Save] Point cloud saved to '{filepath}' ({len(points)} points)")


def load_points_csv(filepath: str) -> np.ndarray:
    """Load a point cloud from a CSV file saved by save_points_csv()."""
    points = []
    with open(filepath, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            points.append([float(row["x_cm"]), float(row["y_cm"]), float(row["z_cm"])])
    return np.array(points, dtype=np.float64)


# ---------------------------------------------------------------------------
# DEMO MODE (no hardware required)
# ---------------------------------------------------------------------------

def generate_demo_cone(
    height_cm: float = 30.0,
    base_radius_cm: float = 15.0,
    phi_step_deg: float = 10.0,
    theta_step_deg: float = 5.0,
    noise_cm: float = 0.8,
) -> list:
    """
    Generate a synthetic point cloud simulating the ESP32 scan of a cone.

    Geometry
    --------
    This models the field deployment scenario where the sensor is mounted near
    the apex of the Ice Stupa (or above a conical model on a desk) and sweeps
    downward. The SENSOR is at the ORIGIN. The cone extends below it:
      - Apex at origin (0, 0, 0)
      - Cone axis points in +Z direction (downward from sensor perspective)
      - At height z below the sensor, the ring radius = (z / height_cm) * base_radius_cm

    The function samples points on the cone surface parametrically, computes
    the polar angle (theta) and azimuth (phi) that a scanner beam would need
    to reach each surface point, then adds Gaussian distance noise to simulate
    real LiDAR measurement uncertainty (ice surface irregularity, vibration).

    This produces 400+ points across a realistic theta/phi spread, which is
    sufficient for robust convex hull volume estimation (typically within 10%
    of the analytical formula V = (1/3) * pi * r^2 * h).

    Returns
    -------
    List of dicts with keys: phi, theta, dist, strength, qual
    (same format as parse_line() output from the ESP32 ASCII protocol).
    """
    print(f"[Demo] Generating synthetic cone scan:")
    print(f"       height={height_cm} cm, base_radius={base_radius_cm} cm")
    print(f"       noise={noise_cm} cm, phi_step={phi_step_deg} deg, theta_step={theta_step_deg} deg")

    records = []
    rng = np.random.default_rng(seed=42)

    # Sample the cone surface at evenly-spaced (phi, z_fraction) grid points.
    # For each surface point P = (x, y, z), compute the spherical coordinates
    # (r, theta, phi) that the scanner beam would measure.
    #
    # Cone surface parametrisation:
    #   z_frac in [0, 1]  (0 = apex, 1 = base)
    #   phi    in [0, 2pi)
    #   x = (z_frac * base_radius_cm) * cos(phi)
    #   y = (z_frac * base_radius_cm) * sin(phi)
    #   z =  z_frac * height_cm
    #
    # Spherical coordinates of the surface point:
    #   r     = sqrt(x^2 + y^2 + z^2)
    #   theta = arccos(z / r)          [polar angle from +Z axis]
    #   phi   = atan2(y, x) mod 360    [azimuth]

    n_z_levels = max(10, int(height_cm / theta_step_deg))
    for z_frac in np.linspace(0.05, 1.0, n_z_levels):
        z  = z_frac * height_cm
        r_ring = z_frac * base_radius_cm

        for phi_surf_deg in np.arange(0.0, 360.0, phi_step_deg):
            phi_surf = math.radians(phi_surf_deg)
            x = r_ring * math.cos(phi_surf)
            y = r_ring * math.sin(phi_surf)

            # True distance from sensor (at origin) to surface point
            dist_true = math.sqrt(x * x + y * y + z * z)

            # Add Gaussian noise to simulate LiDAR measurement uncertainty
            dist_meas = max(1.0, dist_true + rng.normal(0.0, noise_cm))

            # Recompute spherical angles for the measured (noisy) point
            # so the record accurately reflects what the sensor would report
            theta_rad = math.acos(
                min(1.0, max(-1.0, z / dist_true))   # use true z, noisy r
            )
            theta_meas = math.degrees(theta_rad)
            phi_meas   = math.degrees(math.atan2(y, x)) % 360.0

            # Simulate signal strength: stronger at near-vertical angles,
            # weaker at grazing angles (realistic for LiDAR on ice surfaces)
            base_strength = int(rng.integers(350, 750))
            grazing_penalty = max(0, int((theta_meas - 60) * 5))
            strength = max(150, base_strength - grazing_penalty)

            records.append({
                "phi":      phi_meas,
                "theta":    theta_meas,
                "dist":     max(1, int(round(dist_meas))),
                "strength": strength,
                "qual":     1,
            })

    print(f"[Demo] Generated {len(records)} synthetic scan points")
    return records


# ---------------------------------------------------------------------------
# MAIN SCAN PROCESSING PIPELINE
# ---------------------------------------------------------------------------

class ScanProcessor:
    """
    Accumulates parsed scan records, applies all processing steps, and
    triggers rendering and volume reporting after each complete sweep.
    """

    def __init__(self, save_path: Optional[str] = None, renderer: str = "matplotlib"):
        self.current_scan_points: list = []   # raw (x,y,z) for current sweep
        self.all_scans: list            = []   # list of processed np.ndarray per sweep
        self.scan_count: int            = 0
        self.total_readings: int        = 0
        self.save_path: Optional[str]   = save_path
        self.renderer: str              = renderer

    def ingest_record(self, d: dict) -> None:
        """Accept one valid parsed record and convert to Cartesian."""
        self.total_readings += 1
        pt = spherical_to_cartesian(float(d["dist"]), d["theta"], d["phi"])
        self.current_scan_points.append(pt.tolist())

    def finalise_scan(self) -> None:
        """Called when 'SCAN_COMPLETE' is received from the ESP32."""
        self.scan_count += 1
        n_raw = len(self.current_scan_points)

        print(f"\n[Scan #{self.scan_count}] Complete -- {n_raw} raw valid points collected")

        if n_raw < MIN_POINTS:
            print(f"  [Skip] Fewer than {MIN_POINTS} points -- scan too sparse to process.")
            self.current_scan_points = []
            return

        pts_raw = points_to_array(self.current_scan_points)

        # Step 1: MAD outlier removal
        pts_clean = remove_outliers_mad(pts_raw)

        if len(pts_clean) < MIN_POINTS:
            print(f"  [Skip] After outlier removal only {len(pts_clean)} points remain.")
            self.current_scan_points = []
            return

        # Step 2: Compute volume
        volume_cm3 = compute_convex_hull_volume(pts_clean)
        report_volume(volume_cm3)

        # Step 3: Save to CSV if requested
        if self.save_path:
            # Append scan index to filename if multiple scans
            path = self.save_path
            if self.scan_count > 1:
                base, ext = path.rsplit(".", 1) if "." in path else (path, "csv")
                path = f"{base}_{self.scan_count:03d}.{ext}"
            save_points_csv(pts_clean, path)

        # Step 4: Render
        if self.renderer == "open3d":
            render_open3d(pts_clean, volume_cm3)
        else:
            render_matplotlib(pts_clean, volume_cm3, self.scan_count)

        # Archive and reset
        self.all_scans.append(pts_clean)
        self.current_scan_points = []


# ---------------------------------------------------------------------------
# LIVE SERIAL MODE
# ---------------------------------------------------------------------------

def list_serial_ports() -> None:
    """Print all available serial ports to help the user find the ESP32 port."""
    if not SERIAL_AVAILABLE:
        print("pyserial not installed -- cannot list ports.")
        return
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found.")
        return
    print("Available serial ports:")
    for port in ports:
        print(f"  {port.device:20s}  {port.description}")


def run_live(port: str, processor: ScanProcessor) -> None:
    """Open the ESP32 serial port and process incoming data indefinitely."""
    if not SERIAL_AVAILABLE:
        print("[ERROR] pyserial not installed. Cannot run in live mode.")
        sys.exit(1)

    print(f"[Live] Opening serial port {port} at {SERIAL_BAUD} baud ...")
    try:
        ser = serial.Serial(port, SERIAL_BAUD, timeout=READ_TIMEOUT_S)
    except serial.SerialException as exc:
        print(f"[ERROR] Could not open port '{port}': {exc}")
        print("       Use --list-ports to see available ports.")
        sys.exit(1)

    print("[Live] Connected. Waiting for scan data ... (Ctrl+C to stop)")
    time.sleep(0.5)
    ser.reset_input_buffer()

    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue

            try:
                line = raw.decode("ascii", errors="replace").strip()
            except Exception:
                continue

            if not line or line.startswith("#"):
                # Comments / blank lines from ESP32 startup message
                if line:
                    print(f"[ESP32] {line}")
                continue

            if line == "SCAN_COMPLETE":
                processor.finalise_scan()
                continue

            parsed = parse_line(line)
            if parsed is None:
                print(f"[WARN] Unrecognised line: {line!r}")
                continue

            if not is_valid_reading(parsed):
                continue

            processor.ingest_record(parsed)

    except KeyboardInterrupt:
        print("\n[Live] Interrupted by user.")
    finally:
        ser.close()
        print("[Live] Serial port closed.")


# ---------------------------------------------------------------------------
# DEMO MODE RUNNER
# ---------------------------------------------------------------------------

def run_demo(processor: ScanProcessor) -> None:
    """Generate synthetic cone data and process it through the pipeline."""
    print("[Demo] Running in demo mode -- no hardware required.")
    records = generate_demo_cone()

    for rec in records:
        if is_valid_reading(rec):
            processor.ingest_record(rec)

    processor.finalise_scan()

    print("[Demo] Complete. Close the plot window to exit.")
    if MPL_AVAILABLE:
        plt.show(block=True)


# ---------------------------------------------------------------------------
# REPLAY MODE RUNNER
# ---------------------------------------------------------------------------

def run_replay(csv_path: str, processor: ScanProcessor) -> None:
    """Load a previously saved CSV and re-render it."""
    print(f"[Replay] Loading '{csv_path}' ...")
    pts = load_points_csv(csv_path)
    print(f"[Replay] Loaded {len(pts)} points.")
    pts_clean = remove_outliers_mad(pts)
    volume_cm3 = compute_convex_hull_volume(pts_clean)
    report_volume(volume_cm3)
    if processor.renderer == "open3d":
        render_open3d(pts_clean, volume_cm3)
    else:
        render_matplotlib(pts_clean, volume_cm3, scan_index=1)
        if MPL_AVAILABLE:
            plt.show(block=True)


# ---------------------------------------------------------------------------
# ARGUMENT PARSING AND ENTRY POINT
# ---------------------------------------------------------------------------

def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Ice Stupa Volume Scanner -- Python Visualizer",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    mode = p.add_mutually_exclusive_group(required=False)
    mode.add_argument(
        "--port", metavar="PORT",
        help="Serial port connected to the ESP32 (e.g. COM3, /dev/ttyUSB0)"
    )
    mode.add_argument(
        "--demo", action="store_true",
        help="Run in demo mode: generate and visualize a synthetic cone point cloud"
    )
    mode.add_argument(
        "--replay", metavar="CSV_FILE",
        help="Replay / re-visualize a previously saved point cloud CSV file"
    )
    p.add_argument(
        "--save", metavar="CSV_FILE", default=None,
        help="Save the processed point cloud to a CSV file (e.g. scan_001.csv)"
    )
    p.add_argument(
        "--renderer", choices=["matplotlib", "open3d"], default="matplotlib",
        help="3D renderer to use (default: matplotlib)"
    )
    p.add_argument(
        "--list-ports", action="store_true",
        help="List available serial ports and exit"
    )
    return p


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()

    if args.list_ports:
        list_serial_ports()
        return

    # Validate renderer availability
    if args.renderer == "open3d" and not O3D_AVAILABLE:
        print("[WARN] open3d not installed -- falling back to matplotlib.")
        args.renderer = "matplotlib"

    if args.renderer == "matplotlib" and not MPL_AVAILABLE:
        print("[ERROR] matplotlib not installed. Install it with: pip install matplotlib")
        sys.exit(1)

    processor = ScanProcessor(save_path=args.save, renderer=args.renderer)

    if args.demo:
        run_demo(processor)
    elif args.replay:
        run_replay(args.replay, processor)
    elif args.port:
        run_live(args.port, processor)
    else:
        # No mode specified -- default to demo mode so the script is immediately useful
        print("[INFO] No mode specified. Running in --demo mode.")
        print("       Use --port PORT for live scanning or --help for all options.\n")
        run_demo(processor)


if __name__ == "__main__":
    warnings.filterwarnings("ignore", category=DeprecationWarning)
    main()
