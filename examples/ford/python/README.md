# Ford Campus importer, in Python

The same two-pass import as [`../cpp`](../cpp), orchestrated entirely from Python. It exists to
answer one question: can a real importer be written in Python without Python touching a point?

Yes. Python decides *what* to convert and hands whole NumPy arrays across; the pose transform and
quantization are one vectorised expression each; everything after that — Morton ordering, the
octree, LOD, compression, the key join — happens inside the library.

```
python ford_import.py inspect <dataset-root> [--scan N]
python ford_import.py convert <dataset-root> <output.dew> [--scans N] [--scale M] [--stride N]
python ford_import.py colour  <dataset-root> <dataset.dew> [--scans N]
```

## Requirements

```
pip install numpy scipy pillow
```

and the `dew` extension, which is not built by default:

```
cmake -S . -B build -DDEW_BUILD_PYTHON=ON
cmake --build build --target dew_python_package
PYTHONPATH=build/bindings/python python ford_import.py inspect <root>
```

`scipy.io.loadmat` reads the MATLAB v5 scans, so this version needs no MATLAB parser of its own —
that is 500 lines the C++ example has to carry (`mat_v5.cpp`) and this one does not.

## The two passes

**Pass 1** ingests geometry and carries a `scan_key` attribute — scan number in the high 12 bits,
point index in the low 20 — as an ordinary `u32`. Nothing in the library knows it is special.

**Pass 2** reopens the dataset, declares `rgb` joined on `scan_key`, and streams (key, value) pairs
in. The library sorts them against the dataset's own key buffers and writes one new blob per unit.
Nothing is reconverted and no other attribute is recompressed.

Splitting it this way is the point of the exercise: colour arrives from a completely different
source (per-camera images) on a different schedule, and the dataset does not have to be rebuilt to
accept it.

## Two things that are load-bearing

**`pre_init` must report `aabb_min`.** The converter dispatches inputs in rising min-morton order
and derives its done-morton watermark from it. Without it the watermark sits at zero for the whole
run and every collapse and LOD pass is deferred into a single terminal pass — on the full dataset
that meant 62 minutes on a fraction of one core instead of minutes across all of them. It must be a
true lower bound: too low only slows the watermark, too high silently *drops* points.

**`set_lod_all_attributes(1)`.** The default LOD keep-list is rgb/intensity/classification, so
without it `scan_key` would not survive LOD and only leaf nodes could ever be coloured.

**Do not call BLAS from the callbacks.** The converter runs `init` on several worker threads at
once. Writing the pose transform as the obvious `xyz @ r.T` sends an (N,3)×(3,3) product to
OpenBLAS, whose threading layer is not safe under that — it faulted inside its own critical section
about one run in eight, with a native stack and no Python frame. `world_points` multiplies the nine
terms out elementwise instead, which is both crash-free and marginally faster: a 3×3 rotation is
far below the size where BLAS earns its dispatch.

## What it costs

The converter calls `pre_init`/`init`/`convert_data` on its own worker threads, and each takes the
GIL to enter Python — so scan decoding does not run in parallel the way the C++ importer's does.
Expect this version to be read-bound where the C++ one is not. The *dataset* is unaffected: on 40
scans the two importers produce identical output — same 3,087,594 points, identical `scan_key`s,
byte-identical `rgb`, and a maximum coordinate difference of exactly zero.

That equivalence is worth keeping. It is what turned up the container bug described below.

## A note on the "damaged" scans

Five scans in the release (`Scan0111.mat` among them) end without the terminating block of their
deflate stream. Every byte the MATLAB payload declares is still present — 0111 inflates to 7,028,736
bytes for a top-level element declaring exactly 7,028,728 bytes of content — and `scipy.io.loadmat`
reads all five without complaint.

The C++ reader used to reject them, so the first full conversion silently dropped ~385,000 points.
The Python port produced a scan the C++ one had not, which is how it surfaced; `mat_v5.cpp` now
hands back what inflated and lets the element walk judge it.
